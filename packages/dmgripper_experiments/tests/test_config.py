"""通用实验配置的构造、严格 YAML 与跨字段校验测试。"""

from __future__ import annotations

from pathlib import Path
import json

import pytest

from dmgripper_experiments.config import (
    AdaptiveReferenceConfig,
    CurveReferenceConfig,
    ExperimentConfig,
    HardwareConfig,
    PIDConfig,
    ReferenceConfig,
    WaypointConfig,
    load_experiment_config,
    sanitize_directory_component,
)


def _curve(waypoints, interpolation="smoothstep") -> ReferenceConfig:
    return ReferenceConfig(
        curve=CurveReferenceConfig(interpolation=interpolation, waypoints=waypoints)
    )


def test_default_config_constructs_and_selects_curve():
    """默认配置可离线构造，目标来源为恒定曲线。"""
    config = ExperimentConfig()
    assert config.reference.kind == "curve"
    assert config.reference.duration_s == pytest.approx(10.0)
    assert config.reference.initial_force_n == pytest.approx(0.5)
    assert config.estimation.enabled is True
    assert config.controller.stiffness_consumption == "none"
    assert config.hardware.home_position_rad == pytest.approx(0.0)
    assert config.hardware.home_tolerance_rad == pytest.approx(0.03)
    assert config.hardware.feedback_position_margin_rad == pytest.approx(0.05)
    assert config.controller.pid.torque_feedforward_gain is None
    assert config.controller.pid.max_position_adjustment_rad == pytest.approx(0.15)


@pytest.mark.parametrize("limit", [None, 0.15, 0.5])
def test_pid_position_adjustment_accepts_optional_positive_limit(limit):
    """位置偏置允许显式关闭固定上限，正值继续表达有限上限。"""
    assert PIDConfig(max_position_adjustment_rad=limit).max_position_adjustment_rad == limit


@pytest.mark.parametrize("limit", [0.0, -0.01, float("nan"), float("inf"), True, False])
def test_pid_position_adjustment_rejects_invalid_limit(limit):
    """关闭上限必须使用 None，零、负数、非有限值与布尔值均非法。"""
    with pytest.raises(ValueError, match="max_position_adjustment_rad"):
        PIDConfig(max_position_adjustment_rad=limit)


@pytest.mark.parametrize("literal", ["0.0", "-0.1", ".nan", ".inf", "true"])
def test_yaml_rejects_invalid_pid_position_adjustment_limit(tmp_path, literal):
    """严格 YAML 校验可空的位置偏置上限。"""
    path = tmp_path / "pid.yaml"
    path.write_text(
        f"controller:\n  pid:\n    max_position_adjustment_rad: {literal}\n", encoding="utf-8"
    )
    with pytest.raises(ValueError, match="max_position_adjustment_rad"):
        load_experiment_config(path)


def test_yaml_and_cli_accept_disabled_pid_position_adjustment_limit(tmp_path, capsys):
    """YAML null 与命令行 None 都可关闭固定偏置上限。"""
    from dmgripper_experiments.cli import run

    path = tmp_path / "pid.yaml"
    path.write_text(
        "controller:\n  kind: pid\n  pid:\n    max_position_adjustment_rad: null\n",
        encoding="utf-8",
    )
    assert load_experiment_config(path).controller.pid.max_position_adjustment_rad is None
    assert (
        run(["--controller.kind", "pid", "--controller.pid.max-position-adjustment-rad", "None"])
        == 0
    )
    record = json.loads(capsys.readouterr().out)
    assert record["mode"] == "dry-run"
    assert record["config"]["controller"]["pid"]["max_position_adjustment_rad"] is None


@pytest.mark.parametrize("gain", [None, 0.0, 0.4, 1.0])
def test_pid_torque_feedforward_accepts_optional_bounded_gain(gain):
    """独立 PID 力矩前馈接受兼容默认值与闭区间增益。"""
    assert PIDConfig(torque_feedforward_gain=gain).torque_feedforward_gain == gain


@pytest.mark.parametrize("gain", [-0.01, 1.01, float("nan"), float("inf"), True, False])
def test_pid_torque_feedforward_rejects_invalid_gain(gain):
    """负数、越界、非有限值与布尔值不得成为 PID 力矩前馈增益。"""
    with pytest.raises(ValueError, match="torque_feedforward_gain"):
        PIDConfig(torque_feedforward_gain=gain)


@pytest.mark.parametrize("literal, expected", [("null", None), ("0.0", 0.0), ("1.0", 1.0)])
def test_yaml_pid_torque_feedforward_does_not_require_estimation(tmp_path, literal, expected):
    """YAML 可在关闭估计时独立配置 PID 模型力矩前馈。"""
    path = tmp_path / "pid.yaml"
    path.write_text(
        "controller:\n  kind: pid\n  stiffness_consumption: none\n"
        f"  pid:\n    torque_feedforward_gain: {literal}\n"
        "estimation:\n  enabled: false\n",
        encoding="utf-8",
    )
    config = load_experiment_config(path)
    assert config.controller.pid.torque_feedforward_gain == expected
    assert not config.estimation.enabled


@pytest.mark.parametrize("literal", ["-0.1", "1.1", ".nan", "true"])
def test_yaml_rejects_invalid_pid_torque_feedforward_gain(tmp_path, literal):
    """严格 YAML 同样拒绝非法前馈增益。"""
    path = tmp_path / "pid.yaml"
    path.write_text(
        f"controller:\n  pid:\n    torque_feedforward_gain: {literal}\n", encoding="utf-8"
    )
    with pytest.raises(ValueError, match="torque_feedforward_gain"):
        load_experiment_config(path)


def test_water_bottle_pid_feedforward_can_be_overridden_in_cli_dry_run(capsys):
    """水瓶配置启用独立前馈，命令行可离线覆盖而不连接设备。"""
    from dmgripper_experiments.cli import run

    path = (
        Path(__file__).resolve().parents[3] / "configs/hardware/dmgripper/water_bottle_curve.yaml"
    )
    config = load_experiment_config(path)
    assert config.controller.pid.torque_feedforward_gain == 1.0
    assert config.controller.pid.max_position_adjustment_rad is None
    assert config.controller.stiffness_consumption == "none"
    assert (
        run(
            [
                "--config",
                str(path),
                "--controller.kind",
                "pid",
                "--controller.pid.torque-feedforward-gain",
                "0.0",
            ]
        )
        == 0
    )
    record = json.loads(capsys.readouterr().out)
    assert record["mode"] == "dry-run"
    assert record["config"]["controller"]["pid"]["torque_feedforward_gain"] == 0.0


@pytest.mark.parametrize("change", ["risk", "friction", "initial", "rate", "ceiling", "controller"])
def test_unified_config_loads_and_requires_consistent_limits_and_acceptance(change: str):
    """低载荷配置可离线解码，权限与执行边界不允许隐式越过。"""
    from dataclasses import replace

    config = load_experiment_config(
        Path(__file__).resolve().parents[3] / "configs/hardware/dmgripper/unified_adaptive.yaml"
    )
    assert config.unified_adaptive_enabled
    adaptive = config.reference.adaptive
    assert adaptive is not None and adaptive.unified is not None
    assert not adaptive.unified.risk_enabled
    assert not adaptive.unified.friction_update_enabled
    assert config.safety.max_target_force_n == 1.5
    assert config.safety.force_ceiling_n == 2.0
    with pytest.raises(ValueError):
        if change == "risk":
            replace(adaptive, unified=replace(adaptive.unified, risk_enabled=True))
        elif change == "friction":
            replace(
                adaptive,
                risk_validation_passed=True,
                unified=replace(adaptive.unified, risk_enabled=True, friction_update_enabled=True),
            )
        elif change == "initial":
            replace(adaptive, initial_force_n=0.6)
        elif change == "rate":
            replace(adaptive, max_force_rate_n_s=1.0)
        elif change == "ceiling":
            replace(config, safety=replace(config.safety, max_target_force_n=1.6))
        else:
            replace(config, controller=replace(config.controller, kind="pid"))


def test_hardware_home_and_feedback_margin_are_strictly_validated():
    """home 必须位于命令工作范围，反馈余量与容差不得为负。"""
    with pytest.raises(ValueError, match="home_position_rad"):
        HardwareConfig(home_position_rad=-0.01)
    with pytest.raises(ValueError, match="home_tolerance_rad"):
        HardwareConfig(home_tolerance_rad=-0.01)
    with pytest.raises(ValueError, match="feedback_position_margin_rad"):
        HardwareConfig(feedback_position_margin_rad=-0.01)
    with pytest.raises(ValueError, match="反馈位置安全范围.*超出电机协议位置量程"):
        HardwareConfig(feedback_position_margin_rad=2.0)


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("mit_kp", 500.01),
        ("mit_kd", 5.01),
        ("velocity_limit_rad_s", 8.01),
        ("torque_limit_nm", 4.01),
        ("return_mit_kp", 500.01),
        ("return_mit_kd", 5.01),
        ("return_torque_limit_nm", 4.01),
    ],
)
def test_controller_mit_fields_cannot_exceed_dm_protocol_ranges(name: str, value: float):
    """实验配置不得让协议饱和改变共享核的安全计算。"""
    from dmgripper_experiments.config import ControllerConfig

    with pytest.raises(ValueError, match=rf"controller\.{name}.*协议上限"):
        ControllerConfig(**{name: value})


def test_adaptive_reference_constructs_with_explicit_duration():
    """动态模式可离线构造，时长来自显式 duration_s。"""
    config = ExperimentConfig(
        reference=ReferenceConfig(adaptive=AdaptiveReferenceConfig(duration_s=3.0))
    )
    assert config.reference.kind == "adaptive"
    assert config.reference.duration_s == pytest.approx(3.0)
    assert config.reference.initial_force_n == pytest.approx(0.5)


def test_adaptive_hardware_profile_uses_shared_target_limit():
    """动态真机配置使用共享目标上限，并保留导纳单向闭合。"""
    repository_root = Path(__file__).resolve().parents[3]
    config = load_experiment_config(
        repository_root / "configs/hardware/dmgripper/adaptive_grip.yaml"
    )
    assert config.reference.kind == "adaptive"
    assert config.controller.kind == "admittance"
    assert config.controller.admittance.force_deadband_n == pytest.approx(0.1)
    assert config.controller.admittance.prevent_unloading is True
    assert config.lifecycle.preload_tolerance_n == pytest.approx(0.15)
    assert config.safety.max_target_force_n == pytest.approx(30.0)
    assert config.safety.force_ceiling_n == pytest.approx(40.0)


def test_reference_requires_exactly_one_source():
    """目标来源判别必须二选一。"""
    with pytest.raises(ValueError, match="之一"):
        ReferenceConfig()
    with pytest.raises(ValueError, match="之一"):
        ReferenceConfig(
            curve=CurveReferenceConfig(
                waypoints=(WaypointConfig(0.0, 0.5), WaypointConfig(1.0, 0.5))
            ),
            adaptive=AdaptiveReferenceConfig(),
        )


def test_curve_must_start_at_zero_with_strictly_increasing_times():
    """曲线首点必须零时刻且时间严格递增。"""
    with pytest.raises(ValueError, match="首点必须是零时刻"):
        _curve((WaypointConfig(0.5, 0.5), WaypointConfig(1.0, 0.5)))
    with pytest.raises(ValueError, match="严格递增"):
        _curve((WaypointConfig(0.0, 0.5), WaypointConfig(1.0, 0.5), WaypointConfig(1.0, 0.6)))


def test_descending_curve_conflicts_with_admittance_prevent_unloading():
    """导纳单向闭合与曲线下降段在计划阶段报错；关闭单向闭合后合法。"""
    from dataclasses import replace

    from dmgripper_experiments.config import AdmittanceConfig

    waypoints = (WaypointConfig(0.0, 0.6), WaypointConfig(1.0, 0.4))
    with pytest.raises(ValueError, match="prevent_unloading"):
        ExperimentConfig(reference=_curve(waypoints))
    ExperimentConfig(
        reference=_curve(waypoints),
        controller=replace(
            ExperimentConfig().controller,
            admittance=replace(AdmittanceConfig(), prevent_unloading=False),
        ),
    )


def test_adaptive_rejects_reapproach_lost_contact_action():
    """动态模式只接受失接触 fault。"""
    base = ExperimentConfig(reference=ReferenceConfig(adaptive=AdaptiveReferenceConfig()))
    from dataclasses import replace

    with pytest.raises(ValueError, match="fault"):
        replace(base, lifecycle=replace(base.lifecycle, lost_contact_action="reapproach"))


def test_stiffness_feedforward_requires_supported_controller_and_estimation():
    """刚度前馈消费只支持 PID／LADRC 且要求启用估计。"""
    from dataclasses import replace

    from dmgripper_experiments.config import EstimationConfig

    base = ExperimentConfig()
    pid = replace(base.controller, kind="pid", stiffness_consumption="feedforward")
    ExperimentConfig(controller=pid)
    admittance = replace(base.controller, stiffness_consumption="feedforward")
    with pytest.raises(ValueError, match="导纳"):
        ExperimentConfig(controller=admittance)
    with pytest.raises(ValueError, match="estimation.enabled"):
        ExperimentConfig(
            controller=pid,
            estimation=replace(EstimationConfig(), enabled=False),
        )


def test_curve_minimum_force_must_not_cross_release_threshold():
    """曲线最小力低于失接触阈值时在计划阶段报错。"""
    from dataclasses import replace

    from dmgripper_experiments.config import ControllerConfig

    config = replace(ExperimentConfig(), controller=replace(ControllerConfig(), kind="pid"))
    with pytest.raises(ValueError, match="失接触阈值"):
        replace(config, reference=_curve((WaypointConfig(0.0, 0.5), WaypointConfig(1.0, 0.05))))


def test_yaml_strict_loading_rejects_unknown_and_bad_types(tmp_path: Path):
    """严格 YAML 拒绝未知字段、布尔冒充数值与无效枚举。"""
    document = """
metadata:
  task_name: force-curve
  object_name: cube
timing:
  control_rate_hz: 100.0
  max_control_gap_s: 0.05
  tactile_timeout_s: 0.1
reference:
  curve:
    interpolation: linear
    waypoints:
      - t_s: 0.0
        force_n: 0.5
      - t_s: 5.0
        force_n: 0.8
"""
    path = tmp_path / "config.yaml"
    path.write_text(document, encoding="utf-8")
    config = load_experiment_config(path)
    assert config.metadata.task_name == "force-curve"
    assert config.reference.curve is not None
    assert config.reference.curve.interpolation == "linear"

    unknown = tmp_path / "unknown.yaml"
    unknown.write_text(document.replace("timing:", "timing:\n  mystery: 1.0\n"), encoding="utf-8")
    with pytest.raises(ValueError, match="未知字段"):
        load_experiment_config(unknown)

    boolean = tmp_path / "boolean.yaml"
    boolean.write_text(
        document.replace("control_rate_hz: 100.0", "control_rate_hz: true"), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="有限数值"):
        load_experiment_config(boolean)

    enum = tmp_path / "enum.yaml"
    enum.write_text(
        document.replace("interpolation: linear", "interpolation: cubic"), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="linear"):
        load_experiment_config(enum)


def test_yaml_cli_friendly_flat_fields_load(tmp_path: Path):
    """YAML 可用嵌套段完整描述动态增力任务。"""
    document = """
reference:
  adaptive:
    initial_force_n: 0.6
    duration_s: 8.0
safety:
  max_target_force_n: 1.2
  force_ceiling_n: 2.0
controller:
  kind: pid
"""
    path = tmp_path / "adaptive.yaml"
    path.write_text(document, encoding="utf-8")
    config = load_experiment_config(path)
    assert config.reference.kind == "adaptive"
    assert config.controller.kind == "pid"
    assert config.safety.max_target_force_n == pytest.approx(1.2)


def test_sanitize_directory_component_blocks_traversal():
    """目录组件清洗阻止路径分隔符、.. 与空名。"""
    assert sanitize_directory_component("倒水-实验 2") == "倒水-实验_2"
    with pytest.raises(ValueError):
        sanitize_directory_component("a/b")
    with pytest.raises(ValueError):
        sanitize_directory_component("..")
    with pytest.raises(ValueError):
        sanitize_directory_component("  ")
    with pytest.raises(ValueError):
        sanitize_directory_component("a\\b")

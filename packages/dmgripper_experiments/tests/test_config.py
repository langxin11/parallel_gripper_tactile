"""显式阶段与人工冻结先验的配置约束。"""

from dataclasses import replace
from pathlib import Path

import pytest

from dmgripper_experiments.config import (
    AdmittanceConfig,
    AdaptiveReferenceConfig,
    ControllerConfig,
    EstimationConfig,
    HardwareConfig,
    LifecycleConfig,
    RecordingConfig,
    SafetyConfig,
    TimingConfig,
    sanitize_directory_component,
    ExperimentConfig,
    FrozenFrictionConfig,
    load_experiment_config,
)


@pytest.mark.parametrize("number", (1, 2, 3))
def test_object_templates_are_unfinished_preload_candidates(number: int) -> None:
    """三个物体模板均不冒充已标定结果。"""
    root = Path(__file__).parents[3]
    config = load_experiment_config(root / f"configs/hardware/dmgripper/object_{number}.yaml")
    assert config.stage == "preload"
    assert config.reference.friction is None
    assert config.reference.initial_force_n == 0.5
    assert config.metadata.condition


def test_friction_requires_preload_source_but_no_mu() -> None:
    """摩擦试验沿用被人工接受的固定预载。"""
    with pytest.raises(ValueError, match="preload_source"):
        ExperimentConfig(stage="friction")
    config = ExperimentConfig(
        stage="friction", reference=AdaptiveReferenceConfig(preload_source="preload/run-1")
    )
    assert config.reference.friction is None


def test_adaptive_requires_both_positive_mu_and_source() -> None:
    """单侧缺值与缺少来源不能进入自适应。"""
    base = AdaptiveReferenceConfig(preload_source="preload/run-1")
    with pytest.raises(ValueError, match="reference.friction"):
        ExperimentConfig(stage="adaptive", reference=base)
    with pytest.raises(ValueError, match="双侧正数"):
        FrozenFrictionConfig(left_mu=0.3, right_mu=0.0, source="friction/run-1")
    with pytest.raises(ValueError, match="source"):
        FrozenFrictionConfig(left_mu=0.3, right_mu=0.4)
    accepted = FrozenFrictionConfig(left_mu=0.3, right_mu=0.4, source="friction/run-1")
    config = ExperimentConfig(stage="adaptive", reference=replace(base, friction=accepted))
    assert config.adaptive_load_config.max_force_n == config.safety.max_target_force_n


def test_retired_yaml_fields_are_rejected(tmp_path: Path) -> None:
    """旧在线估计入口不能静默成为实机配置。"""
    path = tmp_path / "old.yaml"
    path.write_text("stage: adaptive\nreference:\n  unified: {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="未知字段"):
        load_experiment_config(path)


def test_timing_defaults_and_ordering_constraints():
    """时序默认值为 250 Hz；控制间隙必须落在单周期与触觉超时之间。"""
    timing = TimingConfig()
    assert timing.control_rate_hz == pytest.approx(250.0)
    assert timing.max_control_gap_s == pytest.approx(0.1)
    assert timing.tactile_timeout_s == pytest.approx(0.2)
    assert timing.tactile_cutoff_hz == pytest.approx(20.0)
    with pytest.raises(ValueError, match="max_control_gap_s 必须大于一个控制周期"):
        TimingConfig(max_control_gap_s=1.0 / 250.0)
    with pytest.raises(ValueError, match="max_control_gap_s 不得大于"):
        TimingConfig(max_control_gap_s=0.3)
    with pytest.raises(ValueError, match="tactile_timeout_s"):
        TimingConfig(tactile_timeout_s=0.0)
    with pytest.raises(ValueError, match="tactile_cutoff_hz"):
        TimingConfig(tactile_cutoff_hz=-1.0)
    with pytest.raises(ValueError, match="有限数值"):
        TimingConfig(control_rate_hz=True)


def test_hardware_home_and_feedback_margin_are_strictly_validated():
    """home 必须位于命令工作范围，反馈余量与容差不得为负。"""
    with pytest.raises(ValueError, match="home_position_rad"):
        HardwareConfig(home_position_rad=-0.01)
    with pytest.raises(ValueError, match="home_position_rad"):
        HardwareConfig(home_position_rad=3.15)
    with pytest.raises(ValueError, match="home_tolerance_rad"):
        HardwareConfig(home_tolerance_rad=-0.01)
    with pytest.raises(ValueError, match="feedback_position_margin_rad"):
        HardwareConfig(feedback_position_margin_rad=-0.01)
    with pytest.raises(ValueError, match="反馈位置安全范围.*超出电机协议位置量程"):
        HardwareConfig(feedback_position_margin_rad=2.0)
    with pytest.raises(ValueError, match="dm_port"):
        HardwareConfig(dm_port="  ")
    with pytest.raises(ValueError, match="tactile_port"):
        HardwareConfig(tactile_port="")


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
    with pytest.raises(ValueError, match=rf"controller\.{name}.*协议上限"):
        ControllerConfig(**{name: value})


@pytest.mark.parametrize(
    "name",
    [
        "mit_kp",
        "mit_kd",
        "velocity_limit_rad_s",
        "torque_limit_nm",
        "return_mit_kp",
        "return_mit_kd",
        "return_torque_limit_nm",
    ],
)
@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf"), True])
def test_controller_numeric_fields_must_be_positive_finite(name: str, value: float):
    """控制器数值字段必须为正有限量；零、负与非有限值一律非法。"""
    with pytest.raises(ValueError, match=f"controller\\.{name}"):
        ControllerConfig(**{name: value})


@pytest.mark.parametrize(
    "name,value",
    [
        ("max_closing_velocity_m_s", 0.0),
        ("max_opening_velocity_m_s", -0.1),
        ("max_acceleration_m_s2", float("inf")),
    ],
)
def test_admittance_motion_limits_must_be_positive_finite(name: str, value: float) -> None:
    """独立外环运动边界不能以零、负值或非有限数绕过。"""
    with pytest.raises(ValueError):
        AdmittanceConfig(**{name: value})


def test_safety_ordering_and_positive_bounds():
    """目标上限必须严格低于硬保护线，全部字段必须为正有限量。"""
    with pytest.raises(ValueError, match="max_target_force_n 必须严格小于"):
        SafetyConfig(max_target_force_n=40.0, force_ceiling_n=40.0)
    with pytest.raises(ValueError, match="max_target_force_n"):
        SafetyConfig(max_target_force_n=0.0)
    with pytest.raises(ValueError, match="force_ceiling_n"):
        SafetyConfig(force_ceiling_n=0.0)
    with pytest.raises(ValueError, match="max_force_rate_n_s"):
        SafetyConfig(max_force_rate_n_s=0.0)
    with pytest.raises(ValueError, match="max_force_decrease_rate_n_s"):
        SafetyConfig(max_force_decrease_rate_n_s=-1.0)
    with pytest.raises(ValueError, match="approach_feedforward_force_n"):
        SafetyConfig(approach_feedforward_force_n=0.0)
    with pytest.raises(ValueError, match="max_contact_compression_m"):
        SafetyConfig(max_contact_compression_m=0.0)


def test_reference_bounds_against_contact_and_target():
    """初始目标力必须落在接触阈值与目标上限之间。"""
    with pytest.raises(ValueError, match="不得大于 safety.max_target_force_n"):
        ExperimentConfig(reference=AdaptiveReferenceConfig(initial_force_n=31.0))
    with pytest.raises(ValueError, match="不得小于 lifecycle.contact_on_n"):
        ExperimentConfig(reference=AdaptiveReferenceConfig(initial_force_n=0.1))


def test_reference_duration_accepts_null_and_rejects_invalid_values():
    """显式 null 表示不限时，非法时长一律拒绝。"""
    assert AdaptiveReferenceConfig(duration_s=None).duration_s is None
    for duration in (0.0, -1.0, float("inf"), float("nan"), True):
        with pytest.raises(ValueError):
            AdaptiveReferenceConfig(duration_s=duration)
    with pytest.raises(ValueError, match="initial_force_n"):
        AdaptiveReferenceConfig(initial_force_n=0.0)


def test_lifecycle_contact_thresholds_and_ratios_are_validated():
    """接触阈值滞回与预载比例的非法值一律拒绝。"""
    with pytest.raises(ValueError, match="contact_off_n"):
        LifecycleConfig(contact_on_n=0.1, contact_off_n=0.2)
    with pytest.raises(ValueError, match="contact_on_stable_s"):
        LifecycleConfig(contact_on_stable_s=0.0)
    with pytest.raises(ValueError, match="contact_off_stable_s"):
        LifecycleConfig(contact_off_stable_s=0.0)
    with pytest.raises(ValueError, match="contact_transition_s"):
        LifecycleConfig(contact_transition_s=0.0)
    with pytest.raises(ValueError, match="preload_stable_time_s"):
        LifecycleConfig(preload_stable_time_s=0.0)
    with pytest.raises(ValueError, match="preload_timeout_s"):
        LifecycleConfig(preload_timeout_s=0.0)


@pytest.mark.parametrize("rate", [0.0, -1.0, float("nan"), float("inf"), True])
def test_preload_ramp_rejects_invalid_rate(rate):
    """预载升力速率必须是正有限数值，None 表示常值目标。"""
    with pytest.raises(ValueError):
        LifecycleConfig(preload_force_rate_n_s=rate)
    assert LifecycleConfig(preload_force_rate_n_s=None).preload_force_rate_n_s is None


@pytest.mark.parametrize("ratio", [0.0, -1.0, 1.01, float("nan"), float("inf"), True])
def test_preload_min_force_ratio_rejects_invalid_values(ratio):
    """预载最低比例必须位于零与一之间。"""
    with pytest.raises(ValueError):
        LifecycleConfig(preload_min_force_ratio=ratio)


@pytest.mark.parametrize("behavior", ["release", "loop"])
def test_on_finished_enum_is_strict(behavior: str):
    """任务结束行为只允许 hold 或 return。"""
    with pytest.raises(ValueError, match="hold 或 return"):
        LifecycleConfig(on_finished=behavior)  # type: ignore[arg-type]
    assert LifecycleConfig(on_finished="return").on_finished == "return"


def test_estimation_only_carries_enabled_switch():
    """估计段只保留 enabled 开关，方法字段已固化为内部常数。"""
    assert EstimationConfig().enabled is False
    with pytest.raises(ValueError, match="enabled 必须是布尔值"):
        EstimationConfig(enabled=1)  # type: ignore[arg-type]


def test_recording_config_validates_stride_and_caps():
    """抽样默认 4；上限参数非法值被拒绝，None 表示显式关闭上限。"""
    assert RecordingConfig().tactile_stride == 4
    assert RecordingConfig().max_duration_s == 600.0
    assert RecordingConfig().max_recording_mib == 512.0
    assert RecordingConfig(max_duration_s=None).max_duration_s is None
    assert RecordingConfig(max_recording_mib=None).max_recording_mib is None
    with pytest.raises(ValueError, match="tactile_stride"):
        RecordingConfig(tactile_stride=0)
    with pytest.raises(ValueError, match="tactile_stride"):
        RecordingConfig(tactile_stride=True)
    with pytest.raises(ValueError, match="max_duration_s"):
        RecordingConfig(max_duration_s=0.0)
    with pytest.raises(ValueError, match="max_recording_mib"):
        RecordingConfig(max_recording_mib=-1.0)


def test_yaml_records_recording_section(tmp_path: Path):
    """recording 节可从严格 YAML 加载，未给出的字段使用默认值。"""
    path = tmp_path / "recording.yaml"
    path.write_text(
        "recording:\n  tactile_stride: 8\n  max_duration_s: 300.0\n  max_recording_mib: null\n",
        encoding="utf-8",
    )
    config = load_experiment_config(path)
    assert config.recording == RecordingConfig(
        tactile_stride=8, max_duration_s=300.0, max_recording_mib=None
    )


@pytest.mark.parametrize(
    ("section", "snippet", "field"),
    [
        ("metadata", "  description: 旧说明\n", "description"),
        ("metadata", "  tags:\n  - real\n", "tags"),
        ("lifecycle", "  auto_start: true\n", "auto_start"),
        ("lifecycle", "  lost_contact_action: fault\n", "lost_contact_action"),
        ("reference", "  curve:\n    interpolation: linear\n", "curve"),
        ("reference", "  max_force_rate_n_s: 10.0\n", "max_force_rate_n_s"),
        ("controller", "  kind: pid\n", "kind"),
        ("controller", "  pid:\n    mit_kp: 10.0\n", "pid"),
        ("controller", "  adrc: {}\n", "adrc"),
        ("safety", "  approach_feedforward_ratio: 1.0\n", "approach_feedforward_ratio"),
        ("estimation", "  method: window_linear\n", "method"),
    ],
)
def test_yaml_rejects_removed_fields_as_unknown(
    tmp_path: Path, section: str, snippet: str, field: str
):
    """已删除字段一律按未知字段拒绝，不做静默默认。"""
    path = tmp_path / "legacy.yaml"
    path.write_text(f"{section}:\n{snippet}", encoding="utf-8")
    with pytest.raises(ValueError, match=f"未知字段.*{field}"):
        load_experiment_config(path)


@pytest.mark.parametrize("section", ["terminal", "output"])
def test_yaml_rejects_removed_root_sections(tmp_path: Path, section: str):
    """根节点出现 terminal／output 段必须报未知字段。"""
    path = tmp_path / f"{section}.yaml"
    path.write_text(f"{section}:\n  mode: auto\n", encoding="utf-8")
    with pytest.raises(ValueError, match="配置根节点包含未知字段"):
        load_experiment_config(path)


def test_yaml_root_rejects_multiple_unknown_sections(tmp_path: Path):
    """根节点同时出现两个未知段时按排序列出。"""
    path = tmp_path / "roots.yaml"
    path.write_text("terminal:\n  mode: auto\noutput:\n  root: x\n", encoding="utf-8")
    with pytest.raises(ValueError, match="配置根节点包含未知字段：output, terminal"):
        load_experiment_config(path)


def test_yaml_metadata_rejects_empty_names(tmp_path: Path):
    """metadata 只保留任务与物体名，且必须是非空字符串。"""
    path = tmp_path / "metadata.yaml"
    path.write_text('metadata:\n  task_name: "  "\n', encoding="utf-8")
    with pytest.raises(ValueError, match="task_name"):
        load_experiment_config(path)


def test_yaml_hardware_ports_must_be_non_empty_strings(tmp_path: Path):
    """设备端口在 YAML 层同样拒绝空串与非字符串。"""
    path = tmp_path / "ports.yaml"
    path.write_text('hardware:\n  dm_port: ""\n', encoding="utf-8")
    with pytest.raises(ValueError, match="dm_port"):
        load_experiment_config(path)


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


def test_yaml_strict_loading_rejects_unknown_and_bad_types(tmp_path: Path) -> None:
    """严格 YAML 拒绝未知字段、布尔冒充数值与无效阶段。"""
    document = """
stage: preload
metadata:
  task_name: grasp-demo
  object_name: cube
timing:
  control_rate_hz: 100.0
  max_control_gap_s: 0.15
  tactile_timeout_s: 0.2
reference:
  initial_force_n: 0.6
  duration_s: 5.0
"""
    path = tmp_path / "config.yaml"
    path.write_text(document, encoding="utf-8")
    config = load_experiment_config(path)
    assert config.metadata.task_name == "grasp-demo"
    assert config.reference.duration_s == pytest.approx(5.0)
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
    enum.write_text(document.replace("stage: preload", "stage: unknown"), encoding="utf-8")
    with pytest.raises(ValueError, match="stage 必须是"):
        load_experiment_config(enum)

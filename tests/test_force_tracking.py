"""验证 waypoint 目标力跟踪任务。"""

import csv
import math
from pathlib import Path

import pytest

from parallel_gripper_tactile.experiments.force_tracking import (
    ForceReference,
    ForceTrackingTask,
    ForceWaypoint,
    configure_force_controller,
    run_force_tracking,
)
from parallel_gripper_tactile.profiles import AdrcControl, TorqueAdrcControl, load_profile


ROOT = Path(__file__).resolve().parents[1]


def test_force_reference_waypoints_interpolate_targets() -> None:
    """waypoint 目标力曲线支持保持、线性和平滑插值。"""
    waypoints = (ForceWaypoint(t_s=0.0, force_n=2.0), ForceWaypoint(t_s=1.0, force_n=6.0))

    assert ForceReference(interpolation="hold", waypoints=waypoints).target_at(0.5) == 2.0
    assert ForceReference(interpolation="linear", waypoints=waypoints).target_at(0.5) == 4.0
    assert ForceReference(interpolation="smoothstep", waypoints=waypoints).target_at(0.5) == 4.0
    assert ForceReference(interpolation="linear", waypoints=waypoints).target_at(2.0) == 6.0


def test_force_tracking_task_loads_default_waypoint_config() -> None:
    """默认动态力跟踪任务来自 YAML 配置文件。"""
    task = ForceTrackingTask.load(ROOT / "configs/force_tracking/default_waypoints.yaml")

    assert task.name == "default_waypoint_force_tracking"
    assert task.approach.feedforward_force_n == pytest.approx(1.0)
    assert task.reference.duration_s == pytest.approx(4.5)
    assert task.reference.target_at(3.5) == pytest.approx(10.0)


@pytest.mark.parametrize(
    ("variant", "enabled", "position_gain", "torque_gain", "feedback_gain"),
    [
        ("pid-only", False, 0.25, 1.0, 0.0),
        ("pid-torque-ff", True, 0.0, 1.0, 0.0),
        ("pid-stiffness-ff", True, 0.25, 0.0, 0.0),
        ("full", True, 0.25, 1.0, 0.0),
        ("direct-torque", True, 0.0, 1.0, 1.0),
        ("adrc", True, 0.0, 1.0, 0.0),
        ("adrc-torque", True, 0.0, 1.0, 0.0),
        ("adrc-torque-td", True, 0.0, 1.0, 0.0),
    ],
)
def test_controller_variants_apply_reproducible_ablation_settings(
    variant: str,
    enabled: bool,
    position_gain: float,
    torque_gain: float,
    feedback_gain: float,
) -> None:
    """各控制器档位只修改对应的刚度估计与前馈开关。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(  # type: ignore[arg-type]
        source, variant=variant, sensor_noise_seed=17
    )

    assert configured.normal_force is not None
    assert configured.normal_force.sensor_noise_seed == 17
    assert configured.normal_force.torque_feedback_gain == feedback_gain
    assert configured.normal_force.stiffness is not None
    assert configured.normal_force.stiffness.enabled is enabled
    assert configured.normal_force.stiffness.position_feedforward_gain == position_gain
    assert configured.normal_force.stiffness.torque_feedforward_gain == torque_gain
    assert source.normal_force is not None
    assert source.normal_force.sensor_noise_seed == 20260814
    assert source.normal_force.torque_feedback_gain == 0.0


def test_direct_torque_variant_keeps_mit_gains_for_approach_servo() -> None:
    """direct-torque 不在 profile 层清零 MIT kp/kd，接近阶段保持位置伺服。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(source, variant="direct-torque")

    assert source.mit is not None
    assert configured.mit is not None
    assert configured.mit.kp == source.mit.kp
    assert configured.mit.kd == source.mit.kd
    assert configured.normal_force is not None
    assert configured.normal_force.torque_feedback_gain == 1.0


def test_adrc_variant_enables_ladrc_outer_loop_with_default_parameters() -> None:
    """adrc 变体注入默认 LADRC 参数，位置前馈置 0，力矩前馈与 MIT 增益不动。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(source, variant="adrc")

    assert configured.normal_force is not None
    # LADRC 外环以代码与 AdrcControl 默认值生效，profile 无需显式配置。
    assert configured.normal_force.adrc is not None
    assert configured.normal_force.adrc == AdrcControl()
    assert configured.normal_force.torque_feedback_gain == 0.0
    assert configured.normal_force.stiffness is not None
    # 刚度估计照常运行（trace 可比），位置前馈由 LADRC 取代，力矩前馈对标 full。
    assert configured.normal_force.stiffness.enabled is True
    assert configured.normal_force.stiffness.position_feedforward_gain == 0.0
    assert source.normal_force is not None
    assert source.normal_force.stiffness is not None
    assert (
        configured.normal_force.stiffness.torque_feedforward_gain
        == source.normal_force.stiffness.torque_feedforward_gain
    )
    # MIT kp/kd 不在 profile 层改动，接近阶段与各变体共享位置伺服。
    assert source.mit is not None
    assert configured.mit is not None
    assert configured.mit.kp == source.mit.kp
    assert configured.mit.kd == source.mit.kd
    # 源 profile 不携带 adrc 配置，默认行为保持不变。
    assert source.normal_force.adrc is None


def test_adrc_torque_variant_enables_model_scheduled_direct_torque_loop() -> None:
    """adrc-torque 注入二阶 LADRC 参数，并保留显式名义模型前馈。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(source, variant="adrc-torque")

    assert configured.normal_force is not None
    assert configured.normal_force.torque_adrc == TorqueAdrcControl()
    assert configured.normal_force.torque_adrc.measurement_filter_cutoff_hz == 40.0
    assert configured.normal_force.adrc is None
    assert configured.normal_force.torque_feedback_gain == 0.0
    assert configured.normal_force.stiffness is not None
    assert configured.normal_force.stiffness.enabled is True
    assert configured.normal_force.stiffness.position_feedforward_gain == 0.0
    assert configured.normal_force.stiffness.torque_feedforward_gain == 1.0
    # 接近阶段仍复用相同 MIT 阻抗参数，旁路只发生在跟踪控制周期。
    assert configured.mit == source.mit


def test_adrc_torque_variant_accepts_explicit_tuning_override() -> None:
    """调参 study 可注入二阶直接力矩 ADRC 参数，而不修改源 profile。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    override = TorqueAdrcControl(
        measurement_filter_cutoff_hz=60.0,
        controller_bandwidth_rad_s=40.0,
        observer_bandwidth_rad_s=120.0,
    )

    configured = configure_force_controller(
        source,
        variant="adrc-torque",
        torque_adrc_override=override,
    )

    assert configured.normal_force is not None
    assert configured.normal_force.torque_adrc == override
    with pytest.raises(ValueError, match="requires a torque ADRC"):
        configure_force_controller(source, variant="full", torque_adrc_override=override)


def test_adrc_torque_td_variant_only_adds_reference_shaping() -> None:
    """TD 工程变体与论文式直接力矩 ADRC 仅参考整形配置不同。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    plain = configure_force_controller(source, variant="adrc-torque")
    shaped = configure_force_controller(source, variant="adrc-torque-td")

    assert plain.normal_force is not None
    assert shaped.normal_force is not None
    assert plain.normal_force.torque_adrc is not None
    assert shaped.normal_force.torque_adrc is not None
    assert plain.normal_force.torque_adrc.tracking_differentiator_bandwidth_rad_s is None
    assert shaped.normal_force.torque_adrc.tracking_differentiator_bandwidth_rad_s == 180.0
    assert (
        shaped.normal_force.torque_adrc.model_copy(
            update={"tracking_differentiator_bandwidth_rad_s": None}
        )
        == plain.normal_force.torque_adrc
    )


def test_force_reference_samples_derivatives_for_linear_and_smoothstep() -> None:
    """目标曲线同时提供二阶力矩 ADRC 所需的一、二阶参考导数。"""
    waypoints = (ForceWaypoint(t_s=0.0, force_n=2.0), ForceWaypoint(t_s=2.0, force_n=6.0))

    assert ForceReference(interpolation="linear", waypoints=waypoints).sample_at(1.0) == (
        4.0,
        2.0,
        0.0,
    )
    force, rate, acceleration = ForceReference(
        interpolation="smoothstep", waypoints=waypoints
    ).sample_at(0.5)
    assert force == pytest.approx(2.625)
    assert rate == pytest.approx(2.25)
    assert acceleration == pytest.approx(3.0)


def test_controller_variant_rejects_unknown_name_and_negative_seed() -> None:
    """运行时消融覆盖必须通过名称和随机种子校验。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    with pytest.raises(ValueError, match="controller_variant"):
        configure_force_controller(profile, variant="unknown")  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="non-negative"):
        configure_force_controller(profile, sensor_noise_seed=-1)
    with pytest.raises(ValueError, match="stiffness_estimator_method"):
        configure_force_controller(
            profile,
            stiffness_estimator_method="unknown",  # type: ignore[arg-type]
        )


def test_stiffness_estimator_method_is_a_runtime_profile_override() -> None:
    """估计器对比可覆盖方法而不修改源 profile。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    configured = configure_force_controller(
        source,
        variant="pid-stiffness-ff",
        stiffness_estimator_method="window_quadratic",
    )

    assert configured.normal_force is not None
    assert configured.normal_force.stiffness is not None
    assert configured.normal_force.stiffness.method == "window_quadratic"
    assert configured.normal_force.stiffness.torque_feedforward_gain == 0.0
    assert source.normal_force is not None
    assert source.normal_force.stiffness is not None
    assert source.normal_force.stiffness.method == "window_linear"


def test_force_tracking_run_writes_dynamic_reference_trace(tmp_path: Path) -> None:
    """短版动态力跟踪实验写出 trace，并计算非空跟踪指标。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="short_force_track",
        reference=ForceReference(
            interpolation="linear",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=2.0),
                ForceWaypoint(t_s=0.4, force_n=6.0),
                ForceWaypoint(t_s=0.8, force_n=4.0),
            ),
        ),
    )
    output_csv = tmp_path / "force_track.csv"

    result = run_force_tracking(
        ROOT / "configs/custom_parallel_gripper.yaml",
        task=task,
        output_csv=output_csv,
    )

    assert result.passed
    assert result.rmse_n >= 0.0
    assert output_csv.is_file()
    with output_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    tracking_rows = [row for row in rows if row["phase"] == "track_reference"]
    assert tracking_rows
    assert len({round(float(row["target_normal_force_n"]), 3) for row in tracking_rows}) > 1
    assert float(tracking_rows[0]["force_feedforward_torque_n_m"]) > 0.0
    assert float(tracking_rows[0]["estimated_contact_stiffness_n_per_m"]) > 0.0


def test_force_tracking_direct_torque_run_tracks_reference(tmp_path: Path) -> None:
    """direct-torque 短版运行完成跟踪：位置修正诊断为 0，前馈力矩仍写入命令。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="short_direct_torque",
        reference=ForceReference(
            interpolation="linear",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=2.0),
                ForceWaypoint(t_s=0.4, force_n=6.0),
                ForceWaypoint(t_s=0.8, force_n=4.0),
            ),
        ),
    )
    output_csv = tmp_path / "direct_torque.csv"

    result = run_force_tracking(
        ROOT / "configs/custom_parallel_gripper.yaml",
        task=task,
        controller_variant="direct-torque",
        output_csv=output_csv,
    )

    assert result.passed
    assert math.isfinite(result.rmse_n)
    assert output_csv.is_file()
    with output_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    tracking_rows = [row for row in rows if row["phase"] == "track_reference"]
    assert tracking_rows
    # 直接力矩路径：PID 与刚度位置修正诊断字段为 0，刚度估计仍在运行。
    assert all(float(row["pid_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(float(row["stiffness_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(
        math.isfinite(float(row["estimated_contact_stiffness_n_per_m"])) for row in tracking_rows
    )
    assert any(float(row["force_feedforward_torque_n_m"]) != 0.0 for row in tracking_rows)


@pytest.mark.parametrize("controller_variant", ["adrc-torque", "adrc-torque-td"])
def test_force_tracking_adrc_torque_run_writes_observer_diagnostics(
    tmp_path: Path, controller_variant: str
) -> None:
    """直接力矩 ADRC 两种参考路径均稳定完成并写出完整诊断量。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="short_adrc_torque",
        reference=ForceReference(
            interpolation="smoothstep",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=1.0),
                ForceWaypoint(t_s=0.4, force_n=3.0),
                ForceWaypoint(t_s=0.8, force_n=2.0),
            ),
        ),
    )
    output_csv = tmp_path / f"{controller_variant}.csv"

    result = run_force_tracking(
        ROOT / "configs/custom_parallel_gripper.yaml",
        task=task,
        controller_variant=controller_variant,  # type: ignore[arg-type]
        output_csv=output_csv,
    )

    assert result.passed
    assert math.isfinite(result.rmse_n)
    with output_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    tracking_rows = [row for row in rows if row["phase"] == "track_reference"]
    assert tracking_rows
    assert all(float(row["force_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(float(row["pid_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(float(row["stiffness_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(float(row["force_feedforward_torque_n_m"]) > 0.0 for row in tracking_rows)
    assert all(math.isfinite(float(row["torque_adrc_measurement_n"])) for row in tracking_rows)
    assert all(math.isfinite(float(row["torque_adrc_reference_force_n"])) for row in tracking_rows)
    assert all(math.isfinite(float(row["torque_adrc_estimated_force_n"])) for row in tracking_rows)
    assert all(
        math.isfinite(float(row["torque_adrc_input_gain_n_per_n_m_s2"])) for row in tracking_rows
    )

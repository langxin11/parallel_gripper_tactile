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
from parallel_gripper_tactile.profiles import load_profile


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

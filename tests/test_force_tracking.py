"""验证 waypoint 目标力跟踪任务。"""

import csv
from pathlib import Path

import pytest

from parallel_gripper_tactile.experiments.force_tracking import (
    ForceReference,
    ForceTrackingTask,
    ForceWaypoint,
    run_force_tracking,
)


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
    assert task.approach.feedforward_force_n == pytest.approx(2.0)
    assert task.reference.duration_s == pytest.approx(4.5)
    assert task.reference.target_at(3.5) == pytest.approx(10.0)


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

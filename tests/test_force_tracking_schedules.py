"""验证标准目标力 schedule 配置。"""

from pathlib import Path

import pytest

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    ("filename", "name", "interpolation", "duration"),
    [
        ("step.yaml", "step_force_tracking", "hold", 4.0),
        ("ramp.yaml", "ramp_force_tracking", "linear", 6.0),
        ("mixed_waypoints.yaml", "mixed_waypoint_force_tracking", "smoothstep", 7.5),
    ],
)
def test_standard_force_tracking_schedules(
    filename: str, name: str, interpolation: str, duration: float
) -> None:
    """三套标准曲线遵循统一 schema，并保留各自的插值语义。"""
    task = ForceTrackingTask.load(ROOT / "configs/force_tracking" / filename)

    assert task.name == name
    assert task.reference.interpolation == interpolation
    assert task.reference.duration_s == pytest.approx(duration)
    assert all(waypoint.force_n <= 6.0 for waypoint in task.reference.waypoints)


def test_step_schedule_has_hold_platform_and_steps() -> None:
    """阶跃曲线在平台保持目标，并在阶跃边界切换。"""
    reference = ForceTrackingTask.load(ROOT / "configs/force_tracking/step.yaml").reference

    assert reference.target_at(0.5) == pytest.approx(1.0)
    assert reference.target_at(2.0) == pytest.approx(6.0)
    assert reference.target_at(3.5) == pytest.approx(1.0)


def test_ramp_schedule_is_linear() -> None:
    """斜坡曲线中点目标按线性插值计算。"""
    reference = ForceTrackingTask.load(ROOT / "configs/force_tracking/ramp.yaml").reference

    assert reference.target_at(1.0) == pytest.approx(2.0)
    assert reference.target_at(3.0) == pytest.approx(4.5)
    assert reference.target_at(5.0) == pytest.approx(3.5)


def test_mixed_schedule_has_platform_ramp_and_unload() -> None:
    """综合曲线覆盖平台、平滑加载和卸载。"""
    reference = ForceTrackingTask.load(
        ROOT / "configs/force_tracking/mixed_waypoints.yaml"
    ).reference

    assert reference.target_at(1.0) == pytest.approx(1.0)
    assert reference.target_at(2.25) == pytest.approx(3.5)
    assert reference.target_at(4.5) == pytest.approx(6.0)
    assert reference.target_at(6.75) == pytest.approx(1.0)

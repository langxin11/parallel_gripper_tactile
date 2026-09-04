"""验证基于已知摩擦系数的目标抓取力调度。"""

import math

import pytest

from parallel_gripper_tactile.force_scheduling import (
    OracleTargetForceScheduler,
    TargetForceSchedulerConfig,
)


def test_oracle_scheduler_uses_symmetric_two_finger_friction_formula() -> None:
    """目标平均单侧力包含双指系数与安全系数。"""
    scheduler = OracleTargetForceScheduler(
        TargetForceSchedulerConfig(max_force_n=20.0, max_force_rate_n_s=100.0)
    )
    scheduler.reset(3.0)

    command = scheduler.update(3.2, 0.8, 0.1)

    assert command.raw_target_force_n == pytest.approx(3.0)
    assert command.target_force_n == pytest.approx(3.0)
    assert command.target_force_rate_n_s == pytest.approx(0.0)
    assert command.limited_by == ()


def test_oracle_scheduler_reports_minimum_and_friction_floor_limits() -> None:
    """零载荷使用最小目标力，过小摩擦系数使用配置下限。"""
    scheduler = OracleTargetForceScheduler(
        TargetForceSchedulerConfig(max_force_n=20.0, max_force_rate_n_s=100.0)
    )

    minimum = scheduler.update(tangential_demand_n=0.0, friction_coefficient=0.8, dt=0.1)
    floor = scheduler.update(tangential_demand_n=0.4, friction_coefficient=0.0, dt=0.1)

    assert minimum.target_force_n == pytest.approx(0.5)
    assert minimum.raw_target_force_n == pytest.approx(0.5)
    assert minimum.limited_by == ("minimum",)
    assert floor.raw_target_force_n == pytest.approx(6.0)
    assert floor.target_force_n == pytest.approx(6.0)


def test_oracle_scheduler_caps_force_and_preserves_raw_target() -> None:
    """力上限只改变输出，诊断量仍保留原始目标。"""
    scheduler = OracleTargetForceScheduler(TargetForceSchedulerConfig(max_force_rate_n_s=100.0))

    command = scheduler.update(tangential_demand_n=10.0, friction_coefficient=0.1, dt=0.1)

    assert command.raw_target_force_n == pytest.approx(75.0)
    assert command.target_force_n == pytest.approx(8.0)
    assert command.target_force_rate_n_s == pytest.approx(75.0)
    assert command.limited_by == ("maximum",)


def test_oracle_scheduler_rate_limits_in_both_directions() -> None:
    """上升和下降目标都使用相同的最大变化率。"""
    scheduler = OracleTargetForceScheduler(TargetForceSchedulerConfig(max_force_rate_n_s=2.0))
    scheduler.reset(2.0)

    rising = scheduler.update(tangential_demand_n=10.0, friction_coefficient=0.1, dt=0.25)
    scheduler.reset(6.0)
    falling = scheduler.update(tangential_demand_n=0.0, friction_coefficient=0.8, dt=0.25)

    assert rising.target_force_n == pytest.approx(2.5)
    assert rising.target_force_rate_n_s == pytest.approx(2.0)
    assert rising.limited_by == ("maximum", "rate")
    assert falling.target_force_n == pytest.approx(5.5)
    assert falling.target_force_rate_n_s == pytest.approx(-2.0)
    assert falling.limited_by == ("minimum", "rate")


@pytest.mark.parametrize(
    "updates",
    [
        {"safety_factor": 0.99},
        {"min_force_n": -0.1},
        {"min_force_n": 2.0, "max_force_n": 1.0},
        {"max_force_rate_n_s": 0.0},
        {"friction_floor": 0.0},
        {"safety_factor": math.nan},
        {"max_force_n": math.inf},
    ],
)
def test_scheduler_config_rejects_invalid_values(updates: dict[str, float]) -> None:
    """配置必须为有限值并满足安全范围。"""
    with pytest.raises(ValueError):
        TargetForceSchedulerConfig(**updates)


@pytest.mark.parametrize(
    ("inputs", "message"),
    [
        ({"tangential_demand_n": -0.1, "friction_coefficient": 0.8, "dt": 0.1}, "demand"),
        ({"tangential_demand_n": 1.0, "friction_coefficient": -0.1, "dt": 0.1}, "friction"),
        ({"tangential_demand_n": 1.0, "friction_coefficient": 0.8, "dt": 0.0}, "dt"),
        ({"tangential_demand_n": math.nan, "friction_coefficient": 0.8, "dt": 0.1}, "finite"),
        ({"tangential_demand_n": 1.0, "friction_coefficient": math.inf, "dt": 0.1}, "finite"),
    ],
)
def test_oracle_scheduler_rejects_invalid_updates(inputs: dict[str, float], message: str) -> None:
    """调度输入中的非有限值和无效范围必须显式失败。"""
    scheduler = OracleTargetForceScheduler()

    with pytest.raises(ValueError, match=message):
        scheduler.update(**inputs)


def test_oracle_scheduler_reset_validates_and_reinitializes_state() -> None:
    """reset 支持合法初值，并拒绝非有限或越界目标。"""
    scheduler = OracleTargetForceScheduler()
    scheduler.reset(4.0)

    assert scheduler.target_force_n == pytest.approx(4.0)
    scheduler.reset()
    assert scheduler.target_force_n == pytest.approx(0.5)

    for invalid in (-0.1, 8.1, math.nan, math.inf):
        with pytest.raises(ValueError):
            scheduler.reset(invalid)

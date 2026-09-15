"""固定摩擦先验调度器的关键物理语义与边界回归。"""

from dataclasses import replace
import math

import pytest

from dm_grasp_core.grasp.adaptive import AdaptiveLoadConfig, AdaptiveLoadScheduler


def test_absolute_side_demand_and_capacity_limit() -> None:
    """初始承载不扣基线，弱侧决定需求，裁剪仍保留能力不足证据。"""
    scheduler = AdaptiveLoadScheduler(AdaptiveLoadConfig(left_friction=0.5, right_friction=1.0))
    command = scheduler.update(left_tangential_n=2, right_tangential_n=2, dt=0.01)
    assert command.raw_target_force_n == pytest.approx(6)
    assert command.target_force_n == pytest.approx(0.51)
    limited = AdaptiveLoadScheduler(replace(scheduler.config, max_force_n=1))
    command = limited.update(left_tangential_n=2, right_tangential_n=2, dt=1)
    assert command.capacity_limited
    assert command.raw_target_force_n == pytest.approx(6)
    assert command.load_force_n == command.target_force_n == 1


def test_monotonic_rate_with_irregular_samples_and_unloading() -> None:
    """不同采样间隔下持续追赶绝对需求，卸载后保持已建立目标。"""
    scheduler = AdaptiveLoadScheduler(AdaptiveLoadConfig())
    previous = scheduler.target_force_n
    for index in range(600):
        dt = (0.004, 0.013, 0.007)[index % 3]
        load = 1.0 if index < 400 else 0.0
        command = scheduler.update(left_tangential_n=load, right_tangential_n=load, dt=dt)
        assert previous <= command.target_force_n <= previous + dt + 1e-12
        previous = command.target_force_n
    assert command.target_force_n == pytest.approx(2.5, abs=0.001)


def test_invalid_observation_does_not_advance_state() -> None:
    """坏数据先拒绝，不能污染下一次有效观测。"""
    scheduler = AdaptiveLoadScheduler(AdaptiveLoadConfig())
    for left, dt in ((math.nan, 0.01), (1e308, 0.01), (-1, 0.01), (1, 0)):
        with pytest.raises(ValueError):
            scheduler.update(left_tangential_n=left, right_tangential_n=1, dt=dt)
    assert scheduler.target_force_n == 0.5
    assert scheduler.update(left_tangential_n=1, right_tangential_n=1, dt=0.01).load_rate_n_s == 0

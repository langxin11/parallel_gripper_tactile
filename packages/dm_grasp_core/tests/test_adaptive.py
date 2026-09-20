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


def test_bidirectional_target_tracks_reduced_load_with_rate_limit() -> None:
    """允许下降时，目标按承载需求以同一速率边界缓慢卸力。"""
    config = AdaptiveLoadConfig(
        left_friction=0.5,
        right_friction=0.5,
        min_force_n=0.5,
        max_force_rate_n_s=1.0,
        filter_tau_s=0.01,
        allow_target_decrease=True,
    )
    scheduler = AdaptiveLoadScheduler(config)
    for _ in range(100):
        raised = scheduler.update(left_tangential_n=1.0, right_tangential_n=1.0, dt=0.01)
    assert raised.target_force_n == pytest.approx(1.5)
    previous = raised.target_force_n
    for _ in range(200):
        lowered = scheduler.update(left_tangential_n=0.0, right_tangential_n=0.0, dt=0.01)
        assert previous - 0.01000001 <= lowered.target_force_n <= previous
        previous = lowered.target_force_n
    assert lowered.raw_target_force_n == pytest.approx(0.0, abs=1e-6)
    assert lowered.target_force_n == pytest.approx(config.min_force_n, abs=2e-5)


def test_decrease_rate_can_be_slower_than_fast_load_response() -> None:
    """独立减力限幅不拖慢切向承载到来时的补力。"""
    config = AdaptiveLoadConfig(
        left_friction=0.5,
        right_friction=0.5,
        min_force_n=0.5,
        max_force_rate_n_s=10.0,
        max_force_decrease_rate_n_s=1.0,
        filter_tau_s=0.001,
        allow_target_decrease=True,
    )
    scheduler = AdaptiveLoadScheduler(config)

    raised = scheduler.update(left_tangential_n=2.0, right_tangential_n=2.0, dt=0.1)
    assert raised.target_force_rate_n_s == pytest.approx(10.0)
    lowered = scheduler.update(left_tangential_n=0.0, right_tangential_n=0.0, dt=0.1)
    assert lowered.target_force_rate_n_s == pytest.approx(-1.0)


def test_invalid_observation_does_not_advance_state() -> None:
    """坏数据先拒绝，不能污染下一次有效观测。"""
    scheduler = AdaptiveLoadScheduler(AdaptiveLoadConfig())
    for left, dt in ((math.nan, 0.01), (1e308, 0.01), (-1, 0.01), (1, 0)):
        with pytest.raises(ValueError):
            scheduler.update(left_tangential_n=left, right_tangential_n=1, dt=dt)
    assert scheduler.target_force_n == 0.5
    assert scheduler.update(left_tangential_n=1, right_tangential_n=1, dt=0.01).load_rate_n_s == 0

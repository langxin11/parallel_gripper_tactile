"""粒子摩擦候选只验证后验递推，不以合成数据替代物理实验。"""

from dataclasses import replace

import pytest

from dm_grasp_core.grasp.friction_particle import (
    ParticleFrictionConfig,
    ParticleFrictionEstimator,
)


@pytest.mark.parametrize("true_friction", [0.2, 0.3, 0.45, 0.8])
def test_incipient_event_localizes_friction_with_conservative_control_value(
    true_friction: float,
) -> None:
    """渐增粘着载荷加临界事件应定位真值，控制低分位保持保守。"""
    estimator = ParticleFrictionEstimator(ParticleFrictionConfig(seed=7))
    for index in range(250):
        estimator.update(
            0.8 * true_friction * index / 249,
            dt_s=0.004,
            stable=True,
            event=False,
        )

    snapshot = estimator.update(
        0.85 * true_friction,
        dt_s=0.004,
        stable=False,
        event=True,
        event_taxels=8,
    )

    assert snapshot.mean == pytest.approx(true_friction, rel=0.03)
    assert snapshot.lower <= snapshot.control_value <= snapshot.mean <= snapshot.upper
    assert snapshot.reason.startswith("event_update")


def test_unqualified_event_and_reset_do_not_fabricate_evidence() -> None:
    """无稳定帧及低质量事件不得改写后验，重置后回到先验。"""
    estimator = ParticleFrictionEstimator(ParticleFrictionConfig(seed=11))
    initial = estimator.snapshot(updated=False, reason="initial")
    ignored = estimator.update(
        0.2,
        dt_s=0.004,
        stable=False,
        event=True,
        event_taxels=2,
    )
    reset = estimator.reset()

    assert not ignored.updated and ignored.reason == "insufficient_evidence"
    assert not reset.updated and reset.reason == "reset"
    assert reset.mean == pytest.approx(initial.mean, abs=0.05)


def test_low_utilization_rebalancing_event_is_rejected() -> None:
    """高预载下的低利用率接触重分配不得伪装成摩擦临界事件。"""
    estimator = ParticleFrictionEstimator(ParticleFrictionConfig(seed=13))

    snapshot = estimator.update(
        0.07,
        dt_s=0.004,
        stable=False,
        event=True,
        event_taxels=7,
    )

    assert not snapshot.updated
    assert snapshot.reason == "event_below_utilization"


@pytest.mark.parametrize(
    "changes",
    [
        {"particle_count": 31},
        {"min_friction": 0.5, "max_friction": 0.2},
        {"control_quantile": 0.5},
        {"minimum_event_taxels": 0},
        {"minimum_event_utilization": 0.01},
        {"seed": -1},
    ],
)
def test_particle_config_rejects_invalid_values(changes: dict[str, object]) -> None:
    """粒子数量、范围、概率和随机种子须满足边界。"""
    with pytest.raises(ValueError):
        replace(ParticleFrictionConfig(), **changes)

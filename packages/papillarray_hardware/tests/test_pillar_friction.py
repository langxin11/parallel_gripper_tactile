"""逐 pillar 自主摩擦估计与原厂结果配对测试。"""

from dataclasses import replace

import pytest

from papillarray_hardware import TactileSnapshot
from papillarray_hardware.pillar_friction import PillarFrictionConfig, PillarFrictionEstimator
from papillarray_hardware.standalone import StandaloneSlipConfig, StandaloneSlipSession


def _snapshot(t: float, left_ratios: tuple[float, float], right_ratio: float = 0.1):
    """用单位法向力构造可直接解释切法向比的双侧快照。"""
    return TactileSnapshot(
        received_at_s=t,
        packet_counter=round(t * 100),
        timestamp_us=round(t * 1e6),
        left_force_n=2.0,
        right_force_n=2.0,
        raw_left_fz_n=2.0,
        raw_right_fz_n=2.0,
        left_taxel_forces_n=tuple((ratio, 0.0, 1.0) for ratio in left_ratios),
        right_taxel_forces_n=((right_ratio, 0.0, 1.0), (right_ratio, 0.0, 1.0)),
    )


def _config() -> PillarFrictionConfig:
    """构造适合短合成时序的逐点估计参数。"""
    return PillarFrictionConfig(
        window_duration_s=0.04,
        min_samples_per_half=2,
        min_ratio=0.1,
        arming_ratio_increase=0.04,
        saturation_ratio_increase=0.005,
        redistribution_share_drop=0.02,
        min_side_shear_increase_n=0.01,
        confirm_duration_s=0.02,
        estimate_quantile=0.8,
        safety_discount=0.8,
    )


def test_single_pillar_rise_then_saturation_freezes_conservative_estimate() -> None:
    """局部比值先上升再饱和且剪切继续重分配时，只确认对应 pillar。"""
    estimator = PillarFrictionEstimator(_config())
    mask = ((True, True), (True, True))
    estimator.start(mask)
    pillar_zero = (0.10, 0.15, 0.20, 0.25, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30)
    pillar_one = (0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55)
    events = []
    for index, (first, second) in enumerate(zip(pillar_zero, pillar_one, strict=True)):
        events.extend(estimator.update(_snapshot(index * 0.01, (first, second)), mask))
    left = [event for event in events if event.side == 0]
    assert [(event.pillar_id, event.raw_mu) for event in left] == [(0, pytest.approx(0.23))]
    assert left[0].conservative_mu == pytest.approx(0.184)
    assert [event for event in events if event.side == 1] == []


def test_stable_subcritical_ratio_does_not_claim_friction_coefficient() -> None:
    """稳定粘着力比只是已用摩擦比例，不能被误报为极限摩擦系数。"""
    estimator = PillarFrictionEstimator(_config())
    mask = ((True, True), (True, True))
    estimator.start(mask)
    events = []
    for index in range(20):
        events.extend(estimator.update(_snapshot(index * 0.01, (0.15, 0.12)), mask))
    assert events == []


def test_cross_validation_pairs_same_pillar_once() -> None:
    """自主与原厂估计只在同侧同 pillar 配对，并保存原始值误差。"""
    session = StandaloneSlipSession(StandaloneSlipConfig())
    session._own_estimates[(0, 1)] = (0.36, 0.288)
    session._estimates[(0, 1)] = 0.4
    sample = _snapshot(1.0, (0.2, 0.2))
    events = session._cross_validation_events(sample, 1.0)
    assert len(events) == 1
    assert events[0] == {
        "event": "friction_cross_validation",
        "side": "left",
        "pillar_id": 1,
        "own_raw_mu": 0.36,
        "own_conservative_mu": 0.288,
        "native_mu": 0.4,
        "absolute_error": pytest.approx(0.04),
        "relative_error_to_native": pytest.approx(0.1),
        "device_timestamp_us": 1_000_000,
        "packet_counter": 100,
        "received_at_s": 1.0,
        "host_monotonic_s": 1.0,
    }
    assert session._cross_validation_events(replace(sample, timestamp_us=1_010_000), 1.01) == []


@pytest.mark.parametrize(
    "changes",
    [
        {"window_duration_s": 0.0},
        {"min_samples_per_half": True},
        {"estimate_quantile": 0.0},
        {"safety_discount": 1.1},
        {"max_friction_coefficient": 0.01},
    ],
)
def test_invalid_pillar_friction_configuration_is_rejected(changes: dict[str, object]) -> None:
    """拒绝无效时间窗、样本数、折减和摩擦范围。"""
    with pytest.raises(ValueError):
        replace(_config(), **changes)

"""验证微滑移探测与保守摩擦系数估计。"""

import math

import pytest

import parallel_gripper_tactile.friction_estimation as legacy_friction
import parallel_gripper_tactile.perception.friction as canonical_friction
from parallel_gripper_tactile.friction_estimation import (
    ConservativeFrictionEstimator,
    FrictionEstimatorConfig,
    FrictionProbeObservation,
)


def test_legacy_module_preserves_friction_public_object_identity() -> None:
    """旧入口导出的公共对象必须与新 perception 实现保持同一对象。"""
    for name in legacy_friction.__all__:
        assert getattr(legacy_friction, name) is getattr(canonical_friction, name)


def _update(
    estimator: ConservativeFrictionEstimator,
    *,
    probe: float,
    normal: float = 10.0,
    shear: float,
    dt: float = 0.01,
):
    """把合计力均分到左右两侧，简化测试输入。"""
    return estimator.update(
        FrictionProbeObservation(
            tangential_demand_n=probe,
            probe_excitation_n=probe,
            left_normal_force_n=normal / 2.0,
            left_shear_force_n=shear / 2.0,
            right_normal_force_n=normal / 2.0,
            right_shear_force_n=shear / 2.0,
            dt=dt,
        )
    )


def test_estimator_collects_touch_only_ratios_without_false_detection() -> None:
    """触觉剪切力持续支撑探测载荷时只收集起滑前样本。"""
    estimator = ConservativeFrictionEstimator()

    result = _update(estimator, probe=4.0, shear=4.0)

    assert result.state == "collecting"
    assert not result.slip_detected
    assert result.consecutive_mismatch_count == 0
    assert result.support_residual_n == pytest.approx(0.0)
    assert result.support_utilization == pytest.approx(1.0)
    assert result.sample_count == 1
    assert result.friction_coefficient == pytest.approx(0.2)
    assert result.using_fallback


def test_estimator_detects_persistent_mismatch_and_freezes_conservative_quantile() -> None:
    """持续失配后用起滑前窗口的高分位数乘折减系数。"""
    config = FrictionEstimatorConfig(
        window_size=5,
        min_samples=5,
        estimate_quantile=0.75,
        safety_discount=0.8,
        mismatch_confirm_s=0.02,
    )
    estimator = ConservativeFrictionEstimator(config)
    for ratio in (0.4, 0.5, 0.6, 0.7, 0.8):
        _update(estimator, probe=10.0 * ratio, shear=10.0 * ratio)

    suspected = _update(estimator, probe=10.0, shear=6.0)
    detected = _update(estimator, probe=10.0, shear=6.0)

    assert suspected.state == "suspected_slip"
    assert suspected.consecutive_mismatch_count == 1
    assert suspected.mismatch_duration_s == pytest.approx(0.01)
    assert detected.state == "estimated"
    assert detected.slip_detected
    assert detected.consecutive_mismatch_count == 2
    assert detected.sample_count == 5
    assert detected.raw_friction_coefficient == pytest.approx(0.7)
    assert detected.friction_coefficient == pytest.approx(0.56)
    assert not detected.using_fallback

    frozen = _update(estimator, probe=20.0, normal=2.0, shear=0.0)
    assert frozen.state == "estimated"
    assert frozen.friction_coefficient == pytest.approx(0.56)
    assert frozen.sample_count == 5


def test_estimator_excludes_mismatch_samples_and_falls_back_when_window_is_short() -> None:
    """起滑候选不进入窗口，样本不足时检测成立但继续安全回退。"""
    estimator = ConservativeFrictionEstimator(
        FrictionEstimatorConfig(min_samples=3, mismatch_confirm_s=0.01)
    )
    _update(estimator, probe=4.0, shear=4.0)

    result = _update(estimator, probe=5.0, shear=1.0)

    assert result.state == "detected_fallback"
    assert result.slip_detected
    assert result.sample_count == 1
    assert result.raw_friction_coefficient is None
    assert result.friction_coefficient == pytest.approx(0.2)
    assert result.using_fallback


def test_estimator_requires_continuous_mismatch_duration() -> None:
    """恢复载荷支撑会清零候选计数和累计时长。"""
    estimator = ConservativeFrictionEstimator(
        FrictionEstimatorConfig(min_samples=1, mismatch_confirm_s=0.02)
    )
    _update(estimator, probe=4.0, shear=4.0)
    first = _update(estimator, probe=5.0, shear=1.0)
    recovered = _update(estimator, probe=4.0, shear=4.0)
    second = _update(estimator, probe=5.0, shear=1.0)

    assert first.state == "suspected_slip"
    assert recovered.state == "collecting"
    assert recovered.consecutive_mismatch_count == 0
    assert recovered.mismatch_duration_s == 0.0
    assert second.state == "suspected_slip"
    assert not second.slip_detected


def test_estimator_detects_ratio_saturation_without_force_residual() -> None:
    """摩擦比先上升再饱和时，即使残差判据关闭也能确认力域起滑。"""
    estimator = ConservativeFrictionEstimator(
        FrictionEstimatorConfig(
            window_size=20,
            min_samples=5,
            safety_discount=0.9,
            support_residual_threshold_n=100.0,
            support_utilization_threshold=0.0,
            mismatch_confirm_s=0.02,
            ratio_trend_enabled=True,
            trend_window_size=5,
            arming_slope_threshold_per_s=0.5,
            saturation_slope_threshold_per_s=0.1,
        )
    )
    for index in range(1, 11):
        probe = 0.1 * index
        _update(estimator, probe=probe, normal=1.0, shear=probe)

    result = None
    for index in range(1, 8):
        result = _update(estimator, probe=1.0 + 0.1 * index, normal=1.0, shear=1.0)
        if result.slip_detected:
            break

    assert result is not None
    assert result.slip_detected
    assert result.trend_armed
    assert result.ratio_slope_per_s <= 0.1
    assert result.raw_friction_coefficient == pytest.approx(1.0)
    assert result.friction_coefficient == pytest.approx(0.9)


@pytest.mark.parametrize(
    ("probe", "normal", "expected_state"),
    [(0.1, 10.0, "no_probe"), (1.0, 0.1, "insufficient_contact")],
)
def test_estimator_falls_back_and_clears_discontinuous_probe_sequences(
    probe: float, normal: float, expected_state: str
) -> None:
    """无有效探测或接触不足时使用回退值并清除旧窗口。"""
    estimator = ConservativeFrictionEstimator()
    _update(estimator, probe=2.0, shear=2.0)

    result = _update(estimator, probe=probe, normal=normal, shear=0.0)

    assert result.state == expected_state
    assert result.sample_count == 0
    assert result.consecutive_mismatch_count == 0
    assert result.using_fallback


@pytest.mark.parametrize(
    ("ratio", "expected"),
    [(0.01, 0.1), (4.0, 1.0)],
)
def test_estimator_limits_discounted_estimate(ratio: float, expected: float) -> None:
    """保守估计仍受明确的摩擦系数上下限约束。"""
    estimator = ConservativeFrictionEstimator(
        FrictionEstimatorConfig(
            min_samples=1,
            min_friction_coefficient=0.1,
            max_friction_coefficient=1.0,
            safety_discount=0.5,
            mismatch_confirm_s=0.01,
        )
    )
    normal = 20.0 if ratio < 0.1 else 10.0
    shear = ratio * normal
    _update(estimator, probe=shear, normal=normal, shear=shear)

    result = _update(estimator, probe=max(1.0, shear + 1.0), shear=0.0)

    assert result.friction_coefficient == pytest.approx(expected)


def test_estimator_finalize_does_not_promote_static_utilization_to_friction() -> None:
    """未检测起滑时 finalize 明确回退，不把静摩擦利用率当作极限。"""
    estimator = ConservativeFrictionEstimator(
        FrictionEstimatorConfig(min_samples=1, fallback_friction_coefficient=0.15)
    )
    _update(estimator, probe=4.0, shear=4.0)

    result = estimator.finalize()

    assert result.state == "finalized_fallback"
    assert not result.slip_detected
    assert result.sample_count == 1
    assert result.friction_coefficient == pytest.approx(0.15)
    assert result.using_fallback


@pytest.mark.parametrize(
    "updates",
    [
        {"window_size": 0},
        {"window_size": True},
        {"min_samples": 0},
        {"window_size": 2, "min_samples": 3},
        {"estimate_quantile": 0.0},
        {"safety_discount": 1.1},
        {"min_friction_coefficient": 0.0},
        {"min_friction_coefficient": 0.5, "max_friction_coefficient": 0.4},
        {"fallback_friction_coefficient": 3.0},
        {"min_probe_load_n": 0.0},
        {"min_total_normal_force_n": 0.0},
        {"support_residual_threshold_n": -0.1},
        {"support_utilization_threshold": 1.0},
        {"mismatch_confirm_s": 0.0},
        {"ratio_trend_enabled": 1},
        {"trend_window_size": 1},
        {"arming_slope_threshold_per_s": 0.0},
        {"saturation_slope_threshold_per_s": -0.1},
        {"arming_slope_threshold_per_s": 0.1, "saturation_slope_threshold_per_s": 0.1},
        {"max_side_ratio_difference": -0.1},
        {"estimate_quantile": math.nan},
    ],
)
def test_estimator_config_rejects_invalid_values(updates: dict[str, float]) -> None:
    """配置必须有限且满足窗口、限值和阈值约束。"""
    with pytest.raises(ValueError):
        FrictionEstimatorConfig(**updates)


@pytest.mark.parametrize(
    "inputs",
    [
        (-1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.01),
        (1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 0.01),
        (1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 0.01),
        (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0),
        (math.nan, 1.0, 1.0, 1.0, 1.0, 1.0, 0.01),
        (1.0, 1.0, 1.0, 1.0, 1.0, math.inf, 0.01),
    ],
)
def test_estimator_rejects_invalid_inputs(inputs: tuple[float, ...]) -> None:
    """非有限输入、负力和非正时间步必须显式失败。"""
    estimator = ConservativeFrictionEstimator()

    with pytest.raises(ValueError):
        estimator.update(FrictionProbeObservation(*inputs))


def test_estimator_reset_restores_fallback_and_clears_detection() -> None:
    """reset 清除已锁存估计并恢复默认回退状态。"""
    estimator = ConservativeFrictionEstimator(
        FrictionEstimatorConfig(min_samples=1, mismatch_confirm_s=0.01)
    )
    _update(estimator, probe=4.0, shear=4.0)
    detected = _update(estimator, probe=5.0, shear=1.0)
    assert detected.slip_detected

    estimator.reset()
    result = _update(estimator, probe=0.0, shear=0.0)

    assert result.state == "no_probe"
    assert not result.slip_detected
    assert result.sample_count == 0
    assert result.friction_coefficient == pytest.approx(0.2)
    assert result.using_fallback


def test_estimator_reset_clears_ratio_trend_history() -> None:
    """reset 后的新探测不会继承旧窗口斜率或 armed 状态。"""
    estimator = ConservativeFrictionEstimator(
        FrictionEstimatorConfig(
            ratio_trend_enabled=True,
            trend_window_size=3,
            arming_slope_threshold_per_s=0.5,
            saturation_slope_threshold_per_s=0.1,
        )
    )
    for probe in (0.2, 0.3, 0.4):
        _update(estimator, probe=probe, normal=1.0, shear=probe)

    estimator.reset()
    result = _update(estimator, probe=0.5, normal=1.0, shear=0.5)

    assert result.sample_count == 1
    assert result.ratio_slope_per_s == 0.0
    assert not result.trend_armed

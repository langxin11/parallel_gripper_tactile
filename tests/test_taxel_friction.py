"""验证逐 taxel 接触筛选与局部摩擦利用率。"""

import math

import numpy as np
import pytest

from parallel_gripper_tactile.taxel_friction import (
    ForceOnlySlipConfig,
    ForceOnlyTaxelSlipDetector,
    TaxelFrictionConfig,
    TaxelFrictionObserver,
)


def _forces(normals: list[list[float]], shears: list[list[float]]) -> np.ndarray:
    """构造只沿局部 X 方向承受剪切的触觉网格。"""
    force = np.zeros((3, len(normals), len(normals[0])))
    force[0] = shears
    force[2] = normals
    return force


def test_observer_requires_confirmed_entry_and_exit() -> None:
    """接触进入和退出都必须持续达到确认时间。"""
    observer = TaxelFrictionObserver(
        TaxelFrictionConfig(
            contact_enter_force_n=0.1,
            contact_exit_force_n=0.05,
            transition_confirm_s=0.02,
        )
    )
    contact = _forces([[0.2]], [[0.1]])
    weak = _forces([[0.04]], [[0.01]])

    assert observer.update(contact, contact, dt=0.01).active_count == 0
    entered = observer.update(contact, contact, dt=0.01)
    assert entered.active_count == 2
    assert entered.weighted_ratio == pytest.approx(0.5)
    assert observer.update(weak, weak, dt=0.01).active_count == 2
    exited = observer.update(weak, weak, dt=0.01)
    assert exited.active_count == 0
    assert math.isnan(exited.weighted_ratio)


def test_observer_uses_only_active_taxels_for_summary() -> None:
    """汇总比值排除未达到接触阈值的弱法向 taxel。"""
    observer = TaxelFrictionObserver(
        TaxelFrictionConfig(contact_enter_force_n=0.1, transition_confirm_s=0.01)
    )
    left = _forces([[0.2, 0.01]], [[0.1, 1.0]])
    right = _forces([[0.4, 0.0]], [[0.1, 2.0]])

    result = observer.update(left, right, dt=0.01)

    assert result.active_count == 2
    assert result.left_contact_mask.tolist() == [[True, False]]
    assert result.right_contact_mask.tolist() == [[True, False]]
    assert result.weighted_ratio == pytest.approx(1.0 / 3.0)
    assert result.ratio_p90 == pytest.approx(0.475)
    assert result.maximum_ratio == pytest.approx(0.5)
    assert math.isnan(result.left_ratio[0, 1])


@pytest.mark.parametrize(
    "updates",
    [
        {"contact_enter_force_n": 0.0},
        {"contact_enter_force_n": 0.1, "contact_exit_force_n": 0.1},
        {"contact_exit_force_n": -0.1},
        {"transition_confirm_s": 0.0},
        {"transition_confirm_s": math.nan},
    ],
)
def test_config_rejects_invalid_thresholds(updates: dict[str, float]) -> None:
    """配置拒绝无效阈值和非有限确认时间。"""
    with pytest.raises(ValueError):
        TaxelFrictionConfig(**updates)


def test_observer_rejects_shape_changes() -> None:
    """一次观测序列中的触觉网格形状必须固定。"""
    observer = TaxelFrictionObserver(TaxelFrictionConfig(transition_confirm_s=0.01))
    one = np.zeros((3, 1, 1))
    two = np.zeros((3, 1, 2))
    observer.update(one, one, dt=0.01)

    with pytest.raises(ValueError, match="remain constant"):
        observer.update(two, two, dt=0.01)


def test_force_only_detector_latches_ratio_saturation_during_continued_loading() -> None:
    """局部比值先上升后饱和且同侧剪切继续增加时锁存保守估计。"""
    observer = TaxelFrictionObserver(
        TaxelFrictionConfig(contact_enter_force_n=0.05, transition_confirm_s=0.01)
    )
    detector = ForceOnlyTaxelSlipDetector(
        ForceOnlySlipConfig(
            window_size=4,
            arming_ratio_increase=0.1,
            saturation_ratio_increase=0.01,
            min_side_shear_increase_n=0.01,
            confirm_s=0.02,
            estimate_quantile=1.0,
            safety_discount=0.9,
        )
    )

    result = None
    event_count = 0
    for first, second in (
        (0.2, 0.2),
        (0.3, 0.3),
        (0.4, 0.4),
        (0.5, 0.5),
        (0.5, 0.6),
        (0.5, 0.7),
        (0.5, 0.8),
        (0.5, 0.9),
    ):
        forces = _forces([[1.0, 1.0]], [[first, second]])
        local = observer.update(forces, forces, dt=0.01)
        result = detector.update(local, dt=0.01)
        event_count += result.event_count

    assert result is not None
    assert event_count == 2
    assert result.detected_count == 2
    assert result.left_detected_mask.tolist() == [[True, False]]
    assert result.right_detected_mask.tolist() == [[True, False]]
    assert result.left_friction_estimate == pytest.approx(0.45)
    assert result.right_friction_estimate == pytest.approx(0.45)


def test_force_only_detector_does_not_arm_a_constant_ratio() -> None:
    """没有先经历摩擦比上升的稳定接触不会产生局部起滑事件。"""
    observer = TaxelFrictionObserver(
        TaxelFrictionConfig(contact_enter_force_n=0.05, transition_confirm_s=0.01)
    )
    detector = ForceOnlyTaxelSlipDetector(ForceOnlySlipConfig(window_size=4))
    result = None
    for _ in range(10):
        forces = _forces([[1.0, 1.0]], [[0.4, 0.4]])
        result = detector.update(observer.update(forces, forces, dt=0.01), dt=0.01)

    assert result is not None
    assert result.detected_count == 0
    assert not result.left_armed_mask.any()

"""验证触觉专用接口、候选确认与估计冻结。"""

from dataclasses import fields, replace
import inspect

import numpy as np
import pytest

import parallel_gripper_tactile.perception.slip as canonical_slip
import parallel_gripper_tactile.perception.taxels as canonical_taxels
import parallel_gripper_tactile.tactile_slip as legacy_slip
import parallel_gripper_tactile.taxel_friction as legacy_taxels
from parallel_gripper_tactile.friction_estimation import FrictionEstimatorConfig
from parallel_gripper_tactile.tactile_slip import (
    TactileFeatures,
    TactileFrictionEstimator,
    TactileSlipConfig,
    TactileFeatureComputer,
)
from parallel_gripper_tactile.taxel_friction import TaxelFrictionObserver


def test_legacy_perception_modules_preserve_public_object_identity():
    """旧触觉入口导出的对象必须与新 perception 实现保持同一对象。"""
    for name in legacy_slip.__all__:
        assert getattr(legacy_slip, name) is getattr(canonical_slip, name)
    for name in legacy_taxels.__all__:
        assert getattr(legacy_taxels, name) is getattr(canonical_taxels, name)


def test_interface_excludes_privileged_inputs():
    """数据结构与在线入口均不接受载荷真值、运动和摩擦真值。"""
    names = {f.name for f in fields(TactileFeatures)}
    assert names.isdisjoint(
        {
            "F_demand",
            "tangential_demand_n",
            "probe_excitation_n",
            "mu_true",
            "velocity",
            "displacement",
        }
    )
    assert list(inspect.signature(TactileFrictionEstimator.update_slip_detector).parameters) == [
        "self",
        "features",
        "dt",
    ]


def test_candidate_window_freezes_and_confirmation_is_continuous():
    """短暂候选撤销；持续证据确认后永不采入滑动样本。"""
    estimator = TactileFrictionEstimator(
        FrictionEstimatorConfig(min_samples=3), TactileSlipConfig(confirm_duration_s=0.03)
    )
    for rho in (0.2, 0.3, 0.4, 0.5, 0.6):
        estimator.update_slip_detector(
            TactileFeatures(rho, rho, 1, d_rho_global_dt=0.3, total_normal_n=2, valid=True), 0.01
        )
    plateau = TactileFeatures(0.6, 0.6, 1, total_normal_n=2, valid=True)
    assert not estimator.update_slip_detector(plateau, 0.01).slip_detected
    estimator.update_slip_detector(replace(plateau, d_rho_global_dt=0.3), 0.01)
    assert estimator.duration == 0
    for _ in range(2):
        assert not estimator.update_slip_detector(plateau, 0.01).slip_detected
    result = estimator.update_slip_detector(plateau, 0.01)
    assert result.slip_detected
    assert result.raw_friction_coefficient <= 0.6
    assert result.friction_coefficient == pytest.approx(result.raw_friction_coefficient * 0.8)
    assert estimator.update_slip_detector(replace(plateau, rho_global=10), 0.01) == result


def test_constant_contact_never_arms_and_contact_loss_clears_history():
    """恒定摩擦利用率不能凭幅值宣布起滑；失去接触必须重建历史。"""
    estimator = TactileFrictionEstimator(FrictionEstimatorConfig(), TactileSlipConfig())
    for _ in range(100):
        result = estimator.update_slip_detector(
            TactileFeatures(0.8, 0.8, 1, total_normal_n=2, valid=True), 0.01
        )
    assert not result.slip_detected
    estimator.update_slip_detector(TactileFeatures(), 0.01)
    assert len(estimator.samples) == 0
    assert estimator.finalize().using_fallback


def test_feature_filter_rejects_weak_contacts_and_preserves_force_ratio():
    """无接触点的剪切噪声不进入统计，均匀接触的比值保持正确。"""
    observer = TaxelFrictionObserver()
    computer = TactileFeatureComputer(TactileSlipConfig())
    force = np.zeros((3, 2, 2))
    force[2, 0, 0] = 1
    force[0, 0, 0] = 0.4
    force[0, 1, 1] = 10
    for _ in range(100):
        features = computer.compute_tactile_features(observer.update(force, force, dt=0.01), 0.01)
    assert features.valid
    assert features.rho_global == pytest.approx(0.4)
    assert features.rho_q95 == pytest.approx(0.4)
    assert features.distribution_change == pytest.approx(0)


@pytest.mark.parametrize(
    "settings",
    [{"release_threshold": 0.8}, {"confirm_duration_s": 0}, {"smoothing_s": float("nan")}],
)
def test_invalid_detector_parameters_rejected(settings):
    """配置错误在仿真开始前被拒绝。"""
    with pytest.raises(ValueError):
        TactileSlipConfig(**settings)

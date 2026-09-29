"""深度经验先验的数学边界、接触门控及闭环采用契约。"""

from dataclasses import replace

import numpy as np
import pytest

from dm_grasp_core.grasp.adaptive import AdaptiveLoadConfig
from dm_grasp_core.grasp.friction_depth import DepthFrictionPriorConfig
from dm_grasp_core.grasp.friction_particle import ParticleFrictionConfig
from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig, UnifiedAdaptivePolicy
from dm_grasp_core.tactile.risk import TaxelRiskObservation


def test_log_curve_bounds_monotonicity_and_concavity():
    """端点精确饱和，中间斜率为正且递减，负深度不提高先验。"""
    config = DepthFrictionPriorConfig()
    assert config.friction_at(-1) == config.friction_at(0.0002) == 0.1
    assert config.friction_at(0.002) == config.friction_at(1) == 0.6
    assert config.friction_at(0.001) == pytest.approx(0.3894509614182405)
    values = [config.friction_at(x) for x in np.linspace(0.0002, 0.002, 100)]
    assert np.all(np.diff(values) > 0)
    assert np.all(np.diff(values, n=2) < 0)
    assert DepthFrictionPriorConfig(curvature=1e-12).friction_at(0.0011) == pytest.approx(0.35)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"min_friction": 0},
        {"max_friction": 0.1},
        {"onset_depth_m": -1},
        {"saturation_depth_m": 0.0002},
        {"curvature": 0},
        {"compression_sign": 0},
        {"min_contact_taxels": True},
        {"min_contact_taxels": 4.5},
        {"min_contact_taxels": 10},
        {"stable_time_s": float("nan")},
        {"max_increase_per_s": float("inf")},
    ],
)
def test_invalid_depth_parameters_are_rejected(kwargs):
    """配置错误不能进入在线策略。"""
    with pytest.raises(ValueError):
        DepthFrictionPriorConfig(**kwargs)


def _policy(particle=False, *, rate=0.05):
    """使用可观测门控和固定载荷构造策略，不涉及硬件。"""
    return UnifiedAdaptivePolicy(
        UnifiedAdaptiveConfig(
            load=AdaptiveLoadConfig(
                left_friction=0.1,
                right_friction=0.1,
                allow_target_decrease=True,
                max_force_decrease_rate_n_s=1.0,
            ),
            risk_enabled=True,
            friction_update_enabled=True,
            risk_step_enabled=False,
            friction_expiry_s=None,
            depth_friction_prior=DepthFrictionPriorConfig(max_increase_per_s=rate),
            particle_friction=ParticleFrictionConfig() if particle else None,
        )
    )


def _observation(**kwargs):
    """稳定接触掩码来自受控证据，摩擦利用率低于初始先验。"""
    return replace(
        TaxelRiskObservation(
            valid=True,
            reason="observing",
            left_utilization=0.05,
            right_utilization=0.05,
            left_valid_mask=(True,) * 9,
            right_valid_mask=(True,) * 9,
        ),
        **kwargs,
    )


def _step(policy, time, *, depth=0.002, observation=None, enabled=True):
    """同包深度与力注入共享接口，允许缺失深度。"""
    forces = np.tile([0.01, 0, 0.2], (9, 1))
    displacement = None if depth is None else np.tile([0, 0, depth], (9, 1))
    return policy.update(
        forces,
        forces,
        time_s=time,
        measured_force_n=1.0,
        observation=_observation() if observation is None else observation,
        enabled=enabled,
        left_displacements_m=displacement,
        right_displacements_m=displacement,
    )


@pytest.mark.parametrize("particle", [False, True])
def test_prior_locks_once_and_applied_friction_rises_slowly(particle):
    """深度先验锁定后不因新深度重置后验，采用值每秒最多上升配置量。"""
    policy = _policy(particle)
    for tick in range(4):
        command = _step(policy, tick * 0.05, enabled=False)
    assert command.left_friction == 0.1
    previous = 0.1
    for tick in range(4, 20):
        command = _step(policy, tick * 0.05)
        assert previous <= command.left_friction <= previous + 0.05 * 0.05 + 1e-12
        previous = command.left_friction
    assert command.left_friction > 0.1
    assert command.trace_fields()["adaptive_left_depth_prior_value"] == 0.6
    if particle:
        estimator = policy._particle_filters[0]
        assert estimator.config.prior_mean == 0.6
    for tick in range(20, 30):
        command = _step(policy, tick * 0.05, depth=0.0003)
    assert command.trace_fields()["adaptive_left_depth_prior_value"] == 0.6
    assert command.trace_fields()["adaptive_left_depth_prior_candidate"] < 0.2


@pytest.mark.parametrize("depth", [None, -0.002, float("nan")])
def test_missing_or_wrong_sign_depth_cannot_raise_prior(depth):
    """缺失、非有限或方向错误的位移不得被转换成高摩擦。"""
    policy = _policy()
    for tick in range(20):
        command = _step(policy, tick * 0.05, depth=depth)
    assert command.left_friction == 0.1
    assert not command.trace_fields()["adaptive_left_depth_prior_locked"]


def test_low_coverage_and_unstable_depth_restart_confirmation():
    """三个触点或持续变化的深度不满足先验锁定条件。"""
    policy = _policy()
    sparse = _observation(left_valid_mask=(True,) * 3 + (False,) * 6)
    for tick in range(20):
        command = _step(policy, tick * 0.05, observation=sparse)
    assert command.left_friction == 0.1
    assert not command.trace_fields()["adaptive_left_depth_prior_locked"]
    for tick in range(20, 40):
        command = _step(policy, tick * 0.05, depth=0.0003 if tick % 2 else 0.002)
    assert not command.trace_fields()["adaptive_left_depth_prior_locked"]


def test_contact_reset_restores_point_one_and_particle_prior():
    """接触重置同时撤销已锁定高先验及粒子初始化，重复帧不再次更新。"""
    policy = _policy(True)
    for tick in range(20):
        command = _step(policy, tick * 0.05)
    assert command.left_friction > 0.1
    reset = _step(policy, 1.0, observation=_observation(contact_changed=True))
    assert reset.left_friction == 0.1
    assert not reset.trace_fields()["adaptive_left_depth_prior_locked"]
    assert policy._particle_filters[0].config.prior_mean == 0.1
    duplicate = _step(policy, 1.0)
    assert duplicate.left_friction == reset.left_friction
    assert duplicate.observation_reason == "duplicate_timestamp"


def test_lower_friction_evidence_overrides_depth_prior_and_risk_freezes_unloading():
    """更低摩擦事件可在深度未确认前生效，之后不被高深度先验抹除。"""
    policy = _policy()
    _step(policy, 0)
    event = _observation(
        risk=1,
        event_id=1,
        left_candidate=0.08,
        right_candidate=0.08,
        left_quality=1,
        right_quality=1,
    )
    command = _step(policy, 0.05, observation=event)
    assert command.left_friction == pytest.approx(0.064)
    for tick in range(2, 20):
        command = _step(policy, tick * 0.05)
    assert command.left_friction == pytest.approx(0.064)
    before = command.load.target_force_n
    command = _step(policy, 1.0, observation=_observation(risk=1))
    assert command.load.target_force_n >= before


def test_sample_gap_clears_depth_lock():
    """缺包不能让旧深度先验跨越未知接触段。"""
    policy = _policy()
    for tick in range(20):
        _step(policy, tick * 0.05)
    command = _step(policy, 2.0)
    assert command.left_friction == 0.1
    assert not command.trace_fields()["adaptive_left_depth_prior_locked"]


@pytest.mark.parametrize("particle", [False, True])
def test_event_before_depth_lock_is_never_erased_by_optimistic_prior(particle):
    """控制值尚未上调也不代表没有摩擦证据，先验不得覆盖已消费事件。"""
    policy = _policy(particle)
    _step(policy, 0)
    event = _observation(
        risk=1, event_id=1, left_candidate=0.2, right_candidate=0.2, left_quality=1, right_quality=1
    )
    command = _step(policy, 0.05, observation=event)
    assert command.left_friction == 0.1
    for tick in range(2, 20):
        command = _step(policy, tick * 0.05)
    assert command.trace_fields()["adaptive_left_depth_prior_locked"]
    assert command.left_friction == 0.1
    if particle:
        assert policy._particle_filters[0].config.prior_mean == 0.1
        assert command.left_particle_friction.mean < 0.4


def test_saturated_depth_motion_confirms_constant_prior_without_bypassing_rate():
    """超过曲线饱和点的持续压缩可确认恒定候选，控制上调仍限速。"""
    policy = _policy()
    previous = 0.1
    for tick in range(12):
        command = _step(policy, tick * 0.05, depth=0.006 + tick * 0.00025)
        assert command.left_friction <= previous + 0.05 * 0.05 + 1e-12
        previous = command.left_friction
    assert command.trace_fields()["adaptive_left_depth_prior_locked"]
    assert command.trace_fields()["adaptive_left_depth_prior_value"] == 0.6
    assert 0.1 < command.left_friction < 0.13


def test_saturated_prior_still_requires_uninterrupted_eligible_contact():
    """饱和不绕过风险门控，中断后需重新累计完整确认窗口。"""
    policy = _policy()
    for tick in range(4):
        _step(policy, tick * 0.05, depth=0.006 + tick * 0.001)
    command = _step(policy, 0.2, depth=0.01, observation=_observation(risk=1))
    assert not command.trace_fields()["adaptive_left_depth_prior_locked"]
    for tick in range(5, 9):
        command = _step(policy, tick * 0.05, depth=0.006 + tick * 0.001)
        assert not command.trace_fields()["adaptive_left_depth_prior_locked"]
    command = _step(policy, 0.5, depth=0.012)
    assert command.trace_fields()["adaptive_left_depth_prior_locked"]


def test_depth_leaving_plateau_restarts_confirmation():
    """从饱和区回到曲线斜坡时，较低候选必须重新稳定确认。"""
    policy = _policy()
    for tick in range(4):
        _step(policy, tick * 0.05, depth=0.006)
    command = _step(policy, 0.2, depth=0.001)
    assert not command.trace_fields()["adaptive_left_depth_prior_locked"]
    for tick in range(5, 8):
        command = _step(policy, tick * 0.05, depth=0.001)
        assert not command.trace_fields()["adaptive_left_depth_prior_locked"]
    command = _step(policy, 0.45, depth=0.001)
    assert command.trace_fields()["adaptive_left_depth_prior_value"] == pytest.approx(
        DepthFrictionPriorConfig().friction_at(0.001)
    )


def test_unlimited_adoption_waits_for_confirmation_and_preserves_force_slew():
    """不限制摩擦上调时仍需接触确认，目标力下降继续受限。"""
    policy = _policy(rate=None)
    for tick in range(4):
        previous = _step(policy, tick * 0.05)
        assert previous.left_friction == 0.1
    policy.scheduler.target_force_n = 5.0
    command = _step(policy, 0.25)
    assert command.left_friction == 0.6
    assert command.right_friction == 0.6
    assert 4.9 - 1e-12 <= command.load.target_force_n < 5.0


def test_unlimited_adoption_does_not_bypass_risk_gate():
    """关闭速率限制不允许风险期间抬高先验。"""
    policy = _policy(rate=None)
    for tick in range(12):
        command = _step(policy, tick * 0.05, observation=_observation(risk=1))
        assert command.left_friction == 0.1
        assert not command.trace_fields()["adaptive_left_depth_prior_locked"]

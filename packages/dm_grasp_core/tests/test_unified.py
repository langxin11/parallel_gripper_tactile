"""受控候选注入只验证权限与调度，不证明真实局部滑移。"""

from dataclasses import replace

import numpy as np
import pytest

from dm_grasp_core.grasp.adaptive import AdaptiveLoadConfig
from dm_grasp_core.grasp.friction_particle import ParticleFrictionConfig
from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig, UnifiedAdaptivePolicy
from dm_grasp_core.tactile.risk import TaxelRiskObservation


class _Evidence:
    """给组合策略注入独立事件，避免把检测正确性与控制逻辑混为一谈。"""

    observation = TaxelRiskObservation(valid=True)

    def update(self, *_args, **_kwargs):
        """返回本次指定证据。"""
        return self.observation

    def reset(self):
        """测试证据由用例显式管理。"""


def test_event_budget_deduplication_and_monotonic_target() -> None:
    """零承载缺口时事件仍抬高目标，同一事件只消费一次，预算耗尽明确失败。"""
    config = UnifiedAdaptiveConfig(risk_enabled=True, risk_max_events=2, failure_timeout_s=0.05)
    policy = UnifiedAdaptivePolicy(config)
    evidence = _Evidence()
    policy.observer = evidence
    frame = np.tile([0, 0, 0.1], (9, 1))
    policy.update(frame, frame, time_s=0, measured_force_n=0.9)
    evidence.observation = TaxelRiskObservation(valid=True, risk=1, event_id=1)
    first = policy.update(frame, frame, time_s=0.01, measured_force_n=0.9)
    assert first.load.target_force_n > config.load.min_force_n
    assert first.increase_count == 1
    duplicate = policy.update(frame, frame, time_s=0.01, measured_force_n=0.9)
    assert duplicate.event_id == 0 and duplicate.load.target_force_rate_n_s == 0
    previous = first.load.target_force_n
    for index in range(2, 50):
        command = policy.update(frame, frame, time_s=index * 0.01, measured_force_n=0.9)
        assert previous <= command.load.target_force_n <= previous + 0.01000001
        assert command.increase_count == 1
        previous = command.load.target_force_n
    assert command.load.target_force_n == pytest.approx(0.65, abs=0.005)
    evidence.observation = replace(evidence.observation, event_id=2)
    policy.update(frame, frame, time_s=0.50, measured_force_n=0.9)
    final = policy.update(frame, frame, time_s=0.56, measured_force_n=0.9)
    assert final.risk_budget_exhausted and final.failure_reason == "risk_budget_exhausted"


def test_friction_only_mode_tracks_ratio_without_consuming_risk_budget() -> None:
    """摩擦专用风险事件更新系数，目标可随承载比值下降且不消费固定增力预算。"""
    load = AdaptiveLoadConfig(
        left_friction=0.5,
        right_friction=0.5,
        min_force_n=0.5,
        max_force_rate_n_s=10.0,
        filter_tau_s=0.01,
        allow_target_decrease=True,
    )
    config = UnifiedAdaptiveConfig(
        load=load,
        risk_enabled=True,
        risk_step_enabled=False,
        friction_update_enabled=True,
    )
    policy = UnifiedAdaptivePolicy(config)
    loaded = np.tile([0.1, 0, 0.2], (9, 1))
    event = TaxelRiskObservation(
        valid=True,
        risk=1,
        event_id=1,
        left_candidate=0.25,
        right_candidate=0.25,
        left_quality=1,
        right_quality=1,
    )
    policy.update(loaded, loaded, time_s=0, measured_force_n=0.5)
    raised = policy.update(
        loaded,
        loaded,
        time_s=0.1,
        measured_force_n=0.5,
        observation=event,
    )
    assert raised.left_friction == pytest.approx(0.2)
    assert raised.increase_count == 0
    assert not raised.risk_budget_exhausted
    unloaded = np.tile([0, 0, 0.2], (9, 1))
    lowered = policy.update(
        unloaded,
        unloaded,
        time_s=0.2,
        measured_force_n=raised.load.target_force_n,
        observation=TaxelRiskObservation(valid=True),
    )
    assert lowered.load.target_force_n < raised.load.target_force_n


def test_particle_posterior_controls_from_stick_history_and_event_candidate() -> None:
    """渐增粘着证据配合微滑移候选形成控制后验，并输出完整诊断。"""
    config = UnifiedAdaptiveConfig(
        risk_enabled=True,
        risk_step_enabled=False,
        friction_update_enabled=True,
        friction_discount=0.8,
        min_event_taxels=4,
        friction_expiry_s=0.05,
        particle_friction=ParticleFrictionConfig(seed=19),
    )
    policy = UnifiedAdaptivePolicy(config)
    frame = np.tile([0.0, 0.0, 0.1], (9, 1))
    stable = TaxelRiskObservation(
        valid=True,
        reason="observing",
        left_utilization=0.2,
        right_utilization=0.2,
    )
    policy.update(frame, frame, time_s=0.0, measured_force_n=0.5, observation=stable)
    for index in range(1, 51):
        policy.update(
            frame,
            frame,
            time_s=index * 0.004,
            measured_force_n=0.5,
            observation=stable,
        )
    event = replace(
        stable,
        risk=1,
        event_id=1,
        left_utilization=0.255,
        left_candidate=0.255,
        left_quality=0.9,
    )

    result = policy.update(
        frame,
        frame,
        time_s=0.204,
        measured_force_n=0.5,
        observation=event,
    )

    assert 0.25 < result.left_friction < 0.35
    assert result.left_update_reason.startswith("particle_event_update")
    assert result.left_particle_friction is not None
    trace = result.trace_fields()
    assert trace["adaptive_left_particle_mu_control"] == pytest.approx(
        result.left_particle_friction.control_value
    )
    assert "adaptive_right_particle_mu_ess" in trace

    expired = policy.update(
        frame,
        frame,
        time_s=0.3,
        measured_force_n=0.5,
        observation=stable,
    )
    assert expired.left_friction == config.load.left_friction
    assert expired.left_particle_friction is not None
    assert expired.left_particle_friction.reason == "reset"


def test_asymmetric_friction_quality_and_expiry() -> None:
    """低质量不覆盖已接受低值；提高需独立证据，过期与拓扑变化回退。"""
    policy = UnifiedAdaptivePolicy(
        UnifiedAdaptiveConfig(risk_enabled=True, friction_update_enabled=True)
    )
    evidence = _Evidence()
    policy.observer = evidence
    frame = np.tile([0, 0, 0.1], (9, 1))
    policy.update(frame, frame, time_s=0, measured_force_n=0.9)
    for event in range(1, 4):
        evidence.observation = TaxelRiskObservation(
            valid=True,
            event_id=event,
            left_candidate=0.5,
            right_candidate=1.0,
            left_quality=1,
            right_quality=1,
        )
        result = policy.update(frame, frame, time_s=event * 0.01, measured_force_n=0.9)
        assert result.left_friction == pytest.approx(0.4)
        assert result.right_friction == pytest.approx(0.8 if event == 3 else 0.6)
    evidence.observation = replace(evidence.observation, event_id=4, left_quality=0.1)
    result = policy.update(frame, frame, time_s=0.04, measured_force_n=0.9)
    assert result.left_friction == pytest.approx(0.4)
    assert result.left_update_reason == "insufficient_quality"
    evidence.observation = TaxelRiskObservation(valid=True)
    for index in range(5, 510):
        result = policy.update(frame, frame, time_s=index * 0.01, measured_force_n=0.9)
    assert result.right_friction == 0.6
    evidence.observation = TaxelRiskObservation(valid=True, contact_changed=True)
    result = policy.update(frame, frame, time_s=5.1, measured_force_n=0.9)
    assert result.right_update_reason == "contact_reset"


def test_accepted_friction_can_persist_until_contact_changes() -> None:
    """关闭时间过期后，已接受低值保持到接触拓扑明确改变。"""
    config = UnifiedAdaptiveConfig(
        risk_enabled=True,
        friction_update_enabled=True,
        friction_expiry_s=None,
    )
    policy = UnifiedAdaptivePolicy(config)
    frame = np.tile([0, 0, 0.1], (9, 1))
    policy.update(frame, frame, time_s=0, measured_force_n=0.5)
    accepted = policy.update(
        frame,
        frame,
        time_s=0.01,
        measured_force_n=0.5,
        observation=TaxelRiskObservation(
            valid=True,
            event_id=1,
            left_candidate=0.25,
            left_quality=1,
        ),
    )
    assert accepted.left_friction == pytest.approx(0.2)
    for index in range(1, 122):
        persistent = policy.update(
            frame,
            frame,
            time_s=0.01 + 0.05 * index,
            measured_force_n=0.5,
            observation=TaxelRiskObservation(valid=True),
        )
    assert persistent.left_friction == pytest.approx(0.2)
    reset = policy.update(
        frame,
        frame,
        time_s=6.07,
        measured_force_n=0.5,
        observation=TaxelRiskObservation(valid=True, contact_changed=True),
    )
    assert reset.left_friction == config.load.left_friction
    assert reset.left_update_reason == "contact_reset"


def test_candidate_below_observed_no_slip_lower_bound_is_rejected() -> None:
    """稳定无风险利用率形成下界，后续更低的整侧候选不得接管控制。"""
    config = UnifiedAdaptiveConfig(
        risk_enabled=True,
        friction_update_enabled=True,
        friction_discount=1.0,
        min_event_taxels=4,
    )
    policy = UnifiedAdaptivePolicy(config)
    frame = np.tile([0, 0, 0.1], (9, 1))
    stable = TaxelRiskObservation(
        valid=True,
        reason="observing",
        left_utilization=0.29,
        right_utilization=0.28,
    )
    policy.update(frame, frame, time_s=0, measured_force_n=0.5, observation=stable)
    baseline = policy.update(frame, frame, time_s=0.01, measured_force_n=0.5, observation=stable)
    assert baseline.left_lower_bound == pytest.approx(0.29)
    assert baseline.right_lower_bound == pytest.approx(0.28)
    event = replace(
        stable,
        risk=1,
        event_id=1,
        left_candidate=0.064,
        right_candidate=0.127,
        left_quality=4 / 9,
        right_quality=4 / 9,
    )
    rejected = policy.update(frame, frame, time_s=0.02, measured_force_n=0.5, observation=event)
    assert rejected.left_friction == config.load.left_friction
    assert rejected.right_friction == config.load.right_friction
    assert rejected.left_update_reason == "below_observed_lower_bound"
    assert rejected.right_update_reason == "below_observed_lower_bound"


def test_supported_side_lower_bound_raises_adopted_friction() -> None:
    """稳定低风险的整侧下界可提高采用值，局部高比值仍只作诊断。"""
    load = AdaptiveLoadConfig(left_friction=0.3, right_friction=0.3, safety_factor=1.2)
    policy = UnifiedAdaptivePolicy(
        UnifiedAdaptiveConfig(load=load, risk_enabled=True, friction_update_enabled=True)
    )
    frame = np.tile([0, 0, 0.1], (9, 1))
    stable = TaxelRiskObservation(
        valid=True,
        reason="observing",
        risk=0,
        left_utilization=0.44,
        right_utilization=0.33,
        left_taxel_utilization=(1.5,) + (0.0,) * 8,
        right_taxel_utilization=(1.2,) + (0.0,) * 8,
        left_valid_mask=(True,) + (False,) * 8,
        right_valid_mask=(True,) + (False,) * 8,
    )

    disabled = policy.update(
        frame, frame, time_s=0.0, measured_force_n=0.5, observation=stable, enabled=False
    )
    assert disabled.left_friction == disabled.right_friction == pytest.approx(0.3)

    result = policy.update(frame, frame, time_s=0.01, measured_force_n=0.5, observation=stable)
    assert result.left_friction == pytest.approx(0.44)
    assert result.right_friction == pytest.approx(0.33)
    assert result.left_update_reason == "lower_bound_accepted"
    assert result.right_update_reason == "lower_bound_accepted"
    assert result.left_taxel_lower_bound == pytest.approx(1.5)
    assert result.right_taxel_lower_bound == pytest.approx(1.2)

    lower = replace(stable, left_utilization=0.35, right_utilization=0.31)
    held = policy.update(frame, frame, time_s=0.02, measured_force_n=0.5, observation=lower)
    assert held.left_friction == pytest.approx(0.44)
    assert held.right_friction == pytest.approx(0.33)
    assert held.left_update_reason == held.right_update_reason == "unchanged"


@pytest.mark.parametrize("failure", ["tracking_limited", "capacity_limited", "sample_gap"])
def test_execution_and_failure_boundaries(failure: str) -> None:
    """执行受限时停止积累；持续能力不足及观测失鲜不能报告成功。"""
    config = UnifiedAdaptiveConfig(failure_timeout_s=0.05, tracking_error_n=0.1)
    policy = UnifiedAdaptivePolicy(config)
    frame = np.tile([0.1 if failure == "capacity_limited" else 0, 0, 0.1], (9, 1))
    if failure == "capacity_limited":
        policy = UnifiedAdaptivePolicy(replace(config, load=replace(config.load, max_force_n=0.6)))
    initial = policy.update(frame, frame, time_s=0, measured_force_n=0.0)
    for index in range(1, 10):
        command = policy.update(
            frame,
            frame,
            time_s=index * (0.2 if failure == "sample_gap" else 0.01),
            measured_force_n=0.0,
            execution_limited=failure == "tracking_limited",
        )
    assert command.failure_reason == failure
    if failure == "tracking_limited":
        assert command.load.target_force_n == initial.load.target_force_n
    with pytest.raises(ValueError):
        policy.update(frame, frame, time_s=-1, measured_force_n=0)
    for changes in (
        {"risk_enabled": 1},
        {"friction_update_enabled": True},
        {"failure_timeout_s": "bad"},
    ):
        with pytest.raises(ValueError):
            replace(config, **changes)


@pytest.mark.parametrize("confirmed", [True, False])
def test_fast_persistent_risk_steps_and_freeze(confirmed: bool) -> None:
    """持续风险可续增大步；冻结和执行限幅不积攒步数，持续事件不重复更新摩擦。"""
    config = UnifiedAdaptiveConfig(
        risk_enabled=True,
        friction_update_enabled=True,
        risk_step_n=1,
        risk_rate_n_s=50,
        risk_repeat_interval_s=0.1,
        risk_budget_n=4,
        risk_max_events=4,
    )
    config = replace(config, load=replace(config.load, max_force_rate_n_s=50))
    policy = UnifiedAdaptivePolicy(config)
    frame = np.tile([0, 0, 0.1], (9, 1))
    observation = TaxelRiskObservation(
        valid=True,
        risk=1,
        event_id=1,
        left_candidate=0.5,
        left_quality=1,
        confirmed_risk=confirmed,
    )
    policy.update(frame, frame, time_s=0, measured_force_n=0.5)
    commands = []
    for index in range(1, 81):
        command = policy.update(
            frame,
            frame,
            time_s=index * 0.004,
            measured_force_n=policy.scheduler.target_force_n,
            observation=observation,
        )
        commands.append(command)
    assert commands[4].load.target_force_n == pytest.approx(1.5)
    expected_count = 4 if confirmed else 1
    assert commands[-1].increase_count == expected_count
    assert commands[-1].left_friction == pytest.approx(0.4)
    assert sum(c.left_update_reason == "lower_accepted" for c in commands) == 1
    assert all(0 <= c.load.target_force_rate_n_s <= 50.000001 for c in commands)
    target = commands[-1].load.target_force_n
    for index in range(81, 90):
        command = policy.update(
            frame,
            frame,
            time_s=index * 0.004,
            measured_force_n=0,
            observation=replace(observation, event_id=index),
            execution_limited=True,
        )
        assert command.increase_count == expected_count
        assert command.load.target_force_n == target
    for value in (0, -1, True, float("nan")):
        with pytest.raises(ValueError):
            replace(config, risk_repeat_interval_s=value)


def test_local_lower_bounds_are_diagnostic_and_reset_each_lost_contact() -> None:
    """局部与整侧比值共享事件，异质触点的高局部下界不否决整侧候选。"""
    policy = UnifiedAdaptivePolicy(
        UnifiedAdaptiveConfig(risk_enabled=True, friction_update_enabled=True)
    )
    frame = np.tile([0, 0, 0.1], (9, 1))
    stable = TaxelRiskObservation(
        valid=True,
        reason="observing",
        left_utilization=0.1,
        right_utilization=0.2,
        left_taxel_utilization=(0.9, 0.4) + (0.0,) * 7,
        right_taxel_utilization=(0.6, 0.3) + (0.0,) * 7,
        left_valid_mask=(True, True) + (False,) * 7,
        right_valid_mask=(True, True) + (False,) * 7,
    )
    result = policy.update(frame, frame, time_s=0, measured_force_n=0.5, observation=stable)
    assert result.left_taxel_lower_bound == pytest.approx(0.9)
    assert result.right_taxel_lower_bound == pytest.approx(0.6)
    assert result.left_friction == policy.config.load.left_friction
    assert result.event_id == 0 and result.increase_count == 0
    assert result.trace_fields()["adaptive_left_taxel_mu_lower_bound"] == pytest.approx(0.9)
    event = replace(
        stable,
        risk=1,
        event_id=1,
        left_candidate=0.4,
        left_quality=1,
        left_taxel_utilization=(1.5, 0.4) + (0.0,) * 7,
    )
    result = policy.update(frame, frame, time_s=0.01, measured_force_n=0.5, observation=event)
    assert result.left_friction == pytest.approx(0.32)
    assert result.left_taxel_lower_bound == pytest.approx(0.9)
    assert result.left_update_reason == "lower_accepted"
    lost = replace(stable, left_valid_mask=(False, True) + (False,) * 7)
    result = policy.update(frame, frame, time_s=0.02, measured_force_n=0.5, observation=lost)
    assert result.left_taxel_lower_bound == pytest.approx(0.4)
    reentered = replace(stable, left_taxel_utilization=(0.2, 0.3) + (0.0,) * 7)
    result = policy.update(frame, frame, time_s=0.03, measured_force_n=0.5, observation=reentered)
    assert result.left_taxel_lower_bound == pytest.approx(0.4)
    invalid = replace(stable, valid=False)
    result = policy.update(frame, frame, time_s=0.04, measured_force_n=0.5, observation=invalid)
    assert result.left_taxel_lower_bound == 0
    assert result.right_taxel_lower_bound == 0

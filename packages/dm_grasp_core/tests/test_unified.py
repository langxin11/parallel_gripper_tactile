"""受控候选注入只验证权限与调度，不证明真实局部滑移。"""

from dataclasses import replace

import numpy as np
import pytest

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

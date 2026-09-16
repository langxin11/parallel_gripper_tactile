"""验证多速率预处理与调度边界，不以合成数据证明物理风险识别有效。"""

from dataclasses import FrozenInstanceError, replace
import math

import numpy as np
import pytest

from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig, UnifiedAdaptivePolicy
from dm_grasp_core.tactile.multirate import (
    LatestTactileBuffer,
    TactilePreprocessor,
    TactileSamplingConfig,
)
from dm_grasp_core.tactile.risk import TaxelRiskObservation


def test_latched_event_survives_decimation_but_not_invalid_or_expiry(monkeypatch) -> None:
    """采样间事件送达一次；重复、过期、接触变化和跳过坏帧都不能重放证据。"""
    processor = TactilePreprocessor(TactileSamplingConfig(), load_tau_s=0.01)
    policy = UnifiedAdaptivePolicy(
        UnifiedAdaptiveConfig(
            risk_enabled=True,
            friction_update_enabled=True,
        )
    )
    frame = np.tile([0, 0, 0.1], (9, 1))
    observation = TaxelRiskObservation(valid=True)
    monkeypatch.setattr(processor.observer, "update", lambda *a, **k: observation)

    def sample(index):
        return processor.update(frame, frame, sample_time_s=index * 0.002, sequence_id=index)

    def control(state, time):
        return policy.update_tactile_state(
            state,
            control_time_s=time,
            stale_after_s=0.01,
            measured_force_n=0.5,
        )

    control(sample(0), 0)
    observation = TaxelRiskObservation(
        valid=True,
        event_id=1,
        risk=1,
        left_candidate=0.5,
        left_quality=1,
    )
    sample(1)
    observation = TaxelRiskObservation(valid=True)
    state = sample(2)
    command = control(state, 0.004)
    assert command.event_id == 1 and command.increase_count == 1
    assert command.left_friction == pytest.approx(0.4)
    repeated = control(state, 0.006)
    assert repeated.event_id == 0 and repeated.load.target_force_rate_n_s == 0
    again = control(sample(4), 0.008)
    assert again.event_id == 0 and again.increase_count == 1
    assert sample(7).latest_event is None
    observation = replace(observation, event_id=2, risk=1)
    sample(8)
    observation = TaxelRiskObservation(valid=True, contact_changed=True)
    assert sample(9).latest_event is None
    observation = replace(observation, contact_changed=False, event_id=3)
    sample(10)
    bad = frame.copy()
    bad[0, 0] = math.nan
    processor.update(bad, frame, sample_time_s=0.022, sequence_id=11)
    observation = TaxelRiskObservation(valid=True)
    recovered = sample(12)
    assert recovered.latest_event is None
    command = control(recovered, 0.024)
    assert command.load.target_force_rate_n_s == 0 and command.increase_count == 1


@pytest.mark.parametrize("axis, spike", [(0, 0.5), (0, math.nan), (1, 4.0), (2, 15.0)])
def test_median_rejects_spike_without_hiding_invalid_input(axis: int, spike: float) -> None:
    """量程内单帧毛刺被抑制，持续阶跃通过，坏原始帧直接标记无效。"""
    processor = TactilePreprocessor(TactileSamplingConfig(), load_tau_s=0.01)
    base = np.tile([0.0, 0.0, 0.1], (9, 1))
    processor.update(base, base, sample_time_s=0.0, sequence_id=0)
    noisy = base.copy()
    noisy[0, axis] = spike
    state = processor.update(noisy, base, sample_time_s=0.002, sequence_id=1)
    if axis != 0 or not math.isfinite(spike):
        assert not state.valid
        assert not state.observation.valid
        assert state.observation.reason == "invalid_sample"
        recovered = processor.update(base, base, sample_time_s=0.004, sequence_id=2)
        assert recovered.valid and recovered.invalid_samples == 1
        policy = UnifiedAdaptivePolicy(UnifiedAdaptiveConfig())
        command = policy.update_tactile_state(
            recovered, control_time_s=0.004, stale_after_s=0.01, measured_force_n=0.9
        )
        assert command.observation_reason == "invalid_sample"
        assert command.load.target_force_rate_n_s == 0
        return
    assert state.valid and state.load.left_n == 0.0
    assert state.left_taxels[0][0] == 0.0
    processor.update(base, base, sample_time_s=0.004, sequence_id=2)
    processor.update(base, base, sample_time_s=0.006, sequence_id=3)
    first = processor.update(noisy, base, sample_time_s=0.008, sequence_id=4)
    second = processor.update(noisy, base, sample_time_s=0.010, sequence_id=5)
    assert first.load.left_n == 0.0
    assert second.left_taxels[0][0] == spike
    assert second.load.left_n > first.load.left_n


def test_two_samples_per_control_and_stale_recovery_without_catchup() -> None:
    """采样两次才调度一次；重复或陈旧状态冻结，恢复只增长一个控制周期。"""
    processor = TactilePreprocessor(TactileSamplingConfig(), load_tau_s=0.01)
    policy = UnifiedAdaptivePolicy(UnifiedAdaptiveConfig())
    frame = np.tile([0.1, 0.0, 0.1], (9, 1))
    states = [
        processor.update(frame, frame, sample_time_s=index * 0.002, sequence_id=index)
        for index in range(3)
    ]
    initial = policy.update_tactile_state(
        states[0], control_time_s=0.0, stale_after_s=0.01, measured_force_n=0.9
    )
    active = policy.update_tactile_state(
        states[2], control_time_s=0.004, stale_after_s=0.01, measured_force_n=0.9
    )
    assert states[2].sequence_id - states[0].sequence_id == 2
    assert active.load.measured_tangential_force_n == pytest.approx(
        states[2].load.left_n + states[2].load.right_n
    )
    assert active.load.target_force_n - initial.load.target_force_n == pytest.approx(0.004)
    for time_s in (0.008, 0.012, 0.016):
        frozen = policy.update_tactile_state(
            states[2], control_time_s=time_s, stale_after_s=0.01, measured_force_n=0.9
        )
        assert frozen.load.target_force_n == active.load.target_force_n
        assert frozen.load.target_force_rate_n_s == 0.0
    assert frozen.observation_reason == "stale_sample"
    recovered = processor.update(frame, frame, sample_time_s=0.02, sequence_id=10)
    result = policy.update_tactile_state(
        recovered, control_time_s=0.02, stale_after_s=0.01, measured_force_n=0.9
    )
    assert result.failure_reason is None
    assert result.load.target_force_n - active.load.target_force_n == pytest.approx(0.004)
    with pytest.raises(ValueError, match="回退"):
        policy.update_tactile_state(
            states[2], control_time_s=0.024, stale_after_s=0.01, measured_force_n=0.9
        )
    for index in range(6, 140):
        frozen = policy.update_tactile_state(
            recovered, control_time_s=index * 0.004, stale_after_s=0.01, measured_force_n=0.9
        )
    assert frozen.failure_reason == "stale_tactile"
    assert frozen.load.target_force_n == result.load.target_force_n


def test_buffer_immutability_actual_sample_interval_and_dropped_count() -> None:
    """快照不随输入修改，时间与序号回退拒绝，滤波按真实间隔且计数丢帧。"""
    processor = TactilePreprocessor(TactileSamplingConfig(median_window=1), load_tau_s=0.01)
    buffer = LatestTactileBuffer()
    assert buffer.latest() is None
    frame = np.tile([0.0, 0.0, 0.1], (9, 1))
    first = processor.update(frame, frame, sample_time_s=0.0, sequence_id=0)
    buffer.publish(first)
    frame[:, 0] = 0.1
    second = processor.update(frame, frame, sample_time_s=0.005, sequence_id=3)
    expected = 0.9 * (1 - math.exp(-0.005 / 0.01))
    assert second.load.left_n == pytest.approx(expected)
    assert second.load.rate_n_s == pytest.approx(2 * expected / 0.005)
    assert second.dropped_samples == 2
    buffer.publish(second)
    assert buffer.latest() is second
    frame[:] = 0.0
    assert first.left_taxels[0] == (0.0, 0.0, 0.1)
    assert second.left_taxels[0] == (0.1, 0.0, 0.1)
    with pytest.raises(FrozenInstanceError):
        second.sequence_id = 7
    for invalid in (second, first, replace(second, sequence_id=4, sample_time_s=0.004)):
        with pytest.raises(ValueError, match="递增"):
            buffer.publish(invalid)
    for sequence, time_s in ((3, 0.006), (4, 0.004)):
        with pytest.raises(ValueError, match="递增"):
            processor.update(frame, frame, sample_time_s=time_s, sequence_id=sequence)


@pytest.mark.parametrize(
    "config",
    [
        replace(UnifiedAdaptiveConfig(), risk_enabled=True),
        replace(UnifiedAdaptiveConfig(), risk_enabled=True, friction_update_enabled=True),
    ],
)
def test_update_tactile_state_accepts_risk_and_friction_permissions(
    config: UnifiedAdaptiveConfig,
) -> None:
    """多速率可开放闭环，但无接触时仍不得增力。"""
    policy = UnifiedAdaptivePolicy(config)
    state = TactilePreprocessor(TactileSamplingConfig(), load_tau_s=0.01).update(
        np.zeros((9, 3)), np.zeros((9, 3)), sample_time_s=0.0, sequence_id=0
    )

    command = policy.update_tactile_state(
        state, control_time_s=0.0, stale_after_s=0.01, measured_force_n=0.9
    )
    assert command.load.target_force_n == config.load.min_force_n


def test_update_tactile_state_rejects_non_positive_stale_threshold() -> None:
    """新鲜度阈值必须为正有限数。"""
    policy = UnifiedAdaptivePolicy(UnifiedAdaptiveConfig())
    state = TactilePreprocessor(TactileSamplingConfig(), load_tau_s=0.01).update(
        np.zeros((9, 3)), np.zeros((9, 3)), sample_time_s=0.0, sequence_id=0
    )

    with pytest.raises(ValueError, match="新鲜度阈值"):
        policy.update_tactile_state(
            state, control_time_s=0.0, stale_after_s=0.0, measured_force_n=0.9
        )

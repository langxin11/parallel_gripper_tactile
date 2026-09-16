"""真机多速率适配的纯离线时序与原始安全边界。"""

from dataclasses import FrozenInstanceError, replace
import math
from pathlib import Path

import pytest
from dmgripper_hardware import MotorFeedback, STATUS_ENABLED
from dm_grasp_core.tactile.risk import TaxelRiskObservation

from dmgripper_experiments.config import load_experiment_config
from dmgripper_experiments.observation import PairedObservation
from dmgripper_experiments.tactile import HardwareTactilePreprocessor, TactileSnapshot
from dmgripper_experiments.targets import UnifiedAdaptiveTargetSource


def _config():
    return load_experiment_config(
        Path(__file__).resolve().parents[3]
        / "configs/hardware/dmgripper/unified_adaptive_fast.yaml"
    )


def _snapshot(index: int, *, shear: float = 0.0) -> TactileSnapshot:
    taxels = ((0.0, shear / 9, 1.0 / 9),) * 9
    return TactileSnapshot(
        received_at_s=1000.0 + index * 0.002,
        packet_counter=index,
        timestamp_us=7_000_000 + index * 2000,
        left_force_n=1.0,
        right_force_n=1.0,
        raw_left_fz_n=1.0,
        raw_right_fz_n=1.0,
        raw_left_fx_n=0.0,
        raw_left_fy_n=shear,
        raw_right_fx_n=0.0,
        raw_right_fy_n=shear,
        left_taxel_forces_n=taxels,
        right_taxel_forces_n=taxels,
    )


def test_experimental_permissions_are_explicit_and_do_not_claim_validation():
    """实验权限与验收事实分离，关闭显式授权后原门禁仍生效。"""
    adaptive = _config().reference.adaptive
    assert adaptive.experimental_closed_loop
    assert not adaptive.risk_validation_passed and not adaptive.friction_validation_passed
    assert adaptive.unified.risk_enabled and adaptive.unified.friction_update_enabled
    assert adaptive.unified.risk_step_n == 1.0
    assert adaptive.unified.risk_repeat_interval_s == 0.1
    with pytest.raises(ValueError, match="risk_validation_passed"):
        replace(adaptive, experimental_closed_loop=False)
    with pytest.raises(ValueError, match="布尔"):
        replace(adaptive, experimental_closed_loop=1)
    with pytest.raises(ValueError, match="500 Hz"):
        replace(adaptive, tactile_sampling=replace(adaptive.tactile_sampling, period_s=0.004))


def test_preprocessing_uses_device_clock_and_preserves_immutable_same_packet():
    """中间采样参与中值，主机接收抖动不改变滤波，计数回绕不倒退序号。"""
    config = _config()
    assert config.reference.adaptive.initial_force_n == 1.0
    assert config.safety.max_target_force_n == 30.0
    assert config.reference.adaptive.max_force_rate_n_s == 50.0
    assert config.controller.admittance.prevent_unloading is True
    processors = (HardwareTactilePreprocessor(config), HardwareTactilePreprocessor(config))
    outputs = []
    for index, (counter, gap) in enumerate(((65535, 0), (0, 0), (3, 2))):
        raw = replace(
            _snapshot(index, shear=0.0 if index == 0 else 1.0),
            packet_counter=counter,
            counter_gap=gap,
            counter_event="wrap" if index == 1 else "gap" if gap else "first",
        )
        output = processors[0](raw)
        jittered = processors[1](replace(raw, received_at_s=20_000.0 + index * 0.05))
        assert output.processed == jittered.processed
        assert output.left_taxel_forces_n == raw.left_taxel_forces_n
        assert output.timestamp_us == raw.timestamp_us
        assert output.received_at_s == raw.received_at_s
        outputs.append(output)
    assert [item.processed.sequence_id for item in outputs] == [0, 1, 4]
    assert outputs[-1].processed.dropped_samples == 2
    assert outputs[1].processed.load.left_n == 0.0
    assert outputs[-1].processed.load.left_n == pytest.approx(-math.expm1(-0.002 / 0.01))
    assert outputs[-1].processed.sample_time_s == pytest.approx(7.004)
    with pytest.raises(FrozenInstanceError):
        outputs[-1].received_at_s = 0.0
    with pytest.raises(FrozenInstanceError):
        outputs[-1].processed.valid = False
    with pytest.raises(TypeError):
        outputs[-1].processed.left_taxels[0][0] = 1.0


@pytest.mark.parametrize("fault", ["taxel_range", "global_overforce", "nonfinite"])
def test_intermediate_raw_fault_cannot_be_hidden_by_median_or_control_decimation(fault):
    """两个控制 tick 之间的单帧异常也立即拒绝，不等待滤波后或控制采样。"""
    config = _config()
    processor = HardwareTactilePreprocessor(config)
    processor(_snapshot(0))
    raw = _snapshot(1)
    if fault == "taxel_range":
        limit = config.reference.adaptive.unified.observer.tangential_range_n
        raw = replace(raw, left_taxel_forces_n=((limit, 0.0, 1.0 / 9),) * 9)
    elif fault == "global_overforce":
        raw = replace(raw, raw_right_fz_n=config.safety.force_ceiling_n + 0.1)
    else:
        raw = replace(raw, raw_left_fz_n=math.nan)
    with pytest.raises(ValueError):
        processor(raw)


def test_target_uses_host_age_freezes_repeated_stale_and_does_not_catch_up():
    """设备时间与主机时间相差近千秒仍可调度，恢复只使用最后一个控制间隔。"""
    config = _config()
    processor = HardwareTactilePreprocessor(config)
    source = UnifiedAdaptiveTargetSource(config.reference.adaptive)

    def observe(snapshot, age=0.0):
        paired = PairedObservation(
            snapshot=snapshot,
            feedback=MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED),
            is_new_tactile=age == 0.0,
            tactile_age_s=age,
            tangential_force_n=0.0,
            signed_tangential_force_n=0.0,
            closure_m=0.0,
        )
        source.observe(paired, 0.004)
        return source.active_reference(0.0).force_n

    assert observe(processor(_snapshot(0))) == 1.0
    source.activate()
    processor(_snapshot(1, shear=8.0))
    latest = processor(_snapshot(2, shear=8.0))
    grown = observe(latest)
    assert 1.0 < grown <= 1.2 + 1e-9
    diagnostics = source.trace_fields()
    assert diagnostics["sensor_device_time_s"] == pytest.approx(7.004)
    assert diagnostics["sensor_received_at_s"] == pytest.approx(1000.004)
    assert diagnostics["sensor_age_s"] == 0.0
    for tick in range(1, 7):
        assert observe(latest, tick * 0.004) == grown
    assert source.trace_fields()["sensor_stale"] is True
    resumed = observe(processor(_snapshot(16, shear=8.0)))
    assert 0.0 < resumed - grown <= 50.0 * 0.004 + 1e-9
    assert source.failure_reason is None


@pytest.mark.parametrize("event_age_s,expected", [(0.002, 1), (0.02, 0)])
def test_latched_event_time_is_mapped_with_sample(event_age_s, expected):
    """主机与设备时基分离时，短期事件正常交付，旧事件不能因新包而变新鲜。"""
    config = _config()
    processor = HardwareTactilePreprocessor(config)
    source = UnifiedAdaptiveTargetSource(config.reference.adaptive)

    def observe(snapshot):
        source.observe(
            PairedObservation(
                snapshot=snapshot,
                feedback=MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED),
                is_new_tactile=True,
                tactile_age_s=0.0,
                tangential_force_n=0.0,
                signed_tangential_force_n=0.0,
                closure_m=0.0,
            ),
            0.004,
        )

    observe(processor(_snapshot(0)))
    source.activate()
    snapshot = processor(_snapshot(2))
    evidence = TaxelRiskObservation(
        valid=True, event_id=1, risk=1, left_candidate=0.5, left_quality=1
    )
    snapshot = replace(
        snapshot,
        processed=replace(
            snapshot.processed, latest_event=evidence, event_time_s=7.004 - event_age_s
        ),
    )
    observe(snapshot)
    assert source.trace_fields()["adaptive_increase_count"] == expected
    assert source.trace_fields()["adaptive_left_mu"] == pytest.approx(0.4 if expected else 0.6)

"""人工整侧起滑标记与离线候选的最小行为验证。"""

from dataclasses import replace

import pytest

from dmgripper_experiments.calibrate_friction import calibrate
from dmgripper_experiments.config import AdaptiveReferenceConfig, ExperimentConfig
from dmgripper_experiments.recording import ExperimentRecorder


def test_marked_side_only_and_reaction_time_adjustment(tmp_path) -> None:
    """仅已标记侧产候选，修正时间改变窗口并保留来源。"""
    config = ExperimentConfig(
        stage="friction", reference=AdaptiveReferenceConfig(preload_source="preload/run-1")
    )
    with ExperimentRecorder(
        tmp_path / "run", config, started_monotonic_s=0.0, epoch_ns=1
    ) as recorder:
        for i in range(1, 6):
            t = i * 0.05
            recorder.write(
                {
                    "time_s": t,
                    "phase": "active",
                    "contact_segment": 1,
                    "tactile_timestamp_us": i * 50000,
                }
            )
            recorder.sample(
                {
                    "received_at_s": t,
                    "timestamp_us": i * 50000,
                    "raw_left_fx_n": 0.1 * i,
                    "raw_left_fy_n": 0.0,
                    "raw_left_fz_n": 1.0,
                    "raw_right_fx_n": 0.8,
                    "raw_right_fy_n": 0.0,
                    "raw_right_fz_n": 1.0,
                }
            )
        recorder.append(
            {
                "time_s": 0.25,
                "event": "manual_slip_mark",
                "side": "left",
                "contact_segment": 1,
                "tactile_timestamp_us": 250000,
            }
        )
    candidate = calibrate(tmp_path / "run", window_s=0.1)
    assert candidate["left"]["candidate_t_over_n"] == pytest.approx(0.4)
    assert candidate["right"] is None
    assert candidate["preload_source"] == "preload/run-1"
    adjusted = calibrate(tmp_path / "run", window_s=0.1, slip_time_s=0.15, side="left")
    assert adjusted["markers"][0]["time_adjusted"] is True
    assert adjusted["left"]["candidate_t_over_n"] == pytest.approx(0.15)


def test_adaptive_config_cannot_use_missing_side(tmp_path) -> None:
    """单侧候选不得自动复制成双侧冻结先验。"""
    base = AdaptiveReferenceConfig(preload_source="preload/run-1")
    with pytest.raises(ValueError):
        replace(ExperimentConfig(), stage="adaptive", reference=base)


def test_adjusted_time_outside_active_is_rejected(tmp_path) -> None:
    """释放后的时间不能吸附到旧 active 样本生成候选。"""
    config = ExperimentConfig(
        stage="friction", reference=AdaptiveReferenceConfig(preload_source="preload/run-1")
    )
    with ExperimentRecorder(
        tmp_path / "run", config, started_monotonic_s=0.0, epoch_ns=1
    ) as recorder:
        recorder.write(
            {"time_s": 0.1, "phase": "active", "contact_segment": 1, "tactile_timestamp_us": 100000}
        )
        recorder.write(
            {
                "time_s": 0.2,
                "phase": "returning",
                "contact_segment": 1,
                "tactile_timestamp_us": 200000,
            }
        )
        recorder.sample(
            {
                "received_at_s": 0.1,
                "timestamp_us": 100000,
                "raw_left_fx_n": 0.2,
                "raw_left_fy_n": 0.0,
                "raw_left_fz_n": 1.0,
            }
        )
    with pytest.raises(ValueError, match="不在 active"):
        calibrate(tmp_path / "run", slip_time_s=0.25, side="left")


def test_loss_of_contact_inside_window_is_rejected(tmp_path) -> None:
    """窗口中失接触不会被静默过滤成偏大的 T/N。"""
    config = ExperimentConfig(
        stage="friction", reference=AdaptiveReferenceConfig(preload_source="preload/run-1")
    )
    with ExperimentRecorder(
        tmp_path / "run", config, started_monotonic_s=0.0, epoch_ns=1
    ) as recorder:
        for i, fz in ((1, 1.0), (2, 0.05), (3, 1.0)):
            recorder.write(
                {
                    "time_s": i * 0.05,
                    "phase": "active",
                    "contact_segment": 1,
                    "tactile_timestamp_us": i * 50000,
                }
            )
            recorder.sample(
                {
                    "received_at_s": i * 0.05,
                    "timestamp_us": i * 50000,
                    "raw_left_fx_n": 0.2,
                    "raw_left_fy_n": 0.0,
                    "raw_left_fz_n": fz,
                }
            )
        recorder.append(
            {
                "time_s": 0.15,
                "event": "manual_slip_mark",
                "side": "left",
                "contact_segment": 1,
                "tactile_timestamp_us": 150000,
            }
        )
    with pytest.raises(ValueError, match="失接触"):
        calibrate(tmp_path / "run", window_s=0.2)

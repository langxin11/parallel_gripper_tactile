"""验证真机分析脚本直接读取 MCAP 的原类型记录。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dmgripper_experiments.recording import _trace_fields_schema
from papillarray_hardware.recording import RecordingWriter
from scripts.analysis.export_curve_video import CurveVideo
from scripts.analysis.replay_pillar_friction import (
    iter_snapshots,
    online_friction,
    phase_ranges,
)


def test_analysis_consumers_read_recording_mcap(tmp_path: Path) -> None:
    """同一份 MCAP 供视频曲线与摩擦诊断消费，保持空值和布尔语义。"""
    trace = {
        "time_s": 1.0,
        "phase": "active",
        "tactile_timestamp_us": 1_000_000,
        "measured_force_n": 2.0,
        "target_force_n": 3.0,
        "left_fz_n": 1.0,
        "right_fz_n": 1.0,
        "raw_left_fx_n": 0.1,
        "raw_left_fy_n": 0.2,
        "raw_right_fx_n": 0.2,
        "raw_right_fy_n": 0.1,
        "adaptive_left_mu": 0.4,
        "adaptive_right_mu": 0.5,
        "adaptive_left_mu_lower_bound": 0.3,
        "adaptive_right_mu_lower_bound": 0.4,
        "adaptive_left_candidate": 0.42,
        "adaptive_right_candidate": 0.52,
        "stiffness_n_per_m": 100.0,
        "position_rad": 0.1,
        "q_des_rad": 0.2,
        "torque_nm": 0.3,
        "target_trigger_active": True,
    }
    tactile = {
        "received_at_s": 1.0,
        "packet_counter": 7,
        "timestamp_us": 1_000_000,
        "left_force_n": 1.0,
        "right_force_n": 1.1,
        "raw_left_fz_n": 1.0,
        "raw_right_fz_n": 1.1,
        "left_taxel_forces_n": [[0.0, 0.0, 1.0]],
        "right_taxel_forces_n": [[0.0, 0.0, 1.1]],
        "counter_event": "first",
        "counter_gap": None,
    }
    path = tmp_path / "recording.mcap"
    with RecordingWriter(
        path, started_monotonic_s=1.0, epoch_ns=0, trace_schema=_trace_fields_schema(tuple(trace))
    ) as writer:
        writer.submit("/trace", trace, monotonic_s=1.0)
        writer.submit("/tactile", tactile, monotonic_s=1.0)
        writer.submit(
            "/trace",
            {
                **trace,
                "time_s": 1.1,
                "tactile_timestamp_us": 1_100_000,
                "adaptive_left_candidate": None,
                "target_trigger_active": False,
            },
            monotonic_s=1.1,
        )

    sample = next(iter_snapshots(path))
    assert sample.left_taxel_forces_n == ((0.0, 0.0, 1.0),)
    assert sample.counter_gap is None
    assert phase_ranges(tmp_path) == [("active", 1.0, pytest.approx(1.1))]
    assert online_friction(tmp_path) == {
        "left": (0.4, 0.4, 1),
        "right": (0.5, 0.5, 1),
    }

    video = CurveVideo.__new__(CurveVideo)
    video.run_dir = tmp_path
    video._load()
    assert video.duration == pytest.approx(0.1)
    np.testing.assert_allclose(video.trigger_spans, [(0.0, 0.1)])
    assert np.isnan(video.series["mu_left_rej"][1])


def test_curve_video_accepts_sparse_motor_trace(tmp_path: Path) -> None:
    """仅有电机字段的早期阶段仍可加载为有缺口的曲线。"""
    path = tmp_path / "recording.mcap"
    with RecordingWriter(
        path,
        started_monotonic_s=0.0,
        epoch_ns=0,
        trace_schema=_trace_fields_schema(("time_s", "phase", "position_rad", "torque_nm")),
    ) as writer:
        writer.submit(
            "/trace",
            {"time_s": 0.0, "phase": "ready", "position_rad": 0.1, "torque_nm": 0.2},
            monotonic_s=0.0,
        )
        writer.submit(
            "/trace",
            {"time_s": 0.1, "phase": "ready", "position_rad": 0.2, "torque_nm": 0.3},
            monotonic_s=0.1,
        )

    video = CurveVideo.__new__(CurveVideo)
    video.run_dir = tmp_path
    video._load()
    assert np.isnan(video.series["measured_fz"]).all()
    assert video.trigger_spans == []

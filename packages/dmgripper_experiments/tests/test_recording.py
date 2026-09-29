"""通用真机记录器的 MCAP 产物与时钟契约。"""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import pytest
from mcap.reader import make_reader

from dmgripper_experiments.config import ExperimentConfig, RecordingConfig
from dmgripper_experiments.recording import (
    TRACE_FIELDS,
    ExperimentRecorder,
    create_run_directory,
    strided_sample_sink,
)
from papillarray_hardware.recording import iter_records


def _config() -> ExperimentConfig:
    return ExperimentConfig()


def test_run_directory_is_exclusive_and_scoped_by_names(tmp_path: Path):
    """输出目录按任务／物体分层且同名连续运行不覆盖。"""
    config = _config()
    first = create_run_directory(tmp_path, config)
    second = create_run_directory(tmp_path, config)
    assert first != second
    assert first.parent.name == config.metadata.object_name
    assert first.parent.parent.name == config.metadata.task_name
    assert first.is_dir() and second.is_dir()


def test_recorder_roundtrip_and_shared_time_origin(tmp_path: Path):
    """三种消息原值保留，控制和触觉时间映射到同一 UTC 轴。"""
    directory = create_run_directory(tmp_path, _config())
    recorder = ExperimentRecorder(
        directory, _config(), started_monotonic_s=100.0, epoch_ns=1_000_000_000
    )
    recorder.append({"event": "state", "phase": "preparing", "time_s": 0.2})
    tactile = {
        "received_at_s": 100.3,
        "timestamp_us": 123,
        "left_taxel_forces_n": [[1.234567891234, None, -0.1]],
        "native_slip_active": [True, False],
    }
    recorder.sample(tactile)
    trace = {
        "time_s": 0.4,
        "phase": "preparing",
        "target_force_n": 0.5,
        "stiffness_valid": False,
        "adaptive_capacity_limited": True,
    }
    recorder.write(trace)
    recorder.finish_recording()
    assert list(iter_records(directory / "recording.mcap", "/tactile")) == [tactile]
    assert list(iter_records(directory / "recording.mcap", "/trace")) == [trace]
    assert list(iter_records(directory / "recording.mcap", "/events"))[0]["event"] == "state"
    with (directory / "recording.mcap").open("rb") as handle:
        reader = make_reader(handle)
        messages = {
            channel.topic: message.log_time for _, channel, message in reader.iter_messages()
        }
    assert messages == {
        "/events": 1_200_000_000,
        "/tactile": 1_300_000_000,
        "/trace": 1_400_000_000,
    }
    recorder.register_artifacts({"plot.png": "plot.png"})
    recorder.close()
    manifest = json.loads((directory / "manifest.json").read_text())
    assert manifest["status"] == "completed"
    assert set(manifest["files"]) == {"config.json", "recording.mcap", "plot.png"}
    with pytest.raises(FileExistsError):
        ExperimentRecorder(directory, _config())


def test_recorder_failure_keeps_recording_and_manifest(tmp_path: Path):
    """运行失败仍可读已入队控制轨迹并保留原始错误。"""
    directory = create_run_directory(tmp_path, _config())
    recorder = ExperimentRecorder(directory, _config())
    recorder.write({"time_s": 0.0, "phase": "approach"})
    recorder.close(status="failed", error=RuntimeError("控制周期超时"), disable_confirmed=False)
    assert list(iter_records(directory / "recording.mcap", "/trace"))[0]["phase"] == "approach"
    manifest = json.loads((directory / "manifest.json").read_text())
    assert manifest["status"] == "failed"
    assert manifest["error"]["message"] == "控制周期超时"
    assert manifest["disable_confirmed"] is False


def test_writer_failure_is_visible_on_finish(tmp_path: Path):
    """后台编码失败会保留先前消息，并将运行记为失败。"""
    directory = create_run_directory(tmp_path, _config())
    recorder = ExperimentRecorder(directory, _config(), started_monotonic_s=0.0, epoch_ns=0)
    recorder.sample({"received_at_s": 0.0, "packet_counter": 1})
    recorder.sample({"received_at_s": 0.1, "invalid": object()})
    recorder.close()
    manifest = json.loads((directory / "manifest.json").read_text())
    assert manifest["status"] == "failed"
    assert manifest["error"]["type"] == "RuntimeError"
    assert list(iter_records(directory / "recording.mcap", "/tactile")) == [
        {"received_at_s": 0.0, "packet_counter": 1}
    ]


def test_recording_size_limit_reports_compressed_file_bytes(tmp_path: Path):
    """共享上限按已落盘 MCAP 字节触发正常收尾。"""
    config = replace(_config(), recording=RecordingConfig(max_recording_mib=1e-4))
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config, started_monotonic_s=0.0, epoch_ns=0)
    for index in range(100):
        recorder.sample({"received_at_s": index * 0.001, "packet_counter": index})
    recorder.finish_recording()
    assert recorder.limit_stop_reason is not None
    recorder.close()
    manifest = json.loads((directory / "manifest.json").read_text())
    assert manifest["stop_reason"].startswith("max_recording_mib=")
    assert (directory / "recording.mcap").stat().st_size >= 1e-4 * 1024 * 1024


def test_trace_fields_cover_target_stiffness_and_timing_diagnostics():
    """trace 字段覆盖目标、刚度和实时控制诊断。"""
    assert {
        "time_s",
        "target_force_n",
        "stiffness_n_per_m",
        "adaptive_left_mu",
        "admittance_displacement_m",
        "control_dt_s",
    } <= set(TRACE_FIELDS)


def test_strided_sample_sink_samples_by_packet_counter():
    """按设备包计数抽样；首个、异常与缺计数的包始终保留。"""
    recorded: list[dict[str, object]] = []

    def sink(record):
        recorded.append(dict(record))

    def packet(counter, event="consecutive", gap=None):
        return {"packet_counter": counter, "counter_event": event, "counter_gap": gap}

    assert strided_sample_sink(sink, 1) is sink
    with pytest.raises(ValueError, match="tactile_stride"):
        strided_sample_sink(sink, 0)
    wrapped = strided_sample_sink(sink, 4)
    for counter in range(12):
        wrapped(packet(counter))
    wrapped(packet(5, event="gap"))
    wrapped(packet(6, gap=2))
    wrapped({"left_force_n": 0.0})
    assert [item["packet_counter"] for item in recorded[:5]] == [0, 4, 8, 5, 6]
    assert recorded[5] == {"left_force_n": 0.0}

"""通用记录器与运行目录的产物契约测试。"""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import threading
import time

import pytest

from dmgripper_experiments.config import ExperimentConfig, RecordingConfig
from dmgripper_experiments.recording import (
    TRACE_FIELDS,
    ExperimentRecorder,
    create_run_directory,
    strided_sample_sink,
)


def _config() -> ExperimentConfig:
    return ExperimentConfig()


def test_run_directory_is_exclusive_and_scoped_by_names(tmp_path: Path):
    """输出目录按任务／物体分层且同名连续运行不覆盖。"""
    config = _config()
    first = create_run_directory(tmp_path, config)
    second = create_run_directory(tmp_path, config)
    assert first != second
    assert first.parent.name == sanitize_expected(config.metadata.object_name)
    assert first.parent.parent.name == sanitize_expected(config.metadata.task_name)
    assert first.is_dir() and second.is_dir()


def sanitize_expected(name: str) -> str:
    """与配置清洗保持一致的期望辅助。"""
    from dmgripper_experiments.config import sanitize_directory_component

    return sanitize_directory_component(name)


def test_recorder_writes_config_events_trace_and_manifest(tmp_path: Path):
    """记录器产出 config.json、事件、触觉、trace 与 manifest。"""
    config = _config()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    recorder.append({"event": "state", "phase": "preparing"})
    recorder.sample(
        {
            "counter": 1,
            "timestamp_us": 1000,
            "left_taxel_forces_n": [[0.0, 0.0, 0.1]],
            "right_taxel_forces_n": [[0.0, 0.0, 0.2]],
        }
    )
    recorder.write({"time_s": 0.0, "phase": "preparing", "target_force_n": 0.5})
    recorder.close()
    stored = json.loads((directory / "config.json").read_text(encoding="utf-8"))
    assert stored["schema"] == "dmgripper-experiment/v1"
    assert stored["metadata"]["task_name"] == config.metadata.task_name
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "completed"
    assert manifest["disable_confirmed"] == "not_applicable"
    assert manifest["cleanup_errors"] == []
    tactile_record = json.loads((directory / "tactile.jsonl").read_text(encoding="utf-8"))
    assert tactile_record["left_taxel_forces_n"][0][2] == pytest.approx(0.1)
    assert set(manifest["files"]) >= {
        "config.json",
        "events.jsonl",
        "tactile.jsonl",
        "trace.csv",
    }


def test_recorder_rejects_nonempty_directory_and_unknown_fields(tmp_path: Path):
    """非空目录拒绝覆盖；trace 未知字段拒绝写入。"""
    config = _config()
    directory = create_run_directory(tmp_path, config)
    ExperimentRecorder(directory, config).close()
    with pytest.raises(FileExistsError):
        ExperimentRecorder(directory, config)
    fresh = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(fresh, config)
    with pytest.raises(KeyError, match="未定义字段"):
        recorder.write({"mystery_field": 1.0})
    recorder.close()


def test_recorder_keeps_trace_and_marks_failure(tmp_path: Path):
    """失败收尾保留已写 trace 并在 manifest 记录原始错误与清理错误。"""
    config = _config()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    recorder.write({"time_s": 0.0, "phase": "approach"})
    recorder.close(
        status="failed",
        error=RuntimeError("控制周期超时"),
        disable_confirmed=False,
        cleanup_errors=["失能失败：模拟"],
    )
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["error"]["type"] == "RuntimeError"
    assert manifest["error"]["message"] == "控制周期超时"
    assert manifest["primary_error"] == manifest["error"]
    assert manifest["disable_confirmed"] is False
    assert manifest["cleanup_errors"] == ["失能失败：模拟"]
    assert "trace.csv" in manifest["files"]


def test_recorder_rejects_nonfinite_tactile_without_writing_partial_json(tmp_path: Path):
    """非有限原始触觉值不得形成带 NaN/Infinity 的 JSONL 行。"""
    directory = create_run_directory(tmp_path, _config())
    recorder = ExperimentRecorder(directory, _config())
    with pytest.raises(ValueError, match="JSON"):
        recorder.sample({"left_taxel_forces_n": [[float("nan"), 0.0, 0.0]]})
    recorder.close(status="failed", error="非有限触觉")
    assert (directory / "tactile.jsonl").read_text(encoding="utf-8") == ""


def test_trace_fields_cover_target_stiffness_and_timing_diagnostics():
    """trace 字段覆盖目标来源、刚度诊断与真实时序信息。"""
    required = {
        "time_s",
        "phase",
        "task_time_s",
        "contact_segment",
        "target_source",
        "target_force_n",
        "target_raw_force_n",
        "target_force_rate_n_s",
        "stiffness_n_per_m",
        "stiffness_valid",
        "stiffness_updated",
        "stiffness_reason",
        "control_force_n",
        "control_dt_s",
        "tactile_age_s",
        "command_latency_s",
        "adaptive_left_mu_lower_bound",
        "adaptive_right_mu_lower_bound",
        "adaptive_left_taxel_mu_lower_bound",
        "adaptive_right_taxel_mu_lower_bound",
        "adaptive_left_particle_mu_control",
        "adaptive_left_particle_mu_ess",
        "adaptive_left_particle_mu_reason",
        "adaptive_right_particle_mu_control",
        "adaptive_right_particle_mu_ess",
        "adaptive_right_particle_mu_reason",
        "admittance_displacement_m",
        "admittance_velocity_m_s",
        "admittance_acceleration_m_s2",
        "admittance_velocity_limited",
        "admittance_acceleration_limited",
    }
    assert required <= set(TRACE_FIELDS)


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


def test_tactile_samples_are_queued_and_written_in_order(tmp_path: Path):
    """sample 立即返回，close 前后排空队列并保持包顺序。"""
    directory = create_run_directory(tmp_path, _config())
    recorder = ExperimentRecorder(directory, _config())
    for index in range(50):
        recorder.sample({"packet_counter": index, "left_force_n": index * 0.1})
    recorder.close()
    lines = (directory / "tactile.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 50
    assert json.loads(lines[0])["packet_counter"] == 0
    assert json.loads(lines[-1])["packet_counter"] == 49


def test_tactile_floats_truncated_and_clock_kept_full_precision(tmp_path: Path):
    """浮点截断到 float32 可表示精度，received_at_s 保留 float64。"""
    directory = create_run_directory(tmp_path, _config())
    recorder = ExperimentRecorder(directory, _config())
    received_at_s = 1234567.891234
    left_force_n = 1.234567891234
    taxel = -0.002029961906373501
    recorder.sample(
        {
            "received_at_s": received_at_s,
            "left_force_n": left_force_n,
            "left_taxel_forces_n": [[taxel, 0.1, 2.0]],
        }
    )
    recorder.close()
    stored = json.loads((directory / "tactile.jsonl").read_text(encoding="utf-8"))
    assert stored["received_at_s"] == received_at_s
    assert stored["left_force_n"] == float(f"{left_force_n:.9g}")
    assert stored["left_taxel_forces_n"][0][0] == float(f"{taxel:.9g}")
    raw = (directory / "tactile.jsonl").read_text(encoding="utf-8")
    assert " " not in raw.strip()


def test_tactile_queue_overflow_raises_when_writer_stalls(tmp_path: Path):
    """写线程停顿时队列有界，溢出立即报错而不是阻塞采集线程。"""
    gate = threading.Event()

    class StalledRecorder(ExperimentRecorder):
        def _drain_tactile_queue(self) -> None:
            gate.wait(timeout=10.0)

    directory = create_run_directory(tmp_path, _config())
    recorder = StalledRecorder(
        directory, _config(), tactile_queue_capacity=2, tactile_join_timeout_s=0.2
    )
    recorder.sample({"packet_counter": 0})
    recorder.sample({"packet_counter": 1})
    with pytest.raises(RuntimeError, match="队列已满"):
        recorder.sample({"packet_counter": 2})
    recorder.close()
    gate.set()
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert any("未在超时内排空" in item for item in manifest["cleanup_errors"])


def test_tactile_writer_failure_propagates_and_fails_manifest(tmp_path: Path):
    """写线程 I/O 失败后，后续写入立即抛错且完成收尾升级为失败。"""

    class FailingHandle:
        def write(self, _payload):
            raise OSError("磁盘写入失败")

        def flush(self):
            return None

        def close(self):
            return None

    directory = create_run_directory(tmp_path, _config())
    recorder = ExperimentRecorder(directory, _config())
    recorder._tactile_handle = FailingHandle()
    recorder.sample({"packet_counter": 0})
    deadline = time.monotonic() + 5.0
    while recorder._tactile_writer_error is None and time.monotonic() < deadline:
        time.sleep(0.005)
    with pytest.raises(RuntimeError, match="写线程已失败"):
        recorder.sample({"packet_counter": 1})
    recorder.close()
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["error"]["type"] == "OSError"


def test_tactile_size_limit_stops_writing_and_reports_stop_reason(tmp_path: Path):
    """体积上限触发后停止写盘，manifest 说明原因且收尾仍为完成。"""
    capped = replace(_config(), recording=RecordingConfig(max_tactile_mib=1e-4))
    directory = create_run_directory(tmp_path, capped)
    recorder = ExperimentRecorder(directory, capped)
    for index in range(200):
        recorder.sample({"packet_counter": index, "left_force_n": 1.0})
    deadline = time.monotonic() + 5.0
    while recorder.limit_stop_reason is None and time.monotonic() < deadline:
        time.sleep(0.005)
    assert recorder.limit_stop_reason is not None
    assert recorder.limit_stop_reason.startswith("max_tactile_mib=")
    line_budget = len((directory / "tactile.jsonl").read_text(encoding="utf-8").splitlines()[0])
    for index in range(200, 400):
        recorder.sample({"packet_counter": index})
    recorder.close()
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "completed"
    assert manifest["stop_reason"].startswith("max_tactile_mib=")
    size = (directory / "tactile.jsonl").stat().st_size
    assert size <= int(1e-4 * 1024 * 1024) + line_budget

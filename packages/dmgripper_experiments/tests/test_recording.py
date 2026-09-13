"""通用记录器与运行目录的产物契约测试。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from dmgripper_experiments.config import ExperimentConfig
from dmgripper_experiments.recording import (
    TRACE_FIELDS,
    ExperimentRecorder,
    create_run_directory,
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
    recorder.sample({"counter": 1, "timestamp_us": 1000})
    recorder.write({"time_s": 0.0, "phase": "preparing", "target_force_n": 0.5})
    recorder.close()
    stored = json.loads((directory / "config.json").read_text(encoding="utf-8"))
    assert stored["schema"] == "dmgripper-experiment/v1"
    assert stored["metadata"]["task_name"] == config.metadata.task_name
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "completed"
    assert manifest["disable_confirmed"] == "not_applicable"
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
    assert manifest["disable_confirmed"] is False
    assert manifest["cleanup_errors"] == ["失能失败：模拟"]
    assert "trace.csv" in manifest["files"]


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
    }
    assert required <= set(TRACE_FIELDS)

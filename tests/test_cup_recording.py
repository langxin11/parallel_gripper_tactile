"""验证真机倒水实验记录器。"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from dmgripper_experiments.cup_recording import CupRecorder, TRACE_FIELDS


def test_cup_recorder_writes_config_events_trace_and_manifest(tmp_path: Path) -> None:
    """正常结束时应写入所有基础工件和完成状态。"""
    directory = tmp_path / "run"
    with CupRecorder(directory, {"force_n": 2.0}) as recorder:
        recorder.append({"event": "ready"})
        recorder.sample({"sensor": "left", "raw": [1.0, 2.0, 3.0]})
        recorder.write({"time_s": 0.1, "state": "tracking", "left_fz_n": 1.8})

    assert json.loads((directory / "config.json").read_text(encoding="utf-8")) == {"force_n": 2.0}
    assert json.loads((directory / "events.jsonl").read_text(encoding="utf-8")) == {
        "event": "ready"
    }
    assert json.loads((directory / "tactile.jsonl").read_text(encoding="utf-8")) == {
        "raw": [1.0, 2.0, 3.0],
        "sensor": "left",
    }
    with (directory / "trace.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert tuple(rows[0]) == TRACE_FIELDS
    assert rows[0]["left_fz_n"] == "1.8"
    assert rows[0]["right_fz_n"] == ""
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "completed"
    assert manifest["schema_version"] == 1
    assert set(manifest["files"]) == {"config.json", "events.jsonl", "tactile.jsonl", "trace.csv"}


def test_cup_recorder_rejects_existing_directory(tmp_path: Path) -> None:
    """相同输出目录不得覆盖既有实验。"""
    directory = tmp_path / "run"
    CupRecorder(directory, {}).close()
    with pytest.raises(FileExistsError):
        CupRecorder(directory, {})


def test_cup_recorder_keeps_trace_and_marks_failure(tmp_path: Path) -> None:
    """上下文中的异常不能被吞没，且异常前 trace 必须保留。"""
    directory = tmp_path / "run"
    with pytest.raises(RuntimeError, match="控制失败"):
        with CupRecorder(directory, {}) as recorder:
            recorder.write({"time_s": 0.2, "state": "tracking"})
            raise RuntimeError("控制失败")

    with (directory / "trace.csv").open(encoding="utf-8", newline="") as handle:
        assert len(list(csv.DictReader(handle))) == 1
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["error"] == {"type": "RuntimeError", "message": "控制失败"}

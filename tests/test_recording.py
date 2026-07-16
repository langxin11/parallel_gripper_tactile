"""验证仿真记录钩子。"""

from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path


def _load_recording():
    path = Path(__file__).resolve().parents[1] / "scripts/recording.py"
    spec = importlib.util.spec_from_file_location("recording", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_force_csv_recorder_writes_downsampled_samples(tmp_path: Path) -> None:
    recorder_module = _load_recording()
    output = tmp_path / "forces.csv"
    recorder = recorder_module.ForceCsvRecorder(output, every=2)
    recorder.record(0, 0.0, 1.0, (1, 2, 3), (4, 5, 6))
    recorder.record(1, 0.002, 2.0, (7, 8, 9), (10, 11, 12))
    recorder.close()
    with output.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    assert rows == [
        {
            "step": "0",
            "time_s": "0.0",
            "control": "1.0",
            "left_fx": "1",
            "left_fy": "2",
            "left_fz": "3",
            "right_fx": "4",
            "right_fy": "5",
            "right_fz": "6",
        }
    ]

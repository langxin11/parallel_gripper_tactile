"""验证真机倒水实验离线绘图。"""

from __future__ import annotations

from pathlib import Path

from dmgripper_experiments.cup_plot import plot_cup_run
from dmgripper_experiments.cup_recording import TRACE_FIELDS, CupRecorder
import json


def test_plot_cup_run_writes_pdf_and_png(tmp_path: Path) -> None:
    """合成 trace 应实际渲染 PDF 和高分辨率 PNG。"""
    directory = tmp_path / "run"
    with CupRecorder(directory, {"source": "离线绘图测试"}) as recorder:
        for index in range(20):
            recorder.write(
                {
                    "time_s": index * 0.1,
                    "state": "pour",
                    "left_fz_n": 0.5 + index * 0.01,
                    "right_fz_n": 0.52 + index * 0.01,
                    "target_force_n": 0.5 + index * 0.015,
                    "raw_left_fx_n": index * 0.005,
                    "raw_left_fy_n": 0.1,
                    "raw_right_fx_n": index * 0.006,
                    "raw_right_fy_n": -0.1,
                    "trigger_active": index >= 10,
                    "position_rad": 0.3 + index * 0.001,
                    "q_des_rad": 0.31 + index * 0.001,
                    "torque_nm": 0.1 + index * 0.005,
                }
            )
    paths = plot_cup_run(directory)
    assert paths == (directory / "plot.pdf", directory / "plot.png")
    assert all(path.is_file() and path.stat().st_size > 1000 for path in paths)
    manifest = json.loads((directory / "manifest.json").read_text())
    assert {"plot.pdf", "plot.png"}.issubset(manifest["files"])
    assert manifest["status"] == "completed"


def test_plot_cup_run_returns_empty_for_missing_or_empty_trace(tmp_path: Path) -> None:
    """不存在或仅有表头的 trace 不应生成空白图。"""
    assert plot_cup_run(tmp_path) == ()
    (tmp_path / "trace.csv").write_text(",".join(TRACE_FIELDS) + "\n", encoding="utf-8")
    assert plot_cup_run(tmp_path) == ()

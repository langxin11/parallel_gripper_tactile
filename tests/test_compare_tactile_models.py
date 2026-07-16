"""验证公平触觉模型比较工具。"""

from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path


def _load_comparison():
    scripts = Path(__file__).resolve().parents[1] / "scripts"
    sys.path.insert(0, str(scripts))
    path = scripts / "compare_tactile_models.py"
    spec = importlib.util.spec_from_file_location("compare_tactile_models", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_comparison_summary_uses_steady_tail_and_symmetric_error() -> None:
    comparison = _load_comparison()
    box = [(0.0, 0.0, value) for value in (0, 1, 2, 10, 10)]
    grid = [(0.0, 0.0, value) for value in (0, 1, 2, 9, 9)]
    summary = comparison.summarize_side(box, grid)
    assert summary.box_steady_n == 10
    assert summary.grid_steady_n == 9
    assert abs(summary.relative_error - 1 / 9.5) < 1e-12
    assert summary.box_peak_n == 10
    assert summary.grid_peak_n == 9


def test_comparison_csv_uses_common_positive_pressure_convention(tmp_path: Path) -> None:
    comparison = _load_comparison()
    box = comparison.ForceTrace(
        [0.002, 0.004], [0.0, 220.0], [(1, 2, 3), (4, 5, 6)], [(7, 8, 9), (10, 11, 12)]
    )
    grid = comparison.ForceTrace(
        [0.002, 0.004], [0.0, 220.0], [(1, 2, 2), (4, 5, 5)], [(7, 8, 8), (10, 11, 11)]
    )
    output = tmp_path / "comparison.csv"
    comparison.write_comparison_csv(output, box, grid, record_every=2)
    with output.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == 2
    assert rows[-1]["box_taxel_left_pressure_n"] == "6"
    assert rows[-1]["touch_grid_right_fz"] == "11"


def test_box_taxel_and_touch_grid_steady_forces_agree() -> None:
    comparison = _load_comparison()
    box, grid = comparison.run_comparison(steps=600)
    assert comparison.summarize_side(box.left, grid.left).relative_error <= 0.10
    assert comparison.summarize_side(box.right, grid.right).relative_error <= 0.10

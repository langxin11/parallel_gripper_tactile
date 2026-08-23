"""验证公平触觉模型比较工具。"""

from __future__ import annotations

import csv
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np


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
    """稳态尾段均值与对称相对误差计算。"""
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
    """CSV 输出采用统一的压缩压力为正约定。"""
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


def test_disturbance_protocol_phases_and_waveform() -> None:
    """扰动协议的相位边界与正弦波形。"""
    comparison = _load_comparison()
    protocol = comparison.DisturbanceProtocol()
    assert protocol.release_time == 1.5
    assert protocol.disturbance_start == 2.0
    assert protocol.disturbance_end == 3.0
    assert protocol.total_duration == 3.5
    assert protocol.phase_at(0.5) == "close"
    assert protocol.phase_at(1.25) == "support_settle"
    assert protocol.phase_at(1.75) == "unsupported_hold"
    assert protocol.phase_at(2.25) == "disturbance"
    assert protocol.phase_at(3.25) == "recovery"
    assert abs(protocol.force_y_at(2.125) - 5.0) < 1e-12
    assert abs(protocol.force_y_at(2.375) + 5.0) < 1e-12
    assert protocol.force_y_at(1.999) == 0.0
    assert protocol.force_y_at(3.0) == 0.0


def test_release_disables_support_and_applies_world_y_force() -> None:
    """释放后支撑禁用且注入世界 Y 向外力。"""
    comparison = _load_comparison()
    protocol = comparison.DisturbanceProtocol()

    class Model:
        geom_contype = np.array([1], dtype=int)
        geom_conaffinity = np.array([1], dtype=int)

    class Data:
        time = 2.125
        ctrl = np.zeros(1)
        xfrc_applied = np.ones((2, 6))

    phase = protocol.phase_at(Data.time)
    applied = protocol.step(
        Model(), Data(), actuator_id=0, close_control=220.0, support_geom_id=0, cube_body_id=1
    )
    assert phase == "disturbance"
    assert Model.geom_contype[0] == 0
    assert Model.geom_conaffinity[0] == 0
    assert np.allclose(Data.xfrc_applied[1], [0, 5, 0, 0, 0, 0])
    assert applied == (0.0, 5.0, 0.0)


def test_local_force_rotation_uses_site_frame() -> None:
    """接触力经 site 旋转矩阵变换到世界系。"""
    comparison = _load_comparison()
    rotation = (0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert comparison._rotate_local_to_world(rotation, (2.0, 3.0, 4.0)) == (
        -3.0,
        2.0,
        4.0,
    )


def _disturbance_trace(comparison, total_world_y, y_positions):
    count = len(total_world_y)
    zeros = [(0.0, 0.0, 0.0)] * count
    return comparison.ForceTrace(
        time_s=[0.1 * index for index in range(count)],
        control=[220.0] * count,
        left=zeros.copy(),
        right=zeros.copy(),
        phase=["disturbance"] * (count - 1) + ["recovery"],
        applied_force_world=zeros.copy(),
        left_world=[(0.0, value / 2.0, 0.0) for value in total_world_y],
        right_world=[(0.0, value / 2.0, 0.0) for value in total_world_y],
        cube_position_world=[(0.0, value, 0.0) for value in y_positions],
        cube_velocity_world=[(0.0, value, 0.0) for value in y_positions],
    )


def test_disturbance_summary_compares_world_response_and_slip() -> None:
    """扰动汇总比较世界系响应与滑移判定。"""
    comparison = _load_comparison()
    box = _disturbance_trace(comparison, [1.0, 2.0, 0.0], [0.0, 0.001, 0.003])
    grid = _disturbance_trace(comparison, [1.1, 1.8, 0.0], [0.0, 0.0005, 0.001])
    summary = comparison.summarize_disturbance(box, grid, slip_threshold_m=0.002)
    expected_nrmse = math.sqrt((0.1**2 + 0.2**2) / 2) / 2.0
    assert abs(summary.response_nrmse - expected_nrmse) < 1e-12
    assert summary.box_slipped
    assert not summary.grid_slipped


def test_disturbance_csv_contains_world_force_and_cube_motion(tmp_path: Path) -> None:
    """扰动 CSV 含世界系外力与物体运动列。"""
    comparison = _load_comparison()
    box = _disturbance_trace(comparison, [1.0, 2.0], [0.0, 0.001])
    grid = _disturbance_trace(comparison, [1.1, 1.9], [0.0, 0.001])
    output = tmp_path / "disturbance.csv"
    comparison.write_comparison_csv(output, box, grid)
    with output.open(newline="", encoding="utf-8") as file:
        row = next(csv.DictReader(file))
    assert row["phase"] == "disturbance"
    assert "applied_world_fy" in row
    assert row["box_taxel_total_world_fy"] == "1.0"
    assert "touch_grid_cube_vy" in row


def test_box_taxel_and_touch_grid_steady_forces_agree() -> None:
    """box taxel 与 touch_grid 稳态法向力一致。"""
    comparison = _load_comparison()
    box, grid = comparison.run_comparison(steps=600)
    assert comparison.summarize_side(box.left, grid.left).relative_error <= 0.10
    assert comparison.summarize_side(box.right, grid.right).relative_error <= 0.10


def test_default_disturbance_responses_agree_without_slip() -> None:
    """默认扰动下两种模型响应一致且不滑移。"""
    comparison = _load_comparison()
    protocol = comparison.DisturbanceProtocol()
    box, grid = comparison.run_comparison(protocol=protocol)
    summary = comparison.summarize_disturbance(box, grid, protocol.slip_threshold_m)
    assert summary.response_nrmse <= 0.10
    assert not summary.box_slipped
    assert not summary.grid_slipped

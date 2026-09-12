"""验证切向扰动证据图的产物和面板语义。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from parallel_gripper_tactile.visualization import tangential_disturbance as visualization
from parallel_gripper_tactile.visualization.plotstyle import TEXT_WIDTH_IN


def _rows() -> list[dict[str, object]]:
    """构造包含筛选前后阶段和一次触发的紧凑合成记录。"""
    return [
        {
            "time_s": 0.0,
            "phase": "approach",
            "disturbance_time_s": -0.4,
            "target_force_n": 1.0,
            "actual_normal_force_n": 0.2,
            "measured_tangential_force_n": 0.0,
            "applied_tangential_force_n": 0.0,
            "trigger_score": 0.0,
            "trigger_active": False,
            "tangential_displacement_m": 0.0,
        },
        {
            "time_s": 0.4,
            "phase": "initial_hold",
            "disturbance_time_s": -0.1,
            "target_force_n": 2.0,
            "actual_normal_force_n": 1.9,
            "measured_tangential_force_n": 0.04,
            "applied_tangential_force_n": 0.0,
            "trigger_score": 0.1,
            "trigger_active": False,
            "tangential_displacement_m": 0.0,
        },
        {
            "time_s": 0.5,
            "phase": "disturbance",
            "disturbance_time_s": 0.0,
            "target_force_n": 2.0,
            "actual_normal_force_n": 1.95,
            "measured_tangential_force_n": 0.35,
            "applied_tangential_force_n": 0.3,
            "trigger_score": 1.1,
            "trigger_active": True,
            "tangential_displacement_m": 0.0002,
        },
        {
            "time_s": 0.6,
            "phase": "disturbance",
            "disturbance_time_s": 0.1,
            "target_force_n": 2.0,
            "actual_normal_force_n": 2.02,
            "measured_tangential_force_n": 0.6,
            "applied_tangential_force_n": 0.5,
            "trigger_score": 1.3,
            "trigger_active": True,
            "tangential_displacement_m": 0.0012,
        },
    ]


def test_plot_tangential_disturbance_writes_two_formats_and_three_panels(
    tmp_path: Path, monkeypatch: Any, fast_plot_render: None
) -> None:
    """筛选后使用扰动时间轴，并输出三个互补的证据面板。"""
    captured: list[tuple[str, list[str], list[np.ndarray], int]] = []
    figure_widths: list[float] = []
    original_save = visualization.save_publication_figure

    def capture(figure: Any, path: Path, **kwargs: object) -> Path:
        """在正常保存前记录面板标签和横坐标数据。"""
        figure_widths.append(float(figure.get_size_inches()[0]))
        captured.extend(
            (
                axis.get_ylabel(),
                axis.get_legend_handles_labels()[1],
                [np.asarray(line.get_xdata(), dtype=float) for line in axis.lines],
                len(axis.collections),
            )
            for axis in figure.axes
        )
        return original_save(figure, path, **kwargs)

    monkeypatch.setattr(visualization, "save_publication_figure", capture)
    output = tmp_path / "disturbance.csv"

    visualization.plot_tangential_disturbance(output, _rows(), slip_threshold_m=0.001)

    assert (tmp_path / "disturbance.pdf").is_file()
    assert (tmp_path / "disturbance.png").is_file()
    assert (tmp_path / "disturbance.pdf").stat().st_size > 0
    assert (tmp_path / "disturbance.png").stat().st_size > 0
    assert [panel[0] for panel in captured[:3]] == [
        "Mean-side\nnormal force (N)",
        "Tangential force (N)",
        "Displacement (mm)",
    ]
    assert figure_widths == [TEXT_WIDTH_IN, TEXT_WIDTH_IN]
    assert "Applied load (truth only)" in captured[1][1]
    assert [panel[3] for panel in captured[:3]] == [0, 1, 0]
    assert any(np.array_equal(values, np.asarray([-0.1, 0.0, 0.1])) for values in captured[0][2])


def test_plot_tangential_disturbance_skips_rows_without_plot_phases(tmp_path: Path) -> None:
    """没有保持或扰动阶段时不创建目录或空白工件。"""
    output = tmp_path / "missing" / "disturbance.png"

    visualization.plot_tangential_disturbance(
        output,
        [{"phase": "approach", "disturbance_time_s": -0.2}],
        slip_threshold_m=0.001,
    )

    assert not output.with_suffix(".png").exists()
    assert not output.with_suffix(".pdf").exists()
    assert not output.parent.exists()

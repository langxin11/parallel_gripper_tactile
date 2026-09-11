"""DMgripper 力跟踪独立绘图入口测试。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from parallel_gripper_tactile.visualization import force_tracking as visualization


def _trace() -> list[dict[str, object]]:
    """构造同时包含控制、触觉和有效刚度状态的最小 trace。"""
    return [
        {
            "time_s": 10.0 + index,
            "control_time_s": 9.9 + index,
            "command_time_s": 9.8 + index,
            "reference_start_time_s": 10.0,
            "tracking_time_s": float(index),
            "phase": "approach_contact" if index == 0 else "track_reference",
            "control_state": "approach" if index == 0 else "force_tracking",
            "target_normal_force_n": 1.0 + index,
            "filtered_normal_force_n": 0.8 + index,
            "measured_normal_force_n": 0.7 + index,
            "measured_left_fx": 0.1 * (index + 1),
            "measured_left_fy": 0.2 * (index + 1),
            "measured_left_fz": 1.0 + index,
            "measured_right_fx": 0.3 * (index + 1),
            "measured_right_fy": 0.4 * (index + 1),
            "measured_right_fz": 1.2 + index,
            "desired_position_rad": 0.2 + 0.1 * index,
            "drive_position_rad": 0.15 + 0.1 * index,
            "desired_velocity_rad_s": 0.1,
            "drive_velocity_rad_s": 0.09,
            "commanded_torque_n_m": 0.4 + 0.01 * index,
            "actuator_torque_n_m": 0.39 + 0.01 * index,
            "mit_feedforward_torque_n_m": 0.05,
            "estimated_contact_stiffness_n_per_m": 1_000.0 + 10.0 * index,
            "stiffness_valid": index > 0,
        }
        for index in range(3)
    ]


def _config() -> dict[str, object]:
    """构造兼容 effective_parameters.json 的最小配置。"""
    return {
        "profile": {
            "tactile": {"rows": 3, "cols": 3},
            "control": {
                "mit": {"kp": 20.0, "kd": 0.64},
                "force": {
                    "kp": 0.02,
                    "ki": 0.2,
                    "kd": 0.0,
                    "stiffness": {"enabled": True},
                },
            },
        },
        "runtime": {"force_semantics": "average_side"},
        "task": {
            "reference": {
                "interpolation": "linear",
                "waypoints": [{"t_s": 0.0, "force_n": 1.0}, {"t_s": 2.0, "force_n": 3.0}],
            }
        },
    }


def test_waypoints_use_explicit_reference_start_and_linear_task() -> None:
    """线性任务标记应基于保存的绝对起点，而非任务相对时间。"""
    trace = _trace()

    assert visualization._tracking_start_time(trace, None) == 10.0
    assert visualization._waypoint_times(visualization._task_config(_config()), 10.0) == [
        (10.0, 1.0),
        (12.0, 3.0),
    ]


def test_tracking_start_is_not_inferred_from_approach_time() -> None:
    """未进入跟踪时不能用接近阶段的零相对时间伪造 waypoint 起点。"""
    trace = [
        {"time_s": 2.0, "tracking_time_s": 0.0, "phase": "approach_contact"},
        {"time_s": 2.1, "tracking_time_s": 0.0, "phase": "contact_settle"},
    ]
    assert visualization._tracking_start_time(trace, None) is None


def test_events_do_not_label_initial_approach_as_reapproach() -> None:
    """仅在确认过接触后重新进入 approach 才标记重接近。"""
    trace = [
        {"control_state": "approach", "phase": "approach_contact"},
        {"control_state": "contact_transition", "phase": "contact_settle"},
        {"control_state": "force_tracking", "phase": "track_reference"},
        {"control_state": "approach", "phase": "track_reference"},
    ]
    assert visualization._events(trace, np.arange(4, dtype=float)) == [
        (1.0, "Contact confirmed"),
        (2.0, "Tracking start"),
        (3.0, "Reapproach"),
    ]


def test_plot_series_preserves_nan_breaks() -> None:
    """无效刚度区间必须以 NaN 断线，不能跨区间连接。"""
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots()
    visualization._plot_series(
        axis,
        np.asarray([0.0, 1.0, 2.0]),
        np.asarray([1.0, np.nan, 3.0]),
    )
    assert np.isnan(axis.lines[0].get_ydata()[1])
    plt.close(figure)


def test_render_requires_a_real_time_axis(tmp_path: Path) -> None:
    """缺少物理时间的历史数据不能把行号伪装为秒。"""
    with pytest.raises(ValueError, match="不含可绘制"):
        visualization.render_run_artifacts(
            trace=[{"target_normal_force_n": 1.0, "filtered_normal_force_n": 0.9}],
            metrics=None,
            config=None,
            output_dir=tmp_path,
        )


def test_stiffness_mask_requires_explicit_enabled_and_valid_samples() -> None:
    """刚度面板不能把缺失有效性或禁用估计器的历史数据画成有效曲线。"""
    trace = _trace()

    assert np.array_equal(
        visualization._stiffness_mask(trace, _config()), np.asarray([False, True, True])
    )
    assert not visualization._stiffness_mask(trace, {"profile": {}}).any()
    assert not visualization._stiffness_mask(
        [{key: value for key, value in row.items() if key != "stiffness_valid"} for row in trace],
        _config(),
    ).any()


def test_render_artifacts_uses_symbolic_labels_and_only_requested_format(
    tmp_path: Path, monkeypatch, fast_plot_render: None
) -> None:
    """三张主图使用物理符号，线性 waypoint 与控制时间轴保持准确。"""
    captured: dict[str, list[tuple[str, list[str], list[np.ndarray]]]] = {}
    original_save = visualization.save_publication_figure

    def capture(figure: Any, path: Path, **kwargs: object) -> Path:
        """在正常写图前捕获图例和折线数据，避免像素基线断言。"""
        captured[path.stem] = [
            (
                axis.get_xlabel(),
                axis.get_legend_handles_labels()[1],
                [np.asarray(line.get_xdata(), dtype=float) for line in axis.lines],
            )
            for axis in figure.axes
        ]
        return original_save(figure, path, **kwargs)

    monkeypatch.setattr(visualization, "save_publication_figure", capture)
    paths = visualization.render_run_artifacts(
        trace=_trace(),
        metrics={"tracking_start_time_s": 10.0, "rmse_n": 0.1, "mae_n": 0.08},
        config=_config(),
        output_dir=tmp_path,
    )

    assert [path.name for path in paths] == ["tracking.png", "tactile.png", "controller.png"]
    assert all(path.is_file() and path.stat().st_size > 0 for path in paths)
    assert not list(tmp_path.glob("*.pdf"))
    tracking = captured["tracking"]
    assert r"$F_{\mathrm{ref}}$" in tracking[0][1]
    assert tracking[0][0] == r"$t\,(\mathrm{s})$"
    assert any(np.array_equal(line, np.asarray([10.0])) for line in tracking[0][2])
    assert any(np.array_equal(line, np.asarray([12.0])) for line in tracking[0][2])
    controller = captured["controller"]
    assert any(np.array_equal(line, np.asarray([9.8, 10.8, 11.8])) for line in controller[0][2])


def test_detail_requires_confirmed_contact_and_complete_three_by_three_grid(
    tmp_path: Path, fast_plot_render: None
) -> None:
    """逐 taxel 细节图只接受完整 3×3 数据和明确的接触确认状态。"""
    trace = _trace()
    for side in ("left", "right"):
        for component in ("fx", "fy", "fz"):
            for row in range(3):
                for column in range(3):
                    for index, sample in enumerate(trace):
                        sample[f"{side}_taxel_{component}_{row}_{column}"] = 0.1 * (index + 1)
    paths = visualization.render_run_artifacts(
        trace=trace,
        metrics=None,
        config=_config(),
        output_dir=tmp_path,
        tactile_detail=True,
    )

    assert {path.name for path in paths} == {
        "tracking.png",
        "tactile.png",
        "controller.png",
        "tactile_left_detail.png",
        "tactile_right_detail.png",
    }
    trace[1]["control_state"] = "approach"
    trace[1]["phase"] = "contact_settle"
    trace[2]["control_state"] = "approach"
    trace[2]["phase"] = "contact_settle"
    assert visualization._render_taxel_figure(trace, "left", _config()) is None


def test_taxel_detail_uses_spatial_three_by_three_panels() -> None:
    """每个 taxel 应占据对应空间位置，并用双量程保留三轴原始力。"""
    import matplotlib.pyplot as plt

    trace = _trace()
    for side in ("left", "right"):
        for component in ("fx", "fy", "fz"):
            for row in range(3):
                for column in range(3):
                    for index, sample in enumerate(trace):
                        sample[f"{side}_taxel_{component}_{row}_{column}"] = (
                            (index + 1) * (row * 3 + column + 1) * 0.01
                        )
    figure = visualization._render_taxel_figure(trace, "left", _config())

    assert figure is not None
    titles = {axis.get_title() for axis in figure.axes if axis.get_title()}
    assert titles == {rf"$T_{{{row},{column}}}$" for row in range(3) for column in range(3)}
    assert len(figure.axes) == 18
    assert figure.axes[3].get_ylabel() == r"$F_z\,(\mathrm{N})$"
    assert any(axis.get_ylabel() == r"$F_x,F_y\,(\mathrm{N})$" for axis in figure.axes)
    plt.close(figure)

"""绘制切向扰动实验的紧凑证据图。"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from .plotstyle import paper_figsize, save_publication_figure, science_pyplot


_COLORS = {
    "reference": "#000000",
    "feedback": "#0072B2",
    "measured": "#D55E00",
    "truth": "#666666",
    "trigger": "#009E73",
    "threshold": "#666666",
}
_PLOT_PHASES = frozenset(("initial_hold", "disturbance"))


def _as_float(value: object) -> float:
    """把输入转换为有限浮点数，缺失值保持为 NaN。"""
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _values(rows: list[dict], field: str) -> np.ndarray:
    """提取一列数值，避免以零伪造缺失的实验记录。"""
    return np.asarray([_as_float(row.get(field)) for row in rows], dtype=float)


def _as_bool(value: object) -> bool:
    """兼容原生布尔值、整数和 CSV 序列化后的布尔字符串。"""
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true"}
    return bool(value)


def _trigger_indices(rows: list[dict], times: np.ndarray) -> np.ndarray:
    """返回触发状态的上升沿，避免每个活动样本都产生标记。"""
    active = np.asarray([_as_bool(row.get("trigger_active")) for row in rows], dtype=bool)
    rising = active & ~np.r_[False, active[:-1]]
    return np.flatnonzero(rising & np.isfinite(times))


def _plot_series(axis: Any, times: np.ndarray, values: np.ndarray, **kwargs: object) -> None:
    """按有效时间绘制序列，同时保留数值 NaN 造成的断线。"""
    valid_time = np.isfinite(times)
    if valid_time.any() and np.isfinite(values[valid_time]).any():
        axis.plot(times[valid_time], values[valid_time], **kwargs)


def plot_tangential_disturbance(
    path: Path,
    rows: list[dict],
    *,
    slip_threshold_m: float,
) -> None:
    """生成切向扰动的法向力、载荷和滑移证据图。

    仅消费 ``initial_hold`` 与 ``disturbance`` 阶段的已记录行。图中的外加载荷
    只用于离线评分，因此明确标注为 truth only；函数不计算任何科学指标，也不
    平滑、补造或改变原始记录。

    Args:
        path: 输出基路径；无论后缀为何，均以同一 stem 写出 PDF 与 PNG。
        rows: 内存中的逐行实验记录。
        slip_threshold_m: 已由任务定义的滑移阈值，单位为米。
    """
    selected = [row for row in rows if row.get("phase") in _PLOT_PHASES]
    if not selected:
        return

    times = _values(selected, "disturbance_time_s")
    target_normal = _values(selected, "target_force_n")
    actual_normal = _values(selected, "actual_normal_force_n")
    measured_tangential = _values(selected, "measured_tangential_force_n")
    applied_tangential = _values(selected, "applied_tangential_force_n")
    displacement_mm = 1000.0 * _values(selected, "tangential_displacement_m")
    trigger_indices = _trigger_indices(selected, times)

    plt = science_pyplot()
    figure, axes = plt.subplots(
        3,
        1,
        sharex=True,
        figsize=paper_figsize(4.85, columns=2),
        layout="constrained",
    )

    _plot_series(
        axes[0],
        times,
        target_normal,
        color=_COLORS["reference"],
        ls="--",
        label="Target normal force",
    )
    _plot_series(
        axes[0],
        times,
        actual_normal,
        color=_COLORS["feedback"],
        label="Actual normal force",
    )
    axes[0].set_ylabel("Mean-side\nnormal force (N)")

    _plot_series(
        axes[1],
        times,
        measured_tangential,
        color=_COLORS["measured"],
        label="Measured tangential force",
    )
    _plot_series(
        axes[1],
        times,
        applied_tangential,
        color=_COLORS["truth"],
        ls="--",
        label="Applied load (truth only)",
    )
    if trigger_indices.size:
        axes[1].scatter(
            times[trigger_indices],
            measured_tangential[trigger_indices],
            marker="o",
            s=18,
            facecolors="white",
            edgecolors=_COLORS["trigger"],
            linewidths=0.9,
            label="Trigger",
            zorder=4,
        )
    axes[1].set_ylabel("Tangential force (N)")

    _plot_series(
        axes[2],
        times,
        displacement_mm,
        color=_COLORS["feedback"],
        label="Tangential displacement",
    )
    threshold_mm = 1000.0 * _as_float(slip_threshold_m)
    if math.isfinite(threshold_mm):
        axes[2].axhline(
            threshold_mm,
            color=_COLORS["threshold"],
            ls="--",
            lw=1.0,
            label="Slip threshold",
        )
    axes[2].set_ylabel("Displacement (mm)")
    axes[2].set_xlabel(r"$t_{\mathrm{dist}}\,(\mathrm{s})$")

    for axis in axes:
        axis.grid(alpha=0.18)
        handles, labels = axis.get_legend_handles_labels()
        if handles:
            axis.legend(handles, labels, loc="best", frameon=False)

    base_path = Path(path).with_suffix("")
    try:
        save_publication_figure(figure, base_path.with_suffix(".pdf"))
        save_publication_figure(figure, base_path.with_suffix(".png"))
    finally:
        plt.close(figure)


__all__ = ["plot_tangential_disturbance"]

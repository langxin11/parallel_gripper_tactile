"""绘制真机倒水实验的控制 trace。"""

from __future__ import annotations

import csv
import json
import math
from contextlib import contextmanager
from pathlib import Path
from typing import Any


def plot_cup_run(directory: Path) -> tuple[Path, ...]:
    """从倒水实验 trace 生成三联力控诊断图。

    图只呈现记录中存在的控制量和触觉量。没有物体位移记录时，不推断或绘制
    滑移曲线。

    Args:
        directory: 包含 `trace.csv` 的实验输出目录。

    Returns:
        成功生成时的 PDF 与 PNG 路径；不存在或没有数据的 trace 返回空元组。
    """
    directory = Path(directory)
    trace_path = directory / "trace.csv"
    rows = _read_rows(trace_path)
    if not rows:
        return ()
    time_s = _values(rows, "time_s")
    if not any(math.isfinite(value) for value in time_s):
        return ()

    plt, style_context = _plotting_api()
    with style_context():
        figure, axes = plt.subplots(3, 1, sharex=True, figsize=(7.16, 6.2), layout="constrained")
        _plot_normal_force(axes[0], time_s, rows)
        _plot_tangential_force(axes[1], time_s, rows)
        _plot_joint_command(axes[2], time_s, rows)
        axes[2].set_xlabel("Time (s)")
        for axis in axes:
            axis.grid(True, alpha=0.25)
        pdf_path = directory / "plot.pdf"
        png_path = directory / "plot.png"
        figure.savefig(pdf_path, format="pdf")
        figure.savefig(png_path, format="png", dpi=600)
        plt.close(figure)
    manifest_path = directory / "manifest.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["files"] = list(
            dict.fromkeys([*manifest.get("files", []), pdf_path.name, png_path.name])
        )
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return (pdf_path, png_path)


def _read_rows(trace_path: Path) -> list[dict[str, str]]:
    """读取非空 CSV 行；损坏或缺失文件视为没有可绘制数据。"""
    if not trace_path.is_file() or trace_path.stat().st_size == 0:
        return []
    try:
        with trace_path.open(encoding="utf-8", newline="") as handle:
            return list(csv.DictReader(handle))
    except (csv.Error, OSError, UnicodeError):
        return []


def _plotting_api() -> tuple[Any, Any]:
    """返回独立于仿真包的论文绘图接口与样式上下文。"""
    import matplotlib as mpl
    import matplotlib.pyplot as plt

    styles: list[str] = []
    try:
        import scienceplots  # noqa: F401
    except ModuleNotFoundError:
        pass
    else:
        styles = ["science", "ieee", "no-latex"]
    parameters = {
        "font.family": ["Noto Serif CJK SC", "DejaVu Serif"],
        "font.size": 9,
        "axes.labelsize": 9,
        "axes.titlesize": 10,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8,
        "lines.linewidth": 1.2,
        "axes.linewidth": 0.8,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "savefig.dpi": 600,
    }

    @contextmanager
    def style_context() -> Any:
        with plt.style.context(styles), mpl.rc_context(parameters):
            yield

    return plt, style_context


def main() -> None:
    """提供 `dmgripper-cup-plot` 命令行入口。"""
    import tyro

    tyro.cli(_main)


def _main(directory: Path) -> None:
    """绘制指定目录，并输出生成的文件路径。"""
    for path in plot_cup_run(directory):
        print(path)


def _plot_normal_force(axis: Any, time_s: list[float], rows: list[dict[str, str]]) -> None:
    """绘制平均单侧法向实际力与目标力。"""
    left = _values(rows, "left_fz_n")
    right = _values(rows, "right_fz_n")
    measured = _mean_pair(left, right)
    if not any(math.isfinite(value) for value in measured):
        measured = _values(rows, "measured_force_n")
    _line_if_data(axis, time_s, measured, "Measured, avg. side", "C0")
    _line_if_data(axis, time_s, _values(rows, "target_force_n"), "Target", "C3", linestyle="--")
    axis.set_ylabel("Normal force (N)")
    _legend(axis)


def _plot_tangential_force(axis: Any, time_s: list[float], rows: list[dict[str, str]]) -> None:
    """绘制左右触觉切向力模和触发状态。"""
    left = _magnitude(_values(rows, "raw_left_fx_n"), _values(rows, "raw_left_fy_n"))
    right = _magnitude(_values(rows, "raw_right_fx_n"), _values(rows, "raw_right_fy_n"))
    _line_if_data(axis, time_s, left, "Left tangential", "C0")
    _line_if_data(axis, time_s, right, "Right tangential", "C1")
    axis.set_ylabel("Tangential force (N)")
    trigger = _values(rows, "trigger_active", boolean=True)
    if any(math.isfinite(value) for value in trigger):
        trigger_axis = axis.twinx()
        trigger_axis.step(time_s, trigger, where="post", color="C3", linewidth=1.0, label="Trigger")
        trigger_axis.set_ylabel("Trigger")
        trigger_axis.set_ylim(-0.1, 1.1)
        _legend(axis, trigger_axis)
    else:
        _legend(axis)


def _plot_joint_command(axis: Any, time_s: list[float], rows: list[dict[str, str]]) -> None:
    """绘制关节位置与电机力矩，并明确标注各自单位。"""
    _line_if_data(axis, time_s, _values(rows, "position_rad"), "Joint position", "C0")
    _line_if_data(
        axis, time_s, _values(rows, "q_des_rad"), "Desired position", "C2", linestyle="--"
    )
    axis.set_ylabel("Joint position (rad)")
    torque_axis = axis.twinx()
    _line_if_data(
        torque_axis, time_s, _values(rows, "torque_nm"), "Motor torque", "#CC79A7", linestyle=":"
    )
    torque_axis.set_ylabel("Motor torque (N m)")
    _legend(axis, torque_axis)


def _values(rows: list[dict[str, str]], field: str, *, boolean: bool = False) -> list[float]:
    """解析一列；空值或异常值保持为 NaN，绝不补为零。"""
    if boolean:
        return [_parse_boolean(row.get(field, "")) for row in rows]
    return [_parse_number(row.get(field, "")) for row in rows]


def _parse_number(value: str) -> float:
    """解析有限浮点数，失败时返回 NaN。"""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _parse_boolean(value: str) -> float:
    """解析常用布尔文本；非布尔值保持为 NaN。"""
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return 1.0
    if normalized in {"0", "false", "no", "off"}:
        return 0.0
    return math.nan


def _mean_pair(left: list[float], right: list[float]) -> list[float]:
    """仅在双侧都有效时计算平均单侧量。"""
    return [
        (left_value + right_value) / 2.0
        if math.isfinite(left_value) and math.isfinite(right_value)
        else math.nan
        for left_value, right_value in zip(left, right, strict=True)
    ]


def _magnitude(x: list[float], y: list[float]) -> list[float]:
    """仅在两个分量都有效时计算切向力模。"""
    return [
        math.hypot(x_value, y_value)
        if math.isfinite(x_value) and math.isfinite(y_value)
        else math.nan
        for x_value, y_value in zip(x, y, strict=True)
    ]


def _line_if_data(
    axis: Any, x: list[float], y: list[float], label: str, color: str, **kwargs: Any
) -> None:
    """仅在序列含有限值时绘制曲线。"""
    if any(math.isfinite(value) for value in y):
        axis.plot(x, y, label=label, color=color, **kwargs)


def _legend(*axes: Any) -> None:
    """合并一个或两个纵轴的有效图例。"""
    handles: list[Any] = []
    labels: list[str] = []
    for axis in axes:
        axis_handles, axis_labels = axis.get_legend_handles_labels()
        handles.extend(axis_handles)
        labels.extend(axis_labels)
    if handles:
        axes[0].legend(handles, labels, loc="best")

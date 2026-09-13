"""绘制通用抓取实验的控制 trace，并兼容历史 cup 记录。

图只呈现记录中实际存在的控制量、触觉量与估计诊断；刚度估计在
未有效确认的区间留空，不伪造曲线。离线重绘写入独占重绘目录，
保留源 trace 与原 manifest。
"""

from __future__ import annotations

import csv
import json
import math
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def plot_experiment_run(
    directory: Path, *, output_directory: Path | None = None
) -> tuple[Path, ...]:
    """从通用抓取实验 trace 生成力控诊断图。

    Args:
        directory: 包含 ``trace.csv`` 的实验输出目录。
        output_directory: 离线重绘的独占输出目录；``None`` 时在源目录
            内生成 ``plot.pdf``／``plot.png``。

    Returns:
        成功生成时的 PDF 与 PNG 路径；不存在或没有数据的 trace 返回空元组。
    """
    directory = Path(directory)
    rows = _read_rows(directory / "trace.csv")
    if not rows:
        return ()
    time_s = _values(rows, "time_s")
    if not any(math.isfinite(value) for value in time_s):
        return ()
    has_stiffness = any(math.isfinite(value) for value in _values(rows, "stiffness_n_per_m"))
    panels = 4 if has_stiffness else 3
    plt, style_context = _plotting_api()
    with style_context():
        figure, axes = plt.subplots(
            panels, 1, sharex=True, figsize=(7.16, 2.2 * panels), layout="constrained"
        )
        if panels == 3:
            axes = [axes[0], axes[1], axes[2]]
        _plot_normal_force(axes[0], time_s, rows)
        _plot_tangential_force(axes[1], time_s, rows)
        _plot_joint_command(axes[2], time_s, rows)
        if has_stiffness:
            _plot_stiffness(axes[3], time_s, rows)
            axes[3].set_xlabel("Time (s)")
        else:
            axes[2].set_xlabel("Time (s)")
        for axis in axes:
            axis.grid(True, alpha=0.25)
        target = Path(output_directory) if output_directory is not None else directory
        target.mkdir(parents=True, exist_ok=True)
        pdf_path = target / "plot.pdf"
        png_path = target / "plot.png"
        figure.savefig(pdf_path, format="pdf")
        figure.savefig(png_path, format="png", dpi=600)
        plt.close(figure)
    if output_directory is None:
        _register_plot_files(directory, (pdf_path, png_path))
    else:
        _write_repaint_manifest(output_directory, directory, (pdf_path, png_path))
    return (pdf_path, png_path)


def repaint_run(directory: Path) -> Path:
    """对历史或既有运行目录执行离线重绘，返回独占重绘目录。

    Args:
        directory: 包含 ``trace.csv`` 的源运行目录；源文件不会被修改。

    Returns:
        新创建的重绘目录（``repaint-<UTC时间戳>-<id>``）。

    Raises:
        ValueError: 源目录没有可绘制数据。
    """
    directory = Path(directory)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = directory / f"repaint-{stamp}-{uuid.uuid4().hex[:8]}"
    if not plot_experiment_run(directory, output_directory=output):
        raise ValueError(f"源目录没有可绘制数据：{directory}")
    return output


def read_trace_rows(directory: Path) -> list[dict[str, str]]:
    """读取运行目录的控制 trace 行；供离线分析复用。"""
    return _read_rows(Path(directory) / "trace.csv")


def _read_rows(trace_path: Path) -> list[dict[str, str]]:
    """读取非空 CSV 行；损坏或缺失文件视为没有可绘制数据。

    历史兼容：新 schema 使用 ``phase`` 列；旧 cup trace 使用 ``state``，
    缺失 ``phase`` 时回填同值。
    """
    if not trace_path.is_file() or trace_path.stat().st_size == 0:
        return []
    try:
        with trace_path.open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
    except (csv.Error, OSError, UnicodeError):
        return []
    for row in rows:
        if not row.get("phase") and row.get("state"):
            row["phase"] = row["state"]
    return rows


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
    trigger = _values(rows, "target_trigger_active", boolean=True)
    if not any(math.isfinite(value) for value in trigger):
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


def _plot_stiffness(axis: Any, time_s: list[float], rows: list[dict[str, str]]) -> None:
    """绘制等效接触刚度估计；未有效确认的区间留空。"""
    values = _values(rows, "stiffness_n_per_m")
    valid = _values(rows, "stiffness_valid", boolean=True)
    masked = [
        value if math.isfinite(valid_flag) and valid_flag > 0.5 else math.nan
        for value, valid_flag in zip(values, valid, strict=True)
    ]
    _line_if_data(axis, time_s, masked, "Estimate (valid)", "C0")
    axis.set_ylabel("Stiffness (N/m)")
    _legend(axis)


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


def _register_plot_files(directory: Path, plots: tuple[Path, ...]) -> None:
    """把生成的图登记进源目录 manifest（兼容新 dict 与旧 list 格式）。"""
    manifest_path = directory / "manifest.json"
    if not manifest_path.is_file():
        return
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return
    files = manifest.get("files")
    if isinstance(files, list):
        manifest["files"] = list(dict.fromkeys([*files, *(path.name for path in plots)]))
    elif isinstance(files, dict):
        for path in plots:
            files[path.name] = path.name
    else:
        manifest["files"] = {path.name: path.name for path in plots}
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def _write_repaint_manifest(
    output_directory: Path,
    source_directory: Path,
    plots: tuple[Path, ...],
) -> None:
    """为离线重绘目录写入独立 manifest，指向源 trace。"""
    manifest = {
        "schema": "dmgripper-experiment/repaint/v1",
        "source_directory": str(source_directory),
        "source_trace": str(source_directory / "trace.csv"),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "files": {path.name: path.name for path in plots},
    }
    (output_directory / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

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

from .lifecycle import LifecyclePhase


def plot_experiment_run(
    directory: Path,
    *,
    output_directory: Path | None = None,
    phases: tuple[str, ...] | None = None,
    task_time_range: tuple[float, float] | None = None,
) -> tuple[Path, ...]:
    """从通用抓取实验 trace 生成力控诊断图。

    Args:
        directory: 包含 ``trace.csv`` 的实验输出目录。
        output_directory: 离线重绘的独占输出目录；``None`` 时在源目录
            内生成 ``plot.pdf``／``plot.png``。
        phases: 可选阶段白名单，仅绘制这些阶段的行。
        task_time_range: 可选任务时间窗口 ``[起, 止]``；仅保留有限
            ``task_time_s`` 的行，横轴切换为任务时间。

    Returns:
        成功生成时的 PDF 与 PNG 路径；不存在、没有数据或窗口过滤后
        为空的 trace 返回空元组。

    Raises:
        ValueError: 阶段名未知，或任务时间窗口不是 ``0 ≤ 起 ≤ 止`` 的有限区间。
    """
    directory = Path(directory)
    rows = _read_rows(directory / "trace.csv")
    if not rows:
        return ()
    rows, x_field = _select_window(rows, phases, task_time_range)
    if not rows:
        return ()
    time_s = _values(rows, x_field)
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
            axes[3].set_xlabel(_x_label(x_field))
        else:
            axes[2].set_xlabel(_x_label(x_field))
        for axis in axes:
            axis.grid(True, alpha=0.25)
        target = Path(output_directory) if output_directory is not None else directory
        target.mkdir(parents=True, exist_ok=True)
        pdf_path = target / "plot.pdf"
        png_path = target / "plot.png"
        figure.savefig(pdf_path, format="pdf")
        figure.savefig(png_path, format="png", dpi=600)
        plt.close(figure)
    window = _window_description(phases, task_time_range)
    if output_directory is None:
        _register_plot_files(directory, (pdf_path, png_path))
    else:
        _write_repaint_manifest(output_directory, directory, (pdf_path, png_path), window)
    return (pdf_path, png_path)


def repaint_run(
    directory: Path,
    *,
    phases: tuple[str, ...] | None = None,
    task_time_range: tuple[float, float] | None = None,
) -> Path:
    """对历史或既有运行目录执行离线重绘，返回独占重绘目录。

    Args:
        directory: 包含 ``trace.csv`` 的源运行目录；源文件不会被修改。
        phases: 可选阶段白名单，透传给绘图。
        task_time_range: 可选任务时间窗口，透传给绘图。

    Returns:
        新创建的重绘目录（``repaint-<UTC时间戳>-<id>``）。

    Raises:
        ValueError: 源目录没有可绘制数据，或窗口参数非法。
    """
    directory = Path(directory)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = directory / f"repaint-{stamp}-{uuid.uuid4().hex[:8]}"
    if not plot_experiment_run(
        directory, output_directory=output, phases=phases, task_time_range=task_time_range
    ):
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
        # 覆盖 SciencePlots 的 cm 数学字体：cm*.ttf 的 head 时间戳过旧，
        # PDF 嵌入时 fontTools 会对每个字体打两条无害但扰人的告警；
        # STIX 同为衬线数学字体，观感与 cm 几乎一致且时间戳合法。
        "mathtext.fontset": "stix",
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
    _line_if_data(axis, time_s, measured, r"$F_z$", "C0")
    _line_if_data(
        axis,
        time_s,
        _values(rows, "target_force_n"),
        r"$F_z^{\mathrm{ref}}$",
        "C3",
        linestyle="--",
    )
    axis.set_ylabel(r"$F_z$ (N)")
    _legend(axis)


def _plot_tangential_force(axis: Any, time_s: list[float], rows: list[dict[str, str]]) -> None:
    """绘制左右触觉切向力模和触发状态。"""
    left = _magnitude(_values(rows, "raw_left_fx_n"), _values(rows, "raw_left_fy_n"))
    right = _magnitude(_values(rows, "raw_right_fx_n"), _values(rows, "raw_right_fy_n"))
    _line_if_data(axis, time_s, left, "Left", "C0")
    _line_if_data(axis, time_s, right, "Right", "C1")
    axis.set_ylabel(r"$F_T$ (N)")
    trigger = _values(rows, "target_trigger_active", boolean=True)
    if not any(math.isfinite(value) for value in trigger):
        trigger = _values(rows, "trigger_active", boolean=True)
    if any(math.isfinite(value) for value in trigger):
        active = [math.isfinite(value) and value > 0.5 for value in trigger]
        axis.fill_between(
            time_s,
            0.0,
            1.0,
            where=active,
            step="post",
            transform=axis.get_xaxis_transform(),
            color="#009E73",
            alpha=0.14,
            linewidth=0.0,
            zorder=0.1,
            label="Trigger",
        )
    _legend(axis)


def _plot_joint_command(axis: Any, time_s: list[float], rows: list[dict[str, str]]) -> None:
    """绘制关节位置与电机力矩，并明确标注各自单位。"""
    _line_if_data(axis, time_s, _values(rows, "position_rad"), r"$q$", "C0")
    _line_if_data(axis, time_s, _values(rows, "q_des_rad"), r"$q_d$", "C2", linestyle="--")
    axis.set_ylabel(r"$q$ (rad)")
    torque_axis = axis.twinx()
    _line_if_data(
        torque_axis, time_s, _values(rows, "torque_nm"), r"$\tau_m$", "#CC79A7", linestyle=":"
    )
    torque_axis.set_ylabel(r"$\tau_m$ ($\mathrm{N\,m}$)")
    _legend(axis, torque_axis)


def _plot_stiffness(axis: Any, time_s: list[float], rows: list[dict[str, str]]) -> None:
    """绘制等效接触刚度估计；未有效确认的区间留空。"""
    values = _values(rows, "stiffness_n_per_m")
    valid = _values(rows, "stiffness_valid", boolean=True)
    masked = [
        value if math.isfinite(valid_flag) and valid_flag > 0.5 else math.nan
        for value, valid_flag in zip(values, valid, strict=True)
    ]
    _line_if_data(axis, time_s, masked, r"$\hat{K}$", "C0")
    axis.set_ylabel(r"$K$ ($\mathrm{N\,m^{-1}}$)")
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


def _select_window(
    rows: list[dict[str, str]],
    phases: tuple[str, ...] | None,
    task_time_range: tuple[float, float] | None,
) -> tuple[list[dict[str, str]], str]:
    """按阶段白名单与任务时间窗口截取行，返回剩余行与横轴字段名。

    任务时间窗口天然只保留 active 及其后冻结任务时钟的行；此时横轴
    切换为 ``task_time_s``，使跨运行的评价图共用曲线起点为零的坐标。
    """
    if phases is not None:
        valid = {phase.value for phase in LifecyclePhase}
        unknown = sorted(set(phases) - valid)
        if unknown:
            raise ValueError(f"未知阶段 {unknown}；可用阶段：{sorted(valid)}")
        wanted = set(phases)
        rows = [row for row in rows if row.get("phase") in wanted]
    if task_time_range is not None:
        start_s, end_s = task_time_range
        if not math.isfinite(start_s) or not math.isfinite(end_s) or not 0.0 <= start_s <= end_s:
            raise ValueError("任务时间窗口必须是 0 ≤ 起 ≤ 止 的有限区间")
        # approach 等阶段把未启动的任务时钟写成 0.0，不属于任务时间轴；
        # 只保留任务时钟推进或冻结的行，避免接触前数据堆在 t=0。
        rows = [
            row
            for row in rows
            if row.get("phase") in _TASK_CLOCK_PHASES
            and math.isfinite(value := _parse_number(row.get("task_time_s", "")))
            and start_s - 1e-9 <= value <= end_s + 1e-9
        ]
    return rows, "task_time_s" if task_time_range is not None else "time_s"


_TASK_CLOCK_PHASES = frozenset(
    {
        LifecyclePhase.ACTIVE.value,
        LifecyclePhase.HOLDING.value,
        LifecyclePhase.RETURNING.value,
    }
)


def _x_label(x_field: str) -> str:
    """按横轴字段给出物理量与单位。"""
    return r"$t_{\mathrm{task}}$ (s)" if x_field == "task_time_s" else r"$t$ (s)"


def _window_description(
    phases: tuple[str, ...] | None,
    task_time_range: tuple[float, float] | None,
) -> dict[str, object] | None:
    """把生效的绘图窗口整理为可追溯的 manifest 字段。"""
    if phases is None and task_time_range is None:
        return None
    window: dict[str, object] = {}
    if phases is not None:
        window["phases"] = list(phases)
    if task_time_range is not None:
        window["task_time_s"] = [task_time_range[0], task_time_range[1]]
    return window


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
    window: dict[str, object] | None = None,
) -> None:
    """为离线重绘目录写入独立 manifest，指向源 trace。"""
    manifest = {
        "schema": "dmgripper-experiment/repaint/v1",
        "source_directory": str(source_directory),
        "source_trace": str(source_directory / "trace.csv"),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "files": {path.name: path.name for path in plots},
    }
    if window is not None:
        manifest["window"] = window
    (output_directory / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

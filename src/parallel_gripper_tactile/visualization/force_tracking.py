"""DMgripper 力跟踪运行产物的独立论文绘图入口。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from pathlib import Path
import re
from typing import Any, Literal

import numpy as np

from .plotstyle import (
    FULL_WIDTH_FONT_SCALE,
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)


_COLORS = {
    "reference": "#000000",
    "feedback": "#0072B2",
    "measured": "#D55E00",
    "left": "#0072B2",
    "right": "#D55E00",
    "diagnostic": "#009E73",
    "muted": "#666666",
}
_TAXEL_FIELD = re.compile(r"^(left|right)_taxel_(fx|fy|fz)_([0-9]+)_([0-9]+)$")


def _as_float(value: object) -> float:
    """把标量转换为有限数值或 NaN，避免把缺失量补成零。"""
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _values(trace: Sequence[Mapping[str, object]], field: str) -> np.ndarray:
    """读取一个 trace 字段，缺失行保留为 NaN。"""
    return np.asarray([_as_float(row.get(field)) for row in trace], dtype=float)


def _finite_mask(values: np.ndarray) -> np.ndarray:
    """返回可安全绘制的有限数值掩码。"""
    return np.isfinite(values)


def _has_values(values: np.ndarray) -> bool:
    """判断序列是否包含至少一个有效样本。"""
    return bool(_finite_mask(values).any())


def _as_bool(value: object) -> bool:
    """兼容内存／Parquet 布尔值与 CSV 序列化后的 ``true`` 字符串。"""
    return value is True or (isinstance(value, str) and value.lower() == "true")


def _mapping_at(mapping: Mapping[str, object] | None, *keys: str) -> Mapping[str, object]:
    """安全读取嵌套映射；历史快照缺字段时返回空映射。"""
    current: object = mapping or {}
    for key in keys:
        if not isinstance(current, Mapping):
            return {}
        current = current.get(key, {})
    return current if isinstance(current, Mapping) else {}


def _profile_config(config: Mapping[str, object] | None) -> Mapping[str, object]:
    """兼容 effective 参数和仅含 profile/task 的历史配置。"""
    if not config:
        return {}
    profile = config.get("profile")
    if isinstance(profile, Mapping):
        return profile
    return config


def _task_config(config: Mapping[str, object] | None) -> Mapping[str, object]:
    """返回任务定义；兼容 Hydra 的 ``definition`` 包装。"""
    task = _mapping_at(config, "task")
    definition = task.get("definition")
    return definition if isinstance(definition, Mapping) else task


def _runtime_config(config: Mapping[str, object] | None) -> Mapping[str, object]:
    """返回 effective 参数中的运行时覆盖，历史输入缺失时为空。"""
    return _mapping_at(config, "runtime")


def _time_axis(
    trace: Sequence[Mapping[str, object]],
    *,
    prefer_control_time: bool = False,
) -> np.ndarray:
    """选择与信号采样语义相符的时间轴，并回退到物理步后时间。"""
    names = ("control_time_s", "command_time_s", "time_s") if prefer_control_time else ("time_s",)
    for name in names:
        values = _values(trace, name)
        if _has_values(values):
            fallback = _values(trace, "time_s")
            return np.where(_finite_mask(values), values, fallback)
    return np.full(len(trace), np.nan, dtype=float)


def _command_time_axis(trace: Sequence[Mapping[str, object]]) -> np.ndarray:
    """选择 MIT 命令采样的步前时间，并兼容未记录该字段的历史 trace。"""
    for name in ("command_time_s", "time_s"):
        values = _values(trace, name)
        if _has_values(values):
            fallback = _values(trace, "time_s")
            return np.where(_finite_mask(values), values, fallback)
    return np.full(len(trace), np.nan, dtype=float)


def _tracking_start_time(
    trace: Sequence[Mapping[str, object]], metrics: Mapping[str, object] | None
) -> float | None:
    """从指标或 trace 确定参考跟踪开始的绝对时间。"""
    metric_time = _as_float((metrics or {}).get("tracking_start_time_s"))
    if math.isfinite(metric_time):
        return metric_time
    reference_start = _values(trace, "reference_start_time_s")
    if _has_values(reference_start):
        return float(reference_start[np.flatnonzero(_finite_mask(reference_start))[0]])
    physical_time = _values(trace, "time_s")
    tracking_time = _values(trace, "tracking_time_s")
    in_tracking = np.asarray([row.get("phase") == "track_reference" for row in trace], dtype=bool)
    valid = _finite_mask(physical_time) & _finite_mask(tracking_time) & in_tracking
    if valid.any():
        return float(
            physical_time[np.flatnonzero(valid)[0]] - tracking_time[np.flatnonzero(valid)[0]]
        )
    for row in trace:
        if row.get("phase") == "track_reference":
            value = _as_float(row.get("time_s"))
            return value if math.isfinite(value) else None
    return None


def _waypoint_times(
    task: Mapping[str, object], tracking_start_s: float | None
) -> list[tuple[float, float]]:
    """返回线性任务 waypoint 的真实绝对时间与目标力。"""
    reference = task.get("reference")
    if tracking_start_s is None or not isinstance(reference, Mapping):
        return []
    if reference.get("interpolation") != "linear":
        return []
    waypoints = reference.get("waypoints")
    if not isinstance(waypoints, Sequence) or isinstance(waypoints, (str, bytes)):
        return []
    result: list[tuple[float, float]] = []
    for waypoint in waypoints:
        if not isinstance(waypoint, Mapping):
            continue
        time_s = _as_float(waypoint.get("t_s"))
        force_n = _as_float(waypoint.get("force_n"))
        if math.isfinite(time_s) and math.isfinite(force_n):
            result.append((tracking_start_s + time_s, force_n))
    return result


def _plot_series(axis: Any, times: np.ndarray, values: np.ndarray, **kwargs: object) -> None:
    """保留 NaN 断点，只过滤无意义的时间坐标。"""
    valid_time = _finite_mask(times)
    if valid_time.any() and _has_values(values[valid_time]):
        axis.plot(times[valid_time], values[valid_time], **kwargs)


def _metric_annotation(metrics: Mapping[str, object] | None) -> str | None:
    """把已有指标整理为紧凑的图内数学标注，不重新统计 trace。"""
    if not metrics:
        return None
    symbols = (
        ("rmse_n", r"\mathrm{RMSE}"),
        ("mae_n", r"\mathrm{MAE}"),
        ("peak_abs_error_n", r"|e_F|_{\max}"),
    )
    terms = [
        rf"{symbol}={value:.3g}\,\mathrm{{N}}"
        for field, symbol in symbols
        if math.isfinite(value := _as_float(metrics.get(field)))
    ]
    if not terms:
        return None
    lines = [r",\;".join(terms[:2])]
    if len(terms) > 2:
        lines.append(terms[2])
    return "\n".join(f"${line}$" for line in lines)


def _stiffness_enabled(config: Mapping[str, object] | None) -> bool:
    """仅在配置明确启用刚度估计时允许显示其诊断量。"""
    stiffness = _mapping_at(_profile_config(config), "control", "force", "stiffness")
    return _as_bool(stiffness.get("enabled"))


def _stiffness_mask(
    trace: Sequence[Mapping[str, object]], config: Mapping[str, object] | None
) -> np.ndarray:
    """返回同时满足已启用、显式有效与数值有限的刚度掩码。"""
    values = _values(trace, "estimated_contact_stiffness_n_per_m")
    if not _stiffness_enabled(config) or not any("stiffness_valid" in row for row in trace):
        return np.zeros(len(trace), dtype=bool)
    valid = np.asarray([_as_bool(row.get("stiffness_valid")) for row in trace], dtype=bool)
    return valid & _finite_mask(values)


def _contact_confirmation_index(trace: Sequence[Mapping[str, object]]) -> int | None:
    """从阶段机的明确状态获取接触确认时刻，不由非零触觉力反推。"""
    for index, row in enumerate(trace):
        if (
            row.get("control_state") in {"contact_transition", "force_tracking"}
            or row.get("phase") == "track_reference"
        ):
            return index
    return None


def _events(trace: Sequence[Mapping[str, object]], times: np.ndarray) -> list[tuple[float, str]]:
    """从状态机边沿提取少量可解释事件。"""
    events: list[tuple[float, str]] = []
    previous_state: object = object()
    previous_phase: object = object()
    emitted: set[str] = set()
    for index, row in enumerate(trace):
        time_s = times[index]
        if not math.isfinite(time_s):
            continue
        state = row.get("control_state")
        phase = row.get("phase")
        if (
            state == "contact_transition"
            and state != previous_state
            and "Contact confirmed" not in emitted
        ):
            events.append((float(time_s), "Contact confirmed"))
            emitted.add("Contact confirmed")
        elif (
            state == "force_tracking"
            and state != previous_state
            and "Contact confirmed" not in emitted
        ):
            events.append((float(time_s), "Contact confirmed"))
            emitted.add("Contact confirmed")
        if (
            index > 0
            and state == "approach"
            and previous_state != "approach"
            and "Reapproach" not in emitted
        ):
            events.append((float(time_s), "Reapproach"))
            emitted.add("Reapproach")
        if (
            phase == "track_reference"
            and phase != previous_phase
            and "Tracking start" not in emitted
        ):
            reference_start = _as_float(row.get("reference_start_time_s"))
            events.append(
                (
                    float(reference_start) if math.isfinite(reference_start) else float(time_s),
                    "Tracking start",
                )
            )
            emitted.add("Tracking start")
        previous_state, previous_phase = state, phase
    return events


def _add_events(axis: Any, events: Sequence[tuple[float, str]], *, labels: bool = False) -> None:
    """用轻量竖线标记实际状态切换，不引入推断事件。"""
    for time_s, label in events:
        axis.axvline(time_s, color=_COLORS["muted"], ls=":", lw=0.75, alpha=0.65)
        if labels:
            axis.annotate(
                label,
                xy=(time_s, 1),
                xycoords=("data", "axes fraction"),
                xytext=(2, -3),
                textcoords="offset points",
                rotation=90,
                va="top",
                fontsize=7,
                color=_COLORS["muted"],
            )


def _controller_annotation(config: Mapping[str, object] | None) -> str | None:
    """提取外环、MIT、ADRC 或导纳的关键固定参数。"""
    profile = _profile_config(config)
    control = _mapping_at(profile, "control")
    force = _mapping_at(control, "force")
    mit = _mapping_at(control, "mit")
    torque_adrc = _mapping_at(force, "torque_adrc")
    first_order_adrc = _mapping_at(force, "adrc")
    admittance = _mapping_at(force, "admittance")
    mit_terms = [
        rf"{symbol}={value:.3g}"
        for field, symbol in (("kp", r"k_p^{\mathrm{MIT}}"), ("kd", r"k_d^{\mathrm{MIT}}"))
        if math.isfinite(value := _as_float(mit.get(field)))
    ]
    if admittance:
        units = {
            "mass_kg": r"\mathrm{kg}",
            "damping_ns_m": r"\mathrm{N\,s\,m^{-1}}",
            "stiffness_n_m": r"\mathrm{N\,m^{-1}}",
        }
        terms = [
            rf"{symbol}={value:.3g}\,{units[field]}"
            for field, symbol in (
                ("mass_kg", "M_a"),
                ("damping_ns_m", "B_a"),
                ("stiffness_n_m", "K_a"),
            )
            if math.isfinite(value := _as_float(admittance.get(field)))
        ]
        lines = ["$" + r",\;".join(terms) + "$" for terms in (terms, mit_terms) if terms]
        return "\n".join(lines) if lines else None
    adrc = torque_adrc or first_order_adrc
    if adrc:
        terms = [
            rf"{symbol}={value:.3g}\,\mathrm{{rad\,s^{{-1}}}}"
            for field, symbol in (
                ("controller_bandwidth_rad_s", r"\omega_c"),
                ("observer_bandwidth_rad_s", r"\omega_o"),
            )
            if math.isfinite(value := _as_float(adrc.get(field)))
        ]
        return "$" + r",\;".join(terms) + "$" if terms else None
    torque_feedback = _as_float(force.get("torque_feedback_gain"))
    if math.isfinite(torque_feedback) and torque_feedback > 0:
        return "$" + r",\;".join([rf"K_{{\tau}}={torque_feedback:.3g}", *mit_terms]) + "$"
    pid_terms = [
        rf"{symbol}={value:.3g}"
        for field, symbol in (("kp", "K_P"), ("ki", "K_I"), ("kd", "K_D"))
        if math.isfinite(value := _as_float(force.get(field)))
    ]
    lines = ["$" + r",\;".join(terms) + "$" for terms in (pid_terms, mit_terms) if terms]
    return "\n".join(lines) if lines else None


def _render_tracking_figure(
    trace: Sequence[Mapping[str, object]],
    *,
    metrics: Mapping[str, object] | None,
    config: Mapping[str, object] | None,
) -> Any | None:
    """构建单栏法向力跟踪图，返回缺少必要信号时的 ``None``。"""
    target = _values(trace, "target_normal_force_n")
    filtered = _values(trace, "filtered_normal_force_n")
    measured = _values(trace, "measured_normal_force_n")
    feedback = filtered if _has_values(filtered) else measured
    if not _has_values(target) or not _has_values(feedback):
        return None
    times = _time_axis(trace, prefer_control_time=True)
    if not _has_values(times):
        return None
    plt = science_pyplot()
    figure, axis = plt.subplots(figsize=paper_figsize(2.65, columns=1), layout="constrained")
    _plot_series(
        axis, times, target, color=_COLORS["reference"], ls="--", label=r"$F_{\mathrm{ref}}$"
    )
    feedback_label = r"$F_{\mathrm{filt}}$" if feedback is filtered else r"$F_{\mathrm{meas}}$"
    _plot_series(axis, times, feedback, color=_COLORS["feedback"], label=feedback_label)
    start_s = _tracking_start_time(trace, metrics)
    for waypoint_time_s, force_n in _waypoint_times(_task_config(config), start_s):
        axis.plot(
            waypoint_time_s,
            force_n,
            marker="o",
            ms=4.5,
            mfc="white",
            mec=_COLORS["reference"],
            mew=0.9,
            zorder=4,
        )
    _add_events(axis, _events(trace, times))
    annotation = _metric_annotation(metrics)
    if annotation:
        axis.set_title(annotation, loc="left", fontsize=7.5, pad=4)
    semantics = _runtime_config(config).get("force_semantics")
    if semantics is None:
        semantics = next(
            (row.get("force_semantics") for row in trace if row.get("force_semantics")), None
        )
    force_name = r"F_{n,\Sigma}" if semantics == "total" else r"F_n"
    axis.set_ylabel(rf"${force_name}\,(\mathrm{{N}})$")
    axis.set_xlabel(r"$t\,(\mathrm{s})$")
    axis.legend(loc="best", ncol=2, frameon=False)
    axis.grid(alpha=0.18)
    return figure


def _render_tactile_figure(trace: Sequence[Mapping[str, object]]) -> Any | None:
    """构建左右法向力及切向合力图。"""
    times = _time_axis(trace)
    if not _has_values(times):
        return None
    left_normal, right_normal = (
        _values(trace, "measured_left_fz"),
        _values(trace, "measured_right_fz"),
    )
    left_fx, left_fy = _values(trace, "measured_left_fx"), _values(trace, "measured_left_fy")
    right_fx, right_fy = _values(trace, "measured_right_fx"), _values(trace, "measured_right_fy")
    left_tangent = _values(trace, "left_tangential_force_n")
    right_tangent = _values(trace, "right_tangential_force_n")
    if not _has_values(left_tangent):
        left_tangent = np.hypot(left_fx, left_fy)
    if not _has_values(right_tangent):
        right_tangent = np.hypot(right_fx, right_fy)
    normal_available = _has_values(left_normal) or _has_values(right_normal)
    tangent_available = _has_values(left_tangent) or _has_values(right_tangent)
    if not normal_available and not tangent_available:
        return None
    panels = []
    if normal_available:
        panels.append(
            (left_normal, right_normal, r"$F_n\,(\mathrm{N})$", r"$F_{n,L}$", r"$F_{n,R}$")
        )
    if tangent_available:
        panels.append(
            (left_tangent, right_tangent, r"$F_t\,(\mathrm{N})$", r"$F_{t,L}$", r"$F_{t,R}$")
        )
    plt = science_pyplot()
    figure, axes = plt.subplots(
        len(panels),
        1,
        sharex=True,
        figsize=paper_figsize(1.85 * len(panels) + 0.45, columns=1),
        layout="constrained",
    )
    axes = np.atleast_1d(axes)
    for axis, (left, right, ylabel, left_label, right_label) in zip(axes, panels, strict=True):
        _plot_series(axis, times, left, color=_COLORS["left"], label=left_label)
        _plot_series(axis, times, right, color=_COLORS["right"], ls="--", label=right_label)
        axis.set_ylabel(ylabel)
        axis.legend(loc="best", ncol=2, frameon=False)
        axis.grid(alpha=0.18)
    axes[-1].set_xlabel(r"$t\,(\mathrm{s})$")
    return figure


def _append_panel(
    panels: list[tuple[str, Any]], name: str, values: np.ndarray, **details: object
) -> None:
    """只把存在有效样本的诊断量加入控制器图。"""
    if _has_values(values):
        panels.append((name, {"values": values, **details}))


def _render_controller_figure(
    trace: Sequence[Mapping[str, object]], *, config: Mapping[str, object] | None
) -> Any | None:
    """构建位置、速度、力矩与已确认诊断量的跨栏图。"""
    control_time = _time_axis(trace, prefer_control_time=True)
    command_time = _command_time_axis(trace)
    physical_time = _time_axis(trace)
    if not any(_has_values(values) for values in (control_time, command_time, physical_time)):
        return None
    panels: list[tuple[str, Any]] = []
    target_position = _values(trace, "desired_position_rad")
    if not _has_values(target_position):
        target_position = _values(trace, "control")
    drive_position = _values(trace, "drive_position_rad")
    if _has_values(target_position) or _has_values(drive_position):
        panels.append(("position", {"target": target_position, "drive": drive_position}))
    desired_velocity = _values(trace, "desired_velocity_rad_s")
    drive_velocity = _values(trace, "drive_velocity_rad_s")
    if _has_values(desired_velocity) or _has_values(drive_velocity):
        panels.append(("velocity", {"target": desired_velocity, "drive": drive_velocity}))
    commanded = _values(trace, "commanded_torque_n_m")
    if not _has_values(commanded):
        commanded = _values(trace, "motor_torque_n_m")
    actuator = _values(trace, "actuator_torque_n_m")
    mit_ff = _values(trace, "mit_feedforward_torque_n_m")
    force_ff = _values(trace, "force_feedforward_torque_n_m")
    if any(_has_values(values) for values in (commanded, actuator, mit_ff, force_ff)):
        panels.append(
            (
                "torque",
                {
                    "commanded": commanded,
                    "actuator": actuator,
                    "mit_ff": mit_ff,
                    "force_ff": force_ff,
                },
            )
        )
    stiffness = _values(trace, "estimated_contact_stiffness_n_per_m")
    stiffness_valid = _stiffness_mask(trace, config)
    if stiffness_valid.any():
        panels.append(("stiffness", {"values": np.where(stiffness_valid, stiffness, np.nan)}))
    _append_panel(
        panels,
        "admittance_displacement",
        _values(trace, "admittance_displacement_m"),
    )
    _append_panel(
        panels,
        "admittance_velocity",
        _values(trace, "admittance_velocity_m_s"),
    )
    _append_panel(
        panels,
        "adrc_disturbance",
        _values(trace, "torque_adrc_estimated_disturbance_n_s2"),
    )
    if not panels:
        return None
    plt = science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)
    figure, axes = plt.subplots(
        len(panels),
        1,
        sharex=True,
        figsize=paper_figsize(1.55 * len(panels) + 0.45, columns=2),
        layout="constrained",
    )
    axes = np.atleast_1d(axes)
    events = _events(trace, control_time)
    for axis, (kind, content) in zip(axes, panels, strict=True):
        if kind == "position":
            _plot_series(
                axis,
                command_time,
                content["target"],
                color=_COLORS["reference"],
                ls="--",
                label=r"$q_{\mathrm{des}}$",
            )
            _plot_series(
                axis, command_time, content["drive"], color=_COLORS["feedback"], label=r"$q$"
            )
            axis.set_ylabel(r"$q\,(\mathrm{rad})$")
        elif kind == "velocity":
            _plot_series(
                axis,
                command_time,
                content["target"],
                color=_COLORS["reference"],
                ls="--",
                label=r"$\dot q_{\mathrm{des}}$",
            )
            _plot_series(
                axis, command_time, content["drive"], color=_COLORS["feedback"], label=r"$\dot q$"
            )
            axis.set_ylabel(r"$\dot q\,(\mathrm{rad\,s^{-1}})$")
        elif kind == "torque":
            _plot_series(
                axis,
                command_time,
                content["commanded"],
                color=_COLORS["feedback"],
                label=r"$\tau_{\mathrm{cmd}}$",
            )
            _plot_series(
                axis,
                physical_time,
                content["actuator"],
                color=_COLORS["diagnostic"],
                label=r"$\tau_{\mathrm{act}}$",
            )
            if not _has_values(content["commanded"]) and not _has_values(content["actuator"]):
                _plot_series(
                    axis,
                    command_time,
                    content["mit_ff"],
                    color=_COLORS["reference"],
                    ls=":",
                    label=r"$\tau_{\mathrm{ff}}^{\mathrm{MIT}}$",
                )
                _plot_series(
                    axis,
                    command_time,
                    content["force_ff"],
                    color=_COLORS["right"],
                    ls="-.",
                    label=r"$\tau_{\mathrm{ff}}^{F}$",
                )
            axis.set_ylabel(r"$\tau\,(\mathrm{N\,m})$")
        elif kind == "stiffness":
            _plot_series(
                axis,
                control_time,
                content["values"],
                color=_COLORS["diagnostic"],
                label=r"$\widehat K_c$",
            )
            axis.set_ylabel(r"$\widehat K_c\,(\mathrm{N\,m^{-1}})$")
        elif kind == "admittance_displacement":
            _plot_series(
                axis, control_time, content["values"], color=_COLORS["diagnostic"], label=r"$x_a$"
            )
            axis.set_ylabel(r"$x_a\,(\mathrm{m})$")
        elif kind == "admittance_velocity":
            _plot_series(
                axis,
                control_time,
                content["values"],
                color=_COLORS["diagnostic"],
                label=r"$\dot x_a$",
            )
            axis.set_ylabel(r"$\dot x_a\,(\mathrm{m\,s^{-1}})$")
        elif kind == "adrc_disturbance":
            _plot_series(
                axis,
                control_time,
                content["values"],
                color=_COLORS["diagnostic"],
                label=r"$\widehat d$",
            )
            axis.set_ylabel(r"$\widehat d\,(\mathrm{N\,s^{-2}})$")
        else:
            _plot_series(
                axis,
                control_time,
                content["pid"],
                color=_COLORS["feedback"],
                label=r"$\Delta q_{\mathrm{PID}}$",
            )
            _plot_series(
                axis,
                control_time,
                content["stiffness"],
                color=_COLORS["right"],
                label=r"$\Delta q_K$",
            )
            axis.set_ylabel(r"$\Delta q\,(\mathrm{rad})$")
        _add_events(axis, events, labels=axis is axes[0])
        axis.grid(alpha=0.18)
        handles, labels = axis.get_legend_handles_labels()
        if handles:
            axis.legend(loc="best", ncol=min(4, len(handles)), frameon=False)
    annotation = _controller_annotation(config)
    if annotation:
        axes[0].set_title(annotation, loc="left", fontsize=7.5, pad=4)
    axes[-1].set_xlabel(r"$t\,(\mathrm{s})$")
    return figure


def _render_taxel_figure(
    trace: Sequence[Mapping[str, object]], side: str, config: Mapping[str, object] | None
) -> Any | None:
    """在明确接触确认后的区间绘制单侧 3×3 taxel 三分量时序。"""
    start = _contact_confirmation_index(trace)
    if start is None:
        return None
    tactile = _mapping_at(_profile_config(config), "tactile")
    rows, columns = _as_float(tactile.get("rows")), _as_float(tactile.get("cols"))
    if math.isfinite(rows) and math.isfinite(columns) and (rows, columns) != (3.0, 3.0):
        return None
    fields: dict[str, dict[tuple[int, int], str]] = {"fx": {}, "fy": {}, "fz": {}}
    for key in {key for row in trace for key in row}:
        match = _TAXEL_FIELD.match(key)
        if match is not None and match.group(1) == side:
            fields[match.group(2)][(int(match.group(3)), int(match.group(4)))] = key
    coordinates = [(row, column) for row in range(3) for column in range(3)]
    if any(set(component) != set(coordinates) for component in fields.values()):
        return None
    times = _time_axis(trace)[start:]
    if not _has_values(times):
        return None
    values = {
        component: {
            coordinate: _values(trace, field)[start:]
            for coordinate, field in component_fields.items()
        }
        for component, component_fields in fields.items()
    }
    if not all(
        _has_values(series) for component in values.values() for series in component.values()
    ):
        return None
    shear_scale = max(
        (
            abs(float(value))
            for component in ("fx", "fy")
            for series in values[component].values()
            for value in series
            if math.isfinite(value)
        ),
        default=1.0,
    )
    shear_scale = max(shear_scale * 1.05, 1e-12)
    plt = science_pyplot()
    figure, axes = plt.subplots(
        3,
        3,
        sharex=True,
        sharey=True,
        figsize=paper_figsize(5.35, columns=2),
        layout="constrained",
    )
    shear_axes: list[list[Any]] = []
    legend_handles: list[Any] = []
    for index, coordinate in enumerate(coordinates):
        row, column = coordinate
        axis = axes[row, column]
        shear_axis = axis.twinx()
        if not shear_axes or len(shear_axes[-1]) == 3:
            shear_axes.append([])
        shear_axes[-1].append(shear_axis)
        _plot_series(
            axis,
            times,
            values["fz"][coordinate],
            color=_COLORS["feedback"],
            label=r"$F_z$",
            lw=1.0,
        )
        _plot_series(
            shear_axis,
            times,
            values["fx"][coordinate],
            color=_COLORS["right"],
            label=r"$F_x$",
            ls="--",
            lw=0.85,
        )
        _plot_series(
            shear_axis,
            times,
            values["fy"][coordinate],
            color=_COLORS["diagnostic"],
            label=r"$F_y$",
            ls=":",
            lw=0.9,
        )
        shear_axis.set_ylim(-shear_scale, shear_scale)
        axis.set_title(rf"$T_{{{row},{column}}}$", pad=2)
        axis.grid(alpha=0.16)
        axis.tick_params(labelleft=column == 0, labelbottom=row == 2)
        shear_axis.tick_params(
            axis="y",
            labelright=column == 2,
            right=column == 2,
            colors=_COLORS["muted"],
        )
        if index == 0:
            legend_handles = [axis.lines[0], *shear_axis.lines]
    axes[1, 0].set_ylabel(r"$F_z\,(\mathrm{N})$")
    shear_axes[1][2].set_ylabel(r"$F_x,F_y\,(\mathrm{N})$", color=_COLORS["muted"])
    figure.supxlabel(r"$t\,(\mathrm{s})$")
    figure.legend(
        legend_handles,
        [r"$F_z$", r"$F_x$", r"$F_y$"],
        loc="outside upper center",
        ncol=3,
        frameon=False,
        fontsize=8,
    )
    return figure


def _formats(formats: Sequence[str]) -> tuple[str, ...]:
    """校验并规范输出格式，保持调用方指定的精确产物集合。"""
    result = tuple(format_.removeprefix(".").lower() for format_ in formats)
    if (
        not result
        or any(format_ not in {"png", "pdf"} for format_ in result)
        or len(set(result)) != len(result)
    ):
        raise ValueError("formats 必须是非空且不重复的 png/pdf 序列。")
    return result


def _save_figure(figure: Any, output_dir: Path, stem: str, formats: Sequence[str]) -> list[Path]:
    """按请求格式保存并关闭一张图。"""
    paths = [
        save_publication_figure(figure, output_dir / f"{stem}.{format_}") for format_ in formats
    ]
    import matplotlib.pyplot as plt

    plt.close(figure)
    return paths


def render_tracking_plot(
    path: Path,
    trace: list[dict[str, object]],
    *,
    metrics: Mapping[str, object] | None = None,
    config: Mapping[str, object] | None = None,
) -> Path:
    """渲染兼容历史 ``output_plot`` API 的单张跟踪图。

    Args:
        path: 唯一输出文件路径，扩展名决定 PNG 或 PDF 格式。
        trace: 内存中的力跟踪 trace。
        metrics: 已计算的指标；只用于标注，不参与重新统计。
        config: effective 参数或兼容的 profile/task 映射。

    Returns:
        实际生成的图像路径。

    Raises:
        ValueError: trace 为空或缺少目标、反馈力。
    """
    if not trace:
        raise ValueError("trace 不能为空。")
    suffix = path.suffix.removeprefix(".").lower()
    if suffix not in {"png", "pdf"}:
        raise ValueError("path 必须以 .png 或 .pdf 结尾。")
    figure = _render_tracking_figure(trace, metrics=metrics, config=config)
    if figure is None:
        raise ValueError("trace 缺少可绘制的目标力或反馈力。")
    try:
        return save_publication_figure(figure, path)
    finally:
        import matplotlib.pyplot as plt

        plt.close(figure)


def render_run_artifacts(
    *,
    trace: list[dict[str, object]],
    metrics: Mapping[str, object] | None,
    config: Mapping[str, object] | None,
    output_dir: Path,
    tactile_detail: bool = False,
    plot_mode: Literal["summary", "diagnostic"] = "diagnostic",
    formats: tuple[str, ...] = ("png",),
) -> list[Path]:
    """从内存 trace 生成可重绘的 DMgripper 力跟踪图组。

    函数不读取 run 目录、不访问控制器或 MuJoCo，也不登记 manifest。历史 trace
    缺少某类信号时只省略对应图或面板，绝不以零值补造数据。

    Args:
        trace: 内存中的逐行 trace。
        metrics: 已保存或本次已计算的指标；不在这里重新统计。
        config: ``effective_parameters.json`` 内容，或仅含 ``profile``/``task`` 的映射。
        output_dir: 图像输出目录。
        plot_mode: summary 仅绘制力跟踪，diagnostic 绘制完整诊断图组。
        tactile_detail: 是否在存在完整 3×3 taxel 且明确接触确认时额外输出细节图。
        formats: 需要生成的 ``png`` 与/或 ``pdf`` 格式。

    Returns:
        实际生成的文件路径，按图组和格式顺序排列。

    Raises:
        ValueError: trace 为空、格式无效或没有任何可绘制信号。
    """
    if plot_mode not in {"summary", "diagnostic"}:
        raise ValueError("plot_mode 必须为 summary 或 diagnostic。")
    if not trace:
        raise ValueError("trace 不能为空。")
    resolved_formats = _formats(formats)
    output_dir = Path(output_dir)
    renderers = [
        ("tracking", _render_tracking_figure(trace, metrics=metrics, config=config)),
    ]
    if plot_mode == "diagnostic":
        renderers.extend(
            [
                ("tactile", _render_tactile_figure(trace)),
                ("controller", _render_controller_figure(trace, config=config)),
            ]
        )
    paths: list[Path] = []
    for stem, figure in renderers:
        if figure is not None:
            paths.extend(_save_figure(figure, output_dir, stem, resolved_formats))
    if tactile_detail:
        for side in ("left", "right"):
            figure = _render_taxel_figure(trace, side, config)
            if figure is not None:
                paths.extend(
                    _save_figure(figure, output_dir, f"tactile_{side}_detail", resolved_formats)
                )
    if not paths:
        raise ValueError("trace 不含可绘制的力跟踪信号。")
    return paths


__all__ = ["render_run_artifacts", "render_tracking_plot"]

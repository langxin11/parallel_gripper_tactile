"""摩擦估计与 Ramp 力跟踪的离屏演示视频录制。

在 ``run_friction_estimation`` 与 ``run_force_tracking`` 新增的可选逐帧回调
基础上，把 MuJoCo 场景渲染与 matplotlib 实时曲线面板合成一张带中文标注的
仪表盘画面，再用仓库统一的 ffmpeg 流程编码为 MP4。录制只消费仿真快照，
不修改实验循环语义、不写产物 trace，便于组会与论文口头报告展示。

视觉约定（仅作用于本录制层，不影响实验）：

- 左侧 HUD 信息卡与右侧曲线面板均使用 Matplotlib MathText（不启用系统
  LaTeX，``text.usetex=False``）；HUD 由可复用的 ``_MathHudOverlay`` 渲染成
  透明 RGBA 再与 MuJoCo 帧合成，避免每帧重建 Figure；
- 颜色语义全局一致：黑/灰虚线 = 目标/参考，蓝 = 实测，橙 = 外部切向载荷，
  绿 = 估计摩擦极限，紫 = 在线估计 μ̂，红虚线 = 真值/关键事件；
- 场景相机采用靠近机构、略抬高的整机构视角：夹爪+被抓物体占左画面约
  40–50%，注视夹爪自身 geom 质心（固定基座，构图不随被夹物体漂移），
  默认不做自动拉近；两个演示可分别调整方位/俯仰/距离；
- 阶段不再用大面积底色，改为顶部窄 phase bar + 极淡背景；
- 三个数据子图共享当前时间游标；滑移时刻只标注一次。

典型用法见 ``scripts/demos/record_experiment_demos.py``。
"""

from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
import shutil
import tempfile
from typing import TYPE_CHECKING

import numpy as np

from ..visualization import FULL_WIDTH_FONT_SCALE, PAPER_FONT_STACK, science_pyplot
from ..scenes.custom import CUBE_PREFIX, GRIPPER_PREFIX
from ..video import add_arrow_to_scene, encode_video, save_pixels

if TYPE_CHECKING:
    from ..config.profiles import GripperProfile
    from .force_tracking import ForceTrackingTask
    from .friction_estimation import FrictionEstimationTask

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PROFILE = REPOSITORY_ROOT / "configs" / "dm_gripper.yaml"
DEFAULT_RAMP_TASK = REPOSITORY_ROOT / "configs" / "task" / "force_tracking" / "ramp.yaml"
DEFAULT_FRICTION_TASK = (
    REPOSITORY_ROOT / "configs" / "task" / "friction_estimation" / "nominal_friction.yaml"
)
DEFAULT_DEMOS_DIR = REPOSITORY_ROOT / "outputs" / "demos"

#: 默认使用 1080p 高清画布，右侧数据面板占 45%。
DEFAULT_VIDEO_WIDTH_PX = 1920
DEFAULT_VIDEO_HEIGHT_PX = 1080
PANEL_WIDTH_PX = 864
#: HUD 与数据面板直接复用项目论文字体栈。
FONT_STACK = PAPER_FONT_STACK

#: 场景相机预设：不同演示可分别指定方位角、俯仰角与距离。
#: 经人工验收，夹爪+被抓物体应占左画面约 40–50%：采用整机构视角（夹爪本体
#: 入画、不变形），比默认更近并略抬高，避免下方大片无信息地板。
CAMERA_PRESETS = {
    "ramp": {"azimuth": 135.0, "elevation": -22.0, "distance": 0.26},
    "friction": {"azimuth": 135.0, "elevation": -22.0, "distance": 0.26},
}

# ---- 颜色物理语义（贯穿所有演示与子图） -------------------------------------
C_TARGET = "#5a5a5a"  # 目标 / 参考（灰虚线）
C_MEASURED = "#0072b2"  # 实测（蓝）
C_LOAD = "#e07600"  # 外部切向载荷（橙）
C_CAPACITY = "#1f9d55"  # 估计摩擦极限（绿）
C_ESTIMATE = "#7a3b8f"  # 在线估计 μ̂（紫）
C_TRUE = "#c0392b"  # 真值 / 关键事件（红虚线）
C_CURSOR = "#202020"  # 当前时间游标（细实线，不入图例）

#: phase bar / 背景底色：同一阶段两种演示共用浅色，大面积底色仅作极淡提示。
BAR_COLORS = {
    "接触": "#e8e8e8",
    "接近接触": "#e8e8e8",
    "稳态夹持": "#dce9f7",
    "匀速增载": "#ffe9d6",
    "匀速卸载": "#f4ecdf",
    "力保持": "#ececec",
    "切向增载": "#ffe2cf",
    "滑移": "#ffc9c2",
    "稳定恢复": "#f1ecda",
    "自适应增载": "#dbe9fb",
}


def _load_font(size: int):
    """按字体栈解析可用字体并返回对应尺寸的 PIL 字体。

    Args:
        size: 字体像素尺寸。

    Returns:
        可绘制中英文的 ImageFont 对象。
    """
    from PIL import ImageFont
    from matplotlib import font_manager
    from matplotlib.font_manager import FontProperties

    for family in FONT_STACK:
        try:
            path = font_manager.findfont(FontProperties(family=family), fallback_to_default=False)
        except ValueError:
            continue
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _hud_text(image, draw, x: int, y: int, text: str, font, fill) -> None:
    """在 PIL 画布上绘制文本（仅用于 Ramp 的 PASS 结果卡）。

    数学符号的 HUD 信息卡改用 :class:`_MathHudOverlay` 以支持 MathText；
    此处保留给不含公式的 PASS 徽章。
    """
    draw.text((x, y), text, font=font, fill=fill)


class _MathHudOverlay:
    r"""用可复用的 Matplotlib 画布绘制 HUD 透明信息卡（支持 MathText）。

    PIL 的 ``ImageDraw`` 不具备 MathText 排版能力，因此 HUD 里的数学符号
    （如 :math:`F_n^\\ast`、:math:`\\hat{\\mu}`、:math:`m_f`）只能退化成普通
    字符。这里改用 Matplotlib 渲染成透明 RGBA 叠加层：``text.usetex=False``
    走内置 MathText，不依赖系统 LaTeX。

    画布、坐标轴与文本对象只建一次；逐帧仅 ``set_text`` 更新数值并重绘，
    避免每帧新建 Figure。当某行文字宽度超过当前信息卡宽度时才重建（单调
    增宽，不会来回抖动），稳态下零重建。
    """

    _DPI = 200

    def __init__(
        self,
        scale: float,
        sample_lines: list[tuple[int, str, tuple[int, int, int, int]]],
    ) -> None:
        import matplotlib

        matplotlib.use("Agg")

        self._scale = scale
        self._dpi = self._DPI
        max_size = max(sz for sz, _, _ in sample_lines)
        self._line_h = max(round(max_size * scale * 1.5), round(26 * scale))
        self._pad_x = round(13 * scale)
        self._pad_y0 = round(7 * scale)
        self._n = len(sample_lines)
        self._box_h = self._line_h * self._n + round(14 * scale)
        # 先用一张临时大图量出最宽行，确定信息卡宽度；再按尺寸建正式画布。
        self._box_w = 0
        self._measure(sample_lines)
        self._build(sample_lines)

    def _measure(self, sample_lines: list[tuple[int, str, tuple[int, int, int, int]]]) -> None:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg

        prov_w = round(600 * self._scale)
        figure = Figure(
            figsize=(prov_w / self._dpi, self._box_h / self._dpi), dpi=self._dpi, facecolor="none"
        )
        canvas = FigureCanvasAgg(figure)
        axes = figure.add_axes([0, 0, 1, 1])
        axes.set_axis_off()
        axes.set_xlim(0, prov_w)
        axes.set_ylim(self._box_h, 0)
        renderer = canvas.get_renderer()
        max_width = 0.0
        for index, (size, text, color) in enumerate(sample_lines):
            artist = axes.text(
                self._pad_x,
                self._pad_y0 + index * self._line_h,
                text,
                fontsize=size * self._scale * 72 / self._dpi,
                fontfamily=FONT_STACK,
                color=tuple(c / 255 for c in color[:3]),
                va="top",
                ha="left",
            )
            canvas.draw()
            max_width = max(max_width, artist.get_window_extent(renderer=renderer).width)
        self._box_w = int(np.ceil(max_width)) + round(26 * self._scale)

    def _build(self, sample_lines: list[tuple[int, str, tuple[int, int, int, int]]]) -> None:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.patches import FancyBboxPatch

        self._figure = Figure(
            figsize=(self._box_w / self._dpi, self._box_h / self._dpi),
            dpi=self._dpi,
            facecolor="none",
        )
        self._canvas = FigureCanvasAgg(self._figure)
        self._axes = self._figure.add_axes([0, 0, 1, 1])
        self._axes.set_axis_off()
        self._axes.set_xlim(0, self._box_w)
        self._axes.set_ylim(self._box_h, 0)
        self._renderer = self._canvas.get_renderer()
        self._bg = FancyBboxPatch(
            (0, 0),
            self._box_w,
            self._box_h,
            boxstyle=f"round,pad=0,rounding_size={round(7 * self._scale)}",
            transform=self._axes.transData,
            facecolor=(0, 0, 0, 0.59),
            edgecolor="none",
            zorder=0,
        )
        self._axes.add_patch(self._bg)
        self._texts = []
        for index, (size, text, color) in enumerate(sample_lines):
            artist = self._axes.text(
                self._pad_x,
                self._pad_y0 + index * self._line_h,
                text,
                fontsize=size * self._scale * 72 / self._dpi,
                fontfamily=FONT_STACK,
                color=tuple(c / 255 for c in color[:3]),
                va="top",
                ha="left",
                zorder=2,
            )
            self._texts.append(artist)

    def _render(self, lines: list[tuple[int, str, tuple[int, int, int, int]]]) -> np.ndarray:
        """更新文本、必要时增宽信息卡，返回 RGBA 叠加层数组。"""
        for artist, (size, text, color) in zip(self._texts, lines):
            artist.set_text(text)
            artist.set_fontsize(size * self._scale * 72 / self._dpi)
            artist.set_color(tuple(c / 255 for c in color[:3]))
        self._canvas.draw()
        need = max(
            artist.get_window_extent(renderer=self._renderer).width for artist in self._texts
        )
        if need + round(26 * self._scale) > self._box_w:
            self._box_w = int(np.ceil(need)) + round(26 * self._scale)
            self._build(lines)
            self._canvas.draw()
        return np.asarray(self._canvas.buffer_rgba())

    def blend(
        self,
        pixels: np.ndarray,
        lines: list[tuple[int, str, tuple[int, int, int, int]]],
        left: int,
        top: int,
    ) -> np.ndarray:
        """把 HUD 叠加层合成到 ``pixels`` 的 ``(left, top)`` 处，返回新数组。"""
        rgba = self._render(lines)
        height, width = rgba.shape[:2]
        region = pixels[top : top + height, left : left + width]
        rh, rw = region.shape[:2]
        rr = rgba[:rh, :rw]
        alpha = rr[..., 3:4].astype(np.float32) / 255.0
        blended = rr[..., :3] * alpha + region * (1 - alpha)
        pixels[top : top + rh, left : left + rw] = blended.astype(np.uint8)
        return pixels


def _ensure_headless_style() -> None:
    """把 matplotlib 配置为无界面后端与中文字体（仅首次生效）。"""
    import matplotlib

    matplotlib.use("Agg")
    if not getattr(_ensure_headless_style, "_applied", False):
        plt = science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)
        plt.rcParams.update(
            {
                "axes.unicode_minus": False,
                "text.usetex": False,
                "figure.facecolor": "white",
                "axes.facecolor": "white",
            }
        )
        _ensure_headless_style._applied = True


def _phase_runs(times: list[float], labels: list[str]) -> list[tuple[float, float, str]]:
    """把逐帧展示标签序列折叠为 (起, 止, 标签) 区间。

    Args:
        times: 与标签一一对应的帧时刻。
        labels: 每帧的展示阶段标签。

    Returns:
        闭开区间列表；末段终点为当前时刻。
    """
    runs: list[tuple[float, float, str]] = []
    for t, label in zip(times, labels):
        if not runs or runs[-1][2] != label:
            runs.append((t, t, label))
        else:
            runs[-1] = (runs[-1][0], t, label)
    return runs


def _label_color(label: str) -> str:
    """返回展示标签对应的浅色。"""
    return BAR_COLORS.get(label, "#f0f0f0")


def _make_panel_figure(width_px: int, height_px: int):
    """创建 4 行面板图（顶部窄 phase bar + 三个数据子图）并共享横轴。

    Args:
        width_px: 面板像素宽（图宽 = width_px/100 英寸，dpi=100）。
        height_px: 面板像素高。

    Returns:
        (figure, phase_bar_axes, data_axes)。
    """
    _ensure_headless_style()
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(
        4,
        1,
        sharex=True,
        figsize=(width_px / 100.0, height_px / 100.0),
        dpi=100,
        gridspec_kw={"height_ratios": [0.34, 1, 1, 1], "hspace": 0.16},
    )
    fig.subplots_adjust(left=0.125, right=0.975, top=0.97, bottom=0.06)
    bar_ax, *data_axes = axes
    bar_ax.set_yticks([])
    bar_ax.set_ylim(0, 1)
    bar_ax.set_frame_on(False)
    bar_ax.tick_params(axis="x", labelbottom=False, length=0)
    return fig, bar_ax, data_axes


def _style_data_axis(axis) -> None:
    """统一数据子图样式：内刻度、极淡网格、白底。"""
    axis.grid(True, axis="y", alpha=0.18, linewidth=0.6)
    axis.tick_params(direction="in", which="both", top=True, right=True)


def _render_panel(figure, bar_ax, data_axes, *, x_max: float, x_min: float = 0.0) -> np.ndarray:
    """收尾横轴范围并把整图栅格化为 RGB 数组。

    面板由 pyplot 逐帧创建，渲染后立即关闭对应 Figure，避免录制过程中
    积累大量未关闭图形占用内存（`More than 20 figures` 告警）。
    """
    import matplotlib.pyplot as plt

    for axis in (bar_ax, *data_axes):
        axis.set_xlim(x_min, max(0.05, x_max))
    figure.align_ylabels(data_axes)
    figure.canvas.draw()
    buffer = np.asarray(figure.canvas.buffer_rgba())[:, :, :3]
    plt.close(figure)
    return np.ascontiguousarray(buffer)


def _draw_phase_bar(bar_ax, runs: list[tuple[float, float, str]], t_now: float) -> None:
    """绘制顶部窄 phase bar：浅色块 + 简短阶段文字。"""
    for t0, t1, label in runs:
        bar_ax.axvspan(t0, t1, color=_label_color(label), linewidth=0)
        if t1 - t0 > 0.35:
            bar_ax.text(
                (t0 + t1) / 2.0,
                0.5,
                label,
                ha="center",
                va="center",
                fontsize=7.5,
                color="#3a3a3a",
            )
    bar_ax.axvline(t_now, color=C_CURSOR, linewidth=0.9)


def _draw_time_cursor(data_axes, t_now: float) -> None:
    """在数据子图绘制与当前帧同步的细实线时间游标。"""
    for axis in data_axes:
        axis.axvline(t_now, color=C_CURSOR, linewidth=0.8, alpha=0.85, zorder=1)


def _draw_friction_panel(
    width_px: int,
    height_px: int,
    *,
    times: list[float],
    series: dict[str, list[float]],
    runs: list[tuple[float, float, str]],
    current: dict[str, float],
) -> np.ndarray:
    """绘制摩擦估计三行曲线面板（法向力 / 切向载荷 / 摩擦系数）。

    滑移检测时刻用同一条红虚线贯穿三个数据子图，只在首幅顶部标注一次
    「初始滑移」；第二幅右上角动态显示摩擦裕度 ``m_f``（仅展示）。
    """
    fig, bar_ax, axes = _make_panel_figure(width_px, height_px)
    for t0, t1, label in runs:
        color = _label_color(label)
        for axis in axes:
            axis.axvspan(t0, t1, color=color, alpha=0.15, linewidth=0, zorder=0)
    _draw_phase_bar(bar_ax, runs, times[-1])

    target_ax, load_ax, mu_ax = axes
    target_ax.plot(
        times,
        series["target_fn"],
        color=C_TARGET,
        linestyle="--",
        linewidth=1.7,
        label=r"目标 $F_n^\ast$",
    )
    target_ax.plot(times, series["filtered"], color=C_MEASURED, linewidth=2.1, label=r"实测 $F_n$")
    target_ax.set_ylabel(r"法向力 $F_n$ (N)")
    target_ax.set_ylim(0, 5)
    target_ax.legend(loc="upper left", frameon=False)
    _style_data_axis(target_ax)

    load_ax.plot(times, series["demand"], color=C_LOAD, linewidth=2.1, label=r"切向载荷 $F_t$")
    load_ax.plot(times, series["shear"], color=C_MEASURED, linewidth=1.4, label=r"实测剪切（触觉）")
    load_ax.plot(
        times,
        series["capacity"],
        color=C_CAPACITY,
        linestyle="--",
        linewidth=1.6,
        label=r"估计摩擦极限 $\hat{\mu}F_n$",
    )
    load_ax.set_ylabel(r"切向力 $F_t$ (N)")
    finite_loads = [
        value
        for key in ("demand", "shear", "capacity")
        for value in series[key]
        if math.isfinite(value)
    ]
    y_high = max([3.0, *finite_loads, 1.0])
    load_ax.set_ylim(0, y_high * 1.05)
    load_ax.legend(loc="upper left", frameon=False)
    _style_data_axis(load_ax)

    mu_ax.plot(
        times, series["mu_hat"], color=C_ESTIMATE, linewidth=2.1, label=r"估计值 $\hat{\mu}$"
    )
    mu_ax.axhline(
        current.get("mu_true", 0.8),
        color=C_TRUE,
        linestyle="--",
        linewidth=1.6,
        label=r"真值 $\mu$",
    )
    mu_ax.set_ylabel(r"摩擦系数 $\mu$")
    mu_ax.set_ylim(0, max(1.2, float(current.get("mu_true", 0.8)) + 0.5))
    mu_ax.set_xlabel(r"时间 $t$ (s)")
    mu_ax.legend(loc="upper left", frameon=False)
    _style_data_axis(mu_ax)

    detection_time = current.get("detection_time_s", math.nan)
    if math.isfinite(detection_time):
        for axis in axes:
            axis.axvline(detection_time, color=C_TRUE, linestyle="--", linewidth=1.5, zorder=2)
        target_ax.annotate(
            "初始滑移",
            xy=(detection_time, target_ax.get_ylim()[1]),
            xytext=(detection_time + 0.15, target_ax.get_ylim()[1] * 0.9),
            color=C_TRUE,
            fontsize=8.5,
            fontweight="bold",
        )
    margin = current.get("margin_n", math.nan)
    if math.isfinite(margin):
        load_ax.text(
            0.985,
            0.94,
            rf"$m_f$ = {margin:+.2f} N",
            transform=load_ax.transAxes,
            ha="right",
            va="top",
            fontsize=9.0,
            color="#1a1a1a",
            bbox={
                "boxstyle": "round,pad=0.3",
                "facecolor": "#fdfdfd",
                "edgecolor": "#bbbbbb",
                "linewidth": 0.6,
            },
        )
    _draw_time_cursor(axes, times[-1])
    return _render_panel(fig, bar_ax, axes, x_max=times[-1])


def _draw_ramp_panel(
    width_px: int,
    height_px: int,
    *,
    times: list[float],
    series: dict[str, list[float]],
    runs: list[tuple[float, float, str]],
) -> np.ndarray:
    """绘制 Ramp 力跟踪三行曲线面板（法向力 / 误差 / 电机力矩）。"""
    fig, bar_ax, axes = _make_panel_figure(width_px, height_px)
    for t0, t1, label in runs:
        color = _label_color(label)
        for axis in axes:
            axis.axvspan(t0, t1, color=color, alpha=0.15, linewidth=0, zorder=0)
    _draw_phase_bar(bar_ax, runs, times[-1])

    force_ax, error_ax, torque_ax = axes
    force_ax.plot(
        times,
        series["target"],
        color=C_TARGET,
        linestyle="--",
        linewidth=1.7,
        label=r"目标 $F_n^\ast$",
    )
    force_ax.plot(times, series["filtered"], color=C_MEASURED, linewidth=2.1, label=r"实测 $F_n$")
    force_ax.set_ylabel(r"法向力 $F_n$ (N)")
    force_ax.set_ylim(0, 8)
    force_ax.legend(loc="upper left", frameon=False)
    _style_data_axis(force_ax)

    error_ax.plot(times, series["error"], color="#444444", linewidth=2.0, label=r"跟踪误差 $e_F$")
    error_ax.axhline(0.0, color="#999999", linewidth=0.9)
    error_ax.set_ylabel(r"力误差 (N)")
    error_ax.legend(loc="upper left", frameon=False)
    _style_data_axis(error_ax)

    torque_ax.plot(
        times, series["torque"], color="#7f6bb5", linewidth=2.0, label=r"电机力矩 $\tau$"
    )
    torque_ax.axhline(0.0, color="#999999", linewidth=0.9)
    torque_ax.set_ylabel(r"力矩 (N·m)")
    torque_ax.set_xlabel(r"时间 $t$ (s)")
    torque_ax.legend(loc="upper left", frameon=False)
    _style_data_axis(torque_ax)

    _draw_time_cursor(axes, times[-1])
    return _render_panel(fig, bar_ax, axes, x_max=times[-1])


def _friction_label(phase: str, slip_seen: bool) -> str:
    """把内部阶段映射为摩擦演示的 phase bar 标签。

    检测时刻与后续恢复段合并在「滑移」，完整呈现「增载→接近极限→滑移→
    自适应增载」因果链；真实事件时刻不被修改，只影响展示标签。
    """
    if phase in {"approach_contact", "contact_settle"}:
        return "接触"
    if phase == "probe_settle":
        return "稳态夹持"
    if phase == "probe":
        return "滑移" if slip_seen else "切向增载"
    if phase == "recovery":
        return "滑移" if slip_seen else "稳定恢复"
    if phase == "schedule_load":
        return "自适应增载"
    return phase


def _ramp_label(phase: str, slope: float) -> str:
    """把内部阶段与目标斜率映射为 Ramp 演示的 phase bar 标签。"""
    if phase == "approach_contact":
        return "接近接触"
    if phase == "contact_settle":
        return "稳态夹持"
    if phase == "track_reference":
        if slope > 0.06:
            return "匀速增载"
        if slope < -0.06:
            return "匀速卸载"
        return "力保持"
    return phase


class _DemoRecorder:
    """在离屏仿真循环内逐帧合成场景与曲线面板的演示录制器。

    构造后以 ``on_frame(row, model, data)`` 作为实验循环的回调；仿真结束后
    调用 ``finish(result)`` 关闭渲染器、导出事件关键帧并把 PNG 帧序列编码
    为 MP4。
    """

    def __init__(
        self,
        *,
        kind: str,
        output: Path,
        width: int,
        height: int,
        fps: int,
        panel_width: int,
        draw_arrow: bool,
    ) -> None:
        self.kind = kind
        self.output = Path(output)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.panel_width = int(panel_width)
        if self.height <= 0 or self.fps <= 0 or self.panel_width >= self.width:
            raise ValueError("height/fps 必须为正，panel_width 必须小于画面总宽")
        self.scene_width = self.width - self.panel_width
        self.draw_arrow = draw_arrow
        self._renderer = None
        self._camera = None
        self._math_hud: _MathHudOverlay | None = None
        self._cube_body_id: int | None = None
        self._times: list[float] = []
        self._phases: list[str] = []
        self._labels: list[str] = []
        self._series: dict[str, list[float]] = {}
        self._current: dict[str, float] = {
            "detection_time_s": math.nan,
        }
        self._first_detection_seen = False
        self._detection_frame: int | None = None
        self._probe_start_frame: int | None = None
        self._frames_dir = Path(tempfile.mkdtemp(prefix=f"pgt_{kind}_demo_frames_"))
        self._frame_count = 0
        self._rolling_rmse_accum = 0.0
        self._rolling_rmse_count = 0.0

    def _lazy_setup(self, model, data) -> None:
        """按首个快照建立渲染器，并应用整机构视角的固定相机。

        相机采用经人工验收的构图：靠近并略抬高，让夹爪+被抓物体占左画面
        约 40–50%，避免下方大片无信息地板。注视点取夹爪自身的 geom 质心
        （夹爪为固定基座，构图不随被夹物体运动而漂移），夹爪完整入画。
        """
        import mujoco

        if self._renderer is not None:
            return
        self._renderer = mujoco.Renderer(model, self.height, self.scene_width)
        preset = CAMERA_PRESETS[self.kind]
        cube_id = model.body(f"{CUBE_PREFIX}target_cube").id
        self._cube_body_id = cube_id
        gripper_centroid = self._gripper_centroid(model, data)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.azimuth = float(preset["azimuth"])
        camera.elevation = float(preset["elevation"])
        camera.distance = float(preset["distance"])
        camera.lookat[:] = gripper_centroid
        self._camera = camera
        print(
            f"[{self.kind}] camera distance={camera.distance:.3f} m, "
            f"azimuth={camera.azimuth:.0f}, elevation={camera.elevation:.0f}, "
            f"lookat={np.round(gripper_centroid, 4)}"
        )

    def _gripper_centroid(self, model, data) -> np.ndarray:
        """返回夹爪自身 geom 的世界坐标质心（不含被夹物体）。"""
        positions = [
            data.geom_xpos[index]
            for index in range(model.ngeom)
            if model.geom(index).name.startswith(GRIPPER_PREFIX)
        ]
        return np.asarray(positions, dtype=np.float64).mean(axis=0)

    def _arrow_force(self, model, row: dict) -> tuple[np.ndarray, float] | None:
        """返回摩擦演示当前应绘制的切向力箭头 (方向单位向量, 幅值)。"""
        phase = str(row["phase"])
        magnitude = float(row.get("tangential_demand_n", 0.0))
        if magnitude <= 1e-6 or not self.draw_arrow:
            return None
        if phase == "probe":
            direction = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        else:
            gravity = np.asarray(model.opt.gravity[1:3], dtype=np.float64)
            norm = float(np.linalg.norm(gravity))
            if norm <= 1e-12:
                return None
            direction = np.concatenate(([0.0], gravity / norm)).astype(np.float64)
        return direction, magnitude

    def _last_slope(self, key: str) -> float:
        """返回某序列最近两帧的斜率（单位/秒）；数据不足返回 0。"""
        series = self._series.get(key)
        if not series or len(series) < 2:
            return 0.0
        dt = max(self._times[-1] - self._times[-2], 1e-6)
        return float((series[-1] - series[-2]) / dt)

    def on_frame(self, row: dict, model, data) -> None:
        """接收实验循环的一帧快照：更新历史、渲染并落盘合成画面。"""
        import mujoco

        self._lazy_setup(model, data)
        t = float(row["time_s"])
        phase = str(row["phase"])
        slip_now = bool(row.get("slip_detected", False))
        self._times.append(t)
        self._phases.append(phase)
        series = self._series
        mu_hat = float(row.get("mu_hat", math.nan))

        if self.kind == "ramp":
            values = {
                "target": float(row["target_normal_force_n"]),
                "filtered": float(row["filtered_normal_force_n"]),
                "error": float(row["target_normal_force_n"])
                - float(row["filtered_normal_force_n"]),
                "torque": float(row["motor_torque_n_m"]),
            }
            if phase == "track_reference":
                error = values["error"]
                self._rolling_rmse_accum += error * error
                self._rolling_rmse_count += 1.0
        else:
            if phase == "probe" and self._probe_start_frame is None:
                self._probe_start_frame = self._frame_count
            if slip_now and not self._first_detection_seen:
                self._first_detection_seen = True
                self._detection_frame = self._frame_count
                self._current["detection_time_s"] = t
            filtered = float(row["filtered_normal_force_n"])
            shear = float(row["measured_shear_support_n"])
            # μ̂ 只在初始滑移检出后才被视为可信估计：检测前容量/裕度留空，
            # 避免把探测期回退值画成误导性的摩擦极限。
            if self._first_detection_seen and math.isfinite(mu_hat):
                capacity = mu_hat * 2.0 * filtered
            else:
                capacity = math.nan
            values = {
                "target_fn": float(row["scheduled_target_force_n"]),
                "filtered": filtered,
                "demand": float(row["tangential_demand_n"]),
                "shear": shear,
                "capacity": capacity,
                "mu_hat": mu_hat,
            }
        for key, value in values.items():
            series.setdefault(key, []).append(value)

        self._current.update(
            {
                "mu_hat": mu_hat,
                "mu_true": float(row.get("true_friction_coefficient", math.nan)),
                "target_fn": values.get("target", values.get("target_fn", math.nan)),
                "filtered": values["filtered"],
                "shear": values.get("shear", math.nan),
                "demand": values.get("demand", math.nan),
            }
        )
        if self.kind == "ramp":
            label = _ramp_label(phase, self._last_slope("target"))
        else:
            label = _friction_label(phase, self._first_detection_seen)
            if self._first_detection_seen:
                mu_hat_now = self._current.get("mu_hat", math.nan)
                margin = self._margin_from_current(mu_hat_now, values["filtered"], values["shear"])
                self._current["margin_n"] = margin
        self._labels.append(label)

        assert self._renderer is not None and self._camera is not None
        force = self._arrow_force(model, row) if self.kind == "friction" else None
        scene_option = mujoco.MjvOption()
        scene_option.label = int(mujoco.mjtLabel.mjLABEL_NONE)
        scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
        scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
        self._renderer.update_scene(data, camera=self._camera, scene_option=scene_option)
        scene = self._renderer.scene
        if force is not None:
            direction, magnitude = force
            origin = data.xpos[self._cube_body_id].copy()
            # 固定箭尾；最大长度时箭尖抵达物体上方，因此幅值增加会沿受力方向生长。
            origin[2] += 0.075
            add_arrow_to_scene(
                scene,
                scene.ngeom,
                origin,
                direction * magnitude,
                scale=0.035,
                width=0.0016,
                min_length=0.025,
                max_length=0.055,
                emission=0.05,
                tip_at_origin=False,
            )
        pixels = self._renderer.render()

        runs = _phase_runs(self._times, self._labels)
        if self.kind == "ramp":
            panel = _draw_ramp_panel(
                self.panel_width, self.height, times=self._times, series=series, runs=runs
            )
        else:
            panel = _draw_friction_panel(
                self.panel_width,
                self.height,
                times=self._times,
                series=series,
                runs=runs,
                current=self._current,
            )
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        scene_h = min(pixels.shape[0], self.height)
        scene_w = min(pixels.shape[1], self.scene_width)
        canvas[:scene_h, :scene_w] = pixels[:scene_h, :scene_w]
        panel_h = min(panel.shape[0], self.height)
        panel_w = min(panel.shape[1], self.panel_width)
        canvas[:panel_h, self.scene_width :] = panel[:panel_h, :panel_w]

        lines = self._hud_lines(phase)
        if self._math_hud is None:
            self._math_hud = _MathHudOverlay(self.width / 1600.0, lines)
        left = top = round(14 * (self.width / 1600.0))
        annotated = self._math_hud.blend(canvas, lines, left, top)
        path = self._frames_dir / f"frame_{self._frame_count:06d}.png"
        save_pixels(annotated, path)
        self._frame_count += 1

    def _margin_from_current(self, mu_hat: float, filtered_fn: float, shear: float) -> float:
        """计算展示用摩擦裕度 m_f = μ̂·F_n − |F_t|（仅展示）。

        面板中的 ``F_n`` 为平均单侧法向力，裕度按双侧合计与第二幅的
        估计摩擦极限保持同一尺度；不参与滑移检测或控制。
        """
        return mu_hat * 2.0 * filtered_fn - abs(shear)

    def _status_word(self, phase: str) -> str:
        """返回 HUD 第二行的大字号当前状态（只显示当前阶段）。"""
        if self.kind == "ramp":
            slope = self._last_slope("target") if phase == "track_reference" else 0.0
            return {
                "approach_contact": "接近接触",
                "contact_settle": "稳态夹持",
                "track_reference": (
                    "匀速增载" if slope > 0.06 else "匀速卸载" if slope < -0.06 else "力保持"
                ),
            }.get(phase, phase)
        slip_seen = self._first_detection_seen
        return {
            "approach_contact": "接近接触",
            "contact_settle": "稳态夹持",
            "probe_settle": "稳态夹持",
            "probe": "初始滑移" if slip_seen else "切向增载",
            "recovery": "稳定恢复",
            "schedule_load": "自适应增载",
        }.get(phase, phase)

    def _hud_lines(self, phase: str) -> list[tuple[int, str, tuple[int, int, int, int]]]:
        """按四层结构生成 HUD 信息卡：(字号, 文本, 颜色)。"""
        small, big, medium, tiny = 15, 29, 19, 15
        white = (255, 255, 255, 255)
        soft = (226, 235, 255, 255)
        dim = (210, 222, 240, 255)
        status = self._status_word(phase)
        filtered = self._current["filtered"]
        target = self._current["target_fn"]
        if self.kind == "ramp":
            rmse = (
                math.sqrt(self._rolling_rmse_accum / self._rolling_rmse_count)
                if self._rolling_rmse_count > 0
                else math.nan
            )
            rmse_text = f"{rmse:.3f} N" if math.isfinite(rmse) else "—"
            return [
                (small, "自适应抓取 · Ramp 力跟踪", soft),
                (big, f"状态  {status}", white),
                (
                    medium,
                    f"目标 $F_n^\\ast$  {target:5.2f} N   实测 $F_n$  {filtered:5.2f} N",
                    soft,
                ),
                (tiny, f"跟踪段 RMSE（截至当前） {rmse_text}", dim),
            ]
        demand = self._current.get("demand", math.nan)
        mu_hat = self._current.get("mu_hat", math.nan)
        margin = self._current.get("margin_n", math.nan)
        mu_text = f"{mu_hat:.3f}" if math.isfinite(mu_hat) else "—"
        ft_text = f"{demand:4.2f}" if math.isfinite(demand) else "—"
        margin_text = f"{margin:+.2f}" if math.isfinite(margin) else "—"
        return [
            (small, "自适应抓取 · 摩擦在线估计", soft),
            (big, f"状态  {status}", white),
            (
                medium,
                f"目标 $F_n^\\ast$  {target:5.2f} N   实测 $F_n$  {filtered:5.2f} N",
                soft,
            ),
            (
                tiny,
                f"$F_t$  {ft_text} N   $\\hat{{\\mu}}$ {mu_text}   摩擦裕度 $m_f$ {margin_text} N",
                dim,
            ),
        ]

    def _preview_path(self) -> Path:
        """返回关键帧导出目录。"""
        return self.output.parent / "preview"

    def _export_previews(self) -> list[Path]:
        """按真实事件导出关键帧，供人工验收视觉构图。

        ramp: start（首帧）、mid（目标力达峰）、end（末帧）；
        friction: before_slip（滑移前）、slip（检测瞬间）、after_adaptation（末帧）。
        """
        names = {
            "ramp": (
                "force_tracking_start.png",
                "force_tracking_mid.png",
                "force_tracking_end.png",
            ),
            "friction": (
                "friction_before_slip.png",
                "friction_slip.png",
                "friction_after_adaptation.png",
            ),
        }
        folder = self._preview_path()
        folder.mkdir(parents=True, exist_ok=True)
        exported: list[Path] = []
        indices = (
            self._ramp_preview_indices()
            if self.kind == "ramp"
            else self._friction_preview_indices()
        )
        for name, index in zip(names[self.kind], indices):
            source = self._frames_dir / f"frame_{index:06d}.png"
            if not source.is_file():
                continue
            destination = folder / name
            shutil.copyfile(source, destination)
            exported.append(destination)
        return exported

    def _ramp_preview_indices(self) -> list[int]:
        """返回 Ramp 演示的三帧下标（首帧 / 目标力峰值 / 末帧）。"""
        target = self._series.get("target") or [0.0]
        peak = max(target)
        peak_index = next(i for i, value in enumerate(target) if value == peak)
        return [0, peak_index, self._frame_count - 1]

    def _friction_preview_indices(self) -> list[int]:
        """返回摩擦演示的三帧下标（滑移前 / 检测 / 末帧）。"""
        detection = self._detection_frame
        if detection is None:
            detection = (self._probe_start_frame or 0) + 1
        before = 0
        if self._probe_start_frame is not None and detection is not None:
            before = max(self._probe_start_frame, detection - int(0.5 * self.fps))
        return [before, detection, self._frame_count - 1]

    def _overlay_pass(self) -> None:
        """在 Ramp 视频末尾约 0.7 s 帧上叠加绿色 PASS 结果卡。"""
        if self.kind != "ramp":
            return
        tail = min(max(1, round(0.7 * self.fps)), self._frame_count)
        try:
            from PIL import Image, ImageDraw
        except ImportError:
            return
        scale = self.width / 1600.0
        badge_font = _load_font(round(50 * scale))
        detail_font = _load_font(round(22 * scale))
        rmse = (
            math.sqrt(self._rolling_rmse_accum / self._rolling_rmse_count)
            if self._rolling_rmse_count > 0
            else math.nan
        )
        rmse_text = f"{rmse:.3f} N" if math.isfinite(rmse) else "—"
        detail = f"力跟踪 PASS    RMSE {rmse_text}"
        for frame_index in range(self._frame_count - tail, self._frame_count):
            path = self._frames_dir / f"frame_{frame_index:06d}.png"
            if not path.is_file():
                continue
            image = Image.open(path).convert("RGB")
            draw = ImageDraw.Draw(image, "RGBA")
            cx = image.width // 2
            top = round(image.height * 0.16)
            title_w = draw.textlength("PASS", font=badge_font)
            detail_w = draw.textlength(detail, font=detail_font)
            box_w = max(title_w, detail_w) + round(90 * scale)
            left = cx - box_w // 2
            box_h = round(118 * scale)
            draw.rounded_rectangle(
                (left, top, left + box_w, top + box_h),
                radius=round(12 * scale),
                fill=(20, 90, 45, 225),
            )
            _hud_text(
                image,
                draw,
                cx - int(title_w / 2),
                top + round(8 * scale),
                "PASS",
                badge_font,
                (255, 255, 255, 255),
            )
            _hud_text(
                image,
                draw,
                cx - int(detail_w / 2),
                top + round(76 * scale),
                detail,
                detail_font,
                (226, 246, 232, 255),
            )
            image.save(path)

    def finish(self, result: object) -> None:
        """关闭渲染器、导出关键帧并编码 MP4。"""
        if self._renderer is not None:
            self._renderer.close()
        if self._frame_count == 0:
            raise RuntimeError("no demo frames were recorded")
        if self.kind == "ramp" and bool(getattr(result, "passed", False)):
            self._overlay_pass()
        previews = self._export_previews()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        encode_video(self._frames_dir, self.output, self.fps, self.width, self.height)
        shutil.rmtree(self._frames_dir, ignore_errors=True)
        for preview in previews:
            print(f"Preview: {preview}")

    @property
    def frame_count(self) -> int:
        """返回已录制的帧数。"""
        return self._frame_count


def record_force_tracking_ramp_video(
    *,
    profile_path: Path | GripperProfile = DEFAULT_PROFILE,
    task_path: Path = DEFAULT_RAMP_TASK,
    resolved_task: ForceTrackingTask | None = None,
    output: Path = DEFAULT_DEMOS_DIR / "force_tracking_ramp.mp4",
    width: int = DEFAULT_VIDEO_WIDTH_PX,
    height: int = DEFAULT_VIDEO_HEIGHT_PX,
    fps: int = 30,
    panel_width: int = PANEL_WIDTH_PX,
    controller_variant: str = "full",
    object_material: str = "hard",
    sensor_noise_seed: int | None = None,
) -> tuple[object, int]:
    """录制一次 Ramp 目标力跟踪并输出带实时曲线的 MP4。

    Args:
        profile_path: DM_Gripper profile 路径或已解析的冻结 profile。
        task_path: Ramp 力跟踪任务 YAML 路径。
        resolved_task: 已解析的冻结任务；传入时优先于 ``task_path``。
        output: 输出 MP4 路径。
        width: 画面总宽；右侧曲线面板占 ``panel_width``，其余为场景。
        height: 画面高。
        fps: 视频帧率，也是逐帧回调的仿真采样频率。
        panel_width: 右侧曲线面板宽度。
        controller_variant: 控制器变体，语义与 `pgt run force-track` 相同。
        object_material: 显式接触 preset。
        sensor_noise_seed: 触觉噪声种子；None 使用 profile 默认。

    Returns:
        (实验结果, 录制帧数)。

    Raises:
        RuntimeError: 仿真结束前没有录到任何帧。
    """
    from .force_tracking import ForceTrackingTask, run_force_tracking

    task = (
        resolved_task
        if isinstance(resolved_task, ForceTrackingTask)
        else ForceTrackingTask.load(task_path)
    )
    recorder = _DemoRecorder(
        kind="ramp",
        output=output,
        width=width,
        height=height,
        fps=fps,
        panel_width=panel_width,
        draw_arrow=False,
    )
    result = run_force_tracking(
        profile_path,
        task=task,
        controller_variant=controller_variant,  # type: ignore[arg-type]
        object_material=object_material,  # type: ignore[arg-type]
        sensor_noise_seed=sensor_noise_seed,
        render_fps=fps,
        on_frame=recorder.on_frame,
    )
    recorder.finish(result)
    print(f"Video: {output}")
    print(f"Frames: {recorder.frame_count}")
    return result, recorder.frame_count


def record_friction_demo_video(
    *,
    profile_path: Path | GripperProfile = DEFAULT_PROFILE,
    task_path: Path = DEFAULT_FRICTION_TASK,
    resolved_task: FrictionEstimationTask | None = None,
    output: Path = DEFAULT_DEMOS_DIR / "friction_estimation_with_curves.mp4",
    width: int = DEFAULT_VIDEO_WIDTH_PX,
    height: int = DEFAULT_VIDEO_HEIGHT_PX,
    fps: int = 30,
    panel_width: int = PANEL_WIDTH_PX,
    sensor_noise_seed: int | None = None,
    draw_arrow: bool = True,
) -> tuple[object, int]:
    """录制一次摩擦估计演示并输出带实时曲线的 MP4。

    Args:
        profile_path: DM_Gripper profile 路径或已解析的冻结 profile。
        task_path: 摩擦估计任务 YAML 路径。
        resolved_task: 已解析的冻结任务；传入时优先于 ``task_path``。
        output: 输出 MP4 路径。
        width: 画面总宽；右侧曲线面板占 ``panel_width``，其余为场景。
        height: 画面高。
        fps: 视频帧率，也是逐帧回调的仿真采样频率。
        panel_width: 右侧曲线面板宽度。
        sensor_noise_seed: 触觉噪声种子；None 使用 profile 默认。
        draw_arrow: 是否在场景中绘制当前切向外加载荷箭头。

    Returns:
        (实验结果, 录制帧数)。

    Raises:
        RuntimeError: 仿真结束前没有录到任何帧。
    """
    from .friction_estimation import FrictionEstimationTask, run_friction_estimation

    task = (
        resolved_task
        if isinstance(resolved_task, FrictionEstimationTask)
        else FrictionEstimationTask.load(task_path)
    )
    recorder = _DemoRecorder(
        kind="friction",
        output=output,
        width=width,
        height=height,
        fps=fps,
        panel_width=panel_width,
        draw_arrow=draw_arrow,
    )
    result = run_friction_estimation(
        profile_path,
        task=task,
        sensor_noise_seed=sensor_noise_seed,
        render_fps=fps,
        on_frame=recorder.on_frame,
    )
    recorder.finish(result)
    print(f"Video: {output}")
    print(f"Frames: {recorder.frame_count}")
    return result, recorder.frame_count


#: 可直接注册到 Typer/argparse 的两个录制入口。
DEMO_RECORDERS: dict[str, Callable[..., tuple[object, int]]] = {
    "ramp": record_force_tracking_ramp_video,
    "friction": record_friction_demo_video,
}


__all__ = [
    "DEFAULT_DEMOS_DIR",
    "DEMO_RECORDERS",
    "record_force_tracking_ramp_video",
    "record_friction_demo_video",
]

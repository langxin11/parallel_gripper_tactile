r"""把一次 dmgripper run 的 ``trace.csv`` 导出为高清数据曲线动画视频。

用 Matplotlib ``FuncAnimation`` + ``FFMpegWriter`` 按真实时间 1:1 逐帧渲染并直出
视频：五块面板（法向力、切向力、摩擦系数估计、估计刚度、位置与力矩）的曲线
随时间从左向右生长，全量数据以浅色"幽灵"预先铺在背景里作上下文，播放指针与
顶部阶段彩带指示当前时刻。默认画布 1440x1080（4:3），``--size`` 可改任意宽高。

样式复用 ``parallel_gripper_tactile.visualization.plotstyle`` 的论文版式；
能用数学符号表达处一律用符号（``$F_z$``、``$\\hat{\\mu}_L$``、``$q_{des}$`` 等），
仅阶段彩带与 t=0 说明等符号无法表达处保留中文。

输出白底 H.264 MP4（默认，剪辑通用）；如需把曲线**叠加**在实拍素材上合成，
加 ``--alpha`` 额外输出 Apple ProRes 4444 带透明通道的 ``.mov``（面板带半透明
白底，深浅背景都可读；H.264 不支持透明通道，故透明版只能用 ProRes 封装）。

时间轴从控制回路启动（``trace.csv`` 首行）重基为 0，与相机录像对齐时用
脚本打印的阶段边界锚点（如接触过渡起点）。

不修改数据本身：不做平滑、不删异常点，字符串列按数值解析，缺口保持断线。

用法：

```bash
# 默认 1440x1080 (4:3) @ 30 fps，只输出 mp4
uv run python scripts/analysis/export_curve_video.py \
    outputs/real/unified-adaptive/奶龙玩偶/20260920T143400Z-920a3853_待做视频

# 16:9 全高清 / 竖屏（适配竖拍素材）/ 4K
uv run python scripts/analysis/export_curve_video.py <run_dir> --size 1920x1080
uv run python scripts/analysis/export_curve_video.py <run_dir> --size 1080x1920
uv run python scripts/analysis/export_curve_video.py <run_dir> --size 3840x2160

# 额外输出 ProRes 4444 透明 MOV（叠合成用）/ 自定义质量
uv run python scripts/analysis/export_curve_video.py <run_dir> --alpha
uv run python scripts/analysis/export_curve_video.py <run_dir> --crf 14

# 只渲染若干预览帧自查版式，不编码
uv run python scripts/analysis/export_curve_video.py <run_dir> --preview
```
"""

from __future__ import annotations

import argparse
import colorsys
import json
from pathlib import Path

import numpy as np
import polars as pl

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUN = (
    ROOT
    / "outputs"
    / "real"
    / "unified-adaptive"
    / "奶龙玩偶"
    / "20260920T143400Z-920a3853_待做视频"
)

# 阶段英文名 -> （中文标签， 彩带底色）
PHASE_LABELS: dict[str, tuple[str, str]] = {
    "preparing": ("准备", "#d9d9d9"),
    "ready": ("就绪", "#e6e6e6"),
    "approach": ("接近", "#8dd3c7"),
    "contact_transition": ("接触过渡", "#fdb462"),
    "preload": ("预载", "#80b1d3"),
    "active": ("主动力控", "#b3de69"),
    "returning": ("返回", "#fb8072"),
}

BLACK = "#1a1a1a"
RED = "#d62728"
GREEN = "#2ca02c"
BLUE = "#1f77b4"
PINK = "#c084b8"

# 面板底色：透明 MOV 下给半透明白底保证可读，白底 MP4 下视觉等价于纯白。
PANEL_FACE = "white"
PANEL_FACE_ALPHA = {"opaque": 1.0, "transparent": 0.82}


def _nice_upper(values: list[np.ndarray], step: float, padding: float = 1.1) -> float:
    """以指定刻度向上取整数据上界，保留零点和适量留白。"""
    finite = np.concatenate([value[np.isfinite(value)] for value in values])
    if finite.size == 0:
        return step
    return max(step, np.ceil(float(finite.max()) * padding / step) * step)


def _column(df: pl.DataFrame, name: str) -> np.ndarray:
    """取一列为 float64 数组；字符串列先转数值，空串/缺失变 NaN。"""
    col = df[name]
    if col.dtype == pl.String:
        col = col.str.strip_chars().replace("", None).cast(pl.Float64, strict=False)
    else:
        col = col.cast(pl.Float64, strict=False)
    arr = col.to_numpy(allow_copy=True).astype(np.float64)
    arr[np.isinf(arr)] = np.nan
    return arr


def _flag(df: pl.DataFrame, name: str) -> np.ndarray:
    """取布尔列；字符串列按字面 True 解析。"""
    col = df[name]
    if col.dtype == pl.String:
        return (col.str.strip_chars().str.to_lowercase() == "true").to_numpy()
    if col.dtype == pl.Boolean:
        return col.fill_null(False).to_numpy()
    return col.to_numpy().astype(bool)


def _spans(active: np.ndarray, t: np.ndarray) -> list[tuple[float, float]]:
    """把布尔序列折叠成若干 (start, end) 区间。"""
    spans: list[tuple[float, float]] = []
    start: float | None = None
    for i, flag in enumerate(active):
        if flag and start is None:
            start = t[i]
        elif not flag and start is not None:
            spans.append((start, t[i]))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def _phase_spans(df: pl.DataFrame, t: np.ndarray) -> list[tuple[float, float, str]]:
    """把 phase 列折叠成 (start, end, 英文名) 序列。"""
    phases = df["phase"].to_list()
    spans: list[tuple[float, float, str]] = []
    for i, name in enumerate(phases):
        if spans and spans[-1][2] == name:
            spans[-1] = (spans[-1][0], t[i], name)
        else:
            spans.append((t[i], t[i], name))
    spans[-1] = (spans[-1][0], t[-1], spans[-1][2])
    return spans


def _darken(hex_color: str, factor: float = 0.55) -> str:
    hue, lightness, saturation = colorsys.rgb_to_hls(*_hex_to_rgb(hex_color))
    r, g, b = colorsys.hls_to_rgb(hue, lightness * factor, saturation)
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def _hex_to_rgb(hex_color: str) -> tuple[float, float, float]:
    hex_color = hex_color.lstrip("#")
    return tuple(int(hex_color[i : i + 2], 16) / 255.0 for i in (0, 2, 4))  # type: ignore[return-value]


class CurveVideo:
    """负责版式搭建与逐帧更新；编码由 main 经 FuncAnimation 编排。"""

    def __init__(self, run_dir: Path, width: int, height: int, fps: int) -> None:
        """读取运行数据并初始化视频画布。

        Args:
            run_dir: 包含 ``trace.csv`` 的运行产物目录。
            width: 画布宽度（像素）。
            height: 画布高度（像素）。
            fps: 视频帧率。
        """
        self.run_dir = run_dir
        self.width = width
        self.height = height
        self.fps = fps
        self.dpi = 100.0
        self._load()
        self._build_figure()

    # ------------------------------------------------------------------ 数据
    def _load(self) -> None:
        df = pl.read_csv(self.run_dir / "trace.csv")
        t = _column(df, "time_s")
        t = t - t[0]
        self.t = t
        self.duration = float(t[-1])
        self.phases = _phase_spans(df, t)

        self.series: dict[str, np.ndarray] = {
            "measured_fz": _column(df, "measured_force_n"),
            "target_fz": _column(df, "target_force_n"),
            "left_fz": _column(df, "left_fz_n"),
            "right_fz": _column(df, "right_fz_n"),
            "left_ft": np.hypot(_column(df, "raw_left_fx_n"), _column(df, "raw_left_fy_n")),
            "right_ft": np.hypot(_column(df, "raw_right_fx_n"), _column(df, "raw_right_fy_n")),
            "mu_left": _column(df, "adaptive_left_mu"),
            "mu_right": _column(df, "adaptive_right_mu"),
            "mu_left_lb": _column(df, "adaptive_left_mu_lower_bound"),
            "mu_right_lb": _column(df, "adaptive_right_mu_lower_bound"),
            "mu_left_rej": _column(df, "adaptive_left_candidate"),
            "mu_right_rej": _column(df, "adaptive_right_candidate"),
            "K_hat": _column(df, "stiffness_n_per_m"),
            "q": _column(df, "position_rad"),
            "q_des": _column(df, "q_des_rad"),
            "tau": _column(df, "torque_nm"),
        }
        self.trigger_spans = _spans(_flag(df, "target_trigger_active"), t)

    # ------------------------------------------------------------------ 版式
    def _build_figure(self) -> None:
        import matplotlib as mpl
        from parallel_gripper_tactile.visualization.plotstyle import science_pyplot

        self.plt = science_pyplot(font_scale=1.5)
        # 论文版式按 600 dpi 落盘校准；逐帧渲染必须钉死 dpi 与画布，
        # 否则 science 样式的 figure.dpi=600 / savefig.bbox=tight 会让每帧尺寸漂移。
        mpl.rcParams.update(
            {
                "figure.dpi": self.dpi,
                "savefig.dpi": self.dpi,
                "savefig.bbox": "standard",
                "savefig.pad_inches": 0.0,
                "axes.unicode_minus": False,
                "figure.facecolor": PANEL_FACE,
            }
        )

        w_in, h_in = self.width / self.dpi, self.height / self.dpi
        self.fig = self.plt.figure(figsize=(w_in, h_in), dpi=self.dpi)
        # 右侧留出 twinx 轴标题的空间；彩带、标题与坐标区右缘对齐。
        left, right = 0.058, 0.950
        # 顶部阶段彩带
        self.ribbon_ax = self.fig.add_axes([left, 0.925, right - left, 0.030])
        self._build_ribbon()
        # 标题
        self.title_text = self.fig.text(left, 0.982, self.title(), ha="left", va="top", fontsize=16)
        self.time_text = self.fig.text(
            right, 0.982, "", ha="right", va="top", fontsize=16, color="#333333"
        )
        # 符号无法表达的计时约定，用小号脚注说明。
        self.fig.text(
            right,
            0.012,
            "t = 0 对应控制回路启动",
            ha="right",
            va="bottom",
            fontsize=11,
            color="#777777",
        )

        gs = self.fig.add_gridspec(
            5, 1, left=left, right=right, top=0.905, bottom=0.055, hspace=0.34
        )
        self.axes: list = []
        self.extra_axes: list = []
        self.panels = [
            self._panel_force,
            self._panel_tangential,
            self._panel_friction,
            self._panel_stiffness,
            self._panel_motion,
        ]
        for row, build in enumerate(self.panels):
            ax = self.fig.add_subplot(gs[row, 0])
            ax.tick_params(labelbottom=(row == 4))
            self.axes.append(ax)
            build(ax, row == 4)

        self._ghosts()
        # 播放指针：每块面板内一条 axvline（figure 级线条会被面板白底盖住）。
        play_kw = {"color": "#3a3a3a", "lw": 1.6, "alpha": 0.9, "zorder": 6}
        self.playlines = [ax.axvline(0.0, **play_kw) for ax in self.axes]
        self.playlines += [ax.axvline(0.0, **play_kw) for ax in self.extra_axes]

    def title(self) -> str:
        """生成包含运行元数据、物体名和时间戳的图表标题。"""
        # manifest 不带任务/物体名（由 --metadata 写入时才有），退回目录结构：
        # 父目录 = 物体名，目录名前缀 = UTC 时间戳。
        config_path = self.run_dir / "config.json"
        parts = ["统一自适应抓取"]
        if config_path.exists():
            config = json.loads(config_path.read_text())
            # 硬件运行产物把元数据放在 effective.metadata；旧格式则在顶层。
            effective = config.get("effective")
            metadata = effective.get("metadata", {}) if isinstance(effective, dict) else {}
            task = str(config.get("task_name") or metadata.get("task_name") or "")
            obj = str(config.get("object_name") or metadata.get("object_name") or "")
            parts += [p for p in (obj, task) if p and p != "None"]
        parent = self.run_dir.parent.name
        if parent and parent not in parts:
            parts.append(parent)
        stamp = self.run_dir.name[:15]
        if len(stamp) == 15 and stamp[8] == "T" and (stamp[:8] + stamp[9:]).isdigit():
            parts.append(f"{stamp[:4]}-{stamp[4:6]}-{stamp[6:8]} {stamp[9:11]}:{stamp[11:13]} UTC")
        return " · ".join(parts)

    # ------------------------------------------------------------------ 面板
    def _style_panel(self, ax, ylabel: str, show_xlabel: bool) -> None:
        ax.set_ylabel(ylabel)
        ax.set_xlim(0.0, self.duration * 1.01)
        if show_xlabel:
            ax.set_xlabel("$t$ (s)")

    def _panel_force(self, ax, show_xlabel: bool) -> None:
        self._style_panel(ax, "$F_z$ (N)", show_xlabel)
        s = self.series
        self.l_left_fz = self._line(ax, "$F_z^L$", "#9c9c9c", "-", 1.1)
        self.l_right_fz = self._line(ax, "$F_z^R$", "#c4c4c4", "-", 1.1)
        self.l_meas = self._line(ax, "$F_z$", BLACK, "-", 2.3)
        self.l_tgt = self._line(ax, "$F_z^{ref}$", GREEN, "--", 2.0)
        ax.set_ylim(
            0.0, _nice_upper([s["left_fz"], s["right_fz"], s["measured_fz"], s["target_fz"]], 1.0)
        )
        handles = [self.l_meas, self.l_tgt, self.l_left_fz, self.l_right_fz]
        ax.legend(
            handles, [h.get_label() for h in handles], loc="upper left", ncol=2, framealpha=0.6
        )

    def _panel_tangential(self, ax, show_xlabel: bool) -> None:
        self._style_panel(ax, "$F_T$ (N)", show_xlabel)
        self.trigger_patches = []
        for x0, x1 in self.trigger_spans:
            patch = ax.axvspan(x0, x1, color="#b3de69", alpha=0.0, lw=0, zorder=0)
            self.trigger_patches.append((patch, x0, x1))
        self.l_left_ft = self._line(ax, "$F_T^L$", BLACK, "-", 1.8)
        self.l_right_ft = self._line(ax, "$F_T^R$", RED, "--", 1.8)
        ax.set_ylim(0.0, _nice_upper([self.series["left_ft"], self.series["right_ft"]], 0.25))
        import matplotlib.patches as mpatches

        trigger_handle = mpatches.Patch(
            facecolor="#b3de69", alpha=0.35, lw=0, label="$\\mathrm{trigger}$"
        )
        handles = [self.l_left_ft, self.l_right_ft, trigger_handle]
        ax.legend(
            handles,
            [h.get_label() for h in handles],
            loc="upper left",
            ncol=3,
            framealpha=0.6,
            columnspacing=1.0,
        )

    def _panel_friction(self, ax, show_xlabel: bool) -> None:
        self._style_panel(ax, "$\\hat{\\mu}$", show_xlabel)
        self.l_mu_l_lb = self._line(ax, "$\\mu_L^{obs}$", BLACK, ":", 1.0)
        self.l_mu_r_lb = self._line(ax, "$\\mu_R^{obs}$", RED, ":", 1.0)
        self.l_mu_l = self._line(ax, "$\\hat{\\mu}_L$", BLACK, "-", 2.1)
        self.l_mu_r = self._line(ax, "$\\hat{\\mu}_R$", RED, "--", 2.1)
        self.s_rej_l = ax.scatter(
            [], [], marker="x", color=BLACK, s=46, zorder=5, label="$\\mu_L^{cand}$"
        )
        self.s_rej_r = ax.scatter(
            [], [], marker="x", color=RED, s=46, zorder=5, label="$\\mu_R^{cand}$"
        )
        ax.set_ylim(
            0.0,
            _nice_upper(
                [
                    self.series["mu_left"],
                    self.series["mu_right"],
                    self.series["mu_left_lb"],
                    self.series["mu_right_lb"],
                ],
                0.1,
            ),
        )
        ax.legend(loc="upper left", ncol=3, framealpha=0.6, columnspacing=1.0, handlelength=1.6)

    def _panel_stiffness(self, ax, show_xlabel: bool) -> None:
        self._style_panel(ax, "$\\hat{K}$ (N/m)", show_xlabel)
        self.l_k = self._line(ax, "$\\hat{K}$", BLACK, "-", 2.1)
        # 刚度量级会随物体和接触状态变化。以全部有效样本向外取整到 500 N/m，
        # 既不截断峰值，也避免固定大范围把当前实验中的变化压扁。
        k = self.series["K_hat"]
        finite_k = k[np.isfinite(k)]
        if finite_k.size:
            lower = max(0.0, np.floor(finite_k.min() / 500.0) * 500.0)
            upper = np.ceil(finite_k.max() * 1.1 / 500.0) * 500.0
            if upper <= lower:
                upper = lower + 500.0
            ax.set_ylim(lower, upper)
        ax.legend(loc="upper left", framealpha=0.6)

    def _panel_motion(self, ax, show_xlabel: bool) -> None:
        self._style_panel(ax, "$q$ (rad)", show_xlabel)
        ax_r = ax.twinx()
        ax_r.grid(False)
        self.extra_axes.append(ax_r)
        self.l_tau = self._line(ax_r, "$\\tau_m$", PINK, ":", 1.6)
        self.l_q = self._line(ax, "$q$", BLACK, "-", 2.1)
        self.l_qdes = self._line(ax, "$q_{des}$", BLUE, "--", 2.0)
        ax.set_ylim(0.0, _nice_upper([self.series["q"], self.series["q_des"]], 0.25, padding=1.02))
        tau_limit = _nice_upper([np.abs(self.series["tau"])], 0.25)
        ax_r.set_ylim(-tau_limit, tau_limit)
        ax_r.set_ylabel("$\\tau_m$ (N·m)")
        handles = [self.l_q, self.l_qdes, self.l_tau]
        ax.legend(
            handles,
            [h.get_label() for h in handles],
            loc="upper left",
            ncol=3,
            framealpha=0.6,
            columnspacing=1.0,
            handlelength=1.6,
        )

    def _line(self, ax, label: str, color: str, style: str, lw: float):
        (artist,) = ax.plot([], [], style, color=color, lw=lw, label=label, solid_capstyle="round")
        return artist

    def _build_ribbon(self) -> None:
        ax = self.ribbon_ax
        ax.set_xlim(0.0, self.duration)
        ax.set_ylim(0, 1)
        ax.set_xticks([])
        ax.set_yticks([])
        for side in ("top", "right", "left", "bottom"):
            ax.spines[side].set_visible(False)
        self.ribbon_patches = []
        # 彩带宽度不足以容纳文字的窄段（如接触过渡）只画色块不写字，避免压邻段。
        ribbon_px = self.fig.get_size_inches()[0] * self.dpi * 0.892
        min_label_px = 45.0
        for x0, x1, name in self.phases:
            label, color = PHASE_LABELS.get(name, (name, "#dddddd"))
            patch = ax.axvspan(x0, x1, ymin=0.0, ymax=1.0, color=color, alpha=0.0, lw=0)
            width_px = (x1 - x0) / self.duration * ribbon_px
            text = ax.text(
                (x0 + x1) / 2,
                0.5,
                label,
                ha="center",
                va="center",
                fontsize=12,
                color=_darken(color),
                fontweight="bold",
                visible=width_px >= min_label_px,
            )
            self.ribbon_patches.append((patch, text, x0, x1, color))

    def _ghosts(self) -> None:
        """全量数据浅色铺底，提供上下文；当前进度用实色线覆盖。"""
        t = self.t
        ghost_specs = [
            (self.l_left_fz, "left_fz"),
            (self.l_right_fz, "right_fz"),
            (self.l_meas, "measured_fz"),
            (self.l_tgt, "target_fz"),
            (self.l_left_ft, "left_ft"),
            (self.l_right_ft, "right_ft"),
            (self.l_mu_l, "mu_left"),
            (self.l_mu_r, "mu_right"),
            (self.l_k, "K_hat"),
            (self.l_q, "q"),
            (self.l_qdes, "q_des"),
            (self.l_tau, "tau"),
        ]
        for artist, key in ghost_specs:
            y = self.series[key]
            artist.axes.plot(
                t,
                y,
                color=artist.get_color(),
                lw=artist.get_linewidth() * 0.9,
                ls=artist.get_linestyle(),
                alpha=0.14,
                solid_capstyle="round",
                zorder=1,
            )
        self._scatters = (
            ("mu_left_rej", self.s_rej_l),
            ("mu_right_rej", self.s_rej_r),
        )

    # ------------------------------------------------------------------ 逐帧
    def update(self, now: float) -> None:
        """更新指定时刻之前的曲线、阶段彩带和播放指针。

        Args:
            now: 当前播放时间（秒）。
        """
        t = self.t

        def reveal(artist, key: str) -> None:
            m = t <= now
            if not m.any():
                artist.set_data([], [])
                return
            y = self.series[key]
            artist.set_data(t[m], y[m])

        reveal(self.l_left_fz, "left_fz")
        reveal(self.l_right_fz, "right_fz")
        reveal(self.l_meas, "measured_fz")
        reveal(self.l_tgt, "target_fz")
        reveal(self.l_left_ft, "left_ft")
        reveal(self.l_right_ft, "right_ft")
        reveal(self.l_mu_l_lb, "mu_left_lb")
        reveal(self.l_mu_r_lb, "mu_right_lb")
        reveal(self.l_mu_l, "mu_left")
        reveal(self.l_mu_r, "mu_right")
        reveal(self.l_k, "K_hat")
        reveal(self.l_q, "q")
        reveal(self.l_qdes, "q_des")
        reveal(self.l_tau, "tau")

        for key, scatter in self._scatters:
            y = self.series[key]
            m = np.isfinite(y) & (t <= now)
            scatter.set_offsets(np.column_stack([t[m], y[m]]) if m.any() else np.empty((0, 2)))

        for patch, x0, x1 in self.trigger_patches:
            if x0 > now:
                patch.set_alpha(0.0)
            else:
                patch.set_alpha(0.22)
                patch.set_x(x0)
                patch.set_width(min(x1, now) - x0)

        for patch, text, x0, x1, color in self.ribbon_patches:
            active = x0 <= now <= x1
            patch.set_alpha(0.85 if active else 0.30)
            text.set_color(_darken(color) if active else "#666666")
            text.set_fontweight("bold" if active else "normal")

        for line in self.playlines:
            line.set_xdata([now, now])
        self.time_text.set_text(f"t = {now:5.2f} s")

    def set_face_mode(self, mode: str) -> None:
        """opaque：白底 MP4；transparent：透明 ProRes，面板半透明白底。"""
        self.fig.patch.set_alpha(PANEL_FACE_ALPHA.get(mode, 1.0))
        for ax in [*self.axes, *self.extra_axes, self.ribbon_ax]:
            ax.patch.set_facecolor(PANEL_FACE)
            ax.patch.set_alpha(PANEL_FACE_ALPHA.get(mode, 1.0))

    def close(self) -> None:
        """关闭 Matplotlib 图形并释放其资源。"""
        self.plt.close(self.fig)


def save_animation(
    video: CurveVideo,
    out_path: Path,
    fps: int,
    crf: int,
    transparent: bool,
    title: str,
    total_frames: int,
) -> None:
    """FuncAnimation + FFMpegWriter 直出一段视频；透明模式走 ProRes 4444。"""
    from matplotlib.animation import FFMpegWriter, FuncAnimation

    video.set_face_mode("transparent" if transparent else "opaque")
    anim = FuncAnimation(
        video.fig,
        lambda i: video.update(min(video.duration, i / fps)),
        init_func=lambda: [],
        frames=total_frames,
        blit=False,
        interval=1000.0 / fps,
    )
    if transparent:
        writer = FFMpegWriter(
            fps=fps,
            codec="prores_ks",
            metadata={"title": title},
            extra_args=["-profile:v", "4444", "-pix_fmt", "yuva444p10le", "-vendor", "apl0"],
        )
    else:
        writer = FFMpegWriter(
            fps=fps,
            codec="libx264",
            metadata={"title": title},
            extra_args=[
                "-preset",
                "slow",
                "-crf",
                str(crf),
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
            ],
        )

    def progress(frames_done: int, total: int) -> None:
        if frames_done % 120 == 0 or frames_done == total:
            print(f"  {frames_done}/{total}")

    anim.save(out_path, writer=writer, progress_callback=progress)
    anim.event_source.stop()


def main() -> None:
    """解析命令行参数，并导出动画或预览帧。"""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "run_dir", nargs="?", type=Path, default=DEFAULT_RUN, help="包含 trace.csv 的 run 产物目录"
    )
    parser.add_argument("--fps", type=int, default=30, help="视频帧率，默认 30")
    parser.add_argument(
        "--size",
        default="1440x1080",
        help="画布宽x高，默认 1440x1080（4:3）；"
        "16:9 用 1920x1080，竖屏用 1080x1920，4K 用 3840x2160",
    )
    parser.add_argument("--crf", type=int, default=15, help="H.264 质量因子，越小越清晰，默认 15")
    parser.add_argument(
        "--hold-end", type=float, default=0.8, help="结尾定格秒数，便于剪辑收尾，默认 0.8"
    )
    parser.add_argument(
        "--alpha", action="store_true", help="额外输出 ProRes 4444 透明 MOV（叠合成用，体积大）"
    )
    parser.add_argument("--out-dir", type=Path, default=None, help="输出目录，默认写回 run 目录")
    parser.add_argument(
        "--preview", action="store_true", help="只渲染 5 张预览帧自查版式，不编码视频"
    )
    args = parser.parse_args()

    run_dir: Path = args.run_dir
    if not (run_dir / "trace.csv").exists():
        parser.error(f"{run_dir} 下没有 trace.csv")
    out_dir: Path = args.out_dir or run_dir

    try:
        w_str, h_str = args.size.lower().split("x")
        width, height = int(w_str), int(h_str)
        assert width >= 240 and height >= 240 and width % 2 == 0 and height % 2 == 0
    except (ValueError, AssertionError):
        parser.error(f"--size 期望 偶数宽x偶数高，如 1440x1080，收到 {args.size}")

    video = CurveVideo(run_dir, width, height, args.fps)
    duration, fps, hold = video.duration, args.fps, args.hold_end
    title = video.title()
    total_frames = int(round(duration * fps)) + int(round(hold * fps))

    # 对齐锚点：阶段边界（重基后的秒数），供剪辑时与相机素材对位。
    print("阶段边界（t 从控制开始计时）：")
    for x0, x1, name in video.phases:
        label = PHASE_LABELS.get(name, (name,))[0]
        print(f"  {label:　<4} {name:20s} {x0:7.2f} s -> {x1:7.2f} s")

    if args.preview:
        picks = [0.0, 5.4, 10.0, 17.0, duration]
        for now in picks:
            video.update(min(now, duration))
            path = out_dir / f"curve_preview_{now:05.2f}s.png"
            video.fig.savefig(path, dpi=video.dpi)
            print(f"预览帧已写出：{path}")
        video.close()
        return

    stem = f"curve_video_{height}p{fps}"
    print(f"渲染 {total_frames} 帧（{width}x{height} @ {fps} fps）：MP4 白底 ...")
    save_animation(
        video,
        out_dir / f"{stem}.mp4",
        fps,
        args.crf,
        transparent=False,
        title=title,
        total_frames=total_frames,
    )
    if args.alpha:
        print("继续渲染：ProRes 4444 透明 MOV ...")
        save_animation(
            video,
            out_dir / f"{stem}_alpha.mov",
            fps,
            args.crf,
            transparent=True,
            title=title,
            total_frames=total_frames,
        )
        print(f"完成：\n  {out_dir / f'{stem}.mp4'}\n  {out_dir / f'{stem}_alpha.mov'}")
    else:
        print(f"完成：\n  {out_dir / f'{stem}.mp4'}")
    video.close()


if __name__ == "__main__":
    main()

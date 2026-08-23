"""触觉仿真步骤的统一采样、CSV 记录、Rerun 可视化与演示驱动循环。"""

from __future__ import annotations

import csv
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import time

import mujoco
import numpy as np


@dataclass(frozen=True)
class TactileFrame:
    """一个物理步上的左右触觉表面受力。

    ``left`` 与 ``right`` 均采用 ``(3, rows, cols)`` 的 xyz 通道顺序，
    表达在各自触觉 site 的局部坐标系中。方向统一为物体施加给触觉表面的力，
    因而压缩时局部 Fz 为正。
    """

    step: int
    time_s: float
    control: float
    left: np.ndarray
    right: np.ndarray

    def __post_init__(self) -> None:
        """归一化并校验左右触觉网格形状。"""
        object.__setattr__(self, "left", _force_grid(self.left, "left"))
        object.__setattr__(self, "right", _force_grid(self.right, "right"))

    @property
    def left_force(self) -> np.ndarray:
        """返回左侧完整网格的局部三维合力。"""
        return self.left.sum(axis=(1, 2))

    @property
    def right_force(self) -> np.ndarray:
        """返回右侧完整网格的局部三维合力。"""
        return self.right.sum(axis=(1, 2))


def _force_grid(values, side: str) -> np.ndarray:
    grid = np.asarray(values, dtype=np.float64)
    if grid.ndim != 3 or grid.shape[0] != 3 or 0 in grid.shape[1:]:
        raise ValueError(f"{side} 触觉数据必须具有 (3, rows, cols) 形状。")
    return grid


def run_demo_loop(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    steps: int,
    auto_close: bool,
    no_viewer: bool,
    render_fps: float,
    control_at: Callable[[int], float],
    sample_frame: Callable[[int], TactileFrame],
    recorder: ForceCsvRecorder,
    rerun_logger: RerunTactileLogger,
) -> None:
    """运行抓取演示主循环：viewer 节流、物理推进、采样与记录。

    渲染与物理步解耦：降低 FPS 不会降低仿真时间或物理精度。手动模式
    （``auto_close=False``）下控制量由 viewer 滑块写入 ``data.ctrl``，
    循环不覆盖它；自动闭合模式下每步调用 ``control_at`` 计算控制量。
    每步依次调用 ``sample_frame`` 采样、``recorder`` 与 ``rerun_logger``
    记录，循环结束后统一清理三者。
    """
    viewer = None
    if not no_viewer:
        # 用 from-import 避免在函数内绑定局部名 mujoco，遮蔽模块级导入。
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = (0.0, -0.12, 0.07)
        viewer.cam.distance = 0.38
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25

    try:
        step = 0
        wall_start = time.monotonic()
        simulation_start = data.time
        render_period = 1.0 / render_fps
        next_render_time = wall_start
        while viewer is None or viewer.is_running():
            if viewer is not None:
                target_simulation_time = simulation_start + (time.monotonic() - wall_start)
                while data.time < target_simulation_time:
                    if auto_close:
                        data.ctrl[0] = control_at(step)
                    mujoco.mj_step(model, data)
                    frame = sample_frame(step)
                    recorder.record(frame)
                    rerun_logger.record(frame)
                    if step % 100 == 0 or step == steps - 1:
                        print(
                            f"step={step:4d} ctrl={data.ctrl[0]:6.1f} "
                            f"left={frame.left_force[2]:8.3f} N "
                            f"right={frame.right_force[2]:8.3f} N"
                        )
                    step += 1
                    if auto_close and step >= steps:
                        break
                viewer.sync()
                if auto_close and step >= steps:
                    break
                next_render_time += render_period
                time.sleep(max(0.0, next_render_time - time.monotonic()))
            else:
                if auto_close:
                    data.ctrl[0] = control_at(step)
                mujoco.mj_step(model, data)
                frame = sample_frame(step)
                recorder.record(frame)
                rerun_logger.record(frame)
                if step % 100 == 0 or step == steps - 1:
                    print(
                        f"step={step:4d} ctrl={data.ctrl[0]:6.1f} "
                        f"left={frame.left_force[2]:8.3f} N "
                        f"right={frame.right_force[2]:8.3f} N"
                    )
                step += 1
                if step >= steps:
                    break
    finally:
        recorder.close()
        rerun_logger.close()
        # 用户手动关闭窗口时 viewer 已请求退出；避免对已经销毁的 GLFW
        # 上下文再次调用 close，从而触发退出阶段的 GLFW 警告。
        if viewer is not None and viewer.is_running():
            viewer.close()


def aggregate_shear(
    tactile: np.ndarray, max_rows: int = 8, max_cols: int = 8
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """将切向力按空间分块求和，返回箭头原点、向量和法向压力。

    该降采样只服务于 Rerun 箭头显示；完整三通道张量仍会原样记录。
    分块求和保持总切向力与总法向力不变。
    """
    grid = _force_grid(tactile, "tactile")
    if max_rows <= 0 or max_cols <= 0:
        raise ValueError("箭头网格尺寸必须为正整数。")

    rows, cols = grid.shape[1:]
    row_groups = np.array_split(np.arange(rows), min(rows, max_rows))
    col_groups = np.array_split(np.arange(cols), min(cols, max_cols))
    origins: list[tuple[float, float]] = []
    vectors: list[tuple[float, float]] = []
    pressures: list[float] = []
    for display_row, row_indices in enumerate(row_groups):
        for display_col, col_indices in enumerate(col_groups):
            block = grid[:, row_indices[:, None], col_indices]
            block_force = block.sum(axis=(1, 2))
            origins.append((float(display_col), float(len(row_groups) - 1 - display_row)))
            vectors.append((float(block_force[0]), float(block_force[1])))
            pressures.append(float(block_force[2]))
    return np.asarray(origins), np.asarray(vectors), np.asarray(pressures)


class ForceCsvRecorder:
    """将控制量及左右触觉表面合力记录为可直接绘图的 CSV。"""

    fieldnames = (
        "step",
        "time_s",
        "control",
        "left_fx",
        "left_fy",
        "left_fz",
        "right_fx",
        "right_fy",
        "right_fz",
    )

    def __init__(self, path: Path | None, every: int = 1) -> None:
        """初始化 CSV 记录器；``path`` 为 None 时静默禁用。"""
        if every <= 0:
            raise ValueError("记录间隔必须为正整数。")
        self._every = every
        self._file = None
        self._writer = None
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._file, fieldnames=self.fieldnames)
            self._writer.writeheader()

    def record(self, frame: TactileFrame) -> None:
        """记录一次物理步；此方法可作为仿真循环中的记录钩子调用。"""
        if self._writer is None or frame.step % self._every:
            return
        left_force = frame.left_force
        right_force = frame.right_force
        self._writer.writerow(
            {
                "step": frame.step,
                "time_s": frame.time_s,
                "control": frame.control,
                "left_fx": left_force[0],
                "left_fy": left_force[1],
                "left_fz": left_force[2],
                "right_fx": right_force[0],
                "right_fy": right_force[1],
                "right_fz": right_force[2],
            }
        )

    def close(self) -> None:
        """关闭记录文件。"""
        if self._file is not None:
            self._file.close()
            self._file = None


class RerunTactileLogger:
    """将统一触觉帧流式发送到 Rerun Viewer 和/或 RRD 文件。"""

    _SIDE_COLORS = {"left": (31, 119, 180), "right": (255, 127, 14)}
    _COMPONENT_COLORS = {"fx": (214, 39, 40), "fy": (44, 160, 44), "fz": (31, 119, 180)}

    def __init__(
        self,
        application_id: str,
        *,
        live: bool = True,
        path: Path | None = None,
        hz: float = 100.0,
        pressure_max: float | None = None,
    ) -> None:
        """配置 Rerun 输出：实时 Viewer、RRD 文件或两者。"""
        if hz <= 0:
            raise ValueError("Rerun 记录频率必须为正数。")
        if pressure_max is not None and pressure_max <= 0:
            raise ValueError("压力显示上限必须为正数。")
        self._period_s = 1.0 / hz
        self._next_time_s: float | None = None
        self._pressure_range = (0.0, pressure_max) if pressure_max is not None else None
        self._logged_shear_grids: set[tuple[str, int, int]] = set()
        self._recording = None
        if not live and path is None:
            return

        import rerun as rr

        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
        recording = rr.RecordingStream(application_id)
        blueprint = _rerun_blueprint()
        if live:
            recording.spawn()
            if path is not None:
                recording.set_sinks(rr.GrpcSink(), rr.FileSink(path), default_blueprint=blueprint)
            else:
                recording.send_blueprint(blueprint)
        elif path is not None:
            recording.set_sinks(rr.FileSink(path), default_blueprint=blueprint)
        self._recording = recording
        self._log_static_styles(rr)

    @property
    def enabled(self) -> bool:
        """是否配置了实时或文件 Rerun 输出。"""
        return self._recording is not None

    def _log_static_styles(self, rr) -> None:
        if self._recording is None:
            return
        self._recording.log(
            "control/value",
            rr.SeriesLines(colors=[(127, 127, 127)], names=["control"]),
            static=True,
        )
        for side, side_color in self._SIDE_COLORS.items():
            for component, component_color in self._COMPONENT_COLORS.items():
                self._recording.log(
                    f"tactile/{side}/force/{component}",
                    rr.SeriesLines(colors=[component_color], names=[f"{side} {component.upper()}"]),
                    static=True,
                )
            self._recording.log(
                f"tactile/{side}/total_force",
                rr.SeriesLines(colors=[side_color], names=[f"{side} Fz"]),
                static=True,
            )

    def record(self, frame: TactileFrame) -> None:
        """按仿真时间频率记录完整触觉帧。"""
        if self._recording is None:
            return

        if self._next_time_s is None:
            self._next_time_s = frame.time_s
        tolerance = max(1e-12, self._period_s * 1e-9)
        if frame.time_s + tolerance < self._next_time_s:
            return
        while self._next_time_s <= frame.time_s + tolerance:
            self._next_time_s += self._period_s

        import rerun as rr

        self._recording.set_time("step", sequence=frame.step)
        self._recording.set_time("sim_time", duration=frame.time_s)
        self._recording.log("control/value", rr.Scalars(frame.control))
        for side, tactile, total in (
            ("left", frame.left, frame.left_force),
            ("right", frame.right, frame.right_force),
        ):
            root = f"tactile/{side}"
            self._recording.log(
                f"{root}/raw",
                rr.Tensor(tactile, dim_names=("component", "row", "column")),
            )
            self._recording.log(
                f"{root}/pressure",
                rr.Tensor(
                    tactile[2],
                    dim_names=("row", "column"),
                    value_range=self._pressure_range,
                ),
            )
            origins, vectors, pressures = aggregate_shear(tactile)
            display_vectors = _normalize_arrow_vectors(vectors)
            display_rows = min(tactile.shape[1], 8)
            display_cols = min(tactile.shape[2], 8)
            grid_key = (side, display_rows, display_cols)
            if grid_key not in self._logged_shear_grids:
                self._recording.log(
                    f"{root}/shear/grid",
                    rr.LineStrips2D(
                        _shear_grid_lines(display_rows, display_cols),
                        colors=[(105, 115, 125)],
                        radii=0.01,
                        draw_order=-1.0,
                    ),
                    static=True,
                )
                self._logged_shear_grids.add(grid_key)
            self._recording.log(
                f"{root}/shear",
                rr.Arrows2D(
                    origins=origins,
                    vectors=display_vectors,
                    colors=_pressure_colors(pressures),
                    radii=0.025,
                ),
            )
            for component, value in zip(("fx", "fy", "fz"), total, strict=True):
                self._recording.log(f"{root}/force/{component}", rr.Scalars(float(value)))
            self._recording.log(f"{root}/total_force", rr.Scalars(float(total[2])))

    def close(self) -> None:
        """刷新并关闭 Rerun 的 gRPC 与文件输出。"""
        if self._recording is not None:
            self._recording.flush()
            self._recording.disconnect()
            self._recording = None


def _normalize_arrow_vectors(vectors: np.ndarray) -> np.ndarray:
    """将单帧最长切向箭头缩放到 0.45 个显示网格，保留方向和相对大小。"""
    if len(vectors) == 0:
        return vectors
    maximum = float(np.linalg.norm(vectors, axis=1).max())
    if maximum <= np.finfo(float).eps:
        return np.zeros_like(vectors)
    return vectors * (0.45 / maximum)


def _shear_grid_lines(rows: int, cols: int) -> list[np.ndarray]:
    """返回以整数格点为中心的二维网格边界线。"""
    if rows <= 0 or cols <= 0:
        raise ValueError("切向显示网格尺寸必须为正整数。")
    left, right = -0.5, cols - 0.5
    bottom, top = -0.5, rows - 0.5
    lines = [
        np.asarray(((x - 0.5, bottom), (x - 0.5, top)), dtype=np.float32) for x in range(cols + 1)
    ]
    lines.extend(
        np.asarray(((left, y - 0.5), (right, y - 0.5)), dtype=np.float32) for y in range(rows + 1)
    )
    return lines


def _pressure_colors(pressures: np.ndarray) -> np.ndarray:
    """将单帧法向压力映射为绿到红的 Rerun 箭头颜色。"""
    positive = np.clip(pressures, 0.0, None)
    maximum = float(positive.max(initial=0.0))
    normalized = positive / maximum if maximum > np.finfo(float).eps else positive
    colors = np.zeros((len(pressures), 3), dtype=np.uint8)
    colors[:, 0] = np.rint(255 * normalized).astype(np.uint8)
    colors[:, 1] = np.rint(255 * (1 - normalized)).astype(np.uint8)
    return colors


def _rerun_blueprint():
    """创建两个演示共享的触觉仪表盘布局。"""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.TensorView(origin="tactile/left/pressure", name="Left pressure Fz"),
                rrb.TensorView(origin="tactile/right/pressure", name="Right pressure Fz"),
                rrb.Spatial2DView(origin="tactile/left/shear", name="Left shear Fx/Fy"),
                rrb.Spatial2DView(origin="tactile/right/shear", name="Right shear Fx/Fy"),
                column_shares=[1, 1, 1, 1],
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(origin="control", name="Control"),
                rrb.TimeSeriesView(origin="tactile/left/force", name="Left Fx/Fy/Fz"),
                rrb.TimeSeriesView(origin="tactile/right/force", name="Right Fx/Fy/Fz"),
            ),
            row_shares=[1, 1],
        ),
        collapse_panels=True,
    )

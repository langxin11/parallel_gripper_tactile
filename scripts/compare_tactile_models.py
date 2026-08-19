"""在相同抓取条件下比较 3×3 box taxel 与 3×3 touch_grid。

常见用法::

    uv run scripts/compare_tactile_models.py
    uv run scripts/compare_tactile_models.py --output-csv outputs/comparison.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from statistics import fmean

from grasp_scene import gripper_name_in_model, load_grasp_model, prefixed_gripper_name
from run_cube_grasp_demo import _taxel_surface_force_vector
from run_touch_grid_demo import _read_tactile, _touch_grid_shape


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOX_TAXEL_XML = REPOSITORY_ROOT / "assets/grippers/robotiq_2f85/2f85_taxels_box.xml"
DEFAULT_TOUCH_GRID_XML = REPOSITORY_ROOT / "assets/grippers/robotiq_2f85/2f85_touch_grid_3x3.xml"
DEFAULT_OUTPUT_CSV = REPOSITORY_ROOT / "outputs/tactile_model_comparison.csv"
SUPPORT_GEOM_NAME = "target_cube_support_plate"
CUBE_BODY_NAME = "cube/target_cube"
CUBE_JOINT_NAME = "cube/target_cube_free_joint"


Vector3 = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class DisturbanceProtocol:
    """抓稳、撤去支撑、施加切向正弦力并观察恢复的实验协议。"""

    close_duration: float = 1.0
    support_settle_duration: float = 0.5
    release_settle_duration: float = 0.5
    disturbance_duration: float = 1.0
    recovery_duration: float = 0.5
    force_n: float = 5.0
    frequency_hz: float = 2.0
    slip_threshold_m: float = 0.002

    def __post_init__(self) -> None:
        positive = {
            "close_duration": self.close_duration,
            "disturbance_duration": self.disturbance_duration,
            "frequency_hz": self.frequency_hz,
            "slip_threshold_m": self.slip_threshold_m,
        }
        nonnegative = {
            "support_settle_duration": self.support_settle_duration,
            "release_settle_duration": self.release_settle_duration,
            "recovery_duration": self.recovery_duration,
            "force_n": self.force_n,
        }
        for name, value in positive.items():
            if value <= 0:
                raise ValueError(f"{name} 必须为正数。")
        for name, value in nonnegative.items():
            if value < 0:
                raise ValueError(f"{name} 不得为负数。")

    @property
    def release_time(self) -> float:
        return self.close_duration + self.support_settle_duration

    @property
    def disturbance_start(self) -> float:
        return self.release_time + self.release_settle_duration

    @property
    def disturbance_end(self) -> float:
        return self.disturbance_start + self.disturbance_duration

    @property
    def total_duration(self) -> float:
        return self.disturbance_end + self.recovery_duration

    def phase_at(self, time_s: float) -> str:
        if time_s < self.close_duration:
            return "close"
        if time_s < self.release_time:
            return "support_settle"
        if time_s < self.disturbance_start:
            return "release_settle"
        if time_s < self.disturbance_end:
            return "disturbance"
        return "recovery"

    def force_y_at(self, time_s: float) -> float:
        if not self.disturbance_start <= time_s < self.disturbance_end:
            return 0.0
        elapsed = time_s - self.disturbance_start
        return self.force_n * math.sin(2.0 * math.pi * self.frequency_hz * elapsed)


@dataclass(slots=True)
class ForceTrace:
    """一次确定性抓取的逐步三维力。"""

    time_s: list[float]
    control: list[float]
    left: list[Vector3]
    right: list[Vector3]
    phase: list[str] = field(default_factory=list)
    applied_force_world: list[Vector3] = field(default_factory=list)
    left_world: list[Vector3] = field(default_factory=list)
    right_world: list[Vector3] = field(default_factory=list)
    cube_position_world: list[Vector3] = field(default_factory=list)
    cube_velocity_world: list[Vector3] = field(default_factory=list)


@dataclass(frozen=True, slots=True)
class SideSummary:
    """一侧指尖的稳态与峰值比较。"""

    box_steady_n: float
    grid_steady_n: float
    relative_error: float
    box_peak_n: float
    grid_peak_n: float


@dataclass(frozen=True, slots=True)
class DisturbanceSummary:
    """两个触觉模型对同一切向扰动的响应和滑移指标。"""

    response_nrmse: float
    box_max_displacement_m: float
    grid_max_displacement_m: float
    box_max_speed_m_s: float
    grid_max_speed_m_s: float
    box_slipped: bool
    grid_slipped: bool


def _close_control(step: int, steps: int, close_control: float) -> float:
    return close_control * min(1.0, step / max(1, steps // 3))


def _rotate_local_to_world(site_xmat, force: Vector3) -> Vector3:
    """使用 MuJoCo site 的行主序旋转矩阵将局部向量转到世界系。"""
    return tuple(
        float(sum(site_xmat[3 * row + column] * force[column] for column in range(3)))
        for row in range(3)
    )


def _object_id(mujoco, model, object_type, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"模型中缺少 {name!r}。")
    return object_id


def _gripper_object_name(mujoco, model, object_type, name: str) -> str:
    prefixed = prefixed_gripper_name(name)
    if mujoco.mj_name2id(model, object_type, prefixed) >= 0:
        return prefixed
    return name


def _disturbance_handles(mujoco, model, left_site_name: str, right_site_name: str):
    return (
        _object_id(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, SUPPORT_GEOM_NAME),
        _object_id(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, CUBE_BODY_NAME),
        _object_id(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, CUBE_JOINT_NAME),
        _object_id(mujoco, model, mujoco.mjtObj.mjOBJ_SITE, left_site_name),
        _object_id(mujoco, model, mujoco.mjtObj.mjOBJ_SITE, right_site_name),
    )


def _prepare_disturbance_step(
    model,
    data,
    protocol: DisturbanceProtocol,
    close_control: float,
    support_geom_id: int,
    cube_body_id: int,
) -> tuple[str, Vector3]:
    """设置本步控制、支撑碰撞状态以及施加在方块质心的世界系外力。"""
    time_s = float(data.time)
    data.ctrl[0] = close_control * min(1.0, time_s / protocol.close_duration)
    phase = protocol.phase_at(time_s)
    if time_s >= protocol.release_time:
        model.geom_contype[support_geom_id] = 0
        model.geom_conaffinity[support_geom_id] = 0
    data.xfrc_applied[cube_body_id] = 0.0
    applied = (0.0, protocol.force_y_at(time_s), 0.0)
    data.xfrc_applied[cube_body_id, :3] = applied
    return phase, applied


def _append_disturbance_sample(
    model,
    data,
    trace: ForceTrace,
    phase: str,
    applied: Vector3,
    left_force: Vector3,
    right_force: Vector3,
    cube_body_id: int,
    cube_joint_id: int,
    left_site_id: int,
    right_site_id: int,
) -> None:
    trace.phase.append(phase)
    trace.applied_force_world.append(applied)
    trace.left_world.append(
        _rotate_local_to_world(data.site_xmat[left_site_id], left_force)
    )
    trace.right_world.append(
        _rotate_local_to_world(data.site_xmat[right_site_id], right_force)
    )
    trace.cube_position_world.append(tuple(float(value) for value in data.xpos[cube_body_id]))
    dof_address = int(model.jnt_dofadr[cube_joint_id])
    trace.cube_velocity_world.append(
        tuple(float(value) for value in data.qvel[dof_address : dof_address + 3])
    )


def _run_box_taxels(
    mujoco,
    xml: Path,
    steps: int,
    close_control: float,
    protocol: DisturbanceProtocol | None,
) -> ForceTrace:
    model = load_grasp_model(None, xml)
    data = mujoco.MjData(model)

    def sensor_name(name: str) -> str:
        return gripper_name_in_model(mujoco, model, name)

    trace = ForceTrace([], [], [], [])
    handles = None
    if protocol is not None:
        handles = _disturbance_handles(
            mujoco,
            model,
            _gripper_object_name(
                mujoco, model, mujoco.mjtObj.mjOBJ_SITE, "left_taxel_site_00"
            ),
            _gripper_object_name(
                mujoco, model, mujoco.mjtObj.mjOBJ_SITE, "right_taxel_site_00"
            ),
        )
    for step in range(steps):
        if protocol is None:
            data.ctrl[0] = _close_control(step, steps, close_control)
        else:
            support_id, cube_id, _, _, _ = handles
            phase, applied = _prepare_disturbance_step(
                model, data, protocol, close_control, support_id, cube_id
            )
        mujoco.mj_step(model, data)
        left = _taxel_surface_force_vector(data, "left", sensor_name)
        right = _taxel_surface_force_vector(data, "right", sensor_name)
        trace.time_s.append(float(data.time))
        trace.control.append(float(data.ctrl[0]))
        trace.left.append(left)
        trace.right.append(right)
        if protocol is not None:
            support_id, cube_id, joint_id, left_site_id, right_site_id = handles
            _append_disturbance_sample(
                model,
                data,
                trace,
                phase,
                applied,
                left,
                right,
                cube_id,
                joint_id,
                left_site_id,
                right_site_id,
            )
    return trace


def _run_touch_grid(
    mujoco,
    xml: Path,
    steps: int,
    close_control: float,
    protocol: DisturbanceProtocol | None,
) -> ForceTrace:
    model = load_grasp_model(None, xml)
    data = mujoco.MjData(model)
    left_sensor = gripper_name_in_model(mujoco, model, "touch_left")
    right_sensor = gripper_name_in_model(mujoco, model, "touch_right")
    left_shape = _touch_grid_shape(mujoco, model, left_sensor)
    right_shape = _touch_grid_shape(mujoco, model, right_sensor)

    trace = ForceTrace([], [], [], [])
    handles = None
    if protocol is not None:
        handles = _disturbance_handles(
            mujoco,
            model,
            gripper_name_in_model(mujoco, model, "touch_left"),
            gripper_name_in_model(mujoco, model, "touch_right"),
        )
    for step in range(steps):
        if protocol is None:
            data.ctrl[0] = _close_control(step, steps, close_control)
        else:
            support_id, cube_id, _, _, _ = handles
            phase, applied = _prepare_disturbance_step(
                model, data, protocol, close_control, support_id, cube_id
            )
        mujoco.mj_step(model, data)
        left = _read_tactile(data, left_sensor, left_shape).sum(axis=(1, 2))
        right = _read_tactile(data, right_sensor, right_shape).sum(axis=(1, 2))
        left_force = tuple(float(value) for value in left)
        right_force = tuple(float(value) for value in right)
        trace.time_s.append(float(data.time))
        trace.control.append(float(data.ctrl[0]))
        trace.left.append(left_force)
        trace.right.append(right_force)
        if protocol is not None:
            support_id, cube_id, joint_id, left_site_id, right_site_id = handles
            _append_disturbance_sample(
                model,
                data,
                trace,
                phase,
                applied,
                left_force,
                right_force,
                cube_id,
                joint_id,
                left_site_id,
                right_site_id,
            )
    return trace


def run_comparison(
    box_xml: Path = DEFAULT_BOX_TAXEL_XML,
    grid_xml: Path = DEFAULT_TOUCH_GRID_XML,
    steps: int = 1500,
    close_control: float = 220.0,
    protocol: DisturbanceProtocol | None = None,
) -> tuple[ForceTrace, ForceTrace]:
    """以相同初态和控制轨迹分别运行两个触觉模型。"""
    if steps <= 0:
        raise ValueError("steps 必须为正整数。")
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error
    if protocol is not None:
        box_model = load_grasp_model(None, box_xml)
        steps = math.ceil(protocol.total_duration / float(box_model.opt.timestep))
    return (
        _run_box_taxels(mujoco, box_xml, steps, close_control, protocol),
        _run_touch_grid(mujoco, grid_xml, steps, close_control, protocol),
    )


def _pressure(force: tuple[float, float, float]) -> float:
    return max(0.0, force[2])


def _relative_error(first: float, second: float) -> float:
    """返回对称相对误差，避免任一模型被隐式指定为真值。"""
    return abs(first - second) / max(0.5 * (abs(first) + abs(second)), 1e-12)


def summarize_side(
    box_forces: list[Vector3],
    grid_forces: list[Vector3],
) -> SideSummary:
    """比较末段 20% 的稳态法向力及全过程峰值。"""
    if not box_forces or len(box_forces) != len(grid_forces):
        raise ValueError("两个模型必须包含相同数量的非空样本。")
    steady_count = max(1, len(box_forces) // 5)
    box_pressure = [_pressure(force) for force in box_forces]
    grid_pressure = [_pressure(force) for force in grid_forces]
    box_steady = fmean(box_pressure[-steady_count:])
    grid_steady = fmean(grid_pressure[-steady_count:])
    return SideSummary(
        box_steady_n=box_steady,
        grid_steady_n=grid_steady,
        relative_error=_relative_error(box_steady, grid_steady),
        box_peak_n=max(box_pressure),
        grid_peak_n=max(grid_pressure),
    )


def _total_world_component(trace: ForceTrace, index: int, axis: int) -> float:
    return trace.left_world[index][axis] + trace.right_world[index][axis]


def _tangential_displacement(position: Vector3, reference: Vector3) -> float:
    return math.hypot(position[1] - reference[1], position[2] - reference[2])


def summarize_disturbance(
    box: ForceTrace,
    grid: ForceTrace,
    slip_threshold_m: float,
) -> DisturbanceSummary:
    """比较扰动阶段的世界 Y 合力，并计算方块在世界 YZ 平面的滑移。"""
    if not box.phase or len(box.phase) != len(grid.phase):
        raise ValueError("需要两个模型长度一致的扰动轨迹。")
    disturbance_indices = [
        index for index, phase in enumerate(box.phase) if phase == "disturbance"
    ]
    if not disturbance_indices:
        raise ValueError("轨迹中没有 disturbance 阶段。")
    response_box = [_total_world_component(box, index, 1) for index in disturbance_indices]
    response_grid = [
        _total_world_component(grid, index, 1) for index in disturbance_indices
    ]
    rms_error = math.sqrt(
        fmean((box_value - grid_value) ** 2 for box_value, grid_value in zip(response_box, response_grid))
    )
    response_scale = max(
        max(abs(value) for value in response_box),
        max(abs(value) for value in response_grid),
        1e-12,
    )

    evaluation_indices = [
        index
        for index, phase in enumerate(box.phase)
        if phase in {"disturbance", "recovery"}
    ]
    first = disturbance_indices[0]
    box_reference = box.cube_position_world[first]
    grid_reference = grid.cube_position_world[first]
    box_displacements = [
        _tangential_displacement(box.cube_position_world[index], box_reference)
        for index in evaluation_indices
    ]
    grid_displacements = [
        _tangential_displacement(grid.cube_position_world[index], grid_reference)
        for index in evaluation_indices
    ]
    box_speeds = [
        math.hypot(
            box.cube_velocity_world[index][1], box.cube_velocity_world[index][2]
        )
        for index in evaluation_indices
    ]
    grid_speeds = [
        math.hypot(
            grid.cube_velocity_world[index][1], grid.cube_velocity_world[index][2]
        )
        for index in evaluation_indices
    ]
    box_max_displacement = max(box_displacements)
    grid_max_displacement = max(grid_displacements)
    return DisturbanceSummary(
        response_nrmse=rms_error / response_scale,
        box_max_displacement_m=box_max_displacement,
        grid_max_displacement_m=grid_max_displacement,
        box_max_speed_m_s=max(box_speeds),
        grid_max_speed_m_s=max(grid_speeds),
        box_slipped=box_max_displacement > slip_threshold_m,
        grid_slipped=grid_max_displacement > slip_threshold_m,
    )


def write_comparison_csv(
    path: Path, box: ForceTrace, grid: ForceTrace, record_every: int = 1
) -> None:
    """写出采用公共“压缩 Fz 为正”约定的联合逐步数据。"""
    if record_every <= 0:
        raise ValueError("record_every 必须为正整数。")
    if len(box.time_s) != len(grid.time_s):
        raise ValueError("两个模型的轨迹长度不一致。")
    fieldnames = ["step", "time_s", "control"]
    disturbance = bool(box.phase)
    if disturbance != bool(grid.phase):
        raise ValueError("两个模型必须同时包含或同时不包含扰动轨迹。")
    if disturbance:
        fieldnames.extend(("phase", "applied_world_fx", "applied_world_fy", "applied_world_fz"))
    for model_name in ("box_taxel", "touch_grid"):
        for side in ("left", "right"):
            fieldnames.extend(
                (
                    f"{model_name}_{side}_fx",
                    f"{model_name}_{side}_fy",
                    f"{model_name}_{side}_fz",
                    f"{model_name}_{side}_pressure_n",
                )
            )
        if disturbance:
            for side in ("left", "right"):
                fieldnames.extend(
                    f"{model_name}_{side}_world_f{axis}" for axis in ("x", "y", "z")
                )
            fieldnames.extend(
                f"{model_name}_total_world_f{axis}" for axis in ("x", "y", "z")
            )
            fieldnames.extend(
                f"{model_name}_cube_{axis}"
                for axis in ("x", "y", "z", "vx", "vy", "vz")
            )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        last_step = len(box.time_s) - 1
        for step in range(len(box.time_s)):
            if step % record_every and step != last_step:
                continue
            row: dict[str, float | int | str] = {
                "step": step,
                "time_s": box.time_s[step],
                "control": box.control[step],
            }
            if disturbance:
                row["phase"] = box.phase[step]
                for axis, value in zip(("x", "y", "z"), box.applied_force_world[step]):
                    row[f"applied_world_f{axis}"] = value
            for model_name, trace in (("box_taxel", box), ("touch_grid", grid)):
                for side in ("left", "right"):
                    force = getattr(trace, side)[step]
                    for axis, value in zip(("x", "y", "z"), force):
                        row[f"{model_name}_{side}_f{axis}"] = value
                    row[f"{model_name}_{side}_pressure_n"] = _pressure(force)
                if disturbance:
                    for side in ("left", "right"):
                        force_world = getattr(trace, f"{side}_world")[step]
                        for axis, value in zip(("x", "y", "z"), force_world):
                            row[f"{model_name}_{side}_world_f{axis}"] = value
                    for axis_index, axis in enumerate(("x", "y", "z")):
                        row[f"{model_name}_total_world_f{axis}"] = _total_world_component(
                            trace, step, axis_index
                        )
                    position = trace.cube_position_world[step]
                    velocity = trace.cube_velocity_world[step]
                    for axis, value in zip(("x", "y", "z"), position):
                        row[f"{model_name}_cube_{axis}"] = value
                    for axis, value in zip(("vx", "vy", "vz"), velocity):
                        row[f"{model_name}_cube_{axis}"] = value
            writer.writerow(row)


def plot_comparison(path: Path, box: ForceTrace, grid: ForceTrace) -> None:
    """使用 SciencePlots 绘制常规法向比较或分阶段扰动比较。"""
    try:
        import matplotlib.pyplot as plt
        import scienceplots  # noqa: F401 -- 导入后注册 science 样式。
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    plt.style.use(["science", "ieee", "no-latex"])
    if box.phase:
        _plot_disturbance_comparison(plt, path, box, grid)
        return
    figure, (control_axis, left_axis, right_axis) = plt.subplots(
        3, 1, figsize=(7.16, 6.5), sharex=True, layout="constrained"
    )
    control_axis.plot(box.time_s, box.control, color="black", label="control")
    control_axis.set_ylabel("Control")
    control_axis.legend(frameon=False)
    for axis, side, title in (
        (left_axis, "left", "Left normal pressure"),
        (right_axis, "right", "Right normal pressure"),
    ):
        axis.plot(
            box.time_s,
            [_pressure(force) for force in getattr(box, side)],
            label="box taxel",
        )
        axis.plot(
            grid.time_s,
            [_pressure(force) for force in getattr(grid, side)],
            linestyle="--",
            label="touch grid",
        )
        axis.set_ylabel("Force (N)")
        axis.set_title(title)
        axis.legend(frameon=False)
    right_axis.set_xlabel("Simulation time (s)")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=600)
    plt.close(figure)


def _plot_disturbance_comparison(plt, path: Path, box: ForceTrace, grid: ForceTrace) -> None:
    figure, axes = plt.subplots(
        5, 1, figsize=(7.16, 9.0), sharex=True, layout="constrained"
    )
    control_axis, applied_axis, local_axis, world_axis, displacement_axis = axes
    control_axis.plot(box.time_s, box.control, color="black", label="control")
    control_axis.set_ylabel("Control")
    applied_axis.plot(
        box.time_s,
        [force[1] for force in box.applied_force_world],
        color="tab:orange",
        label="applied world Fy",
    )
    applied_axis.set_ylabel("Force (N)")
    for trace, model_name, linestyle in (
        (box, "box", "-"),
        (grid, "grid", "--"),
    ):
        for side in ("left", "right"):
            local_axis.plot(
                trace.time_s,
                [math.hypot(force[0], force[1]) for force in getattr(trace, side)],
                linestyle=linestyle,
                label=f"{model_name} {side}",
            )
        world_axis.plot(
            trace.time_s,
            [
                _total_world_component(trace, index, 1)
                for index in range(len(trace.time_s))
            ],
            linestyle=linestyle,
            label=f"{model_name} total Fy",
        )
        disturbance_start = trace.phase.index("disturbance")
        reference = trace.cube_position_world[disturbance_start]
        displacement_axis.plot(
            trace.time_s,
            [
                1000.0 * _tangential_displacement(position, reference)
                for position in trace.cube_position_world
            ],
            linestyle=linestyle,
            label=f"{model_name}",
        )
    local_axis.set_ylabel("Local shear (N)")
    world_axis.set_ylabel("World Fy (N)")
    displacement_axis.set_ylabel("YZ disp. (mm)")
    displacement_axis.set_xlabel("Simulation time (s)")
    for axis in axes:
        axis.legend(frameon=False, ncol=2)
    transitions = [
        index
        for index in range(1, len(box.phase))
        if box.phase[index] != box.phase[index - 1]
    ]
    for axis in axes:
        for index in transitions:
            axis.axvline(box.time_s[index], color="0.65", linewidth=0.7, linestyle=":")
    boundaries = [0, *transitions, len(box.phase)]
    for start, end in zip(boundaries, boundaries[1:]):
        left_time = box.time_s[start]
        right_time = box.time_s[end - 1]
        control_axis.text(
            0.5 * (left_time + right_time),
            0.93,
            box.phase[start].replace("_", "\n"),
            ha="center",
            va="top",
            fontsize=7,
            color="0.35",
            transform=control_axis.get_xaxis_transform(),
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=600)
    plt.close(figure)


def _print_summary(left: SideSummary, right: SideSummary, tolerance: float) -> bool:
    print("side   box steady   grid steady   rel. error   box peak   grid peak")
    for side, summary in (("left", left), ("right", right)):
        print(
            f"{side:5s} {summary.box_steady_n:10.3f} N {summary.grid_steady_n:10.3f} N "
            f"{summary.relative_error:9.2%} {summary.box_peak_n:10.3f} N "
            f"{summary.grid_peak_n:10.3f} N"
        )
    passed = left.relative_error <= tolerance and right.relative_error <= tolerance
    print(f"result: {'PASS' if passed else 'FAIL'} (tolerance={tolerance:.1%})")
    return passed


def _print_disturbance_summary(
    summary: DisturbanceSummary, tolerance: float, slip_threshold_m: float
) -> bool:
    print("\ndisturbance response (world Y) and tangential slip (world YZ)")
    print(f"response NRMSE: {summary.response_nrmse:.2%} (tolerance={tolerance:.1%})")
    print("model       max disp.   max speed   slip")
    for name, displacement, speed, slipped in (
        (
            "box taxel",
            summary.box_max_displacement_m,
            summary.box_max_speed_m_s,
            summary.box_slipped,
        ),
        (
            "touch grid",
            summary.grid_max_displacement_m,
            summary.grid_max_speed_m_s,
            summary.grid_slipped,
        ),
    ):
        print(
            f"{name:10s} {1000.0 * displacement:8.3f} mm "
            f"{speed:9.4f} m/s   {'YES' if slipped else 'no'}"
        )
    passed = (
        summary.response_nrmse <= tolerance
        and not summary.box_slipped
        and not summary.grid_slipped
    )
    print(
        f"disturbance result: {'PASS' if passed else 'FAIL'} "
        f"(slip threshold={1000.0 * slip_threshold_m:.1f} mm)"
    )
    return passed


def main() -> int:
    """运行比较，保存联合 CSV 和叠加曲线，并按容差返回状态。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--box-xml", type=Path, default=DEFAULT_BOX_TAXEL_XML)
    parser.add_argument("--grid-xml", type=Path, default=DEFAULT_TOUCH_GRID_XML)
    parser.add_argument("--steps", type=int, default=1500, help="无扰动模式的仿真步数")
    parser.add_argument("--close-control", type=float, default=220.0)
    parser.add_argument("--record-every", type=int, default=1)
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV)
    parser.add_argument("--output-plot", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.10)
    parser.add_argument(
        "--disturbance",
        action="store_true",
        help="抓稳并撤去支撑后，在方块质心施加世界 Y 方向正弦力",
    )
    parser.add_argument("--disturbance-force", type=float, default=5.0, metavar="N")
    parser.add_argument("--disturbance-frequency", type=float, default=2.0, metavar="HZ")
    parser.add_argument("--disturbance-duration", type=float, default=1.0, metavar="S")
    parser.add_argument("--support-settle-duration", type=float, default=0.5, metavar="S")
    parser.add_argument("--release-settle-duration", type=float, default=0.5, metavar="S")
    parser.add_argument("--recovery-duration", type=float, default=0.5, metavar="S")
    parser.add_argument("--slip-threshold", type=float, default=0.002, metavar="M")
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps 必须为正整数。")
    if args.record_every <= 0:
        parser.error("--record-every 必须为正整数。")
    if args.tolerance < 0:
        parser.error("--tolerance 不得为负数。")

    protocol = None
    if args.disturbance:
        try:
            protocol = DisturbanceProtocol(
                support_settle_duration=args.support_settle_duration,
                release_settle_duration=args.release_settle_duration,
                disturbance_duration=args.disturbance_duration,
                recovery_duration=args.recovery_duration,
                force_n=args.disturbance_force,
                frequency_hz=args.disturbance_frequency,
                slip_threshold_m=args.slip_threshold,
            )
        except ValueError as error:
            parser.error(str(error))
    box, grid = run_comparison(
        args.box_xml, args.grid_xml, args.steps, args.close_control, protocol
    )
    output_plot = args.output_plot or args.output_csv.with_suffix(".png")
    write_comparison_csv(args.output_csv, box, grid, args.record_every)
    plot_comparison(output_plot, box, grid)
    print(f"CSV:  {args.output_csv}")
    print(f"Plot: {output_plot}")
    left = summarize_side(box.left, grid.left)
    right = summarize_side(box.right, grid.right)
    passed = _print_summary(left, right, args.tolerance)
    if protocol is not None:
        disturbance = summarize_disturbance(box, grid, protocol.slip_threshold_m)
        passed = (
            _print_disturbance_summary(
                disturbance, args.tolerance, protocol.slip_threshold_m
            )
            and passed
        )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())

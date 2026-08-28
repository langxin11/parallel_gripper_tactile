"""时变目标法向力跟踪实验。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Annotated, Literal

import mujoco
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator
import yaml

from ..control import NormalForceController
from ..profiles import load_profile
from ..scenes.custom import (
    CUBE_PREFIX,
    DEFAULT_CUBE_HALF_CONTACT_SIDE,
    DEFAULT_CUBE_HALF_THICKNESS,
    DEFAULT_CUBE_MASS,
    DEFAULT_PROFILE,
    GRIPPER_PREFIX,
    SUPPORT_GEOM_NAME,
    build_custom_grasp_model,
)
from ..timing import SimulationTimer
from .grasp import (
    CUBE_BODY_NAME,
    CUBE_JOINT_NAME,
    _friction_capacity,
    _prefixed_reader,
    _tactile_measurement,
    _taxel_geom_sides,
)


class ForceTrackingConfigError(ValueError):
    """目标力跟踪任务配置错误。"""


class _TaskModel(BaseModel):
    """拒绝未知字段的任务配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class ForceWaypoint(_TaskModel):
    """一个目标力 waypoint。"""

    t_s: Annotated[float, Field(ge=0)]
    force_n: Annotated[float, Field(ge=0)]


class ForceReference(_TaskModel):
    """由 waypoint 定义的目标法向力曲线。"""

    interpolation: Literal["hold", "linear", "smoothstep"] = "smoothstep"
    waypoints: tuple[ForceWaypoint, ...]

    @model_validator(mode="after")
    def validate_waypoints(self) -> "ForceReference":
        """要求至少两个 waypoint，且时间严格递增。"""
        if len(self.waypoints) < 2:
            raise ValueError("force reference requires at least two waypoints")
        previous = -math.inf
        for waypoint in self.waypoints:
            if waypoint.t_s <= previous:
                raise ValueError("force reference waypoint times must be strictly increasing")
            previous = waypoint.t_s
        return self

    @property
    def duration_s(self) -> float:
        """返回参考曲线持续时间。"""
        return float(self.waypoints[-1].t_s)

    def target_at(self, tracking_time_s: float) -> float:
        """返回指定跟踪时间的目标力。"""
        time_s = max(0.0, float(tracking_time_s))
        if time_s <= self.waypoints[0].t_s:
            return float(self.waypoints[0].force_n)
        for start, end in zip(self.waypoints, self.waypoints[1:]):
            if time_s <= end.t_s:
                if self.interpolation == "hold":
                    return float(start.force_n)
                span = end.t_s - start.t_s
                u = (time_s - start.t_s) / span
                if self.interpolation == "smoothstep":
                    u = u * u * (3.0 - 2.0 * u)
                return float(start.force_n + u * (end.force_n - start.force_n))
        return float(self.waypoints[-1].force_n)


class ForceTrackingApproach(_TaskModel):
    """接近接触阶段配置。"""

    duration_s: Annotated[float, Field(gt=0)] = 1.0
    timeout_s: Annotated[float, Field(gt=0)] = 3.0
    settle_after_contact_s: Annotated[float, Field(ge=0)] = 0.2
    feedforward_force_n: Annotated[float, Field(ge=0)] = 2.0


class ForceTrackingMetricsConfig(_TaskModel):
    """动态力跟踪指标配置。"""

    ignore_initial_s: Annotated[float, Field(ge=0)] = 0.2


class ForceTrackingTask(_TaskModel):
    """完整的动态目标力跟踪任务配置。"""

    schema_version: Literal[1]
    name: Annotated[str, Field(min_length=1)]
    approach: ForceTrackingApproach = ForceTrackingApproach()
    reference: ForceReference
    metrics: ForceTrackingMetricsConfig = ForceTrackingMetricsConfig()
    control_period_s: Annotated[float, Field(gt=0)] = 0.002
    release_support_on_tracking: bool = False

    @classmethod
    def load(cls, path: str | Path) -> "ForceTrackingTask":
        """从 YAML 文件加载任务配置。"""
        task_path = Path(path)
        try:
            with task_path.open(encoding="utf-8") as stream:
                raw = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ForceTrackingConfigError(f"invalid YAML in {task_path}") from error
        if raw is None:
            raise ForceTrackingConfigError(f"force tracking task is empty: {task_path}")
        if not isinstance(raw, dict):
            raise ForceTrackingConfigError(f"force tracking task root must be a mapping: {task_path}")
        try:
            return cls.model_validate(raw)
        except ValidationError as error:
            raise ForceTrackingConfigError(str(error)) from error


@dataclass(frozen=True, slots=True)
class ForceTrackingResult:
    """动态目标力跟踪实验指标。"""

    contact_time_s: float
    tracking_start_time_s: float
    tracking_duration_s: float
    rmse_n: float
    mae_n: float
    peak_abs_error_n: float
    mean_error_n: float
    final_error_n: float
    torque_saturation_ratio: float
    position_saturation_ratio: float
    mean_estimated_stiffness_n_per_m: float
    simulation_stable: bool

    @property
    def passed(self) -> bool:
        """当前只表示仿真成功完成并产生了可评估跟踪段。"""
        return self.simulation_stable and math.isfinite(self.rmse_n)


def _plot_force_tracking(path: Path, rows: list[dict[str, float | str]]) -> None:
    """绘制目标力跟踪结果。"""
    if not rows:
        raise ValueError("cannot plot an empty force tracking trace")
    try:
        import matplotlib.pyplot as plt
        import scienceplots  # noqa: F401 -- 导入后注册 SciencePlots 样式。
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    plt.style.use(["science", "ieee", "no-latex"])
    times = [float(row["time_s"]) for row in rows]
    target = [float(row["target_normal_force_n"]) for row in rows]
    filtered = [float(row["filtered_normal_force_n"]) for row in rows]
    measured = [float(row["measured_normal_force_n"]) for row in rows]
    torque = [float(row["motor_torque_n_m"]) for row in rows]
    stiffness = [float(row["estimated_contact_stiffness_n_per_m"]) for row in rows]
    stiffness = [math.nan if value <= 0 else value for value in stiffness]
    colors = {"black": "#000000", "blue": "#0072B2", "orange": "#D55E00", "green": "#009E73"}
    figure, axes = plt.subplots(3, 1, figsize=(7.16, 6.4), sharex=True, layout="constrained")
    axes[0].plot(times, target, color=colors["black"], label="target", linewidth=1.2)
    axes[0].plot(times, filtered, color=colors["blue"], label="filtered", linewidth=1.2)
    axes[0].plot(times, measured, color=colors["orange"], label="measured", linewidth=0.8, alpha=0.7)
    axes[0].set_ylabel("Normal force (N)")
    axes[0].legend(loc="best")
    axes[1].plot(times, torque, color=colors["green"], linewidth=1.2)
    axes[1].set_ylabel("Motor torque (N m)")
    axes[2].plot(times, stiffness, color=colors["blue"], linewidth=1.2)
    axes[2].set_ylabel("K estimate (N/m)")
    axes[2].set_xlabel("Time (s)")
    for axis in axes:
        axis.grid(True, linewidth=0.3, alpha=0.5)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=200)
    plt.close(figure)


def _evaluate_tracking(
    rows: list[dict[str, float | str]],
    *,
    ignore_initial_s: float,
    mit_t_max: float,
    p_min: float,
    p_max: float,
) -> tuple[float, float, float, float, float, float, float, float]:
    """计算目标力跟踪误差和饱和比例。"""
    tracking_rows = [
        row
        for row in rows
        if row["phase"] == "track_reference" and float(row["tracking_time_s"]) >= ignore_initial_s
    ]
    if not tracking_rows:
        return (math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf)
    errors = np.asarray([float(row["tracking_error_n"]) for row in tracking_rows])
    torques = np.asarray([float(row["motor_torque_n_m"]) for row in tracking_rows])
    positions = np.asarray([float(row["control"]) for row in tracking_rows])
    stiffness = np.asarray(
        [
            float(row["estimated_contact_stiffness_n_per_m"])
            for row in tracking_rows
            if math.isfinite(float(row["estimated_contact_stiffness_n_per_m"]))
        ],
        dtype=np.float64,
    )
    torque_saturation = float(np.mean(np.abs(torques) >= 0.999 * mit_t_max))
    position_saturation = float(
        np.mean((positions <= p_min + 1e-6) | (positions >= p_max - 1e-6))
    )
    return (
        float(np.sqrt(np.mean(errors**2))),
        float(np.mean(np.abs(errors))),
        float(np.max(np.abs(errors))),
        float(np.mean(errors)),
        float(errors[-1]),
        torque_saturation,
        position_saturation,
        float(np.mean(stiffness)) if stiffness.size else math.nan,
    )


def run_force_tracking(
    profile_path: Path = DEFAULT_PROFILE,
    *,
    task: ForceTrackingTask,
    cube_half_thickness: float = DEFAULT_CUBE_HALF_THICKNESS,
    cube_half_contact_side: float = DEFAULT_CUBE_HALF_CONTACT_SIDE,
    cube_mass: float = DEFAULT_CUBE_MASS,
    output_csv: Path | None = None,
    output_plot: Path | None = None,
) -> ForceTrackingResult:
    """运行两阶段目标法向力跟踪测试。"""
    profile = load_profile(profile_path)
    if profile.normal_force is None or profile.mit is None:
        raise ValueError("force tracking requires MIT torque control with control.force")
    model = build_custom_grasp_model(
        profile,
        cube_half_thickness=cube_half_thickness,
        cube_half_contact_side=cube_half_contact_side,
        cube_mass=cube_mass,
    )
    if task.control_period_s + 1e-12 < float(model.opt.timestep):
        raise ValueError("control_period_s must not be smaller than the physics timestep")

    data = mujoco.MjData(model)
    control_timer = SimulationTimer(task.control_period_s, float(data.time))
    reader = _prefixed_reader(model, profile)
    controller = NormalForceController.from_profile(model, profile, name_prefix=GRIPPER_PREFIX)
    force_config = profile.normal_force
    noise_rng = np.random.default_rng(int(force_config.sensor_noise_seed))
    support_id = model.geom(SUPPORT_GEOM_NAME).id
    cube_body_id = model.body(CUBE_BODY_NAME).id
    cube_joint_id = model.joint(CUBE_JOINT_NAME).id
    cube_geom_id = model.geom(f"{CUBE_PREFIX}target_cube_geom").id
    cube_dof = model.jnt_dofadr[cube_joint_id]
    taxel_geom_sides = _taxel_geom_sides(model, profile)
    contact_time_s: float | None = None
    tracking_start_time_s: float | None = None
    support_released = False
    simulation_stable = True
    force_command = None
    rows: list[dict[str, float | str]] = []
    max_duration = (
        task.approach.timeout_s
        + task.approach.settle_after_contact_s
        + task.reference.duration_s
        + task.control_period_s
    )
    steps = math.ceil(max_duration / model.opt.timestep)
    for _ in range(steps):
        time_s = float(data.time)
        if tracking_start_time_s is None:
            tracking_time_s = 0.0
            target_force_n = task.reference.target_at(0.0)
            phase = "approach_contact" if contact_time_s is None else "contact_settle"
        else:
            tracking_time_s = time_s - tracking_start_time_s
            target_force_n = task.reference.target_at(tracking_time_s)
            phase = "track_reference"
            if tracking_time_s > task.reference.duration_s:
                break
            if task.release_support_on_tracking and not support_released:
                model.geom_contype[support_id] = 0
                model.geom_conaffinity[support_id] = 0
                support_released = True

        data.xfrc_applied[cube_body_id] = 0.0
        target_position = profile.open_control + min(1.0, time_s / task.approach.duration_s) * (
            profile.closed_control - profile.open_control
        )
        control_dt = control_timer.pop_due(time_s)
        if control_dt is not None:
            feedback_tactile = reader.read(data)
            feedback_measurement = _tactile_measurement(
                feedback_tactile,
                left_normal_std_n=float(force_config.sensor_taxel_normal_noise_std_n[0]),
                right_normal_std_n=float(force_config.sensor_taxel_normal_noise_std_n[1]),
                left_shear_std_n=float(force_config.sensor_taxel_shear_noise_std_n[0]),
                right_shear_std_n=float(force_config.sensor_taxel_shear_noise_std_n[1]),
                rng=noise_rng,
            )
            feedback_capacity = feedback_measurement.normal_capacity
            force_command = controller.apply(
                data,
                approach_position=target_position,
                total_normal_force_n=feedback_capacity.normal_force_n,
                left_normal_force_n=feedback_capacity.left_normal_force_n,
                right_normal_force_n=feedback_capacity.right_normal_force_n,
                dt=control_dt,
                target_force_n=target_force_n,
                approach_feedforward_force_n=task.approach.feedforward_force_n,
            )
            if contact_time_s is None and force_command.state == "force_tracking":
                contact_time_s = time_s
            if (
                contact_time_s is not None
                and tracking_start_time_s is None
                and time_s - contact_time_s >= task.approach.settle_after_contact_s
            ):
                tracking_start_time_s = time_s

        if force_command is None:
            raise RuntimeError("control timer did not produce an initial command")
        motor_command = force_command.mit
        mujoco.mj_step(model, data)
        if (
            data.time <= time_s
            or not np.isfinite(data.qpos).all()
            or not np.isfinite(data.qvel).all()
        ):
            simulation_stable = False
            break

        tactile = reader.read(data)
        tactile_measurement = _tactile_measurement(
            tactile,
            left_normal_std_n=float(force_config.sensor_taxel_normal_noise_std_n[0]),
            right_normal_std_n=float(force_config.sensor_taxel_normal_noise_std_n[1]),
            left_shear_std_n=float(force_config.sensor_taxel_shear_noise_std_n[0]),
            right_shear_std_n=float(force_config.sensor_taxel_shear_noise_std_n[1]),
            rng=noise_rng,
        )
        capacity = _friction_capacity(
            model,
            data,
            taxel_geom_sides=taxel_geom_sides,
            cube_geom_id=cube_geom_id,
        )
        position = data.xpos[cube_body_id].copy()
        velocity = data.qvel[cube_dof : cube_dof + 3]
        rows.append(
            {
                "time_s": float(data.time),
                "phase": phase,
                "tracking_time_s": tracking_time_s,
                "control_state": force_command.state,
                "target_normal_force_n": force_command.target_force_n,
                "measured_normal_force_n": force_command.measured_force_n,
                "filtered_normal_force_n": force_command.filtered_force_n,
                "tracking_error_n": force_command.force_error_n,
                "control": motor_command.target_position,
                "drive_position_rad": motor_command.position,
                "drive_velocity_rad_s": motor_command.velocity,
                "motor_torque_n_m": motor_command.torque,
                "force_position_adjustment_rad": force_command.position_adjustment,
                "stiffness_position_adjustment_rad": (
                    force_command.stiffness_position_adjustment
                ),
                "force_feedforward_torque_n_m": force_command.force_feedforward_torque,
                "mit_feedforward_torque_n_m": motor_command.feedforward_torque,
                "estimated_contact_stiffness_n_per_m": (
                    force_command.estimated_contact_stiffness_n_per_m
                    if force_command.estimated_contact_stiffness_n_per_m is not None
                    else math.nan
                ),
                "closure_jacobian_m_per_rad": (
                    force_command.closure_jacobian_m_per_rad
                    if force_command.closure_jacobian_m_per_rad is not None
                    else math.nan
                ),
                "aperture_m": (
                    force_command.aperture_m
                    if force_command.aperture_m is not None
                    else math.nan
                ),
                "measured_left_fx": float(tactile_measurement.left_force[0]),
                "measured_left_fy": float(tactile_measurement.left_force[1]),
                "measured_left_fz": float(tactile_measurement.left_force[2]),
                "measured_right_fx": float(tactile_measurement.right_force[0]),
                "measured_right_fy": float(tactile_measurement.right_force[1]),
                "measured_right_fz": float(tactile_measurement.right_force[2]),
                "taxel_normal_force_n": capacity.normal_force_n,
                "left_taxel_normal_force_n": capacity.left_normal_force_n,
                "right_taxel_normal_force_n": capacity.right_normal_force_n,
                "active_taxel_contacts": capacity.active_contacts,
                "cube_y": float(position[1]),
                "cube_z": float(position[2]),
                "cube_vy": float(velocity[1]),
                "cube_vz": float(velocity[2]),
            }
        )

    if contact_time_s is None or tracking_start_time_s is None:
        simulation_stable = False
    if output_csv is not None and rows:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        with output_csv.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    if output_plot is not None and rows:
        _plot_force_tracking(output_plot, rows)

    (
        rmse_n,
        mae_n,
        peak_abs_error_n,
        mean_error_n,
        final_error_n,
        torque_saturation_ratio,
        position_saturation_ratio,
        mean_stiffness,
    ) = _evaluate_tracking(
        rows,
        ignore_initial_s=task.metrics.ignore_initial_s,
        mit_t_max=float(profile.mit.t_max),
        p_min=float(profile.mit.p_min),
        p_max=float(profile.mit.p_max),
    )
    return ForceTrackingResult(
        contact_time_s=math.nan if contact_time_s is None else contact_time_s,
        tracking_start_time_s=math.nan
        if tracking_start_time_s is None
        else tracking_start_time_s,
        tracking_duration_s=task.reference.duration_s,
        rmse_n=rmse_n,
        mae_n=mae_n,
        peak_abs_error_n=peak_abs_error_n,
        mean_error_n=mean_error_n,
        final_error_n=final_error_n,
        torque_saturation_ratio=torque_saturation_ratio,
        position_saturation_ratio=position_saturation_ratio,
        mean_estimated_stiffness_n_per_m=mean_stiffness,
        simulation_stable=simulation_stable,
    )


__all__ = [
    "ForceReference",
    "ForceTrackingApproach",
    "ForceTrackingConfigError",
    "ForceTrackingMetricsConfig",
    "ForceTrackingResult",
    "ForceTrackingTask",
    "ForceWaypoint",
    "run_force_tracking",
]

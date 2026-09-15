"""Oracle 与固定摩擦先验自适应目标力调度的仿真实验。"""

from __future__ import annotations

import csv
from dataclasses import dataclass, replace
import math
from pathlib import Path
from typing import Annotated, Literal

import mujoco
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, ValidationError, model_validator
import yaml
from dm_grasp_core.grasp.adaptive import AdaptiveLoadConfig, AdaptiveLoadScheduler
from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig, UnifiedAdaptivePolicy

from ..control import ForceControlObservation, ForceControlReference, NormalForceController
from ..dm_admittance import DMAdmittanceController
from ..force_scheduling import OracleTargetForceScheduler, TargetForceSchedulerConfig
from ..visualization import (
    FULL_WIDTH_FONT_SCALE,
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)

from ..config.profiles import GripperProfile, load_profile, validate_resolved_profile
from ..scenes.custom import (
    CUBE_PREFIX,
    DEFAULT_PROFILE,
    GRIPPER_PREFIX,
    ObjectMaterial,
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


class ForceSchedulingConfigError(ValueError):
    """目标力调度任务配置无法加载或未通过校验。"""


class _TaskModel(BaseModel):
    """拒绝未知字段并冻结运行配置的任务模型。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class DownwardLoadWaypoint(_TaskModel):
    """重力方向附加载荷曲线上的一个 waypoint。"""

    t_s: Annotated[FiniteFloat, Field(ge=0)]
    force_n: Annotated[FiniteFloat, Field(ge=0)]


class DownwardLoadReference(_TaskModel):
    """施加在物体质心、沿重力方向的附加切向载荷。"""

    interpolation: Literal["hold", "linear", "smoothstep"] = "linear"
    waypoints: tuple[DownwardLoadWaypoint, ...]

    @model_validator(mode="after")
    def validate_waypoints(self) -> "DownwardLoadReference":
        """要求曲线从零时刻开始，并且 waypoint 时间严格递增。"""
        if len(self.waypoints) < 2:
            raise ValueError("downward load reference requires at least two waypoints")
        if self.waypoints[0].t_s != 0:
            raise ValueError("downward load reference must start at t_s=0")
        previous = -math.inf
        for waypoint in self.waypoints:
            if waypoint.t_s <= previous:
                raise ValueError("downward load waypoint times must be strictly increasing")
            previous = waypoint.t_s
        return self

    @property
    def duration_s(self) -> float:
        """返回附加载荷场景持续时间。"""
        return float(self.waypoints[-1].t_s)

    def sample_at(self, scenario_time_s: float) -> tuple[float, float]:
        """返回指定时刻的附加载荷及其变化率。"""
        time_s = max(0.0, float(scenario_time_s))
        if time_s <= 0:
            return float(self.waypoints[0].force_n), 0.0
        for start, end in zip(self.waypoints, self.waypoints[1:]):
            if time_s <= end.t_s:
                if self.interpolation == "hold":
                    return float(start.force_n), 0.0
                span = float(end.t_s - start.t_s)
                u = (time_s - float(start.t_s)) / span
                delta = float(end.force_n - start.force_n)
                rate = delta / span
                if self.interpolation == "smoothstep":
                    rate *= 6.0 * u * (1.0 - u)
                    u = u * u * (3.0 - 2.0 * u)
                return float(start.force_n + u * delta), float(rate)
        return float(self.waypoints[-1].force_n), 0.0


class ForceSchedulingApproach(_TaskModel):
    """接近、接触确认和调度开始前的稳定参数。"""

    duration_s: Annotated[FiniteFloat, Field(gt=0)] = 1.0
    timeout_s: Annotated[FiniteFloat, Field(gt=0)] = 3.0
    settle_after_contact_s: Annotated[FiniteFloat, Field(ge=0)] = 0.3
    feedforward_force_n: Annotated[FiniteFloat, Field(ge=0)] = 1.0


class OracleSchedulerTaskConfig(_TaskModel):
    """Oracle 目标力调度器的任务级配置。"""

    safety_factor: Annotated[FiniteFloat, Field(ge=1)] = 1.5
    min_force_n: Annotated[FiniteFloat, Field(ge=0)] = 0.5
    max_force_n: Annotated[FiniteFloat, Field(gt=0)] = 8.0
    max_force_rate_n_s: Annotated[FiniteFloat, Field(gt=0)] = 4.0
    friction_floor: Annotated[FiniteFloat, Field(gt=0)] = 0.05

    @model_validator(mode="after")
    def validate_force_limits(self) -> "OracleSchedulerTaskConfig":
        """要求最大目标力不小于最小目标力。"""
        if self.max_force_n < self.min_force_n:
            raise ValueError("max_force_n must not be smaller than min_force_n")
        return self

    def to_runtime_config(self) -> TargetForceSchedulerConfig:
        """转换为与仿真无关的纯调度器配置。"""
        return TargetForceSchedulerConfig(**self.model_dump())


class ForceSchedulingMetricsConfig(_TaskModel):
    """目标力调度场景的验收门限。"""

    ignore_initial_s: Annotated[FiniteFloat, Field(ge=0)] = 0.2
    slip_threshold_m: Annotated[FiniteFloat, Field(gt=0)] = 0.002
    force_rmse_threshold_n: Annotated[FiniteFloat, Field(gt=0)] = 0.5


class AdaptivePriorTaskConfig(_TaskModel):
    """独立于场景真值的摩擦先验与连续增力参数。"""

    left_friction: Annotated[FiniteFloat, Field(gt=0)] = 0.6
    right_friction: Annotated[FiniteFloat, Field(gt=0)] = 0.6
    gap_gain_per_s: Annotated[FiniteFloat, Field(gt=0)] = 8.0
    load_rate_gain: Annotated[FiniteFloat, Field(ge=0)] = 1.0
    filter_tau_s: Annotated[FiniteFloat, Field(gt=0)] = 0.05


class ForceSchedulingSolverConfig(_TaskModel):
    """长时准静态抓取使用的 MuJoCo 摩擦求解设置。"""

    noslip_iterations: Annotated[int, Field(ge=0)] = 5


class ForceSchedulingTask(_TaskModel):
    """目标力调度任务；提供 adaptive_prior 时仅由触觉生成承载需求。"""

    schema_version: Literal[1]
    name: Annotated[str, Field(min_length=1)]
    cube_mass_kg: Annotated[FiniteFloat, Field(ge=0.05)] = 0.05
    friction_coefficient: Annotated[FiniteFloat, Field(gt=0)] = 0.8
    object_material: ObjectMaterial = "hard"
    approach: ForceSchedulingApproach = ForceSchedulingApproach()
    scheduler: OracleSchedulerTaskConfig = OracleSchedulerTaskConfig()
    adaptive_prior: AdaptivePriorTaskConfig | None = None
    unified_adaptive: UnifiedAdaptiveConfig | None = None
    downward_load: DownwardLoadReference
    metrics: ForceSchedulingMetricsConfig = ForceSchedulingMetricsConfig()
    solver: ForceSchedulingSolverConfig = ForceSchedulingSolverConfig()
    control_period_s: Annotated[FiniteFloat, Field(gt=0)] = 0.002

    @model_validator(mode="after")
    def validate_adaptive_modes(self) -> "ForceSchedulingTask":
        """显式策略只能选一种，避免两个状态机同时拥有目标。"""
        if self.adaptive_prior is not None and self.unified_adaptive is not None:
            raise ValueError("adaptive_prior 与 unified_adaptive 互斥")
        return self

    @classmethod
    def load(cls, path: str | Path) -> "ForceSchedulingTask":
        """从 YAML 文件加载并校验目标力调度任务。"""
        task_path = Path(path)
        try:
            with task_path.open(encoding="utf-8") as stream:
                raw = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ForceSchedulingConfigError(f"invalid YAML in {task_path}") from error
        if raw is None:
            raise ForceSchedulingConfigError(f"force scheduling task is empty: {task_path}")
        if not isinstance(raw, dict):
            raise ForceSchedulingConfigError(
                f"force scheduling task root must be a mapping: {task_path}"
            )
        if isinstance(raw.get("definition"), dict):
            raw = raw["definition"]
        try:
            return cls.model_validate(raw)
        except ValidationError as error:
            raise ForceSchedulingConfigError(str(error)) from error


@dataclass(frozen=True, slots=True)
class ForceSchedulingResult:
    """一次目标力调度仿真的主要评价指标。"""

    contact_time_s: float
    scenario_start_time_s: float
    scenario_duration_s: float
    max_tangential_displacement_m: float
    force_tracking_rmse_n: float
    mean_target_force_n: float
    peak_target_force_n: float
    final_target_force_n: float
    minimum_friction_margin_n: float
    peak_friction_utilization: float
    target_force_maximum_ratio: float
    target_force_rate_limited_ratio: float
    simulation_stable: bool
    slip_passed: bool
    force_tracking_passed: bool
    capacity_limited: bool = False
    failure_reason: str | None = None

    @property
    def passed(self) -> bool:
        """是否同时满足稳定性、滑移和力跟踪验收。"""
        return (
            self.simulation_stable
            and self.slip_passed
            and self.force_tracking_passed
            and not self.capacity_limited
            and self.failure_reason is None
        )


def _tangential_displacement(position: np.ndarray, reference: np.ndarray) -> float:
    """返回物体在指尖 YZ 接触平面内的位移。"""
    return float(np.linalg.norm(position[1:3] - reference[1:3]))


def _plot_force_scheduling(
    path: Path, rows: list[dict[str, float | str]], *, task: ForceSchedulingTask
) -> None:
    """分开标记控制观测与真值参考；不把承载缺口画成测得的力。"""
    scenario_rows = [row for row in rows if row["phase"] == "schedule_load"]
    if not scenario_rows:
        raise ValueError("cannot plot an empty force scheduling trace")
    plt = science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)
    rows = scenario_rows
    times = np.asarray([float(row["scenario_time_s"]) for row in rows])
    figure, axes = plt.subplots(
        3,
        1,
        figsize=paper_figsize(6.2),
        sharex=True,
        constrained_layout=True,
    )
    axes[0].plot(
        times,
        [float(row["scheduled_target_force_n"]) for row in rows],
        color="0.15",
        linestyle="--",
        label=r"$F_d$ (target)",
    )
    axes[0].plot(
        times,
        [float(row["filtered_normal_force_n"]) for row in rows],
        color="#0072B2",
        linestyle="-",
        label=r"$F_n$ (tactile, filtered)",
    )
    axes[0].set_title("(a) Mean-side grip-force tracking", loc="left", fontsize=9)
    axes[0].set_ylabel(r"$F_d,\ F_n$ (N)")
    axes[0].legend(loc="upper right", fontsize=8)

    # 旧轨迹没有逐侧实测切向合力，不从真值余量反推或伪造观测。
    measured_keys = ("measured_left_tangential_n", "measured_right_tangential_n")
    if all(all(key in row for key in measured_keys) for row in rows):
        axes[1].plot(
            times,
            [sum(float(row[key]) for key in measured_keys) for row in rows],
            color="#0072B2",
            linestyle="-",
            label=r"$T$ (tactile)",
        )
    axes[1].plot(
        times,
        [float(row["tangential_demand_n"]) for row in rows],
        color="0.15",
        linestyle="--",
        label=r"$D^{\mathrm{gt}}$ (true load)",
    )
    axes[1].plot(
        times,
        [float(row["available_friction_n"]) for row in rows],
        color="#D55E00",
        linestyle=":",
        label=r"$C^{\mathrm{gt}}$ (true capacity)",
    )
    axes[1].set_title("(b) Tactile load and ground-truth references", loc="left", fontsize=9)
    axes[1].set_ylabel("Tangential force (N)")
    axes[1].legend(loc="upper right", fontsize=8)

    axes[2].plot(
        times,
        [1000.0 * float(row["tangential_displacement_m"]) for row in rows],
        color="#0072B2",
        linestyle="-",
        label=r"$d_t^{\mathrm{gt}}$ (object displacement)",
    )
    axes[2].axhline(
        1000.0 * float(task.metrics.slip_threshold_m),
        color="0.35",
        linestyle="--",
        linewidth=0.8,
        label=r"$d_{\mathrm{lim}}$ (acceptance limit)",
    )
    axes[2].set_title("(c) Independent displacement evaluation", loc="left", fontsize=9)
    axes[2].set_ylabel(r"$d_t^{\mathrm{gt}}$ (mm)")
    axes[2].set_xlabel(r"Time since support removal $t$ (s)")
    axes[2].legend(loc="lower right", fontsize=8)
    for axis in axes:
        axis.axvline(0, color="0.55", linestyle=":", linewidth=0.7)
        axis.set_xlim(left=0, right=float(times[-1]))
    path.parent.mkdir(parents=True, exist_ok=True)
    save_publication_figure(figure, path)
    if path.suffix.lower() != ".pdf":
        save_publication_figure(figure, path.with_suffix(".pdf"))
    plt.close(figure)


def run_force_scheduling(
    profile_path: Path | GripperProfile = DEFAULT_PROFILE,
    *,
    task: ForceSchedulingTask,
    output_csv: Path | None = None,
    output_plot: Path | None = None,
) -> ForceSchedulingResult:
    """在共用载荷场景中运行 Oracle 或固定先验自适应调度。"""
    profile = (
        validate_resolved_profile(profile_path)
        if isinstance(profile_path, GripperProfile)
        else load_profile(profile_path)
    )
    if profile.normal_force is None:
        raise ValueError("force scheduling requires profile control.force")
    model = build_custom_grasp_model(
        profile,
        cube_mass=float(task.cube_mass_kg),
        object_material=task.object_material,
        friction_coefficient=float(task.friction_coefficient),
    )
    # 默认抓取场景面向较短扰动，未启用 noslip 后处理。目标力调度需要持续
    # 数秒抵抗准静态载荷，因此在任务中显式启用摩擦锥内的无滑移迭代，避免
    # 约束软化造成非物理的长期匀速爬移；超过摩擦锥的真实滑移仍会保留。
    model.opt.noslip_iterations = int(task.solver.noslip_iterations)
    if task.control_period_s + 1e-12 < float(model.opt.timestep):
        raise ValueError("control_period_s must not be smaller than the physics timestep")

    data = mujoco.MjData(model)
    control_timer = SimulationTimer(float(task.control_period_s), float(data.time))
    reader = _prefixed_reader(model, profile)
    controller_type = (
        DMAdmittanceController
        if profile.normal_force.admittance is not None
        else NormalForceController
    )
    if task.unified_adaptive is not None and controller_type is not DMAdmittanceController:
        force = profile.normal_force
        if (
            force.adrc is not None
            or force.torque_adrc is not None
            or force.stiffness_rate is not None
            or force.torque_feedback_gain
        ):
            raise ValueError("统一自适应仿真只支持导纳或位置式 PID")
    controller = controller_type.from_profile(
        model,
        profile,
        name_prefix=GRIPPER_PREFIX,
        **(
            {"saturation_feedback": True}
            if task.unified_adaptive is not None and controller_type is DMAdmittanceController
            else {}
        ),
    )
    scheduler = OracleTargetForceScheduler(task.scheduler.to_runtime_config())
    adaptive = (
        None
        if task.adaptive_prior is None
        else AdaptiveLoadScheduler(
            AdaptiveLoadConfig(
                **task.adaptive_prior.model_dump(),
                safety_factor=task.scheduler.safety_factor,
                min_force_n=task.scheduler.min_force_n,
                max_force_n=task.scheduler.max_force_n,
                max_force_rate_n_s=task.scheduler.max_force_rate_n_s,
            )
        )
    )
    unified = (
        None if task.unified_adaptive is None else UnifiedAdaptivePolicy(task.unified_adaptive)
    )
    adaptive_diagnostics: dict[str, object] = {}
    noise_rng = np.random.default_rng(int(profile.normal_force.sensor_noise_seed))
    support_id = model.geom(SUPPORT_GEOM_NAME).id
    cube_body_id = model.body(CUBE_BODY_NAME).id
    cube_joint_id = model.joint(CUBE_JOINT_NAME).id
    cube_geom_id = model.geom(f"{CUBE_PREFIX}target_cube_geom").id
    cube_dof = int(model.jnt_dofadr[cube_joint_id])
    taxel_geom_sides = _taxel_geom_sides(model, profile)
    cube_mass = float(model.body_mass[cube_body_id])
    gravity_tangent = cube_mass * np.asarray(model.opt.gravity[1:3], dtype=np.float64)
    gravity_direction = gravity_tangent / max(float(np.linalg.norm(gravity_tangent)), 1e-12)

    contact_time_s: float | None = None
    scenario_start_time_s: float | None = None
    release_position: np.ndarray | None = None
    force_command = None
    execution_limited = False
    schedule_command = None
    simulation_stable = True
    rows: list[dict[str, float | str]] = []
    max_duration = (
        float(task.approach.timeout_s)
        + float(task.approach.settle_after_contact_s)
        + task.downward_load.duration_s
        + float(task.control_period_s)
    )
    steps = math.ceil(max_duration / float(model.opt.timestep))
    for _ in range(steps):
        time_s = float(data.time)
        if scenario_start_time_s is None:
            scenario_time_s = 0.0
            phase = "approach_contact" if contact_time_s is None else "contact_settle"
        else:
            scenario_time_s = time_s - scenario_start_time_s
            phase = "schedule_load"
            if scenario_time_s > task.downward_load.duration_s:
                break

        additional_load_n, additional_load_rate_n_s = task.downward_load.sample_at(scenario_time_s)
        applied_tangent = gravity_direction * additional_load_n
        demand_vector = gravity_tangent + applied_tangent
        tangential_demand_n = float(np.linalg.norm(demand_vector))
        data.xfrc_applied[cube_body_id] = 0.0
        if scenario_start_time_s is not None:
            data.xfrc_applied[cube_body_id, 1:3] = applied_tangent

        target_position = profile.open_control + min(
            1.0, time_s / float(task.approach.duration_s)
        ) * (profile.closed_control - profile.open_control)
        control_dt = control_timer.pop_due(time_s)
        if control_dt is not None:
            tactile = reader.read(data)
            measurement = _tactile_measurement(
                tactile,
                left_normal_std_n=float(profile.normal_force.sensor_taxel_normal_noise_std_n[0]),
                right_normal_std_n=float(profile.normal_force.sensor_taxel_normal_noise_std_n[1]),
                left_shear_std_n=float(profile.normal_force.sensor_taxel_shear_noise_std_n[0]),
                right_shear_std_n=float(profile.normal_force.sensor_taxel_shear_noise_std_n[1]),
                rng=noise_rng,
            )
            measured_capacity = measurement.normal_capacity
            if unified is not None:
                unified_command = unified.update(
                    measurement.left.reshape(3, -1).T,
                    measurement.right.reshape(3, -1).T,
                    time_s=time_s,
                    measured_force_n=0.5 * measured_capacity.normal_force_n,
                    execution_limited=execution_limited,
                    enabled=scenario_start_time_s is not None,
                )
                schedule_command = unified_command.load
                adaptive_diagnostics = {
                    "scheduler_kind": "unified_adaptive",
                    "capacity_limited": schedule_command.capacity_limited,
                    **unified_command.trace_fields(),
                }
            elif adaptive is not None:
                # 承载仅从触觉测量产生；场景真值继续供物理施加载荷与离线评分。
                schedule_command = adaptive.update(
                    left_tangential_n=float(np.linalg.norm(measurement.left_force[:2])),
                    right_tangential_n=float(np.linalg.norm(measurement.right_force[:2])),
                    dt=control_dt,
                )
                adaptive_diagnostics = {
                    "scheduler_kind": "adaptive_prior",
                    "measured_left_tangential_n": float(np.linalg.norm(measurement.left_force[:2])),
                    "measured_right_tangential_n": float(
                        np.linalg.norm(measurement.right_force[:2])
                    ),
                    "measured_tangential_force_n": schedule_command.measured_tangential_force_n,
                    "estimated_load_rate_n_s": schedule_command.load_rate_n_s,
                    "load_target_force_n": schedule_command.load_force_n,
                    "schedule_gap_n": schedule_command.schedule_gap_n,
                    "capacity_limited": schedule_command.capacity_limited,
                }
            else:
                schedule_command = scheduler.update(
                    tangential_demand_n=tangential_demand_n,
                    friction_coefficient=float(task.friction_coefficient),
                    dt=control_dt,
                )
            force_command = controller.step(
                data,
                observation=ForceControlObservation(
                    time_s=time_s,
                    approach_position=target_position,
                    total_normal_force_n=measured_capacity.normal_force_n,
                    left_normal_force_n=measured_capacity.left_normal_force_n,
                    right_normal_force_n=measured_capacity.right_normal_force_n,
                    dt=control_dt,
                ),
                reference=ForceControlReference(
                    target_force_n=schedule_command.target_force_n,
                    approach_feedforward_force_n=float(task.approach.feedforward_force_n),
                    target_force_rate_n_s=schedule_command.target_force_rate_n_s,
                ),
            )
            if contact_time_s is None and force_command.state == "force_tracking":
                contact_time_s = time_s
            if (
                contact_time_s is not None
                and scenario_start_time_s is None
                and time_s - contact_time_s >= float(task.approach.settle_after_contact_s)
            ):
                scenario_start_time_s = time_s
                model.geom_contype[support_id] = 0
                model.geom_conaffinity[support_id] = 0
                release_position = data.xpos[cube_body_id].copy()

        if force_command is None or schedule_command is None:
            raise RuntimeError("control timer did not produce an initial command")
        if isinstance(controller, DMAdmittanceController) or unified is not None:
            force_command = replace(force_command, mit=controller.apply_held_command(data))
        if unified is not None:
            if isinstance(controller, DMAdmittanceController):
                execution_limited = controller.admittance.execution_limited
            else:
                # PID 保留原有积分限幅；上层只消费可观测的位置修正／协议／力矩边界。
                mit = force_command.mit
                execution_limited = (
                    abs(force_command.position_adjustment)
                    >= profile.normal_force.max_position_adjustment - 1e-9
                    or abs(mit.torque) >= profile.mit.t_max - 1e-9
                    or mit.target_position <= profile.mit.p_min + 1e-9
                    or mit.target_position >= profile.mit.p_max - 1e-9
                )
        mujoco.mj_step(model, data)
        if (
            data.time <= time_s
            or not np.isfinite(data.qpos).all()
            or not np.isfinite(data.qvel).all()
        ):
            simulation_stable = False
            break

        capacity = _friction_capacity(
            model,
            data,
            taxel_geom_sides=taxel_geom_sides,
            cube_geom_id=cube_geom_id,
        )
        position = data.xpos[cube_body_id].copy()
        displacement = (
            0.0
            if release_position is None
            else _tangential_displacement(position, release_position)
        )
        friction_margin_n = capacity.available_friction_n - tangential_demand_n
        friction_utilization = (
            tangential_demand_n / capacity.available_friction_n
            if capacity.available_friction_n > 1e-12
            else math.inf
        )
        rows.append(
            {
                "time_s": float(data.time),
                "phase": phase,
                "scenario_time_s": scenario_time_s,
                "true_friction_coefficient": float(task.friction_coefficient),
                "cube_mass_kg": cube_mass,
                "gravity_tangential_force_n": float(np.linalg.norm(gravity_tangent)),
                "additional_downward_force_n": additional_load_n,
                "additional_downward_force_rate_n_s": additional_load_rate_n_s,
                "tangential_demand_n": tangential_demand_n,
                "measured_left_tangential_n": float(np.linalg.norm(measurement.left_force[:2])),
                "measured_right_tangential_n": float(np.linalg.norm(measurement.right_force[:2])),
                "raw_target_force_n": schedule_command.raw_target_force_n,
                "scheduled_target_force_n": schedule_command.target_force_n,
                "scheduled_target_force_rate_n_s": schedule_command.target_force_rate_n_s,
                "target_limited_by": ",".join(schedule_command.limited_by),
                "filtered_normal_force_n": force_command.filtered_force_n,
                "measured_normal_force_n": force_command.measured_force_n,
                "force_tracking_error_n": force_command.force_error_n,
                "left_normal_force_n": capacity.left_normal_force_n,
                "right_normal_force_n": capacity.right_normal_force_n,
                "available_friction_n": capacity.available_friction_n,
                "friction_margin_n": friction_margin_n,
                "friction_utilization": friction_utilization,
                "active_taxel_contacts": capacity.active_contacts,
                "tangential_displacement_m": displacement,
                "cube_y": float(position[1]),
                "cube_z": float(position[2]),
                "cube_vy": float(data.qvel[cube_dof + 1]),
                "cube_vz": float(data.qvel[cube_dof + 2]),
                "motor_torque_n_m": force_command.mit.torque,
                "motor_position_rad": force_command.mit.position,
                "motor_velocity_rad_s": force_command.mit.velocity,
                "requested_position_rad": force_command.mit.target_position,
                "force_feedforward_torque_n_m": force_command.force_feedforward_torque,
                "execution_limited": execution_limited,
                **adaptive_diagnostics,
            }
        )

    if contact_time_s is None or scenario_start_time_s is None:
        simulation_stable = False
    scenario_rows = [
        row
        for row in rows
        if row["phase"] == "schedule_load"
        and float(row["scenario_time_s"]) >= float(task.metrics.ignore_initial_s)
    ]
    if scenario_rows:
        errors = np.asarray([float(row["force_tracking_error_n"]) for row in scenario_rows])
        targets = np.asarray([float(row["scheduled_target_force_n"]) for row in scenario_rows])
        displacements = np.asarray(
            [float(row["tangential_displacement_m"]) for row in scenario_rows]
        )
        margins = np.asarray([float(row["friction_margin_n"]) for row in scenario_rows])
        utilizations = np.asarray([float(row["friction_utilization"]) for row in scenario_rows])
        maximum_flags = ["maximum" in str(row["target_limited_by"]) for row in scenario_rows]
        rate_flags = ["rate" in str(row["target_limited_by"]) for row in scenario_rows]
        force_rmse = float(np.sqrt(np.mean(errors**2)))
        maximum_displacement = float(np.max(displacements))
        minimum_margin = float(np.min(margins))
        peak_utilization = float(np.max(utilizations))
        mean_target = float(np.mean(targets))
        peak_target = float(np.max(targets))
        final_target = float(targets[-1])
        maximum_ratio = float(np.mean(maximum_flags))
        rate_ratio = float(np.mean(rate_flags))
    else:
        force_rmse = math.inf
        maximum_displacement = math.inf
        minimum_margin = -math.inf
        peak_utilization = math.inf
        mean_target = math.nan
        peak_target = math.nan
        final_target = math.nan
        maximum_ratio = math.nan
        rate_ratio = math.nan

    if (adaptive is not None or unified is not None) and scenario_rows:
        # 初步自适应验收包含撤支撑后的全部位移，不忽略启动恢复期间的滑移。
        maximum_displacement = max(
            float(row["tangential_displacement_m"])
            for row in rows
            if row["phase"] == "schedule_load"
        )

    if rows and output_csv is not None:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        with output_csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    if rows and output_plot is not None:
        _plot_force_scheduling(output_plot, rows, task=task)

    return ForceSchedulingResult(
        contact_time_s=(math.inf if contact_time_s is None else contact_time_s),
        scenario_start_time_s=(
            math.inf if scenario_start_time_s is None else scenario_start_time_s
        ),
        scenario_duration_s=(
            0.0
            if scenario_start_time_s is None or not scenario_rows
            else float(scenario_rows[-1]["scenario_time_s"])
        ),
        max_tangential_displacement_m=maximum_displacement,
        force_tracking_rmse_n=force_rmse,
        mean_target_force_n=mean_target,
        peak_target_force_n=peak_target,
        final_target_force_n=final_target,
        minimum_friction_margin_n=minimum_margin,
        peak_friction_utilization=peak_utilization,
        target_force_maximum_ratio=maximum_ratio,
        target_force_rate_limited_ratio=rate_ratio,
        simulation_stable=simulation_stable,
        slip_passed=maximum_displacement <= float(task.metrics.slip_threshold_m),
        force_tracking_passed=force_rmse <= float(task.metrics.force_rmse_threshold_n),
        capacity_limited=any(bool(row.get("capacity_limited", False)) for row in rows),
        failure_reason=None
        if unified is None or unified.latest is None
        else unified.latest.failure_reason,
    )


__all__ = [
    "DownwardLoadReference",
    "DownwardLoadWaypoint",
    "ForceSchedulingConfigError",
    "ForceSchedulingResult",
    "ForceSchedulingSolverConfig",
    "ForceSchedulingTask",
    "run_force_scheduling",
]

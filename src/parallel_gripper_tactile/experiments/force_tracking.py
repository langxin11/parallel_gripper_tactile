"""时变目标法向力跟踪实验。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Annotated, Literal

import mujoco
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator
import yaml

from ..control import (
    ForceControlObservation,
    ForceControlReference,
    ForceSemantics,
    ForceTrackingController,
    NormalForceController,
)
from ..profiles import (
    STIFFNESS_ESTIMATOR_METHODS,
    AdrcControl,
    GripperProfile,
    MITTorqueControl,
    StiffnessEstimatorMethod,
    TorqueAdrcControl,
    load_profile,
)
from ..scenes.custom import (
    CUBE_PREFIX,
    DEFAULT_CUBE_HALF_CONTACT_SIDE,
    DEFAULT_CUBE_HALF_THICKNESS,
    DEFAULT_CUBE_MASS,
    DEFAULT_PROFILE,
    GRIPPER_PREFIX,
    ObjectContactModel,
    ObjectMaterial,
    SUPPORT_GEOM_NAME,
    build_custom_grasp_model,
)
from ..timing import RealtimePacer, SimulationTimer
from .grasp import (
    CUBE_BODY_NAME,
    CUBE_JOINT_NAME,
    _friction_capacity,
    _prefixed_reader,
    _tactile_measurement,
    _taxel_geom_sides,
)


ControllerVariant = Literal[
    "pid-only",
    "pid-torque-ff",
    "pid-stiffness-ff",
    "full",
    "direct-torque",
    "adrc",
    "adrc-torque",
    "adrc-torque-td",
]
CONTROLLER_VARIANTS: tuple[ControllerVariant, ...] = (
    "pid-only",
    "pid-torque-ff",
    "pid-stiffness-ff",
    "full",
    "direct-torque",
    "adrc",
    "adrc-torque",
    "adrc-torque-td",
)


def configure_force_controller(
    profile: GripperProfile,
    *,
    variant: ControllerVariant = "full",
    stiffness_estimator_method: StiffnessEstimatorMethod | None = None,
    sensor_noise_seed: int | None = None,
    torque_adrc_override: TorqueAdrcControl | None = None,
) -> GripperProfile:
    """返回用于公平消融的力控 profile 副本，不修改磁盘源配置。

    Args:
        profile: 作为不变基线的 profile。
        variant: 需要启用的控制器变体。
        stiffness_estimator_method: 可选的刚度估计方法覆盖。
        sensor_noise_seed: 可选的传感器噪声随机种子覆盖。
        torque_adrc_override: 仅 ``adrc-torque`` 变体使用的二阶 LADRC 参数覆盖。

    Raises:
        ValueError: 变体、种子或覆盖参数与当前控制器不兼容时抛出。
    """
    if variant not in CONTROLLER_VARIANTS:
        choices = ", ".join(CONTROLLER_VARIANTS)
        raise ValueError(f"controller_variant must be one of: {choices}")
    if sensor_noise_seed is not None and sensor_noise_seed < 0:
        raise ValueError("sensor_noise_seed must be non-negative")
    if torque_adrc_override is not None and variant not in {"adrc-torque", "adrc-torque-td"}:
        raise ValueError("torque_adrc_override requires a torque ADRC controller variant")
    if (
        stiffness_estimator_method is not None
        and stiffness_estimator_method not in STIFFNESS_ESTIMATOR_METHODS
    ):
        choices = ", ".join(STIFFNESS_ESTIMATOR_METHODS)
        raise ValueError(f"stiffness_estimator_method must be one of: {choices}")
    if not isinstance(profile.control, MITTorqueControl) or profile.normal_force is None:
        raise ValueError("controller ablation requires MIT torque control with control.force")
    force = profile.normal_force
    stiffness = force.stiffness
    if stiffness is None and variant not in {"pid-only", "full"}:
        raise ValueError("controller ablation requires control.force.stiffness")

    if variant == "pid-only" and stiffness is not None:
        stiffness = stiffness.model_copy(update={"enabled": False})
    elif variant == "pid-torque-ff" and stiffness is not None:
        stiffness = stiffness.model_copy(update={"enabled": True, "position_feedforward_gain": 0.0})
    elif variant == "pid-stiffness-ff" and stiffness is not None:
        stiffness = stiffness.model_copy(update={"enabled": True, "torque_feedforward_gain": 0.0})
    elif variant == "direct-torque" and stiffness is not None:
        # 直接力矩式对照：保留刚度估计（trace 中刚度曲线可比）并关闭刚度位置前馈，
        # 力矩前馈增益不动。MIT kp/kd 不在 profile 层清零——接近阶段共享同一组
        # 位置伺服增益建立接触，清零动作由控制器在跟踪阶段逐周期 override。
        stiffness = stiffness.model_copy(update={"enabled": True, "position_feedforward_gain": 0.0})
    elif variant == "adrc" and stiffness is not None:
        # LADRC 外环取代刚度位置前馈的角色：保留刚度估计（trace 中刚度曲线
        # 可比），位置前馈置 0，力矩前馈增益不动（对标 full 变体）。
        stiffness = stiffness.model_copy(update={"enabled": True, "position_feedforward_gain": 0.0})
    elif variant in {"adrc-torque", "adrc-torque-td"} and stiffness is not None:
        # 二阶直接力矩 LADRC 使用刚度估计调度 b0；机构力矩前馈作为名义模型
        # 输入，LESO 仅观察实际总力矩扣除该前馈后的残差通道。
        stiffness = stiffness.model_copy(
            update={
                "enabled": True,
                "position_feedforward_gain": 0.0,
                "torque_feedforward_gain": 1.0,
            }
        )

    if stiffness_estimator_method is not None:
        if stiffness is None:
            raise ValueError("stiffness_estimator_method requires control.force.stiffness")
        stiffness = stiffness.model_copy(update={"method": stiffness_estimator_method})

    force_updates: dict[str, object] = {}
    if stiffness is not None:
        force_updates["stiffness"] = stiffness
    if variant == "direct-torque":
        # 力误差直接进入 MIT 前馈力矩的增益；1.0 表示误差力矩全额注入。
        force_updates["torque_feedback_gain"] = 1.0
        force_updates["adrc"] = None
        force_updates["torque_adrc"] = None
    if variant == "adrc":
        # 变体默认值集中在此处代码与 AdrcControl 默认值中，profile 无需显式配置。
        force_updates["adrc"] = AdrcControl()
        force_updates["torque_adrc"] = None
        force_updates["torque_feedback_gain"] = 0.0
    if variant in {"adrc-torque", "adrc-torque-td"}:
        # 跟踪阶段旁路 MIT 阻抗，接近阶段仍使用 profile 中的 kp/kd。
        default_torque_adrc = TorqueAdrcControl(
            tracking_differentiator_bandwidth_rad_s=(180.0 if variant == "adrc-torque-td" else None)
        )
        force_updates["torque_adrc"] = torque_adrc_override or default_torque_adrc
        force_updates["adrc"] = None
        force_updates["torque_feedback_gain"] = 0.0
    if sensor_noise_seed is not None:
        force_updates["sensor_noise_seed"] = sensor_noise_seed
    configured_force = force.model_copy(update=force_updates)
    configured_control = profile.control.model_copy(update={"force": configured_force})
    return profile.model_copy(update={"control": configured_control})


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
        return self.sample_at(tracking_time_s)[0]

    def sample_at(self, tracking_time_s: float) -> tuple[float, float, float]:
        """返回指定时刻的目标力及其一、二阶时间导数。"""
        time_s = max(0.0, float(tracking_time_s))
        if time_s <= self.waypoints[0].t_s:
            return (float(self.waypoints[0].force_n), 0.0, 0.0)
        for start, end in zip(self.waypoints, self.waypoints[1:]):
            if time_s <= end.t_s:
                if self.interpolation == "hold":
                    return (float(start.force_n), 0.0, 0.0)
                span = end.t_s - start.t_s
                u = (time_s - start.t_s) / span
                delta = float(end.force_n - start.force_n)
                rate = delta / span
                acceleration = 0.0
                if self.interpolation == "smoothstep":
                    rate *= 6.0 * u * (1.0 - u)
                    acceleration = delta * (6.0 - 12.0 * u) / span**2
                    u = u * u * (3.0 - 2.0 * u)
                return (float(start.force_n + u * delta), rate, acceleration)
        return (float(self.waypoints[-1].force_n), 0.0, 0.0)


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
            raise ForceTrackingConfigError(
                f"force tracking task root must be a mapping: {task_path}"
            )
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
    rise_time_s: float | None
    overshoot_ratio: float | None
    settling_time_s: float | None
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
    axes[0].plot(
        times, measured, color=colors["orange"], label="measured", linewidth=0.8, alpha=0.7
    )
    force_label = (
        "Total normal force (N)"
        if rows[0].get("force_semantics") == "total"
        else "Mean side normal force (N)"
    )
    axes[0].set_ylabel(force_label)
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
    position_saturation = float(np.mean((positions <= p_min + 1e-6) | (positions >= p_max - 1e-6)))
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


def _evaluate_step_transients(
    rows: list[dict[str, float | str]],
    *,
    waypoints: tuple[ForceWaypoint, ...],
    interpolation: str,
    ignore_initial_s: float,
) -> tuple[float | None, float | None, float | None]:
    """计算加载阶跃的上升时间、超调比与稳定时间。

    仅 ``hold`` 插值任务存在可评估的加载阶跃；在相邻 waypoint 中取力上升量最大
    （且不低于 1 N）的一对作为阶跃，统计滤波后法向力在其后平台段内的瞬态指标。
    任一前提不满足时返回 ``(None, None, None)``。

    Args:
        rows: 实验逐步 trace 行，字段与 ``trace.csv`` 一致。
        waypoints: 任务的目标力 waypoint 序列。
        interpolation: 任务的目标力插值方式。
        ignore_initial_s: 跟踪阶段开头需跳过的秒数。

    Returns:
        ``(rise_time_s, overshoot_ratio, settling_time_s)``，无法判定时对应项为
        ``None``。
    """
    if interpolation != "hold":
        return (None, None, None)
    step_index: int | None = None
    step_delta = 0.0
    for index, (start, end) in enumerate(zip(waypoints, waypoints[1:])):
        delta = float(end.force_n) - float(start.force_n)
        if delta >= 1.0 and delta > step_delta:
            step_delta = delta
            step_index = index
    if step_index is None:
        return (None, None, None)
    step_waypoint = waypoints[step_index + 1]
    t_0 = float(step_waypoint.t_s)
    f_low = float(waypoints[step_index].force_n)
    f_high = float(step_waypoint.force_n)
    # 平台段延伸到下一个力值发生变化的 waypoint；若直至曲线终点都保持不变，则以终点收尾。
    t_end = float(waypoints[-1].t_s)
    for waypoint in waypoints[step_index + 1 :]:
        if float(waypoint.force_n) != f_high:
            t_end = float(waypoint.t_s)
            break
    if t_end - t_0 < 0.5:
        return (None, None, None)
    window = [
        row
        for row in rows
        if row["phase"] == "track_reference"
        and float(row["tracking_time_s"]) >= ignore_initial_s
        and t_0 <= float(row["tracking_time_s"]) < t_end
    ]
    if not window:
        return (None, None, None)
    times = np.asarray([float(row["tracking_time_s"]) for row in window])
    forces = np.asarray([float(row["filtered_normal_force_n"]) for row in window])
    delta_force = f_high - f_low
    rise_mask = forces >= f_low + 0.9 * delta_force
    if not bool(rise_mask.any()):
        return (None, None, None)
    rise_time_s = float(times[int(np.flatnonzero(rise_mask)[0])] - t_0)
    overshoot_ratio = max(0.0, float(np.max(forces) - f_high) / delta_force)
    outside = np.flatnonzero(np.abs(forces - f_high) > 0.05 * delta_force)
    if outside.size == 0:
        # 窗口首采样前就进入稳定带且从无违反，稳定时间记为首采样时刻。
        settling_time_s: float | None = float(times[0] - t_0)
    elif int(outside[-1]) == len(window) - 1:
        settling_time_s = None
    else:
        settling_time_s = float(times[int(outside[-1]) + 1] - t_0)
    return (rise_time_s, overshoot_ratio, settling_time_s)


def run_force_tracking(
    profile_path: Path = DEFAULT_PROFILE,
    *,
    task: ForceTrackingTask,
    cube_half_thickness: float = DEFAULT_CUBE_HALF_THICKNESS,
    cube_half_contact_side: float = DEFAULT_CUBE_HALF_CONTACT_SIDE,
    cube_mass: float = DEFAULT_CUBE_MASS,
    object_material: ObjectMaterial = "hard",
    object_contact_model: ObjectContactModel = "explicit",
    multiccd_enabled: bool = True,
    force_semantics: ForceSemantics = "average_side",
    controller_variant: ControllerVariant = "full",
    stiffness_estimator_method: StiffnessEstimatorMethod | None = None,
    sensor_noise_seed: int | None = None,
    torque_adrc_override: TorqueAdrcControl | None = None,
    output_csv: Path | None = None,
    output_plot: Path | None = None,
    viewer: bool = False,
    render_fps: float = 30.0,
    realtime_factor: float = 1.0,
) -> ForceTrackingResult:
    """运行两阶段目标法向力跟踪测试。"""
    profile = configure_force_controller(
        load_profile(profile_path),
        variant=controller_variant,
        stiffness_estimator_method=stiffness_estimator_method,
        sensor_noise_seed=sensor_noise_seed,
        torque_adrc_override=torque_adrc_override,
    )
    if profile.normal_force is None or profile.mit is None:
        raise ValueError("force tracking requires MIT torque control with control.force")
    if viewer and (render_fps <= 0 or realtime_factor <= 0):
        raise ValueError("render_fps and realtime_factor must be positive when viewer is enabled")
    model = build_custom_grasp_model(
        profile,
        cube_half_thickness=cube_half_thickness,
        cube_half_contact_side=cube_half_contact_side,
        cube_mass=cube_mass,
        object_material=object_material,
        object_contact_model=object_contact_model,
        multiccd_enabled=multiccd_enabled,
    )
    if task.control_period_s + 1e-12 < float(model.opt.timestep):
        raise ValueError("control_period_s must not be smaller than the physics timestep")

    data = mujoco.MjData(model)
    control_timer = SimulationTimer(task.control_period_s, float(data.time))
    reader = _prefixed_reader(model, profile)
    controller: ForceTrackingController = NormalForceController.from_profile(
        model,
        profile,
        name_prefix=GRIPPER_PREFIX,
        force_semantics=force_semantics,
    )
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
    viewer_handle = None
    pacer = None
    next_render_time = 0.0
    render_period = 0.0
    if viewer:
        render_period = 1.0 / render_fps
        from mujoco import viewer as mujoco_viewer

        viewer_handle = mujoco_viewer.launch_passive(
            model, data, show_left_ui=True, show_right_ui=True
        )
        viewer_handle.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer_handle.cam.lookat[:] = (0.0, -0.12, 0.07)
        viewer_handle.cam.distance = 0.38
        viewer_handle.cam.azimuth = 135
        viewer_handle.cam.elevation = -25
        wall_start = time.monotonic()
        pacer = RealtimePacer(realtime_factor, float(data.time), wall_start)
        next_render_time = wall_start
    try:
        for _ in range(steps):
            if viewer_handle is not None and not viewer_handle.is_running():
                break
            time_s = float(data.time)
            if tracking_start_time_s is None:
                tracking_time_s = 0.0
                target_force_n, target_force_rate_n_s, target_force_acceleration_n_s2 = (
                    task.reference.sample_at(0.0)
                )
                phase = "approach_contact" if contact_time_s is None else "contact_settle"
            else:
                tracking_time_s = time_s - tracking_start_time_s
                target_force_n, target_force_rate_n_s, target_force_acceleration_n_s2 = (
                    task.reference.sample_at(tracking_time_s)
                )
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
                force_command = controller.step(
                    data,
                    observation=ForceControlObservation(
                        time_s=time_s,
                        approach_position=target_position,
                        total_normal_force_n=feedback_capacity.normal_force_n,
                        left_normal_force_n=feedback_capacity.left_normal_force_n,
                        right_normal_force_n=feedback_capacity.right_normal_force_n,
                        dt=control_dt,
                    ),
                    reference=ForceControlReference(
                        target_force_n=target_force_n,
                        approach_feedforward_force_n=task.approach.feedforward_force_n,
                        target_force_rate_n_s=target_force_rate_n_s,
                        target_force_acceleration_n_s2=target_force_acceleration_n_s2,
                    ),
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
                    "force_semantics": force_semantics,
                    "multiccd_enabled": str(multiccd_enabled).lower(),
                    "tracking_time_s": tracking_time_s,
                    "control_state": force_command.state,
                    "target_normal_force_n": force_command.target_force_n,
                    "target_force_rate_n_s": target_force_rate_n_s,
                    "target_force_acceleration_n_s2": target_force_acceleration_n_s2,
                    "measured_normal_force_n": force_command.measured_force_n,
                    "filtered_normal_force_n": force_command.filtered_force_n,
                    "torque_adrc_measurement_n": (
                        force_command.torque_adrc_measurement_n
                        if force_command.torque_adrc_measurement_n is not None
                        else math.nan
                    ),
                    "tracking_error_n": force_command.force_error_n,
                    "control": motor_command.target_position,
                    "drive_position_rad": motor_command.position,
                    "drive_velocity_rad_s": motor_command.velocity,
                    "motor_torque_n_m": motor_command.torque,
                    "force_position_adjustment_rad": force_command.position_adjustment,
                    "pid_position_adjustment_rad": force_command.pid_position_adjustment,
                    "stiffness_position_adjustment_rad": (
                        force_command.stiffness_position_adjustment
                    ),
                    "force_feedforward_torque_n_m": force_command.force_feedforward_torque,
                    "mit_feedforward_torque_n_m": motor_command.feedforward_torque,
                    "torque_adrc_estimated_force_n": (
                        force_command.torque_adrc_estimated_force_n
                        if force_command.torque_adrc_estimated_force_n is not None
                        else math.nan
                    ),
                    "torque_adrc_estimated_force_rate_n_s": (
                        force_command.torque_adrc_estimated_force_rate_n_s
                        if force_command.torque_adrc_estimated_force_rate_n_s is not None
                        else math.nan
                    ),
                    "torque_adrc_estimated_disturbance_n_s2": (
                        force_command.torque_adrc_estimated_disturbance_n_s2
                        if force_command.torque_adrc_estimated_disturbance_n_s2 is not None
                        else math.nan
                    ),
                    "torque_adrc_reference_force_n": (
                        force_command.torque_adrc_reference_force_n
                        if force_command.torque_adrc_reference_force_n is not None
                        else math.nan
                    ),
                    "torque_adrc_reference_force_rate_n_s": (
                        force_command.torque_adrc_reference_force_rate_n_s
                        if force_command.torque_adrc_reference_force_rate_n_s is not None
                        else math.nan
                    ),
                    "torque_adrc_reference_force_acceleration_n_s2": (
                        force_command.torque_adrc_reference_force_acceleration_n_s2
                        if force_command.torque_adrc_reference_force_acceleration_n_s2 is not None
                        else math.nan
                    ),
                    "torque_adrc_raw_torque_n_m": (
                        force_command.torque_adrc_raw_torque_n_m
                        if force_command.torque_adrc_raw_torque_n_m is not None
                        else math.nan
                    ),
                    "torque_adrc_limited_torque_n_m": (
                        force_command.torque_adrc_limited_torque_n_m
                        if force_command.torque_adrc_limited_torque_n_m is not None
                        else math.nan
                    ),
                    "torque_adrc_residual_torque_n_m": (
                        force_command.torque_adrc_residual_torque_n_m
                        if force_command.torque_adrc_residual_torque_n_m is not None
                        else math.nan
                    ),
                    "torque_adrc_input_gain_n_per_n_m_s2": (
                        force_command.torque_adrc_input_gain_n_per_n_m_s2
                        if force_command.torque_adrc_input_gain_n_per_n_m_s2 is not None
                        else math.nan
                    ),
                    "torque_adrc_rate_limited": str(force_command.torque_adrc_rate_limited).lower(),
                    "torque_adrc_amplitude_limited": str(
                        force_command.torque_adrc_amplitude_limited
                    ).lower(),
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
            if viewer_handle is not None and pacer is not None:
                while float(data.time) > pacer.target_simulation_time(time.monotonic()):
                    time.sleep(0.001)
                now = time.monotonic()
                if now >= next_render_time:
                    viewer_handle.sync()
                    next_render_time = now + render_period
    finally:
        if viewer_handle is not None and viewer_handle.is_running():
            viewer_handle.close()

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
    rise_time_s, overshoot_ratio, settling_time_s = _evaluate_step_transients(
        rows,
        waypoints=task.reference.waypoints,
        interpolation=task.reference.interpolation,
        ignore_initial_s=task.metrics.ignore_initial_s,
    )
    return ForceTrackingResult(
        contact_time_s=math.nan if contact_time_s is None else contact_time_s,
        tracking_start_time_s=math.nan if tracking_start_time_s is None else tracking_start_time_s,
        tracking_duration_s=task.reference.duration_s,
        rmse_n=rmse_n,
        mae_n=mae_n,
        peak_abs_error_n=peak_abs_error_n,
        mean_error_n=mean_error_n,
        final_error_n=final_error_n,
        torque_saturation_ratio=torque_saturation_ratio,
        position_saturation_ratio=position_saturation_ratio,
        mean_estimated_stiffness_n_per_m=mean_stiffness,
        rise_time_s=rise_time_s,
        overshoot_ratio=overshoot_ratio,
        settling_time_s=settling_time_s,
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

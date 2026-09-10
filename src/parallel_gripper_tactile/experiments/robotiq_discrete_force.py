"""Robotiq 2F-85 离散 tendon 命令的力控制验证实验。"""

from __future__ import annotations

from collections import deque
import csv
from dataclasses import dataclass
import gzip
import math
from pathlib import Path
from typing import Annotated, Literal

import mujoco
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, model_validator
import yaml

from robotiq_grasp_core.discrete_force_control import (
    DiscreteForceControlConfig,
    DiscreteForceController,
)
from ..visualization import paper_figsize, save_publication_figure, science_pyplot
from ..config.profiles import load_profile
from ..scenes.robotiq import RobotiqObjectMaterial, load_grasp_model
from ..tactile import create_tactile_reader
from ..timing import SimulationTimer


ControllerVariant = Literal[
    "quantized-pi",
    "fixed-step",
    "adaptive-deadband",
    "predictive",
    "dynamic-step",
]


class _TaskModel(BaseModel):
    """拒绝额外字段的不可变实验配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class CommandConfig(_TaskModel):
    """整数命令空间、目标力与接近参数。"""

    command_min: Annotated[int, Field(ge=0)] = 0
    command_max: Annotated[int, Field(gt=0)] = 255
    min_nonzero_step: Literal[1] = 1
    max_dynamic_step: Annotated[int, Field(gt=0, le=3)] = 3
    max_force_n: Annotated[FiniteFloat, Field(gt=0)] = 8.0
    approach_step: Annotated[int, Field(gt=0)] = 3
    approach_action_interval_s: Annotated[FiniteFloat, Field(gt=0)] = 0.05
    contact_threshold_n: Annotated[FiniteFloat, Field(gt=0)] = 0.2

    @model_validator(mode="after")
    def validate_command(self) -> "CommandConfig":
        """要求命令、目标与安全力范围严格递增。"""
        if self.command_min >= self.command_max:
            raise ValueError("command_min must be smaller than command_max")
        return self


class ForceEstimationConfig(_TaskModel):
    """稳定窗口和测量链路参数。"""

    stable_window_samples: Annotated[int, Field(ge=2)] = 20
    stable_force_range_n: Annotated[FiniteFloat, Field(gt=0)] = 0.1
    min_settle_time_s: Annotated[FiniteFloat, Field(gt=0)] = 0.2
    filter_cutoff_hz: Annotated[FiniteFloat, Field(gt=0)] = 20.0


class DeltaFTickConfig(_TaskModel):
    """单 tick 力增量的在线估计参数。"""

    ema_alpha: Annotated[FiniteFloat, Field(gt=0, le=1)] = 0.3
    min_valid_samples: Annotated[int, Field(gt=0)] = 3
    minimum_value_n: Annotated[FiniteFloat, Field(gt=0)] = 0.02


class DeadbandConfig(_TaskModel):
    """固定基线和自适应保持死区参数。"""

    fixed_n: Annotated[FiniteFloat, Field(gt=0)] = 0.15
    noise_sigma_factor: Annotated[FiniteFloat, Field(ge=0)] = 3.0
    tick_factor_alpha: Annotated[FiniteFloat, Field(ge=0)] = 0.5


class ReactivationConfig(_TaskModel):
    """HOLD 再激活滞回参数。"""

    fixed_margin_n: Annotated[FiniteFloat, Field(ge=0)] = 0.1
    tick_factor_beta: Annotated[FiniteFloat, Field(ge=0)] = 0.7
    duration_s: Annotated[FiniteFloat, Field(gt=0)] = 0.2


class PredictionConfig(_TaskModel):
    """一步预测的保守裕量参数。"""

    noise_factor: Annotated[FiniteFloat, Field(ge=0)] = 2.0
    tick_margin_factor: Annotated[FiniteFloat, Field(ge=0)] = 0.2


class DynamicStepConfig(_TaskModel):
    """动态动作步长参数。"""

    eta: Annotated[FiniteFloat, Field(gt=0)] = 0.6


class SafetyConfig(_TaskModel):
    """动作前安全力预测参数。"""

    force_margin_n: Annotated[FiniteFloat, Field(gt=0)] = 0.3


class QuantizedPIConfig(_TaskModel):
    """统一经过整数 actuator 接口的 PI 对照参数。"""

    kp_tick_per_n: Annotated[FiniteFloat, Field(ge=0)] = 0.8
    ki_tick_per_n_s: Annotated[FiniteFloat, Field(ge=0)] = 0.2
    max_step_per_update: Annotated[FiniteFloat, Field(gt=0)] = 3.0


class ForceWaypoint(_TaskModel):
    """目标力曲线中的一个时间—力节点。"""

    t_s: Annotated[FiniteFloat, Field(ge=0)]
    force_n: Annotated[FiniteFloat, Field(gt=0)]


class ForceReference(_TaskModel):
    """接触稳定后启动的分段目标力曲线。"""

    interpolation: Literal["hold", "linear", "smoothstep"] = "linear"
    waypoints: tuple[ForceWaypoint, ...]

    @model_validator(mode="after")
    def validate_waypoints(self) -> "ForceReference":
        """要求曲线从零时刻开始，且节点时间严格递增。"""
        if len(self.waypoints) < 2:
            raise ValueError("reference requires at least two waypoints")
        if self.waypoints[0].t_s != 0:
            raise ValueError("the first waypoint must start at t_s=0")
        if any(
            right.t_s <= left.t_s
            for left, right in zip(self.waypoints, self.waypoints[1:], strict=False)
        ):
            raise ValueError("waypoint times must be strictly increasing")
        return self

    @property
    def duration_s(self) -> float:
        """返回目标曲线总时长。"""
        return float(self.waypoints[-1].t_s)

    def force_at(self, time_s: float) -> float:
        """返回指定跟踪时间处的目标力。"""
        time = float(np.clip(time_s, 0.0, self.duration_s))
        for left, right in zip(self.waypoints, self.waypoints[1:], strict=False):
            if time <= right.t_s:
                if self.interpolation == "hold":
                    return float(left.force_n)
                ratio = (time - left.t_s) / (right.t_s - left.t_s)
                if self.interpolation == "smoothstep":
                    ratio = ratio * ratio * (3.0 - 2.0 * ratio)
                return float(left.force_n + ratio * (right.force_n - left.force_n))
        return float(self.waypoints[-1].force_n)

    def platform_intervals(self) -> tuple[tuple[float, float, float], ...]:
        """返回由相邻等力节点定义的平台区间。"""
        return tuple(
            (float(left.t_s), float(right.t_s), float(left.force_n))
            for left, right in zip(self.waypoints, self.waypoints[1:], strict=False)
            if math.isclose(float(left.force_n), float(right.force_n), abs_tol=1e-12)
        )


class RobotiqDiscreteForceTask(_TaskModel):
    """单次 Robotiq 离散力控制验证任务。"""

    schema_version: Literal[1]
    name: Annotated[str, Field(min_length=1)]
    duration_s: Annotated[FiniteFloat, Field(gt=0)] = 24.0
    control_period_s: Annotated[FiniteFloat, Field(gt=0)] = 1.0 / 30.0
    record_period_s: Annotated[FiniteFloat, Field(gt=0)] = 0.01
    control_delay_s: Annotated[FiniteFloat, Field(ge=0)] = 0.0
    force_noise_std_n: Annotated[FiniteFloat, Field(ge=0)] = 0.0
    noise_seed: Annotated[int, Field(ge=0)] = 0
    object_material: RobotiqObjectMaterial = "medium"
    controller_variant: ControllerVariant = "dynamic-step"
    hold_min_duration_s: Annotated[FiniteFloat, Field(gt=0)] = 0.5
    reference: ForceReference
    control: CommandConfig = CommandConfig()
    force_estimation: ForceEstimationConfig = ForceEstimationConfig()
    delta_f_tick: DeltaFTickConfig = DeltaFTickConfig()
    deadband: DeadbandConfig = DeadbandConfig()
    reactivation: ReactivationConfig = ReactivationConfig()
    prediction: PredictionConfig = PredictionConfig()
    dynamic_step: DynamicStepConfig = DynamicStepConfig()
    safety: SafetyConfig = SafetyConfig()
    quantized_pi: QuantizedPIConfig = QuantizedPIConfig()

    @model_validator(mode="after")
    def validate_timing(self) -> "RobotiqDiscreteForceTask":
        """确保任务足以容纳稳定判定与一次 HOLD 验收。"""
        minimum = self.reference.duration_s + self.force_estimation.min_settle_time_s
        if self.duration_s <= minimum:
            raise ValueError("duration_s must leave time for approach before the reference ends")
        if (
            max(waypoint.force_n for waypoint in self.reference.waypoints)
            >= self.control.max_force_n
        ):
            raise ValueError("all reference forces must lie below max_force_n")
        if self.control.contact_threshold_n >= min(
            waypoint.force_n for waypoint in self.reference.waypoints
        ):
            raise ValueError("contact threshold must lie below every reference force")
        return self

    @classmethod
    def load(cls, path: str | Path) -> "RobotiqDiscreteForceTask":
        """从恰好一个 YAML 映射加载任务。"""
        task_path = Path(path)
        if task_path.suffix.lower() not in {".yaml", ".yml"}:
            raise ValueError("discrete force tasks must use YAML")
        with task_path.open(encoding="utf-8") as stream:
            documents = list(yaml.safe_load_all(stream))
        if len(documents) != 1 or not isinstance(documents[0], dict):
            raise ValueError("discrete force task must contain exactly one YAML mapping")
        return cls.model_validate(documents[0])

    def controller_config(self) -> DiscreteForceControlConfig:
        """转换为不依赖 Pydantic 的领域控制器配置。"""
        return DiscreteForceControlConfig(
            command_min=self.control.command_min,
            command_max=self.control.command_max,
            approach_step=self.control.approach_step,
            approach_action_interval_s=self.control.approach_action_interval_s,
            max_dynamic_step=self.control.max_dynamic_step,
            target_force_n=float(self.reference.waypoints[0].force_n),
            max_force_n=self.control.max_force_n,
            contact_threshold_n=self.control.contact_threshold_n,
            stable_window_samples=self.force_estimation.stable_window_samples,
            stable_force_range_n=self.force_estimation.stable_force_range_n,
            min_settle_time_s=self.force_estimation.min_settle_time_s,
            delta_f_ema_alpha=self.delta_f_tick.ema_alpha,
            delta_f_min_valid_samples=self.delta_f_tick.min_valid_samples,
            delta_f_minimum_n=self.delta_f_tick.minimum_value_n,
            fixed_deadband_n=self.deadband.fixed_n,
            fixed_reactivate_margin_n=self.reactivation.fixed_margin_n,
            noise_sigma_factor=self.deadband.noise_sigma_factor,
            tick_deadband_factor=self.deadband.tick_factor_alpha,
            reactivate_tick_factor=self.reactivation.tick_factor_beta,
            reactivate_duration_s=self.reactivation.duration_s,
            prediction_noise_factor=self.prediction.noise_factor,
            prediction_tick_margin_factor=self.prediction.tick_margin_factor,
            dynamic_step_eta=self.dynamic_step.eta,
            safety_force_margin_n=self.safety.force_margin_n,
        )


@dataclass(frozen=True, slots=True)
class ForcePlatformResult:
    """目标力平台内的误差、动作和 HOLD 指标。"""

    start_time_s: float
    end_time_s: float
    target_force_n: float
    steady_force_error_n: float
    rmse_n: float
    action_count: int
    reverse_count: int
    oscillation_count: int
    hold_ratio: float
    settled: bool


@dataclass(frozen=True, slots=True)
class RobotiqDiscreteForceResult:
    """一次离散力控制运行的结构化指标。"""

    passed: bool
    controller_variant: str
    object_material: str
    reference_min_force_n: float
    reference_max_force_n: float
    reference_duration_s: float
    steady_force_error_n: float
    rmse_n: float
    action_count: int
    total_command_movement: float
    reverse_count: int
    oscillation_count: int
    settling_time_s: float | None
    hold_ratio: float
    peak_force_n: float
    delta_f_tick_estimate_n: float | None
    delta_f_tick_sample_count: int
    prediction_mae_n: float | None
    max_command: float
    command_is_integer: bool
    safety_violated: bool
    safety_violation_duration_s: float
    release_count: int
    average_nonzero_action_step: float
    peak_overshoot_n: float
    q_tick_measured_mean_n: float | None
    rho_p_mean: float | None
    simulation_stable: bool
    platform_count: int
    settled_platform_count: int
    platform_metrics: tuple[ForcePlatformResult, ...]


_TRACE_COLUMNS = (
    "time_s",
    "tracking_time_s",
    "state",
    "u",
    "delta_u",
    "requested_delta_u",
    "u_request",
    "u_actual_command",
    "p_actual",
    "e_position",
    "force_left_n",
    "force_right_n",
    "force_raw_n",
    "force_measured_n",
    "force_filtered_n",
    "force_target_n",
    "force_error_n",
    "force_noise_std_n",
    "force_window_mean_n",
    "force_noise_sigma_n",
    "force_window_range_n",
    "delta_f_tick_raw_n",
    "delta_f_tick_est_n",
    "delta_f_tick_samples",
    "hold_deadband_n",
    "reactivate_threshold_n",
    "prediction_margin_n",
    "predicted_force_next_n",
    "predicted_error_next_n",
    "dynamic_step_raw",
    "dynamic_step_command",
    "selected_action",
    "candidate_cost_m3",
    "candidate_cost_m2",
    "candidate_cost_m1",
    "candidate_cost_hold",
    "candidate_cost_p1",
    "candidate_cost_p2",
    "candidate_cost_p3",
    "model_valid",
    "model_sample_count",
    "is_force_stable",
    "action_count",
    "reverse_count",
    "settled_action_id",
    "settled_delta_u",
    "delta_p",
    "delta_e_position",
    "delta_f",
    "q_tick_measured",
    "rho_p",
    "is_hold",
    "is_adjust",
)


def _write_trace(rows: list[dict[str, object]], output_csv: Path) -> None:
    """以稳定列顺序保存 gzip 压缩的采样轨迹。"""
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(output_csv, "wt", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=_TRACE_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def _equivalent_actuator_position(
    actuator_length: float,
    gain_parameter: float,
    bias_parameter: float,
    command_min: float = 0.0,
    command_max: float = 255.0,
) -> float:
    """把 actuator length 换算为与 0～255 命令同向的等价位置。"""
    gain = float(gain_parameter)
    bias = float(bias_parameter)
    scale = -bias / gain if gain != 0.0 else math.nan
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("actuator position conversion scale must be finite and positive")
    position = float(actuator_length) * scale
    if not math.isfinite(position):
        raise ValueError("equivalent actuator position must be finite")
    return float(np.clip(position, command_min, command_max))


def _should_keep_trace_row(
    *,
    is_first: bool,
    is_last: bool,
    record_tick: bool,
    action_applied: bool,
    settled_action_changed: bool,
    state_changed: bool,
) -> bool:
    """判断当前物理步是否需要持久化为完整 trace 行。"""
    return any(
        (
            is_first,
            is_last,
            record_tick,
            action_applied,
            settled_action_changed,
            state_changed,
        )
    )


def _plot_trace(rows: list[dict[str, object]], output_plot: Path) -> None:
    """生成力、命令、局部增益与自适应阈值四面板图。"""
    plt = science_pyplot()
    time = np.asarray([row["time_s"] for row in rows], dtype=float)
    force = np.asarray([row["force_filtered_n"] for row in rows], dtype=float)
    target = np.asarray([row["force_target_n"] for row in rows], dtype=float)
    command = np.asarray([row["u"] for row in rows], dtype=float)
    delta = np.asarray([row["delta_u"] for row in rows], dtype=float)
    tick = np.asarray(
        [np.nan if row["delta_f_tick_est_n"] == "" else row["delta_f_tick_est_n"] for row in rows],
        dtype=float,
    )
    hold = np.asarray([row["hold_deadband_n"] for row in rows], dtype=float)
    reactivate = np.asarray([row["reactivate_threshold_n"] for row in rows], dtype=float)
    error = np.abs(target - force)
    figure, axes = plt.subplots(4, 1, figsize=paper_figsize(7.1), sharex=True, layout="constrained")
    axes[0].plot(time, target, "--", color="black", label=r"目标 $F_d$")
    axes[0].plot(time, force, color="#0072B2", label=r"滤波力 $F$")
    axes[0].set_ylabel("力 / N")
    axes[0].legend(ncol=2)
    axes[1].step(time, command, where="post", color="#009E73", label=r"命令 $u$")
    action_axis = axes[1].twinx()
    action_axis.step(
        time, delta, where="post", color="#D55E00", alpha=0.75, label=r"动作 $\Delta u$"
    )
    axes[1].set_ylabel(r"$u$ / tick")
    action_axis.set_ylabel(r"$\Delta u$ / tick")
    axes[2].plot(time, tick, color="#CC79A7")
    axes[2].set_ylabel(r"$\widehat{\Delta F}_{tick}$ / N")
    axes[3].plot(time, error, color="#D55E00", label=r"$|e|$")
    axes[3].plot(time, hold, "--", color="#0072B2", label=r"$\epsilon_{hold}$")
    axes[3].plot(time, reactivate, ":", color="#009E73", label=r"$\epsilon_{react}$")
    axes[3].set_ylabel("阈值 / N")
    axes[3].set_xlabel("时间 / s")
    axes[3].legend(ncol=3)
    save_publication_figure(figure, output_plot)
    plt.close(figure)


def _first_sustained_time(times: np.ndarray, mask: np.ndarray, duration_s: float) -> float | None:
    """返回布尔条件首次连续保持指定时长的起点。"""
    start: int | None = None
    for index, active in enumerate(mask):
        if active and start is None:
            start = index
        elif not active:
            start = None
        if start is not None and times[index] - times[start] >= duration_s:
            return float(times[start])
    return None


def _oscillation_count(commands: np.ndarray) -> int:
    """统计相邻三个动作位置中的往返模式。"""
    changed = commands[np.r_[True, np.abs(np.diff(commands)) > 1e-12]]
    if changed.size < 3:
        return 0
    return int(
        np.sum((np.abs(changed[2:] - changed[:-2]) < 1e-12) & (np.abs(np.diff(changed)[:-1]) > 0))
    )


def _platform_results(
    rows: list[dict[str, object]],
    reference: ForceReference,
    hold_min_duration_s: float,
) -> tuple[ForcePlatformResult, ...]:
    """按曲线中的恒力平台计算局部验收指标。"""
    results: list[ForcePlatformResult] = []
    for start, end, target in reference.platform_intervals():
        platform_rows = [
            row
            for row in rows
            if row["tracking_time_s"] != "" and start <= float(row["tracking_time_s"]) <= end
        ]
        if not platform_rows:
            continue
        times = np.asarray([row["tracking_time_s"] for row in platform_rows], dtype=float)
        forces = np.asarray([row["force_filtered_n"] for row in platform_rows], dtype=float)
        states = np.asarray([row["state"] for row in platform_rows], dtype=str)
        deltas = np.asarray([row["delta_u"] for row in platform_rows], dtype=float)
        commands = np.asarray([row["u"] for row in platform_rows], dtype=float)
        steady_mask = times >= max(start, end - hold_min_duration_s)
        steady_forces = forces[steady_mask] if np.any(steady_mask) else forces
        nonzero = deltas[np.abs(deltas) > 1e-12]
        reversals = int(np.sum(nonzero[1:] * nonzero[:-1] < 0)) if nonzero.size >= 2 else 0
        results.append(
            ForcePlatformResult(
                start_time_s=start,
                end_time_s=end,
                target_force_n=target,
                steady_force_error_n=float(np.mean(np.abs(steady_forces - target))),
                rmse_n=float(np.sqrt(np.mean(np.square(forces - target)))),
                action_count=int(nonzero.size),
                reverse_count=reversals,
                oscillation_count=_oscillation_count(commands),
                hold_ratio=float(np.mean(states == "HOLD")),
                settled=_first_sustained_time(times, states == "HOLD", hold_min_duration_s)
                is not None,
            )
        )
    return tuple(results)


def run_robotiq_discrete_force(
    profile_path: str | Path,
    *,
    task: RobotiqDiscreteForceTask,
    controller_variant: ControllerVariant | None = None,
    object_material: RobotiqObjectMaterial | None = None,
    force_noise_std_n: float | None = None,
    noise_seed: int | None = None,
    output_csv: Path,
    output_plot: Path,
) -> RobotiqDiscreteForceResult:
    """运行离散力控制仿真并保存事件增强的独立周期采样轨迹。

    RMSE 与平台统计采用均匀控制周期样本，绘图采用持久化 trace；安全、有限性、
    接触与峰值则在每个物理步在线累计，避免保存完整的 500 Hz 物理轨迹。
    """
    profile = load_profile(profile_path)
    if profile.control_mode != "position":
        raise ValueError("Robotiq discrete force control requires a position-control profile")
    variant = controller_variant or task.controller_variant
    material = object_material or task.object_material
    noise_std = task.force_noise_std_n if force_noise_std_n is None else float(force_noise_std_n)
    seed = task.noise_seed if noise_seed is None else int(noise_seed)
    if noise_std < 0 or seed < 0:
        raise ValueError("noise standard deviation and seed must be non-negative")
    effective_control = task.control
    effective_task = task

    model = load_grasp_model(None, profile.model_path, object_material=material)
    data = mujoco.MjData(model)
    actuator = model.actuator(f"gripper/{profile.actuator}").id
    ctrl_min, ctrl_max = (float(value) for value in model.actuator_ctrlrange[actuator])
    if effective_control.command_min < ctrl_min or effective_control.command_max > ctrl_max:
        raise ValueError("task command range exceeds the actuator ctrlrange")
    timestep = float(model.opt.timestep)
    if float(effective_task.control_period_s) + 1e-12 < timestep:
        raise ValueError("control_period_s must not be smaller than the MuJoCo timestep")
    if float(effective_task.record_period_s) + 1e-12 < timestep:
        raise ValueError("record_period_s must not be smaller than the MuJoCo timestep")
    control_timer = SimulationTimer(float(effective_task.control_period_s), float(data.time))
    record_timer = SimulationTimer(float(effective_task.record_period_s), float(data.time))
    reader = create_tactile_reader(
        model,
        profile.tactile,
        name_resolver=lambda name: f"gripper/{name}",
    )
    rng = np.random.default_rng(seed)
    alpha = 1.0 - math.exp(
        -2.0 * math.pi * effective_task.force_estimation.filter_cutoff_hz * timestep
    )
    command = float(effective_control.command_min)
    requested_command = command
    data.ctrl[actuator] = command
    discrete = None
    if variant != "quantized-pi":
        discrete = DiscreteForceController(
            effective_task.controller_config(),
            variant=variant,
        )
    pending: deque[tuple[float, float]] = deque()
    filtered_force = 0.0
    pi_integral = 0.0
    pi_command_continuous = command
    pi_requested_command = command
    pi_state = "APPROACH"
    pi_last_delta = 0.0
    pi_last_approach_request_s = -math.inf
    pi_action_count = 0
    pi_movement = 0.0
    pi_reverse_count = 0
    target_force = float(effective_task.reference.waypoints[0].force_n)
    tracking_start_time_s: float | None = None
    rows: list[dict[str, object]] = []
    metric_rows: list[dict[str, object]] = []
    steps = math.ceil(float(effective_task.duration_s) / timestep)
    previous_state: str | None = None
    previous_settled_action_id = (
        discrete.snapshot().settled_action_id if discrete is not None else None
    )
    pending_position_baseline: tuple[float, float] | None = None
    peak_force_n = -math.inf
    physics_finite = True
    contact_seen = False
    integer_commands = True
    safety_violation_steps = 0
    release_count = 0
    peak_overshoot_n = 0.0

    for step_index in range(steps):
        time_before = float(data.time)
        applied_delta = 0.0
        if filtered_force > effective_control.max_force_n and pending:
            retained = deque(item for item in pending if item[1] <= 0.0)
            if len(retained) != len(pending):
                pending = retained
                if discrete is not None:
                    discrete.cancel_pending_action()
                    requested_command = command + sum(delta for _, delta in pending)
                else:
                    pi_requested_command = command + sum(delta for _, delta in pending)
                    pi_command_continuous = min(pi_command_continuous, pi_requested_command)
        while pending and pending[0][0] <= time_before + 1e-12:
            _, queued_delta = pending.popleft()
            old_command = command
            position_before = _equivalent_actuator_position(
                data.actuator_length[actuator],
                model.actuator_gainprm[actuator, 0],
                model.actuator_biasprm[actuator, 1],
                effective_control.command_min,
                effective_control.command_max,
            )
            command = float(np.clip(command + queued_delta, ctrl_min, ctrl_max))
            actual_delta = command - old_command
            applied_delta += actual_delta
            if discrete is not None:
                if abs(actual_delta) > 1e-12:
                    pending_position_baseline = (
                        position_before,
                        old_command - position_before,
                    )
                discrete.action_applied(int(round(actual_delta)), time_before)
            elif abs(actual_delta) > 1e-12:
                pi_action_count += 1
                pi_movement += abs(actual_delta)
                if pi_last_delta * actual_delta < 0:
                    pi_reverse_count += 1
                pi_last_delta = actual_delta
        data.ctrl[actuator] = command
        mujoco.mj_step(model, data)
        frame = reader.read(data)
        left_force = max(0.0, float(frame.left[2].sum()))
        right_force = max(0.0, float(frame.right[2].sum()))
        raw_force = 0.5 * (left_force + right_force)
        measured_force = raw_force + float(rng.normal(0.0, noise_std))
        filtered_force += alpha * (measured_force - filtered_force)
        p_actual = _equivalent_actuator_position(
            data.actuator_length[actuator],
            model.actuator_gainprm[actuator, 0],
            model.actuator_biasprm[actuator, 1],
            effective_control.command_min,
            effective_control.command_max,
        )
        actual_command = float(data.ctrl[actuator])
        position_error = actual_command - p_actual
        peak_force_n = max(peak_force_n, filtered_force)
        if filtered_force > effective_control.max_force_n:
            safety_violation_steps += 1
        contact_seen = contact_seen or filtered_force >= effective_control.contact_threshold_n
        physics_finite = physics_finite and all(
            math.isfinite(value)
            for value in (
                command,
                actual_command,
                p_actual,
                raw_force,
                measured_force,
                filtered_force,
            )
        )
        integer_commands = integer_commands and math.isclose(
            actual_command,
            round(actual_command),
            abs_tol=1e-12,
        )
        tracking_time_s = (
            None
            if tracking_start_time_s is None
            else min(float(data.time) - tracking_start_time_s, effective_task.reference.duration_s)
        )
        if tracking_time_s is not None:
            target_force = effective_task.reference.force_at(tracking_time_s)
        control_dt_s = control_timer.pop_due(float(data.time))
        record_due = record_timer.pop_due(float(data.time)) is not None

        requested_delta: float = 0.0
        state = pi_state
        if discrete is not None:
            discrete.observe(filtered_force)
            discrete.set_target_force(target_force)
            if control_dt_s is not None:
                request = discrete.decide(float(data.time))
                requested_delta = float(request)
                if request:
                    pending.append(
                        (float(data.time) + float(effective_task.control_delay_s), float(request))
                    )
                    requested_command += float(request)
            snapshot = discrete.snapshot()
            state = snapshot.state.value
            if tracking_start_time_s is None and state in {"ADJUST", "HOLD"}:
                tracking_start_time_s = float(data.time)
                tracking_time_s = 0.0
            diagnostic = {
                "force_window_mean_n": snapshot.force_mean_n,
                "force_noise_sigma_n": snapshot.force_noise_std_n,
                "force_window_range_n": snapshot.force_range_n,
                "delta_f_tick_raw_n": snapshot.delta_f_tick_raw_n or "",
                "delta_f_tick_est_n": snapshot.delta_f_tick_estimate_n or "",
                "delta_f_tick_samples": snapshot.delta_f_tick_samples,
                "hold_deadband_n": snapshot.hold_deadband_n,
                "reactivate_threshold_n": snapshot.reactivate_threshold_n,
                "prediction_margin_n": snapshot.prediction_margin_n,
                "predicted_force_next_n": snapshot.predicted_force_next_n or "",
                "predicted_error_next_n": snapshot.predicted_error_next_n or "",
                "dynamic_step_raw": snapshot.dynamic_step_raw or "",
                "dynamic_step_command": snapshot.dynamic_step_command,
                "selected_action": snapshot.selected_action,
                "candidate_cost_m3": snapshot.candidate_cost_m3,
                "candidate_cost_m2": snapshot.candidate_cost_m2,
                "candidate_cost_m1": snapshot.candidate_cost_m1,
                "candidate_cost_hold": snapshot.candidate_cost_hold,
                "candidate_cost_p1": snapshot.candidate_cost_p1,
                "candidate_cost_p2": snapshot.candidate_cost_p2,
                "candidate_cost_p3": snapshot.candidate_cost_p3,
                "model_valid": int(snapshot.model_valid),
                "model_sample_count": snapshot.model_sample_count,
                "is_force_stable": int(snapshot.force_stable),
                "action_count": snapshot.action_count,
                "reverse_count": snapshot.reverse_count,
            }
            settled_action_id = snapshot.settled_action_id
            settled_action_changed = (
                settled_action_id is not None and settled_action_id != previous_settled_action_id
            )
            settled_diagnostic: dict[str, object] = {
                "settled_action_id": "" if settled_action_id is None else settled_action_id,
                "settled_delta_u": "",
                "delta_p": "",
                "delta_e_position": "",
                "delta_f": "",
                "q_tick_measured": "",
                "rho_p": "",
            }
            if settled_action_changed:
                settled_delta_u = snapshot.settled_delta_u
                settled_delta_f = snapshot.settled_delta_f_n
                settled_diagnostic["settled_delta_u"] = (
                    "" if settled_delta_u is None else settled_delta_u
                )
                settled_diagnostic["delta_f"] = "" if settled_delta_f is None else settled_delta_f
                if pending_position_baseline is not None:
                    baseline_position, baseline_error = pending_position_baseline
                    delta_p = p_actual - baseline_position
                    settled_diagnostic["delta_p"] = delta_p
                    settled_diagnostic["delta_e_position"] = position_error - baseline_error
                    if settled_delta_u is not None and abs(settled_delta_u) > 1e-12:
                        settled_diagnostic["rho_p"] = abs(delta_p) / abs(settled_delta_u)
                if (
                    settled_delta_u is not None
                    and settled_delta_f is not None
                    and abs(settled_delta_u) > 1e-12
                ):
                    settled_diagnostic["q_tick_measured"] = settled_delta_f / settled_delta_u
                pending_position_baseline = None
                previous_settled_action_id = settled_action_id
        else:
            if control_dt_s is not None:
                if raw_force < effective_control.contact_threshold_n and pi_state == "APPROACH":
                    if (
                        float(data.time) - pi_last_approach_request_s
                        >= effective_control.approach_action_interval_s
                    ):
                        pi_command_continuous = float(
                            np.clip(
                                pi_requested_command + effective_control.approach_step,
                                ctrl_min,
                                ctrl_max,
                            )
                        )
                        desired_command = float(round(pi_command_continuous))
                        requested_delta = desired_command - pi_requested_command
                        pi_last_approach_request_s = float(data.time)
                else:
                    if tracking_start_time_s is None:
                        tracking_start_time_s = float(data.time)
                        tracking_time_s = 0.0
                        target_force = effective_task.reference.force_at(0.0)
                    pi_state = "ADJUST"
                    error = target_force - filtered_force
                    if abs(error) <= effective_task.deadband.fixed_n:
                        pi_state = "HOLD"
                        continuous_delta = 0.0
                    else:
                        pi_integral += error * control_dt_s
                        continuous_delta = float(
                            np.clip(
                                effective_task.quantized_pi.kp_tick_per_n * error
                                + effective_task.quantized_pi.ki_tick_per_n_s * pi_integral,
                                -effective_task.quantized_pi.max_step_per_update,
                                effective_task.quantized_pi.max_step_per_update,
                            )
                        )
                    if filtered_force > effective_control.max_force_n:
                        continuous_delta = min(continuous_delta, -1.0)
                    pi_command_continuous = float(
                        np.clip(pi_command_continuous + continuous_delta, ctrl_min, ctrl_max)
                    )
                    desired_command = float(round(pi_command_continuous))
                    requested_delta = desired_command - pi_requested_command
                if abs(requested_delta) > 1e-12:
                    pi_requested_command += requested_delta
                    pending.append(
                        (float(data.time) + float(effective_task.control_delay_s), requested_delta)
                    )
            state = pi_state
            diagnostic = {
                "force_window_mean_n": filtered_force,
                "force_noise_sigma_n": noise_std,
                "force_window_range_n": 0.0,
                "delta_f_tick_raw_n": "",
                "delta_f_tick_est_n": "",
                "delta_f_tick_samples": 0,
                "hold_deadband_n": effective_task.deadband.fixed_n,
                "reactivate_threshold_n": effective_task.deadband.fixed_n
                + effective_task.reactivation.fixed_margin_n,
                "prediction_margin_n": 0.0,
                "predicted_force_next_n": "",
                "predicted_error_next_n": "",
                "dynamic_step_raw": "",
                "dynamic_step_command": requested_delta,
                "selected_action": int(requested_delta),
                "candidate_cost_m3": "",
                "candidate_cost_m2": "",
                "candidate_cost_m1": "",
                "candidate_cost_hold": "",
                "candidate_cost_p1": "",
                "candidate_cost_p2": "",
                "candidate_cost_p3": "",
                "model_valid": 0,
                "model_sample_count": 0,
                "is_force_stable": 0,
                "action_count": pi_action_count,
                "reverse_count": pi_reverse_count,
            }
            requested_command = pi_requested_command
            settled_action_changed = False
            settled_diagnostic = {
                "settled_action_id": "",
                "settled_delta_u": "",
                "delta_p": "",
                "delta_e_position": "",
                "delta_f": "",
                "q_tick_measured": "",
                "rho_p": "",
            }
        state_changed = previous_state is not None and state != previous_state
        if state == "RELEASE" and previous_state != "RELEASE":
            release_count += 1
        previous_state = state
        if tracking_time_s is not None:
            peak_overshoot_n = max(peak_overshoot_n, filtered_force - target_force)
        # trace 与控制器使用独立时钟；关键事件与首尾物理步额外保留。
        keep_trace = _should_keep_trace_row(
            is_first=step_index == 0,
            is_last=step_index == steps - 1,
            record_tick=record_due,
            action_applied=abs(applied_delta) > 1e-12,
            settled_action_changed=settled_action_changed,
            state_changed=state_changed,
        )
        if keep_trace or control_dt_s is not None:
            row = {
                "time_s": float(data.time),
                "tracking_time_s": "" if tracking_time_s is None else tracking_time_s,
                "state": state,
                "u": command,
                "delta_u": applied_delta,
                "requested_delta_u": requested_delta,
                "u_request": requested_command,
                "u_actual_command": actual_command,
                "p_actual": p_actual,
                "e_position": position_error,
                "force_left_n": left_force,
                "force_right_n": right_force,
                "force_raw_n": raw_force,
                "force_measured_n": measured_force,
                "force_filtered_n": filtered_force,
                "force_target_n": target_force,
                "force_error_n": target_force - filtered_force,
                "force_noise_std_n": noise_std,
                **diagnostic,
                **settled_diagnostic,
                "is_hold": int(state == "HOLD"),
                "is_adjust": int(state == "ADJUST"),
            }
            if keep_trace:
                rows.append(row)
            if control_dt_s is not None:
                metric_rows.append(row)

    _write_trace(rows, output_csv)
    _plot_trace(rows, output_plot)
    sampled_rows = metric_rows or rows
    forces = np.asarray([row["force_filtered_n"] for row in sampled_rows], dtype=float)
    commands = np.asarray([row["u"] for row in rows], dtype=float)
    states = np.asarray([row["state"] for row in sampled_rows], dtype=str)
    tracking = np.asarray([row["tracking_time_s"] != "" for row in sampled_rows], dtype=bool)
    evaluation = tracking if np.any(tracking) else np.ones_like(tracking)
    targets = np.asarray([row["force_target_n"] for row in sampled_rows], dtype=float)
    errors = forces[evaluation] - targets[evaluation]
    tracking_times = np.asarray(
        [float(row["tracking_time_s"]) for row in sampled_rows if row["tracking_time_s"] != ""],
        dtype=float,
    )
    tracking_states = states[tracking]
    settling = (
        _first_sustained_time(
            tracking_times,
            tracking_states == "HOLD",
            float(effective_task.hold_min_duration_s),
        )
        if tracking_times.size
        else None
    )
    platforms = _platform_results(
        sampled_rows,
        effective_task.reference,
        float(effective_task.hold_min_duration_s),
    )
    settled_platforms = sum(platform.settled for platform in platforms)
    if discrete is not None:
        final_snapshot = discrete.snapshot()
        action_count = discrete.action_count
        movement = float(discrete.total_command_movement)
        reverse_count = discrete.reverse_count
        delta_estimate = final_snapshot.delta_f_tick_estimate_n
        delta_samples = final_snapshot.delta_f_tick_samples
        prediction_mae = (
            float(np.mean(discrete.prediction_errors_n)) if discrete.prediction_errors_n else None
        )
    else:
        action_count = pi_action_count
        movement = pi_movement
        reverse_count = pi_reverse_count
        delta_estimate = None
        delta_samples = 0
        prediction_mae = None
    finite = bool(physics_finite and np.all(np.isfinite(forces)) and np.all(np.isfinite(commands)))
    safety_violated = bool(peak_force_n > effective_control.max_force_n)
    expected_platforms = len(effective_task.reference.platform_intervals())
    passed = bool(
        finite
        and not safety_violated
        and contact_seen
        and len(platforms) == expected_platforms
        and settled_platforms == expected_platforms
        and integer_commands
    )
    final_reference = float(effective_task.reference.waypoints[-1].force_n)
    final_mask = (
        tracking
        & (targets == final_reference)
        & (
            np.asarray(
                [
                    row["tracking_time_s"] if row["tracking_time_s"] != "" else -1
                    for row in sampled_rows
                ],
                dtype=float,
            )
            >= effective_task.reference.duration_s - effective_task.hold_min_duration_s
        )
    )
    final_forces = forces[final_mask] if np.any(final_mask) else forces[evaluation][-1:]
    measured_tick_values = [
        float(row["q_tick_measured"]) for row in rows if row["q_tick_measured"] != ""
    ]
    position_response_values = [float(row["rho_p"]) for row in rows if row["rho_p"] != ""]
    return RobotiqDiscreteForceResult(
        passed=passed,
        controller_variant=variant,
        object_material=material,
        reference_min_force_n=float(
            min(waypoint.force_n for waypoint in effective_task.reference.waypoints)
        ),
        reference_max_force_n=float(
            max(waypoint.force_n for waypoint in effective_task.reference.waypoints)
        ),
        reference_duration_s=effective_task.reference.duration_s,
        steady_force_error_n=float(np.mean(np.abs(final_forces - final_reference))),
        rmse_n=float(np.sqrt(np.mean(np.square(errors)))),
        action_count=action_count,
        total_command_movement=movement,
        reverse_count=reverse_count,
        oscillation_count=_oscillation_count(commands),
        settling_time_s=settling,
        hold_ratio=float(np.mean(tracking_states == "HOLD")) if tracking_states.size else 0.0,
        peak_force_n=float(peak_force_n),
        delta_f_tick_estimate_n=delta_estimate,
        delta_f_tick_sample_count=delta_samples,
        prediction_mae_n=prediction_mae,
        max_command=float(np.max(commands)),
        command_is_integer=integer_commands,
        safety_violated=safety_violated,
        safety_violation_duration_s=safety_violation_steps * timestep,
        release_count=release_count,
        average_nonzero_action_step=movement / action_count if action_count else 0.0,
        peak_overshoot_n=float(peak_overshoot_n),
        q_tick_measured_mean_n=(
            float(np.mean(measured_tick_values)) if measured_tick_values else None
        ),
        rho_p_mean=(float(np.mean(position_response_values)) if position_response_values else None),
        simulation_stable=finite,
        platform_count=len(platforms),
        settled_platform_count=settled_platforms,
        platform_metrics=platforms,
    )


__all__ = [
    "ControllerVariant",
    "RobotiqDiscreteForceResult",
    "RobotiqDiscreteForceTask",
    "run_robotiq_discrete_force",
]

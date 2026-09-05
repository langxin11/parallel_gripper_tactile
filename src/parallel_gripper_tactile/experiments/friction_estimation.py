"""用触觉力域初始滑移代理估计保守摩擦系数。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Annotated, Literal

import mujoco
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, ValidationError, model_validator
import yaml

from ..control import ForceControlObservation, ForceControlReference, NormalForceController
from ..force_scheduling import OracleTargetForceScheduler
from ..friction_estimation import (
    ConservativeFrictionEstimator,
    FrictionEstimate,
    FrictionEstimatorConfig,
    FrictionProbeObservation,
)
from ..taxel_friction import (
    TaxelFrictionConfig,
    TaxelFrictionObservation,
    TaxelFrictionObserver,
)
from ..plotstyle import (
    FULL_WIDTH_FONT_SCALE,
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)

from ..profiles import load_profile
from ..scenes.custom import (
    CUBE_PREFIX,
    DEFAULT_PROFILE,
    GRIPPER_PREFIX,
    ObjectMaterial,
    SUPPORT_GEOM_NAME,
    build_custom_grasp_model,
)
from ..timing import SimulationTimer
from .force_scheduling import (
    DownwardLoadReference,
    ForceSchedulingApproach,
    ForceSchedulingSolverConfig,
    OracleSchedulerTaskConfig,
)
from .grasp import (
    CUBE_BODY_NAME,
    CUBE_JOINT_NAME,
    _friction_capacity,
    _prefixed_reader,
    _tactile_measurement,
    _taxel_geom_sides,
)


class FrictionEstimationConfigError(ValueError):
    """摩擦估计任务配置无法加载或未通过校验。"""


class _TaskModel(BaseModel):
    """拒绝未知字段并冻结运行配置的任务模型。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class TangentialProbeTaskConfig(_TaskModel):
    """恒定法向力下的世界 Y 向切向探测参数。"""

    normal_force_n: Annotated[FiniteFloat, Field(gt=0)] = 1.0
    settle_after_release_s: Annotated[FiniteFloat, Field(ge=0)] = 0.3
    force_rate_n_s: Annotated[FiniteFloat, Field(gt=0)] = 1.0
    max_force_n: Annotated[FiniteFloat, Field(gt=0)] = 3.0
    hold_at_max_s: Annotated[FiniteFloat, Field(ge=0)] = 0.2
    recovery_s: Annotated[FiniteFloat, Field(ge=0)] = 0.4
    recovery_normal_force_n: Annotated[FiniteFloat, Field(gt=0)] = 2.0

    @property
    def maximum_duration_s(self) -> float:
        """返回探测斜坡和最大载荷保持段的最长持续时间。"""
        return float(self.max_force_n / self.force_rate_n_s + self.hold_at_max_s)

    def force_at(self, elapsed_s: float) -> float:
        """返回探测阶段指定时刻的世界 Y 向载荷。"""
        return min(float(self.max_force_n), max(0.0, elapsed_s) * float(self.force_rate_n_s))


class FrictionEstimatorTaskConfig(_TaskModel):
    """力域初始滑移探测和保守估计参数。"""

    window_size: Annotated[int, Field(gt=0)] = 60
    min_samples: Annotated[int, Field(gt=0)] = 20
    estimate_quantile: Annotated[FiniteFloat, Field(gt=0, le=1)] = 0.85
    safety_discount: Annotated[FiniteFloat, Field(gt=0, le=1)] = 0.9
    fallback_friction_coefficient: Annotated[FiniteFloat, Field(gt=0)] = 0.2
    min_friction_coefficient: Annotated[FiniteFloat, Field(gt=0)] = 0.05
    max_friction_coefficient: Annotated[FiniteFloat, Field(gt=0)] = 2.0
    min_probe_load_n: Annotated[FiniteFloat, Field(gt=0)] = 0.25
    min_total_normal_force_n: Annotated[FiniteFloat, Field(gt=0)] = 0.2
    support_residual_threshold_n: Annotated[FiniteFloat, Field(ge=0)] = 0.04
    support_utilization_threshold: Annotated[FiniteFloat, Field(ge=0, lt=1)] = 0.97
    mismatch_confirm_s: Annotated[FiniteFloat, Field(gt=0)] = 0.012
    ratio_trend_enabled: bool = True
    trend_window_size: Annotated[int, Field(ge=2)] = 60
    arming_slope_threshold_per_s: Annotated[FiniteFloat, Field(gt=0)] = 0.09
    saturation_slope_threshold_per_s: Annotated[FiniteFloat, Field(ge=0)] = 0.055
    max_side_ratio_difference: Annotated[FiniteFloat, Field(ge=0)] = 0.1

    @model_validator(mode="after")
    def validate_friction_limits(self) -> "FrictionEstimatorTaskConfig":
        """要求估计下限、回退值和上限顺序一致。"""
        if self.min_samples > self.window_size:
            raise ValueError("min_samples must not exceed window_size")
        if self.saturation_slope_threshold_per_s >= self.arming_slope_threshold_per_s:
            raise ValueError("saturation slope threshold must be smaller than arming threshold")
        if self.max_friction_coefficient < self.min_friction_coefficient:
            raise ValueError(
                "max_friction_coefficient must not be smaller than min_friction_coefficient"
            )
        if not (
            self.min_friction_coefficient
            <= self.fallback_friction_coefficient
            <= self.max_friction_coefficient
        ):
            raise ValueError("fallback_friction_coefficient must be within friction limits")
        return self

    def to_runtime_config(self) -> FrictionEstimatorConfig:
        """转换为与仿真无关的纯估计器配置。"""
        return FrictionEstimatorConfig(**self.model_dump())


class TaxelFrictionTaskConfig(_TaskModel):
    """逐 taxel 接触筛选与局部摩擦利用率参数。"""

    contact_enter_force_n: Annotated[FiniteFloat, Field(gt=0)] = 0.05
    contact_exit_force_n: Annotated[FiniteFloat, Field(ge=0)] = 0.025
    transition_confirm_s: Annotated[FiniteFloat, Field(gt=0)] = 0.01

    @model_validator(mode="after")
    def validate_hysteresis(self) -> "TaxelFrictionTaskConfig":
        """要求退出阈值严格低于进入阈值。"""
        if self.contact_exit_force_n >= self.contact_enter_force_n:
            raise ValueError("contact_exit_force_n must be smaller than contact_enter_force_n")
        return self

    def to_runtime_config(self) -> TaxelFrictionConfig:
        """转换为与仿真无关的逐 taxel 观测配置。"""
        return TaxelFrictionConfig(**self.model_dump())


class FrictionEstimationMetricsConfig(_TaskModel):
    """探测、估计和后续保持阶段的验收门限。"""

    require_detection: bool = True
    conservative_tolerance: Annotated[FiniteFloat, Field(ge=0)] = 0.02
    minimum_estimate_ratio: Annotated[FiniteFloat, Field(ge=0, le=1)] = 0.55
    probe_slip_threshold_m: Annotated[FiniteFloat, Field(gt=0)] = 0.001
    hold_slip_threshold_m: Annotated[FiniteFloat, Field(gt=0)] = 0.002
    force_rmse_threshold_n: Annotated[FiniteFloat, Field(gt=0)] = 0.5
    ignore_hold_initial_s: Annotated[FiniteFloat, Field(ge=0)] = 0.2


class FrictionEstimationTask(_TaskModel):
    """一次探测、估计并用估计值调度目标力的仿真任务。"""

    schema_version: Literal[1]
    name: Annotated[str, Field(min_length=1)]
    cube_mass_kg: Annotated[FiniteFloat, Field(ge=0.05)] = 0.05
    friction_coefficient: Annotated[FiniteFloat, Field(gt=0)] = 0.8
    object_material: ObjectMaterial = "hard"
    sensor_noise_scale: Annotated[FiniteFloat, Field(ge=0)] = 1.0
    approach: ForceSchedulingApproach = ForceSchedulingApproach()
    probe: TangentialProbeTaskConfig = TangentialProbeTaskConfig()
    estimator: FrictionEstimatorTaskConfig = FrictionEstimatorTaskConfig()
    taxel_observer: TaxelFrictionTaskConfig = TaxelFrictionTaskConfig()
    scheduler: OracleSchedulerTaskConfig = OracleSchedulerTaskConfig()
    downward_load: DownwardLoadReference
    metrics: FrictionEstimationMetricsConfig = FrictionEstimationMetricsConfig()
    solver: ForceSchedulingSolverConfig = ForceSchedulingSolverConfig()
    control_period_s: Annotated[FiniteFloat, Field(gt=0)] = 0.002

    @model_validator(mode="after")
    def validate_probe_force_limit(self) -> "FrictionEstimationTask":
        """要求探测法向力位于后续调度器允许范围内。"""
        for field_name, force_n in (
            ("normal_force_n", self.probe.normal_force_n),
            ("recovery_normal_force_n", self.probe.recovery_normal_force_n),
        ):
            if not self.scheduler.min_force_n <= force_n <= self.scheduler.max_force_n:
                raise ValueError(f"probe {field_name} must be within scheduler force limits")
        return self

    @classmethod
    def load(cls, path: str | Path) -> "FrictionEstimationTask":
        """从 YAML 文件加载并校验摩擦估计任务。"""
        task_path = Path(path)
        try:
            with task_path.open(encoding="utf-8") as stream:
                raw = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise FrictionEstimationConfigError(f"invalid YAML in {task_path}") from error
        if raw is None:
            raise FrictionEstimationConfigError(f"friction estimation task is empty: {task_path}")
        if not isinstance(raw, dict):
            raise FrictionEstimationConfigError(
                f"friction estimation task root must be a mapping: {task_path}"
            )
        try:
            return cls.model_validate(raw)
        except ValidationError as error:
            raise FrictionEstimationConfigError(str(error)) from error


@dataclass(frozen=True, slots=True)
class FrictionEstimationResult:
    """一次盲估计和估计值驱动抓取的主要评价指标。"""

    true_friction_coefficient: float
    estimated_friction_coefficient: float
    raw_friction_coefficient: float | None
    estimate_ratio: float
    absolute_estimation_error: float
    slip_detected: bool
    using_fallback: bool
    probe_detection_time_s: float | None
    probe_force_at_detection_n: float | None
    max_probe_displacement_m: float
    max_hold_displacement_m: float
    hold_force_tracking_rmse_n: float
    minimum_hold_friction_margin_n: float
    peak_active_taxel_count: int
    peak_local_friction_ratio: float
    local_weighted_ratio_at_detection: float | None
    local_ratio_p90_at_detection: float | None
    simulation_stable: bool
    detection_passed: bool
    conservatism_passed: bool
    informativeness_passed: bool
    probe_slip_passed: bool
    hold_slip_passed: bool
    force_tracking_passed: bool

    @property
    def passed(self) -> bool:
        """是否通过探测、估计保守性和后续抓取全部验收。"""
        return (
            self.simulation_stable
            and self.detection_passed
            and self.conservatism_passed
            and self.informativeness_passed
            and self.probe_slip_passed
            and self.hold_slip_passed
            and self.force_tracking_passed
        )


def _tangential_displacement(position: np.ndarray, reference: np.ndarray) -> float:
    """返回物体在指尖 YZ 接触平面内的位移。"""
    return float(np.linalg.norm(position[1:3] - reference[1:3]))


def _measurement_components(measurement) -> tuple[float, float, float, float]:
    """把左右局部三轴触觉合力化为各侧法向力和剪切模。"""
    left = measurement.left_force
    right = measurement.right_force
    return (
        max(0.0, float(left[2])),
        float(np.linalg.norm(left[:2])),
        max(0.0, float(right[2])),
        float(np.linalg.norm(right[:2])),
    )


def _taxel_trace_fields(observation: TaxelFrictionObservation) -> dict[str, float | int | bool]:
    """把逐 taxel 观测展开为稳定的 CSV 标量列。"""
    fields: dict[str, float | int | bool] = {
        "active_taxel_count": observation.active_count,
        "active_taxel_weighted_ratio": observation.weighted_ratio,
        "active_taxel_ratio_p90": observation.ratio_p90,
        "active_taxel_max_ratio": observation.maximum_ratio,
        "active_taxel_ratio_spread": observation.ratio_spread,
    }
    for side, mask, ratios in (
        ("left", observation.left_contact_mask, observation.left_ratio),
        ("right", observation.right_contact_mask, observation.right_ratio),
    ):
        for row, col in np.ndindex(mask.shape):
            fields[f"{side}_taxel_contact_{row}_{col}"] = bool(mask[row, col])
            fields[f"{side}_taxel_ratio_{row}_{col}"] = float(ratios[row, col])
    return fields


def _plot_friction_estimation(
    path: Path,
    rows: list[dict[str, float | str | bool]],
    *,
    task: FrictionEstimationTask,
) -> None:
    """绘制探测残差、摩擦估计、目标力和滑移诊断图。"""
    if not rows:
        raise ValueError("cannot plot an empty friction estimation trace")
    plt = science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)
    times = np.asarray([float(row["time_s"]) for row in rows])
    phases = [str(row["phase"]) for row in rows]
    probe_rows = np.asarray([phase in {"probe_settle", "probe", "recovery"} for phase in phases])
    hold_rows = np.asarray([phase == "schedule_load" for phase in phases])
    figure, axes = plt.subplots(
        4, 1, figsize=paper_figsize(8.0), sharex=True, constrained_layout=True
    )

    axes[0].plot(times, [float(row["tangential_demand_n"]) for row in rows], label="Demand")
    axes[0].plot(times, [float(row["measured_shear_support_n"]) for row in rows], label="Tactile")
    axes[0].set_ylabel("Tangential\nforce (N)")
    axes[0].legend()

    axes[1].plot(times, [float(row["support_residual_n"]) for row in rows])
    axes[1].axhline(
        float(task.estimator.support_residual_threshold_n),
        color="tab:red",
        linestyle="--",
    )
    axes[1].set_ylabel("Support\nresidual (N)")

    estimate = np.asarray([float(row["estimated_friction_coefficient"]) for row in rows])
    axes[2].plot(times, estimate, label="Conservative estimate")
    axes[2].axhline(
        float(task.friction_coefficient), color="0.35", linestyle="--", label="True (score only)"
    )
    axes[2].set_ylabel("Friction\ncoefficient")
    axes[2].legend()

    probe_displacement = 1000.0 * np.asarray([float(row["release_displacement_m"]) for row in rows])
    hold_displacement = 1000.0 * np.asarray([float(row["hold_displacement_m"]) for row in rows])
    axes[3].plot(times[probe_rows], probe_displacement[probe_rows], label="Probe")
    axes[3].plot(times[hold_rows], hold_displacement[hold_rows], label="Scheduled hold")
    axes[3].axhline(
        1000.0 * float(task.metrics.probe_slip_threshold_m),
        color="tab:red",
        linestyle="--",
        label="Probe limit",
    )
    axes[3].axhline(
        1000.0 * float(task.metrics.hold_slip_threshold_m),
        color="tab:orange",
        linestyle=":",
        label="Hold limit",
    )
    axes[3].set_ylabel("Displacement\n(mm)")
    axes[3].set_xlabel("Simulation time (s)")
    axes[3].legend()
    path.parent.mkdir(parents=True, exist_ok=True)
    save_publication_figure(figure, path)
    plt.close(figure)


def _plot_taxel_friction(
    path: Path,
    rows: list[dict[str, float | str | bool]],
    *,
    task: FrictionEstimationTask,
) -> None:
    """绘制局部摩擦利用率时间序列和双侧 taxel 峰值热图。"""
    if not rows:
        raise ValueError("cannot plot an empty taxel friction trace")
    plt = science_pyplot()
    times = np.asarray([float(row["time_s"]) for row in rows])
    probe_mask = np.asarray([str(row["phase"]) == "probe" for row in rows])
    left_keys = sorted(key for key in rows[0] if key.startswith("left_taxel_ratio_"))
    right_keys = sorted(key for key in rows[0] if key.startswith("right_taxel_ratio_"))
    if not left_keys or len(left_keys) != len(right_keys):
        raise ValueError("taxel friction trace does not contain balanced grids")
    coordinates = [tuple(int(value) for value in key.rsplit("_", 2)[-2:]) for key in left_keys]
    grid_shape = (
        max(row for row, _ in coordinates) + 1,
        max(col for _, col in coordinates) + 1,
    )
    if math.prod(grid_shape) != len(left_keys):
        raise ValueError("taxel friction trace requires a complete rectangular grid")

    def peak_grid(keys: list[str]) -> np.ndarray:
        """返回探测阶段逐点的未平滑峰值。"""
        values = np.asarray([[float(row[key]) for key in keys] for row in rows])
        selected = values[probe_mask]
        finite = np.isfinite(selected)
        peak = np.full(selected.shape[1], np.nan)
        for index in range(selected.shape[1]):
            if finite[:, index].any():
                peak[index] = float(np.max(selected[finite[:, index], index]))
        grid = np.full(grid_shape, np.nan)
        for index, coordinate in enumerate(coordinates):
            grid[coordinate] = peak[index]
        return grid

    figure, axes = plt.subplots(2, 2, figsize=paper_figsize(6.0), constrained_layout=True)
    aggregate_ratio = np.asarray(
        [
            (float(row["measured_left_shear_n"]) + float(row["measured_right_shear_n"]))
            / max(
                float(row["measured_left_normal_n"]) + float(row["measured_right_normal_n"]),
                1e-12,
            )
            for row in rows
        ]
    )
    axes[0, 0].plot(times[probe_mask], aggregate_ratio[probe_mask], label="All-taxel resultant")
    axes[0, 0].plot(
        times[probe_mask],
        np.asarray([float(row["active_taxel_weighted_ratio"]) for row in rows])[probe_mask],
        linestyle="--",
        label="Contact-filtered",
    )
    axes[0, 0].set_ylabel("Friction ratio")
    axes[0, 0].set_xlabel("Simulation time (s)")
    axes[0, 0].legend()

    axes[1, 0].plot(
        times[probe_mask],
        np.asarray([float(row["active_taxel_ratio_p90"]) for row in rows])[probe_mask],
        label="Local ratio P90",
    )
    axes[1, 0].plot(
        times[probe_mask],
        np.asarray([float(row["active_taxel_ratio_spread"]) for row in rows])[probe_mask],
        linestyle="--",
        label="Local ratio spread",
    )
    axes[1, 0].set_ylabel("Local diagnostic")
    axes[1, 0].set_xlabel("Simulation time (s)")
    axes[1, 0].legend()

    left_peak = peak_grid(left_keys)
    right_peak = peak_grid(right_keys)
    finite_peaks = np.concatenate(
        (left_peak[np.isfinite(left_peak)], right_peak[np.isfinite(right_peak)])
    )
    color_limit = max(
        1.0,
        1.25 * float(task.friction_coefficient),
        float(np.max(finite_peaks)) if finite_peaks.size else 1.0,
    )
    images = []
    for axis, title, grid in (
        (axes[0, 1], "Left taxel peak ratio", left_peak),
        (axes[1, 1], "Right taxel peak ratio", right_peak),
    ):
        image = axis.imshow(grid, vmin=0.0, vmax=color_limit, cmap="viridis")
        images.append(image)
        axis.set_title(title)
        axis.set_xlabel("Column")
        axis.set_ylabel("Row")
        axis.set_xticks(range(grid_shape[1]))
        axis.set_yticks(range(grid_shape[0]))
        for row, col in np.ndindex(grid.shape):
            label = "--" if not math.isfinite(grid[row, col]) else f"{grid[row, col]:.2f}"
            axis.text(col, row, label, ha="center", va="center", color="black", fontsize=6)
    figure.colorbar(images[0], ax=[axes[0, 1], axes[1, 1]], label="Peak local friction ratio")
    path.parent.mkdir(parents=True, exist_ok=True)
    save_publication_figure(figure, path)
    plt.close(figure)


def run_friction_estimation(
    profile_path: Path = DEFAULT_PROFILE,
    *,
    task: FrictionEstimationTask,
    output_csv: Path | None = None,
    output_plot: Path | None = None,
    output_taxel_plot: Path | None = None,
) -> FrictionEstimationResult:
    """运行探测、保守估计与估计值目标力调度仿真。"""
    profile = load_profile(profile_path)
    if profile.normal_force is None:
        raise ValueError("friction estimation requires profile control.force")
    model = build_custom_grasp_model(
        profile,
        cube_mass=float(task.cube_mass_kg),
        object_material=task.object_material,
        friction_coefficient=float(task.friction_coefficient),
    )
    model.opt.noslip_iterations = int(task.solver.noslip_iterations)
    if task.control_period_s + 1e-12 < float(model.opt.timestep):
        raise ValueError("control_period_s must not be smaller than the physics timestep")

    data = mujoco.MjData(model)
    control_timer = SimulationTimer(float(task.control_period_s), float(data.time))
    reader = _prefixed_reader(model, profile)
    controller = NormalForceController.from_profile(model, profile, name_prefix=GRIPPER_PREFIX)
    estimator = ConservativeFrictionEstimator(task.estimator.to_runtime_config())
    taxel_observer = TaxelFrictionObserver(task.taxel_observer.to_runtime_config())
    scheduler = OracleTargetForceScheduler(task.scheduler.to_runtime_config())
    noise_rng = np.random.default_rng(int(profile.normal_force.sensor_noise_seed))
    support_id = model.geom(SUPPORT_GEOM_NAME).id
    cube_body_id = model.body(CUBE_BODY_NAME).id
    cube_joint_id = model.joint(CUBE_JOINT_NAME).id
    cube_geom_id = model.geom(f"{CUBE_PREFIX}target_cube_geom").id
    cube_dof = int(model.jnt_dofadr[cube_joint_id])
    taxel_geom_sides = _taxel_geom_sides(model, profile)
    cube_mass = float(model.body_mass[cube_body_id])
    gravity_tangent = cube_mass * np.asarray(model.opt.gravity[1:3], dtype=np.float64)
    gravity_force_n = float(np.linalg.norm(gravity_tangent))
    gravity_direction = gravity_tangent / max(gravity_force_n, 1e-12)

    phase = "approach_contact"
    phase_start_time_s = float(data.time)
    contact_time_s: float | None = None
    release_position: np.ndarray | None = None
    hold_reference_position: np.ndarray | None = None
    detection_time_s: float | None = None
    probe_force_at_detection_n: float | None = None
    estimate: FrictionEstimate | None = None
    force_command = None
    schedule_command = None
    latest_measurement = None
    latest_estimator_state: FrictionEstimate | None = None
    latest_taxel_observation: TaxelFrictionObservation | None = None
    simulation_stable = True
    rows: list[dict[str, float | str | bool]] = []
    maximum_duration_s = (
        float(task.approach.timeout_s)
        + float(task.approach.settle_after_contact_s)
        + float(task.probe.settle_after_release_s)
        + float(task.probe.maximum_duration_s)
        + float(task.probe.recovery_s)
        + float(task.downward_load.duration_s)
        + 1.0
    )
    steps = math.ceil(maximum_duration_s / float(model.opt.timestep))
    for _ in range(steps):
        time_s = float(data.time)
        phase_elapsed_s = time_s - phase_start_time_s
        if phase == "probe_settle" and phase_elapsed_s >= float(task.probe.settle_after_release_s):
            phase = "probe"
            phase_start_time_s = time_s
            phase_elapsed_s = 0.0
            estimator.reset()
            taxel_observer.reset()
        elif phase == "probe" and phase_elapsed_s >= float(task.probe.maximum_duration_s):
            estimate = estimator.finalize()
            latest_estimator_state = estimate
            phase = "recovery"
            phase_start_time_s = time_s
            phase_elapsed_s = 0.0
        elif phase == "recovery" and phase_elapsed_s >= float(task.probe.recovery_s):
            if estimate is None:
                raise RuntimeError("recovery completed without a friction estimate")
            phase = "schedule_load"
            phase_start_time_s = time_s
            phase_elapsed_s = 0.0
            hold_reference_position = data.xpos[cube_body_id].copy()
            scheduler.reset(float(task.probe.recovery_normal_force_n))
        elif phase == "schedule_load" and phase_elapsed_s > task.downward_load.duration_s:
            break

        step_phase = phase
        probe_force_n = task.probe.force_at(phase_elapsed_s) if phase == "probe" else 0.0
        additional_downward_force_n = 0.0
        additional_downward_rate_n_s = 0.0
        applied_tangent = np.zeros(2, dtype=np.float64)
        if phase == "probe":
            applied_tangent[0] = probe_force_n
        elif phase == "schedule_load":
            additional_downward_force_n, additional_downward_rate_n_s = (
                task.downward_load.sample_at(phase_elapsed_s)
            )
            applied_tangent = gravity_direction * additional_downward_force_n
        demand_vector = gravity_tangent + applied_tangent
        tangential_demand_n = float(np.linalg.norm(demand_vector))
        data.xfrc_applied[cube_body_id] = 0.0
        if phase in {"probe", "schedule_load"}:
            data.xfrc_applied[cube_body_id, 1:3] = applied_tangent

        target_position = profile.open_control + min(
            1.0, time_s / float(task.approach.duration_s)
        ) * (profile.closed_control - profile.open_control)
        pending_recovery = False
        control_dt = control_timer.pop_due(time_s)
        if control_dt is not None:
            tactile = reader.read(data)
            noise_scale = float(task.sensor_noise_scale)
            latest_measurement = _tactile_measurement(
                tactile,
                left_normal_std_n=noise_scale
                * float(profile.normal_force.sensor_taxel_normal_noise_std_n[0]),
                right_normal_std_n=noise_scale
                * float(profile.normal_force.sensor_taxel_normal_noise_std_n[1]),
                left_shear_std_n=noise_scale
                * float(profile.normal_force.sensor_taxel_shear_noise_std_n[0]),
                right_shear_std_n=noise_scale
                * float(profile.normal_force.sensor_taxel_shear_noise_std_n[1]),
                rng=noise_rng,
            )
            latest_taxel_observation = taxel_observer.update(
                latest_measurement.left,
                latest_measurement.right,
                dt=control_dt,
            )
            measured_capacity = latest_measurement.normal_capacity
            left_normal, left_shear, right_normal, right_shear = _measurement_components(
                latest_measurement
            )
            if phase == "probe":
                latest_estimator_state = estimator.update(
                    FrictionProbeObservation(
                        tangential_demand_n=tangential_demand_n,
                        probe_excitation_n=probe_force_n,
                        left_normal_force_n=left_normal,
                        left_shear_force_n=left_shear,
                        right_normal_force_n=right_normal,
                        right_shear_force_n=right_shear,
                        dt=control_dt,
                    )
                )
                if latest_estimator_state.slip_detected:
                    estimate = latest_estimator_state
                    detection_time_s = phase_elapsed_s
                    probe_force_at_detection_n = probe_force_n
                    pending_recovery = True

            target_force_n = float(task.probe.normal_force_n)
            target_force_rate_n_s = 0.0
            if phase == "recovery":
                target_force_n = float(task.probe.recovery_normal_force_n)
            elif phase == "schedule_load":
                if estimate is None:
                    raise RuntimeError("target force scheduling requires a friction estimate")
                schedule_command = scheduler.update(
                    tangential_demand_n=tangential_demand_n,
                    friction_coefficient=float(estimate.friction_coefficient),
                    dt=control_dt,
                )
                target_force_n = schedule_command.target_force_n
                target_force_rate_n_s = schedule_command.target_force_rate_n_s
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
                    target_force_n=target_force_n,
                    approach_feedforward_force_n=float(task.approach.feedforward_force_n),
                    target_force_rate_n_s=target_force_rate_n_s,
                ),
            )
            if contact_time_s is None and force_command.state == "force_tracking":
                contact_time_s = time_s
            if (
                phase in {"approach_contact", "contact_settle"}
                and contact_time_s is not None
                and time_s - contact_time_s >= float(task.approach.settle_after_contact_s)
            ):
                model.geom_contype[support_id] = 0
                model.geom_conaffinity[support_id] = 0
                release_position = data.xpos[cube_body_id].copy()
                phase = "probe_settle"
                phase_start_time_s = time_s
            elif contact_time_s is not None and phase == "approach_contact":
                phase = "contact_settle"
                phase_start_time_s = contact_time_s

        if force_command is None or latest_measurement is None or latest_taxel_observation is None:
            raise RuntimeError("control timer did not produce an initial command")
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
        left_normal, left_shear, right_normal, right_shear = _measurement_components(
            latest_measurement
        )
        measured_shear_n = left_shear + right_shear
        release_displacement_m = (
            0.0
            if release_position is None
            else _tangential_displacement(data.xpos[cube_body_id], release_position)
        )
        hold_displacement_m = (
            0.0
            if hold_reference_position is None
            else _tangential_displacement(data.xpos[cube_body_id], hold_reference_position)
        )
        estimator_state = latest_estimator_state
        estimated_mu = (
            float(task.estimator.fallback_friction_coefficient)
            if estimator_state is None
            else float(estimator_state.friction_coefficient)
        )
        rows.append(
            {
                "time_s": float(data.time),
                "phase": step_phase,
                "phase_time_s": phase_elapsed_s,
                "true_friction_coefficient": float(task.friction_coefficient),
                "estimated_friction_coefficient": estimated_mu,
                "raw_friction_coefficient": (
                    math.nan
                    if estimator_state is None or estimator_state.raw_friction_coefficient is None
                    else float(estimator_state.raw_friction_coefficient)
                ),
                "slip_detected": False
                if estimator_state is None
                else estimator_state.slip_detected,
                "using_fallback": False
                if estimator_state is None
                else estimator_state.using_fallback,
                "mismatch_duration_s": (
                    0.0 if estimator_state is None else estimator_state.mismatch_duration_s
                ),
                "estimation_sample_count": (
                    0 if estimator_state is None else estimator_state.sample_count
                ),
                "support_residual_n": (
                    0.0 if estimator_state is None else estimator_state.support_residual_n
                ),
                "support_utilization": (
                    0.0 if estimator_state is None else estimator_state.support_utilization
                ),
                "ratio_slope_per_s": (
                    0.0 if estimator_state is None else estimator_state.ratio_slope_per_s
                ),
                "trend_armed": False if estimator_state is None else estimator_state.trend_armed,
                "side_ratio_difference": (
                    math.nan if estimator_state is None else estimator_state.side_ratio_difference
                ),
                "probe_force_n": probe_force_n,
                "additional_downward_force_n": additional_downward_force_n,
                "additional_downward_force_rate_n_s": additional_downward_rate_n_s,
                "tangential_demand_n": tangential_demand_n,
                "measured_shear_support_n": measured_shear_n,
                "measured_left_shear_n": left_shear,
                "measured_right_shear_n": right_shear,
                "measured_left_normal_n": left_normal,
                "measured_right_normal_n": right_normal,
                "scheduled_target_force_n": (
                    (
                        float(task.probe.recovery_normal_force_n)
                        if step_phase == "recovery"
                        else float(task.probe.normal_force_n)
                    )
                    if schedule_command is None or step_phase != "schedule_load"
                    else schedule_command.target_force_n
                ),
                "filtered_normal_force_n": force_command.filtered_force_n,
                "force_tracking_error_n": force_command.force_error_n,
                "available_friction_n": capacity.available_friction_n,
                "friction_margin_n": capacity.available_friction_n - tangential_demand_n,
                "active_taxel_contacts": capacity.active_contacts,
                "release_displacement_m": release_displacement_m,
                "hold_displacement_m": hold_displacement_m,
                "cube_y": float(data.xpos[cube_body_id, 1]),
                "cube_z": float(data.xpos[cube_body_id, 2]),
                "cube_vy": float(data.qvel[cube_dof + 1]),
                "cube_vz": float(data.qvel[cube_dof + 2]),
                "motor_torque_n_m": force_command.mit.torque,
                **_taxel_trace_fields(latest_taxel_observation),
            }
        )
        if pending_recovery:
            phase = "recovery"
            phase_start_time_s = float(data.time)

    if estimate is None:
        estimate = estimator.finalize()
    probe_rows = [row for row in rows if row["phase"] in {"probe_settle", "probe", "recovery"}]
    hold_rows = [
        row
        for row in rows
        if row["phase"] == "schedule_load"
        and float(row["phase_time_s"]) >= float(task.metrics.ignore_hold_initial_s)
    ]
    max_probe_displacement_m = (
        math.inf
        if not probe_rows
        else max(float(row["release_displacement_m"]) for row in probe_rows)
    )
    max_hold_displacement_m = (
        math.inf if not hold_rows else max(float(row["hold_displacement_m"]) for row in hold_rows)
    )
    hold_force_rmse_n = (
        math.inf
        if not hold_rows
        else float(
            np.sqrt(np.mean([float(row["force_tracking_error_n"]) ** 2 for row in hold_rows]))
        )
    )
    minimum_hold_margin_n = (
        -math.inf if not hold_rows else min(float(row["friction_margin_n"]) for row in hold_rows)
    )
    estimated_mu = float(estimate.friction_coefficient)
    estimate_ratio = estimated_mu / float(task.friction_coefficient)
    absolute_error = abs(estimated_mu - float(task.friction_coefficient))
    detection_passed = estimate.slip_detected or not task.metrics.require_detection
    conservatism_passed = estimated_mu <= (
        float(task.friction_coefficient) + float(task.metrics.conservative_tolerance)
    )
    informativeness_passed = estimate_ratio >= float(task.metrics.minimum_estimate_ratio)
    probe_slip_passed = max_probe_displacement_m <= float(task.metrics.probe_slip_threshold_m)
    hold_slip_passed = max_hold_displacement_m <= float(task.metrics.hold_slip_threshold_m)
    force_tracking_passed = hold_force_rmse_n <= float(task.metrics.force_rmse_threshold_n)

    if rows and output_csv is not None:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        with output_csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    if rows and output_plot is not None:
        _plot_friction_estimation(output_plot, rows, task=task)
    if rows and output_taxel_plot is not None:
        _plot_taxel_friction(output_taxel_plot, rows, task=task)

    probe_local_ratios = [
        float(row["active_taxel_max_ratio"])
        for row in rows
        if row["phase"] == "probe" and math.isfinite(float(row["active_taxel_max_ratio"]))
    ]
    detection_rows = [row for row in rows if row["phase"] == "probe" and bool(row["slip_detected"])]
    detection_row = detection_rows[0] if detection_rows else None

    return FrictionEstimationResult(
        true_friction_coefficient=float(task.friction_coefficient),
        estimated_friction_coefficient=estimated_mu,
        raw_friction_coefficient=estimate.raw_friction_coefficient,
        estimate_ratio=estimate_ratio,
        absolute_estimation_error=absolute_error,
        slip_detected=estimate.slip_detected,
        using_fallback=estimate.using_fallback,
        probe_detection_time_s=detection_time_s,
        probe_force_at_detection_n=probe_force_at_detection_n,
        max_probe_displacement_m=max_probe_displacement_m,
        max_hold_displacement_m=max_hold_displacement_m,
        hold_force_tracking_rmse_n=hold_force_rmse_n,
        minimum_hold_friction_margin_n=minimum_hold_margin_n,
        peak_active_taxel_count=max(int(row["active_taxel_count"]) for row in rows),
        peak_local_friction_ratio=max(probe_local_ratios, default=math.nan),
        local_weighted_ratio_at_detection=(
            None if detection_row is None else float(detection_row["active_taxel_weighted_ratio"])
        ),
        local_ratio_p90_at_detection=(
            None if detection_row is None else float(detection_row["active_taxel_ratio_p90"])
        ),
        simulation_stable=simulation_stable and bool(hold_rows),
        detection_passed=detection_passed,
        conservatism_passed=conservatism_passed,
        informativeness_passed=informativeness_passed,
        probe_slip_passed=probe_slip_passed,
        hold_slip_passed=hold_slip_passed,
        force_tracking_passed=force_tracking_passed,
    )


__all__ = [
    "FrictionEstimationConfigError",
    "FrictionEstimationMetricsConfig",
    "FrictionEstimationResult",
    "FrictionEstimationTask",
    "FrictionEstimatorTaskConfig",
    "TaxelFrictionTaskConfig",
    "TangentialProbeTaskConfig",
    "run_friction_estimation",
]

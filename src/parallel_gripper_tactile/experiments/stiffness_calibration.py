"""用准静态闭合扫描建立控制器语义下的等效接触刚度参考。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Annotated

import mujoco
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator
import yaml

from dm_grasp_core.control.kinematics import CrankSliderKinematics

from ..config.profiles import GripperProfile, StiffnessEstimatorMethod, load_profile
from ..contact_taxels import ContactTaxelReader
from ..control import ContactStiffnessEstimator, MITTorqueController
from ..scenes.custom import GRIPPER_PREFIX, ObjectMaterial, build_custom_grasp_model
from .grasp import _prefixed_reader, _tactile_measurement


class _CalibrationModel(BaseModel):
    """拒绝未知字段且不可变的刚度标定配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


class StiffnessCalibrationTask(_CalibrationModel):
    """准静态加载／卸载扫描的时序与闭合偏移。"""

    name: str = Field(default="quasistatic_stiffness_calibration", min_length=1)
    control_period_s: Annotated[float, Field(gt=0)] = 0.004
    approach_velocity_rad_s: Annotated[float, Field(gt=0)] = 0.25
    contact_threshold_n: Annotated[float, Field(gt=0)] = 0.25
    approach_timeout_s: Annotated[float, Field(gt=0)] = 8.0
    ramp_duration_s: Annotated[float, Field(gt=0)] = 0.20
    settle_duration_s: Annotated[float, Field(gt=0)] = 0.35
    sample_duration_s: Annotated[float, Field(gt=0)] = 0.10
    max_closure_span_m: Annotated[float, Field(gt=0)] = 1e-6
    max_force_span_n: Annotated[float, Field(gt=0)] = 0.01
    closure_offsets_m: tuple[float, ...]

    @model_validator(mode="after")
    def validate_scan(self) -> "StiffnessCalibrationTask":
        """要求偏移严格递增，并为中心差分保留至少三个工作点。"""
        if len(self.closure_offsets_m) < 3:
            raise ValueError("closure_offsets_m must contain at least three values")
        if self.closure_offsets_m[0] < 0.0:
            raise ValueError("closure_offsets_m must be non-negative")
        if any(
            right <= left for left, right in zip(self.closure_offsets_m, self.closure_offsets_m[1:])
        ):
            raise ValueError("closure_offsets_m must be strictly increasing")
        if self.sample_duration_s > self.settle_duration_s:
            raise ValueError("sample_duration_s must not exceed settle_duration_s")
        return self

    @classmethod
    def load(cls, path: str | Path) -> "StiffnessCalibrationTask":
        """从 YAML 文件加载准静态标定任务。"""
        with Path(path).open(encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        if not isinstance(raw, dict):
            raise ValueError(f"stiffness calibration task root must be a mapping: {path}")
        return cls.model_validate(raw)


@dataclass(frozen=True, slots=True)
class StiffnessCalibrationResult:
    """一个估计器在一条准静态扫描上的真值误差摘要。"""

    evaluated_points: int
    reference_stiffness_mean_n_per_m: float
    estimate_mean_n_per_m: float
    log_rmse: float
    relative_rmse: float
    relative_bias: float
    underestimation_ratio: float
    estimate_log_jitter: float
    force_increment_rmse_n: float
    force_increment_underprediction_p95_n: float
    loading_unloading_hysteresis_ratio: float
    simulation_stable: bool
    reference_coverage: float
    estimator_valid_fraction: float

    @property
    def passed(self) -> bool:
        """是否形成了足够、有限且物理方向正确的参考点。"""
        metrics = (
            self.reference_stiffness_mean_n_per_m,
            self.estimate_mean_n_per_m,
            self.log_rmse,
            self.relative_rmse,
            self.relative_bias,
            self.underestimation_ratio,
            self.estimate_log_jitter,
            self.force_increment_rmse_n,
            self.force_increment_underprediction_p95_n,
            self.loading_unloading_hysteresis_ratio,
        )
        return (
            self.simulation_stable
            and self.reference_coverage >= 0.8
            and self.estimator_valid_fraction >= 0.8
            and self.evaluated_points >= 2
            and self.reference_stiffness_mean_n_per_m > 0.0
            and all(math.isfinite(value) for value in metrics)
        )


def _kinematics(profile: GripperProfile) -> CrankSliderKinematics:
    """从 profile 构造与在线估计器完全相同的闭合运动学。"""
    if profile.normal_force is None or profile.normal_force.geometry is None:
        raise ValueError("stiffness calibration requires control.force.geometry")
    geometry = profile.normal_force.geometry
    return CrankSliderKinematics(
        theta0_rad=geometry.theta0_rad,
        crank_radius_m=geometry.crank_radius_m,
        link_length_m=geometry.link_length_m,
        offset_m=geometry.offset_m,
    )


def _raw_average_side_force(reader: ContactTaxelReader, data: mujoco.MjData) -> float:
    """返回无传感器噪声的平均单侧法向力。"""
    tactile = reader.read(data)
    left = max(0.0, float(tactile.left[2].sum()))
    right = max(0.0, float(tactile.right[2].sum()))
    return 0.5 * (left + right)


def _reference_slopes(points: list[dict[str, object]]) -> None:
    """在加载和卸载分支内部用相邻平衡点中心差分写入参考刚度。"""
    for branch in ("loading", "unloading"):
        indices = [index for index, row in enumerate(points) if row["branch"] == branch]
        for position, index in enumerate(indices):
            if position == 0 or position == len(indices) - 1:
                points[index]["reference_stiffness_n_per_m"] = None
                continue
            previous = points[indices[position - 1]]
            following = points[indices[position + 1]]
            current = points[index]
            stencil = (previous, current, following)
            x = np.asarray([float(row["closure_m"]) for row in stencil])
            y = np.asarray([float(row["normal_force_n"]) for row in stencil])
            if not all(bool(row.get("settled", True)) for row in stencil) or not (
                np.all(np.diff(x) > 1e-12) or np.all(np.diff(x) < -1e-12)
            ):
                current["reference_stiffness_n_per_m"] = None
                continue
            # 非均匀实际闭合网格上取三点插值导数，避免把弦斜率当作当前点切线。
            current["reference_stiffness_n_per_m"] = float(np.gradient(y, x)[1])


def evaluate_stiffness_estimates(
    points: list[dict[str, object]],
) -> StiffnessCalibrationResult:
    """相对独立中心差分参考计算刚度估计精度与安全相关指标。"""
    _reference_slopes(points)
    valid = [
        row
        for row in points
        if row.get("reference_stiffness_n_per_m") is not None
        and float(row["reference_stiffness_n_per_m"]) > 0.0
        and float(row["estimated_stiffness_n_per_m"]) > 0.0
        and float(row.get("estimator_valid_fraction", 1.0)) >= 0.8
    ]
    if not valid:
        return StiffnessCalibrationResult(
            evaluated_points=0,
            **{
                field: float("nan")
                for field in (
                    "reference_stiffness_mean_n_per_m",
                    "estimate_mean_n_per_m",
                    "log_rmse",
                    "relative_rmse",
                    "relative_bias",
                    "underestimation_ratio",
                    "estimate_log_jitter",
                    "force_increment_rmse_n",
                    "force_increment_underprediction_p95_n",
                    "loading_unloading_hysteresis_ratio",
                )
            },
            simulation_stable=True,
            reference_coverage=0.0,
            estimator_valid_fraction=float(
                np.mean([float(row.get("estimator_valid_fraction", 1.0)) for row in points])
            )
            if points
            else 0.0,
        )
    reference = np.asarray(
        [float(row["reference_stiffness_n_per_m"]) for row in valid], dtype=np.float64
    )
    estimate = np.asarray(
        [float(row["estimated_stiffness_n_per_m"]) for row in valid], dtype=np.float64
    )
    relative_error = (estimate - reference) / reference
    log_error = np.log(estimate) - np.log(reference)
    jitter = np.asarray([float(row["estimate_log_jitter"]) for row in valid], dtype=np.float64)

    increments: list[float] = []
    loading_increments: list[float] = []
    for branch in ("loading", "unloading"):
        branch_rows = [row for row in points if row["branch"] == branch]
        for previous, current in zip(branch_rows, branch_rows[1:]):
            predicted = float(previous["estimated_stiffness_n_per_m"]) * (
                float(current["closure_m"]) - float(previous["closure_m"])
            )
            actual = float(current["normal_force_n"]) - float(previous["normal_force_n"])
            increments.append(actual - predicted)
            if branch == "loading":
                loading_increments.append(actual - predicted)
    increment_errors = np.asarray(increments, dtype=np.float64)

    loading = {
        int(row["offset_index"]): float(row["normal_force_n"])
        for row in points
        if row["branch"] == "loading"
    }
    unloading = {
        int(row["offset_index"]): float(row["normal_force_n"])
        for row in points
        if row["branch"] == "unloading"
    }
    common = sorted(set(loading) & set(unloading))
    force_span = max(loading.values()) - min(loading.values())
    hysteresis = float(np.mean([abs(loading[index] - unloading[index]) for index in common])) / max(
        force_span, 1e-12
    )
    return StiffnessCalibrationResult(
        evaluated_points=len(valid),
        reference_stiffness_mean_n_per_m=float(reference.mean()),
        estimate_mean_n_per_m=float(estimate.mean()),
        log_rmse=float(np.sqrt(np.mean(log_error**2))),
        relative_rmse=float(np.sqrt(np.mean(relative_error**2))),
        relative_bias=float(relative_error.mean()),
        underestimation_ratio=float(np.mean(estimate < reference * (1.0 - 1e-9))),
        estimate_log_jitter=float(jitter.mean()),
        force_increment_rmse_n=float(np.sqrt(np.mean(increment_errors**2))),
        force_increment_underprediction_p95_n=float(
            np.quantile(np.maximum(loading_increments, 0.0), 0.95)
        ),
        loading_unloading_hysteresis_ratio=hysteresis,
        simulation_stable=True,
        reference_coverage=len(valid) / max(len(points) - 4, 1),
        estimator_valid_fraction=float(
            np.mean([float(row.get("estimator_valid_fraction", 1.0)) for row in points])
        ),
    )


def run_stiffness_calibration(
    profile_or_path: GripperProfile | str | Path,
    *,
    task: StiffnessCalibrationTask,
    estimator_method: StiffnessEstimatorMethod,
    object_material: ObjectMaterial,
    sensor_noise_seed: int,
) -> tuple[list[dict[str, object]], StiffnessCalibrationResult]:
    """执行加载／卸载扫描，并返回平衡点及估计误差摘要。"""
    profile = (
        profile_or_path
        if isinstance(profile_or_path, GripperProfile)
        else load_profile(profile_or_path)
    )
    if profile.normal_force is None or profile.normal_force.stiffness is None:
        raise ValueError("stiffness calibration requires control.force.stiffness")
    stiffness_config = profile.normal_force.stiffness.model_copy(
        update={"method": estimator_method}
    )
    kinematics = _kinematics(profile)
    estimator = ContactStiffnessEstimator(stiffness_config, kinematics)
    model = build_custom_grasp_model(profile, object_material=object_material)
    if task.control_period_s + 1e-12 < float(model.opt.timestep):
        raise ValueError("control_period_s must not be smaller than the physics timestep")
    data = mujoco.MjData(model)
    controller = MITTorqueController.from_profile(model, profile, name_prefix=GRIPPER_PREFIX)
    reader = _prefixed_reader(model, profile)
    rng = np.random.default_rng(sensor_noise_seed)
    noise = profile.normal_force

    def measured_force() -> float:
        tactile = _tactile_measurement(
            reader.read(data),
            left_normal_std_n=float(noise.sensor_taxel_normal_noise_std_n[0]),
            right_normal_std_n=float(noise.sensor_taxel_normal_noise_std_n[1]),
            left_shear_std_n=float(noise.sensor_taxel_shear_noise_std_n[0]),
            right_shear_std_n=float(noise.sensor_taxel_shear_noise_std_n[1]),
            rng=rng,
        )
        return 0.5 * (
            tactile.normal_capacity.left_normal_force_n
            + tactile.normal_capacity.right_normal_force_n
        )

    control_steps = max(1, round(task.control_period_s / float(model.opt.timestep)))
    if not math.isclose(control_steps * float(model.opt.timestep), task.control_period_s):
        raise ValueError("control_period_s must be an integer multiple of the physics timestep")
    command_position = float(profile.open_control)
    contact_position: float | None = None
    max_approach_steps = math.ceil(task.approach_timeout_s / float(model.opt.timestep))
    for step in range(max_approach_steps):
        command_position = min(
            float(profile.closed_control),
            float(profile.open_control) + task.approach_velocity_rad_s * float(data.time),
        )
        controller.apply(data, target_position=command_position)
        mujoco.mj_step(model, data)
        tactile = reader.read(data)
        if (
            min(float(tactile.left[2].sum()), float(tactile.right[2].sum()))
            >= task.contact_threshold_n
        ):
            contact_position = command_position
            break
    if contact_position is None:
        return [], evaluate_stiffness_estimates([])

    initial_force = measured_force()
    estimator.reset(position_rad=controller.position(data), normal_force_n=initial_force)
    contact_closure = kinematics.closure(contact_position)
    scan = [("loading", index, offset) for index, offset in enumerate(task.closure_offsets_m)] + [
        ("unloading", index, task.closure_offsets_m[index])
        for index in range(len(task.closure_offsets_m) - 1, -1, -1)
    ]
    points: list[dict[str, object]] = []
    previous_target = contact_closure
    stable = True
    physics_step = 0
    for branch, offset_index, offset in scan:
        target_closure = contact_closure + offset
        ramp_start = float(data.time)
        ramp_end = ramp_start + task.ramp_duration_s
        point_end = ramp_end + task.settle_duration_s
        estimates: list[float] = []
        raw_forces: list[float] = []
        closures: list[float] = []
        validity: list[bool] = []
        while float(data.time) + 1e-12 < point_end:
            time_s = float(data.time)
            fraction = min(1.0, max(0.0, (time_s - ramp_start) / task.ramp_duration_s))
            requested_closure = previous_target + fraction * (target_closure - previous_target)
            target_position = kinematics.position_for_closure(
                requested_closure,
                profile.mit.p_min if profile.mit is not None else 0.0,
                profile.mit.p_max if profile.mit is not None else 1.7,
            )
            if not math.isclose(
                kinematics.closure(target_position), requested_closure, abs_tol=1e-8
            ):
                raise ValueError("requested calibration closure exceeds the kinematic range")
            controller.apply(data, target_position=target_position)
            if physics_step % control_steps == 0:
                estimate = estimator.update(
                    position_rad=controller.position(data), normal_force_n=measured_force()
                )
            else:
                estimate = estimator.estimate_n_per_m
            mujoco.mj_step(model, data)
            physics_step += 1
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                stable = False
                break
            if float(data.time) >= point_end - task.sample_duration_s:
                estimates.append(float(estimate))
                raw_forces.append(_raw_average_side_force(reader, data))
                closures.append(kinematics.closure(controller.position(data)))
                validity.append(estimator.is_valid)
        if not stable or not estimates:
            break
        log_estimates = np.log(np.maximum(np.asarray(estimates), 1e-12))
        points.append(
            {
                "branch": branch,
                "offset_index": offset_index,
                "closure_offset_m": offset,
                "closure_m": float(np.mean(closures)),
                "normal_force_n": float(np.mean(raw_forces)),
                "estimated_stiffness_n_per_m": float(np.mean(estimates)),
                "estimate_log_jitter": float(np.std(log_estimates)),
                "estimator_valid_fraction": float(np.mean(validity)),
                "closure_span_m": float(np.ptp(closures)),
                "force_span_n": float(np.ptp(raw_forces)),
                "settled": bool(
                    np.ptp(closures) <= task.max_closure_span_m
                    and np.ptp(raw_forces) <= task.max_force_span_n
                ),
                "reference_stiffness_n_per_m": None,
            }
        )
        previous_target = target_closure
    if not stable:
        raise RuntimeError("quasistatic stiffness calibration became numerically unstable")
    result = evaluate_stiffness_estimates(points)
    return points, result


__all__ = [
    "StiffnessCalibrationResult",
    "StiffnessCalibrationTask",
    "evaluate_stiffness_estimates",
    "run_stiffness_calibration",
]

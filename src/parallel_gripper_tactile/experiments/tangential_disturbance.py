"""稳定夹持后通过纯触觉反馈增力抵抗切向扰动的 MIT 仿真实验。"""

from dataclasses import dataclass
import csv
import math
from pathlib import Path
from typing import Annotated, Literal

import mujoco
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, model_validator
import yaml

from ..config.profiles import GripperProfile, load_profile, validate_resolved_profile
from ..control import (
    CrankSliderKinematics,
    ForceControlObservation,
    ForceControlReference,
    NormalForceController,
)
from ..scenes.custom import (
    CUBE_PREFIX,
    GRIPPER_PREFIX,
    SUPPORT_GEOM_NAME,
    build_custom_grasp_model,
    ObjectMaterial,
)
from ..tangential_disturbance import DisturbancePolicyConfig, TactileDisturbancePolicy
from ..timing import SimulationTimer
from ..visualization.tangential_disturbance import plot_tangential_disturbance
from .force_scheduling import ForceSchedulingApproach, ForceSchedulingSolverConfig
from .grasp import (
    CUBE_BODY_NAME,
    CUBE_JOINT_NAME,
    _prefixed_reader,
    _tactile_measurement,
    _friction_capacity,
    _taxel_geom_sides,
)


class _TaskModel(BaseModel):
    """冻结任务并拒绝未知字段。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class TangentialDisturbanceLoad(_TaskModel):
    """世界 YZ 接触平面内的单向阶跃、斜坡或脉冲载荷。"""

    kind: Literal["ramp", "step", "pulse"] = "ramp"
    amplitude_n: Annotated[FiniteFloat, Field(ge=0)] = 1.5
    onset_s: Annotated[FiniteFloat, Field(ge=0)] = 0.5
    rise_duration_s: Annotated[FiniteFloat, Field(gt=0)] = 1.5
    pulse_duration_s: Annotated[FiniteFloat, Field(gt=0)] = 0.6
    duration_s: Annotated[FiniteFloat, Field(gt=0)] = 4.0
    direction_yz: tuple[FiniteFloat, FiniteFloat] = (0.0, -1.0)

    @property
    def final_change_time_s(self) -> float:
        """供离线恢复评价使用的最后一次载荷变化时刻。"""
        return self.onset_s + (
            self.rise_duration_s
            if self.kind == "ramp"
            else self.pulse_duration_s
            if self.kind == "pulse"
            else 0
        )

    @model_validator(mode="after")
    def check_duration(self):
        """拒绝零方向及缺少扰动后保持区间的任务。"""
        if math.hypot(*self.direction_yz) < 1e-9 or self.final_change_time_s >= self.duration_s:
            raise ValueError("disturbance needs a nonzero direction and a final holding interval")
        return self

    def sample_at(self, time_s: float) -> float:
        """返回仅供物理施加与离线评分的附加载荷。"""
        elapsed = time_s - self.onset_s
        if elapsed < 0:
            return 0.0
        if self.kind == "pulse":
            return self.amplitude_n if elapsed < self.pulse_duration_s else 0.0
        if self.kind == "ramp":
            return self.amplitude_n * min(1.0, elapsed / self.rise_duration_s)
        return float(self.amplitude_n)


class TangentialDisturbanceMetrics(_TaskModel):
    """含初始保持、滑移和连续恢复窗口的评价阈值。"""

    slip_threshold_m: Annotated[FiniteFloat, Field(gt=0)] = 0.002
    initial_hold_s: Annotated[FiniteFloat, Field(gt=0)] = 0.8
    force_tolerance_n: Annotated[FiniteFloat, Field(gt=0)] = 0.2
    recovery_speed_m_s: Annotated[FiniteFloat, Field(gt=0)] = 0.001
    recovery_dwell_s: Annotated[FiniteFloat, Field(gt=0)] = 0.3


class TangentialDisturbanceTask(_TaskModel):
    """触觉增力实验配置，真实摩擦仅供场景使用。"""

    schema_version: Literal[1] = 1
    name: Annotated[str, Field(min_length=1)] = "tangential_disturbance"
    cube_mass_kg: Annotated[FiniteFloat, Field(ge=0.05)] = 0.05
    friction_coefficient: Annotated[FiniteFloat, Field(gt=0)] = 0.8
    object_material: ObjectMaterial = "hard"
    approach: ForceSchedulingApproach = ForceSchedulingApproach(settle_after_contact_s=0.5)
    solver: ForceSchedulingSolverConfig = ForceSchedulingSolverConfig()
    policy: DisturbancePolicyConfig = DisturbancePolicyConfig()
    disturbance: TangentialDisturbanceLoad = TangentialDisturbanceLoad()
    metrics: TangentialDisturbanceMetrics = TangentialDisturbanceMetrics()
    control_period_s: Annotated[FiniteFloat, Field(gt=0)] = 0.002

    @model_validator(mode="after")
    def check_windows(self):
        """保证完整恢复窗口可以落在末段保持内。"""
        if (
            self.disturbance.duration_s - self.disturbance.final_change_time_s
            < self.metrics.recovery_dwell_s
        ):
            raise ValueError("final hold must cover the recovery dwell")
        if self.policy.update_period_s < self.control_period_s:
            raise ValueError("policy update period must not be smaller than control period")
        return self

    @classmethod
    def load(cls, path: str | Path):
        """从独立 YAML 或 Hydra 任务片段加载配置。"""
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            raise ValueError("disturbance task must be a mapping")
        return cls.model_validate(raw.get("definition", raw))


@dataclass(frozen=True)
class TangentialDisturbanceResult:
    """完整物理轨迹的评价，无法形成的指标明确为空。"""

    simulation_stable: bool
    completed: bool
    initial_hold_passed: bool
    slip_passed: bool
    contact_time_s: float | None
    disturbance_start_time_s: float | None
    max_tangential_displacement_m: float | None
    peak_actual_force_n: float | None
    final_actual_force_n: float | None
    final_target_force_n: float
    recovery_time_s: float | None
    increase_count: int

    @property
    def passed(self) -> bool:
        """完整完成、初始保持、滑移和恢复均合格。"""
        return (
            self.simulation_stable
            and self.completed
            and self.initial_hold_passed
            and self.slip_passed
            and self.recovery_time_s is not None
        )


def validate_tangential_disturbance_configuration(
    profile: GripperProfile, task: TangentialDisturbanceTask
) -> None:
    """在执行前验证 MIT 外环、接触阈值和物理周期。"""
    force = profile.normal_force
    if force is None or profile.control.mode != "mit_torque":
        raise ValueError("tangential disturbance requires MIT normal-force control")
    if force.admittance is not None or force.adrc is not None or force.torque_adrc is not None:
        raise ValueError("tangential disturbance currently supports PID-based MIT controllers")
    if task.policy.initial_force_n < force.contact_threshold_n:
        raise ValueError("initial force must cover the contact threshold")
    model = build_custom_grasp_model(
        profile,
        cube_mass=task.cube_mass_kg,
        object_material=task.object_material,
        friction_coefficient=task.friction_coefficient,
    )
    if task.control_period_s + 1e-12 < model.opt.timestep:
        raise ValueError("control period must not be smaller than physics timestep")


def run_tangential_disturbance(
    profile_path: Path | GripperProfile,
    *,
    task: TangentialDisturbanceTask,
    output_csv: Path | None = None,
    output_plot: Path | None = None,
) -> TangentialDisturbanceResult:
    """推进唯一物理循环，控制读触觉，指标读物体运动真值。"""
    profile = (
        validate_resolved_profile(profile_path)
        if isinstance(profile_path, GripperProfile)
        else load_profile(profile_path)
    )
    validate_tangential_disturbance_configuration(profile, task)
    model = build_custom_grasp_model(
        profile,
        cube_mass=task.cube_mass_kg,
        object_material=task.object_material,
        friction_coefficient=task.friction_coefficient,
    )
    model.opt.noslip_iterations = task.solver.noslip_iterations
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    force = profile.normal_force
    reader = _prefixed_reader(model, profile)
    controller = NormalForceController.from_profile(model, profile, name_prefix=GRIPPER_PREFIX)
    kinematics = CrankSliderKinematics.from_config(force.geometry)
    joint = model.joint(f"{GRIPPER_PREFIX}{profile.actuator}").id
    qadr = model.jnt_qposadr[joint]
    policy = TactileDisturbancePolicy(task.policy)
    rng = np.random.default_rng(force.sensor_noise_seed)
    timer = SimulationTimer(task.control_period_s)
    support = model.geom(SUPPORT_GEOM_NAME).id
    cube = model.body(CUBE_BODY_NAME).id
    cube_dof = model.jnt_dofadr[model.joint(CUBE_JOINT_NAME).id]
    cube_geom = model.geom(f"{CUBE_PREFIX}target_cube_geom").id
    taxels = _taxel_geom_sides(model, profile)
    direction = np.array(task.disturbance.direction_yz) / math.hypot(*task.disturbance.direction_yz)
    contact_time = release_time = start_time = None
    settled_since = None
    release_pos = disturbance_pos = None
    command = control = None
    rows = []
    stable = True
    completed = initial_pass = False
    initial_hold_force_ok = True
    max_hold_displacement = 0.0
    recovery_since = recovery_time = None
    deadline = (
        task.approach.timeout_s
        + task.approach.settle_after_contact_s
        + task.metrics.initial_hold_s
        + task.disturbance.duration_s
        + 1
    )
    for _ in range(math.ceil(deadline / model.opt.timestep)):
        now = float(data.time)
        if (
            release_time is None
            and now > task.approach.timeout_s + task.approach.settle_after_contact_s
        ):
            break
        phase = (
            "approach"
            if contact_time is None
            else "contact_settle"
            if release_time is None
            else "initial_hold"
            if start_time is None
            else "disturbance"
        )
        scenario_t = (
            now - start_time
            if start_time is not None
            else now - release_time - task.metrics.initial_hold_s
            if release_time is not None
            else -task.metrics.initial_hold_s
        )
        if start_time is not None and scenario_t >= task.disturbance.duration_s:
            completed = True
            break
        dt = timer.pop_due(now)
        if dt is not None:
            measurement_time_s = now
            measurement = _tactile_measurement(
                reader.read(data),
                left_normal_std_n=force.sensor_taxel_normal_noise_std_n[0],
                right_normal_std_n=force.sensor_taxel_normal_noise_std_n[1],
                left_shear_std_n=force.sensor_taxel_shear_noise_std_n[0],
                right_shear_std_n=force.sensor_taxel_shear_noise_std_n[1],
                rng=rng,
            )
            capacity = measurement.normal_capacity
            command = policy.update(
                left_normal_n=capacity.left_normal_force_n,
                right_normal_n=capacity.right_normal_force_n,
                tangential_force_n=float(
                    np.linalg.norm(measurement.left_force[:2])
                    + np.linalg.norm(measurement.right_force[:2])
                ),
                # 论文对照保留左侧局部 y 轴符号；不借助外部施力方向翻转。
                signed_tangential_force_n=float(measurement.left_force[1]),
                closure_m=kinematics.closure(float(data.qpos[qadr])),
                dt=dt,
                enabled=start_time is not None,
            )
            position = profile.open_control + min(1.0, now / task.approach.duration_s) * (
                profile.closed_control - profile.open_control
            )
            control = controller.step(
                data,
                observation=ForceControlObservation(
                    time_s=now,
                    approach_position=position,
                    total_normal_force_n=capacity.normal_force_n,
                    left_normal_force_n=capacity.left_normal_force_n,
                    right_normal_force_n=capacity.right_normal_force_n,
                    dt=dt,
                ),
                reference=ForceControlReference(
                    target_force_n=command.target_force_n,
                    target_force_rate_n_s=command.target_force_rate_n_s,
                    approach_feedforward_force_n=task.approach.feedforward_force_n,
                ),
            )
            if control.state == "force_tracking" and contact_time is None:
                contact_time = now
            force_ready = (
                control.state == "force_tracking"
                and abs(capacity.normal_force_n / 2 - task.policy.initial_force_n)
                <= task.metrics.force_tolerance_n
            )
            if release_time is not None and start_time is None:
                initial_hold_force_ok = initial_hold_force_ok and force_ready
            if release_time is None:
                settled_since = (
                    (now if settled_since is None else settled_since) if force_ready else None
                )
                if (
                    settled_since is not None
                    and now - settled_since >= task.approach.settle_after_contact_s
                ):
                    release_time = now
                    release_pos = data.xpos[cube].copy()
                    model.geom_contype[support] = model.geom_conaffinity[support] = 0
            elif start_time is None and now - release_time >= task.metrics.initial_hold_s:
                initial_pass = (
                    initial_hold_force_ok
                    and force_ready
                    and max_hold_displacement <= task.metrics.slip_threshold_m
                )
                if not initial_pass:
                    break
                start_time = now
                disturbance_pos = data.xpos[cube].copy()
        if control is None or command is None:
            raise RuntimeError("missing initial MIT command")
        # 状态转换先于物理步，记录和施力必须使用转换后的阶段。
        phase = (
            "approach"
            if contact_time is None
            else "contact_settle"
            if release_time is None
            else "initial_hold"
            if start_time is None
            else "disturbance"
        )
        load = task.disturbance.sample_at(now - start_time) if start_time is not None else 0.0
        data.xfrc_applied[cube] = 0.0
        data.xfrc_applied[cube, 1:3] = direction * load
        # 电机内环每个物理步读取最新 q/dq，触觉外环只在自己的时钟更新。
        applied = controller.apply_held_command(data)
        mujoco.mj_step(model, data)
        if data.time <= now or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            stable = False
            break
        scenario_t = (
            float(data.time) - start_time
            if start_time is not None
            else float(data.time) - release_time - task.metrics.initial_hold_s
            if release_time is not None
            else -task.metrics.initial_hold_s
        )
        capacity_true = _friction_capacity(
            model, data, taxel_geom_sides=taxels, cube_geom_id=cube_geom
        )
        actual = capacity_true.normal_force_n / 2
        pos = data.xpos[cube].copy()
        if release_pos is not None and disturbance_pos is None:
            max_hold_displacement = max(
                max_hold_displacement, float(np.linalg.norm(pos[1:3] - release_pos[1:3]))
            )
        displacement = (
            0.0
            if disturbance_pos is None
            else float(np.linalg.norm(pos[1:3] - disturbance_pos[1:3]))
        )
        speed = float(np.linalg.norm(data.qvel[cube_dof + 1 : cube_dof + 3]))
        if start_time is not None and scenario_t >= task.disturbance.final_change_time_s:
            recovered = (
                speed <= task.metrics.recovery_speed_m_s
                and abs(actual - command.target_force_n) <= task.metrics.force_tolerance_n
                and controller.state == "force_tracking"
            )
            recovery_since = (
                (scenario_t if recovery_since is None else recovery_since) if recovered else None
            )
            # 最终结果要求稳定维持至结束；中间恢复后再次失稳不能保留旧成功。
            recovery_time = (
                recovery_since - task.disturbance.final_change_time_s
                if recovery_since is not None
                and scenario_t - recovery_since >= task.metrics.recovery_dwell_s
                else None
            )
        rows.append(
            dict(
                time_s=float(data.time),
                phase=phase,
                disturbance_time_s=scenario_t,
                target_force_n=command.target_force_n,
                target_force_rate_n_s=command.target_force_rate_n_s,
                actual_normal_force_n=actual,
                measured_tangential_force_n=command.measured_tangential_force_n,
                measurement_time_s=measurement_time_s,
                measured_left_fx_n=float(measurement.left_force[0]),
                measured_left_fy_n=float(measurement.left_force[1]),
                measured_right_fx_n=float(measurement.right_force[0]),
                measured_right_fy_n=float(measurement.right_force[1]),
                measured_left_normal_n=capacity.left_normal_force_n,
                measured_right_normal_n=capacity.right_normal_force_n,
                applied_tangential_force_n=load,
                true_friction_score_only=task.friction_coefficient,
                trigger_score=command.trigger_score,
                trigger_active=int(command.trigger_active),
                ratio=command.ratio,
                ratio_valid=int(command.ratio_valid),
                increase_count=command.increase_count,
                tangential_displacement_m=displacement,
                tangential_speed_m_s=speed,
                initial_hold_displacement_m=max_hold_displacement,
                controller_state=controller.state,
                motor_torque_n_m=applied.torque,
                drive_position_rad=float(data.qpos[qadr]),
            )
        )
    evaluated = [r for r in rows if r["phase"] == "disturbance"]
    maximum = max((r["tangential_displacement_m"] for r in evaluated), default=None)
    final = [
        r["actual_normal_force_n"]
        for r in evaluated
        if r["disturbance_time_s"] >= task.disturbance.duration_s - task.metrics.recovery_dwell_s
    ]
    result = TangentialDisturbanceResult(
        stable,
        completed,
        initial_pass,
        maximum is not None and maximum <= task.metrics.slip_threshold_m,
        contact_time,
        start_time,
        maximum,
        max((r["actual_normal_force_n"] for r in evaluated), default=None),
        float(np.mean(final)) if final else None,
        policy.target,
        recovery_time,
        policy.increase_count,
    )
    if output_csv is not None and rows:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        with output_csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    if output_plot is not None:
        plot_tangential_disturbance(
            output_plot, rows, slip_threshold_m=task.metrics.slip_threshold_m
        )
    return result

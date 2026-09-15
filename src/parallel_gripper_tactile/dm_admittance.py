"""把 DM 共享导纳及接触阶段适配到 MuJoCo，不推进仿真时钟。"""

from __future__ import annotations

import math

import mujoco

from dm_grasp_core import (
    BilateralContactConfig,
    BilateralContactStateMachine,
    ContactDetector,
    ContactTransition,
    CrankSliderKinematics,
    MITCommand,
    MITCommandConfig,
    MinimumJerkTrajectory,
    SecondOrderAdmittance,
    build_mit_command,
    step_admittance,
)

from .control import (
    ForceControlObservation,
    ForceControlReference,
    ForceSemantics,
    MITControlCommand,
    MITTorqueController,
    NormalForceControlCommand,
)
from .config.profiles import GripperProfile, MITControl, NormalForceControl


class DMAdmittanceController:
    """使用平均单侧力，依次执行接近、速度过渡及二阶导纳跟踪。"""

    def __init__(
        self,
        motor: MITTorqueController,
        force: NormalForceControl,
        mit: MITControl,
        *,
        force_semantics: ForceSemantics = "average_side",
        saturation_feedback: bool = False,
    ) -> None:
        """绑定执行器并验证导纳配置，实际命令仍由现有量化器施加。

        Args:
            motor: 读取关节反馈并写入控制量的 MIT 执行器。
            force: 含导纳与机构几何的法向力控制配置。
            mit: MIT 内环增益及协议量程。
            force_semantics: 仅支持平均单侧法向力 ``average_side``。
            saturation_feedback: 是否把最终命令限幅反馈给导纳状态。

        Raises:
            ValueError: 力语义不支持、缺少导纳或几何配置，或机械限值超出 MIT 范围。
        """
        if force_semantics != "average_side":
            raise ValueError("DM admittance supports average_side only")
        if force.admittance is None or force.geometry is None:
            raise ValueError("DM admittance requires explicit admittance and geometry config")
        if mit.kp <= 0.0:
            raise ValueError("DM admittance requires positive MIT kp for torque limiting")
        self.motor = motor
        self.saturation_feedback = saturation_feedback
        self.config = force.admittance
        self.force = force
        cfg = self.config
        if (
            cfg.position_min_rad < mit.p_min
            or cfg.position_max_rad > mit.p_max
            or cfg.velocity_limit_rad_s > mit.v_max
            or cfg.mit_torque_limit_nm > mit.t_max
        ):
            raise ValueError("admittance limits exceed MIT profile limits")
        self.kinematics = CrankSliderKinematics(**force.geometry.model_dump())
        self.command_config = MITCommandConfig(
            cfg.position_min_rad,
            cfg.position_max_rad,
            cfg.velocity_limit_rad_s,
            cfg.closing_direction,
            mit.kp,
            mit.kd,
            cfg.feedforward_ratio,
            cfg.feedforward_torque_limit_nm,
            cfg.mit_torque_limit_nm,
        )
        self.admittance = SecondOrderAdmittance(cfg.mass_kg, cfg.damping_ns_m, cfg.stiffness_n_m)
        self.detector = ContactDetector(force.contact_threshold_n, cfg.contact_stable_time_s)
        self.supervisor = (
            BilateralContactStateMachine(
                BilateralContactConfig(
                    contact_threshold_n=force.contact_threshold_n,
                    contact_confirm_steps=force.contact_confirm_steps,
                    release_threshold_n=force.release_threshold_n,
                    release_confirm_steps=force.release_confirm_steps,
                    contact_transition_time_s=force.supervisor.contact_transition_time_s,
                    contact_stable_time_s=force.supervisor.contact_stable_time_s,
                    release_policy=force.supervisor.release_policy,
                )
            )
            if force.supervisor is not None
            else None
        )
        self.reset()

    @classmethod
    def from_profile(
        cls,
        model: mujoco.MjModel,
        profile: GripperProfile,
        *,
        name_prefix: str = "",
        force_semantics: ForceSemantics = "average_side",
        saturation_feedback: bool = False,
    ) -> "DMAdmittanceController":
        """根据实验已装配模型创建执行器适配器。

        Args:
            model: 已装配的 MuJoCo 模型。
            profile: 含 MIT、力控和机构配置的夹爪 profile。
            name_prefix: 场景装配附加的关节及执行器名称前缀。
            force_semantics: 仅支持 ``average_side``。
            saturation_feedback: 是否启用最终命令限幅的导纳回投。

        Returns:
            尚未执行外环或写入控制量的导纳适配器。

        Raises:
            ValueError: profile 缺少 MIT 或力控配置，或配置与模型限值不一致。
        """
        if profile.normal_force is None or profile.mit is None:
            raise ValueError("DM admittance requires MIT torque control with control.force")
        return cls(
            MITTorqueController.from_profile(model, profile, name_prefix=name_prefix),
            profile.normal_force,
            profile.mit,
            force_semantics=force_semantics,
            saturation_feedback=saturation_feedback,
        )

    @property
    def actuator_id(self) -> int:
        """返回受控执行器索引。"""
        return self.motor.actuator_id

    def reset(self) -> None:
        """清除导纳、接触确认和阶段轨迹，下次从实测位置重新接近。"""
        self.admittance.reset()
        self.detector.reset()
        if self.supervisor is not None:
            self.supervisor.reset()
        self.state = "approach"
        self.trajectory = None
        self.transition = None
        self.started_s = 0.0
        self.reference_position_rad = 0.0
        self._release_steps = 0
        self._filtered_force_n: float | None = None
        self.last_requested_command: MITCommand | None = None

    def _start_approach(self, q: float, now_s: float) -> None:
        """脱离接触后从当前反馈重建五次轨迹，防止复用旧接触参考。"""
        cfg = self.config
        goal = cfg.position_max_rad if cfg.closing_direction == 1 else cfg.position_min_rad
        self.trajectory = MinimumJerkTrajectory.from_limits(
            q,
            goal,
            cfg.approach_velocity_rad_s,
            cfg.approach_acceleration_rad_s2,
            cfg.approach_jerk_rad_s3,
        )
        self.started_s = now_s
        self.state = "approach"
        self.transition = None
        self.detector.reset()
        self.admittance.reset()
        self._release_steps = 0
        self._filtered_force_n = None

    def _filter_force(self, measured_force_n: float, dt_s: float) -> float:
        """以精确离散一阶低通滤波平均单侧原始力。

        该公式与 ``NormalForceController`` 的公共力滤波器一致。接触和释放
        状态机仍直接使用双侧原始力，避免滤波相位滞后影响安全阶段切换。
        """
        if self._filtered_force_n is None:
            self._filtered_force_n = measured_force_n
        else:
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.force.filter_cutoff_hz * dt_s)
            self._filtered_force_n += alpha * (measured_force_n - self._filtered_force_n)
        return self._filtered_force_n

    def step(
        self,
        data: mujoco.MjData,
        *,
        observation: ForceControlObservation,
        reference: ForceControlReference,
    ) -> NormalForceControlCommand:
        """读取关节反馈，以共享算法生成请求，再交给原 MIT 执行器写入控制量。

        Args:
            data: 当前仿真状态；读取关节位置、速度，并写入执行器控制量。
            observation: 双侧法向力（N）、仿真时间与外环周期（s）。
            reference: 目标平均单侧法向力（N）；接近前馈由导纳配置决定。

        Returns:
            当前阶段、原始平均单侧力及经仿真量化和限幅后的 MIT 命令。
            量化前共享请求另保存在 ``last_requested_command``。

        Raises:
            ValueError: 输入非有限、周期非正，或共享算法无法满足运动学与力矩约束。
        """
        q, dq = self.motor.position(data), self.motor.velocity(data)
        left, right = observation.left_normal_force_n, observation.right_normal_force_n
        dt, now = observation.dt, observation.time_s
        if (
            left is None
            or right is None
            or not all(
                math.isfinite(v) for v in (q, dq, left, right, dt, now, reference.target_force_n)
            )
            or dt <= 0
        ):
            raise ValueError("DM admittance requires finite dual force, feedback and positive dt")
        measured = 0.5 * (left + right)
        if self.supervisor is None and self.trajectory is None:
            self._start_approach(q, now)
        if self.supervisor is None and self.state == "force_tracking":
            one_side_released = min(left, right) <= self.force.release_threshold_n
            self._release_steps = self._release_steps + 1 if one_side_released else 0
            if self._release_steps >= self.force.release_confirm_steps:
                self._start_approach(q, now)
        filtered = self._filter_force(measured, dt)
        cfg = self.config
        if self.supervisor is not None:
            update = self.supervisor.update(
                left_force_n=left,
                right_force_n=right,
                now_s=now,
                approach_velocity_rad_s=observation.approach_velocity,
            )
            self.state = update.state
            if update.entered_transition:
                self.reference_position_rad = min(
                    max(q, cfg.position_min_rad), cfg.position_max_rad
                )
                self.admittance.reset()
            elif update.entered_tracking:
                self.reference_position_rad = min(
                    max(q, cfg.position_min_rad), cfg.position_max_rad
                )
                self.admittance.reset()
                self._release_steps = 0
            elif update.reentered_approach:
                self.reference_position_rad = 0.0
                self.admittance.reset()
                self._filtered_force_n = measured
                filtered = measured
            emitted_state = self.state
        else:
            emitted_state = self.state

        if self.state == "force_tracking":
            command = step_admittance(
                self.admittance,
                self.kinematics,
                self.command_config,
                reference_position_rad=self.reference_position_rad,
                measured_position_rad=q,
                measured_velocity_rad_s=dq,
                left_force_n=filtered,
                right_force_n=filtered,
                target_force_n=reference.target_force_n,
                dt_s=dt,
                saturation_feedback=self.saturation_feedback,
            )
        else:
            if self.state == "approach":
                if self.supervisor is None:
                    position, velocity, _ = self.trajectory.sample(now - self.started_s)
                    ff = cfg.approach_feedforward_force_n
                    ratio = cfg.approach_feedforward_ratio
                else:
                    position = observation.approach_position
                    velocity = observation.approach_velocity
                    ff = reference.approach_feedforward_force_n
                    ratio = 1.0
            else:
                position = self.reference_position_rad
                velocity = (
                    self.transition.velocity_at(now - self.started_s)
                    if self.supervisor is None
                    else self.supervisor.transition_velocity(now)
                )
                ff, ratio = 0.0, 0.0
            if self.supervisor is None:
                command = build_mit_command(
                    self.kinematics,
                    self.command_config,
                    reference_position_rad=position,
                    displacement_m=0.0,
                    velocity_m_s=cfg.closing_direction
                    * velocity
                    * self.kinematics.closure_jacobian(position),
                    measured_position_rad=q,
                    measured_velocity_rad_s=dq,
                    feedforward_force_n=ff,
                    feedforward_ratio=ratio,
                )
            else:
                feedforward_torque_nm = min(
                    max(
                        cfg.closing_direction * ratio * self.kinematics.closure_jacobian(q) * ff,
                        -cfg.feedforward_torque_limit_nm,
                    ),
                    cfg.feedforward_torque_limit_nm,
                )
                command = MITCommand(
                    position,
                    velocity,
                    self.command_config.kp,
                    self.command_config.kd,
                    feedforward_torque_nm,
                )
            if (
                self.supervisor is None
                and self.state == "approach"
                and self.detector.update(left, right, now)
            ):
                self.reference_position_rad = min(
                    max(q, cfg.position_min_rad), cfg.position_max_rad
                )
                self.transition = ContactTransition(velocity, cfg.contact_transition_time_s)
                self.state, self.started_s = "contact_transition", now
                self.detector.reset()
                self.admittance.reset()
            elif self.supervisor is None and (
                self.state == "contact_transition"
                and now - self.started_s >= self.transition.duration_s
            ):
                self.reference_position_rad = min(
                    max(q, cfg.position_min_rad), cfg.position_max_rad
                )
                self.state = "force_tracking"
                self._release_steps = 0
                self.admittance.reset()
        self.last_requested_command = command
        applied = self.apply_held_command(data)
        return NormalForceControlCommand(
            state=emitted_state,
            target_force_n=reference.target_force_n,
            measured_force_n=measured,
            filtered_force_n=filtered,
            force_error_n=reference.target_force_n - filtered,
            position_adjustment=(
                command.position_rad - self.reference_position_rad
                if emitted_state == "force_tracking"
                else 0.0
            ),
            mit=applied,
            force_feedforward_torque=command.feedforward_torque_nm,
            closure_jacobian_m_per_rad=self.kinematics.closure_jacobian(q),
            aperture_m=self.kinematics.aperture(q),
            admittance_displacement_m=(
                self.admittance.displacement_m if emitted_state == "force_tracking" else None
            ),
            admittance_velocity_m_s=(
                self.admittance.velocity_m_s if emitted_state == "force_tracking" else None
            ),
        )

    def apply_held_command(self, data: mujoco.MjData) -> MITControlCommand:
        """在物理步重算 MIT 内环，模拟电机持续执行上一外环请求。

        Args:
            data: 当前物理步反馈；方法写入控制量但不推进仿真或外环状态。

        Returns:
            使用当前关节反馈重新合成并施加的 MIT 力矩命令。

        Raises:
            RuntimeError: 尚未调用 ``step`` 生成第一个共享请求。
        """
        command = self.last_requested_command
        if command is None:
            raise RuntimeError("MIT request is not initialized")
        return self.motor.apply(
            data,
            target_position=command.position_rad,
            target_velocity=command.velocity_rad_s,
            feedforward_torque=command.feedforward_torque_nm,
            stiffness_override=command.kp,
            damping_override=command.kd,
        )

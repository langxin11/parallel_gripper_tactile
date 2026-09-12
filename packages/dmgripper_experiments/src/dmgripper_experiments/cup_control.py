"""将共用 PID、导纳和一阶 LADRC 外环映射到真机 MIT 请求。"""

from dataclasses import asdict
import math

from dm_grasp_core import SecondOrderAdmittance, build_mit_command, step_admittance
from dm_grasp_core.control import (
    AdrcConfig,
    ForceControlObservation,
    ForceControlReference,
    MITControlCommand,
    NormalForceConfig,
    NormalForceController,
)


class CupForceController:
    """只生成受限请求，不打开串口，也不重复定义外环控制律。"""

    def __init__(self, config, kinematics, command_config):
        """以启动时固定的控制器与 MIT 参数创建适配器。"""
        self.config = config
        self.kinematics = kinematics
        self.command_config = command_config
        c = config.control
        self.admittance = SecondOrderAdmittance(
            c.admittance_mass_kg, c.admittance_damping_ns_m, c.admittance_stiffness_n_m
        )
        self.normal = NormalForceController(
            NormalForceConfig(
                target_n=c.target_force_n,
                contact_threshold_n=c.contact_on_n,
                contact_confirm_steps=1,
                release_threshold_n=c.contact_off_n,
                release_confirm_steps=1,
                kp=config.pid.kp,
                ki=config.pid.ki,
                kd=config.pid.kd,
                max_position_adjustment=config.pid.max_position_adjustment_rad,
                filter_cutoff_hz=c.tactile_cutoff_hz,
                geometry=kinematics,
                adrc=AdrcConfig(**asdict(config.adrc)) if config.controller == "adrc" else None,
            )
        )
        self.feedback = None
        self.command = None
        self.reference_position = None
        self.previous_target = None
        self.dt = 0.0

    def reset(self, position_rad):
        """接触建立后重置外环和速度限幅参考。"""
        self.admittance.reset()
        self.normal.reset()
        self.reference_position = self.previous_target = position_rad
        self.command = None

    @property
    def torque_limit_n_m(self):
        """提供共享法向力核心所需的力矩上限。"""
        return self.command_config.torque_limit_nm

    def position(self):
        """只返回当前真机反馈位置。"""
        return self.feedback.position_rad

    def apply(
        self,
        *,
        target_position,
        target_velocity=0.0,
        feedforward_torque=None,
        stiffness_override=None,
        damping_override=None,
    ):
        """将共享外环请求转换为真实机械与 MIT 合成力矩限幅后的命令。"""
        if stiffness_override is not None or damping_override is not None:
            raise ValueError("cup controller does not support runtime MIT gain changes")
        c = self.command_config
        maximum = c.velocity_limit_rad_s * self.dt
        position = max(
            c.position_min_rad,
            min(
                c.position_max_rad,
                max(
                    self.previous_target - maximum,
                    min(self.previous_target + maximum, target_position),
                ),
            ),
        )
        velocity = max(-c.velocity_limit_rad_s, min(c.velocity_limit_rad_s, target_velocity))
        ff = 0.0 if feedforward_torque is None else feedforward_torque
        jacobian = self.kinematics.closure_jacobian(self.feedback.position_rad)
        if not math.isfinite(jacobian) or abs(jacobian) < 1e-9:
            raise ValueError("invalid gripper Jacobian")
        self.command = build_mit_command(
            self.kinematics,
            c,
            reference_position_rad=position,
            displacement_m=0.0,
            velocity_m_s=velocity * self.kinematics.closure_jacobian(position),
            measured_position_rad=self.feedback.position_rad,
            measured_velocity_rad_s=self.feedback.velocity_rad_s,
            feedforward_force_n=ff / jacobian,
            feedforward_ratio=1.0,
        )
        self.previous_target = self.command.position_rad
        torque = (
            c.kp * (self.command.position_rad - self.feedback.position_rad)
            + c.kd * (self.command.velocity_rad_s - self.feedback.velocity_rad_s)
            + self.command.feedforward_torque_nm
        )
        return MITControlCommand(
            self.command.position_rad,
            self.command.velocity_rad_s,
            self.feedback.position_rad,
            self.feedback.velocity_rad_s,
            self.command.feedforward_torque_nm,
            torque,
        )

    def step(self, *, feedback, tactile, target_force_n, time_s, dt_s):
        """执行所选外环，目标统一为平均单侧力。"""
        if not math.isfinite(dt_s) or not 0 < dt_s <= self.config.max_control_gap_s:
            raise RuntimeError("force control cycle exceeded its valid interval")
        if (
            not math.isfinite(target_force_n)
            or not 0 <= target_force_n <= self.config.grip.max_target_force_n
        ):
            raise ValueError("force target outside cup experiment limits")
        self.feedback, self.dt = feedback, dt_s
        if self.reference_position is None:
            self.reset(feedback.position_rad)
        if self.config.controller == "admittance":
            self.command = step_admittance(
                self.admittance,
                self.kinematics,
                self.command_config,
                reference_position_rad=self.reference_position,
                measured_position_rad=feedback.position_rad,
                measured_velocity_rad_s=feedback.velocity_rad_s,
                left_force_n=tactile.left_force_n,
                right_force_n=tactile.right_force_n,
                target_force_n=target_force_n,
                dt_s=dt_s,
            )
            self.previous_target = self.command.position_rad
        else:
            # 外部状态机已经确认接触，核心仅复用原有 PID／LADRC 跟踪计算。
            self.normal.step(
                self,
                observation=ForceControlObservation(
                    time_s=time_s,
                    approach_position=feedback.position_rad,
                    total_normal_force_n=tactile.left_force_n + tactile.right_force_n,
                    left_normal_force_n=tactile.left_force_n,
                    right_normal_force_n=tactile.right_force_n,
                    dt=dt_s,
                ),
                reference=ForceControlReference(target_force_n=target_force_n),
            )
        return self.command

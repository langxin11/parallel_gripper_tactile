"""用于 MuJoCo 电机执行器的 MIT 风格力矩控制。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from simple_pid import PID

from .profiles import GripperProfile, MITControl, NormalForceControl


@dataclass(frozen=True, slots=True)
class MITControlCommand:
    """一个经过饱和处理的 MIT 力矩命令，以及用于计算它的状态。"""

    target_position: float
    target_velocity: float
    position: float
    velocity: float
    feedforward_torque: float
    torque: float


class MITTorqueController:
    """将 ``kp*(p_des-p) + kd*(v_des-v) + t_ff`` 施加到电机执行器。"""

    def __init__(
        self,
        model,
        *,
        actuator_name: str,
        joint_name: str,
        config: MITControl,
    ) -> None:
        """解析模型索引，并拒绝与 MJCF 不一致的限值。"""
        self._actuator_id = model.actuator(actuator_name).id
        joint_id = model.joint(joint_name).id
        self._qpos_address = int(model.jnt_qposadr[joint_id])
        self._dof_address = int(model.jnt_dofadr[joint_id])
        self._config = config
        actuator_force_limit = float(np.min(np.abs(model.actuator_forcerange[self._actuator_id])))
        actuator_control_limit = float(np.min(np.abs(model.actuator_ctrlrange[self._actuator_id])))
        if config.t_max > actuator_force_limit or config.t_max > actuator_control_limit:
            raise ValueError(
                f"MIT t_max={config.t_max:g} exceeds actuator limits "
                f"ctrl={actuator_control_limit:g}, force={actuator_force_limit:g}"
            )

    @classmethod
    def from_profile(
        cls,
        model,
        profile: GripperProfile,
        *,
        name_prefix: str = "",
    ) -> "MITTorqueController":
        """基于 profile 和一个可选的附加模型前缀构建控制器。"""
        if profile.control_mode != "mit_torque" or profile.mit is None:
            raise ValueError("profile does not define MIT torque control")
        return cls(
            model,
            actuator_name=f"{name_prefix}{profile.actuator}",
            joint_name=f"{name_prefix}{profile.actuator}",
            config=profile.mit,
        )

    @property
    def actuator_id(self) -> int:
        """返回此实例所控制的编译后执行器索引。"""
        return self._actuator_id

    def apply(
        self,
        data,
        *,
        target_position: float,
        target_velocity: float = 0.0,
        feedforward_torque: float | None = None,
    ) -> MITControlCommand:
        """计算、饱和、写入并返回一个 MIT 力矩命令。"""
        config = self._config
        desired_position = float(np.clip(target_position, config.p_min, config.p_max))
        desired_velocity = float(np.clip(target_velocity, -config.v_max, config.v_max))
        feedforward = float(
            np.clip(
                config.t_ff if feedforward_torque is None else feedforward_torque,
                -config.t_max,
                config.t_max,
            )
        )
        position = float(data.qpos[self._qpos_address])
        velocity = float(data.qvel[self._dof_address])
        torque = config.kp * (desired_position - position)
        torque += config.kd * (desired_velocity - velocity) + feedforward
        torque = float(np.clip(torque, -config.t_max, config.t_max))
        data.ctrl[self._actuator_id] = torque
        return MITControlCommand(
            target_position=desired_position,
            target_velocity=desired_velocity,
            position=position,
            velocity=velocity,
            feedforward_torque=feedforward,
            torque=torque,
        )


@dataclass(frozen=True, slots=True)
class NormalForceControlCommand:
    """一个混合的接近/力控制命令及其观察到的力状态。"""

    state: str
    target_force_n: float
    measured_force_n: float
    filtered_force_n: float
    force_error_n: float
    position_adjustment: float
    mit: MITControlCommand


class NormalForceController:
    """先以位置控制接近，再跟踪相加后的 taxel 法向力。"""

    def __init__(
        self,
        inner: MITTorqueController,
        config: NormalForceControl,
    ) -> None:
        """创建 simple-pid 外环与接触状态机。"""
        self._inner = inner
        self._config = config
        adjustment = config.max_position_adjustment
        self._pid = PID(
            config.kp,
            config.ki,
            config.kd,
            setpoint=config.target_n,
            sample_time=None,
            output_limits=(-adjustment, adjustment),
            auto_mode=False,
        )
        self.reset()

    @classmethod
    def from_profile(
        cls,
        model,
        profile: GripperProfile,
        *,
        name_prefix: str = "",
    ) -> "NormalForceController":
        """基于一个 profile 构建外环力环路及其 MIT 内环。"""
        if profile.normal_force is None:
            raise ValueError("profile does not define normal-force control")
        return cls(
            MITTorqueController.from_profile(model, profile, name_prefix=name_prefix),
            profile.normal_force,
        )

    @property
    def actuator_id(self) -> int:
        """返回由内环 MIT 环路控制的电机执行器。"""
        return self._inner.actuator_id

    @property
    def state(self) -> str:
        """返回 ``approach`` 或 ``force_tracking``。"""
        return self._state

    def reset(self) -> None:
        """返回接近模式，并清除滤波器、计数器和 PID 历史。"""
        self._state = "approach"
        self._filtered_force: float | None = None
        self._contact_position = 0.0
        self._contact_steps = 0
        self._release_steps = 0
        self._pid.set_auto_mode(False)
        self._pid.reset()

    def apply(
        self,
        data,
        *,
        approach_position: float,
        total_normal_force_n: float,
        left_normal_force_n: float,
        right_normal_force_n: float,
        dt: float,
        approach_velocity: float = 0.0,
    ) -> NormalForceControlCommand:
        """推进接触检测/力跟踪，并写入一个电机力矩。

        接触要求两个指尖都保持在配置阈值之上。一旦确认，simple-pid 会
        调整检测到的接触位置，现有 MIT 控制器再将该位置目标转换为电机力矩。
        """
        if dt <= 0:
            raise ValueError("dt must be positive")
        config = self._config
        measured_force = max(0.0, float(total_normal_force_n))
        if self._filtered_force is None:
            self._filtered_force = measured_force
        else:
            alpha = 1.0 - np.exp(-2.0 * np.pi * config.filter_cutoff_hz * dt)
            self._filtered_force += float(alpha) * (measured_force - self._filtered_force)

        if self._state == "approach":
            both_contacting = min(left_normal_force_n, right_normal_force_n)
            self._contact_steps = (
                self._contact_steps + 1 if both_contacting >= config.contact_threshold_n else 0
            )
            mit = self._inner.apply(
                data,
                target_position=approach_position,
                target_velocity=approach_velocity,
            )
            adjustment = 0.0
            if self._contact_steps >= config.contact_confirm_steps:
                self._state = "force_tracking"
                self._contact_position = mit.position
                self._pid.reset()
                self._pid.set_auto_mode(True, last_output=0.0)
                adjustment = float(self._pid(self._filtered_force, dt=dt))
                mit = self._inner.apply(
                    data,
                    target_position=self._contact_position + adjustment,
                )
        else:
            both_released = max(left_normal_force_n, right_normal_force_n)
            self._release_steps = (
                self._release_steps + 1 if both_released <= config.release_threshold_n else 0
            )
            if self._release_steps >= config.release_confirm_steps:
                self.reset()
                mit = self._inner.apply(
                    data,
                    target_position=approach_position,
                    target_velocity=approach_velocity,
                )
                adjustment = 0.0
            else:
                adjustment = float(self._pid(self._filtered_force, dt=dt))
                mit = self._inner.apply(
                    data,
                    target_position=self._contact_position + adjustment,
                )

        reported_filtered_force = (
            measured_force if self._filtered_force is None else self._filtered_force
        )
        return NormalForceControlCommand(
            state=self._state,
            target_force_n=config.target_n,
            measured_force_n=measured_force,
            filtered_force_n=reported_filtered_force,
            force_error_n=config.target_n - reported_filtered_force,
            position_adjustment=adjustment,
            mit=mit,
        )

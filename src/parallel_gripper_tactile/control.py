"""MIT-style torque control for MuJoCo motor actuators."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from simple_pid import PID

from .profiles import GripperProfile, MITControl, NormalForceControl


@dataclass(frozen=True, slots=True)
class MITControlCommand:
    """One saturated MIT torque command and the state used to compute it."""

    target_position: float
    target_velocity: float
    position: float
    velocity: float
    feedforward_torque: float
    torque: float


class MITTorqueController:
    """Apply ``kp*(p_des-p) + kd*(v_des-v) + t_ff`` to a motor actuator."""

    def __init__(
        self,
        model,
        *,
        actuator_name: str,
        joint_name: str,
        config: MITControl,
    ) -> None:
        """Resolve model indices and reject limits inconsistent with the MJCF."""
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
        """Build a controller from a profile and an optional attached-model prefix."""
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
        """Return the compiled actuator index controlled by this instance."""
        return self._actuator_id

    def apply(
        self,
        data,
        *,
        target_position: float,
        target_velocity: float = 0.0,
        feedforward_torque: float | None = None,
    ) -> MITControlCommand:
        """Compute, saturate, write, and return one MIT torque command."""
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
    """One hybrid approach/force-control command and its observed force state."""

    state: str
    target_force_n: float
    measured_force_n: float
    filtered_force_n: float
    force_error_n: float
    position_adjustment: float
    mit: MITControlCommand


class NormalForceController:
    """Approach in position control, then track summed taxel normal force."""

    def __init__(
        self,
        inner: MITTorqueController,
        config: NormalForceControl,
    ) -> None:
        """Create the simple-pid outer loop and contact state machine."""
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
        """Build the outer force loop and its MIT inner loop from one profile."""
        if profile.normal_force is None:
            raise ValueError("profile does not define normal-force control")
        return cls(
            MITTorqueController.from_profile(model, profile, name_prefix=name_prefix),
            profile.normal_force,
        )

    @property
    def actuator_id(self) -> int:
        """Return the motor actuator controlled by the inner MIT loop."""
        return self._inner.actuator_id

    @property
    def state(self) -> str:
        """Return ``approach`` or ``force_tracking``."""
        return self._state

    def reset(self) -> None:
        """Return to approach mode and clear filter, counters, and PID history."""
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
        """Advance contact detection/force tracking and write one motor torque.

        Contact requires both fingertips to remain above the configured threshold.
        Once confirmed, simple-pid adjusts the detected contact position and the
        existing MIT controller converts that position target into motor torque.
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
                self._contact_steps + 1
                if both_contacting >= config.contact_threshold_n
                else 0
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
                self._release_steps + 1
                if both_released <= config.release_threshold_n
                else 0
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

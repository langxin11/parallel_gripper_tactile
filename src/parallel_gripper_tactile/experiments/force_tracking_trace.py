"""力跟踪实验的逐步记录字段映射。"""

from __future__ import annotations

import math

import numpy as np

from ..control import ForceSemantics, NormalForceControlCommand
from .grasp import FrictionCapacity, TactileMeasurement


def force_tracking_trace_row(
    *,
    time_s: float,
    control_time_s: float,
    command_time_s: float,
    tracking_start_time_s: float | None,
    phase: str,
    force_semantics: ForceSemantics,
    multiccd_enabled: bool,
    tracking_time_s: float,
    force_command: NormalForceControlCommand,
    target_force_rate_n_s: float,
    target_force_acceleration_n_s2: float,
    actuator_torque_n_m: float,
    tactile_measurement: TactileMeasurement,
    capacity: FrictionCapacity,
    position: np.ndarray,
    velocity: np.ndarray,
) -> dict[str, float | str]:
    """把已采样的命令、触觉和物体状态映射为原有顺序的 trace 字段。"""
    motor_command = force_command.mit
    return {
        "time_s": time_s,
        "control_time_s": control_time_s,
        "command_time_s": command_time_s,
        "reference_start_time_s": (
            math.nan if tracking_start_time_s is None else tracking_start_time_s
        ),
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
        "desired_position_rad": motor_command.target_position,
        "desired_velocity_rad_s": motor_command.target_velocity,
        "drive_position_rad": motor_command.position,
        "drive_velocity_rad_s": motor_command.velocity,
        "motor_torque_n_m": motor_command.torque,
        "commanded_torque_n_m": motor_command.torque,
        "actuator_torque_n_m": actuator_torque_n_m,
        "stiffness_valid": str(force_command.stiffness_valid).lower(),
        "admittance_displacement_m": (
            force_command.admittance_displacement_m
            if force_command.admittance_displacement_m is not None
            else math.nan
        ),
        "admittance_velocity_m_s": (
            force_command.admittance_velocity_m_s
            if force_command.admittance_velocity_m_s is not None
            else math.nan
        ),
        "force_position_adjustment_rad": force_command.position_adjustment,
        "pid_position_adjustment_rad": force_command.pid_position_adjustment,
        "stiffness_position_limit_rad": (
            force_command.stiffness_position_limit_rad
            if force_command.stiffness_position_limit_rad is not None
            else math.nan
        ),
        "stiffness_position_limited": str(force_command.stiffness_position_limited).lower(),
        "stiffness_rate_force_command_n_s": (
            force_command.stiffness_rate_force_command_n_s
            if force_command.stiffness_rate_force_command_n_s is not None
            else math.nan
        ),
        "stiffness_rate_joint_velocity_rad_s": (
            force_command.stiffness_rate_joint_velocity_rad_s
            if force_command.stiffness_rate_joint_velocity_rad_s is not None
            else math.nan
        ),
        "stiffness_rate_force_limited": str(force_command.stiffness_rate_force_limited).lower(),
        "stiffness_rate_joint_velocity_limited": str(
            force_command.stiffness_rate_joint_velocity_limited
        ).lower(),
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
        "torque_adrc_amplitude_limited": str(force_command.torque_adrc_amplitude_limited).lower(),
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
            force_command.aperture_m if force_command.aperture_m is not None else math.nan
        ),
        "measured_left_fx": float(tactile_measurement.left_force[0]),
        "measured_left_fy": float(tactile_measurement.left_force[1]),
        "measured_left_fz": float(tactile_measurement.left_force[2]),
        "measured_right_fx": float(tactile_measurement.right_force[0]),
        "measured_right_fy": float(tactile_measurement.right_force[1]),
        "measured_right_fz": float(tactile_measurement.right_force[2]),
        "left_tangential_force_n": float(np.linalg.norm(tactile_measurement.left_force[:2])),
        "right_tangential_force_n": float(np.linalg.norm(tactile_measurement.right_force[:2])),
        **{
            f"{side}_taxel_{component}_{row}_{col}": float(values[axis, row, col])
            for side, values in (
                ("left", tactile_measurement.left),
                ("right", tactile_measurement.right),
            )
            for axis, component in enumerate(("fx", "fy", "fz"))
            for row in range(values.shape[1])
            for col in range(values.shape[2])
        },
        "taxel_normal_force_n": capacity.normal_force_n,
        "left_taxel_normal_force_n": capacity.left_normal_force_n,
        "right_taxel_normal_force_n": capacity.right_normal_force_n,
        "active_taxel_contacts": capacity.active_contacts,
        "cube_y": float(position[1]),
        "cube_z": float(position[2]),
        "cube_vy": float(velocity[1]),
        "cube_vz": float(velocity[2]),
    }

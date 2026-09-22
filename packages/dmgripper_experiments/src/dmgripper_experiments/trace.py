"""真机运行时 trace 行的字段映射。"""

from __future__ import annotations

from dm_grasp_core import MITCommand, StiffnessSnapshot

from .lifecycle import LifecyclePhase
from .observation import PairedObservation
from .targets import ForceTarget


def _trace_row(
    *,
    time_s: float,
    paired: PairedObservation,
    axes: tuple[float, ...],
    phase: LifecyclePhase,
    task_time_s: float | None,
    contact_segment: int,
    target: ForceTarget | None,
    controller_step,
    stiffness: StiffnessSnapshot | None,
    feedback,
    command: MITCommand,
    dt: float,
    tactile_age_s: float,
    latency: float,
) -> dict[str, object]:
    """组装一个控制周期的 trace 行；缺失量保持为空。"""
    snapshot = paired.snapshot
    row: dict[str, object] = {
        f"raw_{side}_f{axis}_n": value
        for (side, axis), value in zip(
            ((side, axis) for side in ("left", "right") for axis in "xyz"), axes, strict=True
        )
    }
    row.update(
        time_s=time_s,
        phase=phase.value,
        task_time_s=task_time_s,
        contact_segment=contact_segment,
        tactile_received_at_s=snapshot.received_at_s,
        tactile_timestamp_us=snapshot.timestamp_us,
        packet_counter=snapshot.packet_counter,
        left_fz_n=snapshot.left_force_n,
        right_fz_n=snapshot.right_force_n,
        measured_force_n=paired.measured_force_n,
        control_force_n=(controller_step.filtered_force_n if controller_step is not None else None),
        target_source=target.source if target is not None else None,
        target_force_n=target.force_n if target is not None else None,
        target_raw_force_n=target.raw_force_n if target is not None else None,
        target_force_rate_n_s=target.rate_n_s if target is not None else None,
        target_force_acceleration_n_s2=(target.acceleration_n_s2 if target is not None else None),
        target_trigger_active=target.trigger_active if target is not None else None,
        target_increase_count=target.increase_count if target is not None else None,
        measured_tangential_force_n=(
            target.measured_tangential_force_n
            if target is not None and target.source == "adaptive"
            else None
        ),
        stiffness_n_per_m=stiffness.value_n_per_m if stiffness is not None else None,
        stiffness_valid=stiffness.valid if stiffness is not None else None,
        stiffness_updated=stiffness.updated if stiffness is not None else None,
        stiffness_reason=stiffness.reason if stiffness is not None else None,
        force_deadband_active=(
            controller_step.force_deadband_active if controller_step is not None else False
        ),
        unloading_blocked=(
            controller_step.unloading_blocked if controller_step is not None else False
        ),
        position_rad=feedback.position_rad,
        velocity_rad_s=feedback.velocity_rad_s,
        torque_nm=feedback.torque_nm,
        q_des_rad=command.position_rad,
        dq_des_rad_s=command.velocity_rad_s,
        kp=command.kp,
        kd=command.kd,
        tau_ff_nm=command.feedforward_torque_nm,
        control_dt_s=dt,
        tactile_age_s=tactile_age_s,
        command_latency_s=latency,
    )
    return row


def _motor_only_trace_row(
    *,
    time_s: float,
    phase: LifecyclePhase,
    feedback,
    command: MITCommand,
    dt: float,
    latency: float,
) -> dict[str, object]:
    """记录不依赖触觉的 homing／故障释放周期。"""
    return {
        "time_s": time_s,
        "phase": phase.value,
        "position_rad": feedback.position_rad,
        "velocity_rad_s": feedback.velocity_rad_s,
        "torque_nm": feedback.torque_nm,
        "q_des_rad": command.position_rad,
        "dq_des_rad_s": command.velocity_rad_s,
        "kp": command.kp,
        "kd": command.kd,
        "tau_ff_nm": command.feedforward_torque_nm,
        "control_dt_s": dt,
        "command_latency_s": latency,
    }

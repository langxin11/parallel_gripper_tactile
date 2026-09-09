"""DM4310P 与 PapillArray 的最小纯 Python 力跟踪运行时。"""

from __future__ import annotations

import csv
import math
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path

from dm_grasp_core import (
    ContactTransition,
    CrankSliderKinematics,
    MITCommand,
    MITCommandConfig,
    SecondOrderAdmittance,
    build_mit_command,
    step_admittance,
)
from dmgripper_hardware import (
    CMD_DISABLE,
    CMD_ENABLE,
    CONTROL_MODE_REGISTER,
    STATUS_DISABLED,
    STATUS_ENABLED,
    DmMitCommandAdapter,
    DmRegisterReader,
    DmResponseReceiver,
    DmStateRefresher,
    MotorFeedback,
    PySerialTransport,
    Usb2CanProtocol,
    make_dm4310p_gripper_config,
    motor_status_is_fault,
)
from papillarray_hardware import PapillArraySerialConfig

from .config import ForceDemoConfig
from .state_machine import ForceTrackingState, ForceTrackingStateMachine
from .tactile import TactileSnapshot, TactileWorker
from .trajectory import ClosureTrajectory

EventSink = Callable[[dict[str, object]], None]


@dataclass(frozen=True, slots=True)
class ForceDemoResult:
    """单次实验的最终结果。"""

    completed: bool
    disable_confirmed: bool
    final_state: str
    final_position_rad: float
    csv_path: str


class _DmSession:
    """由控制线程独占的 DM 串口会话。"""

    def __init__(self, port: str, timeout_s: float) -> None:
        """构造协议对象，不打开串口。"""
        self.deployment = make_dm4310p_gripper_config(port, timeout_s=timeout_s)
        self.protocol = Usb2CanProtocol(self.deployment.motor_limits)
        self.transport = PySerialTransport()
        self.receiver = DmResponseReceiver(self.deployment.device, self.protocol, self.transport)
        self.refresher = DmStateRefresher(
            self.deployment.device,
            self.protocol,
            self.transport,
            receiver=self.receiver,
        )
        self.registers = DmRegisterReader(
            self.deployment.device, self.protocol, self.transport, self.receiver
        )
        self.adapter = DmMitCommandAdapter(self.protocol, self.deployment.motor_id)

    def open(self) -> None:
        """打开 DM 串口。"""
        self.refresher.open()

    def close(self) -> None:
        """关闭 DM 串口。"""
        self.refresher.close()

    def inspect(self) -> MotorFeedback:
        """核对初始反馈和 MIT 模式。"""
        self._validate_feedback(self.refresher.refresh_once())
        mode = self.registers.read_u32(CONTROL_MODE_REGISTER)
        if mode != 1:
            raise RuntimeError(f"控制模式不是 MIT：寄存器 {CONTROL_MODE_REGISTER}={mode}")
        feedback = self.refresher.refresh_once()
        self._validate_feedback(feedback)
        return feedback

    def enable(self) -> MotorFeedback:
        """使能并要求新的使能反馈。"""
        self._write(self.protocol.make_control_packet(self.deployment.motor_id, CMD_ENABLE))
        feedback = self.receiver.receive_feedback()
        self._validate_feedback(feedback)
        if feedback.status_code != STATUS_ENABLED:
            raise RuntimeError(f"DM 使能确认失败：status_code={feedback.status_code}")
        return feedback

    def command(self, command: MITCommand) -> MotorFeedback:
        """发送一个控制核 MIT 请求并返回该命令产生的新反馈。"""
        self._write(self.adapter.prepare(command).frame)
        feedback = self.receiver.receive_feedback()
        self._validate_feedback(feedback)
        if feedback.status_code != STATUS_ENABLED:
            raise RuntimeError(f"DM 运行中失能：status_code={feedback.status_code}")
        return feedback

    def disable(self) -> MotorFeedback:
        """失能并确认状态码。"""
        self._write(self.protocol.make_control_packet(self.deployment.motor_id, CMD_DISABLE))
        feedback = self.refresher.refresh_once()
        if feedback.status_code != STATUS_DISABLED:
            raise RuntimeError(f"DM 最终失能确认失败：status_code={feedback.status_code}")
        return feedback

    def _write(self, frame: bytes) -> None:
        """要求 USB2CAN 完整接受一帧。"""
        written = self.transport.write(frame, timeout_s=self.deployment.device.timeout_s)
        if written != len(frame):
            raise OSError(f"DM 命令短写：期望 {len(frame)} 字节，实际 {written} 字节")

    def _validate_feedback(self, feedback: MotorFeedback) -> None:
        """拒绝故障与机械行程外反馈。"""
        if motor_status_is_fault(feedback.status_code):
            raise RuntimeError(f"DM 电机故障：status_code={feedback.status_code}")
        self.deployment.validate_joint_position(feedback.position_rad)


def run_force_demo(
    config: ForceDemoConfig,
    *,
    dm_port: str,
    tactile_port: str,
    output_path: Path,
    clear_bias: bool,
    verify_zero_force: bool = True,
    event_sink: EventSink | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> ForceDemoResult:
    """执行轨迹接近、接触过渡、导纳力跟踪、回位和失能。"""
    emit = event_sink or (lambda _event: None)
    tactile_config = PapillArraySerialConfig(
        port=tactile_port,
        sampling_rate=500,
        expected_sensors=2,
        timeout_s=0.05,
        packet_timeout_s=config.tactile_timeout_s,
    )
    tactile = TactileWorker(
        tactile_config,
        clear_bias=clear_bias,
        clock=clock,
        cutoff_hz=config.tactile_cutoff_hz,
        filter_reset_gap_s=config.tactile_filter_reset_gap_s,
    )
    dm = _DmSession(dm_port, timeout_s=0.05)
    enabled = False
    feedback: MotorFeedback | None = None
    output_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        tactile.start()
        if verify_zero_force:
            _verify_zero_force(tactile, config, clear_bias, clock, sleep)
        else:
            first = tactile.wait_for_update(None, config.tactile_startup_timeout_s)
            _validate_tactile_freshness(first, clock(), config.tactile_timeout_s)
            emit(
                {
                    "event": "warning",
                    "message": "已显式跳过双侧零力稳定验证",
                    "left_fz_n": first.left_force_n,
                    "right_fz_n": first.right_force_n,
                }
            )
        dm.open()
        feedback = dm.inspect()
        initial_position_rad = feedback.position_rad
        emit(
            {
                "event": "ready",
                "initial_position_rad": initial_position_rad,
                "clear_bias": clear_bias,
            }
        )
        feedback = dm.enable()
        enabled = True
        feedback, final_state = _run_enabled(
            config,
            dm,
            tactile,
            feedback,
            output_path,
            emit,
            clock,
            sleep,
        )
        disabled = dm.disable()
        enabled = False
        return ForceDemoResult(
            completed=final_state is ForceTrackingState.COMPLETE,
            disable_confirmed=True,
            final_state=final_state.value,
            final_position_rad=disabled.position_rad,
            csv_path=str(output_path),
        )
    finally:
        disable_error: BaseException | None = None
        if enabled:
            try:
                dm.disable()
            except BaseException as error:  # noqa: BLE001
                disable_error = error
        try:
            dm.close()
        finally:
            tactile.stop()
        if disable_error is not None:
            raise RuntimeError(f"实验退出后 DM 失能失败：{disable_error}") from disable_error


def _verify_zero_force(
    tactile: TactileWorker,
    config: ForceDemoConfig,
    clear_bias: bool,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    """要求使能前双侧零力窗口持续稳定。"""
    first = tactile.wait_for_update(None, config.tactile_startup_timeout_s)
    if clear_bias:
        sleep(config.tactile_bias_settle_s)
        first = tactile.wait_for_update(first.received_at_s, config.tactile_timeout_s * 2.0)
    deadline = clock() + config.zero_force_timeout_s
    stable_started_s: float | None = None
    previous: float | None = None
    pending: TactileSnapshot | None = first
    last = first
    best_max_abs_n = max(abs(first.left_force_n), abs(first.right_force_n))
    while True:
        remaining = deadline - clock()
        if remaining <= 0.0:
            raise RuntimeError(
                "使能前双侧零力验证超时："
                f"阈值=±{config.zero_force_threshold_n:.3f}N，"
                f"末值 LEFT={last.left_force_n:+.3f}N、RIGHT={last.right_force_n:+.3f}N，"
                f"末原始 Fz LEFT={last.raw_left_fz_n:+.3f}N、"
                f"RIGHT={last.raw_right_fz_n:+.3f}N，"
                f"观测到的最佳双侧最大绝对值={best_max_abs_n:.3f}N"
            )
        snapshot = pending or tactile.wait_for_update(previous, remaining)
        pending = None
        previous = snapshot.received_at_s
        last = snapshot
        best_max_abs_n = min(
            best_max_abs_n,
            max(abs(snapshot.left_force_n), abs(snapshot.right_force_n)),
        )
        if (
            abs(snapshot.left_force_n) <= config.zero_force_threshold_n
            and abs(snapshot.right_force_n) <= config.zero_force_threshold_n
        ):
            if stable_started_s is None:
                stable_started_s = snapshot.received_at_s
            elif snapshot.received_at_s - stable_started_s >= config.zero_force_stable_s:
                return
        else:
            stable_started_s = None


def _run_enabled(
    config: ForceDemoConfig,
    dm: _DmSession,
    tactile: TactileWorker,
    feedback: MotorFeedback,
    output_path: Path,
    emit: EventSink,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> tuple[MotorFeedback, ForceTrackingState]:
    """运行使能后的有限状态机。"""
    kinematics = CrankSliderKinematics(
        theta0_rad=0.7853981633974483,
        crank_radius_m=0.03,
        link_length_m=0.04,
        offset_m=0.021213203435596423,
    )
    command_config = MITCommandConfig(
        position_min_rad=dm.deployment.joint_position_min_rad,
        position_max_rad=dm.deployment.joint_position_max_rad,
        velocity_limit_rad_s=config.velocity_limit_rad_s,
        closing_direction=dm.deployment.closing_direction,
        kp=config.mit_kp,
        kd=config.mit_kd,
        feedforward_ratio=0.0,
        feedforward_torque_limit_nm=config.torque_limit_nm,
        torque_limit_nm=config.torque_limit_nm,
    )
    return_command_config = MITCommandConfig(
        position_min_rad=command_config.position_min_rad,
        position_max_rad=command_config.position_max_rad,
        velocity_limit_rad_s=command_config.velocity_limit_rad_s,
        closing_direction=command_config.closing_direction,
        kp=config.return_mit_kp,
        kd=config.return_mit_kd,
        feedforward_ratio=0.0,
        feedforward_torque_limit_nm=config.return_torque_limit_nm,
        torque_limit_nm=config.return_torque_limit_nm,
    )
    admittance = SecondOrderAdmittance(
        config.admittance_mass_kg,
        config.admittance_damping_ns_m,
        config.admittance_stiffness_n_m,
    )
    machine = ForceTrackingStateMachine(config)
    return_target_rad = command_config.position_min_rad
    now_s = clock()
    machine.begin_approach(now_s, "DM 已使能")
    approach = _approach_trajectory(config, command_config, kinematics, feedback.position_rad)
    approach_started_s = now_s
    transition: ContactTransition | None = None
    contact_reference_rad = feedback.position_rad
    tracking_started_s: float | None = None
    return_trajectory: ClosureTrajectory | None = None
    return_started_s: float | None = None
    return_deadline_s: float | None = None
    last_control_s = now_s
    last_observed_tactile_s: float | None = None
    last_command = _hold_command(kinematics, command_config, feedback)
    previous_state = machine.state
    emit({"event": "state", "state": machine.state.value, "reason": machine.reason})
    fieldnames = [
        "host_monotonic_s",
        "state",
        "packet_counter",
        "left_fz_n",
        "right_fz_n",
        "raw_left_fz_n",
        "raw_right_fz_n",
        "measured_force_n",
        "target_force_n",
        "position_rad",
        "velocity_rad_s",
        "torque_nm",
        "q_des_rad",
        "dq_des_rad_s",
        "kp",
        "kd",
        "tau_ff_nm",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        while machine.state not in {ForceTrackingState.COMPLETE, ForceTrackingState.FAULT}:
            now_s = clock()
            tactile_snapshot = tactile.latest()
            _validate_tactile_freshness(tactile_snapshot, now_s, config.tactile_timeout_s)
            state_before_observation = machine.state
            if tactile_snapshot.received_at_s != last_observed_tactile_s:
                machine.observe_forces(
                    tactile_snapshot.left_force_n,
                    tactile_snapshot.right_force_n,
                    tactile_snapshot.received_at_s,
                )
                last_observed_tactile_s = tactile_snapshot.received_at_s
            if machine.state is ForceTrackingState.FAULT:
                raise RuntimeError(machine.reason)
            if machine.state is not previous_state:
                emit({"event": "state", "state": machine.state.value, "reason": machine.reason})
                previous_state = machine.state
            if (
                machine.state is ForceTrackingState.CONTACT_TRANSITION
                and state_before_observation is ForceTrackingState.APPROACH
            ):
                contact_reference_rad = feedback.position_rad
                transition = ContactTransition(
                    last_command.velocity_rad_s, config.contact_transition_s
                )
            elif machine.state is ForceTrackingState.APPROACH and state_before_observation in {
                ForceTrackingState.CONTACT_TRANSITION,
                ForceTrackingState.FORCE_TRACKING,
            }:
                approach = _approach_trajectory(
                    config, command_config, kinematics, feedback.position_rad
                )
                approach_started_s = now_s
                transition = None
                admittance.reset()
            if machine.state is ForceTrackingState.CONTACT_TRANSITION:
                assert transition is not None
                if now_s - machine.entered_at_s >= transition.duration_s:
                    machine.finish_contact_transition(now_s)
                    contact_reference_rad = feedback.position_rad
                    admittance.reset()
                    last_control_s = now_s
                    if tracking_started_s is None:
                        tracking_started_s = now_s
                    emit({"event": "state", "state": machine.state.value, "reason": machine.reason})
                    previous_state = machine.state
            if (
                machine.state is ForceTrackingState.FORCE_TRACKING
                and tracking_started_s is not None
                and now_s - tracking_started_s >= config.tracking_duration_s
            ):
                machine.begin_return(now_s, "力跟踪时长完成")
                return_trajectory = ClosureTrajectory.from_limits(
                    kinematics.closure(feedback.position_rad),
                    kinematics.closure(return_target_rad),
                    config.return_closure_velocity_m_s,
                    config.return_closure_acceleration_m_s2,
                    config.return_closure_jerk_m_s3,
                )
                return_started_s = now_s
                return_deadline_s = now_s + max(
                    config.return_timeout_s,
                    return_trajectory.duration_s + config.return_settle_timeout_s,
                )
                emit({"event": "state", "state": machine.state.value, "reason": machine.reason})
                emit(
                    {
                        "event": "return_plan",
                        "target_position_rad": return_target_rad,
                        "trajectory_duration_s": return_trajectory.duration_s,
                        "total_timeout_s": return_deadline_s - now_s,
                    }
                )
                previous_state = machine.state
            command = _command_for_state(
                config,
                machine,
                kinematics,
                command_config,
                return_command_config,
                admittance,
                feedback,
                tactile_snapshot,
                approach,
                approach_started_s,
                transition,
                contact_reference_rad,
                return_trajectory,
                return_started_s,
                now_s,
                last_control_s,
            )
            feedback = dm.command(command)
            last_command = command
            last_control_s = now_s
            writer.writerow(
                {
                    "host_monotonic_s": now_s,
                    "state": machine.state.value,
                    "packet_counter": tactile_snapshot.packet_counter,
                    "left_fz_n": tactile_snapshot.left_force_n,
                    "right_fz_n": tactile_snapshot.right_force_n,
                    "raw_left_fz_n": tactile_snapshot.raw_left_fz_n,
                    "raw_right_fz_n": tactile_snapshot.raw_right_fz_n,
                    "measured_force_n": 0.5
                    * (tactile_snapshot.left_force_n + tactile_snapshot.right_force_n),
                    "target_force_n": config.target_force_n,
                    "position_rad": feedback.position_rad,
                    "velocity_rad_s": feedback.velocity_rad_s,
                    "torque_nm": feedback.torque_nm,
                    "q_des_rad": command.position_rad,
                    "dq_des_rad_s": command.velocity_rad_s,
                    "kp": command.kp,
                    "kd": command.kd,
                    "tau_ff_nm": command.feedforward_torque_nm,
                }
            )
            if machine.state is ForceTrackingState.APPROACH and (
                now_s - approach_started_s > approach.duration_s + config.approach_endpoint_hold_s
            ):
                machine.fault(now_s, "预接触轨迹到达闭合端后仍未检测到双侧接触")
                raise RuntimeError(machine.reason)
            if machine.state is ForceTrackingState.RETURN:
                assert (
                    return_trajectory is not None
                    and return_started_s is not None
                    and return_deadline_s is not None
                )
                return_elapsed_s = now_s - return_started_s
                if now_s > return_deadline_s:
                    position_error_rad = return_target_rad - feedback.position_rad
                    raise RuntimeError(
                        "回位轨迹执行超时："
                        f"规划时长={return_trajectory.duration_s:.3f}s，"
                        f"实际位置误差={position_error_rad:+.4f}rad"
                    )
                if (
                    return_elapsed_s >= return_trajectory.duration_s
                    and abs(return_target_rad - feedback.position_rad)
                    <= config.return_position_tolerance_rad
                ):
                    machine.complete(now_s)
                    emit({"event": "state", "state": machine.state.value, "reason": machine.reason})
            period_s = 1.0 / config.control_rate_hz
            sleep(max(0.0, period_s - (clock() - now_s)))
    return feedback, machine.state


def _approach_trajectory(
    config: ForceDemoConfig,
    command_config: MITCommandConfig,
    kinematics: CrankSliderKinematics,
    start_position_rad: float,
) -> ClosureTrajectory:
    """从当前反馈位置规划到闭合机械端点的闭合量轨迹。"""
    goal = (
        command_config.position_max_rad
        if command_config.closing_direction > 0
        else command_config.position_min_rad
    )
    return ClosureTrajectory.from_limits(
        kinematics.closure(start_position_rad),
        kinematics.closure(goal),
        config.approach_closure_velocity_m_s,
        config.approach_closure_acceleration_m_s2,
        config.approach_closure_jerk_m_s3,
    )


def _command_for_state(
    config: ForceDemoConfig,
    machine: ForceTrackingStateMachine,
    kinematics: CrankSliderKinematics,
    command_config: MITCommandConfig,
    return_command_config: MITCommandConfig,
    admittance: SecondOrderAdmittance,
    feedback: MotorFeedback,
    tactile: TactileSnapshot,
    approach: ClosureTrajectory,
    approach_started_s: float,
    transition: ContactTransition | None,
    contact_reference_rad: float,
    return_trajectory: ClosureTrajectory | None,
    return_started_s: float | None,
    now_s: float,
    last_control_s: float,
) -> MITCommand:
    """根据当前有限状态生成一个 MIT 命令。"""
    if machine.state is ForceTrackingState.APPROACH:
        closure_m, closure_velocity_m_s, _ = approach.sample(now_s - approach_started_s)
        return _closure_trajectory_command(
            kinematics,
            command_config,
            feedback,
            closure_m,
            closure_velocity_m_s,
            config.approach_feedforward_force_n,
            config.approach_feedforward_ratio,
        )
    if machine.state is ForceTrackingState.CONTACT_TRANSITION:
        assert transition is not None
        velocity_rad_s = transition.velocity_at(now_s - machine.entered_at_s)
        return _trajectory_command(
            kinematics,
            command_config,
            feedback,
            contact_reference_rad,
            velocity_rad_s,
            0.0,
            0.0,
        )
    if machine.state is ForceTrackingState.FORCE_TRACKING:
        dt_s = min(max(now_s - last_control_s, 1e-6), 0.02)
        return step_admittance(
            admittance,
            kinematics,
            command_config,
            reference_position_rad=contact_reference_rad,
            measured_position_rad=feedback.position_rad,
            measured_velocity_rad_s=feedback.velocity_rad_s,
            left_force_n=tactile.left_force_n,
            right_force_n=tactile.right_force_n,
            target_force_n=config.target_force_n,
            dt_s=dt_s,
        )
    if machine.state is ForceTrackingState.RETURN:
        assert return_trajectory is not None and return_started_s is not None
        closure_m, closure_velocity_m_s, _ = return_trajectory.sample(now_s - return_started_s)
        return _closure_trajectory_command(
            kinematics,
            return_command_config,
            feedback,
            closure_m,
            closure_velocity_m_s,
            0.0,
            0.0,
            torque_limit_nm=config.return_torque_limit_nm,
        )
    raise RuntimeError(f"状态 {machine.state.value} 不应生成 MIT 命令")


def _closure_trajectory_command(
    kinematics: CrankSliderKinematics,
    config: MITCommandConfig,
    feedback: MotorFeedback,
    closure_m: float,
    closure_velocity_m_s: float,
    feedforward_force_n: float,
    feedforward_ratio: float,
    *,
    torque_limit_nm: float | None = None,
) -> MITCommand:
    """把闭合量轨迹采样通过运动学逆解转换为 MIT 关节命令。"""
    position_rad = kinematics.position_for_closure(
        closure_m,
        config.position_min_rad,
        config.position_max_rad,
    )
    velocity_rad_s = closure_velocity_m_s / kinematics.closure_jacobian(position_rad)
    return _trajectory_command(
        kinematics,
        config,
        feedback,
        position_rad,
        velocity_rad_s,
        feedforward_force_n,
        feedforward_ratio,
        torque_limit_nm=torque_limit_nm,
    )


def _trajectory_command(
    kinematics: CrankSliderKinematics,
    config: MITCommandConfig,
    feedback: MotorFeedback,
    position_rad: float,
    velocity_rad_s: float,
    feedforward_force_n: float,
    feedforward_ratio: float,
    *,
    torque_limit_nm: float | None = None,
) -> MITCommand:
    """把规划的关节位置与速度映射为受限 MIT 命令。"""
    jacobian = kinematics.closure_jacobian(position_rad)
    return build_mit_command(
        kinematics,
        config,
        reference_position_rad=position_rad,
        displacement_m=0.0,
        velocity_m_s=config.closing_direction * velocity_rad_s * jacobian,
        measured_position_rad=feedback.position_rad,
        measured_velocity_rad_s=feedback.velocity_rad_s,
        feedforward_force_n=feedforward_force_n,
        feedforward_ratio=feedforward_ratio,
        torque_limit_nm=torque_limit_nm,
    )


def _hold_command(
    kinematics: CrankSliderKinematics,
    config: MITCommandConfig,
    feedback: MotorFeedback,
) -> MITCommand:
    """构造当前位置零速度保持命令。"""
    return _trajectory_command(kinematics, config, feedback, feedback.position_rad, 0.0, 0.0, 0.0)


def _validate_tactile_freshness(snapshot: TactileSnapshot, now_s: float, timeout_s: float) -> None:
    """拒绝过期或非有限触觉输入。"""
    values = (snapshot.left_force_n, snapshot.right_force_n, snapshot.received_at_s, now_s)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("触觉快照包含非有限数值")
    age_s = now_s - snapshot.received_at_s
    if age_s < 0.0 or age_s > timeout_s:
        raise RuntimeError(f"触觉快照过期：age={age_s:.3f}s")


def config_record(config: ForceDemoConfig) -> dict[str, object]:
    """返回便于 CLI 输出的配置字典。"""
    return asdict(config)

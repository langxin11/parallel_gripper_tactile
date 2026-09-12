"""DMgripper 手托杯接触、撤手增力和人工释放运行时。"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import asdict, replace
from pathlib import Path

from dm_grasp_core import (
    ContactTransition,
    CrankSliderKinematics,
    MITCommandConfig,
    within_zero_window,
)
from dmgripper_hardware import STATUS_DISABLED
from papillarray_hardware import PapillArraySerialConfig

from .cup_config import CupConfig
from .cup_control import CupForceController
from .cup_flow import CupFlow
from .cup_plot import plot_cup_run
from .cup_recording import CupRecorder
from .runtime import (
    _DmSession,
    _approach_trajectory,
    _closure_trajectory_command,
    _hold_command,
    _trajectory_command,
    _validate_tactile_freshness,
)
from .tactile import TactileWorker
from .trajectory import ClosureTrajectory

_MESSAGES = {
    "place_cup": "空载验证通过。手托空杯放入夹爪，输入 ready 开始闭合。",
    "approach": "正在建立双侧接触，请继续托住杯子。",
    "contact_transition": "双侧接触已确认，正在减速。",
    "supported_hold": "正在建立初始抓力，请继续托住杯子。",
    "takeover": "增力已启用，可以缓慢撤手；完全撤手后输入 ready。",
    "stabilizing": "正在等待撤手后的抓握稳定，请暂勿倒水。",
    "pour": "抓握已稳定，可以缓慢倒水。",
    "await_release": "倒水计时结束，仍保持抓握；托住杯子后输入 release。",
    "return": "已收到 release，正在张开回位，请托住杯子。",
    "complete": "回位完成。",
}


def _raw_axes(snapshot):
    """取得完整双侧三轴力，缺失或非有限数据立即拒绝。"""
    values = tuple(
        getattr(snapshot, f"raw_{side}_f{axis}_n") for side in ("left", "right") for axis in "xyz"
    )
    if any(value is None or not math.isfinite(value) for value in values):
        raise RuntimeError("触觉三轴力缺失或包含非有限数值")
    return values


def _validate_snapshot(snapshot, now_s, config):
    """原始力用于保护，滤波法向力用于控制。"""
    _validate_tactile_freshness(snapshot, now_s, config.tactile_timeout_s)
    axes = _raw_axes(snapshot)
    if max(abs(axes[2]), abs(axes[5])) > config.force_ceiling_n:
        raise RuntimeError("原始法向力超过保护上限")
    if abs(axes[2] - axes[5]) > config.max_force_imbalance_n:
        raise RuntimeError("双侧原始法向力不平衡超过保护上限")
    return axes


def _verify_zero(tactile, config, clear_bias, clock, sleep):
    """按 ROS 2 语义验证滤波双侧法向力的连续空载窗口。"""
    sample = tactile.wait_for_update(None, config.tactile_startup_timeout_s)
    if clear_bias:
        sleep(config.tactile_bias_settle_s)
        sample = tactile.wait_for_update(sample.received_at_s, config.tactile_timeout_s)
    deadline, stable, previous_timestamp = clock() + config.zero_force_timeout_s, None, None
    last_left_force_n = sample.left_force_n
    last_right_force_n = sample.right_force_n
    best_stable_s = 0.0
    while clock() < deadline:
        _validate_snapshot(sample, clock(), config)
        if previous_timestamp is not None and sample.timestamp_us <= previous_timestamp:
            raise RuntimeError("空载验证期间触觉设备时间未递增")
        previous_timestamp = sample.timestamp_us
        last_left_force_n = sample.left_force_n
        last_right_force_n = sample.right_force_n
        if within_zero_window(
            last_left_force_n,
            last_right_force_n,
            config.zero_force_threshold_n,
        ):
            stable = sample.received_at_s if stable is None else stable
            best_stable_s = max(best_stable_s, sample.received_at_s - stable)
            if sample.received_at_s - stable >= config.zero_force_stable_s:
                return
        else:
            stable = None
        remaining_s = deadline - clock()
        if remaining_s <= 0.0:
            break
        try:
            sample = tactile.wait_for_update(
                sample.received_at_s,
                min(config.tactile_timeout_s, remaining_s),
            )
        except TimeoutError:
            if clock() >= deadline:
                break
            raise
    raise RuntimeError(
        "使能前滤波双侧 Fz 零力验证超时："
        f"阈值=±{config.zero_force_threshold_n:.3f}N，"
        f"末值 LEFT={last_left_force_n:+.3f}N、RIGHT={last_right_force_n:+.3f}N，"
        f"最长稳定={best_stable_s:.3f}/{config.zero_force_stable_s:.3f}s；"
        "请保持传感器空载"
    )


def run_cup(
    config: CupConfig,
    *,
    output_directory: Path,
    clear_bias: bool,
    action_source: Callable[[], str | None],
    event_sink=None,
    clock=time.monotonic,
    sleep=time.sleep,
    session_factory=_DmSession,
    tactile_factory=TactileWorker,
) -> dict[str, object]:
    """执行一次交互倒水实验；异常失能，正常结束必须先显式释放。"""
    recorder = CupRecorder(output_directory, {**asdict(config), "clear_bias": clear_bias})
    started = clock()

    def emit(event):
        """为事件附加与控制 trace 共用的相对时间。"""
        event = {"time_s": clock() - started, **event}
        recorder.append(event)
        if event_sink is not None:
            event_sink(event)

    tactile = tactile_factory(
        PapillArraySerialConfig(
            port=config.tactile_port,
            sampling_rate=500,
            expected_sensors=2,
            timeout_s=0.05,
            packet_timeout_s=config.control.tactile_timeout_s,
        ),
        clear_bias=clear_bias,
        clock=clock,
        cutoff_hz=config.control.tactile_cutoff_hz,
        filter_reset_gap_s=config.control.tactile_filter_reset_gap_s,
        sample_sink=recorder.sample,
    )
    dm = session_factory(config.dm_port, timeout_s=0.05)
    enabled, opened = False, False
    failure = None
    try:
        tactile.start()
        emit(
            {
                "event": "state",
                "state": "zero_check",
                "message": "请保持传感器空载，正在验证滤波双侧 Fz 零力。",
            }
        )
        _verify_zero(tactile, config.control, clear_bias, clock, sleep)
        emit({"event": "state", "state": "place_cup", "message": _MESSAGES["place_cup"]})
        while True:
            _validate_snapshot(tactile.latest(), clock(), config.control)
            action = action_source()
            if action == "__input_closed__":
                raise RuntimeError("交互输入已关闭")
            if action == "ready":
                break
            if action == "release":
                raise RuntimeError("用户在使能前取消实验")
            if action is not None:
                emit({"event": "status", "state": "place_cup", "message": _MESSAGES["place_cup"]})
            sleep(1 / config.control.control_rate_hz)
        opened = True
        dm.open()
        feedback = dm.inspect()
        if feedback.status_code != STATUS_DISABLED:
            raise RuntimeError("开始前电机必须处于失能状态")
        # 使能报文可能已生效但确认丢失，必须仍尝试失能。
        enabled = True
        feedback = dm.enable()
        _run_enabled(
            config, dm, tactile, feedback, action_source, recorder, emit, clock, sleep, started
        )
    except BaseException as error:
        failure = error
    finally:
        cleanup_errors = []
        if enabled:
            try:
                dm.disable()
                emit({"event": "disabled", "confirmed": True})
            except BaseException as error:
                cleanup_errors.append(f"失能失败：{error}")
        if opened:
            try:
                dm.close()
            except BaseException as error:
                cleanup_errors.append(f"DM 串口关闭失败：{error}")
        try:
            tactile.stop()
        except BaseException as error:
            cleanup_errors.append(f"触觉采集退出失败：{error}")
        if cleanup_errors:
            failure = RuntimeError(f"原始错误：{failure}；" + "；".join(cleanup_errors))
        if failure is not None:
            try:
                recorder.append(
                    {"time_s": clock() - started, "event": "fault", "error": str(failure)}
                )
            finally:
                recorder.close(status="failed", error=failure)
        else:
            recorder.close()
    if failure is not None:
        raise failure
    # 绘图发生在设备失能、串口和记录文件关闭之后。
    plots = plot_cup_run(output_directory)
    return {
        "completed": True,
        "disable_confirmed": True,
        "output_directory": str(output_directory),
        "plots": [str(path) for path in plots],
    }


def _run_enabled(
    config, dm, tactile, feedback, action_source, recorder, emit, clock, sleep, started
):
    """使用真实单调时钟驱动阶段机，不对丢失的控制周期补算。"""
    c = config.control
    kinematics = CrankSliderKinematics(
        theta0_rad=math.pi / 4,
        crank_radius_m=0.03,
        link_length_m=0.04,
        offset_m=0.021213203435596423,
    )
    command_config = MITCommandConfig(
        position_min_rad=dm.deployment.joint_position_min_rad,
        position_max_rad=dm.deployment.joint_position_max_rad,
        velocity_limit_rad_s=c.velocity_limit_rad_s,
        closing_direction=dm.deployment.closing_direction,
        kp=c.mit_kp,
        kd=c.mit_kd,
        feedforward_ratio=0.0,
        feedforward_torque_limit_nm=c.torque_limit_nm,
        torque_limit_nm=c.torque_limit_nm,
    )
    return_config = replace(
        command_config,
        kp=c.return_mit_kp,
        kd=c.return_mit_kd,
        feedforward_torque_limit_nm=c.return_torque_limit_nm,
        torque_limit_nm=c.return_torque_limit_nm,
    )
    controller = CupForceController(config, kinematics, command_config)
    flow = CupFlow(config)
    phase, previous_phase = "approach", None
    phase_started = last_control = clock()
    period = 1 / c.control_rate_hz
    last_sample = None
    contact_since = lost_since = None
    transition = None
    reference_position = feedback.position_rad
    trajectory = _approach_trajectory(c, command_config, kinematics, feedback.position_rad)
    last_command = _hold_command(kinematics, command_config, feedback)
    target = c.target_force_n
    policy_command = None
    while phase != "complete":
        now = clock()
        dt = now - last_control if last_sample is not None else period
        if not 0 < dt <= config.max_control_gap_s:
            raise RuntimeError(f"控制周期超时：{dt:.4f}s")
        sample = tactile.latest()
        axes = _validate_snapshot(sample, now, c)
        new_sample = last_sample is None or sample.received_at_s != last_sample.received_at_s
        if (
            new_sample
            and last_sample is not None
            and sample.timestamp_us <= last_sample.timestamp_us
        ):
            raise RuntimeError("触觉设备时间未递增或设备发生重启")
        action = action_source()
        if action == "__input_closed__":
            raise RuntimeError("交互输入已关闭")
        if action == "release" and phase != "return":
            phase, phase_started = "return", now
            trajectory = ClosureTrajectory.from_limits(
                kinematics.closure(feedback.position_rad),
                kinematics.closure(command_config.position_min_rad),
                c.return_closure_velocity_m_s,
                c.return_closure_acceleration_m_s2,
                c.return_closure_jerk_m_s3,
            )
        elif action == "ready" and phase == "takeover":
            flow.action(action)
            phase = flow.phase
        elif action is not None:
            emit(
                {
                    "event": "status",
                    "state": phase,
                    "action": action,
                    "target_force_n": target,
                    "message": _MESSAGES[phase],
                }
            )
        if new_sample:
            if phase == "approach":
                if min(sample.left_force_n, sample.right_force_n) >= c.contact_on_n:
                    contact_since = sample.received_at_s if contact_since is None else contact_since
                    if sample.received_at_s - contact_since >= c.contact_on_stable_s:
                        phase, phase_started = "contact_transition", now
                        reference_position = feedback.position_rad
                        transition = ContactTransition(
                            last_command.velocity_rad_s, c.contact_transition_s
                        )
                else:
                    contact_since = None
            elif phase != "return":
                if min(sample.left_force_n, sample.right_force_n) <= c.contact_off_n:
                    lost_since = sample.received_at_s if lost_since is None else lost_since
                    if sample.received_at_s - lost_since >= c.contact_off_stable_s:
                        raise RuntimeError("已接触后至少一侧持续失去接触")
                else:
                    lost_since = None
            if phase not in {"approach", "contact_transition", "return"}:
                policy_dt = (
                    period
                    if last_sample is None
                    else sample.received_at_s - last_sample.received_at_s
                )
                policy_command = flow.update(
                    time_s=now - started,
                    dt_s=policy_dt,
                    left_normal_n=sample.left_force_n,
                    right_normal_n=sample.right_force_n,
                    tangential_force_n=math.hypot(*axes[:2]) + math.hypot(*axes[3:5]),
                    signed_tangential_force_n=axes[1],
                    closure_m=kinematics.closure(feedback.position_rad),
                )
                phase, target = flow.phase, policy_command.target_force_n
            last_sample = sample
        if phase == "contact_transition" and now - phase_started >= c.contact_transition_s:
            phase = "supported_hold"
            controller.reset(feedback.position_rad)
        if phase != previous_phase:
            emit({"event": "state", "state": phase, "message": _MESSAGES[phase]})
            previous_phase = phase
        if phase in {"approach", "return"}:
            closure, velocity, _ = trajectory.sample(now - phase_started)
            command = _closure_trajectory_command(
                kinematics,
                return_config if phase == "return" else command_config,
                feedback,
                closure,
                velocity,
                c.approach_feedforward_force_n if phase == "approach" else 0.0,
                c.approach_feedforward_ratio if phase == "approach" else 0.0,
            )
        elif phase == "contact_transition":
            command = _trajectory_command(
                kinematics,
                command_config,
                feedback,
                reference_position,
                transition.velocity_at(now - phase_started),
                0.0,
                0.0,
            )
        else:
            command = controller.step(
                feedback=feedback,
                tactile=sample,
                target_force_n=target,
                time_s=now - started,
                dt_s=dt,
            )
        # 交互输出／存储也可能延迟；发送前再次检查时限和输入新鲜度。
        if clock() - last_control > config.max_control_gap_s:
            raise RuntimeError("发送前控制计算或事件记录超时")
        _validate_snapshot(sample, clock(), c)
        send_started = clock()
        feedback = dm.command(command)
        latency = clock() - send_started
        row = {
            f"raw_{side}_f{axis}_n": value
            for (side, axis), value in zip(
                ((side, axis) for side in ("left", "right") for axis in "xyz"), axes, strict=True
            )
        }
        row.update(
            time_s=now - started,
            state=phase,
            tactile_received_at_s=sample.received_at_s,
            tactile_timestamp_us=sample.timestamp_us,
            packet_counter=sample.packet_counter,
            left_fz_n=sample.left_force_n,
            right_fz_n=sample.right_force_n,
            measured_force_n=(sample.left_force_n + sample.right_force_n) / 2,
            target_force_n=target,
            measured_tangential_force_n=policy_command.measured_tangential_force_n
            if policy_command
            else None,
            trigger_active=policy_command.trigger_active if policy_command else False,
            force_limited=target >= config.grip.max_target_force_n - 1e-9,
            force_deadband_active=(
                controller.force_deadband_active
                if config.controller == "admittance"
                and phase not in {"approach", "contact_transition", "return"}
                else False
            ),
            unloading_blocked=(
                controller.unloading_blocked
                if config.controller == "admittance"
                and phase not in {"approach", "contact_transition", "return"}
                else False
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
            tactile_age_s=send_started - sample.received_at_s,
            command_latency_s=latency,
        )
        recorder.write(row)
        if clock() - last_control > config.max_control_gap_s:
            raise RuntimeError("命令反馈或控制记录超时")
        if (
            phase == "approach"
            and now - phase_started > trajectory.duration_s + c.approach_endpoint_hold_s
        ):
            raise RuntimeError("预接触轨迹到达闭合端后仍未检测到双侧接触")
        if phase == "return":
            if now - phase_started > max(
                c.return_timeout_s, trajectory.duration_s + c.return_settle_timeout_s
            ):
                raise RuntimeError("回位轨迹执行超时")
            if (
                now - phase_started >= trajectory.duration_s
                and abs(feedback.position_rad - command_config.position_min_rad)
                <= c.return_position_tolerance_rad
            ):
                phase = "complete"
                emit({"event": "state", "state": phase, "message": _MESSAGES[phase]})
        last_control, last_command = now, command
        sleep(max(0.0, period - (clock() - now)))

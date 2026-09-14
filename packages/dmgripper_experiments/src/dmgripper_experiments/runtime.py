"""通用抓取实验的唯一设备运行时。

一次运行只存在这一个调度循环：它拥有生命周期、设备会话、观测配对、
目标来源、控制器调度与记录的时序。清理路径保证任一退出方式都会
终止采集、关闭已打开的设备与文件；失败产物保留原始故障与清理故障。
"""

from __future__ import annotations

import math
import statistics
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import NoReturn

from papillarray_hardware import PapillArraySerialConfig

from dm_grasp_core import (
    ContactTransition,
    CrankSliderKinematics,
    ForceReferenceCurve,
    ForceWaypoint,
    MITCommand,
    MITCommandConfig,
    StiffnessSnapshot,
    build_mit_command,
)

from .config import ExperimentConfig
from .control import GripController
from .lifecycle import Lifecycle, LifecyclePhase
from .observation import (
    PairedObservation,
    StiffnessDiagnostics,
    pair_observation,
    raw_axes,
)
from .plotting import plot_experiment_run
from .recording import ExperimentRecorder
from .session import DmSession
from .tactile import TactileSnapshot, TactileWorker
from .targets import ForceTarget, TargetSource, build_target_source
from .terminal import RunSnapshot, TerminalDisplay, format_event_line
from .trajectory import ClosureTrajectory

EventSink = Callable[[dict[str, object]], None]

_KINEMATICS = CrankSliderKinematics(
    theta0_rad=math.pi / 4,
    crank_radius_m=0.03,
    link_length_m=0.04,
    offset_m=0.021213203435596423,
)

_PHASE_MESSAGES = {
    LifecyclePhase.PREPARING: "正在预检：建立采集并验证空载零力。",
    LifecyclePhase.READY: (
        "预检完成。电机未使能；输入 start 后可能先自动回零，再开始闭合；输入 release 取消。"
    ),
    LifecyclePhase.HOMING: "启动位置不在 home 附近，正在执行受限自动回零。",
    LifecyclePhase.APPROACH: "正在受限闭合接近，等待双侧接触。",
    LifecyclePhase.CONTACT_TRANSITION: "双侧接触已确认，正在平滑衰减接近速度。",
    LifecyclePhase.PRELOAD: "正在建立初始抓力并学习基线。",
    LifecyclePhase.ACTIVE: "目标策略已启用，运行中。",
    LifecyclePhase.HOLDING: "任务计时完成，保持抓握；输入 release 后按 Enter 结束。",
    LifecyclePhase.FAULT_HOLDING: (
        "实验已失败，电机仍受控保持。请先承接物体，再输入 release 后按 Enter。"
    ),
    LifecyclePhase.RETURNING: "已收到 release，正在受限张开回位。",
    LifecyclePhase.COMPLETED: "回位完成，实验结束。",
    LifecyclePhase.CANCELLED: "使能前取消，未发送运动命令。",
    LifecyclePhase.FAULT: "发生故障，已执行退出清理。",
}


@dataclass
class _DeviceState:
    """跨阶段传递的设备占用与使能登记。"""

    opened: bool = False
    # 发送使能报文前即置位：确认丢失时仍要尽力失能。
    enable_attempted: bool = False
    enable_confirmed: bool = False
    phase: str | None = None


class HoldableExperimentFault(RuntimeError):
    """电机通道仍健康时应转入故障保持的实验级故障。"""


class FatalHardwareFault(RuntimeError):
    """不能继续声称安全保持的底层设备故障。"""

    def __init__(
        self,
        message: str,
        *,
        phase: str | None = None,
        resolution: str = "forced_disable",
        holding_entered: bool = False,
        holding_duration_s: float = 0.0,
        primary_error: BaseException | None = None,
    ) -> None:
        """保存故障发生阶段。"""
        super().__init__(message)
        self.phase = phase
        self.resolution = resolution
        self.holding_entered = holding_entered
        self.holding_duration_s = holding_duration_s
        self.primary_error = primary_error


def run_experiment(
    config: ExperimentConfig,
    *,
    output_directory: Path,
    clear_bias: bool,
    action_source: Callable[[], str | None],
    event_sink: EventSink | None = None,
    terminal: TerminalDisplay | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    session_factory: type[DmSession] = DmSession,
    tactile_factory: type[TactileWorker] = TactileWorker,
    input_config_path: Path | None = None,
) -> dict[str, object]:
    """执行一次通用抓取实验；异常失能，正常结束必须显式释放或配置回位。

    Args:
        config: 通过全部校验的冻结实验配置。
        output_directory: 尚不存在的独占输出目录。
        clear_bias: 是否在采集启动时请求触觉清零。
        action_source: 非阻塞操作事件来源（start／status／release）。
        event_sink: 结构化事件接收器；``None`` 时只写记录。
        terminal: 可选终端展示；``None`` 时不显示。
        clock: 单调时钟，测试可注入假时钟。
        sleep: 睡眠函数，测试可注入假实现。
        session_factory: DM 会话工厂，测试可注入假设备。
        tactile_factory: 触觉采集工厂，测试可注入假流。
        input_config_path: 原始 YAML 输入路径，写入 manifest。

    Returns:
        运行结果摘要（最终状态、失能确认、输出目录与图路径）。

    Raises:
        BaseException: 运行失败时在完成清理后重新抛出原始错误。
    """
    recorder = ExperimentRecorder(output_directory, config, input_config_path=input_config_path)
    started = clock()
    devices = _DeviceState()

    def emit(event: dict[str, object]) -> None:
        """为事件附加与控制 trace 共用的相对时间并落盘。"""
        stamped = {"time_s": clock() - started, **event}
        if event.get("event") == "state" and event.get("phase") is not None:
            devices.phase = str(event["phase"])
        errors: list[BaseException] = []
        try:
            recorder.append(stamped)
        except BaseException as error:  # noqa: BLE001
            errors.append(error)
        if event_sink is not None:
            try:
                event_sink(stamped)
            except BaseException as error:  # noqa: BLE001
                errors.append(error)
        if terminal is not None and stamped.get("event") in {
            "command_received",
            "disabled",
            "state",
            "status",
            "warning",
        }:
            try:
                terminal.show_event(
                    format_event_line(stamped),
                    phase=str(stamped.get("phase") or ""),
                    event=str(stamped.get("event") or ""),
                    action=str(stamped.get("action") or ""),
                )
            except BaseException as error:  # noqa: BLE001
                errors.append(error)
        if errors:
            primary_error, *secondary_errors = errors
            for secondary_error in secondary_errors:
                primary_error.add_note(f"同一事件的后续输出失败：{secondary_error}")
            raise primary_error

    tactile: TactileWorker | None = None
    dm: DmSession | None = None
    disable_confirmed: bool | str = "not_applicable"
    failure: BaseException | None = None
    outcome: dict[str, object] | None = None
    fault_phase: str | None = None
    fault_holding_entered = False
    fault_holding_duration_s = 0.0
    fault_resolution: str | None = None
    runtime_secondary_errors: list[str] = []
    try:
        tactile = tactile_factory(
            PapillArraySerialConfig(
                port=config.hardware.tactile_port,
                sampling_rate=500,
                expected_sensors=2,
                timeout_s=0.05,
                packet_timeout_s=config.timing.tactile_timeout_s,
            ),
            clear_bias=clear_bias,
            clock=clock,
            cutoff_hz=config.timing.tactile_cutoff_hz,
            filter_reset_gap_s=config.timing.tactile_filter_reset_gap_s,
            sample_sink=recorder.sample,
        )
        dm = session_factory(
            config.hardware.dm_port,
            timeout_s=0.05,
            feedback_position_margin_rad=config.hardware.feedback_position_margin_rad,
        )
        tactile.start()
        if terminal is not None:
            terminal.start()
        emit(
            {
                "event": "state",
                "phase": LifecyclePhase.PREPARING.value,
                "message": _PHASE_MESSAGES[LifecyclePhase.PREPARING],
            }
        )
        if config.lifecycle.verify_zero_force:
            _verify_zero(tactile, config, clear_bias, clock, sleep, warning_sink=emit)
        else:
            first = tactile.wait_for_update(None, config.timing.tactile_startup_timeout_s)
            _validate_snapshot(first, clock(), config)
            emit(
                {
                    "event": "warning",
                    "message": "已显式跳过双侧零力稳定验证",
                    "left_fz_n": first.left_force_n,
                    "right_fz_n": first.right_force_n,
                }
            )
        outcome = _run_ready_and_enabled(
            config,
            dm,
            tactile,
            recorder,
            emit,
            terminal,
            action_source,
            clock,
            sleep,
            started,
            devices,
        )
        if outcome.get("status") == "failed":
            primary_error = outcome.get("primary_error")
            assert isinstance(primary_error, BaseException)
            failure = primary_error
            fault_phase = str(outcome.get("fault_phase") or "") or None
            fault_holding_entered = bool(outcome.get("fault_holding_entered"))
            fault_holding_duration_s = float(outcome.get("fault_holding_duration_s") or 0.0)
            fault_resolution = str(outcome.get("fault_resolution") or "") or None
            runtime_secondary_errors = list(outcome.get("secondary_errors") or [])
    except BaseException as error:  # noqa: BLE001
        failure = error
        if isinstance(error, FatalHardwareFault):
            if error.primary_error is not None:
                failure = error.primary_error
                runtime_secondary_errors.append(str(error))
            fault_phase = error.phase
            fault_resolution = error.resolution
            fault_holding_entered = error.holding_entered
            fault_holding_duration_s = error.holding_duration_s
        elif devices.enable_attempted:
            fault_phase = devices.phase
            fault_resolution = "forced_disable"
    finally:
        had_primary_failure = failure is not None
        cleanup_errors: list[str] = list(runtime_secondary_errors)
        if devices.enable_attempted and dm is not None:
            try:
                dm.disable()
            except BaseException as error:  # noqa: BLE001
                disable_confirmed = False
                cleanup_errors.append(f"失能失败：{error}")
            else:
                disable_confirmed = True
                try:
                    emit(
                        {
                            "event": "disabled",
                            "confirmed": True,
                            "message": "电机已确认失能。",
                        }
                    )
                except BaseException as error:  # noqa: BLE001
                    cleanup_errors.append(f"失能确认事件记录失败：{error}")
        if devices.opened and dm is not None:
            try:
                dm.close()
            except BaseException as error:  # noqa: BLE001
                cleanup_errors.append(f"DM 串口关闭失败：{error}")
        if tactile is not None:
            try:
                tactile.stop()
            except BaseException as error:  # noqa: BLE001
                cleanup_errors.append(f"触觉采集退出失败：{error}")
        if terminal is not None:
            try:
                terminal.stop()
            except BaseException as error:  # noqa: BLE001
                cleanup_errors.append(f"终端显示退出失败：{error}")
        if failure is None and cleanup_errors:
            failure = RuntimeError("退出清理失败：" + "；".join(cleanup_errors))
        post_processing_error: BaseException | None = None
        plots: list[str] = []
        if failure is None and outcome is not None and outcome.get("status") != "cancelled":
            if config.output.plots:
                try:
                    generated = plot_experiment_run(Path(output_directory))
                    plots = [str(path) for path in generated]
                    recorder.register_artifacts({path.name: path.name for path in generated})
                except BaseException as error:  # noqa: BLE001
                    post_processing_error = error
        if failure is not None:
            try:
                try:
                    recorder.append(
                        {"time_s": clock() - started, "event": "fault", "error": str(failure)}
                    )
                except BaseException as error:  # noqa: BLE001
                    cleanup_errors.append(f"最终故障事件记录失败：{error}")
                recorder.close(
                    status="failed",
                    error=failure,
                    disable_confirmed=disable_confirmed,
                    cleanup_errors=cleanup_errors,
                    input_config_path=input_config_path,
                    fault_phase=fault_phase,
                    fault_holding_entered=fault_holding_entered,
                    fault_holding_duration_s=fault_holding_duration_s,
                    fault_resolution=fault_resolution,
                )
            except BaseException as error:  # noqa: BLE001
                failure.add_note(f"运行记录最终化失败：{error}")
        elif outcome is not None and outcome.get("status") == "cancelled":
            recorder.close(
                status="cancelled",
                disable_confirmed="not_applicable",
                cleanup_errors=cleanup_errors,
                input_config_path=input_config_path,
            )
        else:
            recorder.close(
                status="completed",
                disable_confirmed=disable_confirmed,
                cleanup_errors=cleanup_errors,
                post_processing_error=post_processing_error,
                input_config_path=input_config_path,
            )
    if failure is not None:
        if had_primary_failure and cleanup_errors:
            failure.add_note("警告：退出清理未完全成功：" + "；".join(cleanup_errors))
        failure.add_note(f"运行记录：{output_directory}")
        raise failure
    assert outcome is not None
    return {
        **outcome,
        "disable_confirmed": disable_confirmed,
        "output_directory": str(output_directory),
        "plots": plots,
    }


def _run_ready_and_enabled(
    config: ExperimentConfig,
    dm: DmSession,
    tactile: TactileWorker,
    recorder: ExperimentRecorder,
    emit: EventSink,
    terminal: TerminalDisplay | None,
    action_source: Callable[[], str | None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    started: float,
    devices: _DeviceState,
) -> dict[str, object]:
    """等待启动、完成使能序列并进入控制循环。"""
    lifecycle = Lifecycle()
    lifecycle.mark_ready(clock())
    emit(
        {
            "event": "state",
            "phase": LifecyclePhase.READY.value,
            "message": _PHASE_MESSAGES[LifecyclePhase.READY],
        }
    )
    period = 1.0 / config.timing.control_rate_hz
    while True:
        _validate_snapshot(tactile.latest(), clock(), config)
        action = action_source()
        if action == "release":
            lifecycle.cancel_before_enable(clock())
            emit(
                {
                    "event": "state",
                    "phase": LifecyclePhase.CANCELLED.value,
                    "message": _PHASE_MESSAGES[LifecyclePhase.CANCELLED],
                }
            )
            return {"status": "cancelled", "final_phase": LifecyclePhase.CANCELLED.value}
        if action == "start":
            emit(
                {
                    "event": "command_received",
                    "phase": LifecyclePhase.READY.value,
                    "action": "start",
                    "message": "已收到 start，正在连接、检查并使能电机；请勿重复输入。",
                }
            )
            break
        if config.lifecycle.auto_start:
            emit(
                {
                    "event": "command_received",
                    "phase": LifecyclePhase.READY.value,
                    "action": "auto_start",
                    "message": "已按配置自动启动，正在连接、检查并使能电机。",
                }
            )
            break
        if action == "status":
            emit(
                {
                    "event": "status",
                    "phase": LifecyclePhase.READY.value,
                    "action": action,
                    "message": _PHASE_MESSAGES[LifecyclePhase.READY],
                }
            )
        elif action is not None:
            emit(
                {
                    "event": "warning",
                    "code": "unknown_command",
                    "phase": LifecyclePhase.READY.value,
                    "action": action,
                    "message": (
                        f"无法识别命令 {action!r}；可用命令为 start、status、release，"
                        "输入后按 Enter。"
                    ),
                }
            )
        sleep(period)
    dm.open()
    devices.opened = True
    dm.inspect()
    dm.require_disabled()
    # 使能报文可能已生效但确认丢失，发送前即登记，清理阶段仍会尽力失能。
    devices.enable_attempted = True
    feedback = dm.enable()
    devices.enable_confirmed = True
    # 只有确认成功才发送接近命令；ready 等待时间不得计入首个控制 dt。
    now = clock()
    needs_homing = (
        abs(feedback.position_rad - config.hardware.home_position_rad)
        > config.hardware.home_tolerance_rad
    )
    lifecycle.start(now, needs_homing=needs_homing)
    return _run_control_loop(
        config,
        dm,
        tactile,
        recorder,
        emit,
        terminal,
        action_source,
        clock,
        sleep,
        started,
        lifecycle,
        feedback,
    )


def _run_control_loop(
    config: ExperimentConfig,
    dm: DmSession,
    tactile: TactileWorker,
    recorder: ExperimentRecorder,
    emit: EventSink,
    terminal: TerminalDisplay | None,
    action_source: Callable[[], str | None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    started: float,
    lifecycle: Lifecycle,
    feedback,
) -> dict[str, object]:
    """运行使能后状态机，并把实验级故障收敛到 DM-only 故障保持。"""
    feedback_state = [feedback]
    try:
        emit(
            {
                "event": "state",
                "phase": lifecycle.phase.value,
                "message": _PHASE_MESSAGES[lifecycle.phase],
                "position_rad": feedback.position_rad,
                "home_position_rad": config.hardware.home_position_rad,
                "home_tolerance_rad": config.hardware.home_tolerance_rad,
            }
        )
        return _run_control_loop_inner(
            config,
            dm,
            tactile,
            recorder,
            emit,
            terminal,
            action_source,
            clock,
            sleep,
            started,
            lifecycle,
            feedback,
            feedback_state,
        )
    except FatalHardwareFault:
        raise
    except KeyboardInterrupt:
        raise
    except Exception as error:
        return _run_fault_holding(
            config,
            dm,
            recorder,
            emit,
            action_source,
            clock,
            sleep,
            started,
            lifecycle,
            feedback_state[0],
            error,
        )


def _run_control_loop_inner(
    config: ExperimentConfig,
    dm: DmSession,
    tactile: TactileWorker,
    recorder: ExperimentRecorder,
    emit: EventSink,
    terminal: TerminalDisplay | None,
    action_source: Callable[[], str | None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    started: float,
    lifecycle: Lifecycle,
    feedback,
    feedback_state: list[object],
) -> dict[str, object]:
    """使用真实单调时钟驱动唯一控制循环，不对丢失周期补算。"""
    timing = config.timing
    lifecycle_config = config.lifecycle
    kinematics = _KINEMATICS
    command_config = MITCommandConfig(
        position_min_rad=dm.deployment.joint_position_min_rad,
        position_max_rad=dm.deployment.joint_position_max_rad,
        velocity_limit_rad_s=config.controller.velocity_limit_rad_s,
        closing_direction=dm.deployment.closing_direction,
        kp=config.controller.mit_kp,
        kd=config.controller.mit_kd,
        feedforward_ratio=0.0,
        feedforward_torque_limit_nm=config.controller.torque_limit_nm,
        torque_limit_nm=config.controller.torque_limit_nm,
    )
    return_config = replace(
        command_config,
        kp=config.controller.return_mit_kp,
        kd=config.controller.return_mit_kd,
        feedforward_torque_limit_nm=config.controller.return_torque_limit_nm,
        torque_limit_nm=config.controller.return_torque_limit_nm,
    )
    controller = GripController(config, kinematics=kinematics, command_config=command_config)
    target_source = build_target_source(
        curve=_curve_from_config(config),
        adaptive=config.reference.adaptive,
        max_target_force_n=config.safety.max_target_force_n,
        control_rate_hz=timing.control_rate_hz,
        contact_floor_n=lifecycle_config.contact_off_n,
    )
    stiffness = StiffnessDiagnostics(config.estimation, kinematics)

    period = 1.0 / timing.control_rate_hz
    phase_started = last_control = clock()
    trajectory = (
        _home_trajectory(config, kinematics, feedback.position_rad)
        if lifecycle.phase is LifecyclePhase.HOMING
        else _approach_trajectory(config, command_config, kinematics, feedback.position_rad)
    )
    last_command = _hold_command(kinematics, command_config, feedback)
    transition: ContactTransition | None = None
    reference_position = feedback.position_rad
    last_sample: TactileSnapshot | None = None
    contact_since: float | None = None
    lost_since: float | None = None
    preload_stable_since: float | None = None
    preload_started = 0.0
    task_time_s = 0.0
    tracking_begun = False
    reapproach_attempts = 0
    reapproach_total_s = 0.0
    current_target: ForceTarget | None = None
    latest_stiffness: StiffnessSnapshot | None = None

    def fail(reason: str) -> NoReturn:
        """把实验级故障交给外层故障保持状态机。"""
        raise HoldableExperimentFault(reason)

    def enter_phase(phase: LifecyclePhase, message: str, **extra: object) -> None:
        """登记阶段事件。"""
        emit({"event": "state", "phase": phase.value, "message": message, **extra})

    while not lifecycle.is_terminal:
        now = clock()
        dt = now - last_control if last_sample is not None else period
        if not 0 < dt <= timing.max_control_gap_s:
            fail(f"控制周期超时：{dt:.4f}s")
        if lifecycle.phase is LifecyclePhase.HOMING:
            try:
                action = action_source()
            except KeyboardInterrupt:
                raise
            except BaseException as error:  # noqa: BLE001
                raise FatalHardwareFault(
                    str(error),
                    phase=lifecycle.phase.value,
                ) from error
            if action == "status":
                emit(
                    {
                        "event": "status",
                        "phase": lifecycle.phase.value,
                        "action": action,
                        "message": _PHASE_MESSAGES[lifecycle.phase],
                    }
                )
            elif action is not None:
                emit(
                    {
                        "event": "warning",
                        "code": "command_not_available",
                        "phase": lifecycle.phase.value,
                        "action": action,
                        "message": "自动回零阶段只接受 status；紧急停止请按 Ctrl+C。",
                    }
                )
            closure, velocity, _ = trajectory.sample(now - phase_started)
            command = _closure_trajectory_command(
                kinematics,
                return_config,
                feedback,
                closure,
                velocity,
                0.0,
                0.0,
            )
            if clock() - last_control > timing.max_control_gap_s:
                fail("自动回零命令计算或事件记录超时")
            send_started = clock()
            try:
                feedback = dm.command(command)
            except KeyboardInterrupt:
                raise
            except BaseException as error:  # noqa: BLE001
                raise FatalHardwareFault(
                    f"自动回零期间 DM 命令或反馈失败：{error}",
                    phase=lifecycle.phase.value,
                ) from error
            feedback_state[0] = feedback
            recorder.write(
                _motor_only_trace_row(
                    time_s=now - started,
                    phase=lifecycle.phase,
                    feedback=feedback,
                    command=command,
                    dt=dt,
                    latency=clock() - send_started,
                )
            )
            timeout_s = max(
                lifecycle_config.return_timeout_s,
                trajectory.duration_s + lifecycle_config.return_settle_timeout_s,
            )
            if now - phase_started > timeout_s:
                fail("自动回零轨迹执行超时")
            if (
                now - phase_started >= trajectory.duration_s
                and abs(feedback.position_rad - config.hardware.home_position_rad)
                <= config.hardware.home_tolerance_rad
            ):
                lifecycle.finish_homing(now)
                enter_phase(LifecyclePhase.APPROACH, _PHASE_MESSAGES[LifecyclePhase.APPROACH])
                phase_started = now
                trajectory = _approach_trajectory(
                    config,
                    command_config,
                    kinematics,
                    feedback.position_rad,
                )
                last_control = now
                sleep(max(0.0, period - (clock() - now)))
                continue
            last_control = now
            sleep(max(0.0, period - (clock() - now)))
            continue
        sample = tactile.latest()
        axes = _validate_snapshot(sample, now, config)
        paired = pair_observation(
            snapshot=sample,
            feedback=feedback,
            previous=last_sample,
            now_s=now,
            kinematics=kinematics,
        )
        try:
            action = action_source()
        except KeyboardInterrupt:
            raise
        except BaseException as error:  # noqa: BLE001
            raise FatalHardwareFault(
                str(error),
                phase=lifecycle.phase.value,
            ) from error
        if action == "release" and lifecycle.phase in {
            LifecyclePhase.ACTIVE,
            LifecyclePhase.HOLDING,
        }:
            emit(
                {
                    "event": "command_received",
                    "phase": lifecycle.phase.value,
                    "action": "release",
                    "message": "已收到 release，正在受限张开回位。",
                }
            )
            lifecycle.begin_return(now, "用户请求释放")
            enter_phase(LifecyclePhase.RETURNING, _PHASE_MESSAGES[LifecyclePhase.RETURNING])
            trajectory = _home_trajectory(config, kinematics, feedback.position_rad)
            phase_started = now
        elif action == "status":
            emit(
                {
                    "event": "status",
                    "phase": lifecycle.phase.value,
                    "action": action,
                    "message": _PHASE_MESSAGES[lifecycle.phase],
                }
            )
        elif action == "release":
            emit(
                {
                    "event": "warning",
                    "code": "command_not_available",
                    "phase": lifecycle.phase.value,
                    "action": action,
                    "message": (
                        f"阶段 {lifecycle.phase.value} 尚不能执行 release；"
                        "输入 status 查看状态，紧急停止请按 Ctrl+C。"
                    ),
                }
            )
        elif action is not None:
            emit(
                {
                    "event": "warning",
                    "code": "unknown_command",
                    "phase": lifecycle.phase.value,
                    "action": action,
                    "message": (
                        f"无法识别命令 {action!r}；可用命令为 status、release，输入后按 Enter。"
                    ),
                }
            )
        if paired.is_new_tactile:
            left_n, right_n = sample.left_force_n, sample.right_force_n
            if lifecycle.phase is LifecyclePhase.APPROACH:
                if min(left_n, right_n) >= lifecycle_config.contact_on_n:
                    contact_since = sample.received_at_s if contact_since is None else contact_since
                    if sample.received_at_s - contact_since >= lifecycle_config.contact_on_stable_s:
                        lifecycle.confirm_contact(now)
                        phase_started = now
                        reference_position = feedback.position_rad
                        transition = ContactTransition(
                            last_command.velocity_rad_s, lifecycle_config.contact_transition_s
                        )
                        enter_phase(
                            LifecyclePhase.CONTACT_TRANSITION,
                            _PHASE_MESSAGES[LifecyclePhase.CONTACT_TRANSITION],
                        )
                        emit(
                            {
                                "event": "contact_confirmed",
                                "contact_segment": lifecycle.contact_segment,
                                "left_normal_n": left_n,
                                "right_normal_n": right_n,
                            }
                        )
                else:
                    contact_since = None
            elif lifecycle.phase not in {
                LifecyclePhase.CONTACT_TRANSITION,
                LifecyclePhase.RETURNING,
                LifecyclePhase.COMPLETED,
            }:
                lost = (
                    min(left_n, right_n) <= lifecycle_config.contact_off_n
                    if lifecycle_config.lost_contact_scope == "any_side"
                    else max(left_n, right_n) <= lifecycle_config.contact_off_n
                )
                if lost:
                    lost_since = sample.received_at_s if lost_since is None else lost_since
                    if sample.received_at_s - lost_since >= lifecycle_config.contact_off_stable_s:
                        if lifecycle_config.lost_contact_action == "fault" or (
                            target_source.kind == "adaptive"
                        ):
                            fail("跟踪阶段持续失去接触")
                        reapproach_attempts += 1
                        if reapproach_attempts > lifecycle_config.reapproach_max_attempts:
                            fail("重新接近次数超过上限")
                        lifecycle.reenter_approach(
                            now, f"失接触后重新接近（第 {reapproach_attempts} 次）"
                        )
                        enter_phase(
                            LifecyclePhase.APPROACH,
                            _PHASE_MESSAGES[LifecyclePhase.APPROACH],
                            attempts=reapproach_attempts,
                        )
                        phase_started = now
                        trajectory = _approach_trajectory(
                            config, command_config, kinematics, feedback.position_rad
                        )
                        controller.reset(feedback.position_rad)
                        tracking_begun = False
                        contact_since = None
                else:
                    lost_since = None
            if lifecycle.is_tracking:
                policy_dt = (
                    period
                    if last_sample is None
                    else sample.received_at_s - last_sample.received_at_s
                )
                target_source.observe(paired, policy_dt)
                latest_stiffness = stiffness.update(
                    position_rad=feedback.position_rad,
                    normal_force_n=paired.measured_force_n,
                    time_s=now - started,
                    sample_id=sample.packet_counter,
                )
                if lifecycle.phase is LifecyclePhase.PRELOAD:
                    target_value = target_source.preload_target(task_time_s)
                    minimum_stable_force_n = target_value - lifecycle_config.preload_tolerance_n
                    stable = (
                        min(left_n, right_n) >= lifecycle_config.contact_on_n
                        and paired.measured_force_n >= minimum_stable_force_n
                    )
                    preload_stable_since = (
                        (now if preload_stable_since is None else preload_stable_since)
                        if stable
                        else None
                    )
                    if preload_stable_since is not None and (
                        now - preload_stable_since >= lifecycle_config.preload_stable_time_s
                    ):
                        target_source.activate()
                        lifecycle.activate(now)
                        phase_started = now
                        emit(
                            {
                                "event": "target_activated",
                                "source": target_source.kind,
                                "message": "初始抓力稳定，目标策略已启用",
                            }
                        )
                        enter_phase(LifecyclePhase.ACTIVE, _PHASE_MESSAGES[LifecyclePhase.ACTIVE])
            last_sample = sample
        if lifecycle.phase is LifecyclePhase.CONTACT_TRANSITION:
            assert transition is not None
            if now - phase_started >= lifecycle_config.contact_transition_s:
                lifecycle.finish_contact_transition(now)
                phase_started = preload_started = now
                controller.reset(feedback.position_rad)
                stiffness.reset_contact(
                    position_rad=feedback.position_rad,
                    normal_force_n=paired.measured_force_n,
                )
                target_source.stabilize_preload()
                preload_stable_since = None
                tracking_begun = False
                enter_phase(LifecyclePhase.PRELOAD, _PHASE_MESSAGES[LifecyclePhase.PRELOAD])
        if lifecycle.phase is LifecyclePhase.PRELOAD and now - preload_started > (
            lifecycle_config.preload_timeout_s
        ):
            target_value = target_source.preload_target(task_time_s)
            minimum_stable_force_n = target_value - lifecycle_config.preload_tolerance_n
            detail = (
                "初始抓力在等待上限内未达到稳定："
                f"目标={target_value:.3f}N，当前均值={paired.measured_force_n:.3f}N"
                f"（LEFT={sample.left_force_n:.3f}N、RIGHT={sample.right_force_n:.3f}N），"
                f"最低预载力={minimum_stable_force_n:.3f}N"
            )
            if (
                config.controller.kind == "admittance"
                and config.controller.admittance.prevent_unloading
                and paired.measured_force_n > target_value
            ):
                detail += "；导纳 prevent_unloading=true，力偏高时不会反向纠偏"
            fail(detail)
        if lifecycle.phase is LifecyclePhase.ACTIVE and task_time_s >= target_source.duration_s:
            lifecycle.finish_task(now)
            phase_started = now
            emit(
                {
                    "event": "task_finished",
                    "task_time_s": task_time_s,
                    "message": "任务计时完成，保持抓握",
                }
            )
            enter_phase(LifecyclePhase.HOLDING, _PHASE_MESSAGES[LifecyclePhase.HOLDING])
            if lifecycle_config.on_finished == "return":
                lifecycle.begin_return(now, "配置 on_finished=return，自动回位")
                trajectory = _home_trajectory(config, kinematics, feedback.position_rad)
                phase_started = now
                enter_phase(LifecyclePhase.RETURNING, _PHASE_MESSAGES[LifecyclePhase.RETURNING])

        # 目标与命令生成。
        command: MITCommand
        controller_step = None
        if lifecycle.phase in {LifecyclePhase.APPROACH, LifecyclePhase.RETURNING}:
            closure, velocity, _ = trajectory.sample(now - phase_started)
            command = _closure_trajectory_command(
                kinematics,
                return_config if lifecycle.phase is LifecyclePhase.RETURNING else command_config,
                feedback,
                closure,
                velocity,
                config.safety.approach_feedforward_force_n
                if lifecycle.phase is LifecyclePhase.APPROACH
                else 0.0,
                config.safety.approach_feedforward_ratio
                if lifecycle.phase is LifecyclePhase.APPROACH
                else 0.0,
            )
        elif lifecycle.phase is LifecyclePhase.CONTACT_TRANSITION:
            assert transition is not None
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
            if lifecycle.phase is LifecyclePhase.PRELOAD:
                current_target = _preload_force_target(target_source, task_time_s)
            else:
                current_target = target_source.active_reference(task_time_s)
            stiffness_value = (
                stiffness.control_value()
                if config.controller.stiffness_consumption == "feedforward"
                else None
            )
            if not tracking_begun:
                controller_step = controller.begin_contact_tracking(
                    paired=paired,
                    target=current_target,
                    time_s=now - started,
                    dt=dt,
                    stiffness_value=stiffness_value,
                )
                tracking_begun = True
                emit(
                    {
                        "event": "tracking_initialized",
                        "contact_segment": lifecycle.contact_segment,
                    }
                )
            else:
                controller_step = controller.step_tracking(
                    paired=paired,
                    target=current_target,
                    time_s=now - started,
                    dt=dt,
                    stiffness_value=stiffness_value,
                )
            command = controller_step.command

        # 交互输出／存储也可能延迟；发送前再次检查时限和输入新鲜度。
        if clock() - last_control > timing.max_control_gap_s:
            fail("发送前控制计算或事件记录超时")
        _validate_snapshot(sample, clock(), config)
        send_started = clock()
        try:
            feedback = dm.command(command)
        except KeyboardInterrupt:
            raise
        except BaseException as error:  # noqa: BLE001
            raise FatalHardwareFault(
                f"DM 命令或反馈失败：{error}",
                phase=lifecycle.phase.value,
            ) from error
        feedback_state[0] = feedback
        latency = clock() - send_started
        row = _trace_row(
            time_s=now - started,
            paired=paired,
            axes=axes,
            phase=lifecycle.phase,
            task_time_s=task_time_s if lifecycle.phase is not LifecyclePhase.PRELOAD else None,
            contact_segment=lifecycle.contact_segment,
            target=current_target,
            controller_step=controller_step,
            stiffness=latest_stiffness,
            feedback=feedback,
            command=command,
            dt=dt,
            tactile_age_s=send_started - sample.received_at_s,
            latency=latency,
        )
        recorder.write(row)
        if clock() - last_control > timing.max_control_gap_s:
            fail("命令反馈或控制记录超时")
        if lifecycle.phase is LifecyclePhase.ACTIVE:
            task_time_s += dt
        if lifecycle.phase is LifecyclePhase.APPROACH:
            if reapproach_attempts:
                reapproach_total_s += dt
                if reapproach_total_s > lifecycle_config.reapproach_timeout_s:
                    fail("重新接近累计时间超过上限")
            if (
                now - phase_started
                > trajectory.duration_s + lifecycle_config.approach_endpoint_hold_s
            ):
                fail("预接触轨迹到达闭合端后仍未检测到双侧接触")
        if lifecycle.phase is LifecyclePhase.RETURNING:
            if now - phase_started > max(
                lifecycle_config.return_timeout_s,
                trajectory.duration_s + lifecycle_config.return_settle_timeout_s,
            ):
                fail("回位轨迹执行超时")
            if (
                now - phase_started >= trajectory.duration_s
                and abs(feedback.position_rad - config.hardware.home_position_rad)
                <= lifecycle_config.return_position_tolerance_rad
            ):
                lifecycle.complete_return(now)
                enter_phase(LifecyclePhase.COMPLETED, _PHASE_MESSAGES[LifecyclePhase.COMPLETED])
        if terminal is not None:
            terminal.publish(
                _run_snapshot(
                    config,
                    lifecycle,
                    elapsed_s=now - phase_started,
                    task_time_s=task_time_s,
                    paired=paired,
                    target=current_target,
                    stiffness=latest_stiffness,
                    feedback=feedback,
                    dt=dt,
                    output_directory=str(recorder.directory),
                    controller_step=controller_step,
                )
            )
        last_control, last_command = now, command
        sleep(max(0.0, period - (clock() - now)))
    if lifecycle.phase is LifecyclePhase.FAULT:
        raise RuntimeError(lifecycle.fault_reason or "未知故障")
    return {
        "status": lifecycle.phase.value,
        "final_phase": lifecycle.phase.value,
        "task_time_s": task_time_s,
    }


def _run_fault_holding(
    config: ExperimentConfig,
    dm: DmSession,
    recorder: ExperimentRecorder,
    emit: EventSink,
    action_source: Callable[[], str | None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    started: float,
    lifecycle: Lifecycle,
    feedback,
    primary_error: BaseException,
) -> dict[str, object]:
    """建立并守护故障保持，确保后续异常不能覆盖首个实验故障。"""
    try:
        return _run_fault_holding_impl(
            config,
            dm,
            recorder,
            emit,
            action_source,
            clock,
            sleep,
            started,
            lifecycle,
            feedback,
            primary_error,
        )
    except FatalHardwareFault:
        raise
    except KeyboardInterrupt as error:
        holding_entered = lifecycle.fault_holding_started_s is not None
        raise FatalHardwareFault(
            "故障保持控制期间收到 Ctrl+C，立即请求失能",
            phase=(
                lifecycle.fault_phase.value
                if lifecycle.fault_phase is not None
                else lifecycle.phase.value
            ),
            resolution="forced_disable",
            holding_entered=holding_entered,
            holding_duration_s=_fault_holding_duration(lifecycle, clock()),
            primary_error=primary_error,
        ) from error
    except BaseException as error:  # noqa: BLE001
        holding_entered = lifecycle.fault_holding_started_s is not None
        raise FatalHardwareFault(
            f"故障保持后续控制所有权丢失：{error}",
            phase=(
                lifecycle.fault_phase.value
                if lifecycle.fault_phase is not None
                else lifecycle.phase.value
            ),
            resolution="hold_lost" if holding_entered else "forced_disable",
            holding_entered=holding_entered,
            holding_duration_s=_fault_holding_duration(lifecycle, clock()),
            primary_error=primary_error,
        ) from error


def _fault_holding_duration(lifecycle: Lifecycle, now_s: float) -> float:
    """返回从首次建立故障保持起累计的非负时长。"""
    if lifecycle.fault_holding_started_s is None:
        return 0.0
    return max(0.0, now_s - lifecycle.fault_holding_started_s)


def _run_fault_holding_impl(
    config: ExperimentConfig,
    dm: DmSession,
    recorder: ExperimentRecorder,
    emit: EventSink,
    action_source: Callable[[], str | None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    started: float,
    lifecycle: Lifecycle,
    feedback,
    primary_error: BaseException,
) -> dict[str, object]:
    """以 DM-only 循环保持首个故障位置，直到人工释放并回到 home。"""
    source_phase = lifecycle.phase.value
    kinematics = _KINEMATICS
    base_config = MITCommandConfig(
        position_min_rad=dm.deployment.joint_position_min_rad,
        position_max_rad=dm.deployment.joint_position_max_rad,
        velocity_limit_rad_s=config.controller.velocity_limit_rad_s,
        closing_direction=dm.deployment.closing_direction,
        kp=config.controller.return_mit_kp,
        kd=config.controller.return_mit_kd,
        feedforward_ratio=0.0,
        feedforward_torque_limit_nm=config.controller.return_torque_limit_nm,
        torque_limit_nm=config.controller.return_torque_limit_nm,
    )
    period = 1.0 / config.timing.control_rate_hz
    secondary_errors: list[str] = []

    def safe_emit(event: dict[str, object]) -> None:
        """记录故障阶段事件；记录通道失败不得夺走电机控制权。"""
        try:
            emit(event)
        except KeyboardInterrupt as error:
            raise FatalHardwareFault(
                "故障保持事件记录期间收到 Ctrl+C，立即请求失能",
                phase=source_phase,
                resolution="forced_disable",
                holding_entered=True,
                holding_duration_s=clock() - entered_s,
                primary_error=primary_error,
            ) from error
        except Exception as error:
            secondary_errors.append(f"故障保持事件记录失败：{error}")

    try:
        hold_command = _hold_command(kinematics, base_config, feedback)
        feedback = dm.hold(hold_command)
    except KeyboardInterrupt as error:
        raise FatalHardwareFault(
            "首个故障后收到 Ctrl+C，立即请求失能",
            phase=source_phase,
            resolution="forced_disable",
            primary_error=primary_error,
        ) from error
    except BaseException as error:  # noqa: BLE001
        raise FatalHardwareFault(
            f"首个故障后无法建立位置保持：{error}",
            phase=source_phase,
            resolution="forced_disable",
            primary_error=primary_error,
        ) from error
    entered_s = clock()
    lifecycle.hold_fault(entered_s, str(primary_error))
    safe_emit(
        {
            "event": "fault_detected",
            "phase": source_phase,
            "error": str(primary_error),
            "hold_established": True,
        }
    )
    safe_emit(
        {
            "event": "state",
            "phase": LifecyclePhase.FAULT_HOLDING.value,
            "message": _PHASE_MESSAGES[LifecyclePhase.FAULT_HOLDING],
            "primary_error": str(primary_error),
            "fault_phase": source_phase,
            "hold_position_rad": hold_command.position_rad,
        }
    )
    return_trajectory: ClosureTrajectory | None = None
    return_started_s = 0.0
    last_control_s = entered_s
    while True:
        now = clock()
        if now - last_control_s > config.timing.max_control_gap_s:
            safe_emit(
                {
                    "event": "warning",
                    "code": "fault_holding_control_gap",
                    "phase": lifecycle.phase.value,
                    "message": "故障保持周期超时，正在立即刷新安全命令。",
                }
            )
        try:
            action = action_source()
        except KeyboardInterrupt as error:
            raise FatalHardwareFault(
                "故障保持期间收到 Ctrl+C，立即请求失能",
                phase=source_phase,
                resolution="forced_disable",
                holding_entered=True,
                holding_duration_s=clock() - entered_s,
                primary_error=primary_error,
            ) from error
        except BaseException as error:  # noqa: BLE001
            raise FatalHardwareFault(
                f"故障保持期间无法继续接收人工 release：{error}",
                phase=source_phase,
                resolution="hold_lost",
                holding_entered=True,
                holding_duration_s=clock() - entered_s,
                primary_error=primary_error,
            ) from error
        if lifecycle.phase is LifecyclePhase.FAULT_HOLDING:
            if action == "release":
                safe_emit(
                    {
                        "event": "command_received",
                        "phase": lifecycle.phase.value,
                        "action": "release",
                        "message": "已收到 release，正在执行不依赖触觉的受限回位。",
                    }
                )
                lifecycle.begin_return(now, "故障保持后人工 release")
                return_trajectory = _home_trajectory(config, kinematics, feedback.position_rad)
                return_started_s = now
                safe_emit(
                    {
                        "event": "state",
                        "phase": LifecyclePhase.RETURNING.value,
                        "message": _PHASE_MESSAGES[LifecyclePhase.RETURNING],
                        "fault_release": True,
                    }
                )
            elif action == "status":
                safe_emit(
                    {
                        "event": "status",
                        "phase": lifecycle.phase.value,
                        "action": action,
                        "message": _PHASE_MESSAGES[LifecyclePhase.FAULT_HOLDING],
                        "primary_error": str(primary_error),
                    }
                )
            elif action is not None:
                safe_emit(
                    {
                        "event": "warning",
                        "code": "unknown_command",
                        "phase": lifecycle.phase.value,
                        "action": action,
                        "message": "故障保持阶段只接受 status 或 release。",
                    }
                )
            if lifecycle.phase is LifecyclePhase.FAULT_HOLDING:
                try:
                    command = _hold_command(kinematics, base_config, feedback)
                    feedback = dm.hold(command)
                except KeyboardInterrupt as error:
                    raise FatalHardwareFault(
                        "故障保持命令期间收到 Ctrl+C，立即请求失能",
                        phase=source_phase,
                        resolution="forced_disable",
                        holding_entered=True,
                        holding_duration_s=clock() - entered_s,
                        primary_error=primary_error,
                    ) from error
                except BaseException as error:  # noqa: BLE001
                    raise FatalHardwareFault(
                        f"故障保持丢失：{error}",
                        phase=source_phase,
                        resolution="hold_lost",
                        holding_entered=True,
                        holding_duration_s=clock() - entered_s,
                        primary_error=primary_error,
                    ) from error
        else:
            assert lifecycle.phase is LifecyclePhase.RETURNING
            assert return_trajectory is not None
            closure, velocity, _ = return_trajectory.sample(now - return_started_s)
            command = _closure_trajectory_command(
                kinematics,
                base_config,
                feedback,
                closure,
                velocity,
                0.0,
                0.0,
            )
            try:
                feedback = dm.command(command)
            except KeyboardInterrupt as error:
                raise FatalHardwareFault(
                    "故障释放回位期间收到 Ctrl+C，立即请求失能",
                    phase=source_phase,
                    resolution="forced_disable",
                    holding_entered=True,
                    holding_duration_s=clock() - entered_s,
                    primary_error=primary_error,
                ) from error
            except BaseException as error:  # noqa: BLE001
                raise FatalHardwareFault(
                    f"故障释放回位期间 DM 命令或反馈失败：{error}",
                    phase=source_phase,
                    resolution="hold_lost",
                    holding_entered=True,
                    holding_duration_s=clock() - entered_s,
                    primary_error=primary_error,
                ) from error
            try:
                recorder.write(
                    _motor_only_trace_row(
                        time_s=now - started,
                        phase=lifecycle.phase,
                        feedback=feedback,
                        command=command,
                        dt=max(period, now - last_control_s),
                        latency=clock() - now,
                    )
                )
            except Exception as error:
                secondary_errors.append(f"故障释放回位记录失败：{error}")
            if (
                now - return_started_s >= return_trajectory.duration_s
                and abs(feedback.position_rad - config.hardware.home_position_rad)
                <= config.hardware.home_tolerance_rad
            ):
                lifecycle.complete_return(now)
                safe_emit(
                    {
                        "event": "state",
                        "phase": LifecyclePhase.COMPLETED.value,
                        "message": "故障后回位完成；本次实验结果仍为 failed。",
                    }
                )
                return {
                    "status": "failed",
                    "final_phase": LifecyclePhase.COMPLETED.value,
                    "primary_error": primary_error,
                    "fault_phase": source_phase,
                    "fault_holding_entered": True,
                    "fault_holding_duration_s": now - entered_s,
                    "fault_resolution": "released",
                    "secondary_errors": secondary_errors,
                }
            timeout_s = max(
                config.lifecycle.return_timeout_s,
                return_trajectory.duration_s + config.lifecycle.return_settle_timeout_s,
            )
            if now - return_started_s > timeout_s:
                try:
                    feedback = dm.hold(_hold_command(kinematics, base_config, feedback))
                except KeyboardInterrupt as error:
                    raise FatalHardwareFault(
                        "故障释放超时恢复保持时收到 Ctrl+C，立即请求失能",
                        phase=source_phase,
                        resolution="forced_disable",
                        holding_entered=True,
                        holding_duration_s=clock() - entered_s,
                        primary_error=primary_error,
                    ) from error
                except BaseException as error:  # noqa: BLE001
                    raise FatalHardwareFault(
                        f"故障释放回位超时且无法恢复保持：{error}",
                        phase=source_phase,
                        resolution="hold_lost",
                        holding_entered=True,
                        holding_duration_s=clock() - entered_s,
                        primary_error=primary_error,
                    ) from error
                lifecycle.resume_fault_holding(now, "故障释放回位超时，已恢复位置保持")
                safe_emit(
                    {
                        "event": "state",
                        "phase": LifecyclePhase.FAULT_HOLDING.value,
                        "message": "故障释放回位未达 home，已恢复保持；请检查后再次输入 release。",
                        "primary_error": str(primary_error),
                    }
                )
                return_trajectory = None
        last_control_s = now
        try:
            sleep(max(0.0, period - (clock() - now)))
        except KeyboardInterrupt as error:
            raise FatalHardwareFault(
                "故障保持等待期间收到 Ctrl+C，立即请求失能",
                phase=source_phase,
                resolution="forced_disable",
                holding_entered=True,
                holding_duration_s=clock() - entered_s,
                primary_error=primary_error,
            ) from error


def _preload_force_target(target_source: TargetSource, task_time_s: float) -> ForceTarget:
    """构造 preload 阶段的常值目标。"""
    value = target_source.preload_target(task_time_s)
    return ForceTarget(
        force_n=value,
        rate_n_s=None,
        acceleration_n_s2=None,
        source=target_source.kind,
    )


def _curve_from_config(config: ExperimentConfig) -> ForceReferenceCurve | None:
    """把配置曲线转换为共享核曲线；动态模式返回 ``None``。"""
    if config.reference.curve is None:
        return None
    return ForceReferenceCurve(
        interpolation=config.reference.curve.interpolation,
        waypoints=tuple(
            ForceWaypoint(t_s=waypoint.t_s, force_n=waypoint.force_n)
            for waypoint in config.reference.curve.waypoints
        ),
    )


def _run_snapshot(
    config: ExperimentConfig,
    lifecycle: Lifecycle,
    *,
    elapsed_s: float,
    task_time_s: float,
    paired: PairedObservation,
    target: ForceTarget | None,
    stiffness: StiffnessSnapshot | None,
    feedback,
    dt: float,
    output_directory: str,
    controller_step,
) -> RunSnapshot:
    """构造发布给终端的不可变快照。"""
    return RunSnapshot(
        phase=lifecycle.phase,
        phase_elapsed_s=elapsed_s,
        task_time_s=task_time_s,
        task_name=config.metadata.task_name,
        object_name=config.metadata.object_name,
        left_normal_n=paired.snapshot.left_force_n,
        right_normal_n=paired.snapshot.right_force_n,
        target_force_n=target.force_n if target is not None else 0.0,
        tangential_force_n=paired.tangential_force_n,
        stiffness_n_per_m=stiffness.value_n_per_m if stiffness is not None else None,
        stiffness_valid=stiffness.valid if stiffness is not None else None,
        stiffness_updated=stiffness.updated if stiffness is not None else None,
        aperture_m=_KINEMATICS.aperture(feedback.position_rad),
        position_limited=False,
        control_dt_s=dt,
        tactile_age_s=paired.tactile_age_s,
        output_directory=output_directory,
        force_deadband_active=(
            controller_step.force_deadband_active if controller_step is not None else False
        ),
        unloading_blocked=(
            controller_step.unloading_blocked if controller_step is not None else False
        ),
    )


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


def _validate_snapshot(
    snapshot: TactileSnapshot, now_s: float, config: ExperimentConfig
) -> tuple[float, ...]:
    """原始力用于保护，滤波法向力用于控制。"""
    values = (snapshot.left_force_n, snapshot.right_force_n, snapshot.received_at_s, now_s)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("触觉快照包含非有限数值")
    age_s = now_s - snapshot.received_at_s
    if age_s < 0.0 or age_s > config.timing.tactile_timeout_s:
        raise RuntimeError(f"触觉快照过期：age={age_s:.3f}s")
    axes = raw_axes(snapshot)
    for side, taxels in (
        ("LEFT", snapshot.left_taxel_forces_n),
        ("RIGHT", snapshot.right_taxel_forces_n),
    ):
        if not taxels:
            raise RuntimeError(f"{side} 逐 taxel 数据缺失")
        for taxel_index, vector in enumerate(taxels):
            if len(vector) != 3 or not all(math.isfinite(value) for value in vector):
                raise RuntimeError(f"{side} taxel {taxel_index} 三轴数据结构错误或包含非有限数值")
    if max(abs(axes[2]), abs(axes[5])) > config.safety.force_ceiling_n:
        raise RuntimeError("原始法向力超过保护上限")
    return axes


def _verify_zero(
    tactile: TactileWorker,
    config: ExperimentConfig,
    clear_bias: bool,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    *,
    warning_sink: EventSink | None = None,
) -> None:
    """以滤波双侧 Fz 的窗口均值验证空载，并报告接触级别峰值。

    瞬时峰值只作为操作告警，不重置零力窗口；双侧窗口均值、原始力上限与
    双侧不平衡保护仍然是使能前的硬性门禁。
    """
    lifecycle = config.lifecycle
    sample = tactile.wait_for_update(None, config.timing.tactile_startup_timeout_s)
    if clear_bias:
        sleep(config.timing.tactile_bias_settle_s)
        settle_boundary = tactile.latest().received_at_s
        sample = tactile.wait_for_update(settle_boundary, config.timing.tactile_timeout_s)
    deadline = clock() + lifecycle.zero_force_timeout_s
    previous_timestamp: int | None = None
    previous_counter: int | None = None
    taxel_counts: tuple[int, int] | None = None
    window: deque[tuple[float, float, float, TactileSnapshot]] = deque()
    left_sum_n = right_sum_n = 0.0
    window_duration_s = 0.0
    left_mean_n = right_mean_n = 0.0
    maximum_peak_n = 0.0
    peak_side: str | None = None
    while clock() < deadline:
        _validate_snapshot(sample, clock(), config)
        if previous_timestamp is not None and sample.timestamp_us <= previous_timestamp:
            raise RuntimeError("空载验证期间触觉设备时间未递增")
        if previous_counter is not None:
            counter_delta = (sample.packet_counter - previous_counter) % (1 << 32)
            if counter_delta == 0 or counter_delta >= (1 << 31):
                raise RuntimeError("空载验证期间触觉包计数未按前进规则递增")
        current_counts = (
            len(sample.left_taxel_forces_n),
            len(sample.right_taxel_forces_n),
        )
        if taxel_counts is None:
            taxel_counts = current_counts
        elif current_counts != taxel_counts:
            raise RuntimeError(
                "空载验证期间逐侧 taxel 数量变化："
                f"期望 LEFT={taxel_counts[0]}、RIGHT={taxel_counts[1]}，"
                f"实际 LEFT={current_counts[0]}、RIGHT={current_counts[1]}"
            )
        previous_timestamp = sample.timestamp_us
        previous_counter = sample.packet_counter
        left_force_n = abs(sample.left_force_n)
        right_force_n = abs(sample.right_force_n)
        peak_n = max(left_force_n, right_force_n)
        if peak_n > maximum_peak_n:
            maximum_peak_n = peak_n
            if left_force_n == right_force_n:
                peak_side = "both"
            elif left_force_n > right_force_n:
                peak_side = "left"
            else:
                peak_side = "right"
        window.append((sample.received_at_s, left_force_n, right_force_n, sample))
        left_sum_n += left_force_n
        right_sum_n += right_force_n
        cutoff_s = sample.received_at_s - lifecycle.zero_force_stable_s
        while len(window) > 1 and window[1][0] <= cutoff_s:
            _, expired_left_n, expired_right_n, _ = window.popleft()
            left_sum_n -= expired_left_n
            right_sum_n -= expired_right_n
        window_duration_s = sample.received_at_s - window[0][0]
        left_mean_n = left_sum_n / len(window)
        right_mean_n = right_sum_n / len(window)
        if (
            window_duration_s >= lifecycle.zero_force_stable_s
            and left_mean_n <= lifecycle.zero_force_threshold_n
            and right_mean_n <= lifecycle.zero_force_threshold_n
        ):
            if warning_sink is not None:
                warning_sink(
                    {
                        "event": "zero_force_diagnostics",
                        "window_duration_s": window_duration_s,
                        "left_global_mean_n": left_mean_n,
                        "right_global_mean_n": right_mean_n,
                        "taxels": _zero_taxel_diagnostics([item[3] for item in window]),
                    }
                )
            if maximum_peak_n >= lifecycle.contact_on_n and warning_sink is not None:
                side_label = {"left": "LEFT", "right": "RIGHT", "both": "双侧"}.get(
                    peak_side, "未知侧"
                )
                warning_sink(
                    {
                        "event": "warning",
                        "code": "zero_force_peak",
                        "message": (
                            "使能前零力均值验证通过，但滤波双侧 Fz 峰值为 "
                            f"{maximum_peak_n:.3f}N（{side_label}），达到 "
                            f"{lifecycle.contact_on_n:.3f}N 警告阈值；"
                            "请确认传感器无持续受力"
                        ),
                        "maximum_peak_n": maximum_peak_n,
                        "peak_side": peak_side,
                        "peak_warning_threshold_n": lifecycle.contact_on_n,
                    }
                )
            return
        remaining_s = deadline - clock()
        if remaining_s <= 0.0:
            break
        try:
            sample = tactile.wait_for_update(
                sample.received_at_s,
                min(config.timing.tactile_timeout_s, remaining_s),
            )
        except TimeoutError:
            if clock() >= deadline:
                break
            raise
    raise RuntimeError(
        "使能前滤波双侧 Fz 零力验证超时："
        f"{lifecycle.zero_force_stable_s:.3f}s 窗口均值阈值="
        f"{lifecycle.zero_force_threshold_n:.3f}N；"
        f"末窗口={window_duration_s:.3f}s，均值 LEFT={left_mean_n:.3f}N、"
        f"RIGHT={right_mean_n:.3f}N，验证期最大峰值={maximum_peak_n:.3f}N"
        f"（警告阈值={lifecycle.contact_on_n:.3f}N）；"
        "请保持传感器空载"
    )


def _zero_taxel_diagnostics(samples: list[TactileSnapshot]) -> dict[str, object]:
    """汇总零力窗口的逐侧 taxel 三轴残余，不参与通过判定。"""
    diagnostics: dict[str, object] = {}
    for side in ("left", "right"):
        frames = [getattr(sample, f"{side}_taxel_forces_n") for sample in samples]
        taxel_count = len(frames[0]) if frames else 0
        axis_values = [
            [float(vector[axis]) for frame in frames for vector in frame] for axis in range(3)
        ]
        per_taxel = []
        for taxel_index in range(taxel_count):
            values_by_axis = [
                [float(frame[taxel_index][axis]) for frame in frames] for axis in range(3)
            ]
            per_taxel.append(
                {
                    "taxel_index": taxel_index,
                    "axis_mean_n": [statistics.fmean(values) for values in values_by_axis],
                    "axis_std_n": [statistics.pstdev(values) for values in values_by_axis],
                    "maximum_abs_residual_n": max(
                        abs(value) for values in values_by_axis for value in values
                    ),
                }
            )
        maximum = max(
            (
                (abs(float(value)), taxel_index, axis, float(value))
                for frame in frames
                for taxel_index, vector in enumerate(frame)
                for axis, value in enumerate(vector)
            ),
            default=(0.0, 0, 0, 0.0),
        )
        diagnostics[side] = {
            "taxel_count": taxel_count,
            "axis_mean_n": [statistics.fmean(values) for values in axis_values],
            "axis_std_n": [statistics.pstdev(values) for values in axis_values],
            "maximum_abs_residual_n": maximum[0],
            "maximum_residual_taxel_index": maximum[1],
            "maximum_residual_axis": ("x", "y", "z")[maximum[2]],
            "maximum_residual_signed_n": maximum[3],
            "per_taxel": per_taxel,
        }
    return diagnostics


def _approach_trajectory(
    config: ExperimentConfig,
    command_config: MITCommandConfig,
    kinematics: CrankSliderKinematics,
    start_position_rad: float,
) -> ClosureTrajectory:
    """从当前反馈位置规划到闭合机械端点的闭合量轨迹。"""
    lifecycle = config.lifecycle
    goal = (
        command_config.position_max_rad
        if command_config.closing_direction > 0
        else command_config.position_min_rad
    )
    return ClosureTrajectory.from_limits(
        kinematics.closure(start_position_rad),
        kinematics.closure(goal),
        lifecycle.approach_closure_velocity_m_s,
        lifecycle.approach_closure_acceleration_m_s2,
        lifecycle.approach_closure_jerk_m_s3,
    )


def _home_trajectory(
    config: ExperimentConfig,
    kinematics: CrankSliderKinematics,
    start_position_rad: float,
) -> ClosureTrajectory:
    """复用受限回位参数，从当前反馈规划到配置的 home 位置。"""
    return ClosureTrajectory.from_limits(
        kinematics.closure(start_position_rad),
        kinematics.closure(config.hardware.home_position_rad),
        config.lifecycle.return_closure_velocity_m_s,
        config.lifecycle.return_closure_acceleration_m_s2,
        config.lifecycle.return_closure_jerk_m_s3,
    )


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


def _closure_trajectory_command(
    kinematics: CrankSliderKinematics,
    config: MITCommandConfig,
    feedback,
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
    feedback,
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
    feedback,
) -> MITCommand:
    """构造当前位置零速度保持命令。"""
    command = _trajectory_command(
        kinematics,
        config,
        feedback,
        feedback.position_rad,
        0.0,
        0.0,
        0.0,
    )
    expected_position_rad = min(
        max(feedback.position_rad, config.position_min_rad),
        config.position_max_rad,
    )
    if not math.isclose(command.position_rad, expected_position_rad, abs_tol=1e-9):
        raise ValueError("当前位置投影无法同时满足故障保持的合成力矩限制")
    return command

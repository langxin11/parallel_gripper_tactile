"""DM4310P 受限小步运动探针。"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Final

from .deployment import Dm4310PGripperConfig
from .protocol import (
    CMD_DISABLE,
    CMD_ENABLE,
    CONTROL_MODE_REGISTER,
    MitCommand,
    MotorFeedback,
    STATUS_DISABLED,
    STATUS_ENABLED,
    Usb2CanProtocol,
    motor_status_is_fault,
)
from .runtime import DmRegisterReader, DmResponseReceiver, DmStateRefresher, ShortWriteError
from .transport import ByteTransport, TransportClosedError

DEFAULT_CLOSING_STEP_RAD: Final = 0.03
"""首次真机探针固定使用的闭合增量（rad）。"""

DEFAULT_MIT_KP: Final = 2.0
"""首次受限探针使用的 MIT 位置刚度。

来源为 `dm_force_tracking.yaml` 的 `mit_kp=2.0`；它不是通用控制参数。
"""

DEFAULT_MIT_KD: Final = 0.5
"""首次受限探针使用的 MIT 速度阻尼。

来源为 `dm_force_tracking.yaml` 的 `mit_kd=0.5`；它不是通用控制参数。
"""

MIT_CONTROL_MODE: Final = 1
"""参考 `SetControlMode.srv` 定义的只读寄存器 10 的 MIT 模式值。"""

MAX_PLAN_AGE_S: Final = 0.2


class MotionSafetyError(RuntimeError):
    """运动探针发现反馈、目标或保护条件不安全时抛出的异常。"""


class MotionStageTimeoutError(TimeoutError):
    """一个运动阶段在限定时间内未收到安全反馈时抛出的异常。"""


class MotionDisableError(RuntimeError):
    """最终失能帧未完整写入，无法将运动探针报告为成功。"""


@dataclass(frozen=True, slots=True)
class MotionProbeSafetyConfig:
    """受限小步运动探针的协议和机械边界。

    该配置不构成急停、功能安全或硬实时控制。

    Args:
        closing_step_rad: 相对于初始角的闭合步长（rad），必须为正有限数值。
        mit_kp: MIT 位置刚度，实际运动必须位于协议范围 `(0, 500]`。
        mit_kd: MIT 速度阻尼，必须位于协议范围 `[0, 5]`。
        stage_duration_s: 每个保持、闭合和回位阶段的固定持续时间（秒）。
        control_interval_s: 两次命令之间的等待时间（秒）。
        position_tolerance_rad: 目标到达判定的位置误差（rad）。
    """

    closing_step_rad: float = DEFAULT_CLOSING_STEP_RAD
    mit_kp: float = DEFAULT_MIT_KP
    mit_kd: float = DEFAULT_MIT_KD
    stage_duration_s: float = 1.0
    control_interval_s: float = 0.02
    position_tolerance_rad: float = 0.005

    def __post_init__(self) -> None:
        """拒绝协议无法表示或数值不确定的输入。"""
        finite_positive = (
            ("阶段持续时间", self.stage_duration_s),
            ("控制间隔", self.control_interval_s),
            ("位置容差", self.position_tolerance_rad),
        )
        for name, value in finite_positive:
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ValueError(f"{name}必须是有限数值")
            if value <= 0.0:
                raise ValueError(f"{name}必须大于 0")
        if not 0.0 < self.closing_step_rad:
            raise ValueError("闭合步长必须大于 0")
        if not 0.0 < self.mit_kp <= 500.0:
            raise ValueError("MIT kp 必须位于 (0, 500]")
        if not 0.0 <= self.mit_kd <= 5.0:
            raise ValueError("MIT kd 必须位于 [0, 5]")


@dataclass(frozen=True, slots=True)
class MotionProbePlan:
    """由安全初始反馈生成的不可变三阶段运动计划。"""

    initial_feedback: MotorFeedback
    control_mode: int
    prepared_at_s: float
    initial_position_rad: float
    hold_target_rad: float
    close_target_rad: float
    return_target_rad: float


@dataclass(frozen=True, slots=True)
class MotionStageResult:
    """固定持续时间运动阶段的目标和运行诊断。"""

    stage: str
    target_position_rad: float
    target_reached: bool
    position_error_rad: float
    start_position_rad: float
    final_position_rad: float
    max_position_rad: float
    min_position_rad: float
    max_abs_velocity_rad_s: float
    max_abs_torque_nm: float
    command_count: int
    elapsed_s: float


class DmSafeMotionProbe:
    """在显式许可下执行保持、闭合和回位阻抗命令的探针。

    本类的 `inspect_and_plan()` 只读状态并拒绝故障或机械越界反馈；仅当调用方随后显式
    调用 `execute()` 时，才会写入使能、MIT 和失能帧。执行期间每次反馈都检查故障与机械位置，
    并记录速度和力矩；任何异常及正常结束都会尽力发送 `CMD_DISABLE`。这不是硬实时控制器，
    实际保护响应仍受串口、USB2CAN 与电机固件时延限制。

    Args:
        deployment: 固定 CAN ID、协议量程与机械行程的夹爪部署配置。
        protocol: 与 `deployment.motor_limits` 对应的 USB2CAN 编解码器。
        transport: 显式注入的字节传输。
        safety: 阻抗命令参数、阶段时长与位置诊断容差。
        clock: 单调时钟，测试可注入。
        sleep: 等待函数，测试可注入。
    """

    def __init__(
        self,
        deployment: Dm4310PGripperConfig,
        protocol: Usb2CanProtocol,
        transport: ByteTransport,
        *,
        safety: MotionProbeSafetyConfig = MotionProbeSafetyConfig(),
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """保存依赖，不打开端口也不发送报文。"""
        if protocol.limits != deployment.motor_limits:
            raise ValueError("运动探针协议量程必须与部署配置一致")
        self._deployment = deployment
        self._protocol = protocol
        self._transport = transport
        self._safety = safety
        self._clock = clock
        self._sleep = sleep
        self._receiver = DmResponseReceiver(deployment.device, protocol, transport, clock=clock)
        self._refresher = DmStateRefresher(
            deployment.device, protocol, transport, clock=clock, receiver=self._receiver
        )
        self._register_reader = DmRegisterReader(
            deployment.device, protocol, transport, self._receiver
        )
        self._prepared_plan: MotionProbePlan | None = None
        self._last_disable_confirmed = False

    @property
    def is_open(self) -> bool:
        """报告底层传输是否已由调用方显式打开。"""
        return self._refresher.is_open

    @property
    def last_disable_confirmed(self) -> bool:
        """报告最近一次成功执行是否已读回 `STATUS_DISABLED`。"""
        return self._last_disable_confirmed

    def open(self) -> None:
        """显式打开传输。"""
        self._refresher.open()

    def close(self) -> None:
        """关闭传输，不隐式发送任何控制帧。"""
        self._refresher.close()

    def inspect_and_plan(self) -> MotionProbePlan:
        """读取并验证初始状态，生成不执行的三阶段计划。

        Returns:
            以精确初始反馈位置为保持和回位目标的计划。

        Raises:
            MotionSafetyError: 初始反馈故障、机械越界或闭合目标越界。
        """
        first_feedback = self._refresher.refresh_once()
        self._validate_feedback(first_feedback)
        control_mode = self._register_reader.read_u32(CONTROL_MODE_REGISTER)
        feedback = self._refresher.refresh_once()
        self._validate_feedback(feedback)
        initial = feedback.position_rad
        close_target = self._deployment.validate_joint_position(
            initial + self._deployment.closing_direction * self._safety.closing_step_rad
        )
        plan = MotionProbePlan(
            initial_feedback=feedback,
            control_mode=control_mode,
            prepared_at_s=self._clock(),
            initial_position_rad=initial,
            hold_target_rad=initial,
            close_target_rad=close_target,
            return_target_rad=initial,
        )
        self._prepared_plan = plan
        return plan

    def execute(self, plan: MotionProbePlan) -> tuple[MotionStageResult, ...]:
        """执行已检查计划，并在所有退出路径尽力失能。

        Args:
            plan: 必须来自当前安全初始反馈的三阶段计划。

        Returns:
            保持、闭合、回位三个固定时长阶段的运行诊断。

        Raises:
            MotionSafetyError: 反馈触发故障、机械位置越界或使能状态异常。
            MotionStageTimeoutError: 某阶段未在单次通信时限内收到反馈。
            TransportClosedError: 传输尚未显式打开。
        """
        if not self._transport.is_open:
            raise TransportClosedError("运动探针传输尚未打开")
        if plan is not self._prepared_plan:
            raise MotionSafetyError("执行前必须先对同一计划完成新的初始状态检查")
        if self._clock() - plan.prepared_at_s > MAX_PLAN_AGE_S:
            raise MotionSafetyError(f"初始状态检查已超过 {MAX_PLAN_AGE_S} 秒，必须重新检查")
        self._validate_plan(plan)
        if plan.control_mode != MIT_CONTROL_MODE:
            raise MotionSafetyError(
                f"控制模式不是 MIT：寄存器 {CONTROL_MODE_REGISTER}={plan.control_mode}"
            )
        self._prepared_plan = None
        self._last_disable_confirmed = False
        enable_attempted = False
        try:
            enable_attempted = True
            self._write_packet(
                self._protocol.make_control_packet(self._deployment.motor_id, CMD_ENABLE)
            )
            enabled_feedback = self._receiver.receive_feedback()
            self._validate_feedback(enabled_feedback)
            if enabled_feedback.status_code != STATUS_ENABLED:
                raise MotionSafetyError(
                    f"使能确认失败：期望 status_code={STATUS_ENABLED}，实际为 "
                    f"{enabled_feedback.status_code}"
                )
            stages = (
                ("hold", plan.hold_target_rad),
                ("close", plan.close_target_rad),
                ("return", plan.return_target_rad),
            )
            results = tuple(self._run_stage(stage, target) for stage, target in stages)
        except BaseException as primary_error:
            self._disable_after_failure(enable_attempted, primary_error)
            raise
        else:
            self._disable_after_success(enable_attempted)
            self._last_disable_confirmed = True
            return results

    def _run_stage(self, stage: str, target_position_rad: float) -> MotionStageResult:
        """在固定持续时间内发送阻抗平衡点，并持续检查反馈。"""
        started_at_s = self._clock()
        deadline = started_at_s + self._safety.stage_duration_s
        command_count = 0
        first_position_rad: float | None = None
        last_feedback: MotorFeedback | None = None
        max_position_rad: float | None = None
        min_position_rad: float | None = None
        max_abs_velocity_rad_s = 0.0
        max_abs_torque_nm = 0.0
        while True:
            if command_count > 0 and self._clock() >= deadline:
                assert first_position_rad is not None
                assert last_feedback is not None
                assert max_position_rad is not None
                assert min_position_rad is not None
                position_error_rad = target_position_rad - last_feedback.position_rad
                return MotionStageResult(
                    stage=stage,
                    target_position_rad=target_position_rad,
                    target_reached=abs(position_error_rad) <= self._safety.position_tolerance_rad,
                    position_error_rad=position_error_rad,
                    start_position_rad=first_position_rad,
                    final_position_rad=last_feedback.position_rad,
                    max_position_rad=max_position_rad,
                    min_position_rad=min_position_rad,
                    max_abs_velocity_rad_s=max_abs_velocity_rad_s,
                    max_abs_torque_nm=max_abs_torque_nm,
                    command_count=command_count,
                    elapsed_s=self._clock() - started_at_s,
                )
            self._write_packet(
                self._protocol.make_mit_packet(
                    self._deployment.motor_id,
                    MitCommand(
                        position_rad=target_position_rad,
                        velocity_rad_s=0.0,
                        torque_nm=0.0,
                        kp=self._safety.mit_kp,
                        kd=self._safety.mit_kd,
                    ),
                )
            )
            command_count += 1
            try:
                feedback = self._receiver.receive_feedback()
            except TimeoutError as exc:
                raise self._stage_timeout_error(
                    stage,
                    command_count,
                    first_position_rad,
                    last_feedback,
                ) from exc
            self._validate_feedback(feedback)
            if first_position_rad is None:
                first_position_rad = feedback.position_rad
            last_feedback = feedback
            max_position_rad = (
                feedback.position_rad
                if max_position_rad is None
                else max(feedback.position_rad, max_position_rad)
            )
            min_position_rad = (
                feedback.position_rad
                if min_position_rad is None
                else min(feedback.position_rad, min_position_rad)
            )
            max_abs_velocity_rad_s = max(max_abs_velocity_rad_s, abs(feedback.velocity_rad_s))
            max_abs_torque_nm = max(max_abs_torque_nm, abs(feedback.torque_nm))
            if feedback.status_code != STATUS_ENABLED:
                raise MotionSafetyError(
                    f"使能确认失败：期望 status_code={STATUS_ENABLED}，实际为 "
                    f"{feedback.status_code}"
                )
            self._sleep(self._safety.control_interval_s)

    def _stage_timeout_error(
        self,
        stage: str,
        command_count: int,
        first_position_rad: float | None,
        last_feedback: MotorFeedback | None,
    ) -> MotionStageTimeoutError:
        """构造包含阶段反馈证据的通信超时异常。"""
        first = "无" if first_position_rad is None else f"{first_position_rad:.9f}"
        if last_feedback is None:
            last_position = last_velocity = last_torque = "无"
        else:
            last_position = f"{last_feedback.position_rad:.9f}"
            last_velocity = f"{last_feedback.velocity_rad_s:.9f}"
            last_torque = f"{last_feedback.torque_nm:.9f}"
        return MotionStageTimeoutError(
            f"{stage} 阶段等待反馈超时："
            f"首位置={first} rad，末位置={last_position} rad，"
            f"末速度={last_velocity} rad/s，"
            f"末力矩={last_torque} N·m，命令次数={command_count}"
        )

    def _validate_plan(self, plan: MotionProbePlan) -> None:
        """验证计划目标仍严格位于当前机械行程内。"""
        for target in (plan.hold_target_rad, plan.close_target_rad, plan.return_target_rad):
            self._deployment.validate_joint_position(target)
        if (
            plan.hold_target_rad != plan.initial_position_rad
            or plan.return_target_rad != plan.initial_position_rad
        ):
            raise MotionSafetyError("保持和回位目标必须精确等于记录的初始位置")
        expected_close = (
            plan.initial_position_rad
            + self._deployment.closing_direction * self._safety.closing_step_rad
        )
        if plan.close_target_rad != expected_close:
            raise MotionSafetyError("闭合目标必须为初始位置加配置的闭合步长")

    def _validate_feedback(self, feedback: MotorFeedback) -> None:
        """检查每条反馈的故障与机械位置硬边界。"""
        if motor_status_is_fault(feedback.status_code):
            raise MotionSafetyError(f"电机反馈故障：status_code={feedback.status_code}")
        try:
            self._deployment.validate_joint_position(feedback.position_rad)
        except ValueError as exc:
            raise MotionSafetyError(
                f"电机反馈位置超出机械行程：{feedback.position_rad} rad"
            ) from exc

    def _write_packet(self, packet: bytes) -> None:
        """要求传输完整接受一帧控制或 MIT 命令。"""
        written = self._transport.write(packet, timeout_s=self._deployment.device.timeout_s)
        if written != len(packet):
            raise ShortWriteError(f"运动探针命令短写：期望 {len(packet)} 字节，实际 {written} 字节")

    def _disable_after_success(self, enable_attempted: bool) -> None:
        """正常完成后要求失能帧完整写入，失败不得报告成功。"""
        if not enable_attempted:
            return
        try:
            self._write_packet(
                self._protocol.make_control_packet(self._deployment.motor_id, CMD_DISABLE)
            )
            feedback = self._refresher.refresh_once()
            if feedback.status_code != STATUS_DISABLED:
                raise MotionDisableError(
                    f"最终失能未确认：期望 status_code={STATUS_DISABLED}，实际为 {feedback.status_code}"
                )
        except Exception as exc:  # noqa: BLE001
            raise MotionDisableError(f"运动阶段已完成，但最终失能失败：{exc}") from exc

    def _disable_after_failure(self, enable_attempted: bool, primary_error: BaseException) -> None:
        """异常路径尽力失能；失败时保留原始失败的可诊断文本。"""
        if not enable_attempted:
            return
        try:
            self._write_packet(
                self._protocol.make_control_packet(self._deployment.motor_id, CMD_DISABLE)
            )
            try:
                feedback = self._refresher.refresh_once()
                if feedback.status_code != STATUS_DISABLED:
                    raise MotionDisableError(
                        f"异常后的失能未确认：实际 status_code={feedback.status_code}"
                    )
            except Exception as verification_error:  # noqa: BLE001
                raise MotionDisableError(
                    f"异常后的失能验证失败：{verification_error}"
                ) from verification_error
        except Exception as disable_error:  # noqa: BLE001
            raise MotionDisableError(
                f"运动失败后最终失能也失败：{disable_error}；原始失败：{primary_error}"
            ) from disable_error

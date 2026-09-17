"""DM4310P 串口会话：协议对象组装与显式设备生命周期。"""

from __future__ import annotations

import math

from dm_grasp_core import MITCommand

from .adapter import DmMitCommandAdapter
from .deployment import Dm4310PGripperConfig, make_dm4310p_gripper_config
from .protocol import (
    CMD_DISABLE,
    CMD_ENABLE,
    CONTROL_MODE_REGISTER,
    STATUS_DISABLED,
    STATUS_ENABLED,
    MotorFeedback,
    Usb2CanProtocol,
    motor_status_is_fault,
)
from .runtime import DmRegisterReader, DmResponseReceiver, DmStateRefresher
from .serial_transport import PySerialTransport
from .transport import ByteTransport


class DmSession:
    """由单个控制线程独占的 DM4310P 串口会话。

    构造只组装协议与传输对象。所有设备 I/O 都要求调用方显式触发；本类
    不拥有实验阶段、故障处理策略、自动回零轨迹或运行记录。

    Args:
        port: USB2CAN 串口端点。
        timeout_s: 单次写入或反馈等待的总时限（秒）。
        transport: 可选字节传输；测试可注入纯内存实现。
        feedback_position_margin_rad: 反馈安全范围在命令工作行程两端的余量（rad）。
    """

    def __init__(
        self,
        port: str,
        timeout_s: float,
        *,
        transport: ByteTransport | None = None,
        feedback_position_margin_rad: float = 0.05,
    ) -> None:
        """构造协议对象，不打开串口。"""
        self.deployment: Dm4310PGripperConfig = make_dm4310p_gripper_config(
            port,
            timeout_s=timeout_s,
            feedback_position_margin_rad=feedback_position_margin_rad,
        )
        self.protocol = Usb2CanProtocol(self.deployment.motor_limits)
        self.transport = transport or PySerialTransport()
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
        self.last_feedback: MotorFeedback | None = None

    def open(self) -> None:
        """打开 DM 串口。"""
        self.refresher.open()

    def close(self) -> None:
        """关闭 DM 串口，不隐式发送控制命令。"""
        self.refresher.close()

    def inspect(self) -> MotorFeedback:
        """核对初始反馈和 MIT 模式。"""
        self._accept_feedback(self.refresher.refresh_once())
        mode = self.registers.read_u32(CONTROL_MODE_REGISTER)
        if mode != 1:
            raise RuntimeError(f"控制模式不是 MIT：寄存器 {CONTROL_MODE_REGISTER}={mode}")
        return self._accept_feedback(self.refresher.refresh_once())

    def require_disabled(self) -> MotorFeedback:
        """确认电机当前处于失能状态。"""
        feedback = self._accept_feedback(self.refresher.refresh_once())
        if feedback.status_code != STATUS_DISABLED:
            raise RuntimeError("开始前电机必须处于失能状态")
        return feedback

    def enable(self) -> MotorFeedback:
        """使能并要求该命令产生新的使能反馈。"""
        self._write(self.protocol.make_control_packet(self.deployment.motor_id, CMD_ENABLE))
        feedback = self._accept_feedback(self.receiver.receive_feedback())
        if feedback.status_code != STATUS_ENABLED:
            raise RuntimeError(f"DM 使能确认失败：status_code={feedback.status_code}")
        return feedback

    def command(self, command: MITCommand) -> MotorFeedback:
        """发送工作范围内的 MIT 请求并返回该命令产生的新反馈。

        协议编码器保留既有的固件量程饱和语义；会话在编码前额外拒绝超出
        夹爪命令工作范围的位置目标，避免调用方绕过共享控制核的机械边界。
        """
        self.deployment.validate_joint_position(command.position_rad)
        self._validate_command_fields(command)
        self._write(self.adapter.prepare(command).frame)
        feedback = self._accept_feedback(self.receiver.receive_feedback())
        if feedback.status_code != STATUS_ENABLED:
            raise RuntimeError(f"DM 运行中失能：status_code={feedback.status_code}")
        return feedback

    def hold(self, command: MITCommand) -> MotorFeedback:
        """验证并发送当前位置保持请求。

        保持请求的位置应由调用方基于最新有效反馈生成，并投影到命令工作范围；
        会话强制要求零目标速度和零前馈力矩，再复用普通命令的写入与反馈检查。

        Args:
            command: 已由共享控制核生成的受限 MIT 保持请求。

        Returns:
            保持命令产生的新使能反馈。

        Raises:
            RuntimeError: 尚无有效使能反馈。
            ValueError: 位置不在命令工作范围，或不是零速度、零前馈力矩请求。
        """
        feedback = self.last_feedback
        if feedback is None:
            raise RuntimeError("尚无有效 DM 反馈，无法发送保持命令")
        self._validate_feedback(feedback)
        if feedback.status_code != STATUS_ENABLED:
            raise RuntimeError("DM 未处于使能状态，无法发送保持命令")
        self.deployment.validate_joint_position(command.position_rad)
        expected_position_rad = min(
            max(feedback.position_rad, self.deployment.joint_position_min_rad),
            self.deployment.joint_position_max_rad,
        )
        position_resolution_rad = (
            self.deployment.motor_limits.position_max_rad
            - self.deployment.motor_limits.position_min_rad
        ) / 65535.0
        if not math.isclose(
            command.position_rad,
            expected_position_rad,
            abs_tol=position_resolution_rad,
        ):
            raise ValueError("DM 保持命令目标必须等于最新反馈位置在工作范围内的投影")
        if command.velocity_rad_s != 0.0:
            raise ValueError("DM 保持命令的目标速度必须为 0")
        if command.feedforward_torque_nm != 0.0:
            raise ValueError("DM 保持命令的前馈力矩必须为 0")
        return self.command(command)

    def disable(self) -> MotorFeedback:
        """失能并以该命令自身的反馈确认状态码。"""
        self._write(self.protocol.make_control_packet(self.deployment.motor_id, CMD_DISABLE))
        # 失能命令自身会回复；追加查询会残留一条反馈，使再次使能的确认错位。
        feedback = self._accept_feedback(self.receiver.receive_feedback())
        if feedback.status_code != STATUS_DISABLED:
            raise RuntimeError(f"DM 最终失能确认失败：status_code={feedback.status_code}")
        return feedback

    def _write(self, frame: bytes) -> None:
        """要求 USB2CAN 完整接受一帧。"""
        written = self.transport.write(frame, timeout_s=self.deployment.device.timeout_s)
        if written != len(frame):
            raise OSError(f"DM 命令短写：期望 {len(frame)} 字节，实际 {written} 字节")

    def _accept_feedback(self, feedback: MotorFeedback) -> MotorFeedback:
        """验证反馈并保存为最近一次有效反馈。"""
        self._validate_feedback(feedback)
        self.last_feedback = feedback
        return feedback

    def _validate_feedback(self, feedback: MotorFeedback) -> None:
        """拒绝故障与扩展反馈安全范围外的位置。"""
        if motor_status_is_fault(feedback.status_code):
            raise RuntimeError(f"DM 电机故障：status_code={feedback.status_code}")
        self.deployment.validate_feedback_position(feedback.position_rad)

    def _validate_command_fields(self, command: MITCommand) -> None:
        """拒绝会被协议静默饱和并改变安全计算语义的 MIT 字段。"""
        limits = self.deployment.motor_limits
        fields = (
            (
                "目标速度",
                command.velocity_rad_s,
                limits.velocity_min_rad_s,
                limits.velocity_max_rad_s,
            ),
            ("位置增益 kp", command.kp, 0.0, 500.0),
            ("阻尼增益 kd", command.kd, 0.0, 5.0),
            (
                "前馈力矩",
                command.feedforward_torque_nm,
                limits.torque_min_nm,
                limits.torque_max_nm,
            ),
        )
        for name, value, lower, upper in fields:
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f"DM {name}必须位于协议范围 [{lower}, {upper}]")

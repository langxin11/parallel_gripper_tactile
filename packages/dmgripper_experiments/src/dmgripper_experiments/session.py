"""DM4310P 串口会话：协议对象组装与显式生命周期。

会话由控制线程独占；所有 I/O 都要求显式调用，构造不打开串口。
"""

from __future__ import annotations

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

from dm_grasp_core import MITCommand


class DmSession:
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

    def require_disabled(self) -> MotorFeedback:
        """确认电机当前处于失能状态。"""
        feedback = self.refresher.refresh_once()
        self._validate_feedback(feedback)
        if feedback.status_code != STATUS_DISABLED:
            raise RuntimeError("开始前电机必须处于失能状态")
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

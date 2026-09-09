"""将共享 DMgripper 控制核的命令适配为 USB2CAN 协议帧。"""

from __future__ import annotations

from dataclasses import dataclass

from dm_grasp_core import MITCommand as CoreMITCommand

from .protocol import USB2CAN_TX_FRAME_SIZE, MitCommand, MotorFeedback, Usb2CanProtocol


@dataclass(frozen=True, slots=True)
class PreparedMitCommand:
    """一次离线 MIT 命令准备的不可变结果。

    该回执保留控制核生成的原始请求、适配后的协议命令以及最终 USB2CAN 帧，便于由上层在
    生命周期与安全互锁完成后审计或发送。本类型不持有传输对象，也不执行 I/O。

    Attributes:
        core_command: `dm_grasp_core` 生成的未量化 MIT 请求。
        protocol_command: 显式映射后的 USB2CAN MIT 命令。
        frame: 长度固定为 30 字节的 USB2CAN 发送帧。
    """

    core_command: CoreMITCommand
    protocol_command: MitCommand
    frame: bytes

    def __post_init__(self) -> None:
        """验证回执中的协议帧长度。"""
        if len(self.frame) != USB2CAN_TX_FRAME_SIZE:
            raise ValueError("MIT 命令回执必须包含 30 字节 USB2CAN 帧")


@dataclass(frozen=True, slots=True)
class CoreMotorObservation:
    """供 DMgripper 控制核下一步使用的显式电机观测。

    `dm_grasp_core` 当前没有统一的观测接口。该 DTO 仅保留其 `build_mit_command` 与
    `step_admittance` 所需的实测位置、速度；力矩、状态码和触觉数据仍由上层按各自边界处理。

    Attributes:
        position_rad: 实测电机角位置（rad）。
        velocity_rad_s: 实测电机角速度（rad/s）。
    """

    position_rad: float
    velocity_rad_s: float


def map_core_mit_command(command: CoreMITCommand) -> MitCommand:
    """显式映射共享控制核的 MIT 请求到 USB2CAN MIT 命令。

    两个同名 `MITCommand` 类型有意隔离，避免控制核泄漏协议细节。字段语义逐一保持：
    `position_rad` 是目标位置，`velocity_rad_s` 是目标速度，`kp` 与 `kd` 分别是位置刚度和
    速度阻尼，`feedforward_torque_nm` 映射到 USB2CAN 协议的 `torque_nm` 前馈力矩字段。
    本函数不进行量化、限幅或 I/O；这些行为由 `Usb2CanProtocol` 保持既有语义。

    Args:
        command: 共享控制核生成的未量化 MIT 请求。

    Returns:
        MitCommand: 供 USB2CAN 协议编码的独立命令对象。
    """
    return MitCommand(
        position_rad=command.position_rad,
        velocity_rad_s=command.velocity_rad_s,
        torque_nm=command.feedforward_torque_nm,
        kp=command.kp,
        kd=command.kd,
    )


def core_observation_from_feedback(feedback: MotorFeedback) -> CoreMotorObservation:
    """从 USB2CAN 反馈提取控制核下一步所需的位姿观测。

    Args:
        feedback: 已由 `Usb2CanProtocol` 解码的电机反馈。

    Returns:
        CoreMotorObservation: 只含实测位置与速度的控制核观测 DTO。
    """
    return CoreMotorObservation(
        position_rad=feedback.position_rad,
        velocity_rad_s=feedback.velocity_rad_s,
    )


class DmMitCommandAdapter:
    """把 DMgripper 控制核命令离线准备为指定 CAN ID 的 MIT 帧。

    适配器只依赖无状态 `Usb2CanProtocol`，因此不会打开端口、写入传输或改变设备生命周期。

    Args:
        protocol: 使用已核验电机量程创建的 USB2CAN 编解码器。
        motor_id: 目标电机的 CAN ID。
    """

    def __init__(self, protocol: Usb2CanProtocol, motor_id: int) -> None:
        """保存协议编码器与目标电机 CAN ID。"""
        self._protocol = protocol
        self._motor_id = motor_id

    @property
    def motor_id(self) -> int:
        """返回准备命令使用的目标 CAN ID。"""
        return self._motor_id

    def prepare(self, command: CoreMITCommand) -> PreparedMitCommand:
        """映射并编码命令，但不向任何传输写入帧。

        `Usb2CanProtocol.make_mit_packet` 继续负责 CAN ID、NaN 和协议量程的验证与量化。

        Args:
            command: 共享控制核生成的未量化 MIT 请求。

        Returns:
            PreparedMitCommand: 含原始请求、协议命令与 30 字节帧的不可变回执。

        Raises:
            ValueError: CAN ID 或命令数值不符合既有协议要求。
        """
        protocol_command = map_core_mit_command(command)
        frame = self._protocol.make_mit_packet(self._motor_id, protocol_command)
        return PreparedMitCommand(command, protocol_command, frame)

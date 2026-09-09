"""编码和解码 DM USB2CAN 协议帧。"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

USB2CAN_TX_FRAME_SIZE = 30
"""USB2CAN 发送帧长度（字节）。"""

USB2CAN_RX_FRAME_SIZE = 16
"""USB2CAN 反馈帧长度（字节）。"""

USB2CAN_RX_BUFFER_MAX_SIZE = 4096
"""与 ROS 2 C++ 驱动一致的接收缓存上限（字节）。"""

CAN_BROADCAST_ID = 0x7FF
"""达妙参数读写与状态刷新使用的广播 CAN ID。"""

CMD_ENABLE = 0xFC
CMD_DISABLE = 0xFD
CMD_SET_ZERO = 0xFE
CONTROL_MODE_REGISTER = 10

STATUS_DISABLED = 0x0
STATUS_ENABLED = 0x1

_USB2CAN_TX_TEMPLATE = bytes(
    [
        0x55,
        0xAA,
        0x1E,
        0x03,
        0x01,
        0x00,
        0x00,
        0x00,
        0x0A,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x08,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    ]
)


@dataclass(frozen=True, slots=True)
class MotorLimits:
    """MIT 协议定点量化使用的固件物理量程。

    调用方必须从电机固件读回值或已验证的部署配置提供全部量程。本类故意不提供
    DM4310P 的默认值：型号名不能替代具体电机已烧写的 PMAX、VMAX 与 TMAX。

    Args:
        position_min_rad: 位置编码下界（rad）。
        position_max_rad: 位置编码上界（rad）。
        velocity_min_rad_s: 速度编码下界（rad/s）。
        velocity_max_rad_s: 速度编码上界（rad/s）。
        torque_min_nm: 力矩编码下界（N·m）。
        torque_max_nm: 力矩编码上界（N·m）。
    """

    position_min_rad: float
    position_max_rad: float
    velocity_min_rad_s: float
    velocity_max_rad_s: float
    torque_min_nm: float
    torque_max_nm: float

    def __post_init__(self) -> None:
        """拒绝非有限或退化量程。"""
        pairs = (
            ("位置", self.position_min_rad, self.position_max_rad),
            ("速度", self.velocity_min_rad_s, self.velocity_max_rad_s),
            ("力矩", self.torque_min_nm, self.torque_max_nm),
        )
        for name, minimum, maximum in pairs:
            if not math.isfinite(minimum) or not math.isfinite(maximum):
                raise ValueError(f"{name}量程必须为有限数值")
            if minimum >= maximum:
                raise ValueError(f"{name}量程下界必须小于上界")


@dataclass(frozen=True, slots=True)
class MitCommand:
    """MIT 控制命令。"""

    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    kp: float
    kd: float


@dataclass(frozen=True, slots=True)
class PositionVelocityCommand:
    """位置速度控制命令。"""

    position_rad: float
    velocity_limit_rad_s: float


@dataclass(frozen=True, slots=True)
class VelocityCommand:
    """速度控制命令。"""

    velocity_rad_s: float


@dataclass(frozen=True, slots=True)
class TorqueCommand:
    """力矩控制命令。"""

    torque_nm: float


@dataclass(frozen=True, slots=True)
class ForcePositionCommand:
    """力位混合控制命令。"""

    position_rad: float
    velocity_limit_rad_s: float
    torque_limit_ratio: float


@dataclass(frozen=True, slots=True)
class MotorFeedback:
    """USB2CAN 反馈帧中的电机状态。"""

    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    status_code: int


@dataclass(frozen=True, slots=True)
class RegisterReply:
    """电机寄存器读写的回复。"""

    motor_id: int
    register_id: int
    value: int


def motor_status_is_fault(status_code: int) -> bool:
    """判断达妙反馈状态码是否为故障。

    官方协议只将 0（失能）和 1（使能）定义为正常状态，其余值按故障处理。

    Args:
        status_code: 反馈帧 `D[0]` 的高四位。

    Returns:
        非正常状态码时为 `True`。
    """
    _validate_uint(status_code, bits=4, name="状态码")
    return status_code not in (STATUS_DISABLED, STATUS_ENABLED)


def motor_status_to_fault_code(status_code: int) -> int:
    """将正常使能状态映射为零故障码。"""
    return status_code if motor_status_is_fault(status_code) else 0


class Usb2CanProtocol:
    """DM USB2CAN 协议的无状态编解码器。

    该类只创建或解释 `bytes`，从不打开串口、不会向任何设备发送命令。位置、速度与
    力矩的 MIT 量化按传入的 `MotorLimits` 饱和；调用方仍须在控制层完成安全范围、
    使能状态、命令时效与实机急停策略。

    Args:
        limits: 与电机固件一致的 MIT 编码量程。
    """

    def __init__(self, limits: MotorLimits) -> None:
        """使用给定的固件量程创建编解码器。"""
        self._limits = limits

    @property
    def limits(self) -> MotorLimits:
        """返回创建该编解码器时的不可变量程。"""
        return self._limits

    def make_mit_packet(self, can_id: int, command: MitCommand) -> bytes:
        """编码 MIT 命令为 USB2CAN 发送帧。"""
        position = _float_to_uint(
            command.position_rad,
            self._limits.position_min_rad,
            self._limits.position_max_rad,
            16,
        )
        velocity = _float_to_uint(
            command.velocity_rad_s,
            self._limits.velocity_min_rad_s,
            self._limits.velocity_max_rad_s,
            12,
        )
        kp = _float_to_uint(command.kp, 0.0, 500.0, 12)
        kd = _float_to_uint(command.kd, 0.0, 5.0, 12)
        torque = _float_to_uint(
            command.torque_nm,
            self._limits.torque_min_nm,
            self._limits.torque_max_nm,
            12,
        )
        payload = bytes(
            (
                position >> 8,
                position & 0xFF,
                velocity >> 4,
                ((velocity & 0xF) << 4) | (kp >> 8),
                kp & 0xFF,
                kd >> 4,
                ((kd & 0xF) << 4) | (torque >> 8),
                torque & 0xFF,
            )
        )
        return self._make_packet(can_id, payload)

    def make_position_velocity_packet(self, can_id: int, command: PositionVelocityCommand) -> bytes:
        """编码位置速度模式命令，CAN ID 使用 `0x100 + can_id`。"""
        payload = _float_to_le_bytes(command.position_rad) + _float_to_le_bytes(
            command.velocity_limit_rad_s
        )
        return self._make_packet(_offset_can_id(can_id, 0x100), payload)

    def make_velocity_packet(self, can_id: int, command: VelocityCommand) -> bytes:
        """编码速度模式命令，CAN ID 使用 `0x200 + can_id`。"""
        return self._make_packet(
            _offset_can_id(can_id, 0x200), _float_to_le_bytes(command.velocity_rad_s) + bytes(4)
        )

    def make_position_velocity_csp_packet(
        self, can_id: int, command: PositionVelocityCommand
    ) -> bytes:
        """编码位置速度 CSP 命令，CAN ID 使用 `0x400 + can_id`。"""
        payload = _float_to_le_bytes(command.position_rad) + _float_to_le_bytes(
            command.velocity_limit_rad_s
        )
        return self._make_packet(_offset_can_id(can_id, 0x400), payload)

    def make_velocity_csp_packet(self, can_id: int, command: VelocityCommand) -> bytes:
        """编码速度 CSP 命令，CAN ID 使用 `0x500 + can_id`。"""
        return self._make_packet(
            _offset_can_id(can_id, 0x500), _float_to_le_bytes(command.velocity_rad_s) + bytes(4)
        )

    def make_torque_csp_packet(self, can_id: int, command: TorqueCommand) -> bytes:
        """编码力矩 CSP 命令，CAN ID 使用 `0x600 + can_id`。"""
        return self._make_packet(
            _offset_can_id(can_id, 0x600), _float_to_le_bytes(command.torque_nm) + bytes(4)
        )

    def make_force_position_packet(self, can_id: int, command: ForcePositionCommand) -> bytes:
        """编码力位混合命令，CAN ID 使用 `0x300 + can_id`。"""
        if not math.isfinite(command.velocity_limit_rad_s) or not (
            0.0 <= command.velocity_limit_rad_s <= 100.0
        ):
            raise ValueError("力位混合速度上限必须位于 [0, 100] rad/s")
        if not math.isfinite(command.torque_limit_ratio) or not (
            0.0 <= command.torque_limit_ratio <= 1.0
        ):
            raise ValueError("力位混合电流限制比例必须位于 [0, 1]")

        velocity = _round_nonnegative(command.velocity_limit_rad_s * 100.0)
        torque = _round_nonnegative(command.torque_limit_ratio * 10000.0)
        payload = _float_to_le_bytes(command.position_rad) + bytes(
            (velocity & 0xFF, velocity >> 8, torque & 0xFF, torque >> 8)
        )
        return self._make_packet(_offset_can_id(can_id, 0x300), payload)

    def make_control_packet(self, can_id: int, command: int) -> bytes:
        """编码 8 字节全 `0xFF` 结尾控制命令帧。"""
        _validate_uint(command, bits=8, name="控制命令")
        return self._make_packet(can_id, bytes((0xFF,) * 7 + (command,)))

    def make_feedback_request(self, can_id: int) -> bytes:
        """编码状态刷新请求，外层 CAN ID 固定为广播 ID `0x7FF`。"""
        _validate_can_id(can_id)
        payload = bytes((can_id & 0xFF, can_id >> 8, 0xCC, 0, 0, 0, 0, 0))
        return self._make_packet(CAN_BROADCAST_ID, payload)

    def make_read_register_packet(self, motor_id: int, register_id: int) -> bytes:
        """编码寄存器读取请求，外层 CAN ID 固定为广播 ID `0x7FF`。"""
        _validate_can_id(motor_id)
        _validate_uint(register_id, bits=8, name="寄存器 ID")
        payload = bytes((motor_id & 0xFF, motor_id >> 8, 0x33, register_id, 0, 0, 0, 0))
        return self._make_packet(CAN_BROADCAST_ID, payload)

    def make_write_register_u32_packet(self, motor_id: int, register_id: int, value: int) -> bytes:
        """编码 32 位无符号寄存器写入请求，数值按小端序放置。"""
        _validate_can_id(motor_id)
        _validate_uint(register_id, bits=8, name="寄存器 ID")
        _validate_uint(value, bits=32, name="寄存器值")
        payload = bytes(
            (
                motor_id & 0xFF,
                motor_id >> 8,
                0x55,
                register_id,
                value & 0xFF,
                (value >> 8) & 0xFF,
                (value >> 16) & 0xFF,
                (value >> 24) & 0xFF,
            )
        )
        return self._make_packet(CAN_BROADCAST_ID, payload)

    def parse_feedback(self, frame: bytes, motor_id: int, master_id: int) -> MotorFeedback | None:
        """解析一帧电机反馈；非目标帧返回 `None`。"""
        if not _is_valid_receive_frame(frame):
            return None
        _validate_can_id(motor_id)
        _validate_can_id(master_id)
        can_id = _receive_can_id(frame)
        if can_id not in (motor_id, master_id, 0):
            return None

        position = (frame[8] << 8) | frame[9]
        velocity = (frame[10] << 4) | (frame[11] >> 4)
        torque = ((frame[11] & 0xF) << 8) | frame[12]
        return MotorFeedback(
            position_rad=_uint_to_float(
                position,
                self._limits.position_min_rad,
                self._limits.position_max_rad,
                16,
            ),
            velocity_rad_s=_uint_to_float(
                velocity,
                self._limits.velocity_min_rad_s,
                self._limits.velocity_max_rad_s,
                12,
            ),
            torque_nm=_uint_to_float(
                torque,
                self._limits.torque_min_nm,
                self._limits.torque_max_nm,
                12,
            ),
            status_code=frame[7] >> 4,
        )

    def parse_register_reply(self, frame: bytes, master_id: int) -> RegisterReply | None:
        """解析寄存器读写回复；非目标帧返回 `None`。"""
        if not _is_valid_receive_frame(frame):
            return None
        _validate_can_id(master_id)
        if _receive_can_id(frame) not in (master_id, 0):
            return None
        if frame[9] not in (0x33, 0x55):
            return None

        return RegisterReply(
            motor_id=frame[7] | (frame[8] << 8),
            register_id=frame[10],
            value=frame[11] | (frame[12] << 8) | (frame[13] << 16) | (frame[14] << 24),
        )

    def encode_feedback_frame(self, can_id: int, feedback: MotorFeedback) -> bytes:
        """编码模拟或测试用反馈帧。

        此方法对应参考 C++ 驱动 `MockIo` 的反馈帧生成器，不会写入任何传输。
        """
        _validate_can_id(can_id)
        _validate_uint(feedback.status_code, bits=4, name="状态码")
        position = _float_to_uint(
            feedback.position_rad,
            self._limits.position_min_rad,
            self._limits.position_max_rad,
            16,
        )
        velocity = _float_to_uint(
            feedback.velocity_rad_s,
            self._limits.velocity_min_rad_s,
            self._limits.velocity_max_rad_s,
            12,
        )
        torque = _float_to_uint(
            feedback.torque_nm,
            self._limits.torque_min_nm,
            self._limits.torque_max_nm,
            12,
        )
        return bytes(
            (
                0xAA,
                0x11,
                0x08,
                can_id & 0xFF,
                can_id >> 8,
                0,
                0,
                feedback.status_code << 4,
                position >> 8,
                position & 0xFF,
                velocity >> 4,
                ((velocity & 0xF) << 4) | (torque >> 8),
                torque & 0xFF,
                0,
                0,
                0x55,
            )
        )

    def _make_packet(self, can_id: int, payload: bytes) -> bytes:
        """将 8 字节 CAN 载荷填入固定 USB2CAN 外层帧。"""
        _validate_can_id(can_id)
        if len(payload) != 8:
            raise ValueError("USB2CAN CAN 载荷必须恰为 8 字节")
        packet = bytearray(_USB2CAN_TX_TEMPLATE)
        packet[13] = can_id & 0xFF
        packet[14] = can_id >> 8
        packet[21:29] = payload
        return bytes(packet)


class Usb2CanFrameExtractor:
    """按 16 字节 USB2CAN 接收帧边界重组分段字节流。"""

    def __init__(self, max_buffer_size: int = USB2CAN_RX_BUFFER_MAX_SIZE) -> None:
        """创建接收缓存。

        Args:
            max_buffer_size: 超过该总长度时清空缓存，与参考 C++ 驱动一致。
        """
        if max_buffer_size < USB2CAN_RX_FRAME_SIZE:
            raise ValueError("接收缓存上限不能小于一个完整反馈帧")
        self._max_buffer_size = max_buffer_size
        self._buffer = bytearray()

    @property
    def buffered_byte_count(self) -> int:
        """返回尚未组成完整帧的缓存字节数。"""
        return len(self._buffer)

    def feed(self, received: bytes) -> tuple[bytes, ...]:
        """写入新接收字节并返回完整、头尾正确的帧。"""
        if len(self._buffer) + len(received) > self._max_buffer_size:
            self._buffer.clear()
        self._buffer.extend(received)

        frames: list[bytes] = []
        while len(self._buffer) >= USB2CAN_RX_FRAME_SIZE:
            if self._buffer[0] != 0xAA or self._buffer[USB2CAN_RX_FRAME_SIZE - 1] != 0x55:
                del self._buffer[0]
                continue
            frames.append(bytes(self._buffer[:USB2CAN_RX_FRAME_SIZE]))
            del self._buffer[:USB2CAN_RX_FRAME_SIZE]
        return tuple(frames)


def _float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    """按参考 C++ 驱动规则饱和并量化浮点数。"""
    if not math.isfinite(value):
        raise ValueError("电机命令包含非有限数值")
    clamped = min(max(value, minimum), maximum)
    return _round_nonnegative((clamped - minimum) * ((1 << bits) - 1) / (maximum - minimum))


def _uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    """将协议无符号定点数还原为物理量。"""
    return minimum + value * (maximum - minimum) / ((1 << bits) - 1)


def _float_to_le_bytes(value: float) -> bytes:
    """按小端 IEEE-754 单精度编码有限数值。"""
    if not math.isfinite(value):
        raise ValueError("电机命令包含非有限数值")
    try:
        return struct.pack("<f", value)
    except OverflowError as error:
        raise ValueError("电机命令超出单精度浮点范围") from error


def _round_nonnegative(value: float) -> int:
    """复现 C++ `std::llround` 在非负值上的最近整数规则。"""
    return math.floor(value + 0.5)


def _validate_can_id(can_id: int) -> None:
    """验证参考协议 API 使用的 16 位 CAN ID 容器。"""
    _validate_uint(can_id, bits=16, name="CAN ID")


def _offset_can_id(can_id: int, offset: int) -> int:
    """将模式偏移加入 16 位 CAN ID，并拒绝参考 API 无法表示的结果。"""
    _validate_can_id(can_id)
    result = can_id + offset
    _validate_can_id(result)
    return result


def _validate_uint(value: int, *, bits: int, name: str) -> None:
    """验证 Python 输入可无损映射为协议无符号整数。"""
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < (1 << bits):
        raise ValueError(f"{name}必须位于 [0, {(1 << bits) - 1}]")


def _is_valid_receive_frame(frame: bytes) -> bool:
    """检查固定长度、帧头、命令字和帧尾。"""
    return (
        len(frame) == USB2CAN_RX_FRAME_SIZE
        and frame[0] == 0xAA
        and frame[1] == 0x11
        and frame[-1] == 0x55
    )


def _receive_can_id(frame: bytes) -> int:
    """从接收帧第 3 至 6 字节按小端序读取 CAN ID。"""
    return frame[3] | (frame[4] << 8) | (frame[5] << 16) | (frame[6] << 24)

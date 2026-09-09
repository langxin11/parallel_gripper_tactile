"""验证 DM USB2CAN 协议的离线编解码与分帧。"""

from __future__ import annotations

import math
import struct

import pytest

from dmgripper_hardware import (
    CAN_BROADCAST_ID,
    CMD_ENABLE,
    ForcePositionCommand,
    MitCommand,
    MotorFeedback,
    MotorLimits,
    PositionVelocityCommand,
    STATUS_DISABLED,
    STATUS_ENABLED,
    TorqueCommand,
    Usb2CanFrameExtractor,
    Usb2CanProtocol,
    VelocityCommand,
    motor_status_is_fault,
    motor_status_to_fault_code,
)


@pytest.fixture
def protocol() -> Usb2CanProtocol:
    """提供参考 ROS 2 部署配置对应的纯协议实例。"""
    return Usb2CanProtocol(MotorLimits(-1.7, 1.7, -8.0, 8.0, -4.0, 4.0))


def test_mit_packet_matches_fixed_usb2can_layout(protocol: Usb2CanProtocol) -> None:
    """MIT 中点命令保留固定外层、CAN ID 小端序与 12 位字段布局。"""
    packet = protocol.make_mit_packet(0x01, MitCommand(0.0, 0.0, 0.0, 0.0, 0.0))

    assert packet == bytes(
        (
            0x55,
            0xAA,
            0x1E,
            0x03,
            0x01,
            0,
            0,
            0,
            0x0A,
            0,
            0,
            0,
            0,
            0x01,
            0,
            0,
            0,
            0,
            0x08,
            0,
            0,
            0x80,
            0,
            0x80,
            0,
            0,
            0,
            0x08,
            0,
            0,
        )
    )


def test_mit_quantization_saturates_to_firmware_limits(protocol: Usb2CanProtocol) -> None:
    """MIT 物理量超出量程时与参考 C++ 编码器一致地饱和。"""
    packet = protocol.make_mit_packet(1, MitCommand(3.0, -20.0, 10.0, 600.0, -1.0))

    assert packet[21:29] == bytes((0xFF, 0xFF, 0x00, 0x0F, 0xFF, 0x00, 0x0F, 0xFF))


def test_position_velocity_and_velocity_packets_use_id_offsets_and_le_floats(
    protocol: Usb2CanProtocol,
) -> None:
    """其他控制律分别使用 `0x100`、`0x200` ID 偏移和小端单精度载荷。"""
    position_velocity = protocol.make_position_velocity_packet(
        1, PositionVelocityCommand(1.25, 2.5)
    )
    velocity = protocol.make_velocity_packet(1, VelocityCommand(-3.0))

    assert position_velocity[13:15] == bytes((0x01, 0x01))
    assert struct.unpack("<ff", position_velocity[21:29]) == (1.25, 2.5)
    assert velocity[13:15] == bytes((0x01, 0x02))
    assert struct.unpack("<f", velocity[21:25]) == (-3.0,)
    assert velocity[25:29] == bytes(4)


def test_csp_packets_preserve_official_vendor_id_offsets(protocol: Usb2CanProtocol) -> None:
    """三种 CSP 控制律沿用官方 Python SDK 的 `0x400`、`0x500`、`0x600` 偏移。"""
    position_velocity = protocol.make_position_velocity_csp_packet(
        1, PositionVelocityCommand(1.25, 2.5)
    )
    velocity = protocol.make_velocity_csp_packet(1, VelocityCommand(-3.0))
    torque = protocol.make_torque_csp_packet(1, TorqueCommand(1.5))

    assert position_velocity[13:15] == bytes((0x01, 0x04))
    assert struct.unpack("<ff", position_velocity[21:29]) == (1.25, 2.5)
    assert velocity[13:15] == bytes((0x01, 0x05))
    assert struct.unpack("<f", velocity[21:25]) == (-3.0,)
    assert torque[13:15] == bytes((0x01, 0x06))
    assert struct.unpack("<f", torque[21:25]) == (1.5,)


def test_force_position_packet_scales_velocity_and_torque_ratio(protocol: Usb2CanProtocol) -> None:
    """力位混合命令保留 `0x300` ID 偏移和 100／10000 倍缩放。"""
    packet = protocol.make_force_position_packet(1, ForcePositionCommand(1.0, 2.5, 0.1))

    assert packet[13:15] == bytes((0x01, 0x03))
    assert struct.unpack("<f", packet[21:25]) == (1.0,)
    assert packet[25:29] == bytes((250, 0, 1000 & 0xFF, 1000 >> 8))


@pytest.mark.parametrize(
    ("command", "message"),
    [
        (ForcePositionCommand(0.0, -0.1, 0.5), "速度"),
        (ForcePositionCommand(0.0, 2.0, 1.1), "比例"),
    ],
)
def test_force_position_rejects_invalid_protocol_fields(
    protocol: Usb2CanProtocol, command: ForcePositionCommand, message: str
) -> None:
    """力位混合的协议定义域外输入应明确失败。"""
    with pytest.raises(ValueError, match=message):
        protocol.make_force_position_packet(1, command)


def test_protocol_rejects_nonfinite_float(protocol: Usb2CanProtocol) -> None:
    """协议编码器拒绝 `NaN`，不会悄然构造不确定帧。"""
    with pytest.raises(ValueError, match="非有限"):
        protocol.make_velocity_packet(1, VelocityCommand(math.nan))


def test_register_and_feedback_request_packets_use_broadcast_id(protocol: Usb2CanProtocol) -> None:
    """状态刷新与寄存器报文保留广播 ID 和小端整数布局。"""
    refresh = protocol.make_feedback_request(1)
    register = protocol.make_write_register_u32_packet(1, 10, 0x12345678)
    control = protocol.make_control_packet(1, CMD_ENABLE)

    assert refresh[13:15] == bytes((CAN_BROADCAST_ID & 0xFF, CAN_BROADCAST_ID >> 8))
    assert refresh[21:29] == bytes((1, 0, 0xCC, 0, 0, 0, 0, 0))
    assert register[13:15] == bytes((CAN_BROADCAST_ID & 0xFF, CAN_BROADCAST_ID >> 8))
    assert register[21:29] == bytes((1, 0, 0x55, 10, 0x78, 0x56, 0x34, 0x12))
    assert control[13:15] == bytes((1, 0))
    assert control[21:29] == bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, CMD_ENABLE))


def test_feedback_parser_decodes_fixed_frame_and_accepts_master_or_zero_id(
    protocol: Usb2CanProtocol,
) -> None:
    """反馈解码保留 16 字节帧、状态高四位和原有 CAN ID 接受规则。"""
    frame = bytes((0xAA, 0x11, 0x08, 0x01, 0, 0, 0, 0x10, 0x80, 0, 0x80, 0x08, 0, 0, 0, 0x55))
    feedback = protocol.parse_feedback(frame, motor_id=1, master_id=17)

    assert feedback is not None
    assert feedback.status_code == STATUS_ENABLED
    assert feedback.position_rad == pytest.approx(1.7 / 65535, abs=1e-12)
    assert feedback.velocity_rad_s == pytest.approx(8.0 / 4095, abs=1e-12)
    assert feedback.torque_nm == pytest.approx(4.0 / 4095, abs=1e-12)
    master_frame = bytearray(frame)
    master_frame[3] = 17
    assert protocol.parse_feedback(bytes(master_frame), 1, 17) is not None

    broadcast = bytearray(frame)
    broadcast[3:7] = bytes(4)
    assert protocol.parse_feedback(bytes(broadcast), 1, 17) is not None


def test_register_reply_parser_uses_feedback_layout_and_little_endian_value(
    protocol: Usb2CanProtocol,
) -> None:
    """寄存器回复复用接收帧并从数据区按官方小端序读取数值。"""
    frame = bytes((0xAA, 0x11, 0x08, 17, 0, 0, 0, 1, 0, 0x33, 10, 0x78, 0x56, 0x34, 0x12, 0x55))

    reply = protocol.parse_register_reply(frame, master_id=17)

    assert reply is not None
    assert (reply.motor_id, reply.register_id, reply.value) == (1, 10, 0x12345678)


def test_feedback_round_trip_is_bounded_by_quantization(protocol: Usb2CanProtocol) -> None:
    """模拟反馈编码和解析在各字段量化分辨率内互相一致。"""
    frame = protocol.encode_feedback_frame(1, MotorFeedback(1.0, 2.0, 3.0, STATUS_ENABLED))
    feedback = protocol.parse_feedback(frame, motor_id=1, master_id=17)

    assert feedback is not None
    assert feedback.position_rad == pytest.approx(1.0, abs=3.4 / 65535)
    assert feedback.velocity_rad_s == pytest.approx(2.0, abs=16.0 / 4095)
    assert feedback.torque_nm == pytest.approx(3.0, abs=8.0 / 4095)


def test_feedback_and_status_reject_invalid_data(protocol: Usb2CanProtocol) -> None:
    """反馈头尾错误应被忽略，故障状态保持保守语义。"""
    assert protocol.parse_feedback(bytes(16), 1, 17) is None
    assert motor_status_is_fault(STATUS_DISABLED) is False
    assert motor_status_is_fault(STATUS_ENABLED) is False
    assert motor_status_to_fault_code(STATUS_ENABLED) == 0
    assert motor_status_is_fault(0x8) is True
    assert motor_status_to_fault_code(0x8) == 0x8


def test_frame_extractor_handles_fragmentation_noise_and_overflow(
    protocol: Usb2CanProtocol,
) -> None:
    """分帧器按参考驱动的逐字节重同步策略处理串口分段与噪声。"""
    frame = protocol.encode_feedback_frame(1, MotorFeedback(0.0, 0.0, 0.0, STATUS_DISABLED))
    extractor = Usb2CanFrameExtractor()

    assert extractor.feed(b"\x00\x10" + frame[:7]) == ()
    assert extractor.feed(frame[7:]) == (frame,)
    assert extractor.buffered_byte_count == 0

    limited = Usb2CanFrameExtractor(max_buffer_size=16)
    assert limited.feed(b"x" * 16) == ()
    assert limited.feed(frame) == (frame,)

"""验证 DM4310P 会话下沉后的离线设备边界。"""

from __future__ import annotations

import math

import pytest
from dm_grasp_core import MITCommand

from dmgripper_hardware import (
    CMD_DISABLE,
    CMD_ENABLE,
    DmSession,
    FakeTransport,
    MotorFeedback,
    STATUS_DISABLED,
    STATUS_ENABLED,
)


def _feedback(session: DmSession, position_rad: float, status_code: int) -> bytes:
    """生成与当前会话量程一致的目标电机反馈帧。"""
    return session.protocol.encode_feedback_frame(
        session.deployment.motor_id,
        MotorFeedback(position_rad, 0.0, 0.0, status_code),
    )


def _register_reply(value: int) -> bytes:
    """生成电机 1、主机 17 的控制模式寄存器回复。"""
    return bytes((0xAA, 0x11, 0x08, 17, 0, 0, 0, 1, 0, 0x33, 10, value, 0, 0, 0, 0x55))


def _open_session(*, margin: float = 0.05) -> tuple[DmSession, FakeTransport]:
    """创建并打开只使用内存传输的会话。"""
    transport = FakeTransport()
    session = DmSession(
        "fake://usb2can",
        0.1,
        transport=transport,
        feedback_position_margin_rad=margin,
    )
    session.open()
    return session, transport


def test_constructor_is_io_free() -> None:
    """构造只组装对象，不打开或写入传输。"""
    transport = FakeTransport()
    session = DmSession("fake://usb2can", 0.1, transport=transport)

    assert not transport.is_open
    assert transport.written_payloads == []
    assert session.last_feedback is None


def test_inspect_accepts_slightly_negative_feedback_and_remembers_latest() -> None:
    """MIT 初检使用扩展反馈范围，同时保留最新有效反馈。"""
    session, transport = _open_session()
    transport.inject_received(
        _feedback(session, -0.02, STATUS_DISABLED)
        + _register_reply(1)
        + _feedback(session, -0.019, STATUS_DISABLED)
    )

    feedback = session.inspect()

    assert feedback.position_rad == pytest.approx(-0.019, abs=3.4 / 65535)
    assert session.last_feedback is feedback
    assert transport.written_payloads == [
        session.protocol.make_feedback_request(1),
        session.protocol.make_read_register_packet(1, 10),
        session.protocol.make_feedback_request(1),
    ]


def test_inspect_rejects_feedback_outside_extended_safety_range() -> None:
    """扩展反馈安全范围外的位置不得进入模式检查。"""
    session, transport = _open_session()
    transport.inject_received(_feedback(session, -0.06, STATUS_DISABLED))

    with pytest.raises(ValueError, match="反馈角"):
        session.inspect()

    assert session.last_feedback is None
    assert transport.written_payloads == [session.protocol.make_feedback_request(1)]


def test_command_rejects_position_outside_work_range_before_writing() -> None:
    """会话不得依赖协议饱和来处理机械工作范围外目标。"""
    session, transport = _open_session()

    with pytest.raises(ValueError, match="目标角"):
        session.command(MITCommand(-0.001, 0.0, 2.0, 0.5, 0.0))

    assert transport.written_payloads == []


@pytest.mark.parametrize(
    "command",
    [
        MITCommand(0.2, 8.01, 2.0, 0.5, 0.0),
        MITCommand(0.2, 0.0, 500.01, 0.5, 0.0),
        MITCommand(0.2, 0.0, 2.0, 5.01, 0.0),
        MITCommand(0.2, 0.0, 2.0, 0.5, 4.01),
    ],
)
def test_command_rejects_fields_that_protocol_would_saturate(command: MITCommand) -> None:
    """会话编码前拒绝协议饱和，避免改变共享核已验证的力矩语义。"""
    session, transport = _open_session()

    with pytest.raises(ValueError, match="协议范围"):
        session.command(command)

    assert transport.written_payloads == []


def test_command_accepts_safe_feedback_margin_and_requires_enabled_reply() -> None:
    """目标仍在工作范围时，返回反馈可位于扩展安全范围但必须保持使能。"""
    session, transport = _open_session()
    command = MITCommand(0.0, 0.0, 2.0, 0.5, 0.0)
    transport.inject_received(_feedback(session, -0.02, STATUS_ENABLED))

    feedback = session.command(command)

    assert feedback.position_rad == pytest.approx(-0.02, abs=3.4 / 65535)
    assert session.last_feedback is feedback
    assert transport.written_payloads == [session.adapter.prepare(command).frame]


@pytest.mark.parametrize(
    ("position_rad", "status_code", "message"),
    [
        (0.2, STATUS_DISABLED, "运行中失能"),
        (-0.06, STATUS_ENABLED, "反馈角"),
    ],
)
def test_command_rejects_disabled_or_unsafe_feedback(
    position_rad: float, status_code: int, message: str
) -> None:
    """命令回包必须同时满足使能状态与扩展反馈位置边界。"""
    session, transport = _open_session()
    command = MITCommand(0.2, 0.0, 2.0, 0.5, 0.0)
    transport.inject_received(_feedback(session, position_rad, status_code))

    with pytest.raises((RuntimeError, ValueError), match=message):
        session.command(command)

    assert transport.written_payloads == [session.adapter.prepare(command).frame]


def test_hold_requires_zero_velocity_and_zero_feedforward_without_writing() -> None:
    """非零速度或前馈力矩请求不能伪装成安全保持命令。"""
    session, transport = _open_session()
    transport.inject_received(_feedback(session, 0.2, STATUS_ENABLED))
    session.enable()
    written_before = list(transport.written_payloads)

    with pytest.raises(ValueError, match="目标速度"):
        session.hold(MITCommand(0.2, 0.01, 2.0, 0.5, 0.0))
    with pytest.raises(ValueError, match="前馈力矩"):
        session.hold(MITCommand(0.2, 0.0, 2.0, 0.5, 0.01))

    assert transport.written_payloads == written_before


def test_hold_rejects_target_that_is_not_latest_feedback_projection() -> None:
    """任意零速位置命令不能借用 hold 接口伪装成当前位置保持。"""
    session, transport = _open_session()
    transport.inject_received(_feedback(session, 0.2, STATUS_ENABLED))
    session.enable()
    written_before = list(transport.written_payloads)

    with pytest.raises(ValueError, match="最新反馈位置"):
        session.hold(MITCommand(0.4, 0.0, 2.0, 0.5, 0.0))

    assert transport.written_payloads == written_before


def test_hold_sends_projected_work_range_target_and_updates_feedback() -> None:
    """轻微负反馈可由调用方投影到零目标后进入一发一收保持。"""
    session, transport = _open_session()
    hold = MITCommand(0.0, 0.0, 2.0, 0.5, 0.0)
    transport.inject_received(
        _feedback(session, -0.02, STATUS_ENABLED) + _feedback(session, -0.01, STATUS_ENABLED)
    )
    session.enable()

    feedback = session.hold(hold)

    assert feedback.position_rad == pytest.approx(-0.01, abs=3.4 / 65535)
    assert session.last_feedback is feedback
    assert transport.written_payloads == [
        session.protocol.make_control_packet(1, CMD_ENABLE),
        session.adapter.prepare(hold).frame,
    ]


def test_hold_rejects_without_latest_enabled_feedback() -> None:
    """保持不能在没有有效使能反馈时发送。"""
    session, transport = _open_session()
    hold = MITCommand(0.0, 0.0, 2.0, 0.5, 0.0)

    with pytest.raises(RuntimeError, match="尚无有效"):
        session.hold(hold)

    transport.inject_received(_feedback(session, 0.0, STATUS_DISABLED))
    session.require_disabled()
    with pytest.raises(RuntimeError, match="未处于使能"):
        session.hold(hold)


def test_disable_only_returns_after_disabled_feedback_is_validated() -> None:
    """失能确认仍需完整写入、主动刷新和真实 status=0 反馈。"""
    session, transport = _open_session()
    transport.inject_received(_feedback(session, 0.1, STATUS_DISABLED))

    feedback = session.disable()

    assert feedback.status_code == STATUS_DISABLED
    assert session.last_feedback is feedback
    assert transport.written_payloads == [
        session.protocol.make_control_packet(1, CMD_DISABLE),
        session.protocol.make_feedback_request(1),
    ]


def test_disable_does_not_confirm_when_fresh_feedback_is_not_disabled() -> None:
    """发送失能帧不等于失能成功，非零新状态必须拒绝确认。"""
    session, transport = _open_session()
    transport.inject_received(_feedback(session, 0.1, STATUS_ENABLED))

    with pytest.raises(RuntimeError, match="最终失能确认失败"):
        session.disable()

    assert transport.written_payloads == [
        session.protocol.make_control_packet(1, CMD_DISABLE),
        session.protocol.make_feedback_request(1),
    ]


class _ShortWriteTransport(FakeTransport):
    """记录帧后模拟 USB2CAN 只接受部分字节。"""

    def write(self, payload: bytes, timeout_s: float | None = None) -> int:
        """对每一帧报告少写一个字节。"""
        written = super().write(payload, timeout_s=timeout_s)
        return written - 1


def test_session_control_write_rejects_short_frame() -> None:
    """会话控制路径继续把短写视为通信失败。"""
    transport = _ShortWriteTransport()
    session = DmSession("fake://usb2can", 0.1, transport=transport)
    session.open()

    with pytest.raises(OSError, match="短写"):
        session.enable()

    assert len(transport.written_payloads) == 1
    assert transport.written_payloads[0] == session.protocol.make_control_packet(1, CMD_ENABLE)


def test_margin_keyword_is_forwarded_to_session_deployment() -> None:
    """会话构造关键字应成为部署反馈范围的唯一配置来源。"""
    session = DmSession(
        "fake://usb2can",
        0.1,
        transport=FakeTransport(),
        feedback_position_margin_rad=0.03,
    )

    assert session.deployment.feedback_position_min_rad == -0.03
    assert session.deployment.feedback_position_max_rad == pytest.approx(math.pi / 2 + 0.03)

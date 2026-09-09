"""验证 DM4310P 受限小步运动探针命令行。"""

from __future__ import annotations

import io
import json

from dmgripper_hardware import (
    FakeTransport,
    MotorFeedback,
    Usb2CanProtocol,
    make_dm4310p_gripper_config,
)
from dmgripper_hardware.motion_cli import run


def _register_reply(value: int) -> bytes:
    """编码电机 1、主机 17 对控制模式寄存器的只读回复。"""
    return bytes((0xAA, 0x11, 0x08, 17, 0, 0, 0, 1, 0, 0x33, 10, value, 0, 0, 0, 0x55))


class ScriptedTransport(FakeTransport):
    """按写入顺序注入 CLI 执行所需的设备回复。"""

    def __init__(self, responses: list[bytes]) -> None:
        """创建有限响应脚本。"""
        super().__init__()
        self._responses = list(responses)

    def write(self, payload: bytes, timeout_s: float | None = None) -> int:
        """记录写入并注入当前帧的预置响应。"""
        written = super().write(payload, timeout_s=timeout_s)
        if self._responses:
            self.inject_received(self._responses.pop(0))
        return written


def test_cli_defaults_to_dry_run_without_actuator_commands() -> None:
    """未给出 `--execute` 时，命令行只读初始反馈并输出计划。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = FakeTransport()
    transport.inject_received(protocol.encode_feedback_frame(1, MotorFeedback(1.0, 0.0, 0.0, 0)))
    transport.inject_received(_register_reply(1))
    transport.inject_received(protocol.encode_feedback_frame(1, MotorFeedback(1.0, 0.0, 0.0, 0)))
    output = io.StringIO()

    result = run(
        ["--port", "fake://usb2can"],
        transport_factory=lambda: transport,
        stdout=output,
        stderr=io.StringIO(),
    )

    record = json.loads(output.getvalue())
    assert result == 0
    assert record["mode"] == "dry-run"
    assert record["initial_status_code"] == 0
    assert record["control_mode"] == 1
    assert record["mit_kp"] == 2.0
    assert record["mit_kd"] == 0.5
    assert record["closing_step_rad"] == 0.03
    assert record["stage_duration_s"] == 1.0
    assert record["actuator_command_writes"] == 0
    assert transport.written_payloads == [
        protocol.make_feedback_request(1),
        protocol.make_read_register_packet(1, 10),
        protocol.make_feedback_request(1),
    ]
    assert transport.is_open is False


def test_cli_rejects_kp_above_protocol_limit() -> None:
    """命令行必须拒绝协议无法表示的 MIT kp。"""
    error = io.StringIO()

    result = run(
        ["--port", "fake://usb2can", "--mit-kp", "500.1"],
        transport_factory=FakeTransport,
        stdout=io.StringIO(),
        stderr=error,
    )

    assert result == 1
    assert "MIT kp" in error.getvalue()


def test_cli_dry_run_echoes_selected_closing_step() -> None:
    """dry-run 必须回显实际采用的闭合步长。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = FakeTransport()
    transport.inject_received(protocol.encode_feedback_frame(1, MotorFeedback(1.0, 0.0, 0.0, 0)))
    transport.inject_received(_register_reply(1))
    transport.inject_received(protocol.encode_feedback_frame(1, MotorFeedback(1.0, 0.0, 0.0, 0)))
    output = io.StringIO()

    result = run(
        ["--port", "fake://usb2can", "--mit-kp", "50", "--closing-step", "0.06"],
        transport_factory=lambda: transport,
        stdout=output,
        stderr=io.StringIO(),
    )

    record = json.loads(output.getvalue())
    assert result == 0
    assert record["mit_kp"] == 50.0
    assert record["closing_step_rad"] == 0.06


def test_cli_emits_complete_event_only_after_final_disable_confirmation() -> None:
    """成功执行应在三阶段记录后输出失能确认终止事件。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)

    def feedback(position: float, status: int) -> bytes:
        """创建脚本使用的目标电机反馈。"""
        return protocol.encode_feedback_frame(1, MotorFeedback(position, 0.0, 0.0, status))

    transport = ScriptedTransport(
        [
            feedback(1.0, 0),
            _register_reply(1),
            feedback(1.0, 0),
            feedback(1.0, 1),
            feedback(1.0, 1),
            feedback(1.03, 1),
            feedback(1.0, 1),
            b"",
            feedback(1.0, 0),
        ]
    )
    output = io.StringIO()

    result = run(
        ["--port", "fake://usb2can", "--execute", "--stage-duration", "0.001"],
        transport_factory=lambda: transport,
        stdout=output,
        stderr=io.StringIO(),
    )

    rows = [json.loads(line) for line in output.getvalue().splitlines()]
    assert result == 0
    assert [row["stage"] for row in rows[:3]] == ["hold", "close", "return"]
    assert rows[-1] == {"mode": "execute", "event": "complete", "disable_confirmed": True}
    assert "final_position_rad" not in rows[-1]

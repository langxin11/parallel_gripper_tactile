"""验证 DM4310P 只读状态探针命令行。"""

from __future__ import annotations

import io
import json

import pytest

from dmgripper_hardware import (
    FakeTransport,
    MotorFeedback,
    Usb2CanProtocol,
    make_dm4310p_gripper_config,
)
from dmgripper_hardware.cli import run_probe


def test_probe_writes_only_feedback_requests_and_emits_json_lines() -> None:
    """探针只发送状态查询帧，并输出稳定的 JSON Lines。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = FakeTransport()
    for _ in range(2):
        transport.inject_received(
            protocol.encode_feedback_frame(1, MotorFeedback(0.2, 0.3, 0.4, 1))
        )
    output = io.StringIO()
    error = io.StringIO()

    result = run_probe(
        ["--port", "fake://usb2can", "--count", "2", "--interval", "0"],
        transport_factory=lambda: transport,
        output=output,
        error=error,
        sleep=lambda _: pytest.fail("零间隔不应等待"),
    )

    assert result == 0
    assert error.getvalue() == ""
    rows = [json.loads(line) for line in output.getvalue().splitlines()]
    assert [row["sequence"] for row in rows] == [1, 2]
    assert all(row["within_mechanical_range"] is True for row in rows)
    assert all(row["fault"] is False for row in rows)
    assert all(isinstance(row["host_monotonic_receipt_ns"], int) for row in rows)
    assert transport.written_payloads == [protocol.make_feedback_request(1)] * 2
    assert transport.is_open is False


def test_probe_marks_feedback_outside_mechanical_range_without_acting() -> None:
    """机械行程外反馈只标记，不截断、不发送运动或修正命令。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = FakeTransport()
    transport.inject_received(protocol.encode_feedback_frame(1, MotorFeedback(-1.0, 0.0, 0.0, 3)))
    output = io.StringIO()

    result = run_probe(
        ["--port", "fake://usb2can", "--count", "1"],
        transport_factory=lambda: transport,
        output=output,
        error=io.StringIO(),
    )

    row = json.loads(output.getvalue())
    assert result == 0
    assert row["within_mechanical_range"] is False
    assert row["fault"] is True
    assert transport.written_payloads == [protocol.make_feedback_request(1)]
    assert transport.is_open is False


def test_probe_reports_errors_and_closes_transport() -> None:
    """反馈超时时以中文诊断失败，并确保传输关闭。"""
    transport = FakeTransport()
    error = io.StringIO()

    result = run_probe(
        ["--port", "fake://usb2can", "--count", "1", "--timeout", "0.01"],
        transport_factory=lambda: transport,
        output=io.StringIO(),
        error=error,
    )

    assert result != 0
    assert "状态探针失败" in error.getvalue()
    assert transport.written_payloads
    assert transport.is_open is False

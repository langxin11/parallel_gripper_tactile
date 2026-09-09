"""验证 DM4310P 受限小步运动探针的离线安全边界。"""

from __future__ import annotations

from collections.abc import Callable

import pytest

from dmgripper_hardware import (
    CMD_DISABLE,
    CMD_ENABLE,
    DmSafeMotionProbe,
    FakeTransport,
    MotionDisableError,
    MotionProbePlan,
    MotionProbeSafetyConfig,
    MotionSafetyError,
    MotorFeedback,
    STATUS_ENABLED,
    Usb2CanProtocol,
    make_dm4310p_gripper_config,
)


def _register_reply(value: int) -> bytes:
    """编码电机 1、主机 17 对控制模式寄存器的只读回复。"""
    return bytes((0xAA, 0x11, 0x08, 17, 0, 0, 0, 1, 0, 0x33, 10, value, 0, 0, 0, 0x55))


class ScriptedTransport(FakeTransport):
    """按每次写入顺序注入对应响应，模拟命令产生的一次反馈。"""

    def __init__(self, responses: list[bytes | Callable[[bytes], bytes]]) -> None:
        """创建带有限响应脚本的内存传输。"""
        super().__init__()
        self._responses = list(responses)

    def write(self, payload: bytes, timeout_s: float | None = None) -> int:
        """记录写入并注入该写入对应的串口响应。"""
        written = super().write(payload, timeout_s=timeout_s)
        if self._responses:
            response = self._responses.pop(0)
            self.inject_received(response(payload) if callable(response) else response)
        return written


class DisableShortWriteTransport(ScriptedTransport):
    """只对最终失能帧模拟短写。"""

    def write(self, payload: bytes, timeout_s: float | None = None) -> int:
        """记录失能帧后报告短写，保留其已尝试写入的证据。"""
        written = super().write(payload, timeout_s=timeout_s)
        if payload[21:29] == bytes((0xFF,) * 7 + (CMD_DISABLE,)):
            return written - 1
        return written


def _make_probe(transport: FakeTransport) -> tuple[DmSafeMotionProbe, Usb2CanProtocol]:
    """创建尚未打开的离线运动探针。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    return (
        DmSafeMotionProbe(
            deployment,
            protocol,
            transport,
            safety=MotionProbeSafetyConfig(stage_duration_s=0.001, control_interval_s=0.002),
        ),
        protocol,
    )


def _feedback(protocol: Usb2CanProtocol, position: float, status: int = STATUS_ENABLED) -> bytes:
    """为脚本生成一条静止目标电机反馈。"""
    return protocol.encode_feedback_frame(1, MotorFeedback(position, 0.0, 0.0, status))


def _normal_responses(protocol: Usb2CanProtocol, start: float = 1.0) -> list[bytes]:
    """生成初检、模式、使能、三阶段和失能确认的完整脚本。"""
    return [
        _feedback(protocol, start, status=0),
        _register_reply(1),
        _feedback(protocol, start, status=0),
        _feedback(protocol, start),
        _feedback(protocol, start),
        _feedback(protocol, start + 0.03),
        _feedback(protocol, start),
        b"",
        _feedback(protocol, start, status=0),
    ]


def test_inspection_only_writes_two_read_requests_and_returns_fixed_small_step() -> None:
    """dry-run 仅查询状态与模式，绝不发送使能、MIT 或失能。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = ScriptedTransport(
        [_feedback(protocol, 1.0, status=0), _register_reply(1), _feedback(protocol, 1.0, status=0)]
    )
    probe, _ = _make_probe(transport)
    probe.open()

    plan = probe.inspect_and_plan()

    assert plan.initial_position_rad == pytest.approx(1.0, abs=3.4 / 65535)
    assert plan.close_target_rad == pytest.approx(plan.initial_position_rad + 0.03)
    assert plan.control_mode == 1
    assert transport.written_payloads == [
        protocol.make_feedback_request(1),
        protocol.make_read_register_packet(1, 10),
        protocol.make_feedback_request(1),
    ]


def test_execute_consumes_one_reply_per_command_without_extra_feedback_queries() -> None:
    """使能和每条 MIT 命令均只消费自身反馈，不得插入状态查询造成积压。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = ScriptedTransport(_normal_responses(protocol))
    probe, _ = _make_probe(transport)
    probe.open()
    plan = probe.inspect_and_plan()

    results = probe.execute(plan)

    assert [result.stage for result in results] == ["hold", "close", "return"]
    assert probe.last_disable_confirmed is True
    assert len(transport.written_payloads) == 9
    assert transport.written_payloads[3] == protocol.make_control_packet(1, CMD_ENABLE)
    assert transport.written_payloads[-2] == protocol.make_control_packet(1, CMD_DISABLE)
    assert transport.written_payloads[-1] == protocol.make_feedback_request(1)
    assert transport.written_payloads.count(protocol.make_feedback_request(1)) == 3
    assert results[1].target_reached is True
    assert results[1].position_error_rad == pytest.approx(0.0, abs=3.4 / 65535)


def test_disabled_enable_reply_rejects_before_hold_and_close_then_disables() -> None:
    """使能后 status=0 不可继续发送 MIT；仍必须尽力失能。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = ScriptedTransport(
        [
            _feedback(protocol, 1.0, status=0),
            _register_reply(1),
            _feedback(protocol, 1.0, status=0),
            _feedback(protocol, 1.0, status=0),
            b"",
            _feedback(protocol, 1.0, status=0),
        ]
    )
    probe, _ = _make_probe(transport)
    probe.open()
    plan = probe.inspect_and_plan()

    with pytest.raises(MotionSafetyError, match="使能确认失败"):
        probe.execute(plan)

    assert transport.written_payloads == [
        protocol.make_feedback_request(1),
        protocol.make_read_register_packet(1, 10),
        protocol.make_feedback_request(1),
        protocol.make_control_packet(1, CMD_ENABLE),
        protocol.make_control_packet(1, CMD_DISABLE),
        protocol.make_feedback_request(1),
    ]


def test_execute_rejects_non_mit_mode_before_enable() -> None:
    """模式寄存器非 MIT 时，只读计划后拒绝执行，不隐式切换模式。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = ScriptedTransport(
        [_feedback(protocol, 1.0, status=0), _register_reply(2), _feedback(protocol, 1.0, status=0)]
    )
    probe, _ = _make_probe(transport)
    probe.open()
    plan = probe.inspect_and_plan()

    with pytest.raises(MotionSafetyError, match="不是 MIT"):
        probe.execute(plan)

    assert len(transport.written_payloads) == 3


def test_normal_disable_short_write_is_not_reported_as_success() -> None:
    """正常三阶段完成后，失能短写必须使执行失败并保留帧记录。"""
    deployment = make_dm4310p_gripper_config("fake://usb2can")
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = DisableShortWriteTransport(_normal_responses(protocol))
    probe, _ = _make_probe(transport)
    probe.open()
    plan = probe.inspect_and_plan()

    with pytest.raises(MotionDisableError, match="最终失能失败"):
        probe.execute(plan)

    assert transport.written_payloads[-1] == protocol.make_control_packet(1, CMD_DISABLE)


def test_execute_rejects_plan_not_created_by_fresh_inspection() -> None:
    """调用方不能手工构造合法目标来跳过动作前的只读安全检查。"""
    transport = FakeTransport()
    probe, _ = _make_probe(transport)
    probe.open()
    plan = MotionProbePlan(MotorFeedback(1.0, 0.0, 0.0, 0), 1, 0.0, 1.0, 1.0, 1.03, 1.0)

    with pytest.raises(MotionSafetyError, match="初始状态检查"):
        probe.execute(plan)

    assert transport.written_payloads == []


def test_stage_duration_completes_when_close_target_is_not_reached() -> None:
    """MIT 平衡点未到达仅作诊断，固定时长内反馈安全即为阶段成功。"""

    class Clock:
        """由测试 sleep 推进的最小单调时钟。"""

        value = 0.0

        def __call__(self) -> float:
            """返回当前测试时间。"""
            return self.value

        def sleep(self, duration: float) -> None:
            """推进测试时间。"""
            self.value += duration

    deployment = make_dm4310p_gripper_config("fake://usb2can", timeout_s=0.1)
    protocol = Usb2CanProtocol(deployment.motor_limits)
    transport = ScriptedTransport(
        [
            _feedback(protocol, 1.0, status=0),
            _register_reply(1),
            _feedback(protocol, 1.0, status=0),
            _feedback(protocol, 1.0),
            protocol.encode_feedback_frame(1, MotorFeedback(1.0, 0.1, 1.0, 1)),
            _feedback(protocol, 1.0),
            _feedback(protocol, 1.0),
            b"",
            _feedback(protocol, 1.0, status=0),
        ]
    )
    clock = Clock()
    probe = DmSafeMotionProbe(
        deployment,
        protocol,
        transport,
        safety=MotionProbeSafetyConfig(stage_duration_s=0.01, control_interval_s=0.02),
        clock=clock,
        sleep=clock.sleep,
    )
    probe.open()
    plan = probe.inspect_and_plan()

    results = probe.execute(plan)

    assert [result.command_count for result in results] == [1, 1, 1]
    assert results[1].target_reached is False
    assert results[1].position_error_rad == pytest.approx(0.03, abs=3.4 / 65535)
    assert results[0].max_abs_velocity_rad_s == pytest.approx(0.1, abs=16.0 / 4095)
    assert results[0].max_abs_torque_nm == pytest.approx(1.0, abs=8.0 / 4095)

    mit_packets = [
        packet
        for packet in transport.written_payloads
        if packet[13:15] == bytes((1, 0)) and packet[21:28] != bytes((0xFF,) * 7)
    ]
    assert len(mit_packets) == 3
    assert transport.written_payloads[-2] == protocol.make_control_packet(1, CMD_DISABLE)
    assert transport.written_payloads[-1] == protocol.make_feedback_request(1)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"closing_step_rad": 0.0},
        {"mit_kp": 500.1},
        {"mit_kd": 5.1},
    ],
)
def test_safety_config_rejects_invalid_protocol_or_step_settings(kwargs: dict[str, float]) -> None:
    """库 API 仅拒绝非正步长和协议不能表示的 MIT 增益。"""
    with pytest.raises(ValueError):
        MotionProbeSafetyConfig(**kwargs)


def test_safety_config_allows_protocol_kp_and_arbitrary_positive_step() -> None:
    """闭合目标是否越界由部署机械行程校验，而非配置层作隐式截断。"""
    config = MotionProbeSafetyConfig(closing_step_rad=0.4, mit_kp=500.0)

    assert config.closing_step_rad == 0.4
    assert config.mit_kp == 500.0

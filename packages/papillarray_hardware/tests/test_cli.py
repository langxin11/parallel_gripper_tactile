"""PapillArray 只读采集探针的离线测试。"""

from __future__ import annotations

import io
import json

import numpy as np
import pytest

from papillarray_hardware import PtsPacket
from papillarray_hardware.cli import _counter_status, build_parser, run
from papillarray_hardware.protocol import ProtocolError


class FakeClient:
    """记录探针生命周期与设备命令的离线客户端。"""

    def __init__(
        self, packets: list[PtsPacket | BaseException], *, close_error: Exception | None = None
    ) -> None:
        """保存要依次返回或抛出的结果。"""
        self._packets = list(packets)
        self._close_error = close_error
        self.opened = False
        self.closed = False
        self.configure_count = 0
        self.clear_bias_count = 0
        self.start_slip_count = 0
        self.stop_slip_count = 0

    def open(self) -> None:
        """标记已显式打开。"""
        self.opened = True

    def configure_stream(self) -> None:
        """记录唯一允许的采样率配置命令。"""
        self.configure_count += 1

    def clear_bias(self) -> None:
        """记录若有错误发出的清零调用。"""
        self.clear_bias_count += 1

    def start_slip_detection(self) -> None:
        """记录若有错误发出的滑动检测启动调用。"""
        self.start_slip_count += 1

    def stop_slip_detection(self) -> None:
        """记录若有错误发出的滑动检测停止调用。"""
        self.stop_slip_count += 1

    def read_packet(self) -> PtsPacket:
        """按预设返回数据包或抛出采集异常。"""
        item = self._packets.pop(0)
        if isinstance(item, BaseException):
            raise item
        return item

    def close(self) -> None:
        """标记资源已被释放。"""
        self.closed = True
        if self._close_error is not None:
            raise self._close_error


def make_packet(counter: int, sensor_count: int = 2) -> PtsPacket:
    """构造带可验证全局力、力矩与 pillar 数的离线数据包。"""
    pillar_forces = [np.zeros((sensor_index + 1, 3)) for sensor_index in range(sensor_count)]
    return PtsPacket(
        packet_counter=counter,
        timestamp_us=1000 + counter,
        pillar_forces=pillar_forces,
        pillar_displacements=[values.copy() for values in pillar_forces],
        global_forces=[np.array((sensor_index, 2.0, 3.0)) for sensor_index in range(sensor_count)],
        global_torques=[np.array((0.1, 0.2, sensor_index)) for sensor_index in range(sensor_count)],
    )


def test_probe_parser_requires_port_and_has_safe_finite_defaults() -> None:
    """端口必须显式提供，其他采集参数使用有限且可审计的默认值。"""
    parser = build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args([])
    args = parser.parse_args(["--port", "/dev/fake"])

    assert args.baud == 115200
    assert args.rate == 500
    assert args.expected_sensors == 2
    assert args.count == 10
    assert args.timeout == 1.0


@pytest.mark.parametrize("port", ("", " /dev/fake", "/dev/fake ", "/dev/\x00fake"))
def test_probe_parser_rejects_ambiguous_port(port: str) -> None:
    """空白或 NUL 串口端点在 argparse 阶段被拒绝，不会进入串口配置。"""
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--port", port])


def test_probe_outputs_stable_json_lines_and_only_configures_rate() -> None:
    """每包输出 JSON 行，且默认路径不发送清零或滑动检测命令。"""
    client = FakeClient([make_packet(7), make_packet(8)])
    stdout = io.StringIO()
    stderr = io.StringIO()

    result = run(
        ["--port", "/dev/fake", "--count", "2"],
        client_factory=lambda _config: client,
        monotonic_ns=iter((101, 202)).__next__,
        stdout=stdout,
        stderr=stderr,
    )

    lines = [json.loads(line) for line in stdout.getvalue().splitlines()]
    assert result == 0
    assert stderr.getvalue() == ""
    assert client.opened and client.closed
    assert client.configure_count == 1
    assert client.clear_bias_count == 0
    assert client.start_slip_count == 0
    assert client.stop_slip_count == 0
    assert lines == [
        {
            "host_monotonic_receipt_ns": 101,
            "packet_counter": 7,
            "timestamp_us": 1007,
            "sensors": [
                {
                    "global_force": [0.0, 2.0, 3.0],
                    "global_torque": [0.1, 0.2, 0.0],
                    "pillar_count": 1,
                },
                {
                    "global_force": [1.0, 2.0, 3.0],
                    "global_torque": [0.1, 0.2, 1.0],
                    "pillar_count": 2,
                },
            ],
            "counter_event": "first",
            "counter_gap": None,
        },
        {
            "host_monotonic_receipt_ns": 202,
            "packet_counter": 8,
            "timestamp_us": 1008,
            "sensors": [
                {
                    "global_force": [0.0, 2.0, 3.0],
                    "global_torque": [0.1, 0.2, 0.0],
                    "pillar_count": 1,
                },
                {
                    "global_force": [1.0, 2.0, 3.0],
                    "global_torque": [0.1, 0.2, 1.0],
                    "pillar_count": 2,
                },
            ],
            "counter_event": "consecutive",
            "counter_gap": 0,
        },
    ]


@pytest.mark.parametrize(
    ("failure", "expected_text"),
    [
        (TimeoutError("无数据"), "超时"),
        (ProtocolError("帧损坏"), "协议错误"),
    ],
)
def test_probe_closes_client_and_reports_actionable_read_errors(
    failure: Exception, expected_text: str
) -> None:
    """超时或协议错误均以非零退出，并在错误路径关闭串口。"""
    client = FakeClient([failure])
    stderr = io.StringIO()

    result = run(
        ["--port", "/dev/fake", "--count", "1"],
        client_factory=lambda _config: client,
        stdout=io.StringIO(),
        stderr=stderr,
    )

    assert result == 1
    assert client.closed
    assert expected_text in stderr.getvalue()


def test_probe_rejects_sensor_count_mismatch_and_closes_client() -> None:
    """收到与参数不符的传感器数时不得继续输出观测。"""
    client = FakeClient([make_packet(1, sensor_count=1)])
    stderr = io.StringIO()

    result = run(
        ["--port", "/dev/fake", "--expected-sensors", "2"],
        client_factory=lambda _config: client,
        stdout=io.StringIO(),
        stderr=stderr,
    )

    assert result == 1
    assert client.closed
    assert "期望 2，实际 1" in stderr.getvalue()


@pytest.mark.parametrize(
    ("previous", "current", "event", "gap"),
    [
        (10, 11, "consecutive", 0),
        (10, 13, "gap", 2),
        (0xFFFFFFFF, 0, "wrap", 0),
        (0xFFFFFFFE, 1, "wrap", 2),
        (10, 10, "duplicate", None),
        (10, 9, "out_of_order", None),
    ],
)
def test_counter_status_classifies_all_forward_and_nonforward_cases(
    previous: int, current: int, event: str, gap: int | None
) -> None:
    """重复或乱序不得伪装成巨大丢包，跨回绕仍保留可解释的 gap。"""
    assert _counter_status(previous, current) == (event, gap)


def test_counter_status_marks_first_packet() -> None:
    """没有前序观测的数据包必须显式标记为首次接收。"""
    assert _counter_status(None, 10) == ("first", None)


def test_probe_does_not_advance_counter_baseline_for_out_of_order_packet() -> None:
    """乱序包不污染前进基线，后续连续包仍必须被正确识别。"""
    client = FakeClient([make_packet(10), make_packet(9), make_packet(11)])
    stdout = io.StringIO()

    result = run(
        ["--port", "/dev/fake", "--count", "3"],
        client_factory=lambda _config: client,
        monotonic_ns=iter((101, 202, 303)).__next__,
        stdout=stdout,
        stderr=io.StringIO(),
    )

    records = [json.loads(line) for line in stdout.getvalue().splitlines()]
    assert result == 0
    assert [(record["counter_event"], record["counter_gap"]) for record in records] == [
        ("first", None),
        ("out_of_order", None),
        ("consecutive", 0),
    ]


def test_probe_closes_client_after_ctrl_c() -> None:
    """Ctrl-C 不会遗留打开的串口，并返回 shell 可识别的中断状态。"""
    client = FakeClient([KeyboardInterrupt()])
    stderr = io.StringIO()

    result = run(
        ["--port", "/dev/fake", "--count", "1"],
        client_factory=lambda _config: client,
        stdout=io.StringIO(),
        stderr=stderr,
    )

    assert result == 130
    assert client.closed
    assert "Ctrl-C" in stderr.getvalue()


def test_probe_reports_close_failure_after_successful_read() -> None:
    """关闭失败使原本成功的探针改为非零退出，并留下可行动诊断。"""
    client = FakeClient([make_packet(1)], close_error=OSError("设备忙"))
    stderr = io.StringIO()

    result = run(
        ["--port", "/dev/fake", "--count", "1"],
        client_factory=lambda _config: client,
        stdout=io.StringIO(),
        stderr=stderr,
    )

    assert result == 1
    assert client.closed
    assert "关闭串口失败" in stderr.getvalue()


@pytest.mark.parametrize(
    ("failure", "expected_status", "primary_text"),
    [(TimeoutError("无数据"), 1, "超时"), (KeyboardInterrupt(), 130, "Ctrl-C")],
)
def test_probe_preserves_primary_failure_when_close_also_fails(
    failure: BaseException, expected_status: int, primary_text: str
) -> None:
    """关闭异常不能覆盖读取错误或 Ctrl-C 的状态与诊断。"""
    client = FakeClient([failure], close_error=OSError("设备忙"))
    stderr = io.StringIO()

    result = run(
        ["--port", "/dev/fake", "--count", "1"],
        client_factory=lambda _config: client,
        stdout=io.StringIO(),
        stderr=stderr,
    )

    assert result == expected_status
    assert client.closed
    assert primary_text in stderr.getvalue()
    assert "关闭串口失败" in stderr.getvalue()

"""PapillArray 只读采集探针的命令行入口。"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections.abc import Callable, Sequence
from typing import NoReturn, TextIO

from .client import (
    DEFAULT_PAPILLARRAY_PORT,
    PapillArraySerialClient,
    PapillArraySerialConfig,
    SUPPORTED_SAMPLING_RATES,
)
from .protocol import ProtocolError, PtsPacket, PtsReadDiagnostics, PtsReadTimeout

_COUNTER_MODULUS = 2**32

ClientFactory = Callable[[PapillArraySerialConfig], PapillArraySerialClient]


def _positive_int(value: str) -> int:
    """解析正整数命令行参数。"""
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是正整数") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须是正整数")
    return parsed


def _positive_float(value: str) -> float:
    """解析有限正浮点数命令行参数。"""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是有限正数") from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("必须是有限正数")
    return parsed


def _port(value: str) -> str:
    """校验串口端点，避免将歧义字符串交给底层串口库。"""
    if not value or value != value.strip() or "\x00" in value:
        raise argparse.ArgumentTypeError("必须是非空、无首尾空白且不含 NUL 的串口端点")
    return value


def build_parser() -> argparse.ArgumentParser:
    """创建只读探针的参数解析器。"""
    parser = argparse.ArgumentParser(
        description="从 PapillArray 输出有限个只读触觉包，每行输出一个 JSON 对象。"
    )
    parser.add_argument(
        "--port",
        type=_port,
        default=DEFAULT_PAPILLARRAY_PORT,
        help=f"串口端点，默认 {DEFAULT_PAPILLARRAY_PORT}",
    )
    parser.add_argument("--baud", type=_positive_int, default=115200, help="波特率，默认 115200")
    parser.add_argument(
        "--rate",
        type=_positive_int,
        choices=sorted(SUPPORTED_SAMPLING_RATES),
        default=500,
        help="采样率（Hz），默认 500",
    )
    parser.add_argument(
        "--expected-sensors", type=_positive_int, default=2, help="期望传感器数，默认 2"
    )
    parser.add_argument("--count", type=_positive_int, default=10, help="采集包数，默认 10")
    parser.add_argument(
        "--timeout",
        type=_positive_float,
        default=1.0,
        help="单次底层串口读取超时秒数，默认 1",
    )
    parser.add_argument(
        "--packet-timeout",
        type=_positive_float,
        default=3.0,
        help="单包等待有效 PTS 帧的总时限秒数，默认 3",
    )
    return parser


def _counter_status(previous_counter: int | None, packet_counter: int) -> tuple[str, int | None]:
    """按无符号 32 位半模规则分类包计数器变化。"""
    if previous_counter is None:
        return "first", None
    delta = (packet_counter - previous_counter) % _COUNTER_MODULUS
    if delta == 0:
        return "duplicate", None
    if delta >= _COUNTER_MODULUS // 2:
        return "out_of_order", None
    gap = delta - 1
    if packet_counter < previous_counter:
        return "wrap", gap
    if gap:
        return "gap", gap
    return "consecutive", 0


def _packet_record(
    packet: PtsPacket,
    receipt_ns: int,
    counter_event: str,
    counter_gap: int | None,
) -> dict[str, object]:
    """将一个协议包转成没有 NumPy 标量的 JSON 兼容记录。"""
    if (
        len(packet.global_forces) != packet.n_sensors
        or len(packet.global_torques) != packet.n_sensors
    ):
        raise ProtocolError("PTS 包的全局力／力矩传感器数不一致")
    sensors = []
    for sensor_index in range(packet.n_sensors):
        sensors.append(
            {
                "global_force": [
                    float(value) for value in packet.global_forces[sensor_index].tolist()
                ],
                "global_torque": [
                    float(value) for value in packet.global_torques[sensor_index].tolist()
                ],
                "pillar_count": int(len(packet.pillar_forces[sensor_index])),
            }
        )
    return {
        "host_monotonic_receipt_ns": receipt_ns,
        "packet_counter": int(packet.packet_counter),
        "timestamp_us": int(packet.timestamp_us),
        "sensors": sensors,
        "counter_event": counter_event,
        "counter_gap": counter_gap,
    }


def _default_client_factory(config: PapillArraySerialConfig) -> PapillArraySerialClient:
    """按默认生产实现构造串口客户端。"""
    return PapillArraySerialClient(config)


def _format_diagnostics(diagnostics: PtsReadDiagnostics) -> str:
    """将读取器诊断压缩为适合标准错误的一行中文摘要。"""
    last_error = diagnostics.last_protocol_error or "无"
    return (
        "协议诊断："
        f"接收字节={diagnostics.received_bytes}，"
        f"空读取={diagnostics.empty_reads}，"
        f"起始标志={diagnostics.start_markers}，"
        f"结束标志={diagnostics.end_markers}，"
        f"候选帧={diagnostics.candidate_frames}，"
        f"校验失败={diagnostics.checksum_failures}，"
        f"结构失败={diagnostics.protocol_failures}，"
        f"超长丢弃={diagnostics.oversize_discards}，"
        f"最后协议错误={last_error}，"
        f"原始十六进制预览={diagnostics.raw_hex_preview or '无'}。"
    )


def run(
    argv: Sequence[str] | None = None,
    *,
    client_factory: ClientFactory = _default_client_factory,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
) -> int:
    """运行有限次只读采集，并返回进程退出码。

    只在显式打开端口后发送一次采样率配置命令；不会发送清零或滑动检测命令。

    Args:
        argv: 命令行参数；省略时读取当前进程参数。
        client_factory: 可注入客户端工厂，供离线测试使用。
        monotonic_ns: 可注入的主机单调时钟。
        stdout: JSON Lines 输出流。
        stderr: 诊断输出流。

    Returns:
        成功为 `0`，采集错误为非零值。
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.packet_timeout < args.timeout:
        parser.error("--packet-timeout 必须大于或等于 --timeout")
    config = PapillArraySerialConfig(
        port=args.port,
        baud_rate=args.baud,
        sampling_rate=args.rate,
        expected_sensors=args.expected_sensors,
        timeout_s=args.timeout,
        packet_timeout_s=args.packet_timeout,
    )
    client: PapillArraySerialClient | None = None
    previous_counter: int | None = None
    result = 0
    try:
        client = client_factory(config)
        client.open()
        client.configure_stream()
        for _ in range(args.count):
            packet = client.read_packet()
            if packet.n_sensors != config.expected_sensors:
                raise RuntimeError(
                    "PapillArray 传感器数与部署配置不一致："
                    f"期望 {config.expected_sensors}，实际 {packet.n_sensors}"
                )
            receipt_ns = monotonic_ns()
            counter_event, gap = _counter_status(previous_counter, packet.packet_counter)
            record = _packet_record(packet, receipt_ns, counter_event, gap)
            print(
                json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":")),
                file=stdout,
                flush=True,
            )
            if counter_event != "out_of_order":
                previous_counter = packet.packet_counter
    except KeyboardInterrupt:
        print("采集已由 Ctrl-C 中断，串口已关闭。", file=stderr)
        result = 130
    except PtsReadTimeout as exc:
        print(
            f"PapillArray 探针超时：{exc}。{_format_diagnostics(exc.diagnostics)}"
            "请检查供电、接线、设备协议，以及 --packet-timeout／--timeout。",
            file=stderr,
        )
        result = 1
    except TimeoutError as exc:
        print(
            f"PapillArray 探针超时：{exc}。请检查供电、接线和 --packet-timeout／--timeout。",
            file=stderr,
        )
        result = 1
    except ProtocolError as exc:
        print(f"PapillArray 探针协议错误：{exc}。请检查设备输出与协议版本。", file=stderr)
        result = 1
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"PapillArray 探针失败：{exc}。请检查端口、参数和传感器数量。", file=stderr)
        result = 1
    finally:
        if client is not None:
            try:
                client.close()
            except Exception as exc:
                print(f"PapillArray 探针关闭串口失败：{exc}。请手动检查端口状态。", file=stderr)
                result = result or 1
    return result


def main() -> NoReturn:
    """作为 console script 执行探针，并以采集结果退出。"""
    raise SystemExit(run())

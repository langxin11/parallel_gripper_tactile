#!/usr/bin/env python3
"""最接近驱动主循环的同步教学版：持续观测并做新鲜度与丢包统计。

运行示例：

    uv run --package papillarray-hardware \
        python packages/papillarray_hardware/examples/monitor.py --port /dev/papillarray
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from typing import NoReturn, TextIO

from papillarray_hardware import (
    PapillArraySerialClient,
    PapillArraySerialConfig,
    ProtocolError,
    PtsReadDiagnostics,
    PtsReadTimeout,
)
from papillarray_hardware.client import SUPPORTED_SAMPLING_RATES

ClientFactory = Callable[[PapillArraySerialConfig], PapillArraySerialClient]

_COUNTER_MODULUS = 2**32
_WARMUP_MAX_PACKETS = 100
"""预热阶段最多读取的包数；超过即放弃等待计数器恢复连续。"""


def build_parser() -> argparse.ArgumentParser:
    """创建本示例的参数解析器。"""
    parser = argparse.ArgumentParser(
        description="持续读取 PapillArray 数据包并做观测新鲜度与丢包统计。"
    )
    parser.add_argument(
        "--port",
        default="/dev/papillarray",
        help="串口端点，默认 udev 别名，无别名时用 /dev/ttyACM*",
    )
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument(
        "--rate",
        type=int,
        choices=sorted(SUPPORTED_SAMPLING_RATES),
        default=1000,
        help="采样率（Hz），默认 1000",
    )
    parser.add_argument("--expected-sensors", type=int, default=2, help="期望传感器数，默认 2")
    parser.add_argument("--count", type=int, default=1000, help="采集包数，默认 1000")
    parser.add_argument(
        "--timeout", type=float, default=1.0, help="单次底层串口读取超时秒数，默认 1"
    )
    parser.add_argument(
        "--packet-timeout", type=float, default=3.0, help="单包等待总时限秒数，默认 3"
    )
    parser.add_argument(
        "--bias",
        action="store_true",
        help="发送清零命令，执行前传感器必须完全无负载",
    )
    parser.add_argument("--stale-sec", type=float, default=0.1, help="包间隔警告阈值秒数，默认 0.1")
    return parser


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


def _counter_status(
    previous_counter: int | None, packet_counter: int
) -> tuple[str, int | None, int]:
    """按无符号 32 位半模规则分类包计数器变化。

    Args:
        previous_counter: 上一包计数器；首包时为 `None`。
        packet_counter: 当前包计数器。

    Returns:
        (事件, 模差值, 缺包数)：事件为 first／consecutive／gap／duplicate／
        out_of_order 之一；模差值按模 2**32 计算，首包为 `None`；缺包数为可解释
        的缺失包数，非 gap 事件为 `0`。
    """
    if previous_counter is None:
        return "first", None, 0
    delta = (packet_counter - previous_counter) % _COUNTER_MODULUS
    if delta == 0:
        return "duplicate", delta, 0
    if delta >= _COUNTER_MODULUS // 2:
        return "out_of_order", delta, 0
    if delta > 1:
        return "gap", delta, delta - 1
    return "consecutive", delta, 0


def run(
    argv: Sequence[str] | None = None,
    *,
    client_factory: ClientFactory = lambda config: PapillArraySerialClient(config),
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
) -> int:
    """持续读取触觉包并统计新鲜度与丢包，返回退出码。

    统计从预热（观察到计数器启动跳变并恢复连续）之后开始，串口重开残留的
    陈旧包不计入缺包。

    Args:
        argv: 命令行参数；省略时读取当前进程参数。
        client_factory: 可注入客户端工厂，供离线测试使用。
        monotonic_ns: 可注入的主机单调时钟，单位纳秒。
        stdout: 统计输出流。
        stderr: 警告与诊断输出流。

    Returns:
        成功为 `0`，采集错误为非零值。
    """
    args = build_parser().parse_args(argv)
    config = PapillArraySerialConfig(
        port=args.port,
        baud_rate=args.baud,
        sampling_rate=args.rate,
        expected_sensors=args.expected_sensors,
        timeout_s=args.timeout,
        packet_timeout_s=args.packet_timeout,
    )
    client: PapillArraySerialClient | None = None
    result = 0
    received = 0
    missing_total = 0
    duplicates = 0
    out_of_order = 0
    max_interval_ms = 0.0
    previous_counter: int | None = None
    previous_arrival_ns: int | None = None
    try:
        client = client_factory(config)
        client.open()
        if args.bias:
            print("即将清零：请确认传感器完全无负载。", file=stderr)
            client.clear_bias()
        client.configure_stream()
        # 预热：串口重开后设备会先吐出上次会话残留的旧帧（这些旧帧彼此连续），
        # 随后才跳到当前帧。因此仅"相邻两包连续"不构成新鲜证据，必须观察到一次
        # 计数器大步跳变、且跳变后恢复连续，才以该处为统计起点；若整个预热窗口
        # 内没有跳变，说明没有积压旧帧，直接开始统计（首包按 first 计，不计缺包）。
        prev_warm_counter: int | None = None
        jumped = False
        discarded_stale = 0
        skipped_samples = 0
        for _ in range(_WARMUP_MAX_PACKETS):
            warmup_packet = client.read_packet()
            warmup_arrival_ns = monotonic_ns()
            if prev_warm_counter is not None:
                delta = (warmup_packet.packet_counter - prev_warm_counter) % _COUNTER_MODULUS
                if 0 < delta < _COUNTER_MODULUS // 2:
                    if delta > config.sampling_rate // 10:
                        jumped = True
                        skipped_samples += delta - 1
                    elif jumped:
                        previous_counter = warmup_packet.packet_counter
                        previous_arrival_ns = warmup_arrival_ns
                        print(
                            f"已丢弃 {discarded_stale} 个启动陈旧包"
                            f"（跨过 {skipped_samples} 个样本的启动跳变），"
                            f"从计数器 {previous_counter} 开始统计。",
                            file=stderr,
                        )
                        break
            prev_warm_counter = warmup_packet.packet_counter
            discarded_stale += 1
        for _ in range(args.count):
            packet = client.read_packet()
            arrival_ns = monotonic_ns()
            if previous_arrival_ns is not None:
                interval_ms = (arrival_ns - previous_arrival_ns) / 1e6
                if interval_ms > max_interval_ms:
                    max_interval_ms = interval_ms
                if interval_ms > args.stale_sec * 1000.0:
                    print(
                        f"包间隔 {interval_ms:.1f} ms 超过 --stale-sec {args.stale_sec}",
                        file=stderr,
                    )
            previous_arrival_ns = arrival_ns
            event, delta, missing = _counter_status(previous_counter, packet.packet_counter)
            delta_text = "first" if delta is None else str(delta)
            print(f"counter={packet.packet_counter} Δ={delta_text}", file=stdout, flush=True)
            if event == "duplicate":
                duplicates += 1
            elif event == "out_of_order":
                out_of_order += 1
            elif event == "gap":
                missing_total += missing
            if event != "out_of_order":
                previous_counter = packet.packet_counter
            received += 1
            if received % 500 == 0:
                print(
                    f"已收 {received} 包，缺包 {missing_total}，重复 {duplicates}，"
                    f"乱序 {out_of_order}，最大间隔 {max_interval_ms:.1f} ms",
                    file=stdout,
                    flush=True,
                )
    except KeyboardInterrupt:
        print("已由 Ctrl-C 中断，串口已关闭。", file=stderr)
        result = 130
    except PtsReadTimeout as exc:
        print(
            f"读取 PapillArray 数据包超时：{exc}。{_format_diagnostics(exc.diagnostics)}"
            "可尝试 --baud 9600（出厂波特率）排查，并确认供电与接线。",
            file=stderr,
        )
        result = 1
    except TimeoutError as exc:
        print(f"读取 PapillArray 超时：{exc}。请检查供电与接线。", file=stderr)
        result = 1
    except ProtocolError as exc:
        print(f"PapillArray 协议错误：{exc}。请检查设备输出与协议版本。", file=stderr)
        result = 1
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"持续监测失败：{exc}。请检查端口、参数和传感器数量。", file=stderr)
        result = 1
    finally:
        print(
            f"最终摘要：已收 {received} 包，缺包 {missing_total}，重复 {duplicates}，"
            f"乱序 {out_of_order}，最大间隔 {max_interval_ms:.1f} ms",
            file=stdout,
            flush=True,
        )
        if client is not None:
            try:
                client.close()
            except Exception as exc:
                print(f"关闭串口失败：{exc}。请手动检查端口状态。", file=stderr)
                result = result or 1
    return result


def main() -> NoReturn:
    """作为脚本执行本示例，并以监测结果退出。"""
    raise SystemExit(run())


if __name__ == "__main__":
    main()

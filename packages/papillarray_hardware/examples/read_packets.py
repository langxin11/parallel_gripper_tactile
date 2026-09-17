#!/usr/bin/env python3
"""读取有限个 PapillArray 触觉包并打印人类可读的全局力表格。

运行示例：

    uv run --package papillarray-hardware \
        python packages/papillarray_hardware/examples/read_packets.py --port /dev/papillarray
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable, Sequence
from typing import NoReturn, TextIO

from papillarray_hardware import (
    PapillArraySerialClient,
    PapillArraySerialConfig,
    ProtocolError,
    PtsReadTimeout,
)
from papillarray_hardware.client import SUPPORTED_SAMPLING_RATES

ClientFactory = Callable[[PapillArraySerialConfig], PapillArraySerialClient]


def build_parser() -> argparse.ArgumentParser:
    """创建本示例的参数解析器。"""
    parser = argparse.ArgumentParser(description="读取有限个触觉包并打印指定传感器的全局力表格。")
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
    parser.add_argument("--sensor", type=int, default=0, help="打印哪个传感器，默认 0")
    parser.add_argument("--count", type=int, default=10, help="采集包数，默认 10")
    parser.add_argument(
        "--timeout", type=float, default=1.0, help="单次底层串口读取超时秒数，默认 1"
    )
    parser.add_argument(
        "--packet-timeout", type=float, default=3.0, help="单包等待总时限秒数，默认 3"
    )
    return parser


def run(
    argv: Sequence[str] | None = None,
    *,
    client_factory: ClientFactory = lambda config: PapillArraySerialClient(config),
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
) -> int:
    """读取有限个触觉包并打印全局力表格，返回退出码。

    Args:
        argv: 命令行参数；省略时读取当前进程参数。
        client_factory: 可注入客户端工厂，供离线测试使用。
        stdout: 表格输出流。
        stderr: 诊断输出流。

    Returns:
        成功为 `0`，读取错误为非零值。
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
    try:
        client = client_factory(config)
        client.open()
        client.configure_stream()
        print("Sample | Timestamp (µs) | Fx (N) | Fy (N) | Fz (N)", file=stdout, flush=True)
        for index in range(args.count):
            packet = client.read_packet()
            if not 0 <= args.sensor < len(packet.global_forces):
                raise RuntimeError(
                    f"数据包仅包含 {len(packet.global_forces)} 个传感器，无法读取 SEN{args.sensor}"
                )
            fx, fy, fz = packet.global_forces[args.sensor].tolist()
            print(
                f"{index:<8} | {packet.timestamp_us:<18} | {fx:<12.4f} | {fy:<12.4f} | {fz:<12.4f}",
                file=stdout,
                flush=True,
            )
    except KeyboardInterrupt:
        print("已由 Ctrl-C 中断，串口已关闭。", file=stderr)
        result = 130
    except PtsReadTimeout as exc:
        print(
            f"读取 PapillArray 数据包超时：{exc}。"
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
        print(f"读取触觉数据包失败：{exc}。请检查端口、参数和传感器数量。", file=stderr)
        result = 1
    finally:
        if client is not None:
            try:
                client.close()
            except Exception as exc:
                print(f"关闭串口失败：{exc}。请手动检查端口状态。", file=stderr)
                result = result or 1
    return result


def main() -> NoReturn:
    """作为脚本执行本示例，并以读取结果退出。"""
    raise SystemExit(run())


if __name__ == "__main__":
    main()

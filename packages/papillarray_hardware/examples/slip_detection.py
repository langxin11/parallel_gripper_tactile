#!/usr/bin/env python3
"""演示向 PapillArray 发送滑动检测启停命令 S/s 并读取滑动状态。

`papillarray-probe` 探针 CLI 刻意不发送这两条命令，本示例补全演示。运行示例：

    uv run --package papillarray-hardware \
        python packages/papillarray_hardware/examples/slip_detection.py --port /dev/papillarray
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
    parser = argparse.ArgumentParser(
        description="启动滑动检测并打印每传感器的 pillar 滑动状态与目标抓握力。"
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
        default=500,
        help="采样率（Hz），默认 500",
    )
    parser.add_argument("--expected-sensors", type=int, default=2, help="期望传感器数，默认 2")
    parser.add_argument("--count", type=int, default=20, help="采集包数，默认 20")
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
    r"""启动滑动检测并读取有限个包，返回退出码。

    每包一行紧凑输出后，再逐传感器缩进打印滑动检测状态。finally 中始终尽力发送
    `s\n` 停止滑动检测，然后关闭串口。

    Args:
        argv: 命令行参数；省略时读取当前进程参数。
        client_factory: 可注入客户端工厂，供离线测试使用。
        stdout: 状态输出流。
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
    warned_missing_slip = False
    try:
        client = client_factory(config)
        client.open()
        client.configure_stream()
        client.start_slip_detection()
        for index in range(args.count):
            packet = client.read_packet()
            print(
                f"[{index}] counter={packet.packet_counter} ts={packet.timestamp_us}µs",
                file=stdout,
                flush=True,
            )
            has_slip_block = (
                len(packet.pillar_slip_states) >= packet.n_sensors
                and len(packet.slip_detection_active) >= packet.n_sensors
            )
            if not has_slip_block:
                # 刚发 S\n 后的部分包可能尚未携带 Type 5/6 块，提示一次后跳过该包。
                if not warned_missing_slip:
                    print(
                        "部分数据包未包含滑动状态块（Type 5/6），已跳过其传感器行。",
                        file=stderr,
                    )
                    warned_missing_slip = True
                continue
            for sensor_index in range(packet.n_sensors):
                pillar_text = " ".join(
                    str(state) for state in packet.pillar_slip_states[sensor_index].tolist()
                )
                print(
                    f"  SEN{sensor_index} active={packet.slip_detection_active[sensor_index]}"
                    f" ref_loaded={packet.reference_pillar_loaded[sensor_index]}"
                    f" 摩擦={packet.sensor_friction_estimates[sensor_index]:.2f}"
                    f" 目标力={packet.target_grip_forces[sensor_index]:.2f}N"
                    f" pillar=[{pillar_text}]",
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
        print(f"演示滑动检测失败：{exc}。请检查端口、参数和传感器数量。", file=stderr)
        result = 1
    finally:
        if client is not None and client.is_open:
            try:
                client.stop_slip_detection()
            except Exception as exc:
                print(f"停止滑动检测命令发送失败：{exc}", file=stderr)
        if client is not None:
            try:
                client.close()
            except Exception as exc:
                print(f"关闭串口失败：{exc}。请手动检查端口状态。", file=stderr)
                result = result or 1
    return result


def main() -> NoReturn:
    """作为脚本执行本示例，并以采集结果退出。"""
    raise SystemExit(run())


if __name__ == "__main__":
    main()

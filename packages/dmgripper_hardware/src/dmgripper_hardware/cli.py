"""DM4310P 只读状态探针命令行入口。"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections.abc import Callable, Sequence
from typing import NoReturn, TextIO

from .deployment import Dm4310PGripperConfig, make_dm4310p_gripper_config
from .protocol import MotorFeedback, Usb2CanProtocol, motor_status_is_fault
from .runtime import DmStateRefresher
from .serial_transport import PySerialTransport
from .transport import ByteTransport

DEFAULT_COUNT = 10
DEFAULT_INTERVAL_S = 0.1
DEFAULT_TIMEOUT_S = 0.05

TransportFactory = Callable[[], ByteTransport]
"""创建一个尚未打开的字节传输对象。"""


def _finite_float(value: str) -> float:
    """解析有限浮点数，用于命令行参数校验。"""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是数字") from exc
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("必须是有限数值")
    return parsed


def _positive_float(value: str) -> float:
    """解析正有限浮点数。"""
    parsed = _finite_float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def _nonnegative_float(value: str) -> float:
    """解析非负有限浮点数。"""
    parsed = _finite_float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("不能小于 0")
    return parsed


def _positive_int(value: str) -> int:
    """解析正整数。"""
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是整数") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    """创建状态探针参数解析器。"""
    parser = argparse.ArgumentParser(description="DM4310P 只读状态探针（不会发送运动命令）")
    parser.add_argument("--port", required=True, help="USB2CAN 串口端点")
    parser.add_argument(
        "--count",
        type=_positive_int,
        default=DEFAULT_COUNT,
        help=f"读取次数，默认 {DEFAULT_COUNT}",
    )
    parser.add_argument(
        "--interval",
        type=_nonnegative_float,
        default=DEFAULT_INTERVAL_S,
        help=f"两次读取之间的间隔（秒），默认 {DEFAULT_INTERVAL_S}",
    )
    parser.add_argument(
        "--timeout",
        type=_positive_float,
        default=DEFAULT_TIMEOUT_S,
        help=f"每次等待反馈的超时（秒），默认 {DEFAULT_TIMEOUT_S}",
    )
    return parser


def _record(
    feedback: MotorFeedback,
    deployment: Dm4310PGripperConfig,
    sequence: int,
    receipt_ns: int,
) -> dict[str, object]:
    """将一条反馈转换为稳定字段顺序的 JSON 记录。"""
    position = feedback.position_rad
    within_range = (
        deployment.joint_position_min_rad <= position <= deployment.joint_position_max_rad
    )
    return {
        "host_monotonic_receipt_ns": receipt_ns,
        "sequence": sequence,
        "position_rad": position,
        "velocity_rad_s": feedback.velocity_rad_s,
        "torque_nm": feedback.torque_nm,
        "status_code": feedback.status_code,
        "fault": motor_status_is_fault(feedback.status_code),
        "within_mechanical_range": within_range,
    }


def _default_transport_factory() -> ByteTransport:
    """创建默认的延迟打开串口传输。"""
    return PySerialTransport()


def run(
    argv: Sequence[str] | None = None,
    *,
    transport_factory: TransportFactory = _default_transport_factory,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """执行有限次数的只读状态探针。

    Args:
        argv: 命令行参数；为 `None` 时读取进程参数。
        transport_factory: 创建传输的依赖注入工厂，生产环境默认使用 PySerial。
        monotonic_ns: 主机单调时钟，测试可注入。
        stdout: JSON Lines 输出流，默认标准输出。
        stderr: 中文诊断输出流，默认标准错误。
        sleep: 间隔等待函数，测试可注入而不实际等待。

    Returns:
        成功返回 `0`；异常或用户中断返回非零值。
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    refresher: DmStateRefresher | None = None
    result = 0
    try:
        deployment = make_dm4310p_gripper_config(args.port, timeout_s=args.timeout)
        protocol = Usb2CanProtocol(deployment.motor_limits)
        refresher = DmStateRefresher(deployment.device, protocol, transport_factory())
        refresher.open()
        for sequence in range(1, args.count + 1):
            feedback = refresher.refresh_once()
            record = _record(feedback, deployment, sequence, monotonic_ns())
            stdout.write(
                json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                + "\n"
            )
            stdout.flush()
            if sequence != args.count and args.interval > 0.0:
                sleep(args.interval)
    except KeyboardInterrupt:
        stderr.write("状态探针已由用户中断，传输已关闭。\n")
        result = 130
    except Exception as exc:  # noqa: BLE001
        stderr.write(f"状态探针失败：{exc}\n")
        result = 1
    finally:
        if refresher is not None:
            try:
                refresher.close()
            except Exception as exc:  # noqa: BLE001
                stderr.write(f"关闭状态探针传输失败：{exc}\n")
                result = 1
    return result


def run_probe(
    argv: Sequence[str] | None = None,
    *,
    transport_factory: TransportFactory | None = None,
    output: TextIO | None = None,
    error: TextIO | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """兼容旧调用名，执行只读状态探针。"""
    if transport_factory is None:
        transport_factory = _default_transport_factory
    if output is None:
        output = sys.stdout
    if error is None:
        error = sys.stderr
    return run(
        argv,
        transport_factory=transport_factory,
        stdout=output,
        stderr=error,
        sleep=sleep,
    )


def main(argv: Sequence[str] | None = None) -> NoReturn:
    """运行 DM4310P 只读状态探针命令行。"""
    raise SystemExit(run(argv))


if __name__ == "__main__":  # pragma: no cover
    main()

"""最小纯 Python 真机力跟踪命令行。"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import NoReturn

from dmgripper_hardware import DEFAULT_USB2CAN_PORT
from papillarray_hardware import DEFAULT_PAPILLARRAY_PORT

from .config import ForceDemoConfig
from .runtime import config_record, run_force_demo


def build_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="DM4310P 轨迹预接触与 PapillArray 导纳力跟踪（默认 dry-run）",
        allow_abbrev=False,
    )
    parser.add_argument("--execute", action="store_true", help="显式许可使能电机并执行实验")
    parser.add_argument("--bias", action="store_true", help="使能前向无负载触觉传感器发送清零命令")
    parser.add_argument("--dm-port", default=DEFAULT_USB2CAN_PORT)
    parser.add_argument("--tactile-port", default=DEFAULT_PAPILLARRAY_PORT)
    parser.add_argument("--target-force", type=float, default=0.5, metavar="N")
    parser.add_argument("--duration", type=float, default=10.0, metavar="S")
    parser.add_argument(
        "--return-closure-velocity",
        type=float,
        default=0.012,
        metavar="M/S",
        help="回位闭合量轨迹最大速度，默认 0.012 m/s",
    )
    parser.add_argument(
        "--zero-force-threshold",
        type=float,
        default=0.1,
        metavar="N",
        help="使能前每侧允许的零力绝对值，默认 0.1 N",
    )
    parser.add_argument(
        "--bias-settle",
        type=float,
        default=2.0,
        metavar="S",
        help="bias 后保持无负载的等待时间，默认与 ROS 2 一致为 2 秒",
    )
    parser.add_argument(
        "--zero-force-stable",
        type=float,
        default=0.5,
        metavar="S",
        help="零力窗口连续稳定时间，默认 0.5 秒",
    )
    parser.add_argument(
        "--zero-force-timeout",
        type=float,
        default=5.0,
        metavar="S",
        help="零力验证总时限，默认 5 秒",
    )
    parser.add_argument(
        "--skip-zero-check",
        action="store_true",
        help="显式跳过零力稳定验证；仅在已独立确认清零后使用",
    )
    parser.add_argument(
        "--tactile-startup-timeout",
        type=float,
        default=5.0,
        metavar="S",
        help="首次连接触觉并等待有效包的总时限，默认 5 秒",
    )
    parser.add_argument("--output", type=Path, default=None, metavar="CSV")
    return parser


def run(argv: Sequence[str] | None = None) -> int:
    """解析参数；dry-run 只显示计划，execute 才访问设备。"""
    args = build_parser().parse_args(argv)
    try:
        config = ForceDemoConfig(
            target_force_n=args.target_force,
            tracking_duration_s=args.duration,
            tactile_startup_timeout_s=args.tactile_startup_timeout,
            tactile_bias_settle_s=args.bias_settle,
            return_closure_velocity_m_s=args.return_closure_velocity,
            zero_force_threshold_n=args.zero_force_threshold,
            zero_force_stable_s=args.zero_force_stable,
            zero_force_timeout_s=args.zero_force_timeout,
        )
        output = args.output or Path(
            f"outputs/real/dm_force_demo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        if not args.execute:
            print(
                json.dumps(
                    {
                        "mode": "dry-run",
                        "dm_port": args.dm_port,
                        "tactile_port": args.tactile_port,
                        "clear_bias": args.bias,
                        "verify_zero_force": not args.skip_zero_check,
                        "output": str(output),
                        "config": config_record(config),
                    },
                    ensure_ascii=False,
                    allow_nan=False,
                )
            )
            return 0

        def emit(event: dict[str, object]) -> None:
            print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)

        result = run_force_demo(
            config,
            dm_port=args.dm_port,
            tactile_port=args.tactile_port,
            output_path=output,
            clear_bias=args.bias,
            verify_zero_force=not args.skip_zero_check,
            event_sink=emit,
        )
        print(
            json.dumps(
                {"event": "complete", **as_result_dict(result)},
                ensure_ascii=False,
                allow_nan=False,
            )
        )
        return 0
    except KeyboardInterrupt:
        print("真机力跟踪已中断，已执行退出失能流程。", file=sys.stderr)
        return 130
    except Exception as error:  # noqa: BLE001
        print(f"真机力跟踪失败：{error}", file=sys.stderr)
        return 1


def as_result_dict(result: object) -> dict[str, object]:
    """将带 slots 的结果对象转换为稳定 JSON 字段。"""
    return {
        "completed": getattr(result, "completed"),
        "disable_confirmed": getattr(result, "disable_confirmed"),
        "final_state": getattr(result, "final_state"),
        "final_position_rad": getattr(result, "final_position_rad"),
        "csv_path": getattr(result, "csv_path"),
    }


def main(argv: Sequence[str] | None = None) -> NoReturn:
    """运行命令行并返回进程状态。"""
    raise SystemExit(run(argv))

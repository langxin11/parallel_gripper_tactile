"""DM4310P 受限小步运动探针命令行入口。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections.abc import Callable, Sequence
from typing import NoReturn, TextIO

from .cli import TransportFactory, _default_transport_factory, _positive_float
from .deployment import make_dm4310p_gripper_config
from .motion import DmSafeMotionProbe, MotionProbeSafetyConfig, MotionProbePlan, MotionStageResult
from .protocol import Usb2CanProtocol


def build_parser() -> argparse.ArgumentParser:
    """创建受限小步运动探针参数解析器。"""
    parser = argparse.ArgumentParser(
        description="DM4310P 受限小步运动探针（默认 dry-run；仅 --execute 才会驱动电机）",
        allow_abbrev=False,
    )
    parser.add_argument("--port", required=True, help="USB2CAN 串口端点")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="显式许可使能、保持、按配置步长闭合、回位及最终失能",
    )
    parser.add_argument(
        "--timeout", type=_positive_float, default=0.05, help="每次反馈读取超时（秒），默认 0.05"
    )
    parser.add_argument(
        "--stage-duration",
        type=_positive_float,
        default=1.0,
        help="每个阻抗平衡点阶段的固定持续时间（秒），默认 1.0",
    )
    parser.add_argument(
        "--mit-kp",
        type=_positive_float,
        default=2.0,
        help="MIT 位置刚度，默认 2.0；实际运动必须位于 (0, 500]",
    )
    parser.add_argument(
        "--closing-step",
        type=_positive_float,
        default=0.03,
        help="闭合增量（rad），默认 0.03；最终机械目标必须位于部署行程内",
    )
    return parser


def _plan_record(plan: MotionProbePlan, safety: MotionProbeSafetyConfig) -> dict[str, object]:
    """将 dry-run 计划转换为稳定字段顺序的 JSON 记录。"""
    return {
        "mode": "dry-run",
        "initial_position_rad": plan.initial_position_rad,
        "initial_velocity_rad_s": plan.initial_feedback.velocity_rad_s,
        "initial_torque_nm": plan.initial_feedback.torque_nm,
        "initial_status_code": plan.initial_feedback.status_code,
        "control_mode": plan.control_mode,
        "hold_target_rad": plan.hold_target_rad,
        "close_target_rad": plan.close_target_rad,
        "return_target_rad": plan.return_target_rad,
        "mit_kp": safety.mit_kp,
        "mit_kd": safety.mit_kd,
        "closing_step_rad": safety.closing_step_rad,
        "stage_duration_s": safety.stage_duration_s,
        "actuator_command_writes": 0,
    }


def _stage_record(result: MotionStageResult) -> dict[str, object]:
    """将已完成阶段转换为稳定字段顺序的 JSON 记录。"""
    return {
        "mode": "execute",
        "stage": result.stage,
        "target_position_rad": result.target_position_rad,
        "target_reached": result.target_reached,
        "position_error_rad": result.position_error_rad,
        "start_position_rad": result.start_position_rad,
        "final_position_rad": result.final_position_rad,
        "max_position_rad": result.max_position_rad,
        "min_position_rad": result.min_position_rad,
        "max_abs_velocity_rad_s": result.max_abs_velocity_rad_s,
        "max_abs_torque_nm": result.max_abs_torque_nm,
        "command_count": result.command_count,
        "elapsed_s": result.elapsed_s,
    }


def run(
    argv: Sequence[str] | None = None,
    *,
    transport_factory: TransportFactory = _default_transport_factory,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """运行默认只读的 DM4310P 受限小步运动探针。"""
    args = build_parser().parse_args(argv)
    probe: DmSafeMotionProbe | None = None
    execute_attempted = False
    result = 0
    try:
        deployment = make_dm4310p_gripper_config(args.port, timeout_s=args.timeout)
        safety = MotionProbeSafetyConfig(
            stage_duration_s=args.stage_duration,
            mit_kp=args.mit_kp,
            closing_step_rad=args.closing_step,
        )
        probe = DmSafeMotionProbe(
            deployment,
            Usb2CanProtocol(deployment.motor_limits),
            transport_factory(),
            safety=safety,
            sleep=sleep,
        )
        probe.open()
        plan = probe.inspect_and_plan()
        if not args.execute:
            stdout.write(
                json.dumps(_plan_record(plan, safety), ensure_ascii=False, allow_nan=False) + "\n"
            )
            stdout.flush()
        else:
            execute_attempted = True
            for stage in probe.execute(plan):
                stdout.write(
                    json.dumps(_stage_record(stage), ensure_ascii=False, allow_nan=False) + "\n"
                )
                stdout.flush()
            if not probe.last_disable_confirmed:
                raise RuntimeError("运动返回成功但未确认最终失能")
            stdout.write(
                json.dumps(
                    {"mode": "execute", "event": "complete", "disable_confirmed": True},
                    ensure_ascii=False,
                    allow_nan=False,
                )
                + "\n"
            )
            stdout.flush()
    except KeyboardInterrupt:
        if execute_attempted:
            stderr.write("受限小步运动探针已由用户中断，已尽力失能并关闭传输。\n")
        else:
            stderr.write("受限小步运动探针已由用户中断，未发送执行命令，传输已关闭。\n")
        result = 130
    except Exception as exc:  # noqa: BLE001
        stderr.write(f"受限小步运动探针失败：{exc}\n")
        result = 1
    finally:
        if probe is not None:
            try:
                probe.close()
            except Exception as exc:  # noqa: BLE001
                stderr.write(f"关闭受限小步运动探针传输失败：{exc}\n")
                result = 1
    return result


def main(argv: Sequence[str] | None = None) -> NoReturn:
    """运行 DM4310P 受限小步运动探针命令行。"""
    raise SystemExit(run(argv))

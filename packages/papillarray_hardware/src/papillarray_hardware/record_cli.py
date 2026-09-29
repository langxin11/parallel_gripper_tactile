"""PapillArray 独立清零、连续记录与短时滑移辨识入口。"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import select
import sys
import threading
import time
from typing import NoReturn, TextIO
import uuid

from .acquisition import TactileWorker
from .client import (
    DEFAULT_PAPILLARRAY_PORT,
    PapillArraySerialConfig,
    SUPPORTED_SAMPLING_RATES,
)
from .standalone import StandaloneSlipConfig, StandaloneSlipSession


def _positive_float(value: str) -> float:
    """解析正有限浮点数。"""
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("必须是数字") from error
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("必须是正有限数")
    return result


def _positive_int(value: str) -> int:
    """解析正整数。"""
    try:
        result = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("必须是整数") from error
    if result <= 0:
        raise argparse.ArgumentTypeError("必须是正整数")
    return result


def build_parser() -> argparse.ArgumentParser:
    """创建独立记录器命令行参数。"""
    parser = argparse.ArgumentParser(
        description=(
            "PapillArray 独立记录器：可在空载时 bias，持续记录双侧逐触点数据，"
            "并在稳定接触后运行自主逐 pillar 摩擦估计及可选原厂对照。"
        ),
        allow_abbrev=False,
    )
    parser.add_argument("--port", default=DEFAULT_PAPILLARRAY_PORT)
    parser.add_argument("--baud", type=_positive_int, default=115200)
    parser.add_argument(
        "--rate", type=_positive_int, choices=sorted(SUPPORTED_SAMPLING_RATES), default=1000
    )
    parser.add_argument("--timeout", type=_positive_float, default=0.05)
    parser.add_argument("--packet-timeout", type=_positive_float, default=0.2)
    parser.add_argument(
        "--bias",
        action="store_true",
        help="首个完整包后发送一次清零命令；启动时传感器必须完全无负载",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="新的运行目录；省略时自动写入 outputs/real/papillarray/",
    )
    parser.add_argument(
        "--estimate-friction",
        action="store_true",
        help="稳定接触后只用逐 pillar 三轴力自主估计摩擦，不调用原厂服务",
    )
    parser.add_argument(
        "--native-slip",
        action="store_true",
        help="稳定接触后启动一次原厂滑移服务，由操作者按 Enter 停止",
    )
    parser.add_argument(
        "--native-slip-duration",
        type=_positive_float,
        default=None,
        help="可选最长运行秒数；传入时也会启用原厂滑移服务",
    )
    parser.add_argument("--stable-duration", type=_positive_float, default=0.5)
    parser.add_argument("--max-force-rate", type=_positive_float, default=0.5)
    parser.add_argument("--contact-on", type=_positive_float, default=0.15)
    parser.add_argument("--contact-off", type=_positive_float, default=0.1)
    parser.add_argument("--sample-timeout", type=_positive_float, default=0.05)
    parser.add_argument("--confirmation-timeout", type=_positive_float, default=0.2)
    parser.add_argument("--min-estimates-per-side", type=_positive_int, default=1)
    parser.add_argument(
        "--stop-on-estimates-ready",
        action="store_true",
        help="仅原厂对照模式：双侧取得所需原厂估计后自动停止服务",
    )
    parser.add_argument(
        "--max-seconds",
        type=_positive_float,
        default=None,
        help="可选自动结束时长；省略时按 Ctrl-C 结束",
    )
    return parser


class JsonlRunRecorder:
    """线程安全地保存触觉包、事件、配置与最终状态。"""

    def __init__(self, directory: Path, config: dict[str, object]) -> None:
        """独占创建运行目录并打开两个 JSON Lines 文件。"""
        directory.mkdir(parents=True, exist_ok=False)
        self.directory = directory
        self._lock = threading.Lock()
        self._tactile = (directory / "tactile.jsonl").open("x", encoding="utf-8")
        self._events = (directory / "events.jsonl").open("x", encoding="utf-8")
        (directory / "config.json").write_text(
            json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        self._closed = False

    def sample(self, record: dict[str, object]) -> None:
        """从采集线程追加一个完整快照。"""
        self._write(self._tactile, record)

    def event(self, record: dict[str, object]) -> None:
        """从主线程追加一个生命周期或逐触点估计事件。"""
        self._write(self._events, record)

    def close(self, *, status: str, error: BaseException | None = None) -> None:
        """刷新并关闭数据文件，再原子语义地写入最终清单。"""
        with self._lock:
            if self._closed:
                return
            self._tactile.close()
            self._events.close()
            self._closed = True
        manifest: dict[str, object] = {
            "schema": "papillarray-standalone/v2",
            "status": status,
            "tactile": "tactile.jsonl",
            "events": "events.jsonl",
        }
        if error is not None:
            manifest["error"] = f"{type(error).__name__}: {error}"
        (self.directory / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )

    def _write(self, handle: TextIO, record: dict[str, object]) -> None:
        """锁内写入并立即刷新；进程退出后即可读，不保证断电与内核崩溃。"""
        with self._lock:
            if self._closed:
                raise RuntimeError("记录器已经关闭")
            handle.write(
                json.dumps(record, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n"
            )
            handle.flush()


def _default_output() -> Path:
    """生成不会覆盖既有记录的默认运行目录。"""
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("outputs/real/papillarray") / f"{timestamp}-{uuid.uuid4().hex[:8]}"


def _manual_stop_requested(stream: TextIO) -> bool:
    """非阻塞地读取交互终端中的一次 Enter；重定向输入不触发人工停止。"""
    if not stream.isatty():
        return False
    try:
        readable, _, _ = select.select([stream], [], [], 0.0)
    except (OSError, TypeError, ValueError):
        return False
    return bool(readable) and stream.readline() != ""


def run(argv: list[str] | None = None) -> int:
    """运行独立记录器；Ctrl-C 被视为人工完成并执行确认停止。"""
    args = build_parser().parse_args(argv)
    if args.packet_timeout < args.timeout:
        raise ValueError("--packet-timeout 不得小于 --timeout")
    serial_config = PapillArraySerialConfig(
        port=args.port,
        baud_rate=args.baud,
        sampling_rate=args.rate,
        expected_sensors=2,
        timeout_s=args.timeout,
        packet_timeout_s=args.packet_timeout,
    )
    slip_requested = args.native_slip or args.native_slip_duration is not None
    estimation_requested = args.estimate_friction or slip_requested
    slip_config = (
        StandaloneSlipConfig(
            max_duration_s=args.native_slip_duration,
            native_slip_enabled=slip_requested,
            own_friction_enabled=estimation_requested,
            stable_duration_s=args.stable_duration,
            max_force_rate_n_s=args.max_force_rate,
            contact_on_n=args.contact_on,
            contact_off_n=args.contact_off,
            sample_timeout_s=args.sample_timeout,
            confirmation_timeout_s=args.confirmation_timeout,
            min_estimates_per_side=args.min_estimates_per_side,
            stop_when_estimates_ready=args.stop_on_estimates_ready,
        )
        if estimation_requested
        else None
    )
    directory = args.output or _default_output()
    config_record: dict[str, object] = {
        "serial": asdict(serial_config),
        "bias": args.bias,
        "stable_contact": (
            {
                "stable_duration_s": slip_config.stable_duration_s,
                "max_force_rate_n_s": slip_config.max_force_rate_n_s,
                "contact_on_n": slip_config.contact_on_n,
                "contact_off_n": slip_config.contact_off_n,
                "contact_loss_confirm_s": slip_config.contact_loss_confirm_s,
                "sample_timeout_s": slip_config.sample_timeout_s,
                "min_estimates_per_side": slip_config.min_estimates_per_side,
            }
            if slip_config is not None
            else None
        ),
        "own_friction": (
            asdict(slip_config.own_friction)
            if slip_config is not None and slip_config.own_friction_enabled
            else None
        ),
        "native_slip": (
            {
                "max_duration_s": slip_config.max_duration_s,
                "confirmation_timeout_s": slip_config.confirmation_timeout_s,
                "stop_when_estimates_ready": slip_config.stop_when_estimates_ready,
            }
            if slip_config is not None and slip_config.native_slip_enabled
            else None
        ),
        "max_seconds": args.max_seconds,
    }
    recorder = JsonlRunRecorder(directory, config_record)
    worker = TactileWorker(serial_config, clear_bias=args.bias, sample_sink=recorder.sample)
    policy = StandaloneSlipSession(slip_config) if slip_config is not None else None
    failure: BaseException | None = None
    started_s = time.monotonic()
    previous_received_at_s: float | None = None
    first_sample = True
    manual_stop_announced = False
    manual_stop_sent = False
    print(f"记录目录：{directory}", flush=True)
    if args.bias:
        print("将对首个完整包执行 bias；此时传感器必须完全无负载。", flush=True)
    worker.start()
    try:
        while args.max_seconds is None or time.monotonic() - started_s < args.max_seconds:
            sample = worker.wait_for_update(previous_received_at_s, args.packet_timeout + 0.1)
            previous_received_at_s = sample.received_at_s
            if first_sample:
                print(
                    "bias 命令已发送，并已收到其后的首个有效包；现在可以用手柄闭合夹爪。"
                    if args.bias
                    else "已收到首个有效包，开始记录。",
                    flush=True,
                )
                first_sample = False
            if policy is not None:
                for event in policy.update(sample, now_s=time.monotonic(), worker=worker):
                    recorder.event(event)
                    if event["event"] == "native_slip_state":
                        print(
                            f"原厂滑移会话：{event['phase']}（{event['reason']}）",
                            flush=True,
                        )
                    elif event["event"] == "native_slip_estimate":
                        print(
                            f"原厂估计：{event['side']} pillar={event['pillar_id']} "
                            f"mu={event['native_mu']:.4f}",
                            flush=True,
                        )
                    elif event["event"] == "own_friction_estimate":
                        print(
                            f"自主估计：{event['side']} pillar={event['pillar_id']} "
                            f"mu_raw={event['raw_mu']:.4f} "
                            f"mu_control={event['conservative_mu']:.4f}",
                            flush=True,
                        )
                    elif event["event"] == "own_friction_state":
                        if event["phase"] == "active":
                            print(
                                "自主逐 pillar 摩擦估计已启动；请沿触觉面缓慢施加切向扰动。",
                                flush=True,
                            )
                        else:
                            print(
                                f"自主逐 pillar 摩擦估计已停止（{event['reason']}）。",
                                flush=True,
                            )
                    elif event["event"] == "friction_cross_validation":
                        print(
                            f"交叉验证：{event['side']} pillar={event['pillar_id']} "
                            f"own={event['own_raw_mu']:.4f} "
                            f"native={event['native_mu']:.4f} "
                            f"abs_error={event['absolute_error']:.4f}",
                            flush=True,
                        )
                    if (
                        event["event"] == "native_slip_state"
                        and event["phase"] == "stopped"
                        and event["reason"] == "detector_inactive"
                    ):
                        print(
                            "原厂检测器已自行退出；自主逐 pillar 估计继续随触觉记录运行。",
                            flush=True,
                        )
                phase = worker.native_slip_status.phase
                if slip_requested and phase == "active" and not manual_stop_announced:
                    print(
                        "原厂滑移检测已激活；施加扰动后，按 Enter 只停止滑移检测并继续记录。",
                        flush=True,
                    )
                    manual_stop_announced = True
                if (
                    slip_requested
                    and phase == "active"
                    and not manual_stop_sent
                    and _manual_stop_requested(sys.stdin)
                ):
                    worker.stop_native_slip("operator_requested")
                    manual_stop_sent = True
                    print("已请求停止原厂滑移检测；触觉记录继续运行。", flush=True)
    except KeyboardInterrupt:
        print(
            "收到 Ctrl-C，正在停止估计、关闭原厂服务并结束记录。"
            if slip_requested
            else "收到 Ctrl-C，正在停止自主估计并结束记录。",
            flush=True,
        )
    except BaseException as error:  # noqa: BLE001
        failure = error
    finally:
        try:
            worker.stop()
        except BaseException as error:  # noqa: BLE001
            if failure is None:
                failure = error
        recorder.close(status="failed" if failure is not None else "completed", error=failure)
    if failure is not None:
        print(f"PapillArray 独立记录失败：{failure}；记录目录：{directory}", file=sys.stderr)
        return 1
    print(f"记录完成：{directory}", flush=True)
    return 0


def main() -> NoReturn:
    """执行独立记录器并返回进程状态。"""
    raise SystemExit(run())


if __name__ == "__main__":  # pragma: no cover
    main()

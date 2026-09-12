"""记录真机倒水实验的可恢复运行产物。"""

from __future__ import annotations

import csv
import json
import threading
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Self


TRACE_FIELDS = (
    "time_s",
    "state",
    "tactile_received_at_s",
    "tactile_timestamp_us",
    "packet_counter",
    "raw_left_fx_n",
    "raw_left_fy_n",
    "raw_left_fz_n",
    "raw_right_fx_n",
    "raw_right_fy_n",
    "raw_right_fz_n",
    "left_fz_n",
    "right_fz_n",
    "measured_force_n",
    "target_force_n",
    "measured_tangential_force_n",
    "trigger_active",
    "force_limited",
    "position_rad",
    "velocity_rad_s",
    "torque_nm",
    "q_des_rad",
    "dq_des_rad_s",
    "kp",
    "kd",
    "tau_ff_nm",
    "control_dt_s",
    "tactile_age_s",
    "command_latency_s",
)

SCHEMA_VERSION = 1
RECORDER_VERSION = "0.1.0"


class CupRecorder:
    """将一趟真机倒水实验写入一个独占目录。

    记录器刻意不对控制时钟与触觉时钟作同步推断。每个控制周期只写入调用方
    提供的实际值，缺失值保持为空，方便离线分析辨认数据缺口。

    Args:
        output_directory: 本趟实验专用且尚不存在的输出目录。
        config: 本趟实验使用的可 JSON 序列化配置。
    """

    def __init__(self, output_directory: Path, config: dict[str, Any]) -> None:
        """构造记录器并立即创建输出目录和基础文件。"""
        self.directory = Path(output_directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self._started_at = _utc_now()
        self._closed = False
        self._lock = threading.RLock()
        self._config_path = self.directory / "config.json"
        self._events_path = self.directory / "events.jsonl"
        self._trace_path = self.directory / "trace.csv"
        self._tactile_path = self.directory / "tactile.jsonl"
        self._manifest_path = self.directory / "manifest.json"

        with self._config_path.open("w", encoding="utf-8") as handle:
            json.dump(config, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
        self._events_handle = self._events_path.open("a", encoding="utf-8")
        self._tactile_handle = self._tactile_path.open("a", encoding="utf-8")
        self._trace_handle = self._trace_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._trace_handle, fieldnames=TRACE_FIELDS)
        self._writer.writeheader()
        self._trace_handle.flush()

    def __enter__(self) -> Self:
        """返回当前记录器。"""
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        _traceback: object,
    ) -> bool:
        """按上下文退出原因写入最终 manifest，且不吞没异常。"""
        if exception is None:
            self.close()
        else:
            self.close(status="failed", error=exception)
        return False

    def append(self, event: Mapping[str, Any]) -> None:
        """追加一个结构化事件并立即刷新到磁盘。

        Args:
            event: 可 JSON 序列化的事件字典。

        Raises:
            RuntimeError: 记录器已经关闭。
        """
        with self._lock:
            self._ensure_open()
            json.dump(dict(event), self._events_handle, ensure_ascii=False, sort_keys=True)
            self._events_handle.write("\n")
            self._events_handle.flush()

    def sample(self, record: Mapping[str, Any]) -> None:
        """追加一个原始触觉包，并立即刷新以保留高频采样。

        该方法可由触觉工作线程调用；它与控制 trace、事件和关闭操作共享锁，
        因而不会使 JSONL 行相互交错。

        Args:
            record: 单个设备触觉包的完整可 JSON 序列化记录。

        Raises:
            RuntimeError: 记录器已经关闭。
        """
        with self._lock:
            self._ensure_open()
            json.dump(dict(record), self._tactile_handle, ensure_ascii=False, sort_keys=True)
            self._tactile_handle.write("\n")
            self._tactile_handle.flush()

    def write(self, row: Mapping[str, Any]) -> None:
        """写入一个控制周期 trace，并立即刷新以保留异常前的数据。

        Args:
            row: 控制周期数据。未提供的固定字段写为空，额外字段会被拒绝。

        Raises:
            KeyError: 行中包含未定义的字段。
            RuntimeError: 记录器已经关闭。
        """
        with self._lock:
            self._ensure_open()
            unknown = set(row).difference(TRACE_FIELDS)
            if unknown:
                unknown_names = ", ".join(sorted(unknown))
                raise KeyError(f"trace 包含未定义字段：{unknown_names}")
            self._writer.writerow({field: row.get(field, "") for field in TRACE_FIELDS})
            self._trace_handle.flush()

    def close(self, status: str = "completed", error: BaseException | str | None = None) -> None:
        """关闭数据文件并写入最终 manifest。

        Args:
            status: 运行最终状态，正常结束时为 `completed`。
            error: 失败原因；异常会保留其类型和消息。
        """
        with self._lock:
            if self._closed:
                return
            self._events_handle.close()
            self._tactile_handle.close()
            self._trace_handle.close()
            manifest: dict[str, Any] = {
                "schema_version": SCHEMA_VERSION,
                "version": RECORDER_VERSION,
                "started_at": self._started_at,
                "ended_at": _utc_now(),
                "status": status,
                "files": [
                    path.name
                    for path in (
                        self._config_path,
                        self._events_path,
                        self._tactile_path,
                        self._trace_path,
                    )
                    if path.is_file()
                ],
            }
            if error is not None:
                manifest["error"] = _error_details(error)
            with self._manifest_path.open("w", encoding="utf-8") as handle:
                json.dump(manifest, handle, ensure_ascii=False, indent=2, sort_keys=True)
                handle.write("\n")
            self._closed = True

    def _ensure_open(self) -> None:
        """拒绝关闭后的写入。"""
        if self._closed:
            raise RuntimeError("CupRecorder 已关闭。")


def _utc_now() -> str:
    """返回带时区的 UTC 墙钟时间。"""
    return datetime.now(timezone.utc).isoformat()


def _error_details(error: BaseException | str) -> dict[str, str]:
    """把失败原因转为可序列化且可读的 manifest 字段。"""
    if isinstance(error, BaseException):
        return {"type": type(error).__name__, "message": str(error)}
    return {"type": "Error", "message": error}

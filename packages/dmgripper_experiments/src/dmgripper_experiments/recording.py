"""通用抓取实验的运行目录、trace、事件与 manifest。

记录器刻意不对控制时钟与触觉时钟作同步推断：每个控制周期只写入
调用方提供的实际值，缺失量保持为空。数据写入失败必须向上传播，
不得为了保护终端显示而吞掉。
"""

from __future__ import annotations

import csv
import json
import threading
import uuid
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Self

from .config import ExperimentConfig, experiment_config_record, sanitize_directory_component

TRACE_FIELDS = (
    "time_s",
    "phase",
    "task_time_s",
    "contact_segment",
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
    "control_force_n",
    "target_source",
    "target_force_n",
    "target_raw_force_n",
    "target_force_rate_n_s",
    "target_force_acceleration_n_s2",
    "target_trigger_active",
    "target_increase_count",
    "measured_tangential_force_n",
    "stiffness_n_per_m",
    "stiffness_valid",
    "stiffness_updated",
    "stiffness_reason",
    "force_deadband_active",
    "unloading_blocked",
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
    "adaptive_risk",
    "adaptive_event_id",
    "adaptive_increase_count",
    "adaptive_risk_budget_exhausted",
    "adaptive_left_mu",
    "adaptive_right_mu",
    "adaptive_left_candidate",
    "adaptive_right_candidate",
    "adaptive_left_quality",
    "adaptive_right_quality",
    "adaptive_left_update_reason",
    "adaptive_right_update_reason",
    "adaptive_observation_reason",
    "adaptive_left_valid_mask",
    "adaptive_right_valid_mask",
    "adaptive_load_target_n",
    "adaptive_schedule_gap_n",
    "adaptive_track_error_n",
    "adaptive_capacity_limited",
    "adaptive_execution_limited",
    "adaptive_failure_reason",
    "sensor_sequence_id",
    "sensor_device_time_s",
    "sensor_received_at_s",
    "sensor_age_s",
    "sensor_stale",
    "sensor_dropped_samples",
    "sensor_observed_events",
)

SCHEMA_NAME = "dmgripper-experiment/v1"
RECORDER_VERSION = "1.3.0"


def create_run_directory(root: Path | str, config: ExperimentConfig) -> Path:
    """创建 ``<root>/<task>/<object>/<UTC时间戳>-<run_id>`` 的独占目录。

    目录组件使用清理后的名称；原始名称保留在配置与 manifest 中。
    run_id 为随机短标识，同名任务连续运行不会覆盖。

    Args:
        root: 输出根目录（相对路径按调用 cwd 解析）。
        config: 冻结实验配置。

    Returns:
        新创建且不存在的运行目录。

    Raises:
        FileExistsError: 碰撞后重试仍命中已有目录。
    """
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    task = sanitize_directory_component(config.metadata.task_name)
    obj = sanitize_directory_component(config.metadata.object_name)
    for _ in range(8):
        run_id = uuid.uuid4().hex[:8]
        directory = Path(root) / task / obj / f"{timestamp}-{run_id}"
        try:
            directory.mkdir(parents=True, exist_ok=False)
            return directory
        except FileExistsError:
            continue
    raise FileExistsError(f"无法创建独占运行目录：{directory}")


class ExperimentRecorder:
    """把一趟通用抓取实验写入一个独占目录。

    Args:
        directory: 本趟实验专用且尚不存在的输出目录。
        config: 本趟实验使用的冻结配置。
        input_config_path: 原始 YAML 路径；Python 构造可为 ``None``。
        code_version: 记录代码版本标识。
    """

    def __init__(
        self,
        directory: Path,
        config: ExperimentConfig,
        *,
        input_config_path: Path | None = None,
        code_version: str = RECORDER_VERSION,
    ) -> None:
        """构造记录器并立即创建基础文件。"""
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        if any(self.directory.iterdir()):
            raise FileExistsError(f"运行目录不为空，拒绝覆盖：{self.directory}")
        self._started_at = _utc_now()
        self._closed = False
        self._lock = threading.RLock()
        self._extra_artifacts: dict[str, str] = {}
        self._config_path = self.directory / "config.json"
        self._events_path = self.directory / "events.jsonl"
        self._trace_path = self.directory / "trace.csv"
        self._tactile_path = self.directory / "tactile.jsonl"
        self._manifest_path = self.directory / "manifest.json"
        record: dict[str, Any] = {
            "schema": SCHEMA_NAME,
            "input_config_path": (
                str(input_config_path) if input_config_path is not None else None
            ),
            "metadata": experiment_config_record(config)["metadata"],
            "effective": experiment_config_record(config),
        }
        with self._config_path.open("w", encoding="utf-8") as handle:
            json.dump(record, handle, ensure_ascii=False, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        self._events_handle = self._events_path.open("a", encoding="utf-8")
        self._tactile_handle = self._tactile_path.open("a", encoding="utf-8")
        self._trace_handle = self._trace_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._trace_handle, fieldnames=TRACE_FIELDS)
        self._writer.writeheader()
        self._trace_handle.flush()
        self._code_version = code_version

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
        """
        with self._lock:
            self._ensure_open()
            payload = json.dumps(
                dict(record),
                ensure_ascii=False,
                sort_keys=True,
                allow_nan=False,
            )
            self._tactile_handle.write(payload + "\n")
            self._tactile_handle.flush()

    def write(self, row: Mapping[str, Any]) -> None:
        """写入一个控制周期 trace，并立即刷新以保留异常前的数据。

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

    def register_artifacts(self, names: Mapping[str, str]) -> None:
        """登记后处理产物（如绘图文件）的名称与路径。"""
        with self._lock:
            if self._closed:
                raise RuntimeError("ExperimentRecorder 已关闭。")
            self._extra_artifacts.update(dict(names))

    def close(
        self,
        status: str = "completed",
        error: BaseException | str | None = None,
        *,
        disable_confirmed: bool | str = "not_applicable",
        cleanup_errors: list[str] | None = None,
        post_processing_error: BaseException | str | None = None,
        input_config_path: Path | None = None,
        fault_phase: str | None = None,
        fault_holding_entered: bool = False,
        fault_holding_duration_s: float = 0.0,
        fault_resolution: str | None = None,
    ) -> None:
        """关闭数据文件并写入最终 manifest。

        Args:
            status: 运行最终状态（completed／failed／cancelled）。
            error: 原始失败原因；异常保留类型与消息。
            disable_confirmed: 失能确认结果；使能前取消记 ``not_applicable``。
            cleanup_errors: 退出清理阶段的次要故障列表。
            post_processing_error: 设备正常结束后的绘图等后处理失败。
            input_config_path: 原始输入 YAML 路径。
            fault_phase: 首个可保持故障发生时的运行阶段。
            fault_holding_entered: 是否实际进入过故障保持。
            fault_holding_duration_s: 故障保持持续时间。
            fault_resolution: ``released``／``forced_disable``／``hold_lost`` 等处置结果。
        """
        with self._lock:
            if self._closed:
                return
            self._events_handle.close()
            self._tactile_handle.close()
            self._trace_handle.close()
            manifest: dict[str, Any] = {
                "schema": SCHEMA_NAME,
                "version": self._code_version,
                "created_at": self._started_at,
                "ended_at": _utc_now(),
                "status": status,
                "disable_confirmed": disable_confirmed,
                "fault_phase": fault_phase,
                "fault_holding_entered": fault_holding_entered,
                "fault_holding_duration_s": fault_holding_duration_s,
                "fault_resolution": fault_resolution,
                "cleanup_errors": list(cleanup_errors or []),
                "input_config_path": (
                    str(input_config_path) if input_config_path is not None else None
                ),
                "files": {
                    path.name: path.name
                    for path in (
                        self._config_path,
                        self._events_path,
                        self._tactile_path,
                        self._trace_path,
                    )
                    if path.is_file()
                },
            }
            manifest["files"].update(self._extra_artifacts)
            if error is not None:
                details = _error_details(error)
                manifest["error"] = details
                manifest["primary_error"] = details
            if post_processing_error is not None:
                manifest["post_processing_error"] = _error_details(post_processing_error)
            with self._manifest_path.open("w", encoding="utf-8") as handle:
                json.dump(manifest, handle, ensure_ascii=False, indent=2, sort_keys=True)
                handle.write("\n")
            self._closed = True

    def _ensure_open(self) -> None:
        """拒绝关闭后的写入。"""
        if self._closed:
            raise RuntimeError("ExperimentRecorder 已关闭。")


def _utc_now() -> str:
    """返回带时区的 UTC 墙钟时间。"""
    return datetime.now(timezone.utc).isoformat()


def _error_details(error: BaseException | str) -> dict[str, str]:
    """把失败原因转为可序列化且可读的 manifest 字段。"""
    if isinstance(error, BaseException):
        return {"type": type(error).__name__, "message": str(error)}
    return {"type": "Error", "message": error}

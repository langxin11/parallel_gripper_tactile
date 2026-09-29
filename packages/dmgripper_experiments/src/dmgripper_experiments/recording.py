"""通用抓取实验的运行目录、压缩 MCAP 时序流与 manifest。

三个 topic 共用有界队列及写线程，保留原数值与各自采样时间。
正常关闭会排空队列并完成索引；写线程失败向运行时传播。
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from collections.abc import Callable, Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Self

from papillarray_hardware.recording import QUEUE_CAPACITY, RECORDING_NAME, RecordingWriter

from .config import (
    ExperimentConfig,
    experiment_config_delta,
    experiment_config_record,
    sanitize_directory_component,
)

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
    "contact_closure_m",
    "contact_compression_m",
    "contact_compression_limit_m",
    "adaptive_left_mu",
    "adaptive_right_mu",
    "adaptive_load_target_n",
    "adaptive_schedule_gap_n",
    "adaptive_capacity_limited",
    "adaptive_execution_limited",
    "admittance_displacement_m",
    "admittance_velocity_m_s",
    "admittance_acceleration_m_s2",
    "admittance_velocity_limited",
    "admittance_acceleration_limited",
)


def _trace_schema(properties: Mapping[str, object]) -> bytes:
    """将全部 trace 字段写入 Foxglove 可读取的 JSON Schema。"""
    return json.dumps(
        {
            "$schema": "https://json-schema.org/draft/2020-12/schema",
            "type": "object",
            "properties": dict(properties),
            "additionalProperties": False,
        },
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")


_NUMBER = {"type": ["number", "null"]}
_INTEGER = {"type": ["integer", "null"]}
_STRING = {"type": ["string", "null"]}
_BOOLEAN = {"type": ["boolean", "null"]}


def _trace_fields_schema(fields: tuple[str, ...]) -> bytes:
    """返回全部控制 trace 字段的固定 JSON Schema。"""
    strings = {"phase", "target_source", "stiffness_reason"}
    booleans = {
        "stiffness_valid",
        "stiffness_updated",
        "force_deadband_active",
        "unloading_blocked",
        "adaptive_capacity_limited",
        "adaptive_execution_limited",
        "admittance_velocity_limited",
        "admittance_acceleration_limited",
    }
    integers = {"contact_segment", "tactile_timestamp_us", "packet_counter"}
    return _trace_schema(
        {
            field: (
                _STRING
                if field in strings
                else _BOOLEAN
                if field in booleans
                else _INTEGER
                if field in integers
                else _NUMBER
            )
            for field in fields
        }
    )


SCHEMA_NAME = "dmgripper-experiment/v2"
RECORDER_VERSION = "2.0.0"

TACTILE_QUEUE_CAPACITY = QUEUE_CAPACITY


def strided_sample_sink(
    sink: Callable[[Mapping[str, object]], None], stride: int
) -> Callable[[Mapping[str, object]], None]:
    """返回按设备包计数抽样的触觉 sink，供 ``TactileWorker`` 使用。

    正常包每隔 ``stride`` 个记录一个（按 ``packet_counter`` 取模，
    对齐设备计数而非墙钟）；首个包、出现间隙或回绕的异常包始终
    保留，硬件丢包仍可由 ``packet_counter`` 差值审计。

    Args:
        sink: 底层样本写入函数（如 ``ExperimentRecorder.sample``）。
        stride: 抽样间隔；1 表示全量记录。

    Returns:
        包装后的 sink；``stride == 1`` 时直接返回原 ``sink``。

    Raises:
        ValueError: ``stride`` 不是不小于 1 的整数。
    """
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1:
        raise ValueError("tactile_stride 必须是不小于 1 的整数")
    if stride == 1:
        return sink

    def sink_with_stride(record: Mapping[str, object]) -> None:
        counter = record.get("packet_counter")
        if (
            record.get("counter_event") == "consecutive"
            and not record.get("counter_gap")
            and isinstance(counter, int)
            and counter % stride != 0
        ):
            return
        sink(record)

    return sink_with_stride


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
        queue_capacity: int = TACTILE_QUEUE_CAPACITY,
        started_monotonic_s: float | None = None,
        epoch_ns: int | None = None,
    ) -> None:
        """构造记录器并立即创建基础文件。

        Args:
            directory: 本趟实验专用且尚不存在的输出目录。
            config: 本趟实验使用的冻结配置。
            input_config_path: 原始 YAML 路径；Python 构造可为 ``None``。
            code_version: 记录代码版本标识。
            queue_capacity: 三个 topic 共用的异步写队列容量。
            started_monotonic_s: 与控制时钟共用的实验起始单调时间。
            epoch_ns: 实验起点对应的 UTC 纳秒时间。
        """
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        if any(self.directory.iterdir()):
            raise FileExistsError(f"运行目录不为空，拒绝覆盖：{self.directory}")
        self._started_at = _utc_now()
        self._closed = False
        self._lock = threading.RLock()
        self._extra_artifacts: dict[str, str] = {}
        self._config_path = self.directory / "config.json"
        self._recording_path = self.directory / RECORDING_NAME
        self._manifest_path = self.directory / "manifest.json"
        self._started_monotonic_s = (
            time.monotonic() if started_monotonic_s is None else started_monotonic_s
        )
        self._recording_finished = False
        self._recording_max_bytes = (
            None
            if config.recording.max_recording_mib is None
            else config.recording.max_recording_mib * 1024 * 1024
        )
        self._recording_stop_reason: str | None = None
        record: dict[str, Any] = {
            "schema": SCHEMA_NAME,
            "input_config_path": (
                str(input_config_path) if input_config_path is not None else None
            ),
            "metadata": experiment_config_record(config)["metadata"],
            "effective": experiment_config_record(config),
            # 与默认配置的净差异；review 短清单比通读全量更易发现漂移。
            "delta": experiment_config_delta(config),
        }
        with self._config_path.open("w", encoding="utf-8") as handle:
            json.dump(record, handle, ensure_ascii=False, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        self._writer = RecordingWriter(
            self._recording_path,
            started_monotonic_s=self._started_monotonic_s,
            epoch_ns=epoch_ns,
            trace_schema=_trace_fields_schema(TRACE_FIELDS),
            queue_capacity=queue_capacity,
        )
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
        """把事件按生产端单调时间提交给异步记录器。"""
        with self._lock:
            self._ensure_recording_open()
            relative = event.get("time_s")
            monotonic_s = (
                self._started_monotonic_s + float(relative)
                if relative is not None
                else float(event.get("received_at_s", time.monotonic()))
            )
            self._writer.submit("/events", event, monotonic_s=monotonic_s)
            self._update_recording_limit()

    def sample(self, record: Mapping[str, Any]) -> None:
        """将完整触觉快照入队，保持采集线程不做序列化与磁盘 I/O。"""
        self._ensure_recording_open()
        self._writer._raise_error()
        if self._recording_stop_reason is not None:
            return
        self._writer.submit("/tactile", record, monotonic_s=float(record["received_at_s"]))
        self._update_recording_limit()

    @property
    def limit_stop_reason(self) -> str | None:
        """返回压缩 MCAP 文件达到体积上限时的正常收尾原因。"""
        self._update_recording_limit()
        return self._recording_stop_reason

    def _update_recording_limit(self) -> None:
        if (
            self._recording_stop_reason is None
            and self._recording_max_bytes is not None
            and self._writer.written_bytes >= self._recording_max_bytes
        ):
            self._recording_stop_reason = (
                f"max_recording_mib={self._recording_max_bytes / 1048576:g}"
            )

    def write(self, row: Mapping[str, Any]) -> None:
        """按实验起始单调时间写入一个控制周期的原类型 trace。"""
        with self._lock:
            self._ensure_recording_open()
            unknown = set(row).difference(TRACE_FIELDS)
            if unknown:
                raise KeyError(f"trace 包含未定义字段：{', '.join(sorted(unknown))}")
            self._writer.submit(
                "/trace",
                row,
                monotonic_s=self._started_monotonic_s + float(row["time_s"]),
            )
            self._update_recording_limit()

    def finish_recording(self) -> None:
        """排空三个 topic 并完成 MCAP 索引，供后处理立即读取。"""
        with self._lock:
            if self._recording_finished:
                return
            self._writer.close()
            self._recording_finished = True
            self._update_recording_limit()

    def _ensure_recording_open(self) -> None:
        self._ensure_open()
        if self._recording_finished:
            raise RuntimeError("MCAP 数据流已完成。")

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
        stop_reason: str | None = None,
        capacity_limited_ever: bool = False,
    ) -> None:
        """关闭数据文件并写入最终 manifest。

        先等触觉写线程排空队列再关闭文件；写线程此前失败时，若本次
        收尾本为成功，则升级为失败并以其异常为原始原因，保证不把
        缺数据的运行静默记为完成。

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
            stop_reason: 正常收尾时提前停止的原因（如记录上限）。
            capacity_limited_ever: 自适应目标曾受容量上限约束。
        """
        with self._lock:
            if self._closed:
                return
            try:
                self.finish_recording()
            except RuntimeError as writer_error:
                if error is None:
                    status = "failed"
                    error = writer_error
                else:
                    cleanup_errors = [*(cleanup_errors or []), f"MCAP 写线程失败：{writer_error}"]
            manifest: dict[str, Any] = {
                "schema": SCHEMA_NAME,
                "version": self._code_version,
                "created_at": self._started_at,
                "ended_at": _utc_now(),
                "status": status,
                "scientific_evaluation": "not_evaluated",
                "capacity_limited_ever": capacity_limited_ever,
                "stop_reason": stop_reason
                if stop_reason is not None
                else self._recording_stop_reason,
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
                        self._recording_path,
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

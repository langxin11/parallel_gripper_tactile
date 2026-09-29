"""通用抓取实验的运行目录、trace、事件与 manifest。

记录器刻意不对控制时钟与触觉时钟作同步推断：每个控制周期只写入
调用方提供的实际值，缺失量保持为空。数据写入失败必须向上传播，
不得为了保护终端显示而吞掉。

触觉流由专用写线程异步落盘：``sample()`` 只做有限性校验并入队，
序列化（浮点截断到 float32 可表示精度）与写盘在后台完成，使磁盘
延迟不进入触觉观测链路。flush 只保证进程退出（含异常收尾）后已
提交数据可见；进程被杀死时丢失一个提交周期内的队列与缓冲，断电
与内核崩溃不在保证范围内。
"""

from __future__ import annotations

import csv
import json
import math
import queue
import threading
import time
import uuid
from collections.abc import Callable, Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Self

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
    "depth_prior_contact_closure_m",
    "contact_closure_m",
    "contact_compression_m",
    "contact_compression_limit_m",
    "adaptive_left_depth_prior_depth_m",
    "adaptive_left_depth_prior_candidate",
    "adaptive_left_depth_prior_value",
    "adaptive_left_depth_prior_locked",
    "adaptive_left_depth_prior_reason",
    "adaptive_right_depth_prior_depth_m",
    "adaptive_right_depth_prior_candidate",
    "adaptive_right_depth_prior_value",
    "adaptive_right_depth_prior_locked",
    "adaptive_right_depth_prior_reason",
    "adaptive_left_mu",
    "adaptive_right_mu",
    "adaptive_left_candidate",
    "adaptive_right_candidate",
    "adaptive_left_quality",
    "adaptive_right_quality",
    "adaptive_left_mu_lower_bound",
    "adaptive_right_mu_lower_bound",
    "adaptive_left_taxel_mu_lower_bound",
    "adaptive_right_taxel_mu_lower_bound",
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
    "adaptive_left_particle_mu_mean",
    "adaptive_left_particle_mu_control",
    "adaptive_left_particle_mu_lower",
    "adaptive_left_particle_mu_upper",
    "adaptive_left_particle_mu_ess",
    "adaptive_left_particle_mu_updated",
    "adaptive_left_particle_mu_reason",
    "adaptive_right_particle_mu_mean",
    "adaptive_right_particle_mu_control",
    "adaptive_right_particle_mu_lower",
    "adaptive_right_particle_mu_upper",
    "adaptive_right_particle_mu_ess",
    "adaptive_right_particle_mu_updated",
    "adaptive_right_particle_mu_reason",
    "sensor_sequence_id",
    "sensor_device_time_s",
    "sensor_received_at_s",
    "sensor_age_s",
    "sensor_stale",
    "sensor_dropped_samples",
    "sensor_observed_events",
    "native_session_id",
    "native_session_phase",
    "native_session_reason",
    "native_left_estimate_count",
    "native_right_estimate_count",
    "stiffness_preload_stage",
    "stiffness_preload_reason",
    "contact_force_goal_n",
    "preload_stiffness_used_n_per_m",
    "admittance_stiffness_used_n_per_m",
    "admittance_adaptation_reason",
    "admittance_mass_kg",
    "admittance_damping_ns_m",
    "admittance_stiffness_n_m",
    "admittance_displacement_m",
    "admittance_velocity_m_s",
    "admittance_acceleration_m_s2",
    "admittance_velocity_limited",
    "admittance_acceleration_limited",
)

SCHEMA_NAME = "dmgripper-experiment/v1"
RECORDER_VERSION = "1.12.0"

# 触觉异步写线程的队列容量（约 10 s 的 1000 Hz 积压）与提交节奏。
TACTILE_QUEUE_CAPACITY = 10_000
_TACTILE_BATCH_ROWS = 256
_TACTILE_FLUSH_INTERVAL_S = 0.25
_TACTILE_DRAIN_TIMEOUT_S = 10.0

# 这些字段是主机单调钟或展开时间，float32 的 24 位尾数保不住微秒
# 分辨率，必须保留 float64 全精度；其余浮点源自传感器 float32 读数。
_FLOAT64_FIELDS = frozenset({"received_at_s"})


def _ensure_json_finite(value: object, key: str = "") -> None:
    """递归拒绝 NaN 与无穷大，保证不产生非标准 JSON 数字。

    Args:
        value: 待写入的记录（dict、序列或标量的任意嵌套）。
        key: 当前字段名，仅用于错误信息。

    Raises:
        ValueError: 任一浮点不是有限数值。
    """
    if isinstance(value, float):
        if not math.isfinite(value):
            where = f"字段 {key}" if key else "记录"
            raise ValueError(f"触觉记录 {where} 包含非有限数值，不符合 JSON 规范")
    elif isinstance(value, Mapping):
        for name, item in value.items():
            _ensure_json_finite(item, str(name))
    elif isinstance(value, (list, tuple)):
        for item in value:
            _ensure_json_finite(item, key)


def _truncate_float32(value: object) -> object:
    """把浮点截断到 9 位有效数字（float32 可表示精度）。

    传感器上行数据在协议层即 float32；更高的十进制位数只复制滤波
    运算的舍入噪声。实现为先格式化为 9 位有效数字再解析回
    float64，使 ``json.dumps`` 输出短文本且解析值与原值之差不超过
    一个 float32 精度。

    Args:
        value: 完整触觉记录或其任意嵌套子结构。

    Returns:
        同结构的新对象；``_FLOAT64_FIELDS`` 中的顶层标量原样保留。
    """
    if isinstance(value, float):
        return float(f"{value:.9g}")
    if isinstance(value, Mapping):
        return {
            name: (item if name in _FLOAT64_FIELDS else _truncate_float32(item))
            for name, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_truncate_float32(item) for item in value]
    return value


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
        tactile_queue_capacity: int = TACTILE_QUEUE_CAPACITY,
        tactile_join_timeout_s: float = _TACTILE_DRAIN_TIMEOUT_S,
    ) -> None:
        """构造记录器并立即创建基础文件。

        Args:
            directory: 本趟实验专用且尚不存在的输出目录。
            config: 本趟实验使用的冻结配置。
            input_config_path: 原始 YAML 路径；Python 构造可为 ``None``。
            code_version: 记录代码版本标识。
            tactile_queue_capacity: 触觉异步写队列容量。
            tactile_join_timeout_s: 关闭时等待写线程排空的超时秒数。
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
        self._events_path = self.directory / "events.jsonl"
        self._trace_path = self.directory / "trace.csv"
        self._tactile_path = self.directory / "tactile.jsonl"
        self._manifest_path = self.directory / "manifest.json"
        self._tactile_queue: queue.Queue[Mapping[str, object]] = queue.Queue(
            maxsize=tactile_queue_capacity
        )
        self._tactile_state_lock = threading.Lock()
        self._tactile_stop_event = threading.Event()
        self._tactile_accepting = True
        self._tactile_stop_reason: str | None = None
        self._tactile_writer_error: BaseException | None = None
        self._tactile_written_bytes = 0
        self._tactile_max_bytes = (
            None
            if config.recording.max_tactile_mib is None
            else config.recording.max_tactile_mib * 1024 * 1024
        )
        self._tactile_join_timeout_s = tactile_join_timeout_s
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
        self._events_handle = self._events_path.open("a", encoding="utf-8")
        self._tactile_handle = self._tactile_path.open("a", encoding="utf-8")
        self._trace_handle = self._trace_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._trace_handle, fieldnames=TRACE_FIELDS)
        self._writer.writeheader()
        self._trace_handle.flush()
        self._code_version = code_version
        self._tactile_thread = threading.Thread(
            target=self._drain_tactile_queue, name="tactile-recorder", daemon=True
        )
        self._tactile_thread.start()

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
        """追加一个结构化事件并立即刷新；进程退出后即可读，不保证断电。

        Raises:
            RuntimeError: 记录器已经关闭。
        """
        with self._lock:
            self._ensure_open()
            json.dump(dict(event), self._events_handle, ensure_ascii=False, sort_keys=True)
            self._events_handle.write("\n")
            self._events_handle.flush()

    def sample(self, record: Mapping[str, Any]) -> None:
        """校验并把一个原始触觉包交给后台写线程异步落盘。

        有限性在调用线程同步校验以立即拒绝非有限数值；序列化与写盘
        延迟到写线程，磁盘延迟因此不进入触觉观测链路。写线程已失败
        或队列已满时立即向上抛错；体积上限触发后静默丢弃，由控制
        循环经 ``limit_stop_reason`` 正常收尾。

        Raises:
            ValueError: 记录包含 NaN 或无穷大。
            RuntimeError: 记录器已关闭、写队列已满或写线程先前已失败。
        """
        self._ensure_open()
        self._raise_tactile_writer_failure()
        if self._tactile_stop_reason is not None:
            return
        _ensure_json_finite(record)
        payload = dict(record)
        with self._tactile_state_lock:
            if not self._tactile_accepting:
                raise RuntimeError("ExperimentRecorder 已关闭。")
            try:
                self._tactile_queue.put_nowait(payload)
            except queue.Full as error:
                raise RuntimeError("触觉写队列已满，磁盘写入速度不足") from error

    @property
    def limit_stop_reason(self) -> str | None:
        """返回写线程设置的触觉体积上限原因；``None`` 表示未触发。"""
        return self._tactile_stop_reason

    def _raise_tactile_writer_failure(self) -> None:
        """把写线程锁存的失败以链式异常抛给调用方。"""
        error = self._tactile_writer_error
        if error is not None:
            raise RuntimeError("触觉写线程已失败") from error

    def _drain_tactile_queue(self) -> None:
        """后台序列化并批量提交触觉行；异常被锁存待调用方收取。

        每 256 行或 0.25 s 提交一次；达到体积上限后设置停止原因并
        丢弃后续样本，使生产端不被阻塞。停机由事件驱动：排空队列后
        才退出，避免关闭路径向已满队列投递哨兵造成死锁。
        """
        pending_rows = 0
        last_flush_s = time.monotonic()
        try:
            while True:
                try:
                    record = self._tactile_queue.get(timeout=0.05)
                except queue.Empty:
                    if self._tactile_stop_event.is_set():
                        break
                    continue
                if self._tactile_stop_reason is not None:
                    continue
                payload = json.dumps(
                    _truncate_float32(dict(record)),
                    ensure_ascii=False,
                    sort_keys=True,
                    allow_nan=False,
                    separators=(",", ":"),
                )
                self._tactile_handle.write(payload + "\n")
                self._tactile_written_bytes += len(payload) + 1
                pending_rows += 1
                if (
                    self._tactile_max_bytes is not None
                    and self._tactile_written_bytes >= self._tactile_max_bytes
                ):
                    self._tactile_handle.flush()
                    with self._tactile_state_lock:
                        self._tactile_stop_reason = (
                            f"max_tactile_mib={self._tactile_max_bytes / 1048576:g}"
                        )
                    continue
                now_s = time.monotonic()
                if pending_rows < _TACTILE_BATCH_ROWS and now_s - last_flush_s < (
                    _TACTILE_FLUSH_INTERVAL_S
                ):
                    continue
                self._tactile_handle.flush()
                pending_rows = 0
                last_flush_s = now_s
        except BaseException as error:  # noqa: BLE001
            with self._tactile_state_lock:
                self._tactile_writer_error = error
        finally:
            try:
                self._tactile_handle.flush()
            except BaseException:  # noqa: BLE001
                pass

    def write(self, row: Mapping[str, Any]) -> None:
        """写入一个控制周期 trace 并立即刷新；进程退出后即可读，不保证断电。

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
        stop_reason: str | None = None,
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
        """
        with self._lock:
            if self._closed:
                return
            with self._tactile_state_lock:
                self._tactile_accepting = False
            self._tactile_stop_event.set()
            self._tactile_thread.join(timeout=self._tactile_join_timeout_s)
            writer_error = self._tactile_writer_error
            if self._tactile_thread.is_alive():
                cleanup_errors = [
                    *(cleanup_errors or []),
                    "触觉写线程未在超时内排空，末批数据可能缺失",
                ]
            if writer_error is not None:
                if error is None and status == "completed":
                    status = "failed"
                    error = writer_error
                else:
                    cleanup_errors = [
                        *(cleanup_errors or []),
                        f"触觉写线程失败：{writer_error}",
                    ]
            self._events_handle.close()
            self._tactile_handle.close()
            self._trace_handle.close()
            manifest: dict[str, Any] = {
                "schema": SCHEMA_NAME,
                "version": self._code_version,
                "created_at": self._started_at,
                "ended_at": _utc_now(),
                "status": status,
                "stop_reason": stop_reason
                if stop_reason is not None
                else self._tactile_stop_reason,
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

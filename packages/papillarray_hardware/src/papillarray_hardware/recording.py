"""真机时序记录的 MCAP 写入与读取。"""

from __future__ import annotations

import json
import queue
import threading
import time
from collections.abc import Iterator, Mapping
from pathlib import Path
from typing import Any, Self

from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

RECORDING_NAME = "recording.mcap"
QUEUE_CAPACITY = 10_000

_NUMBER = {"type": ["number", "null"]}
_INTEGER = {"type": ["integer", "null"]}
_STRING = {"type": ["string", "null"]}
_BOOLEAN = {"type": ["boolean", "null"]}
_VECTOR = {"type": "array", "items": _NUMBER}
_MATRIX = {"type": "array", "items": _VECTOR}
_TACTILE_FIELDS = (
    "received_at_s",
    "packet_counter",
    "timestamp_us",
    "left_force_n",
    "right_force_n",
    "raw_left_fz_n",
    "raw_right_fz_n",
    "raw_left_fx_n",
    "raw_left_fy_n",
    "raw_right_fx_n",
    "raw_right_fy_n",
    "left_taxel_forces_n",
    "right_taxel_forces_n",
    "counter_event",
    "counter_gap",
    "raw_timestamp_us",
    "timestamp_wrap_count",
    "native_session_id",
    "native_session_phase",
    "native_session_reason",
    "native_slip_active",
    "native_reference_loaded",
    "native_pillar_states",
    "native_pillar_friction",
    "native_sensor_friction",
    "native_target_grip_force_n",
    "left_taxel_displacements_mm",
    "right_taxel_displacements_mm",
)
_TACTILE_INTEGERS = {
    "packet_counter",
    "timestamp_us",
    "counter_gap",
    "raw_timestamp_us",
    "timestamp_wrap_count",
    "native_session_id",
}
_TACTILE_STRINGS = {"counter_event", "native_session_phase", "native_session_reason"}
_TACTILE_BOOL_ARRAYS = {"native_slip_active", "native_reference_loaded"}
_TACTILE_MATRICES = {
    "left_taxel_forces_n",
    "right_taxel_forces_n",
    "native_pillar_states",
    "native_pillar_friction",
    "left_taxel_displacements_mm",
    "right_taxel_displacements_mm",
}


def _schema(properties: Mapping[str, object]) -> bytes:
    """编码完整字段定义，供 Foxglove 直接发现数值路径。"""
    return json.dumps(
        {
            "$schema": "https://json-schema.org/draft/2020-12/schema",
            "type": "object",
            "properties": dict(properties),
            "additionalProperties": True,
        },
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")


def tactile_schema() -> bytes:
    """返回采集快照的固定 JSON Schema。"""
    properties: dict[str, object] = {}
    for field in _TACTILE_FIELDS:
        if field in _TACTILE_INTEGERS:
            properties[field] = _INTEGER
        elif field in _TACTILE_STRINGS:
            properties[field] = _STRING
        elif field in _TACTILE_BOOL_ARRAYS:
            properties[field] = {"type": "array", "items": {"type": "boolean"}}
        elif field in _TACTILE_MATRICES:
            properties[field] = _MATRIX
        elif field.startswith("native_") and field != "native_session_id":
            properties[field] = _VECTOR
        else:
            properties[field] = _NUMBER
    return _schema(properties)


_EVENT_SCHEMA = _schema(
    {
        "time_s": _NUMBER,
        "received_at_s": _NUMBER,
        "event": _STRING,
        "phase": _STRING,
        "message": _STRING,
        "reason": _STRING,
        "timestamp_us": _INTEGER,
        "packet_counter": _INTEGER,
        "side": _STRING,
        "pillar_id": _INTEGER,
        "session_id": _INTEGER,
        "raw_mu": _NUMBER,
        "conservative_mu": _NUMBER,
        "native_mu": _NUMBER,
        "normal_force_n": _NUMBER,
        "shear_force_n": _NUMBER,
        "own_raw_mu": _NUMBER,
        "own_conservative_mu": _NUMBER,
        "absolute_error": _NUMBER,
        "relative_error_to_native": _NUMBER,
        "device_timestamp_us": _INTEGER,
        "host_monotonic_s": _NUMBER,
    }
)


class RecordingWriter:
    """用一个有界队列和单个写线程写入压缩的多 topic MCAP。"""

    def __init__(
        self,
        path: Path,
        *,
        started_monotonic_s: float,
        epoch_ns: int | None = None,
        trace_schema: bytes | None = None,
        queue_capacity: int = QUEUE_CAPACITY,
    ) -> None:
        """配置队列、固定 schema 和共用时间原点。"""
        self.path = Path(path)
        self.started_monotonic_s = started_monotonic_s
        self.epoch_ns = time.time_ns() if epoch_ns is None else epoch_ns
        self._queue: queue.Queue[tuple[str, dict[str, object], int]] = queue.Queue(
            maxsize=queue_capacity
        )
        self._stop = threading.Event()
        self._closed = False
        self._error: BaseException | None = None
        self.written_bytes = 0
        self._schemas = {
            "/tactile": tactile_schema(),
            "/events": _EVENT_SCHEMA,
        }
        if trace_schema is not None:
            self._schemas["/trace"] = trace_schema
        self._thread = threading.Thread(target=self._run, name="mcap-recorder", daemon=True)
        self._thread.start()

    def __enter__(self) -> Self:
        """返回正在写入的记录器。"""
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        """离开上下文时排空并完成索引。"""
        self.close()

    def submit(self, topic: str, record: Mapping[str, object], *, monotonic_s: float) -> None:
        """入队原始值；时间在生产端映射，序列化与压缩在写线程执行。"""
        if topic not in self._schemas:
            raise ValueError(f"未知 MCAP topic：{topic}")
        if self._closed:
            raise RuntimeError("MCAP 记录器已关闭")
        self._raise_error()
        log_time = self.epoch_ns + round((monotonic_s - self.started_monotonic_s) * 1e9)
        try:
            self._queue.put_nowait((topic, dict(record), log_time))
        except queue.Full as error:
            raise RuntimeError("MCAP 写队列已满，磁盘写入速度不足") from error

    def close(self) -> None:
        """等待全部已入队消息和索引写完。"""
        if not self._closed:
            self._closed = True
            self._stop.set()
            self._thread.join()
        self._raise_error()

    def _raise_error(self) -> None:
        if self._error is not None:
            raise RuntimeError(
                f"MCAP 写线程已失败：{type(self._error).__name__}: {self._error}"
            ) from self._error

    def _run(self) -> None:
        try:
            with self.path.open("xb") as stream:
                writer = Writer(stream, chunk_size=256 * 1024, compression=CompressionType.ZSTD)
                writer.start()
                channels = {}
                for topic, schema in self._schemas.items():
                    schema_id = writer.register_schema(
                        name=f"parallel-gripper-tactile{topic}/v1",
                        encoding="jsonschema",
                        data=schema,
                    )
                    channels[topic] = writer.register_channel(
                        topic=topic, message_encoding="json", schema_id=schema_id
                    )
                try:
                    while True:
                        try:
                            topic, record, log_time = self._queue.get(timeout=0.05)
                        except queue.Empty:
                            if self._stop.is_set():
                                break
                            continue
                        data = json.dumps(
                            record, ensure_ascii=False, allow_nan=False, separators=(",", ":")
                        ).encode("utf-8")
                        writer.add_message(
                            channel_id=channels[topic],
                            log_time=log_time,
                            publish_time=log_time,
                            data=data,
                        )
                        self.written_bytes = stream.tell()
                except BaseException as write_error:  # noqa: BLE001
                    try:
                        writer.finish()
                    except BaseException as finish_error:  # noqa: BLE001
                        write_error.add_note(f"MCAP 索引最终化失败：{finish_error}")
                    raise
                writer.finish()
                self.written_bytes = stream.tell()
        except BaseException as error:  # noqa: BLE001
            self._error = error


def iter_records(path: Path, topic: str) -> Iterator[dict[str, Any]]:
    """按 MCAP 日志时间读取一个 topic 的原类型 JSON 记录。"""
    with Path(path).open("rb") as stream:
        reader = make_reader(stream)
        for _schema_record, _channel, message in reader.iter_messages(topics=[topic]):
            yield json.loads(message.data)

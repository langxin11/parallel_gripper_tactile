"""Contactile PapillArray PTS v2.0 帧解析与流重同步。"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

START_MARKER = bytes((0x55, 0x66, 0x77, 0x88))
"""PTS v2.0 帧起始标志。"""

END_MARKER = bytes((0xAA, 0xBB, 0xCC, 0xDD))
"""PTS v2.0 帧结束标志。"""

_TYPE_HUB = 1
_TYPE_PILLAR = 3
_TYPE_GLOBAL = 4
_TYPE_PILLAR_SLIP = 5
_TYPE_SENSOR_SLIP = 6
_TYPE_RESERVED = 7
_MIN_FRAME_DATA_BYTES = 3


class ProtocolError(ValueError):
    """表示 PTS 帧结构或字段不符合 v2.0 协议。"""


class PacketChecksumError(ProtocolError):
    """表示 PTS 帧校验和不匹配。"""


@dataclass(frozen=True)
class PtsReadDiagnostics:
    """一次 PTS 包等待期间收集的不可变协议诊断快照。

    `raw_hex_preview` 仅保留本次等待最先收到的有限字节，方便识别设备输出格式，且不会因持续
    噪声增长而无限占用内存。

    Args:
        received_bytes: 本次等待从串口收到的总字节数。
        start_markers: 识别到的起始标志次数。
        end_markers: 识别到的结束标志次数。
        candidate_frames: 已取得完整边界的候选帧数。
        checksum_failures: 候选帧校验和失败次数。
        protocol_failures: 候选帧通过校验和但结构解析失败次数。
        oversize_discards: 因超过单帧上限丢弃的候选或半包次数。
        last_protocol_error: 最后一次候选帧解析错误；无错误时为 `None`。
        raw_hex_preview: 最多 `preview_bytes` 个原始接收字节的十六进制预览。
    """

    received_bytes: int
    start_markers: int
    end_markers: int
    candidate_frames: int
    checksum_failures: int
    protocol_failures: int
    oversize_discards: int
    last_protocol_error: str | None
    raw_hex_preview: str


class PtsReadTimeout(TimeoutError):
    """表示一次 PTS 包等待在无有效包时结束，并携带协议诊断。"""

    def __init__(self, message: str, diagnostics: PtsReadDiagnostics) -> None:
        """保存可供 CLI 和调用方展示的不可变诊断快照。"""
        super().__init__(message)
        self.diagnostics = diagnostics


class _ReadablePort(Protocol):
    """协议读取器所需的最小串口读取接口。"""

    def read(self, size: int) -> bytes:
        """读取至多指定数量的字节。"""


@dataclass
class _DiagnosticsAccumulator:
    """只在单次 `read_packet()` 内使用的可变诊断状态。"""

    preview_bytes: int
    _start_tail: bytes = b""
    _end_tail: bytes = b""
    received_bytes: int = 0
    start_markers: int = 0
    end_markers: int = 0
    candidate_frames: int = 0
    checksum_failures: int = 0
    protocol_failures: int = 0
    oversize_discards: int = 0
    last_protocol_error: str | None = None
    _preview: bytearray = field(default_factory=bytearray)

    def record_chunk(self, chunk: bytes) -> None:
        """累计字节、有限预览和跨分片标志计数。"""
        self.received_bytes += len(chunk)
        remaining = self.preview_bytes - len(self._preview)
        if remaining > 0:
            self._preview.extend(chunk[:remaining])
        self.start_markers, self._start_tail = _count_stream_markers(
            self.start_markers, self._start_tail, chunk, START_MARKER
        )
        self.end_markers, self._end_tail = _count_stream_markers(
            self.end_markers, self._end_tail, chunk, END_MARKER
        )

    @property
    def marker_tails(self) -> tuple[bytes, bytes]:
        """返回供后续等待继承的起止标志跨分片尾部。"""
        return self._start_tail, self._end_tail

    def snapshot(self) -> PtsReadDiagnostics:
        """冻结当前状态，避免异常抛出后诊断被后续读取改变。"""
        return PtsReadDiagnostics(
            received_bytes=self.received_bytes,
            start_markers=self.start_markers,
            end_markers=self.end_markers,
            candidate_frames=self.candidate_frames,
            checksum_failures=self.checksum_failures,
            protocol_failures=self.protocol_failures,
            oversize_discards=self.oversize_discards,
            last_protocol_error=self.last_protocol_error,
            raw_hex_preview=bytes(self._preview).hex(),
        )


def _count_stream_markers(
    count: int, tail: bytes, chunk: bytes, marker: bytes
) -> tuple[int, bytes]:
    """累计跨分片标志，并保留至多一个标志长度减一的尾部。"""
    combined = tail + chunk
    tail_length = len(tail)
    start = 0
    while True:
        marker_index = combined.find(marker, start)
        if marker_index < 0:
            break
        if marker_index + len(marker) > tail_length:
            count += 1
        start = marker_index + 1
    return count, combined[-(len(marker) - 1) :]


@dataclass(frozen=True)
class PtsPacket:
    """一个校验通过的 PTS v2.0 观测包。

    Args:
        packet_counter: 控制器递增的包计数器。
        timestamp_us: 控制器时间戳，单位为微秒。
        pillar_forces: 每个传感器的 pillar 力，形状均为 `(Np, 3)`，单位 N。
        pillar_displacements: 每个传感器的 pillar 位移，形状均为 `(Np, 3)`，单位 mm。
        global_forces: 每个传感器的全局力，形状均为 `(3,)`，单位 N。
        global_torques: 每个传感器的全局力矩，形状均为 `(3,)`，单位 N·mm。
        pillar_slip_states: 每个传感器的 pillar 滑动状态。
        pillar_friction_estimates: 每个传感器的 pillar 摩擦估计。
        slip_detection_active: 每个传感器的滑动检测激活状态。
        reference_pillar_loaded: 每个传感器的参考 pillar 载荷状态。
        sensor_friction_estimates: 每个传感器的摩擦估计。
        target_grip_forces: 每个传感器的防滑目标抓握力，单位 N。
        type_7_data: Type 7 块的原始字节；布局未定义时保持原样。
    """

    packet_counter: int
    timestamp_us: int
    pillar_forces: list[np.ndarray]
    pillar_displacements: list[np.ndarray]
    global_forces: list[np.ndarray]
    global_torques: list[np.ndarray]
    pillar_slip_states: list[np.ndarray] = field(default_factory=list)
    pillar_friction_estimates: list[np.ndarray] = field(default_factory=list)
    slip_detection_active: list[bool] = field(default_factory=list)
    reference_pillar_loaded: list[bool] = field(default_factory=list)
    sensor_friction_estimates: list[float] = field(default_factory=list)
    target_grip_forces: list[float] = field(default_factory=list)
    type_7_data: bytes | None = None

    @property
    def n_sensors(self) -> int:
        """返回此包中报告 pillar 数据的传感器数。"""
        return len(self.pillar_forces)


def verify_checksum(data: bytes) -> bool:
    """验证帧内部数据末尾的 PTS 16 位小端校验和。

    Args:
        data: 去除起止标志后的帧数据，包含最后两个校验和字节。

    Returns:
        校验和是否匹配。
    """
    return len(data) >= 2 and (sum(data[:-2]) & 0xFFFF) == _read_u16(data, len(data) - 2)


def parse_packet(data: bytes) -> PtsPacket:
    """解析一个完整、校验通过的 PTS 帧内部数据。

    Args:
        data: 去除起止标志后的帧数据，包含最后两个校验和字节。

    Returns:
        保留设备时间与计数器的触觉观测。

    Raises:
        PacketChecksumError: 校验和不正确。
        ProtocolError: 索引、偏移或数据块结构不正确。
    """
    if len(data) < _MIN_FRAME_DATA_BYTES:
        raise ProtocolError("PTS 帧数据长度不足")
    if not verify_checksum(data):
        raise PacketChecksumError("PTS 帧校验和不匹配")

    index_size = data[0]
    offsets = _parse_top_level_index(data, index_size)
    hub = _block_bytes(data, offsets, _TYPE_HUB)
    if hub is None:
        raise ProtocolError("PTS 数据包缺少 Type 1 Hub 块")
    if len(hub) != 12:
        raise ProtocolError("Type 1 Hub 块长度必须为 12 字节")

    pillar = _block_bytes(data, offsets, _TYPE_PILLAR)
    global_data = _block_bytes(data, offsets, _TYPE_GLOBAL)
    if pillar is None or global_data is None:
        raise ProtocolError("PTS 数据包缺少 Type 3 pillar 或 Type 4 global 块")

    packet_counter = int.from_bytes(hub[:4], "little")
    timestamp_us = int.from_bytes(hub[4:], "little")
    pillar_forces, pillar_displacements = _parse_pillar_block(pillar, index_size)
    global_forces, global_torques = _parse_global_block(global_data, index_size)
    pillar_slip = _block_bytes(data, offsets, _TYPE_PILLAR_SLIP)
    sensor_slip = _block_bytes(data, offsets, _TYPE_SENSOR_SLIP)
    slip_states, pillar_friction = (
        _parse_pillar_slip_block(pillar_slip, index_size) if pillar_slip is not None else ([], [])
    )
    active, reference_loaded, sensor_friction, target_grip = (
        _parse_sensor_slip_block(sensor_slip, index_size)
        if sensor_slip is not None
        else ([], [], [], [])
    )
    _validate_sensor_counts(
        pillar_forces,
        global_forces,
        slip_states,
        pillar_friction,
        active,
        reference_loaded,
        sensor_friction,
        target_grip,
        has_pillar_slip=pillar_slip is not None,
        has_sensor_slip=sensor_slip is not None,
    )

    return PtsPacket(
        packet_counter=packet_counter,
        timestamp_us=timestamp_us,
        pillar_forces=pillar_forces,
        pillar_displacements=pillar_displacements,
        global_forces=global_forces,
        global_torques=global_torques,
        pillar_slip_states=slip_states,
        pillar_friction_estimates=pillar_friction,
        slip_detection_active=active,
        reference_pillar_loaded=reference_loaded,
        sensor_friction_estimates=sensor_friction,
        target_grip_forces=target_grip,
        type_7_data=_block_bytes(data, offsets, _TYPE_RESERVED),
    )


class PtsStreamReader:
    """从字节流寻找、校验并解析下一个 PTS v2.0 数据包。

    构造函数只保留可读端口，不执行读取。坏校验和或结构异常的候选帧会被丢弃，读取器继续寻找
    后续起始标志；这使上层无需因单包损坏而重建串口连接。

    Args:
        serial_port: 已打开且可读的串口对象。
        max_packet_bytes: 含起止标志的单帧最大长度。
        read_size: 每次向串口请求的最大字节数。
        packet_timeout_s: 单次 `read_packet()` 等待有效包的总单调时钟时限，单位 s。
        monotonic: 可注入的单调时钟，供无需休眠的离线测试使用。
        preview_bytes: 单次等待保留原始十六进制预览的最大字节数。
    """

    def __init__(
        self,
        serial_port: _ReadablePort,
        max_packet_bytes: int = 8192,
        read_size: int = 4096,
        packet_timeout_s: float = 3.0,
        monotonic: Callable[[], float] = time.monotonic,
        preview_bytes: int = 64,
    ) -> None:
        """保存读取设置而不发生 I/O。"""
        if isinstance(max_packet_bytes, bool) or not isinstance(max_packet_bytes, int):
            raise ValueError("单帧最大长度必须是整数")
        if max_packet_bytes < len(START_MARKER) + len(END_MARKER) + _MIN_FRAME_DATA_BYTES:
            raise ValueError("单帧最大长度小于 PTS 最小帧")
        if isinstance(read_size, bool) or not isinstance(read_size, int) or read_size <= 0:
            raise ValueError("单次读取长度必须是正整数")
        if (
            isinstance(packet_timeout_s, bool)
            or not isinstance(packet_timeout_s, (int, float))
            or packet_timeout_s <= 0
        ):
            raise ValueError("单包总等待时限必须是有限正数")
        if not np.isfinite(packet_timeout_s):
            raise ValueError("单包总等待时限必须是有限正数")
        if (
            isinstance(preview_bytes, bool)
            or not isinstance(preview_bytes, int)
            or preview_bytes <= 0
        ):
            raise ValueError("原始预览长度必须是正整数")
        self._serial_port = serial_port
        self._max_packet_bytes = max_packet_bytes
        self._read_size = read_size
        self._packet_timeout_s = float(packet_timeout_s)
        self._monotonic = monotonic
        self._preview_bytes = preview_bytes
        self._buffer = bytearray()
        self._last_diagnostics: PtsReadDiagnostics | None = None
        self._start_marker_tail = b""
        self._end_marker_tail = b""

    @property
    def last_diagnostics(self) -> PtsReadDiagnostics | None:
        """返回上一次 `read_packet()` 的冻结诊断快照。"""
        return self._last_diagnostics

    def reset_buffer(self) -> None:
        """丢弃协议层残留字节与跨分片标志状态。"""
        self._buffer.clear()
        self._start_marker_tail = b""
        self._end_marker_tail = b""

    def read_packet(self, packet_timeout_s: float | None = None) -> PtsPacket:
        """阻塞读取下一个可用的校验通过数据包。

        此时限覆盖不断收到噪声、坏帧或不完整帧的情况。底层串口 `read()` 自身仍可能阻塞至
        配置的单次读取超时，因此实际返回最多可能晚于总时限一个单次读取周期。

        Args:
            packet_timeout_s: 覆盖构造配置的单包总等待时限，单位 s。

        Raises:
            PtsReadTimeout: 未收到有效包时达到总时限，或底层读取未返回数据。
        """
        timeout_s = self._packet_timeout_s if packet_timeout_s is None else packet_timeout_s
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not np.isfinite(timeout_s)
            or timeout_s <= 0
        ):
            raise ValueError("单包总等待时限必须是有限正数")
        diagnostics = _DiagnosticsAccumulator(
            self._preview_bytes,
            self._start_marker_tail,
            self._end_marker_tail,
        )
        deadline = self._monotonic() + float(timeout_s)
        while True:
            if self._monotonic() >= deadline:
                self._last_diagnostics = diagnostics.snapshot()
                raise PtsReadTimeout("等待有效 PTS 包超过总时限", self._last_diagnostics)

            frame = self._pop_candidate_frame(diagnostics)
            if frame is not None:
                try:
                    packet = parse_packet(frame)
                except PacketChecksumError as exc:
                    diagnostics.checksum_failures += 1
                    diagnostics.last_protocol_error = str(exc)
                    self._restore_nested_start(frame)
                    continue
                except ProtocolError as exc:
                    diagnostics.protocol_failures += 1
                    diagnostics.last_protocol_error = str(exc)
                    self._restore_nested_start(frame)
                    continue
                self._last_diagnostics = diagnostics.snapshot()
                return packet

            chunk = self._serial_port.read(self._read_size)
            if not chunk:
                self._last_diagnostics = diagnostics.snapshot()
                raise PtsReadTimeout(
                    "PapillArray 串口单次读取超时无数据，请检查传感器供电和接线",
                    self._last_diagnostics,
                )
            diagnostics.record_chunk(chunk)
            self._start_marker_tail, self._end_marker_tail = diagnostics.marker_tails
            self._buffer.extend(chunk)

    def _pop_candidate_frame(self, diagnostics: _DiagnosticsAccumulator) -> bytes | None:
        """从缓冲区取出一个候选帧；无完整帧时保留可能的半包。"""
        start_index = self._buffer.find(START_MARKER)
        if start_index < 0:
            self._retain_start_suffix()
            return None
        if start_index > 0:
            del self._buffer[:start_index]

        end_index = self._buffer.find(END_MARKER, len(START_MARKER))
        if end_index < 0:
            if len(self._buffer) > self._max_packet_bytes:
                del self._buffer[: len(START_MARKER)]
                diagnostics.oversize_discards += 1
            return None
        if end_index + len(END_MARKER) > self._max_packet_bytes:
            del self._buffer[: end_index + len(END_MARKER)]
            diagnostics.oversize_discards += 1
            return None

        frame_data = bytes(self._buffer[len(START_MARKER) : end_index])
        del self._buffer[: end_index + len(END_MARKER)]
        diagnostics.candidate_frames += 1
        return frame_data

    def _retain_start_suffix(self) -> None:
        """保留缓冲区末尾可能构成起始标志前缀的字节。"""
        longest = 0
        for length in range(1, len(START_MARKER)):
            if self._buffer.endswith(START_MARKER[:length]):
                longest = length
        if longest:
            del self._buffer[:-longest]
        else:
            self._buffer.clear()

    def _restore_nested_start(self, frame: bytes) -> None:
        """候选帧异常时恢复其中可能是下一包起点的标志。

        若损坏帧缺少结束标志，下一有效帧的起始标志会落在当前候选帧内部。仅在当前候选帧
        已解析失败时恢复该后续标志，因此合法包中恰好出现起始字节序列不会受影响。
        """
        nested_start = frame.find(START_MARKER)
        if nested_start >= 0:
            self._buffer[:0] = frame[nested_start:] + END_MARKER


def _validate_sensor_counts(
    pillar_forces: list[np.ndarray],
    global_forces: list[np.ndarray],
    slip_states: list[np.ndarray],
    pillar_friction: list[np.ndarray],
    active: list[bool],
    reference_loaded: list[bool],
    sensor_friction: list[float],
    target_grip: list[float],
    *,
    has_pillar_slip: bool,
    has_sensor_slip: bool,
) -> None:
    """验证同一包中不同 Type 块报告的是同一组传感器。"""
    sensor_count = len(pillar_forces)
    if len(global_forces) != sensor_count:
        raise ProtocolError(
            f"Type 3 与 Type 4 传感器数不一致：{sensor_count} 与 {len(global_forces)}"
        )
    if has_pillar_slip:
        if len(slip_states) != sensor_count or len(pillar_friction) != sensor_count:
            raise ProtocolError("Type 5 与 Type 3 传感器数不一致")
        for sensor_index, pillar_force in enumerate(pillar_forces):
            pillar_count = len(pillar_force)
            if (
                len(slip_states[sensor_index]) != pillar_count
                or len(pillar_friction[sensor_index]) != pillar_count
            ):
                raise ProtocolError(f"Type 5 sensor {sensor_index} 与 Type 3 的 pillar 数不一致")
    if has_sensor_slip:
        type_6_counts = (
            len(active),
            len(reference_loaded),
            len(sensor_friction),
            len(target_grip),
        )
        if any(count != sensor_count for count in type_6_counts):
            raise ProtocolError("Type 6 与 Type 3 传感器数不一致")


def _parse_top_level_index(data: bytes, index_size: int) -> dict[int, int]:
    """解析 Type 到帧内偏移的索引表，并拒绝不安全的布局。"""
    if not 1 <= index_size <= 8:
        raise ProtocolError("PTS 索引偏移宽度必须在 1 至 8 字节之间")
    payload_end = len(data) - 2
    entry_size = 2 + index_size
    position = 1
    min_block_offset: int | None = None
    offsets: dict[int, int] = {}

    while True:
        index_limit = payload_end if min_block_offset is None else min_block_offset
        if position + entry_size > index_limit:
            if min_block_offset is None:
                raise ProtocolError("PTS 索引表截断")
            break
        block_type = _read_u16(data, position)
        if block_type == 0:
            break
        offset = int.from_bytes(data[position + 2 : position + entry_size], "little")
        if block_type in offsets:
            raise ProtocolError(f"PTS 索引包含重复的 Type {block_type}")
        if offset < position + entry_size or offset >= payload_end:
            raise ProtocolError(f"PTS Type {block_type} 的块偏移越界")
        if offset in offsets.values():
            raise ProtocolError(f"PTS Type {block_type} 与其他 Type 共用块偏移")
        offsets[block_type] = offset
        min_block_offset = offset if min_block_offset is None else min(min_block_offset, offset)
        position += entry_size

    if not offsets:
        raise ProtocolError("PTS 索引表为空")
    assert min_block_offset is not None
    if any(data[position:min_block_offset]):
        raise ProtocolError("PTS 索引表与首个数据块之间的填充必须全为零")
    if any(offset < min_block_offset for offset in offsets.values()):
        raise ProtocolError("PTS 块偏移落入索引表")
    return offsets


def _block_bytes(data: bytes, offsets: dict[int, int], block_type: int) -> bytes | None:
    """按顶层索引边界提取一个完整数据块。"""
    start = offsets.get(block_type)
    if start is None:
        return None
    end = min((offset for offset in offsets.values() if offset > start), default=len(data) - 2)
    if start >= end:
        raise ProtocolError(f"PTS Type {block_type} 的块范围为空或重叠")
    return data[start:end]


def _parse_pillar_block(block: bytes, index_size: int) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """解析 Type 3 的每 pillar 力与位移。"""
    reader = _BlockReader(block)
    sensor_count = reader.read_u16()
    sensor_offsets = reader.read_offsets(sensor_count, index_size)
    forces: list[np.ndarray] = []
    displacements: list[np.ndarray] = []
    for sensor_index in range(sensor_count):
        _require_offset(
            sensor_offsets[sensor_index], reader.position, f"Type 3 sensor {sensor_index}"
        )
        sensor_start = reader.position
        pillar_count = reader.read_u16()
        pillar_offsets = reader.read_offsets(pillar_count, index_size)
        force = np.empty((pillar_count, 3), dtype=np.float64)
        displacement = np.empty((pillar_count, 3), dtype=np.float64)
        for pillar_index in range(pillar_count):
            _require_offset(
                pillar_offsets[pillar_index],
                reader.position - sensor_start,
                f"Type 3 sensor {sensor_index} pillar {pillar_index}",
            )
            entry_type = reader.read_u16()
            if entry_type != 2:
                raise ProtocolError(f"Type 3 pillar 条目类型必须为 2，实际为 {entry_type}")
            fx, fy, fz, dx, dy, dz = reader.read_floats(6)
            force[pillar_index] = (fx, fy, fz)
            displacement[pillar_index] = (dx, dy, dz)
        forces.append(force)
        displacements.append(displacement)
    reader.require_end()
    return forces, displacements


def _parse_global_block(block: bytes, index_size: int) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """解析 Type 4 的每传感器全局力与力矩。"""
    reader = _BlockReader(block)
    sensor_count = reader.read_u16()
    sensor_offsets = reader.read_offsets(sensor_count, index_size)
    forces: list[np.ndarray] = []
    torques: list[np.ndarray] = []
    for sensor_index in range(sensor_count):
        _require_offset(
            sensor_offsets[sensor_index], reader.position, f"Type 4 sensor {sensor_index}"
        )
        entry_type = reader.read_u16()
        if entry_type != 2:
            raise ProtocolError(f"Type 4 sensor 条目类型必须为 2，实际为 {entry_type}")
        fx, fy, fz, tx, ty, tz = reader.read_floats(6)
        forces.append(np.array((fx, fy, fz), dtype=np.float64))
        torques.append(np.array((tx, ty, tz), dtype=np.float64))
    reader.require_end()
    return forces, torques


def _parse_pillar_slip_block(
    block: bytes, index_size: int
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """解析 Type 5 的每 pillar 滑动状态与摩擦估计。"""
    reader = _BlockReader(block)
    sensor_count = reader.read_u16()
    sensor_offsets = reader.read_offsets(sensor_count, index_size)
    states_by_sensor: list[np.ndarray] = []
    friction_by_sensor: list[np.ndarray] = []
    for sensor_index in range(sensor_count):
        _require_offset(
            sensor_offsets[sensor_index], reader.position, f"Type 5 sensor {sensor_index}"
        )
        sensor_start = reader.position
        pillar_count = reader.read_u16()
        pillar_offsets = reader.read_offsets(pillar_count, index_size)
        states = np.empty(pillar_count, dtype=np.int8)
        friction = np.empty(pillar_count, dtype=np.float64)
        for pillar_index in range(pillar_count):
            _require_offset(
                pillar_offsets[pillar_index],
                reader.position - sensor_start,
                f"Type 5 sensor {sensor_index} pillar {pillar_index}",
            )
            entry_type = reader.read_u16()
            if entry_type != 1:
                raise ProtocolError(f"Type 5 pillar 条目类型必须为 1，实际为 {entry_type}")
            states[pillar_index] = reader.read_int8()
            friction[pillar_index] = reader.read_floats(1)[0]
        states_by_sensor.append(states)
        friction_by_sensor.append(friction)
    reader.require_end()
    return states_by_sensor, friction_by_sensor


def _parse_sensor_slip_block(
    block: bytes, index_size: int
) -> tuple[list[bool], list[bool], list[float], list[float]]:
    """解析 Type 6 的传感器级滑动检测与抓握力估计。"""
    reader = _BlockReader(block)
    sensor_count = reader.read_u16()
    sensor_offsets = reader.read_offsets(sensor_count, index_size)
    active: list[bool] = []
    reference_loaded: list[bool] = []
    friction: list[float] = []
    target_grip: list[float] = []
    for sensor_index in range(sensor_count):
        _require_offset(
            sensor_offsets[sensor_index], reader.position, f"Type 6 sensor {sensor_index}"
        )
        entry_type = reader.read_u16()
        if entry_type != 1:
            raise ProtocolError(f"Type 6 sensor 条目类型必须为 1，实际为 {entry_type}")
        state = reader.read_uint8()
        friction_estimate, target_grip_force = reader.read_floats(2)
        active.append(state != 0)
        reference_loaded.append(state == 2)
        friction.append(friction_estimate)
        target_grip.append(target_grip_force)
    reader.require_end()
    return active, reference_loaded, friction, target_grip


@dataclass
class _BlockReader:
    """只在单个已切分块内移动的带边界检查读取器。"""

    data: bytes
    position: int = 0

    def read_u16(self) -> int:
        """读取一个小端无符号 16 位整数。"""
        self._require(2)
        value = _read_u16(self.data, self.position)
        self.position += 2
        return value

    def read_uint8(self) -> int:
        """读取一个无符号 8 位整数。"""
        self._require(1)
        value = self.data[self.position]
        self.position += 1
        return value

    def read_int8(self) -> int:
        """读取一个有符号 8 位整数。"""
        self._require(1)
        value = struct.unpack_from("<b", self.data, self.position)[0]
        self.position += 1
        return value

    def read_offsets(self, count: int, width: int) -> list[int]:
        """读取指定数量的相对偏移。"""
        self._require(count * width)
        offsets = [
            int.from_bytes(self.data[position : position + width], "little")
            for position in range(self.position, self.position + count * width, width)
        ]
        self.position += count * width
        return offsets

    def read_floats(self, count: int) -> tuple[float, ...]:
        """读取指定数量的小端单精度浮点数。"""
        size = count * 4
        self._require(size)
        values = struct.unpack_from(f"<{count}f", self.data, self.position)
        self.position += size
        return values

    def require_end(self) -> None:
        """确保块没有未解释的尾部字节。"""
        if self.position != len(self.data):
            raise ProtocolError("PTS 数据块长度与其声明结构不一致")

    def _require(self, size: int) -> None:
        """确保接下来读取不会越过块边界。"""
        if self.position + size > len(self.data):
            raise ProtocolError("PTS 数据块在字段中途结束")


def _require_offset(actual: int, expected: int, label: str) -> None:
    """验证连续布局的声明偏移恰好指向当前条目起点。"""
    if actual != expected:
        raise ProtocolError(f"{label} 相对偏移应为 {expected}，实际为 {actual}")


def _read_u16(data: bytes, position: int) -> int:
    """读取已由调用方保证范围的小端无符号 16 位整数。"""
    return int.from_bytes(data[position : position + 2], "little")

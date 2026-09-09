"""Contactile PapillArray PTS v2.0 帧解析与流重同步。"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Protocol

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


class _ReadablePort(Protocol):
    """协议读取器所需的最小串口读取接口。"""

    def read(self, size: int) -> bytes:
        """读取至多指定数量的字节。"""


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
    """

    def __init__(
        self,
        serial_port: _ReadablePort,
        max_packet_bytes: int = 8192,
        read_size: int = 4096,
    ) -> None:
        """保存读取设置而不发生 I/O。"""
        if isinstance(max_packet_bytes, bool) or not isinstance(max_packet_bytes, int):
            raise ValueError("单帧最大长度必须是整数")
        if max_packet_bytes < len(START_MARKER) + len(END_MARKER) + _MIN_FRAME_DATA_BYTES:
            raise ValueError("单帧最大长度小于 PTS 最小帧")
        if isinstance(read_size, bool) or not isinstance(read_size, int) or read_size <= 0:
            raise ValueError("单次读取长度必须是正整数")
        self._serial_port = serial_port
        self._max_packet_bytes = max_packet_bytes
        self._read_size = read_size
        self._buffer = bytearray()

    def reset_buffer(self) -> None:
        """丢弃协议层尚未组成完整帧的残留字节。"""
        self._buffer.clear()

    def read_packet(self) -> PtsPacket:
        """阻塞读取下一个可用的校验通过数据包。

        Raises:
            TimeoutError: 底层串口本次读取未返回数据。
        """
        while True:
            frame = self._pop_candidate_frame()
            if frame is not None:
                try:
                    return parse_packet(frame)
                except ProtocolError:
                    self._restore_nested_start(frame)
                    continue

            chunk = self._serial_port.read(self._read_size)
            if not chunk:
                raise TimeoutError("PapillArray 串口超时无数据，请检查传感器供电和接线")
            self._buffer.extend(chunk)

    def _pop_candidate_frame(self) -> bytes | None:
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
            return None
        if end_index + len(END_MARKER) > self._max_packet_bytes:
            del self._buffer[: end_index + len(END_MARKER)]
            return None

        frame_data = bytes(self._buffer[len(START_MARKER) : end_index])
        del self._buffer[: end_index + len(END_MARKER)]
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
    index_end: int | None = None
    offsets: dict[int, int] = {}

    while index_end is None or position < index_end:
        if position + entry_size > payload_end:
            raise ProtocolError("PTS 索引表截断")
        block_type = _read_u16(data, position)
        offset = int.from_bytes(data[position + 2 : position + entry_size], "little")
        if block_type == 0:
            raise ProtocolError("PTS 索引块类型不能为零")
        if block_type in offsets:
            raise ProtocolError(f"PTS 索引包含重复的 Type {block_type}")
        if offset < position + entry_size or offset >= payload_end:
            raise ProtocolError(f"PTS Type {block_type} 的块偏移越界")
        if offset in offsets.values():
            raise ProtocolError(f"PTS Type {block_type} 与其他 Type 共用块偏移")
        offsets[block_type] = offset
        index_end = offset if index_end is None else min(index_end, offset)
        position += entry_size

    if position != index_end:
        raise ProtocolError("PTS 索引表长度与最小块偏移不一致")
    if not offsets:
        raise ProtocolError("PTS 索引表为空")
    if any(offset < index_end for offset in offsets.values()):
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

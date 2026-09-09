"""PTS v2.0 解析和字节流重同步的离线回归测试。"""

from __future__ import annotations

from collections.abc import Callable
import struct

import numpy as np
import numpy.testing as npt
import pytest

from papillarray_hardware.protocol import (
    END_MARKER,
    START_MARKER,
    PacketChecksumError,
    ProtocolError,
    PtsReadTimeout,
    PtsStreamReader,
    _parse_top_level_index,
    parse_packet,
    verify_checksum,
)

_INDEX_SIZE = 2


class FakeReadableSerial:
    """按给定分片返回字节，完全不访问真实串口。"""

    def __init__(self, chunks: list[bytes]) -> None:
        """保存待返回的字节分片。"""
        self._chunks = list(chunks)

    def read(self, _size: int) -> bytes:
        """返回下一个分片，耗尽后模拟读取超时。"""
        return self._chunks.pop(0) if self._chunks else b""


class StepClock:
    """每次读取单调时钟都前进固定步长，无需测试休眠。"""

    def __init__(self, step_s: float = 1.0) -> None:
        """从零开始保存每次调用的推进量。"""
        self._value = 0.0
        self._step_s = step_s

    def __call__(self) -> float:
        """返回当前时刻并推进至下一离散时刻。"""
        value = self._value
        self._value += self._step_s
        return value


def test_parse_packet_preserves_hub_timestamp_and_type_7_data() -> None:
    """完整包解码 Type 1／3／4／5／6，并原样保留 Type 7。"""
    packet_data = build_packet_data(
        packet_counter=41,
        timestamp_us=987_654_321,
        sensors=[
            {
                "pillars": [(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)],
                "global": (4.0, 5.0, 6.0, 0.4, 0.5, 0.6),
            },
            {
                "pillars": [
                    (7.0, 8.0, 9.0, 0.7, 0.8, 0.9),
                    (10.0, 11.0, 12.0, 1.0, 1.1, 1.2),
                ],
                "global": (13.0, 14.0, 15.0, 1.3, 1.4, 1.5),
            },
        ],
        include_slip=True,
        type_7_data=b"\xde\xad\xbe\xef",
    )

    packet = parse_packet(packet_data)

    assert packet.packet_counter == 41
    assert packet.timestamp_us == 987_654_321
    assert packet.n_sensors == 2
    npt.assert_allclose(packet.pillar_forces[1][1], (10.0, 11.0, 12.0))
    npt.assert_allclose(packet.pillar_displacements[0][0], (0.1, 0.2, 0.3))
    npt.assert_allclose(packet.global_torques[1], (1.3, 1.4, 1.5))
    npt.assert_array_equal(packet.pillar_slip_states[0], np.array((3,), dtype=np.int8))
    npt.assert_allclose(packet.pillar_friction_estimates[1], (0.82, 0.92))
    assert packet.slip_detection_active == [True, True]
    assert packet.reference_pillar_loaded == [True, False]
    assert packet.sensor_friction_estimates == pytest.approx([0.68, 0.78])
    assert packet.target_grip_forces == pytest.approx([12.5, 13.5])
    assert packet.type_7_data == b"\xde\xad\xbe\xef"


def test_parse_packet_accepts_controller_v20_absolute_nested_offsets() -> None:
    """真实 Controller v2.0 帧头的 Type 3 两级绝对偏移可进入条目解析。"""
    sensors = [
        {
            "pillars": [(float(index), 2.0, 3.0, 0.1, 0.2, 0.3) for index in range(9)],
            "global": (4.0, 5.0, 6.0, 0.4, 0.5, 0.6),
        },
        {
            "pillars": [(float(index), 8.0, 9.0, 0.7, 0.8, 0.9) for index in range(9)],
            "global": (13.0, 14.0, 15.0, 1.3, 1.4, 1.5),
        },
    ]
    type_3_offset = 0x0027
    pillar = build_pillar_block(sensors, type_3_offset)
    type_4_offset = type_3_offset + len(pillar)
    global_data = build_global_block(sensors, type_4_offset)
    type_5_offset = type_4_offset + len(global_data)
    pillar_slip = build_pillar_slip_block(sensors, type_5_offset)
    type_6_offset = type_5_offset + len(pillar_slip)
    sensor_slip = build_sensor_slip_block(sensors, type_6_offset)
    type_7_offset = type_6_offset + len(sensor_slip)

    assert (type_4_offset, type_5_offset, type_6_offset, type_7_offset) == (
        0x0229,
        0x0263,
        0x030F,
        0x032B,
    )
    data = bytearray.fromhex("0201001b0003002700040029020500630206000f0307002b030000")
    data.extend((1234).to_bytes(4, "little") + (5678).to_bytes(8, "little"))
    data.extend(pillar)
    data.extend(global_data)
    data.extend(pillar_slip)
    data.extend(sensor_slip)
    data.extend(b"\x00")
    data.extend((sum(data) & 0xFFFF).to_bytes(2, "little"))

    assert int.from_bytes(data[7:9], "little") == type_3_offset
    assert int.from_bytes(data[41:43], "little") == 45
    assert int.from_bytes(data[43:45], "little") == 299
    assert int.from_bytes(data[47:49], "little") == 65

    packet = parse_packet(bytes(data))

    assert packet.n_sensors == 2
    assert [len(force) for force in packet.pillar_forces] == [9, 9]
    npt.assert_allclose(packet.pillar_forces[1][8], (8.0, 8.0, 9.0))


def test_parse_packet_rejects_bad_checksum_and_invalid_offsets() -> None:
    """直接解析时明确报告坏校验和和越界偏移。"""
    good = build_packet_data(
        packet_counter=1,
        timestamp_us=2,
        sensors=[{"pillars": [], "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}],
    )
    corrupted = bytearray(good)
    corrupted[-3] ^= 0x01

    with pytest.raises(PacketChecksumError, match="校验和"):
        parse_packet(bytes(corrupted))

    invalid = bytearray(good)
    invalid[3:5] = (65_535).to_bytes(2, "little")
    invalid[-2:] = (sum(invalid[:-2]) & 0xFFFF).to_bytes(2, "little")
    with pytest.raises(ProtocolError, match="偏移越界"):
        parse_packet(bytes(invalid))


def test_parse_packet_rejects_nested_pillar_offset_into_its_index_header() -> None:
    """Type 3 pillar 偏移不得回指传感器索引头。"""
    packet_data = bytearray(
        build_packet_data(
            packet_counter=1,
            timestamp_us=2,
            sensors=[
                {
                    "pillars": [(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)],
                    "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                }
            ],
        )
    )
    pillar_block_offset = int.from_bytes(packet_data[7:9], "little")
    pillar_offset_position = pillar_block_offset + 6
    packet_data[pillar_offset_position : pillar_offset_position + _INDEX_SIZE] = (5).to_bytes(
        _INDEX_SIZE, "little"
    )
    packet_data[-2:] = (sum(packet_data[:-2]) & 0xFFFF).to_bytes(2, "little")

    with pytest.raises(ProtocolError, match="Type 3 sensor 0 pillar 0 绝对偏移落入索引头"):
        parse_packet(bytes(packet_data))


def test_parse_packet_rejects_two_types_sharing_one_top_level_offset() -> None:
    """不同 Type 不能将同一段字节重复解释为两种数据块。"""
    packet_data = bytearray(
        build_packet_data(
            packet_counter=1,
            timestamp_us=2,
            sensors=[{"pillars": [], "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}],
        )
    )
    pillar_block_offset = packet_data[7:9]
    packet_data[11:13] = pillar_block_offset
    packet_data[-2:] = (sum(packet_data[:-2]) & 0xFFFF).to_bytes(2, "little")

    with pytest.raises(ProtocolError, match="共用块偏移"):
        parse_packet(bytes(packet_data))


@pytest.mark.parametrize("padding", (b"\x00\x00", b"\x00" * 6))
def test_parse_packet_accepts_zero_top_level_index_padding(padding: bytes) -> None:
    """顶层索引与首个块之间允许设备写入任意长度的零对齐填充。"""
    packet = parse_packet(
        build_packet_data(
            packet_counter=1,
            timestamp_us=2,
            sensors=[{"pillars": [], "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}],
            top_level_index_padding=padding,
        )
    )

    assert packet.packet_counter == 1


def test_parse_packet_rejects_nonzero_top_level_index_padding() -> None:
    """顶层索引后的填充不能掩盖未声明的数据。"""
    packet_data = build_packet_data(
        packet_counter=1,
        timestamp_us=2,
        sensors=[{"pillars": [], "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}],
        top_level_index_padding=b"\x00\x01",
    )

    with pytest.raises(ProtocolError, match="填充必须全为零"):
        parse_packet(packet_data)


def test_parse_packet_without_top_level_index_padding_remains_compatible() -> None:
    """无顶层索引填充的既有连续布局仍可解析。"""
    packet = parse_packet(
        build_packet_data(
            packet_counter=3,
            timestamp_us=4,
            sensors=[{"pillars": [], "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}],
        )
    )

    assert packet.packet_counter == 3


def test_real_preview_top_level_index_accepts_zero_alignment_padding() -> None:
    """真实设备预览不会将两字节对齐填充误解为 Type 0 索引项。"""
    preview = bytes.fromhex(
        "556677880201001b0003002700040029020500630206000f0307002b030000"
        "4efa320031aa768f0000000002002d002b01090041005b0075008f00a900c300dd"
    )

    with pytest.raises(ProtocolError) as caught:
        _parse_top_level_index(preview[len(START_MARKER) :], _INDEX_SIZE)

    assert "偏移越界" in str(caught.value)
    assert "Type 0" not in str(caught.value)


def test_parse_packet_rejects_inconsistent_type_sensor_and_pillar_counts() -> None:
    """Type 3 是一个观测包的基准，其他已存在块必须与它逐项对齐。"""
    sensors = [
        {
            "pillars": [(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)],
            "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        },
        {
            "pillars": [(4.0, 5.0, 6.0, 0.4, 0.5, 0.6)],
            "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        },
    ]
    with pytest.raises(ProtocolError, match="Type 3 与 Type 4"):
        parse_packet(
            build_packet_data(
                packet_counter=1,
                timestamp_us=2,
                sensors=sensors,
                global_sensors=sensors[:1],
            )
        )
    with pytest.raises(ProtocolError, match="Type 5 与 Type 3"):
        parse_packet(
            build_packet_data(
                packet_counter=1,
                timestamp_us=2,
                sensors=sensors,
                pillar_slip_sensors=sensors[:1],
            )
        )
    with pytest.raises(ProtocolError, match="Type 5 sensor 0"):
        parse_packet(
            build_packet_data(
                packet_counter=1,
                timestamp_us=2,
                sensors=sensors,
                pillar_slip_sensors=[
                    {
                        "pillars": [
                            (1.0, 2.0, 3.0, 0.1, 0.2, 0.3),
                            (7.0, 8.0, 9.0, 0.7, 0.8, 0.9),
                        ],
                        "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    },
                    sensors[1],
                ],
            )
        )
    with pytest.raises(ProtocolError, match="Type 6 与 Type 3"):
        parse_packet(
            build_packet_data(
                packet_counter=1,
                timestamp_us=2,
                sensors=sensors,
                sensor_slip_sensors=sensors[:1],
            )
        )


def test_stream_reader_recovers_from_noise_bad_frame_and_half_packets() -> None:
    """读取器跳过噪声与坏帧，并跨多个读取分片重组有效包。"""
    good = build_packet_data(
        packet_counter=123,
        timestamp_us=777_888_999,
        sensors=[{"pillars": [], "global": (6.0, 5.0, 4.0, 0.6, 0.5, 0.4)}],
    )
    bad = bytearray(good)
    bad[8] ^= 0x10
    good_frame = START_MARKER + good + END_MARKER
    bad_frame = START_MARKER + bytes(bad) + END_MARKER
    reader = PtsStreamReader(
        FakeReadableSerial(
            [
                b"noise\x55\x66",
                b"\x77\x88" + bad_frame[:19],
                bad_frame[19:] + good_frame[:7],
                good_frame[7:25],
                good_frame[25:],
            ]
        )
    )

    packet = reader.read_packet()

    assert packet.packet_counter == 123
    assert packet.timestamp_us == 777_888_999
    npt.assert_allclose(packet.global_forces[0], (6.0, 5.0, 4.0))


def test_stream_reader_recovers_good_frame_after_bad_frame_without_end_marker() -> None:
    """损坏帧缺少结束标志时，不得吞掉紧随其后的完整有效帧。"""
    good = build_packet_data(
        packet_counter=124,
        timestamp_us=777_889_000,
        sensors=[{"pillars": [], "global": (6.0, 5.0, 4.0, 0.6, 0.5, 0.4)}],
    )
    good_frame = START_MARKER + good + END_MARKER
    reader = PtsStreamReader(FakeReadableSerial([START_MARKER + b"broken" + good_frame]))

    packet = reader.read_packet()

    assert packet.packet_counter == 124


def test_stream_reader_accepts_valid_type_7_data_containing_start_marker() -> None:
    """合法包内偶然出现起始字节序列时，不能错误触发重同步。"""
    packet_data = build_packet_data(
        packet_counter=125,
        timestamp_us=777_889_001,
        sensors=[{"pillars": [], "global": (6.0, 5.0, 4.0, 0.6, 0.5, 0.4)}],
        type_7_data=START_MARKER,
    )
    reader = PtsStreamReader(FakeReadableSerial([START_MARKER + packet_data + END_MARKER]))

    packet = reader.read_packet()

    assert packet.packet_counter == 125
    assert packet.type_7_data == START_MARKER


def test_stream_reader_discards_oversize_half_packet_and_times_out() -> None:
    """没有结束标志的超长残片不会令缓冲无限增长。"""
    reader = PtsStreamReader(
        FakeReadableSerial([START_MARKER + b"x" * 40, b""]),
        max_packet_bytes=32,
        packet_timeout_s=3.0,
        monotonic=StepClock(),
    )

    with pytest.raises(PtsReadTimeout, match="总时限") as caught:
        reader.read_packet()

    diagnostics = caught.value.diagnostics
    assert diagnostics.oversize_discards == 1
    assert diagnostics.received_bytes == len(START_MARKER) + 40


def test_stream_reader_accepts_packet_after_initial_empty_reads() -> None:
    """首次配置流后的若干次空读取不应抢先终止总启动等待。"""
    packet_data = build_packet_data(
        packet_counter=129,
        timestamp_us=777_889_005,
        sensors=[{"pillars": [], "global": (6.0, 5.0, 4.0, 0.6, 0.5, 0.4)}],
    )
    frame = START_MARKER + packet_data + END_MARKER
    reader = PtsStreamReader(
        FakeReadableSerial([b"", b"", frame]),
        packet_timeout_s=4.0,
        monotonic=StepClock(),
    )

    packet = reader.read_packet()

    assert packet.packet_counter == 129
    assert reader.last_diagnostics is not None
    assert reader.last_diagnostics.empty_reads == 2
    assert reader.last_diagnostics.received_bytes == len(frame)


def test_stream_reader_empty_reads_reach_total_deadline_with_diagnostics() -> None:
    """持续空读取应在总时限退出并冻结准确的空读取次数。"""
    reader = PtsStreamReader(
        FakeReadableSerial([b""] * 10),
        packet_timeout_s=3.0,
        monotonic=StepClock(),
    )

    with pytest.raises(PtsReadTimeout, match="总时限") as caught:
        reader.read_packet()

    diagnostics = caught.value.diagnostics
    assert diagnostics.empty_reads == 2
    assert diagnostics.received_bytes == 0
    assert diagnostics.raw_hex_preview == ""
    assert reader.last_diagnostics == diagnostics


def test_stream_reader_continuous_noise_reaches_total_deadline_with_diagnostics() -> None:
    """持续收到不可解析字节时，单包总时限仍必须终止读取。"""
    reader = PtsStreamReader(
        FakeReadableSerial([b"noise"] * 10),
        packet_timeout_s=3.0,
        monotonic=StepClock(),
    )

    with pytest.raises(PtsReadTimeout, match="总时限") as caught:
        reader.read_packet()

    diagnostics = caught.value.diagnostics
    assert diagnostics.received_bytes == 10
    assert diagnostics.candidate_frames == 0
    assert diagnostics.raw_hex_preview == b"noisenoise".hex()
    assert reader.last_diagnostics == diagnostics


def test_stream_reader_counts_markers_across_chunks() -> None:
    """跨串口分片拆开的起止标志仍各计数一次。"""
    reader = PtsStreamReader(
        FakeReadableSerial([b"noise\x55\x66", b"\x77\x88bad\xaa", b"\xbb\xcc\xdd"]),
        packet_timeout_s=3.0,
        monotonic=StepClock(0.5),
    )

    with pytest.raises(PtsReadTimeout) as caught:
        reader.read_packet()

    diagnostics = caught.value.diagnostics
    assert diagnostics.start_markers == 1
    assert diagnostics.end_markers == 1
    assert diagnostics.candidate_frames == 1
    assert diagnostics.checksum_failures == 1


def test_stream_reader_counts_second_packet_markers_across_read_calls() -> None:
    """前一包返回时留下的第二包标志半段必须记入下一次等待诊断。"""
    first_data = build_packet_data(
        packet_counter=127,
        timestamp_us=777_889_003,
        sensors=[{"pillars": [], "global": (6.0, 5.0, 4.0, 0.6, 0.5, 0.4)}],
    )
    second_data = build_packet_data(
        packet_counter=128,
        timestamp_us=777_889_004,
        sensors=[{"pillars": [], "global": (5.0, 4.0, 3.0, 0.5, 0.4, 0.3)}],
    )
    first_frame = START_MARKER + first_data + END_MARKER
    reader = PtsStreamReader(
        FakeReadableSerial(
            [
                first_frame + START_MARKER[:2],
                START_MARKER[2:] + second_data + END_MARKER[:2],
                END_MARKER[2:],
            ]
        ),
        monotonic=StepClock(),
    )

    first_packet = reader.read_packet()
    first_diagnostics = reader.last_diagnostics
    second_packet = reader.read_packet()
    second_diagnostics = reader.last_diagnostics

    assert first_packet.packet_counter == 127
    assert first_diagnostics is not None
    assert second_packet.packet_counter == 128
    assert second_diagnostics is not None
    assert second_diagnostics.start_markers == 1
    assert second_diagnostics.end_markers == 1


def test_stream_reader_reset_clears_cross_chunk_marker_state() -> None:
    """设备清零前后的字节流边界不能拼成虚假的协议标志。"""
    reader = PtsStreamReader(
        FakeReadableSerial([START_MARKER[:2], b"", START_MARKER[2:], b""]),
        monotonic=StepClock(),
    )

    with pytest.raises(PtsReadTimeout):
        reader.read_packet()
    reader.reset_buffer()
    with pytest.raises(PtsReadTimeout) as caught:
        reader.read_packet()

    assert caught.value.diagnostics.start_markers == 0


def test_stream_reader_classifies_checksum_and_structure_failures() -> None:
    """坏校验和与通过校验但结构错误的候选帧必须分开计数。"""
    good = build_packet_data(
        packet_counter=1,
        timestamp_us=2,
        sensors=[{"pillars": [], "global": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}],
    )
    checksum_bad = bytearray(good)
    checksum_bad[-3] ^= 0x01
    structure_bad = bytearray(good)
    structure_bad[3:5] = (65_535).to_bytes(2, "little")
    structure_bad[-2:] = (sum(structure_bad[:-2]) & 0xFFFF).to_bytes(2, "little")
    reader = PtsStreamReader(
        FakeReadableSerial(
            [
                START_MARKER + bytes(checksum_bad) + END_MARKER,
                START_MARKER + bytes(structure_bad) + END_MARKER,
            ]
        ),
        monotonic=StepClock(),
    )

    with pytest.raises(PtsReadTimeout) as caught:
        reader.read_packet()

    diagnostics = caught.value.diagnostics
    assert diagnostics.candidate_frames == 2
    assert diagnostics.checksum_failures == 1
    assert diagnostics.protocol_failures == 1
    assert diagnostics.last_protocol_error is not None
    assert "偏移越界" in diagnostics.last_protocol_error


def test_stream_reader_limits_preview_and_exposes_success_diagnostics() -> None:
    """诊断预览有固定上限，且成功读取后调用方仍可检查诊断。"""
    packet_data = build_packet_data(
        packet_counter=126,
        timestamp_us=777_889_002,
        sensors=[{"pillars": [], "global": (6.0, 5.0, 4.0, 0.6, 0.5, 0.4)}],
    )
    frame = START_MARKER + packet_data + END_MARKER
    reader = PtsStreamReader(FakeReadableSerial([frame]), monotonic=lambda: 0.0, preview_bytes=8)

    packet = reader.read_packet()

    diagnostics = reader.last_diagnostics
    assert packet.packet_counter == 126
    assert diagnostics is not None
    assert diagnostics.received_bytes == len(frame)
    assert diagnostics.raw_hex_preview == frame[:8].hex()
    assert diagnostics.candidate_frames == 1


def test_verify_checksum_rejects_too_short_data() -> None:
    """校验函数不会把没有校验字段的短数据误判为有效。"""
    assert not verify_checksum(b"")
    assert not verify_checksum(b"\x01")


def build_packet_data(
    *,
    packet_counter: int,
    timestamp_us: int,
    sensors: list[dict[str, object]],
    include_slip: bool = False,
    type_7_data: bytes | None = None,
    global_sensors: list[dict[str, object]] | None = None,
    pillar_slip_sensors: list[dict[str, object]] | None = None,
    sensor_slip_sensors: list[dict[str, object]] | None = None,
    top_level_index_padding: bytes = b"",
) -> bytes:
    """构造严格符合测试所需布局且带校验和的 PTS 帧内部数据。"""
    block_builders: list[tuple[int, Callable[[int], bytes]]] = [
        (
            1,
            lambda _base: packet_counter.to_bytes(4, "little") + timestamp_us.to_bytes(8, "little"),
        ),
        (3, lambda base: build_pillar_block(sensors, base)),
        (
            4,
            lambda base: build_global_block(
                global_sensors if global_sensors is not None else sensors, base
            ),
        ),
    ]
    if include_slip:
        block_builders.extend(
            (
                (5, lambda base: build_pillar_slip_block(sensors, base)),
                (6, lambda base: build_sensor_slip_block(sensors, base)),
            )
        )
    if pillar_slip_sensors is not None:
        block_builders.append((5, lambda base: build_pillar_slip_block(pillar_slip_sensors, base)))
    if sensor_slip_sensors is not None:
        block_builders.append((6, lambda base: build_sensor_slip_block(sensor_slip_sensors, base)))
    if type_7_data is not None:
        type_7_raw = type_7_data
        block_builders.append((7, lambda _base: type_7_raw))

    first_block_offset = 1 + len(block_builders) * (2 + _INDEX_SIZE) + len(top_level_index_padding)
    offset = first_block_offset
    blocks: list[tuple[int, bytes]] = []
    for block_type, builder in block_builders:
        block = builder(offset)
        blocks.append((block_type, block))
        offset += len(block)
    packet = bytearray((_INDEX_SIZE,))
    offset = first_block_offset
    for block_type, block in blocks:
        packet.extend(block_type.to_bytes(2, "little"))
        packet.extend(offset.to_bytes(_INDEX_SIZE, "little"))
        offset += len(block)
    packet.extend(top_level_index_padding)
    for _, block in blocks:
        packet.extend(block)
    packet.extend((sum(packet) & 0xFFFF).to_bytes(2, "little"))
    return bytes(packet)


def build_pillar_block(sensors: list[dict[str, object]], base_offset: int = 0) -> bytes:
    """构造 Type 3，其嵌套偏移均以整帧起点为基址。"""
    sensor_blocks: list[bytes] = []
    sensor_offsets: list[int] = []
    running_offset = 2 + len(sensors) * _INDEX_SIZE
    for sensor in sensors:
        pillars = list(sensor["pillars"])
        payload = bytearray(len(pillars).to_bytes(2, "little"))
        pillar_offsets: list[int] = []
        pillar_offset = base_offset + running_offset + 2 + len(pillars) * _INDEX_SIZE
        for pillar in pillars:
            pillar_offsets.append(pillar_offset)
            payload.extend((2).to_bytes(2, "little"))
            payload.extend(struct.pack("<6f", *pillar))
            pillar_offset += 26
        indexed = bytearray(len(pillars).to_bytes(2, "little"))
        for value in pillar_offsets:
            indexed.extend(value.to_bytes(_INDEX_SIZE, "little"))
        indexed.extend(payload[2:])
        sensor_offsets.append(base_offset + running_offset)
        sensor_blocks.append(bytes(indexed))
        running_offset += len(indexed)

    result = bytearray(len(sensors).to_bytes(2, "little"))
    for offset in sensor_offsets:
        result.extend(offset.to_bytes(_INDEX_SIZE, "little"))
    for block in sensor_blocks:
        result.extend(block)
    return bytes(result)


def build_global_block(sensors: list[dict[str, object]], base_offset: int = 0) -> bytes:
    """构造 Type 4，其传感器偏移以整帧起点为基址。"""
    sensor_offsets: list[int] = []
    sensor_blocks: list[bytes] = []
    running_offset = 2 + len(sensors) * _INDEX_SIZE
    for sensor in sensors:
        block = (2).to_bytes(2, "little") + struct.pack("<6f", *sensor["global"])
        sensor_offsets.append(base_offset + running_offset)
        sensor_blocks.append(block)
        running_offset += len(block)
    result = bytearray(len(sensors).to_bytes(2, "little"))
    for offset in sensor_offsets:
        result.extend(offset.to_bytes(_INDEX_SIZE, "little"))
    for block in sensor_blocks:
        result.extend(block)
    return bytes(result)


def build_pillar_slip_block(sensors: list[dict[str, object]], base_offset: int = 0) -> bytes:
    """构造 Type 5，其嵌套偏移以整帧起点为基址。"""
    sensor_offsets: list[int] = []
    sensor_blocks: list[bytes] = []
    running_offset = 2 + len(sensors) * _INDEX_SIZE
    for sensor_index, sensor in enumerate(sensors):
        pillars = list(sensor["pillars"])
        pillar_offsets = [
            base_offset + running_offset + 2 + len(pillars) * _INDEX_SIZE + 7 * index
            for index in range(len(pillars))
        ]
        block = bytearray(len(pillars).to_bytes(2, "little"))
        for offset in pillar_offsets:
            block.extend(offset.to_bytes(_INDEX_SIZE, "little"))
        for pillar_index in range(len(pillars)):
            block.extend((1).to_bytes(2, "little"))
            block.extend(struct.pack("<bf", 3, 0.72 + sensor_index * 0.1 + pillar_index * 0.1))
        sensor_offsets.append(base_offset + running_offset)
        sensor_blocks.append(bytes(block))
        running_offset += len(block)
    result = bytearray(len(sensors).to_bytes(2, "little"))
    for offset in sensor_offsets:
        result.extend(offset.to_bytes(_INDEX_SIZE, "little"))
    for block in sensor_blocks:
        result.extend(block)
    return bytes(result)


def build_sensor_slip_block(sensors: list[dict[str, object]], base_offset: int = 0) -> bytes:
    """构造 Type 6，其传感器偏移以整帧起点为基址。"""
    result = bytearray(len(sensors).to_bytes(2, "little"))
    running_offset = 2 + len(sensors) * _INDEX_SIZE
    offsets: list[int] = []
    blocks: list[bytes] = []
    for sensor_index, _sensor in enumerate(sensors):
        block = (1).to_bytes(2, "little") + bytes((2 if sensor_index == 0 else 1,))
        block += struct.pack("<2f", 0.68 + sensor_index * 0.1, 12.5 + sensor_index)
        offsets.append(base_offset + running_offset)
        blocks.append(block)
        running_offset += len(block)
    for offset in offsets:
        result.extend(offset.to_bytes(_INDEX_SIZE, "little"))
    for block in blocks:
        result.extend(block)
    return bytes(result)

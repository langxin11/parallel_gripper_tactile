"""PapillArray 采集会话与逐 taxel 完整性的纯 fake 测试。"""

from __future__ import annotations

import math
import time
from dataclasses import asdict

import numpy as np
import pytest

from papillarray_hardware import (
    PacketIntegrityError,
    PacketIntegrityTracker,
    PapillArraySerialConfig,
    PtsPacket,
    TactileWorker,
)


def _packet(
    counter: int,
    timestamp_us: int,
    *,
    counts: tuple[int, int] = (2, 2),
    global_forces: tuple[tuple[float, float, float], ...] = (
        (0.1, -0.2, 0.3),
        (-0.4, 0.5, 0.6),
    ),
) -> PtsPacket:
    """构造完整双侧 fake PTS 包。"""
    taxels = [np.arange(count * 3, dtype=np.float64).reshape(count, 3) for count in counts]
    return PtsPacket(
        packet_counter=counter,
        timestamp_us=timestamp_us,
        pillar_forces=taxels,
        pillar_displacements=[np.zeros_like(values) for values in taxels],
        global_forces=[np.asarray(values, dtype=np.float64) for values in global_forces],
        global_torques=[np.zeros(3) for _ in counts],
    )


def test_tracker_accepts_complete_packet_and_reports_gap_and_wrap() -> None:
    """缺包和 uint32 合法回绕保留诊断，但不阻断有效数据。"""
    tracker = PacketIntegrityTracker(2)
    first = tracker.inspect(_packet(0xFFFFFFFD, 1_000))
    gap = tracker.inspect(_packet(0xFFFFFFFF, 2_000))
    wrapped = tracker.inspect(_packet(1, 3_000))

    assert first.counter_event == "first"
    assert first.taxel_counts == (2, 2)
    assert (gap.counter_event, gap.counter_gap) == ("gap", 1)
    assert (wrapped.counter_event, wrapped.counter_gap) == ("wrap", 1)


@pytest.mark.parametrize(
    ("field", "expected"),
    [
        ("taxel_nan", r"sensor 1 taxel 0 Fy"),
        ("taxel_inf", r"sensor 1 taxel 0 Fy"),
        ("global_nan", r"sensor 1 全局 Fy"),
    ],
)
def test_tracker_rejects_nonfinite_taxel_and_global_axes(field: str, expected: str) -> None:
    """全局与逐 taxel 任一轴非有限时都报告精确位置。"""
    packet = _packet(1, 1_000)
    if field == "taxel_nan":
        packet.pillar_forces[1][0, 1] = math.nan
    elif field == "taxel_inf":
        packet.pillar_forces[1][0, 1] = math.inf
    else:
        packet.global_forces[1][1] = math.nan

    with pytest.raises(PacketIntegrityError, match=expected):
        PacketIntegrityTracker(2).inspect(packet)


@pytest.mark.parametrize("counts", ((0, 2), (2, 0)))
def test_tracker_rejects_empty_taxel_side(counts: tuple[int, int]) -> None:
    """每个部署传感器都必须提供至少一个 taxel。"""
    with pytest.raises(PacketIntegrityError, match="不包含 taxel"):
        PacketIntegrityTracker(2).inspect(_packet(1, 1_000, counts=counts))


def test_tracker_rejects_sensor_count_and_malformed_shapes() -> None:
    """部署传感器数及逐 taxel／全局三轴 shape 必须逐包成立。"""
    one_sensor = _packet(1, 1_000)
    one_sensor.pillar_forces.pop()
    one_sensor.global_forces.pop()
    with pytest.raises(PacketIntegrityError, match="传感器数.*不一致"):
        PacketIntegrityTracker(2).inspect(one_sensor)

    malformed_taxels = _packet(1, 1_000)
    malformed_taxels.pillar_forces[0] = np.zeros((2, 2))
    with pytest.raises(PacketIntegrityError, match=r"shape 必须为 \(N, 3\)"):
        PacketIntegrityTracker(2).inspect(malformed_taxels)

    malformed_global = _packet(1, 1_000)
    malformed_global.global_forces[0] = np.zeros(2)
    with pytest.raises(PacketIntegrityError, match=r"shape 必须为 \(3,\)"):
        PacketIntegrityTracker(2).inspect(malformed_global)


def test_tracker_preserves_topology_across_sequence_reset() -> None:
    """bias 可重置设备计数和时间，但不能掩盖本次会话的 taxel 数变化。"""
    tracker = PacketIntegrityTracker(2)
    tracker.inspect(_packet(20, 20_000))
    tracker.reset_stream_sequence()
    assert tracker.inspect(_packet(1, 1_000)).counter_event == "first"

    with pytest.raises(PacketIntegrityError, match="taxel 数.*发生变化"):
        tracker.inspect(_packet(2, 2_000, counts=(3, 2)))


@pytest.mark.parametrize(
    ("second_counter", "second_timestamp", "expected"),
    [
        (10, 2_000, "计数器重复"),
        (9, 2_000, "计数器乱序"),
        (11, 1_000, "设备时间未严格递增"),
        (11, 999, "设备时间未严格递增"),
    ],
)
def test_tracker_rejects_duplicate_out_of_order_and_nonincreasing_time(
    second_counter: int, second_timestamp: int, expected: str
) -> None:
    """非前进计数和非递增设备时间均为硬故障。"""
    tracker = PacketIntegrityTracker(2)
    tracker.inspect(_packet(10, 1_000))
    with pytest.raises(PacketIntegrityError, match=expected):
        tracker.inspect(_packet(second_counter, second_timestamp))


class _ScriptedClient:
    """按序返回 fake 包并记录配置和 bias 命令。"""

    def __init__(self, packets: list[PtsPacket]) -> None:
        self._packets = packets
        self.commands: list[str] = []

    def __enter__(self):
        return self

    def __exit__(self, *_args: object) -> None:
        return None

    def configure_stream(self) -> None:
        self.commands.append("configure")

    def clear_bias(self) -> None:
        self.commands.append("bias")

    def read_packet(self) -> PtsPacket:
        if self._packets:
            time.sleep(0.001)
            return self._packets.pop(0)
        time.sleep(0.001)
        return _packet(4, 4_000)


def test_worker_biases_once_and_only_publishes_post_bias_taxels() -> None:
    """首个完整包触发唯一 bias，bias 前包既不发布也不写入采样 sink。"""
    pre_bias = _packet(
        1,
        1_000,
        global_forces=((0.1, -0.2, 9.0), (-0.4, 0.5, 8.0)),
    )
    post_bias = _packet(2, 2_000)
    post_bias.pillar_forces[0][0] = (7.0, 8.0, 9.0)
    client = _ScriptedClient([pre_bias, post_bias])
    records: list[dict[str, object]] = []
    transformed: list[int] = []

    def transform(snapshot):
        """转换仅收到 bias 后的完整快照，结果先记录再原子发布。"""
        transformed.append(snapshot.packet_counter)
        return snapshot

    worker = TactileWorker(
        PapillArraySerialConfig(timeout_s=0.01, packet_timeout_s=0.02),
        clear_bias=True,
        client_factory=lambda _config: client,
        sample_sink=records.append,
        snapshot_transform=transform,
    )
    worker.start()
    try:
        snapshot = worker.wait_for_update(None, 0.5)
        assert snapshot.packet_counter == 2
        assert transformed == [2]
        assert snapshot.left_force_n == pytest.approx(0.3)
        assert snapshot.right_force_n == pytest.approx(0.6)
        assert snapshot.left_taxel_forces_n[0] == (7.0, 8.0, 9.0)
        assert snapshot.counter_event == "first"
        assert client.commands == ["configure", "bias"]
        assert records and all(record["packet_counter"] != 1 for record in records)
        assert asdict(snapshot) in records
    finally:
        worker.stop()


def test_worker_propagates_taxel_count_change_without_publishing_bad_packet() -> None:
    """运行中 taxel 数变化终止采集，坏包不会进入采样 sink。"""
    client = _ScriptedClient([_packet(1, 1_000), _packet(2, 2_000, counts=(3, 2))])
    records: list[dict[str, object]] = []
    worker = TactileWorker(
        PapillArraySerialConfig(timeout_s=0.01, packet_timeout_s=0.02),
        clear_bias=False,
        client_factory=lambda _config: client,
        sample_sink=records.append,
    )
    worker.start()
    try:
        first = worker.wait_for_update(None, 0.5)
        with pytest.raises(RuntimeError, match="taxel 数.*发生变化"):
            worker.wait_for_update(first.received_at_s, 0.5)
        assert [record["packet_counter"] for record in records] == [1]
    finally:
        worker.stop()

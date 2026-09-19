"""原厂逐触点滑移、摩擦和位移进入快照与日志的离线测试。"""

from __future__ import annotations

import json
from dataclasses import asdict, replace

import numpy as np

from papillarray_hardware import (
    PacketIntegrityTracker,
    PapillArraySerialConfig,
    PtsPacket,
    TactileSnapshot,
    TactileWorker,
)


def _packet() -> PtsPacket:
    """构造两侧触点数量不同的完整基础包，便于发现左右映射错误。"""
    return PtsPacket(
        packet_counter=17,
        timestamp_us=123_000,
        pillar_forces=[np.array([[1.0, 2.0, 3.0]]), np.array([[4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])],
        pillar_displacements=[
            np.array([[0.1, -0.2, 0.3]]),
            np.array([[-0.4, 0.5, 0.6], [0.7, -0.8, 0.9]]),
        ],
        global_forces=[np.array([1.0, 2.0, 3.0]), np.array([11.0, 13.0, 15.0])],
        global_torques=[np.zeros(3), np.zeros(3)],
    )


def _snapshot(packet: PtsPacket) -> TactileSnapshot:
    """直接转换离线数据，不启动采集线程或打开设备。"""
    worker = TactileWorker(PapillArraySerialConfig(), clear_bias=False, clock=lambda: 0.5)
    return worker._snapshot_from_packet(packet, PacketIntegrityTracker(2).inspect(packet))


def test_missing_native_extensions_stay_absent() -> None:
    """未包含扩展的包保留缺失语义，不冒充检测器已关闭或摩擦为零。"""
    snapshot = _snapshot(_packet())
    record = asdict(snapshot)

    assert snapshot.native_slip_active == ()
    assert snapshot.native_reference_loaded == ()
    assert snapshot.native_pillar_states == ()
    assert snapshot.native_pillar_friction == ()
    assert snapshot.native_sensor_friction == ()
    assert snapshot.native_target_grip_force_n == ()
    assert json.loads(json.dumps(record, allow_nan=False))["native_pillar_friction"] == []


def test_snapshot_preserves_each_side_pillar_and_displacement() -> None:
    """所有原厂字段按包内侧序和触点序保存，且与可变 NumPy 缓冲区脱离。"""
    packet = replace(
        _packet(),
        slip_detection_active=[np.bool_(True), np.bool_(False)],
        reference_pillar_loaded=[np.bool_(False), np.bool_(True)],
        pillar_slip_states=[np.array([3], dtype=np.int8), np.array([1, -1], dtype=np.int8)],
        pillar_friction_estimates=[np.array([0.25]), np.array([0.5, 0.75])],
        sensor_friction_estimates=[np.float64(0.3), np.float64(0.6)],
        target_grip_forces=[np.float64(4.0), np.float64(8.0)],
    )

    snapshot = _snapshot(packet)
    packet.pillar_slip_states[0][0] = 0
    packet.pillar_friction_estimates[0][0] = 0.0
    packet.pillar_displacements[0][0, 0] = 99.0

    assert snapshot.native_slip_active == (True, False)
    assert snapshot.native_reference_loaded == (False, True)
    assert snapshot.native_pillar_states == ((3,), (1, -1))
    assert snapshot.native_pillar_friction == ((0.25,), (0.5, 0.75))
    assert snapshot.native_sensor_friction == (0.3, 0.6)
    assert snapshot.native_target_grip_force_n == (4.0, 8.0)
    assert snapshot.left_taxel_displacements_mm == ((0.1, -0.2, 0.3),)
    assert snapshot.right_taxel_displacements_mm == ((-0.4, 0.5, 0.6), (0.7, -0.8, 0.9))
    assert snapshot.left_taxel_forces_n == ((1.0, 2.0, 3.0),)
    assert snapshot.right_taxel_forces_n == ((4.0, 5.0, 6.0), (7.0, 8.0, 9.0))
    assert (snapshot.left_force_n, snapshot.right_force_n) == (3.0, 15.0)
    record = json.loads(json.dumps(asdict(snapshot), allow_nan=False))
    assert record["native_pillar_states"] == [[3], [1, -1]]
    assert record["left_taxel_displacements_mm"] == [[0.1, -0.2, 0.3]]


def test_nonfinite_native_estimates_are_json_null_without_losing_positions() -> None:
    """非有限摩擦和推荐力留空，仍保留对应侧与触点位置及有限原始值。"""
    packet = replace(
        _packet(),
        pillar_friction_estimates=[np.array([np.nan]), np.array([np.inf, -np.inf])],
        sensor_friction_estimates=[np.nan, -0.5],
        target_grip_forces=[np.inf, 0.0],
    )

    snapshot = _snapshot(packet)
    record = json.loads(json.dumps(asdict(snapshot), allow_nan=False))

    assert snapshot.native_pillar_friction == ((None,), (None, None))
    assert snapshot.native_sensor_friction == (None, -0.5)
    assert snapshot.native_target_grip_force_n == (None, 0.0)
    assert record["native_pillar_friction"] == [[None], [None, None]]
    assert record["native_sensor_friction"] == [None, -0.5]
    assert record["native_target_grip_force_n"] == [None, 0.0]

"""PapillArray 同步客户端的离线假串口测试。"""

from __future__ import annotations

import numpy as np
import pytest

from papillarray_hardware import PapillArraySerialClient, PapillArraySerialConfig, PtsPacket


class FakeSerial:
    """记录读写和关闭状态的内存串口。"""

    def __init__(self, read_chunks: list[bytes] | None = None, *, is_open: bool = True) -> None:
        """创建指定打开状态的 fake 串口。"""
        self.is_open = is_open
        self.read_chunks = list(read_chunks or [])
        self.writes: list[bytes] = []
        self.flush_count = 0
        self.close_count = 0

    def read(self, _size: int) -> bytes:
        """返回预置读取分片。"""
        return self.read_chunks.pop(0) if self.read_chunks else b""

    def write(self, data: bytes) -> int:
        """记录完整写入。"""
        self.writes.append(bytes(data))
        return len(data)

    def flush(self) -> None:
        """记录 flush 调用。"""
        self.flush_count += 1

    def close(self) -> None:
        """关闭 fake 串口。"""
        self.close_count += 1
        self.is_open = False


class FakePacketReader:
    """返回固定观测包的协议读取器替身。"""

    def __init__(self, packet: PtsPacket) -> None:
        """保存待返回的观测包。"""
        self._packet = packet

    def read_packet(self) -> PtsPacket:
        """返回固定观测包。"""
        return self._packet


def test_config_defaults_and_sampling_rate_validation_do_not_open_serial() -> None:
    """配置构造保持纯数据，且拒绝设备不支持的采样率。"""
    config = PapillArraySerialConfig()

    assert config.port == "/dev/ttyACM0"
    assert config.baud_rate == 115200
    assert config.sampling_rate == 500
    assert config.expected_sensors == 2
    assert config.packet_timeout_s == 3.0
    with pytest.raises(ValueError, match="采样率"):
        PapillArraySerialConfig(sampling_rate=333)
    with pytest.raises(ValueError, match="期望传感器数"):
        PapillArraySerialConfig(expected_sensors=0)
    with pytest.raises(ValueError, match="期望传感器数"):
        PapillArraySerialConfig(expected_sensors=True)
    with pytest.raises(ValueError, match="总等待"):
        PapillArraySerialConfig(packet_timeout_s=0)


def test_open_and_commands_are_explicit_and_use_expected_wire_values() -> None:
    """构造不触发工厂；设备命令只在调用方法时写入。"""
    serial_port = FakeSerial()
    calls: list[tuple[str, int, float]] = []

    def factory(port: str, baud_rate: int, timeout_s: float) -> FakeSerial:
        calls.append((port, baud_rate, timeout_s))
        return serial_port

    client = PapillArraySerialClient(PapillArraySerialConfig(), factory)
    assert calls == []
    assert not client.is_open

    client.open()
    client.configure_stream()
    client.clear_bias()
    client.start_slip_detection()
    client.stop_slip_detection()
    client.close()

    assert calls == [("/dev/ttyACM0", 115200, 1.0)]
    assert serial_port.writes == [b"f500\n", b"z\n", b"S\n", b"s\n"]
    assert serial_port.flush_count == 4
    assert serial_port.close_count == 1


def test_client_rejects_implicit_io_and_unopened_factory_result() -> None:
    """未显式打开时不允许读写，工厂返回关闭对象会被清理。"""
    client = PapillArraySerialClient(PapillArraySerialConfig(), lambda *_args: FakeSerial())
    with pytest.raises(RuntimeError, match="尚未打开"):
        client.configure_stream()

    closed = FakeSerial(is_open=False)
    client = PapillArraySerialClient(PapillArraySerialConfig(), lambda *_args: closed)
    with pytest.raises(RuntimeError, match="未打开"):
        client.open()
    assert closed.close_count == 1


def test_client_accepts_matching_one_and_two_sensor_packets() -> None:
    """一侧或双侧部署均只接受与显式配置相符的观测包。"""
    for expected_sensors in (1, 2):
        client = PapillArraySerialClient(
            PapillArraySerialConfig(expected_sensors=expected_sensors),
            lambda *_args: FakeSerial(),
        )
        client.open()
        client._reader = FakePacketReader(make_packet(expected_sensors))

        packet = client.read_packet()

        assert packet.n_sensors == expected_sensors
        client.close()


def test_client_rejects_packet_with_unexpected_sensor_count() -> None:
    """双侧部署收到单侧包时必须停止上送，不能悄然按单侧继续运行。"""
    client = PapillArraySerialClient(
        PapillArraySerialConfig(expected_sensors=2), lambda *_args: FakeSerial()
    )
    client.open()
    client._reader = FakePacketReader(make_packet(1))

    with pytest.raises(RuntimeError, match="期望 2，实际 1"):
        client.read_packet()

    client.close()


def make_packet(sensor_count: int) -> PtsPacket:
    """构造仅用于客户端数量互锁测试的最小观测包。"""
    force = np.empty((0, 3), dtype=np.float64)
    return PtsPacket(
        packet_counter=1,
        timestamp_us=2,
        pillar_forces=[force.copy() for _ in range(sensor_count)],
        pillar_displacements=[force.copy() for _ in range(sensor_count)],
        global_forces=[np.zeros(3, dtype=np.float64) for _ in range(sensor_count)],
        global_torques=[np.zeros(3, dtype=np.float64) for _ in range(sensor_count)],
    )

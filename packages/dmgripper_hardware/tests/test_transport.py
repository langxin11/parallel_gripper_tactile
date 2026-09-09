"""验证内存传输替身绝不访问真实设备。"""

from __future__ import annotations

import pytest

from dmgripper_hardware import FakeTransport, TransportClosedError


def test_fake_transport_records_writes_and_reads_injected_bytes() -> None:
    """打开后的 fake transport 仅保留内存副本并按请求长度取回字节。"""
    transport = FakeTransport()
    transport.open("/dev/never-opened", 921600)
    transport.inject_received(b"abcdef")

    assert transport.write(b"command") == 7
    assert transport.written_payloads == [b"command"]
    assert transport.read(2) == b"ab"
    assert transport.read(10) == b"cdef"
    assert transport.open_calls == [("/dev/never-opened", 921600)]


def test_fake_transport_rejects_closed_operations_without_device_access() -> None:
    """关闭状态的读写明确失败，避免测试把遗漏的打开流程当作真机操作。"""
    transport = FakeTransport()

    with pytest.raises(TransportClosedError):
        transport.write(b"ignored")
    with pytest.raises(TransportClosedError):
        transport.read(1)

    transport.open("ignored", 0)
    transport.close()
    with pytest.raises(TransportClosedError):
        transport.write(b"ignored")


def test_fake_transport_rejects_negative_read_length() -> None:
    """非法读取长度在内存层直接失败。"""
    transport = FakeTransport()
    transport.open("ignored", 0)

    with pytest.raises(ValueError, match="不能为负数"):
        transport.read(-1)

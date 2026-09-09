"""验证延迟打开的 PySerial 适配器不需要真实串口。"""

from __future__ import annotations

import pytest

from dmgripper_hardware import PySerialTransport, TransportClosedError


class FakeSerialPort:
    """只在内存中模拟最小 PySerial 对象。"""

    def __init__(self, reads: list[bytes] | None = None, *, is_open: bool = True) -> None:
        """创建可指定读队列和打开状态的替身。"""
        self.is_open = is_open
        self.timeout: float | None = None
        self.write_timeout: float | None = None
        self.reads = list(reads or [])
        self.writes: list[bytes] = []
        self.read_sizes: list[int] = []
        self.closed_count = 0
        self.raise_on_read: Exception | None = None

    def close(self) -> None:
        """记录关闭且变为关闭状态。"""
        self.closed_count += 1
        self.is_open = False

    def read(self, size: int = 1) -> bytes:
        """返回预置块或按要求抛出异常。"""
        self.read_sizes.append(size)
        if self.raise_on_read is not None:
            raise self.raise_on_read
        return self.reads.pop(0) if self.reads else b""

    def write(self, data: bytes) -> int:
        """记录写入并假装完整接受。"""
        self.writes.append(bytes(data))
        return len(data)


def test_constructor_does_not_call_injected_factory() -> None:
    """传输构造只保存工厂，默认不执行打开或设备访问。"""
    calls: list[tuple[str, int]] = []

    def factory(port: str, baud_rate: int) -> FakeSerialPort:
        calls.append((port, baud_rate))
        return FakeSerialPort()

    transport = PySerialTransport(factory)

    assert transport.is_open is False
    assert calls == []


def test_open_read_write_and_close_use_only_injected_serial() -> None:
    """显式生命周期把参数转给 fake serial，且读取临时恢复超时。"""
    calls: list[tuple[str, int]] = []
    serial_port = FakeSerialPort([b"response"])
    serial_port.timeout = 3.0

    def factory(port: str, baud_rate: int) -> FakeSerialPort:
        calls.append((port, baud_rate))
        return serial_port

    transport = PySerialTransport(factory)
    transport.open("fake://usb2can", 921600)

    assert transport.write(b"request", timeout_s=0.2) == 7
    assert transport.read(16, timeout_s=0.2) == b"response"
    assert calls == [("fake://usb2can", 921600)]
    assert serial_port.writes == [b"request"]
    assert serial_port.read_sizes == [16]
    assert serial_port.timeout == 3.0
    assert serial_port.write_timeout is None

    transport.close()
    assert transport.is_open is False
    assert serial_port.closed_count == 1


def test_read_restores_timeout_when_fake_serial_raises() -> None:
    """底层读取异常也不能把临时超时泄漏给下一次操作。"""
    serial_port = FakeSerialPort()
    serial_port.timeout = 4.0
    serial_port.raise_on_read = RuntimeError("模拟串口读取失败")
    transport = PySerialTransport(lambda _port, _baud_rate: serial_port)
    transport.open("fake://usb2can", 921600)

    with pytest.raises(RuntimeError, match="模拟串口"):
        transport.read(1, timeout_s=0.1)

    assert serial_port.timeout == 4.0


def test_write_restores_timeout_when_fake_serial_raises() -> None:
    """底层写入异常也不能把临时写超时泄漏给下一次操作。"""
    serial_port = FakeSerialPort()
    serial_port.write_timeout = 4.0
    transport = PySerialTransport(lambda _port, _baud_rate: serial_port)
    transport.open("fake://usb2can", 921600)

    def failing_write(_data: bytes) -> int:
        """模拟 PySerial 写入异常。"""
        raise RuntimeError("模拟串口写入失败")

    serial_port.write = failing_write  # type: ignore[method-assign]
    with pytest.raises(RuntimeError, match="模拟串口写入"):
        transport.write(b"x", timeout_s=0.1)

    assert serial_port.write_timeout == 4.0


def test_closed_and_invalid_operations_fail_without_factory_call() -> None:
    """关闭状态和无效参数不能退化为隐式真实串口访问。"""
    calls: list[tuple[str, int]] = []
    transport = PySerialTransport(lambda port, baud_rate: calls.append((port, baud_rate)))  # type: ignore[arg-type]

    with pytest.raises(TransportClosedError):
        transport.write(b"x")
    with pytest.raises(TransportClosedError):
        transport.read(1)
    with pytest.raises(ValueError, match="端点"):
        transport.open(" ", 921600)
    with pytest.raises(ValueError, match="波特率"):
        transport.open("fake://usb2can", 0)

    assert calls == []


def test_open_rejects_closed_factory_result_and_cleans_it_up() -> None:
    """工厂失败性返回不会被保留为可误用的传输状态。"""
    serial_port = FakeSerialPort(is_open=False)
    transport = PySerialTransport(lambda _port, _baud_rate: serial_port)

    with pytest.raises(RuntimeError, match="未打开"):
        transport.open("fake://usb2can", 921600)

    assert transport.is_open is False
    assert serial_port.closed_count == 1

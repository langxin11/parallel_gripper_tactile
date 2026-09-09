"""通过 PySerial 实现 USB2CAN 字节传输。"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Protocol, cast

from .transport import TransportClosedError


class _SerialPort(Protocol):
    """本模块实际需要的最小 PySerial 对象接口。"""

    @property
    def is_open(self) -> bool:
        """报告串口是否已打开。"""

    @property
    def timeout(self) -> float | None:
        """读取超时。"""

    @timeout.setter
    def timeout(self, value: float | None) -> None:
        """设置读取超时。"""

    @property
    def write_timeout(self) -> float | None:
        """写入超时。"""

    @write_timeout.setter
    def write_timeout(self, value: float | None) -> None:
        """设置写入超时。"""

    def close(self) -> None:
        """关闭串口。"""

    def read(self, size: int = 1) -> bytes:
        """读取至多指定数量的字节。"""

    def write(self, data: bytes) -> int:
        """写入字节并返回已接受字节数。"""


SerialFactory = Callable[[str, int], _SerialPort]
"""创建并打开串口对象的可注入工厂类型。"""


class PySerialTransport:
    """依赖注入、延迟打开的 PySerial 字节传输适配器。

    构造函数只保存工厂，不导入 PySerial、不开启端口。只有显式调用 `open` 才会由
    工厂创建串口；测试应传入内存 fake factory，因而不会访问 `/dev`。

    Args:
        serial_factory: 用端点和波特率创建已打开串口的工厂；省略时在 `open` 中延迟
            导入 PySerial 并创建 `serial.Serial`。
    """

    def __init__(self, serial_factory: SerialFactory | None = None) -> None:
        """创建关闭状态的传输，不执行 I/O。"""
        self._serial_factory = serial_factory
        self._serial: _SerialPort | None = None

    @property
    def is_open(self) -> bool:
        """报告底层串口是否存在且已打开。"""
        return self._serial is not None and self._serial.is_open

    def open(self, port: str, baud_rate: int) -> None:
        """显式创建并打开指定端点。

        Args:
            port: 传给 PySerial 的串口端点。
            baud_rate: 传给 PySerial 的正整数波特率。

        Raises:
            RuntimeError: 传输已经打开。
            ValueError: 端点或波特率无效。
            ModuleNotFoundError: 未安装 PySerial 且未注入工厂。
        """
        if self.is_open:
            raise RuntimeError("串口传输已经打开")
        if not isinstance(port, str) or not port or port != port.strip():
            raise ValueError("串口端点必须是非空且无首尾空白的字符串")
        if isinstance(baud_rate, bool) or not isinstance(baud_rate, int) or baud_rate <= 0:
            raise ValueError("波特率必须是正整数")

        factory = self._serial_factory or _default_serial_factory
        serial_port = factory(port, baud_rate)
        if not serial_port.is_open:
            try:
                serial_port.close()
            finally:
                raise RuntimeError("串口工厂返回了未打开的串口")
        self._serial = serial_port

    def close(self) -> None:
        """关闭底层串口；重复关闭安全且不执行隐式打开。"""
        serial_port = self._serial
        self._serial = None
        if serial_port is not None and serial_port.is_open:
            serial_port.close()

    def write(self, payload: bytes, timeout_s: float | None = None) -> int:
        """在临时写入时限内写入一个字节序列并恢复底层设置。"""
        if timeout_s is not None:
            _validate_timeout(timeout_s)
        serial_port = self._require_open()
        if timeout_s is None:
            return serial_port.write(bytes(payload))
        previous_timeout = serial_port.write_timeout
        serial_port.write_timeout = timeout_s
        try:
            return serial_port.write(bytes(payload))
        finally:
            serial_port.write_timeout = previous_timeout

    def read(self, max_bytes: int, timeout_s: float | None = None) -> bytes:
        """在临时超时内读取至多指定数量的字节。

        传入超时时仅在本次读取期间修改底层 `timeout`，无论读取成败都会恢复原值。

        Args:
            max_bytes: 本次最多读取的字节数。
            timeout_s: 本次读取超时；`None` 保留底层当前设置。
        """
        if isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or max_bytes < 0:
            raise ValueError("最大读取长度不能为负整数")
        if timeout_s is not None:
            _validate_timeout(timeout_s)
        serial_port = self._require_open()
        if timeout_s is None:
            return bytes(serial_port.read(max_bytes))

        previous_timeout = serial_port.timeout
        serial_port.timeout = timeout_s
        try:
            return bytes(serial_port.read(max_bytes))
        finally:
            serial_port.timeout = previous_timeout

    def _require_open(self) -> _SerialPort:
        """返回打开的串口，或拒绝隐式 I/O。"""
        if not self.is_open:
            raise TransportClosedError("串口传输尚未打开")
        return cast("_SerialPort", self._serial)


def _default_serial_factory(port: str, baud_rate: int) -> _SerialPort:
    """按需导入 PySerial，并在显式 `open` 时创建串口。"""
    import serial

    return cast("_SerialPort", serial.Serial(port=port, baudrate=baud_rate, timeout=None))


def _validate_timeout(timeout_s: float) -> None:
    """验证单次底层读取的有限非负超时。"""
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
        raise ValueError("读取超时必须是有限非负数")
    if not math.isfinite(timeout_s) or timeout_s < 0.0:
        raise ValueError("读取超时必须是有限非负数")

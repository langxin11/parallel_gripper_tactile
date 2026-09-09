"""DMgripper 硬件层的最小传输抽象与内存替身。"""

from __future__ import annotations

from typing import Protocol, runtime_checkable


class TransportClosedError(RuntimeError):
    """对未打开的传输读写时抛出的异常。"""


@runtime_checkable
class ByteTransport(Protocol):
    """供未来 USB2CAN 串口适配器实现的字节传输接口。"""

    @property
    def is_open(self) -> bool:
        """报告传输是否已打开。"""

    def open(self, port: str, baud_rate: int) -> None:
        """打开传输。

        Args:
            port: 设备地址；具体含义由实现决定。
            baud_rate: 波特率；具体含义由实现决定。
        """

    def close(self) -> None:
        """关闭传输。"""

    def write(self, payload: bytes, timeout_s: float | None = None) -> int:
        """在可选有界时限内写入字节并返回已接受的字节数。"""

    def read(self, max_bytes: int, timeout_s: float | None = None) -> bytes:
        """读取至多 `max_bytes` 个字节。"""


class FakeTransport:
    """仅在内存中记录写入并提供可注入接收字节的传输替身。

    本类没有文件描述符、串口或 USB 依赖；`write` 只复制到 `written_payloads`，
    因此适合协议与上层控制循环的离线测试。
    """

    def __init__(self) -> None:
        """创建关闭状态的空内存传输。"""
        self._is_open = False
        self._received = bytearray()
        self.written_payloads: list[bytes] = []
        self.open_calls: list[tuple[str, int]] = []

    @property
    def is_open(self) -> bool:
        """报告该内存传输是否已打开。"""
        return self._is_open

    def open(self, port: str, baud_rate: int) -> None:
        """记录打开参数并使内存传输可用。"""
        self._is_open = True
        self.open_calls.append((port, baud_rate))

    def close(self) -> None:
        """关闭内存传输，但保留已记录数据供断言。"""
        self._is_open = False

    def write(self, payload: bytes, timeout_s: float | None = None) -> int:
        """记录一份写入副本，不会访问任何真实设备或等待。"""
        del timeout_s
        self._require_open()
        copied = bytes(payload)
        self.written_payloads.append(copied)
        return len(copied)

    def read(self, max_bytes: int, timeout_s: float | None = None) -> bytes:
        """取出最多指定数量的已注入接收字节。

        `timeout_s` 只为匹配未来真实传输接口而保留；内存替身不等待。
        """
        del timeout_s
        self._require_open()
        if max_bytes < 0:
            raise ValueError("最大读取长度不能为负数")
        result = bytes(self._received[:max_bytes])
        del self._received[:max_bytes]
        return result

    def inject_received(self, payload: bytes) -> None:
        """向下次 `read` 注入接收字节。

        Args:
            payload: 要追加到接收队列的原始字节。
        """
        self._received.extend(payload)

    def _require_open(self) -> None:
        """保证调用发生在打开状态。"""
        if not self._is_open:
            raise TransportClosedError("传输尚未打开")

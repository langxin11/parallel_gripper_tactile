"""PapillArray 的显式生命周期同步串口客户端。"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol, cast

from .protocol import PtsPacket, PtsStreamReader

SUPPORTED_SAMPLING_RATES = frozenset((100, 250, 500, 1000))
"""PTS 控制器已知支持的采样率，单位 Hz。"""

DEFAULT_PAPILLARRAY_PORT = "/dev/papillarray"
"""本工作区 PapillArray 控制器的默认 udev 串口别名。"""

_CLEAR_BIAS_COMMAND = b"z\n"
_START_SLIP_COMMAND = b"S\n"
_STOP_SLIP_COMMAND = b"s\n"


class _SerialPort(Protocol):
    """客户端实际需要的最小 PySerial 对象接口。"""

    @property
    def is_open(self) -> bool:
        """报告串口是否已打开。"""

    def close(self) -> None:
        """关闭串口。"""

    def flush(self) -> None:
        """等待已写字节发送完成。"""

    def read(self, size: int) -> bytes:
        """读取至多指定数量的字节。"""

    def reset_input_buffer(self) -> None:
        """丢弃操作系统串口接收队列中尚未读取的字节。"""

    def write(self, data: bytes) -> int:
        """写入字节并返回接受的字节数。"""


SerialFactory = Callable[[str, int, float], _SerialPort]
"""创建已打开串口的可注入工厂。"""


@dataclass(frozen=True)
class PapillArraySerialConfig:
    """PapillArray 串口连接及协议读取配置。

    Args:
        port: 操作系统串口端点。
        baud_rate: 串口波特率，单位 baud。
        sampling_rate: 请求控制器输出的频率，单位 Hz。
        expected_sensors: 此部署应接入的传感器数。
        timeout_s: 单次底层读取超时，单位 s。
        packet_timeout_s: 单次等待有效 PTS 包的总单调时钟时限，单位 s。
        max_packet_bytes: 含起止标志的 PTS 单帧最大长度。
    """

    port: str = DEFAULT_PAPILLARRAY_PORT
    baud_rate: int = 115200
    sampling_rate: int = 1000
    expected_sensors: int = 2
    timeout_s: float = 1.0
    packet_timeout_s: float = 3.0
    max_packet_bytes: int = 8192

    def __post_init__(self) -> None:
        """在纯配置阶段校验值，但不访问串口。"""
        if not isinstance(self.port, str) or not self.port or self.port != self.port.strip():
            raise ValueError("串口端点必须是非空且无首尾空白的字符串")
        if (
            isinstance(self.baud_rate, bool)
            or not isinstance(self.baud_rate, int)
            or self.baud_rate <= 0
        ):
            raise ValueError("波特率必须是正整数")
        if self.sampling_rate not in SUPPORTED_SAMPLING_RATES:
            values = "、".join(str(value) for value in sorted(SUPPORTED_SAMPLING_RATES))
            raise ValueError(f"采样率必须是 {values} Hz 之一")
        if (
            isinstance(self.expected_sensors, bool)
            or not isinstance(self.expected_sensors, int)
            or self.expected_sensors <= 0
        ):
            raise ValueError("期望传感器数必须是正整数")
        if isinstance(self.timeout_s, bool) or not isinstance(self.timeout_s, (int, float)):
            raise ValueError("串口超时必须是有限正数")
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("串口超时必须是有限正数")
        if isinstance(self.packet_timeout_s, bool) or not isinstance(
            self.packet_timeout_s, (int, float)
        ):
            raise ValueError("单包总等待时限必须是有限正数")
        if not math.isfinite(self.packet_timeout_s) or self.packet_timeout_s <= 0:
            raise ValueError("单包总等待时限必须是有限正数")
        if self.packet_timeout_s < self.timeout_s:
            raise ValueError("单包总等待时限不得小于串口超时")
        if isinstance(self.max_packet_bytes, bool) or not isinstance(self.max_packet_bytes, int):
            raise ValueError("单帧最大长度必须是正整数")
        if self.max_packet_bytes < 11:
            raise ValueError("单帧最大长度小于 PTS 最小帧")


class PapillArraySerialClient:
    """显式打开、配置和读取 PapillArray 的同步客户端。

    构造不执行 I/O。调用 `open()` 后，调用方必须显式调用 `configure_stream()` 才会写入采样率
    命令。这一边界使设备命令在实机联调中可见且可审计。

    Args:
        config: 串口与采样配置。
        serial_factory: 可注入的串口工厂；省略时仅在 `open()` 内延迟导入 PySerial。
    """

    def __init__(
        self,
        config: PapillArraySerialConfig,
        serial_factory: SerialFactory | None = None,
    ) -> None:
        """保存配置与工厂而不打开串口。"""
        self._config = config
        self._serial_factory = serial_factory
        self._serial_port: _SerialPort | None = None
        self._reader: PtsStreamReader | None = None

    @property
    def is_open(self) -> bool:
        """报告是否持有已打开的底层串口。"""
        return self._serial_port is not None and self._serial_port.is_open

    def open(self) -> None:
        """显式创建并打开串口，不发送设备命令。

        Raises:
            RuntimeError: 串口已经打开，或工厂返回未打开的对象。
            ModuleNotFoundError: 未注入工厂且当前环境未安装 PySerial。
        """
        if self.is_open:
            raise RuntimeError("PapillArray 串口已经打开")
        factory = self._serial_factory or _default_serial_factory
        serial_port = factory(self._config.port, self._config.baud_rate, self._config.timeout_s)
        if not serial_port.is_open:
            try:
                serial_port.close()
            finally:
                raise RuntimeError("串口工厂返回了未打开的对象")
        self._serial_port = serial_port
        self._reader = PtsStreamReader(
            serial_port,
            self._config.max_packet_bytes,
            packet_timeout_s=self._config.packet_timeout_s,
        )

    def close(self) -> None:
        """关闭串口并释放协议读取状态；重复关闭安全。"""
        serial_port = self._serial_port
        self._serial_port = None
        self._reader = None
        if serial_port is not None and serial_port.is_open:
            serial_port.close()

    def configure_stream(self) -> None:
        """向控制器发送配置中的采样率命令。"""
        self._write_command(f"f{self._config.sampling_rate}\n".encode("ascii"))

    def clear_bias(self) -> None:
        """请求设备执行清零／偏置清除，并丢弃当前半包。

        调用前必须让传感器完全无负载；本方法不会自行等待或判断无负载条件。
        """
        self._write_command(_CLEAR_BIAS_COMMAND)
        self._require_open().reset_input_buffer()
        reader = self._require_reader()
        reader.reset_buffer()

    def start_slip_detection(self) -> None:
        """请求设备开始滑动检测。"""
        self._write_command(_START_SLIP_COMMAND)

    def stop_slip_detection(self) -> None:
        """请求设备停止滑动检测。"""
        self._write_command(_STOP_SLIP_COMMAND)

    def read_packet(self) -> PtsPacket:
        """读取下一个校验通过且数量符合部署配置的触觉观测包。

        Raises:
            RuntimeError: 设备报告的传感器数与部署配置不一致。
        """
        packet = self._require_reader().read_packet()
        if packet.n_sensors != self._config.expected_sensors:
            raise RuntimeError(
                "PapillArray 传感器数与部署配置不一致："
                f"期望 {self._config.expected_sensors}，实际 {packet.n_sensors}"
            )
        return packet

    def __enter__(self) -> PapillArraySerialClient:
        """打开客户端并返回自身。"""
        self.open()
        return self

    def __exit__(self, _exc_type: object, _exc_value: object, _traceback: object) -> None:
        """无论上下文如何退出都关闭串口。"""
        self.close()

    def _write_command(self, payload: bytes) -> None:
        """完整写入并 flush 一个明确的设备控制命令。"""
        serial_port = self._require_open()
        written = serial_port.write(payload)
        if written != len(payload):
            raise OSError(f"PapillArray 串口仅写入 {written}/{len(payload)} 字节")
        serial_port.flush()

    def _require_open(self) -> _SerialPort:
        """返回打开的串口，拒绝隐式 I/O。"""
        if not self.is_open:
            raise RuntimeError("PapillArray 串口尚未打开")
        return cast("_SerialPort", self._serial_port)

    def _require_reader(self) -> PtsStreamReader:
        """返回打开串口绑定的协议读取器。"""
        self._require_open()
        if self._reader is None:
            raise RuntimeError("PapillArray 协议读取器尚未创建")
        return self._reader


def _default_serial_factory(port: str, baud_rate: int, timeout_s: float) -> _SerialPort:
    """按需导入 PySerial 并创建已打开串口。"""
    import serial

    return cast(
        "_SerialPort",
        serial.Serial(port=port, baudrate=baud_rate, timeout=timeout_s),
    )

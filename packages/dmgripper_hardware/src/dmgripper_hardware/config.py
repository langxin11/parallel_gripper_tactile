"""USB2CAN 只读状态刷新使用的设备配置。"""

from __future__ import annotations

import math
from dataclasses import dataclass


DEFAULT_USB2CAN_BAUD_RATE = 921600
"""参考 USB2CAN 串口波特率。"""

DEFAULT_USB2CAN_PORT = "/dev/dmj4310_can"
"""本工作区 DM4310P USB2CAN 的默认 udev 串口别名。"""


@dataclass(frozen=True, slots=True)
class Usb2CanDeviceConfig:
    """一台 DM 电机的 USB2CAN 只读连接配置。

    配置对象只保存数据，创建它不会打开串口。电机与主机 ID 必须是不同的非零
    16 位整数；量程不在此处推断，必须由调用方构造 `Usb2CanProtocol` 时显式提供。

    Args:
        port: 串口设备路径或由传输实现解释的端点名称。
        motor_id: 目标电机 CAN ID。
        master_id: 本机／主机 CAN ID。
        timeout_s: 单次状态刷新可等待反馈的总时长（秒）。
        baud_rate: 串口波特率，默认使用 USB2CAN 常用的 921600。
    """

    port: str
    motor_id: int
    master_id: int
    timeout_s: float
    baud_rate: int = DEFAULT_USB2CAN_BAUD_RATE

    def __post_init__(self) -> None:
        """拒绝可能误连设备或无限等待的配置。"""
        if not isinstance(self.port, str) or not self.port or self.port != self.port.strip():
            raise ValueError("串口端点必须是非空且无首尾空白的字符串")
        if "\x00" in self.port:
            raise ValueError("串口端点不能包含空字节")
        _validate_positive_uint(self.baud_rate, bits=32, name="波特率")
        _validate_positive_uint(self.motor_id, bits=16, name="电机 CAN ID")
        _validate_positive_uint(self.master_id, bits=16, name="主机 CAN ID")
        if self.motor_id == self.master_id:
            raise ValueError("电机 CAN ID 与主机 CAN ID 必须不同")
        if isinstance(self.timeout_s, bool) or not isinstance(self.timeout_s, (int, float)):
            raise ValueError("状态刷新超时必须是有限正数")
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0:
            raise ValueError("状态刷新超时必须是有限正数")


def _validate_positive_uint(value: int, *, bits: int, name: str) -> None:
    """验证可映射为非零无符号协议整数的配置字段。"""
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value < (1 << bits):
        raise ValueError(f"{name}必须位于 [1, {(1 << bits) - 1}]")

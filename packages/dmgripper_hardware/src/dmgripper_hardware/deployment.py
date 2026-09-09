"""DM4310P 平行夹爪的用户提供部署参数。

本模块把电机协议量程与夹爪机械关节行程明确分开。配置只保存数据，构造和工厂
函数均不会打开串口或执行其他设备 I/O。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import DEFAULT_USB2CAN_BAUD_RATE, Usb2CanDeviceConfig
from .protocol import MotorLimits


@dataclass(frozen=True, slots=True)
class Dm4310PGripperConfig:
    """DM4310P 平行夹爪的纯数据部署配置。

    `motor_limits` 描述 DM 固件协议进行 MIT 定点编码时使用的电机量程；
    `joint_position_*` 描述当前夹爪机构允许的关节角行程，二者不能混用。
    机械关节角从 0 到 ``pi / 2`` 增大时表示夹爪闭合，因此
    `closing_direction` 固定为 ``1``。

    Args:
        device: USB2CAN 连接与 CAN ID 配置。
        motor_limits: DM4310P 协议位置、速度和力矩量程。
        joint_position_min_rad: 机械关节角下界（rad）。
        joint_position_max_rad: 机械关节角上界（rad）。
        closing_direction: 角度增大方向是否为闭合方向，必须为 ``1`` 或 ``-1``。

    Raises:
        ValueError: 配置含非有限数值、退化边界、非法方向，或机械行程超出协议位置量程。
    """

    device: Usb2CanDeviceConfig
    motor_limits: MotorLimits
    joint_position_min_rad: float = 0.0
    joint_position_max_rad: float = math.pi / 2.0
    closing_direction: int = 1

    def __post_init__(self) -> None:
        """验证机械行程与协议量程的关系。"""
        for name, value in (
            ("机械关节角下界", self.joint_position_min_rad),
            ("机械关节角上界", self.joint_position_max_rad),
        ):
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ValueError(f"{name}必须是有限数值")
        if self.joint_position_min_rad >= self.joint_position_max_rad:
            raise ValueError("机械关节角下界必须小于上界")
        if not isinstance(self.closing_direction, int) or isinstance(self.closing_direction, bool):
            raise ValueError("闭合方向必须为 1 或 -1")
        if self.closing_direction not in (-1, 1):
            raise ValueError("闭合方向必须为 1 或 -1")
        if self.joint_position_min_rad < self.motor_limits.position_min_rad:
            raise ValueError("机械关节角下界超出电机协议位置量程")
        if self.joint_position_max_rad > self.motor_limits.position_max_rad:
            raise ValueError("机械关节角上界超出电机协议位置量程")

    @property
    def motor_id(self) -> int:
        """返回目标电机 CAN ID。"""
        return self.device.motor_id

    @property
    def master_id(self) -> int:
        """返回主机 CAN ID。"""
        return self.device.master_id

    def validate_joint_position(self, position_rad: float) -> float:
        """验证机械关节目标角，不做静默截断。

        Args:
            position_rad: 待发送的机械关节目标角（rad）。

        Returns:
            原样返回通过验证的目标角，便于在命令构造中显式传递。

        Raises:
            ValueError: 目标角不是有限数值或超出机械行程。
        """
        if (
            isinstance(position_rad, bool)
            or not isinstance(position_rad, (int, float))
            or not math.isfinite(position_rad)
        ):
            raise ValueError("机械关节目标角必须是有限数值")
        if not self.joint_position_min_rad <= position_rad <= self.joint_position_max_rad:
            raise ValueError(
                "机械关节目标角必须位于 "
                f"[{self.joint_position_min_rad}, {self.joint_position_max_rad}] rad"
            )
        return float(position_rad)


def make_dm4310p_gripper_config(
    port: str,
    *,
    timeout_s: float = 0.05,
    baud_rate: int = DEFAULT_USB2CAN_BAUD_RATE,
) -> Dm4310PGripperConfig:
    """创建当前 DM4310P 平行夹爪的无 I/O 部署配置。

    Args:
        port: 用户显式提供的 USB2CAN 串口端点。
        timeout_s: 单次状态刷新超时（秒）。
        baud_rate: USB2CAN 串口波特率。

    Returns:
        带有用户端点、固定 CAN ID、协议量程和机械行程的冻结配置。
    """
    device = Usb2CanDeviceConfig(
        port,
        motor_id=1,
        master_id=17,
        timeout_s=timeout_s,
        baud_rate=baud_rate,
    )
    return Dm4310PGripperConfig(
        device=device,
        motor_limits=MotorLimits(
            position_min_rad=-1.7,
            position_max_rad=1.7,
            velocity_min_rad_s=-8.0,
            velocity_max_rad_s=8.0,
            torque_min_nm=-4.0,
            torque_max_nm=4.0,
        ),
    )

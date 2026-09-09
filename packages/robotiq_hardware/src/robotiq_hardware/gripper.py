"""Robotiq 2F85 的设备命令端口和输入校验。"""

from __future__ import annotations

from dataclasses import dataclass
from numbers import Integral
from typing import Protocol


MIN_POSITION = 0
MAX_POSITION = 255


class RobotiqPositionBackend(Protocol):
    """接收一个已校验 Robotiq 位置命令的后端协议。"""

    def send_position(self, position: int) -> None:
        """向底层设备发送一个 0–255 的位置命令。"""


class PyRobotiqGripper3312Like(Protocol):
    """`pyrobotiqgripper` 3.3.12 的已连接、已激活对象所需的最小接口。"""

    def move(
        self,
        position: int,
        *,
        speed: int,
        force: int,
        wait: bool,
        readStatus: bool,
        refreshStatus: bool,
        start: bool,
    ) -> object:
        """执行 3.3.12 版本的位置命令。"""


@dataclass(frozen=True, slots=True)
class PyRobotiqGripper3312Config:
    """3.3.12 版本运动调用的固定参数。"""

    speed: int = MAX_POSITION
    force: int = MAX_POSITION
    read_status: bool = False

    def __post_init__(self) -> None:
        """校验速度、力和状态读取策略。"""
        validate_position_command(self.speed)
        validate_position_command(self.force)
        if not isinstance(self.read_status, bool):
            raise TypeError("read_status must be bool")


def validate_position_command(position: object) -> int:
    """校验并规范化一个 Robotiq 位置命令。

    Args:
        position: 待发送的位置命令。

    Returns:
        与输入数值相等的内置 `int`。

    Raises:
        TypeError: 输入不是整数，或输入是布尔值。
        ValueError: 整数不在 0–255 范围内。
    """
    if isinstance(position, bool) or not isinstance(position, Integral):
        raise TypeError("position must be an integer, not bool")
    value = int(position)
    if not MIN_POSITION <= value <= MAX_POSITION:
        raise ValueError(f"position must lie in [{MIN_POSITION}, {MAX_POSITION}]")
    return value


class Robotiq2F85Hardware:
    """通过依赖注入后端控制 Robotiq 2F85。

    本类不负责发现、打开或配置串口。只有调用 `move`、`open` 或 `close` 时，
    才会调用注入后端的 `send_position` 方法。
    """

    def __init__(self, backend: RobotiqPositionBackend) -> None:
        """创建一个不自动连接设备的 Robotiq 2F85 适配器。

        Args:
            backend: 实际传输层实现，需提供 `send_position` 方法。
        """
        if not callable(getattr(backend, "send_position", None)):
            raise TypeError("backend must provide a callable send_position method")
        self._backend = backend

    def move(self, position: object) -> int:
        """发送一个经过严格校验的 0–255 位置命令。

        Args:
            position: 目标位置，只接受整数 0–255。

        Returns:
            实际交给后端的内置 `int` 命令。

        Raises:
            TypeError: 输入不是整数，或输入是布尔值。
            ValueError: 整数不在 0–255 范围内。
        """
        command = validate_position_command(position)
        self._backend.send_position(command)
        return command

    def open(self) -> int:
        """发送完全打开命令 0。"""
        return self.move(MIN_POSITION)

    def close(self) -> int:
        """发送完全闭合命令 255。"""
        return self.move(MAX_POSITION)


class PyRobotiqGripper3312Backend:
    """把 3.3.12 的已连接对象接入本包的位置后端协议。

    本适配器不发现、连接或激活设备。调用方必须先完成这些生命周期步骤，
    并传入一个提供 3.3.12 `move` 方法的对象。
    """

    def __init__(
        self,
        gripper: PyRobotiqGripper3312Like,
        *,
        speed: object = MAX_POSITION,
        force: object = MAX_POSITION,
        read_status: bool = False,
    ) -> None:
        """创建非阻塞的 3.3.12 后端适配器。

        Args:
            gripper: 调用方已连接且已激活的 3.3.12 gripper 对象。
            speed: 传给 `move` 的速度，必须是 0–255 整数。
            force: 传给 `move` 的力，必须是 0–255 整数。
            read_status: 是否请求 `readStatus`。默认为 `False`，避免控制周期
                因附加状态读取引入阻塞；需要状态读取时必须显式设为 `True`。
        """
        if not callable(getattr(gripper, "move", None)):
            raise TypeError("gripper must provide a callable move method")
        self._gripper = gripper
        self.config = PyRobotiqGripper3312Config(
            speed=validate_position_command(speed),
            force=validate_position_command(force),
            read_status=read_status,
        )

    def send_position(self, position: int) -> None:
        """以 3.3.12 的非阻塞参数发送一个位置命令。

        Args:
            position: 目标位置，必须是 0–255 整数。
        """
        command = validate_position_command(position)
        self._gripper.move(
            command,
            speed=self.config.speed,
            force=self.config.force,
            wait=False,
            readStatus=self.config.read_status,
            refreshStatus=False,
            start=False,
        )

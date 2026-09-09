"""Robotiq 2F85 适配器的 fake backend 测试。"""

from dataclasses import dataclass, field

import pytest

from robotiq_hardware import (
    PyRobotiqGripper3312Backend,
    PyRobotiqGripper3312Config,
    Robotiq2F85Hardware,
    validate_position_command,
)


@dataclass
class FakeBackend:
    """记录命令但不连接真实设备的后端。"""

    commands: list[int] = field(default_factory=list)

    def send_position(self, position: int) -> None:
        """记录命令，不执行设备 I/O。"""
        self.commands.append(position)


@dataclass
class FakePyRobotiqGripper:
    """记录 3.3.12 风格调用但不执行设备 I/O 的 fake 对象。"""

    calls: list[tuple[int, dict[str, object]]] = field(default_factory=list)

    def move(self, position: int, **kwargs: object) -> None:
        """记录位置和关键字参数。"""
        self.calls.append((position, kwargs))


def test_move_forwards_valid_integer_without_real_io() -> None:
    """验证合法位置会原样交给 fake backend。"""
    backend = FakeBackend()
    gripper = Robotiq2F85Hardware(backend)

    assert gripper.move(128) == 128
    assert backend.commands == [128]


def test_open_and_close_use_the_endpoints() -> None:
    """验证打开和闭合使用两个端点命令。"""
    backend = FakeBackend()
    gripper = Robotiq2F85Hardware(backend)

    assert gripper.open() == 0
    assert gripper.close() == 255
    assert backend.commands == [0, 255]


@pytest.mark.parametrize("position", [-1, 256, 1.0, "128", True, False])
def test_invalid_position_is_rejected_without_backend_call(position: object) -> None:
    """验证非法命令会拒绝且不触发后端。"""
    backend = FakeBackend()
    gripper = Robotiq2F85Hardware(backend)

    expected = TypeError if isinstance(position, (float, str, bool)) else ValueError
    with pytest.raises(expected):
        gripper.move(position)
    assert backend.commands == []


def test_validator_returns_builtin_int() -> None:
    """验证校验器返回内置整数类型。"""
    command = validate_position_command(255)

    assert type(command) is int


def test_backend_requires_position_sink() -> None:
    """验证适配器拒绝不完整的后端。"""
    with pytest.raises(TypeError, match="send_position"):
        Robotiq2F85Hardware(object())


def test_pyrobotiq3312_backend_uses_nonblocking_move() -> None:
    """验证 3.3.12 后端传入固定的非阻塞调用参数。"""
    gripper = FakePyRobotiqGripper()
    backend = PyRobotiqGripper3312Backend(gripper, speed=100, force=200)

    backend.send_position(128)

    assert gripper.calls == [
        (
            128,
            {
                "speed": 100,
                "force": 200,
                "wait": False,
                "readStatus": False,
                "refreshStatus": False,
                "start": False,
            },
        )
    ]


def test_pyrobotiq3312_backend_can_explicitly_read_status() -> None:
    """验证状态读取必须通过显式配置开启。"""
    gripper = FakePyRobotiqGripper()
    backend = PyRobotiqGripper3312Backend(gripper, read_status=True)

    backend.send_position(0)

    assert gripper.calls[0][1]["readStatus"] is True
    assert gripper.calls[0][1]["wait"] is False


@pytest.mark.parametrize("field", ["speed", "force"])
@pytest.mark.parametrize("value", [-1, 256, 1.0, True, False])
def test_pyrobotiq3312_config_rejects_invalid_command_settings(field: str, value: object) -> None:
    """验证速度和力严格遵循 0–255 整数约束。"""
    kwargs = {field: value}

    expected = TypeError if isinstance(value, (float, bool)) else ValueError
    with pytest.raises(expected):
        PyRobotiqGripper3312Config(**kwargs)


def test_pyrobotiq3312_backend_requires_injected_gripper() -> None:
    """验证后端不会替调用方发现或创建设备对象。"""
    with pytest.raises(TypeError, match="move"):
        PyRobotiqGripper3312Backend(object())

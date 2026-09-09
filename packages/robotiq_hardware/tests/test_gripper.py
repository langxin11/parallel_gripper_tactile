"""Robotiq 2F85 适配器的 fake backend 测试。"""

from dataclasses import dataclass, field

import pytest

from robotiq_hardware import (
    RobotiqCommandReceipt,
    PyRobotiqGripper3312Backend,
    PyRobotiqGripper3312Config,
    Robotiq2F85Hardware,
    validate_position_command,
)


@dataclass
class FakeBackend:
    """记录命令但不连接真实设备的后端。"""

    commands: list[int] = field(default_factory=list)
    feedback: object = 0
    reads: int = 0

    def send_position(self, position: int) -> None:
        """记录命令，不执行设备 I/O。"""
        self.commands.append(position)

    def read_position(self) -> object:
        """返回预置反馈，不执行设备 I/O。"""
        self.reads += 1
        return self.feedback


@dataclass
class FakePyRobotiqGripper:
    """记录 3.3.12 风格调用但不执行设备 I/O 的 fake 对象。"""

    calls: list[tuple[int, dict[str, object]]] = field(default_factory=list)
    feedback: object = 0
    position_calls: int = 0

    def move(self, position: int, **kwargs: object) -> None:
        """记录位置和关键字参数。"""
        self.calls.append((position, kwargs))

    def position(self) -> object:
        """返回预置反馈，不执行设备 I/O。"""
        self.position_calls += 1
        return self.feedback


def test_move_forwards_valid_integer_without_real_io() -> None:
    """验证合法位置会原样交给 fake backend。"""
    backend = FakeBackend()
    gripper = Robotiq2F85Hardware(backend)

    receipt = gripper.move(128)

    assert receipt == RobotiqCommandReceipt(requested_position=128)
    assert receipt.requested_position == 128
    assert backend.commands == [128]


def test_open_and_close_use_the_endpoints() -> None:
    """验证打开和闭合使用两个端点命令。"""
    backend = FakeBackend()
    gripper = Robotiq2F85Hardware(backend)

    assert gripper.open().requested_position == 0
    assert gripper.close().requested_position == 255
    assert backend.commands == [0, 255]


def test_command_receipt_is_immutable() -> None:
    """验证命令收据是不可变的最小记录。"""
    receipt = RobotiqCommandReceipt(requested_position=128)

    with pytest.raises(AttributeError):
        receipt.requested_position = 129


def test_read_position_uses_only_feedback_interface() -> None:
    """验证 facade 读取反馈时不发送位置命令。"""
    backend = FakeBackend(feedback=127)
    gripper = Robotiq2F85Hardware(backend)

    assert gripper.read_position() == 127
    assert backend.reads == 1
    assert backend.commands == []


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


def test_backend_requires_position_feedback_source() -> None:
    """验证 facade 不接受缺少反馈读取接口的后端。"""

    class SendOnlyBackend:
        def send_position(self, position: int) -> None:
            pass

    with pytest.raises(TypeError, match="read_position"):
        Robotiq2F85Hardware(SendOnlyBackend())


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


@pytest.mark.parametrize("feedback", [-1, 256, 1.0, "128", True, False])
def test_pyrobotiq3312_backend_rejects_invalid_position_feedback(feedback: object) -> None:
    """验证 3.3.12 的非法位置反馈会被拒绝。"""
    gripper = FakePyRobotiqGripper(feedback=feedback)
    backend = PyRobotiqGripper3312Backend(gripper)

    expected = TypeError if isinstance(feedback, (float, str, bool)) else ValueError
    with pytest.raises(expected):
        backend.read_position()
    assert gripper.position_calls == 1
    assert gripper.calls == []


def test_pyrobotiq3312_backend_reads_position_without_sending_command() -> None:
    """验证后端反馈读取只调用 `position()`。"""
    gripper = FakePyRobotiqGripper(feedback=42)
    backend = PyRobotiqGripper3312Backend(gripper)

    assert backend.read_position() == 42
    assert gripper.position_calls == 1
    assert gripper.calls == []


def test_pyrobotiq3312_backend_transparently_propagates_position_error() -> None:
    """验证底层位置读取异常不会被适配器吞掉。"""

    class FailingGripper(FakePyRobotiqGripper):
        def position(self) -> object:
            raise RuntimeError("fake position read failure")

    with pytest.raises(RuntimeError, match="fake position read failure"):
        PyRobotiqGripper3312Backend(FailingGripper()).read_position()


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


def test_pyrobotiq3312_backend_does_not_construct_or_start_device() -> None:
    """验证后端构造只保存注入对象，不触发设备生命周期操作。"""
    gripper = FakePyRobotiqGripper()

    backend = PyRobotiqGripper3312Backend(gripper)

    assert backend.read_position() == 0
    assert gripper.calls == []

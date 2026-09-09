"""验证 Robotiq 离散控制单步适配器。"""

from dataclasses import dataclass, field

import pytest
from robotiq_grasp_core import (
    DiscreteControlState,
    DiscreteForceControlConfig,
    DiscreteForceController,
)

from robotiq_hardware import (
    Robotiq2F85Hardware,
    RobotiqControlStepResult,
    RobotiqDiscreteControlStep,
    run_discrete_control_step,
)


@dataclass
class FakeBackend:
    """记录位置命令，不访问真实设备。"""

    commands: list[int] = field(default_factory=list)
    reads: int = 0
    fail_send: bool = False

    def send_position(self, position: int) -> None:
        """记录命令或模拟发送失败。"""
        if self.fail_send:
            raise RuntimeError("fake send failure")
        self.commands.append(position)

    def read_position(self) -> int:
        """记录反馈读取次数。"""
        self.reads += 1
        return self.commands[-1] if self.commands else 0


def _controller(**kwargs: object) -> DiscreteForceController:
    """创建适合单步测试的控制器。"""
    return DiscreteForceController(DiscreteForceControlConfig(**kwargs))


def test_zero_action_does_not_send_or_read_feedback() -> None:
    """零动作只返回控制状态，不发送命令或隐式读取位置。"""
    backend = FakeBackend()
    controller = _controller()

    result = run_discrete_control_step(
        controller,
        Robotiq2F85Hardware(backend),
        time_s=0.0,
        force_n=1.0,
    )

    assert result == RobotiqControlStepResult(
        state=DiscreteControlState.WAIT_STABLE,
        requested_delta=0,
        command_receipt=None,
    )
    assert backend.commands == []
    assert backend.reads == 0


def test_approach_action_is_not_repeated_before_interval() -> None:
    """控制器的接近动作在最小间隔内不会重复发送。"""
    backend = FakeBackend()
    hardware = Robotiq2F85Hardware(backend)
    controller = _controller()

    first = run_discrete_control_step(controller, hardware, time_s=0.0, force_n=0.0)
    second = run_discrete_control_step(controller, hardware, time_s=0.01, force_n=0.0)

    assert first.requested_delta == 3
    assert first.command_receipt is not None
    assert first.command_receipt.requested_position == 3
    assert second.requested_delta == 0
    assert second.command_receipt is None
    assert backend.commands == [3]


def test_move_succeeds_before_action_is_recorded() -> None:
    """只有硬件命令成功后才登记控制器动作。"""
    events: list[str] = []

    class OrderedBackend(FakeBackend):
        def send_position(self, position: int) -> None:
            events.append("move")
            super().send_position(position)

    class OrderedController(DiscreteForceController):
        def action_applied(self, delta: int, time_s: float) -> None:
            events.append("action_applied")
            super().action_applied(delta, time_s)

    backend = OrderedBackend()
    controller = OrderedController(DiscreteForceControlConfig())
    result = run_discrete_control_step(
        controller,
        Robotiq2F85Hardware(backend),
        time_s=0.0,
        force_n=0.0,
    )

    assert events == ["move", "action_applied"]
    assert result.command_receipt is not None
    assert controller.action_count == 1


def test_failed_move_does_not_record_action() -> None:
    """发送异常会原样传播，且控制器保持未登记动作。"""
    backend = FakeBackend(fail_send=True)
    controller = _controller()

    with pytest.raises(RuntimeError, match="fake send failure"):
        run_discrete_control_step(
            controller,
            Robotiq2F85Hardware(backend),
            time_s=0.0,
            force_n=0.0,
        )

    assert controller.command == 0
    assert controller.action_count == 0
    assert controller.snapshot().requested_delta == 0
    assert controller.decide(0.1) == 3


def test_absolute_command_stays_inside_0_to_255_boundary() -> None:
    """接近上边界时按控制器增量发送绝对位置命令。"""
    backend = FakeBackend()
    hardware = Robotiq2F85Hardware(backend)
    controller = _controller()
    controller.command = 255
    controller.state = DiscreteControlState.RELEASE

    result = run_discrete_control_step(controller, hardware, time_s=0.0, force_n=9.0)

    assert result.requested_delta == -1
    assert result.command_receipt is not None
    assert result.command_receipt.requested_position == 254
    assert controller.command == 254
    assert backend.commands == [254]


def test_adapter_construction_has_no_io() -> None:
    """构造单步适配器只保存依赖，不连接、激活或读取设备。"""
    backend = FakeBackend()
    controller = _controller()

    adapter = RobotiqDiscreteControlStep(controller, Robotiq2F85Hardware(backend))

    assert adapter is not None
    assert backend.commands == []
    assert backend.reads == 0

"""Robotiq 离散控制核心与硬件命令之间的单步适配。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from robotiq_grasp_core import DiscreteControlSnapshot, DiscreteControlState

from .gripper import Robotiq2F85Hardware, RobotiqCommandReceipt


class RobotiqDiscreteController(Protocol):
    """单步适配器所需的离散控制器最小接口。"""

    command: int

    def observe(self, force_n: float) -> None:
        """接收一个力观测。"""

    def decide(self, time_s: float) -> int:
        """按当前状态决定命令增量。"""

    def action_applied(self, delta: int, time_s: float) -> None:
        """登记已成功发送的命令增量。"""

    def cancel_pending_action(self) -> None:
        """取消尚未成功发送的待处理动作。"""

    def snapshot(self) -> DiscreteControlSnapshot:
        """返回当前控制状态快照。"""


@dataclass(frozen=True, slots=True)
class RobotiqControlStepResult:
    """一次控制单步的不可变结果。

    `position_feedback` 不属于本结果：单步适配器不会隐式读取设备反馈，
    以避免把可能阻塞的串口读取塞进控制周期。调用方可独立读取并记录反馈。
    """

    state: DiscreteControlState
    requested_delta: int
    command_receipt: RobotiqCommandReceipt | None


def run_discrete_control_step(
    controller: RobotiqDiscreteController,
    hardware: Robotiq2F85Hardware,
    *,
    time_s: float,
    force_n: float,
) -> RobotiqControlStepResult:
    """执行一次 Robotiq 离散力控制单步。

    Args:
        controller: 已创建的 Robotiq 离散控制器。
        hardware: 已注入后端的 Robotiq 2F85 硬件 facade。
        time_s: 当前控制时刻，传给控制器。
        force_n: 当前触觉或力传感器读数，传给控制器。

    Returns:
        包含动作后控制状态、请求增量和成功命令收据的不可变结果。零动作时
        `command_receipt` 为 `None`。

    Raises:
        ValueError: 力样本、时刻或最终位置命令不满足下层约束。
        Exception: 底层硬件发送失败时原样向调用方传播；此时控制器不会登记动作。
    """
    controller.observe(force_n)
    delta = controller.decide(time_s)
    if delta == 0:
        return RobotiqControlStepResult(
            state=controller.snapshot().state,
            requested_delta=0,
            command_receipt=None,
        )

    absolute_command = controller.command + delta
    try:
        receipt = hardware.move(absolute_command)
    except Exception:
        controller.cancel_pending_action()
        raise
    controller.action_applied(delta, time_s)
    return RobotiqControlStepResult(
        state=controller.snapshot().state,
        requested_delta=delta,
        command_receipt=receipt,
    )


class RobotiqDiscreteControlStep:
    """保存已注入依赖并提供无设备发现的单步调用入口。"""

    def __init__(
        self,
        controller: RobotiqDiscreteController,
        hardware: Robotiq2F85Hardware,
    ) -> None:
        """创建单步适配器，不连接、激活或读取设备。"""
        self._controller = controller
        self._hardware = hardware

    def step(self, *, time_s: float, force_n: float) -> RobotiqControlStepResult:
        """执行一次控制单步。"""
        return run_discrete_control_step(
            self._controller,
            self._hardware,
            time_s=time_s,
            force_n=force_n,
        )


__all__ = [
    "RobotiqControlStepResult",
    "RobotiqDiscreteController",
    "RobotiqDiscreteControlStep",
    "run_discrete_control_step",
]

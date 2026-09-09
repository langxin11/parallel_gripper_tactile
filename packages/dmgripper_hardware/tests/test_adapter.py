"""验证 DMgripper 控制核到 USB2CAN MIT 协议的离线适配。"""

from __future__ import annotations

from dataclasses import FrozenInstanceError
import math

import pytest
from dm_grasp_core import MITCommand as CoreMITCommand

from dmgripper_hardware import (
    CoreMotorObservation,
    DmMitCommandAdapter,
    MitCommand,
    MotorFeedback,
    MotorLimits,
    PreparedMitCommand,
    Usb2CanProtocol,
    core_observation_from_feedback,
    map_core_mit_command,
)


@pytest.fixture
def adapter() -> DmMitCommandAdapter:
    """提供使用已知量程与 CAN ID 的纯离线适配器。"""
    protocol = Usb2CanProtocol(MotorLimits(-1.7, 1.7, -8.0, 8.0, -4.0, 4.0))
    return DmMitCommandAdapter(protocol, motor_id=1)


def test_map_core_mit_command_keeps_each_field_semantics() -> None:
    """位置、速度、刚度、阻尼与前馈力矩应逐字段映射到独立协议对象。"""
    core_command = CoreMITCommand(0.25, -1.5, 123.0, 0.75, 2.5)

    protocol_command = map_core_mit_command(core_command)

    assert protocol_command == MitCommand(0.25, -1.5, 2.5, 123.0, 0.75)
    assert type(protocol_command) is MitCommand
    assert type(protocol_command) is not type(core_command)


def test_prepare_returns_fixed_usb2can_vector_and_immutable_receipt(
    adapter: DmMitCommandAdapter,
) -> None:
    """准备操作应返回固定 30 字节帧，且只创建不可变数据而不执行 I/O。"""
    core_command = CoreMITCommand(0.0, 0.0, 500.0, 5.0, 0.0)

    receipt = adapter.prepare(core_command)

    assert receipt.core_command is core_command
    assert receipt.protocol_command == MitCommand(0.0, 0.0, 0.0, 500.0, 5.0)
    assert receipt.frame == bytes(
        (
            0x55,
            0xAA,
            0x1E,
            0x03,
            0x01,
            0,
            0,
            0,
            0x0A,
            0,
            0,
            0,
            0,
            0x01,
            0,
            0,
            0,
            0,
            0x08,
            0,
            0,
            0x80,
            0,
            0x80,
            0x0F,
            0xFF,
            0xFF,
            0xF8,
            0,
            0,
        )
    )
    assert isinstance(receipt, PreparedMitCommand)
    with pytest.raises(FrozenInstanceError):
        receipt.frame = bytes()


def test_prepare_preserves_protocol_nan_rejection_and_range_saturation(
    adapter: DmMitCommandAdapter,
) -> None:
    """适配层不得绕过既有的非有限值拒绝和 MIT 量程饱和规则。"""
    with pytest.raises(ValueError, match="非有限"):
        adapter.prepare(CoreMITCommand(math.nan, 0.0, 0.0, 0.0, 0.0))

    receipt = adapter.prepare(CoreMITCommand(3.0, -20.0, 600.0, -1.0, 10.0))

    assert receipt.frame[21:29] == bytes((0xFF, 0xFF, 0, 0x0F, 0xFF, 0, 0x0F, 0xFF))


def test_core_observation_from_feedback_keeps_only_next_step_inputs() -> None:
    """反馈 DTO 只暴露当前控制核下一步所需的位置和速度。"""
    observation = core_observation_from_feedback(MotorFeedback(0.3, -0.4, 1.2, 8))

    assert observation == CoreMotorObservation(0.3, -0.4)
    assert not hasattr(observation, "torque_nm")
    assert not hasattr(observation, "status_code")

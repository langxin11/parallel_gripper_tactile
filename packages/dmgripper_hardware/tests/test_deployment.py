"""验证 DM4310P 平行夹爪部署参数和机械边界。"""

from __future__ import annotations

import math

import pytest

from dmgripper_hardware import Dm4310PGripperConfig, make_dm4310p_gripper_config


def test_factory_contains_exact_dm4310p_gripper_deployment_values() -> None:
    """工厂固定用户确认的 CAN ID、协议量程和闭合方向。"""
    config = make_dm4310p_gripper_config("fake://usb2can")

    assert isinstance(config, Dm4310PGripperConfig)
    assert config.motor_id == 1
    assert config.master_id == 17
    assert config.device.port == "fake://usb2can"
    assert config.motor_limits.position_min_rad == -1.7
    assert config.motor_limits.position_max_rad == 1.7
    assert config.motor_limits.velocity_min_rad_s == -8.0
    assert config.motor_limits.velocity_max_rad_s == 8.0
    assert config.motor_limits.torque_min_nm == -4.0
    assert config.motor_limits.torque_max_nm == 4.0
    assert config.joint_position_min_rad == 0.0
    assert config.joint_position_max_rad == math.pi / 2.0
    assert config.feedback_position_margin_rad == 0.05
    assert config.feedback_position_min_rad == -0.05
    assert config.feedback_position_max_rad == math.pi / 2.0 + 0.05
    assert config.closing_direction == 1


def test_joint_position_accepts_both_mechanical_boundaries() -> None:
    """机械行程的 0 和 pi/2 边界均可作为目标角。"""
    config = make_dm4310p_gripper_config("fake://usb2can")

    assert config.validate_joint_position(0.0) == 0.0
    assert config.validate_joint_position(math.pi / 2.0) == math.pi / 2.0


@pytest.mark.parametrize("position_rad", [-1e-12, math.pi / 2.0 + 1e-12, math.inf, math.nan])
def test_joint_position_rejects_out_of_range_or_nonfinite_target(position_rad: float) -> None:
    """机械目标越界或非有限时必须拒绝，不能静默 clamp。"""
    config = make_dm4310p_gripper_config("fake://usb2can")

    with pytest.raises(ValueError, match="机械关节目标角"):
        config.validate_joint_position(position_rad)


def test_feedback_position_uses_explicit_safety_margin_without_relaxing_commands() -> None:
    """反馈可略微越过工作端点，但同一位置仍不得作为目标命令。"""
    config = make_dm4310p_gripper_config("fake://usb2can", feedback_position_margin_rad=0.05)

    assert config.validate_feedback_position(-0.049) == -0.049
    assert config.validate_feedback_position(math.pi / 2.0 + 0.049) == pytest.approx(
        math.pi / 2.0 + 0.049
    )
    with pytest.raises(ValueError, match="目标角"):
        config.validate_joint_position(-0.001)


@pytest.mark.parametrize(
    "position_rad",
    [-0.050001, math.pi / 2.0 + 0.050001, math.inf, math.nan],
)
def test_feedback_position_rejects_values_outside_safety_range(position_rad: float) -> None:
    """扩展安全范围外或非有限反馈必须失败。"""
    config = make_dm4310p_gripper_config("fake://usb2can")

    with pytest.raises(ValueError, match="反馈角"):
        config.validate_feedback_position(position_rad)


@pytest.mark.parametrize("margin", [-0.001, math.inf, math.nan])
def test_feedback_margin_must_be_finite_and_nonnegative(margin: float) -> None:
    """反馈安全余量不能为负或非有限数。"""
    with pytest.raises(ValueError, match="反馈位置安全余量"):
        make_dm4310p_gripper_config("fake://usb2can", feedback_position_margin_rad=margin)


def test_feedback_margin_must_remain_inside_protocol_position_range() -> None:
    """反馈安全范围不得越过固件协议可解码的位置量程。"""
    with pytest.raises(ValueError, match="反馈位置安全范围"):
        make_dm4310p_gripper_config("fake://usb2can", feedback_position_margin_rad=2.0)


@pytest.mark.parametrize(
    ("joint_min", "joint_max", "message"),
    [
        (-1.8, 1.0, "下界"),
        (0.0, 1.8, "上界"),
        (1.0, 1.0, "下界必须小于上界"),
    ],
)
def test_config_rejects_invalid_mechanical_range(
    joint_min: float, joint_max: float, message: str
) -> None:
    """机械行程必须是协议位置量程内的非退化区间。"""
    base = make_dm4310p_gripper_config("fake://usb2can")

    with pytest.raises(ValueError, match=message):
        Dm4310PGripperConfig(
            device=base.device,
            motor_limits=base.motor_limits,
            joint_position_min_rad=joint_min,
            joint_position_max_rad=joint_max,
        )


def test_factory_does_not_open_or_access_io() -> None:
    """创建部署配置只校验和保存数据，不触发串口访问。"""
    config = make_dm4310p_gripper_config("/path/that/need/not/exist")

    assert config.device.port == "/path/that/need/not/exist"

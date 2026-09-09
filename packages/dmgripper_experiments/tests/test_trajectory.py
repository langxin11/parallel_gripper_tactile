"""验证夹爪闭合量空间轨迹。"""

import pytest

from dm_grasp_core import CrankSliderKinematics, MITCommandConfig
from dmgripper_hardware import MotorFeedback, STATUS_ENABLED

from dmgripper_experiments.runtime import _closure_trajectory_command
from dmgripper_experiments.trajectory import ClosureTrajectory


def test_closure_trajectory_round_trips_through_joint_kinematics() -> None:
    """闭合量轨迹可以连续反解为关节位置和速度。"""
    kinematics = CrankSliderKinematics(0.7853981633974483, 0.03, 0.04, 0.021213203435596423)
    start_q = 0.15
    goal_q = 1.1
    trajectory = ClosureTrajectory.from_limits(
        kinematics.closure(start_q),
        kinematics.closure(goal_q),
        0.01,
        0.025,
        0.1,
    )
    closure_m, closure_velocity_m_s, _ = trajectory.sample(trajectory.duration_s / 2.0)
    position_rad = kinematics.position_for_closure(closure_m, 0.0, 1.5707963267948966)
    velocity_rad_s = closure_velocity_m_s / kinematics.closure_jacobian(position_rad)
    assert start_q < position_rad < goal_q
    assert velocity_rad_s > 0.0
    assert trajectory.sample(0.0) == pytest.approx((kinematics.closure(start_q), 0.0, 0.0))
    assert trajectory.sample(trajectory.duration_s) == pytest.approx(
        (kinematics.closure(goal_q), 0.0, 0.0)
    )


def test_opening_closure_trajectory_has_negative_joint_velocity() -> None:
    """闭合量减小映射为负关节速度。"""
    kinematics = CrankSliderKinematics(0.7853981633974483, 0.03, 0.04, 0.021213203435596423)
    trajectory = ClosureTrajectory.from_limits(kinematics.closure(1.0), 0.0, 0.012, 0.025, 0.1)
    closure_m, closure_velocity_m_s, _ = trajectory.sample(trajectory.duration_s / 2.0)
    position_rad = kinematics.position_for_closure(closure_m, 0.0, 1.5707963267948966)
    assert closure_velocity_m_s / kinematics.closure_jacobian(position_rad) < 0.0


def test_return_command_uses_stronger_gain_and_reaches_zero_target() -> None:
    """回位闭合量命令使用独立增益，且零闭合量映射到机械零位。"""
    kinematics = CrankSliderKinematics(0.7853981633974483, 0.03, 0.04, 0.021213203435596423)
    config = MITCommandConfig(
        position_min_rad=0.0,
        position_max_rad=1.5707963267948966,
        velocity_limit_rad_s=0.3,
        closing_direction=1,
        kp=10.0,
        kd=0.5,
        feedforward_ratio=0.0,
        feedforward_torque_limit_nm=2.0,
        torque_limit_nm=2.0,
    )
    feedback = MotorFeedback(0.0784, 0.0, -0.15, STATUS_ENABLED)
    command = _closure_trajectory_command(
        kinematics,
        config,
        feedback,
        closure_m=0.0,
        closure_velocity_m_s=0.0,
        feedforward_force_n=0.0,
        feedforward_ratio=0.0,
        torque_limit_nm=2.0,
    )
    assert command.position_rad == pytest.approx(0.0, abs=1e-12)
    assert command.velocity_rad_s == 0.0
    assert command.kp == 10.0
    assert command.kd == 0.5

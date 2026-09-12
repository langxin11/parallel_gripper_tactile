"""离线验证倒水实验的纯控制核与阶段流程。"""

from __future__ import annotations

import math

import pytest
from dm_grasp_core import CrankSliderKinematics, MITCommandConfig
from dmgripper_hardware import MotorFeedback, STATUS_ENABLED

from dmgripper_experiments.config import ForceDemoConfig
from dmgripper_experiments.cup_config import CupConfig
from dmgripper_experiments.cup_control import CupForceController
from dmgripper_experiments.cup_flow import CupFlow
from dmgripper_experiments.tactile import TactileSnapshot


KINEMATICS = CrankSliderKinematics(
    theta0_rad=math.pi / 4,
    crank_radius_m=0.03,
    link_length_m=0.04,
    offset_m=0.021213203435596423,
)
COMMAND_CONFIG = MITCommandConfig(
    position_min_rad=0.0,
    position_max_rad=math.pi / 2,
    velocity_limit_rad_s=0.3,
    closing_direction=1,
    kp=2.0,
    kd=0.5,
    feedforward_ratio=0.5,
    feedforward_torque_limit_nm=2.0,
    torque_limit_nm=4.0,
)


def _feedback() -> MotorFeedback:
    """构造不访问设备的正常反馈。"""
    return MotorFeedback(
        position_rad=0.4, velocity_rad_s=0.0, torque_nm=0.0, status_code=STATUS_ENABLED
    )


def _tactile(left_force_n: float = 0.5, right_force_n: float = 0.5) -> TactileSnapshot:
    """构造完整双侧触觉快照。"""
    return TactileSnapshot(
        received_at_s=0.0,
        packet_counter=1,
        timestamp_us=1,
        left_force_n=left_force_n,
        right_force_n=right_force_n,
        raw_left_fz_n=left_force_n,
        raw_right_fz_n=right_force_n,
        raw_left_fx_n=0.0,
        raw_left_fy_n=0.0,
        raw_right_fx_n=0.0,
        raw_right_fy_n=0.0,
    )


@pytest.mark.parametrize("kind", ["pid", "adrc", "admittance"])
def test_cup_force_controllers_emit_finite_limited_mit_commands(kind: str) -> None:
    """三种外环均应输出有限且受机械、速度和力矩边界约束的命令。"""
    controller = CupForceController(CupConfig(controller=kind), KINEMATICS, COMMAND_CONFIG)
    feedback = _feedback()
    positions = []
    for index in range(30):
        command = controller.step(
            feedback=feedback,
            tactile=_tactile(0.4, 0.42),
            target_force_n=0.5 + index * 0.02,
            time_s=(index + 1) * 0.01,
            dt_s=0.01,
        )
        values = (command.position_rad, command.velocity_rad_s, command.feedforward_torque_nm)
        assert all(math.isfinite(value) for value in values)
        assert (
            COMMAND_CONFIG.position_min_rad
            <= command.position_rad
            <= COMMAND_CONFIG.position_max_rad
        )
        assert abs(command.velocity_rad_s) <= COMMAND_CONFIG.velocity_limit_rad_s
        predicted_torque_nm = (
            COMMAND_CONFIG.kp * (command.position_rad - feedback.position_rad)
            + COMMAND_CONFIG.kd * (command.velocity_rad_s - feedback.velocity_rad_s)
            + command.feedforward_torque_nm
        )
        assert abs(predicted_torque_nm) <= COMMAND_CONFIG.torque_limit_nm + 1e-9
        positions.append(command.position_rad)
        feedback = MotorFeedback(
            command.position_rad, command.velocity_rad_s, predicted_torque_nm, STATUS_ENABLED
        )
    assert positions[-1] > positions[0]


def test_takeover_enables_force_increase_before_ready() -> None:
    """手托稳定后进入 takeover，撤手确认前策略已经可以按切向扰动增力。"""
    flow = CupFlow(CupConfig())
    flow.update(
        time_s=0.0,
        dt_s=0.01,
        left_normal_n=0.5,
        right_normal_n=0.5,
        tangential_force_n=0.0,
        signed_tangential_force_n=0.0,
        closure_m=0.0,
    )
    baseline = flow.update(
        time_s=2.0,
        dt_s=0.01,
        left_normal_n=0.5,
        right_normal_n=0.5,
        tangential_force_n=0.0,
        signed_tangential_force_n=0.0,
        closure_m=0.0,
    )
    assert flow.phase == "takeover"

    increased = flow.update(
        time_s=2.01,
        dt_s=0.01,
        left_normal_n=0.5,
        right_normal_n=0.5,
        tangential_force_n=1.0,
        signed_tangential_force_n=1.0,
        closure_m=0.001,
    )
    assert increased.target_force_n > baseline.target_force_n
    assert increased.target_force_n <= flow.config.grip.max_target_force_n
    assert flow.phase == "takeover"


def test_ready_requires_stability_before_pour_then_waits_for_release() -> None:
    """ready 后仍需稳定窗口，倒水计时结束后只等待显式 release。"""
    control = ForceDemoConfig(tracking_duration_s=0.1)
    flow = CupFlow(CupConfig(control=control))
    _advance_to_takeover(flow)

    flow.action("ready")
    assert flow.phase == "stabilizing"
    flow.update(
        time_s=2.01,
        dt_s=0.01,
        left_normal_n=0.5,
        right_normal_n=0.5,
        tangential_force_n=0.0,
        signed_tangential_force_n=0.0,
        closure_m=0.0,
    )
    assert flow.phase == "stabilizing"
    flow.update(
        time_s=4.01,
        dt_s=0.01,
        left_normal_n=0.5,
        right_normal_n=0.5,
        tangential_force_n=0.0,
        signed_tangential_force_n=0.0,
        closure_m=0.0,
    )
    assert flow.phase == "pour"

    flow.update(
        time_s=4.12,
        dt_s=0.01,
        left_normal_n=0.5,
        right_normal_n=0.5,
        tangential_force_n=0.0,
        signed_tangential_force_n=0.0,
        closure_m=0.0,
    )
    assert flow.phase == "await_release"
    assert not flow.release_requested
    flow.action("release")
    assert flow.release_requested
    assert flow.phase == "await_release"


def _advance_to_takeover(flow: CupFlow) -> None:
    """以手托阶段的稳定触觉数据推进到接管阶段。"""
    for time_s in (0.0, 2.0):
        flow.update(
            time_s=time_s,
            dt_s=0.01,
            left_normal_n=0.5,
            right_normal_n=0.5,
            tangential_force_n=0.0,
            signed_tangential_force_n=0.0,
            closure_m=0.0,
        )
    assert flow.phase == "takeover"

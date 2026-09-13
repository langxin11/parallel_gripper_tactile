"""三控制器真机适配的限幅与导纳死区行为锚点（自根目录 cup 测试迁移）。"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest
from dmgripper_hardware import MotorFeedback, STATUS_ENABLED

from dmgripper_experiments.config import ControllerConfig, ExperimentConfig, TimingConfig
from dmgripper_experiments.control import GripController
from dmgripper_experiments.observation import PairedObservation, pair_observation
from dmgripper_experiments.runtime import _KINEMATICS as KINEMATICS
from dmgripper_experiments.tactile import TactileSnapshot
from dmgripper_experiments.targets import ForceTarget

from dm_grasp_core import MITCommandConfig

COMMAND_CONFIG = MITCommandConfig(
    position_min_rad=0.0,
    position_max_rad=math.pi / 2,
    velocity_limit_rad_s=0.3,
    closing_direction=1,
    kp=2.0,
    kd=0.5,
    feedforward_ratio=0.0,
    feedforward_torque_limit_nm=4.0,
    torque_limit_nm=4.0,
)


def _config(kind: str) -> ExperimentConfig:
    timing = TimingConfig()
    controller = replace(ControllerConfig(), kind=kind)
    return replace(
        ExperimentConfig(),
        timing=timing,
        controller=controller,
    )


def _tactile(left: float, right: float, received_at_s: float = 0.0) -> TactileSnapshot:
    """构造完整三轴触觉快照。"""
    return TactileSnapshot(
        received_at_s=received_at_s,
        packet_counter=1,
        timestamp_us=10_000,
        left_force_n=left,
        right_force_n=right,
        raw_left_fz_n=left,
        raw_right_fz_n=right,
        raw_left_fx_n=0.0,
        raw_left_fy_n=0.0,
        raw_right_fx_n=0.0,
        raw_right_fy_n=0.0,
    )


def _paired(feedback: MotorFeedback, left: float, right: float, counter: int = 1):
    """构造配对观测。"""
    from dataclasses import replace as _replace

    snapshot = _replace(
        _tactile(left, right), packet_counter=counter, timestamp_us=counter * 10_000
    )
    return pair_observation(
        snapshot=snapshot,
        feedback=feedback,
        previous=None,
        now_s=snapshot.received_at_s,
        kinematics=KINEMATICS,
    )


def _target(force_n: float) -> ForceTarget:
    return ForceTarget(force_n=force_n, rate_n_s=None, acceleration_n_s2=None, source="curve")


@pytest.mark.parametrize("kind", ["pid", "adrc", "admittance"])
def test_controllers_emit_finite_limited_mit_commands(kind: str) -> None:
    """三种外环均应输出有限且受机械、速度和力矩边界约束的命令。"""
    controller = GripController(_config(kind), kinematics=KINEMATICS, command_config=COMMAND_CONFIG)
    feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
    positions = []
    for index in range(30):
        paired = _paired(feedback, 0.4, 0.42, counter=index + 1)
        target = _target(0.5 + index * 0.02)
        step = (
            controller.begin_contact_tracking(
                paired=paired, target=target, time_s=(index + 1) * 0.01, dt=0.01
            )
            if index == 0
            else controller.step_tracking(
                paired=paired, target=target, time_s=(index + 1) * 0.01, dt=0.01
            )
        )
        command = step.command
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


def test_admittance_holds_position_for_deadband_and_overforce() -> None:
    """导纳进入死区或力偏高时均不得反向穿越传动间隙。"""
    controller = GripController(
        _config("admittance"), kinematics=KINEMATICS, command_config=COMMAND_CONFIG
    )
    feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
    closing = controller.begin_contact_tracking(
        paired=_paired(feedback, 0.3, 0.3), target=_target(0.6), time_s=0.01, dt=0.01
    )
    assert closing.command.position_rad > feedback.position_rad

    deadband = controller.step_tracking(
        paired=_paired(
            MotorFeedback(
                closing.command.position_rad, closing.command.velocity_rad_s, 0.0, STATUS_ENABLED
            ),
            0.55,
            0.55,
            counter=2,
        ),
        target=_target(0.6),
        time_s=0.02,
        dt=0.01,
    )
    assert deadband.command.position_rad == pytest.approx(closing.command.position_rad)
    assert deadband.command.velocity_rad_s == 0.0
    assert deadband.force_deadband_active

    held = controller.step_tracking(
        paired=_paired(
            MotorFeedback(deadband.command.position_rad, 0.0, 0.0, STATUS_ENABLED),
            0.9,
            0.9,
            counter=3,
        ),
        target=_target(0.6),
        time_s=0.03,
        dt=0.01,
    )
    assert held.command.position_rad == pytest.approx(deadband.command.position_rad)
    assert held.command.velocity_rad_s == 0.0
    assert held.unloading_blocked


def test_adaptive_source_activates_only_after_preload_baseline() -> None:
    """动态来源在 preload 学基线、activate 后才启用增长。"""
    from .fakes import FakeClock, FakeTactile, PhaseActions

    from dmgripper_experiments.config import AdaptiveReferenceConfig
    from dmgripper_experiments.targets import AdaptiveTargetSource

    config = _config("admittance")
    source = AdaptiveTargetSource(
        AdaptiveReferenceConfig(),
        control_rate_hz=config.timing.control_rate_hz,
        contact_floor_n=config.lifecycle.contact_off_n,
    )
    clock = FakeClock()
    actions = PhaseActions()
    FakeTactile(None, clock=clock, phase=actions)
    feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)

    def observe(force_tangential: float, counter: int) -> PairedObservation:
        snapshot = _tactile(0.5, 0.5)
        snapshot = replace(
            snapshot,
            raw_left_fy_n=force_tangential,
            received_at_s=clock.now_s,
            packet_counter=counter,
            timestamp_us=counter * 10_000,
        )
        clock.advance(0.01)
        return pair_observation(
            snapshot=snapshot,
            feedback=feedback,
            previous=None,
            now_s=snapshot.received_at_s,
            kinematics=KINEMATICS,
        )

    for index in range(5):
        source.observe(observe(0.0, index + 1), 0.01)
    assert source.latest_command is not None
    baseline_target = source.preload_target(0.0)
    assert baseline_target == pytest.approx(0.5)

    source.activate()
    for index in range(20):
        source.observe(observe(1.0, 10 + index), 0.01)
    grown = source.active_reference(0.2)
    assert grown.force_n > baseline_target
    assert grown.trigger_active is True

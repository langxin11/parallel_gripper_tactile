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


def test_pid_follows_moving_feedback_beyond_contact_offset_with_velocity_limit() -> None:
    """真机 PID 可越过旧接触偏置范围，最终指令仍遵守逐周期限速。"""
    config = _config("pid")
    controller = GripController(config, kinematics=KINEMATICS, command_config=COMMAND_CONFIG)
    initial_position = 0.4
    previous_target = initial_position
    for index in range(120):
        feedback = MotorFeedback(initial_position + index * 0.002, 0.0, 0.0, STATUS_ENABLED)
        operation = controller.begin_contact_tracking if index == 0 else controller.step_tracking
        step = operation(
            paired=_paired(feedback, 0.4, 0.4, counter=index + 1),
            target=_target(config.safety.max_target_force_n),
            time_s=(index + 1) * 0.01,
            dt=0.01,
        )
        assert abs(step.command.position_rad - previous_target) <= 0.003 + 1e-12
        assert step.command.position_rad > feedback.position_rad
        assert abs(step.position_adjustment) <= config.controller.pid.max_position_adjustment_rad
        previous_target = step.command.position_rad
    assert previous_target > initial_position + config.controller.pid.max_position_adjustment_rad


def _pid_feedforward_config(gain, *, estimation=True, kind="pid"):
    """配置独立模型前馈与可选刚度诊断。"""
    config = _config(kind)
    return replace(
        config,
        estimation=replace(config.estimation, enabled=estimation),
        controller=replace(
            config.controller,
            pid=replace(config.controller.pid, torque_feedforward_gain=gain),
        ),
    )


def _pid_unbounded_config(*, kind="pid"):
    """关闭固定偏置上限，保留后端机械、速度与力矩约束。"""
    config = _pid_feedforward_config(0.0, kind=kind)
    return replace(
        config,
        controller=replace(
            config.controller,
            pid=replace(config.controller.pid, max_position_adjustment_rad=None),
        ),
    )


def test_pid_unbounded_bias_can_exceed_old_limit_with_stationary_feedback():
    """受力不足且测量位置固定时，积分偏置可超过旧的 0.15 rad 上限。"""
    controller = GripController(
        _pid_unbounded_config(), kinematics=KINEMATICS, command_config=COMMAND_CONFIG
    )
    feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
    previous_position = feedback.position_rad
    for index in range(220):
        operation = controller.begin_contact_tracking if index == 0 else controller.step_tracking
        step = operation(
            paired=_paired(feedback, 0.4, 0.4, counter=index + 1),
            target=_target(0.9),
            time_s=(index + 1) * 0.01,
            dt=0.01,
        )
        assert abs(step.command.position_rad - previous_position) <= 0.003 + 1e-12
        previous_position = step.command.position_rad
    assert step.position_adjustment > 0.15
    assert step.command.position_rad - feedback.position_rad > 0.15


@pytest.mark.parametrize("constraint", ["position", "velocity", "torque"])
def test_pid_unbounded_bias_obeys_backend_limits_and_recovers_after_error_reversal(constraint):
    """取消固定偏置裁剪后，后端饱和仍受保护且反向误差能恢复调节。"""
    options = {
        "position": {"position_max_rad": 0.405},
        "velocity": {"velocity_limit_rad_s": 0.01},
        "torque": {"torque_limit_nm": 0.02, "feedforward_torque_limit_nm": 0.02},
    }[constraint]
    command_config = replace(COMMAND_CONFIG, **options)
    controller = GripController(
        _pid_unbounded_config(), kinematics=KINEMATICS, command_config=command_config
    )
    feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
    previous_position = feedback.position_rad
    # 最慢速度情形需要至少 6 s 才能撤回前半程的指令行程，留出滤波与反向余量。
    for index in range(1500):
        reverse = index >= 600
        operation = controller.begin_contact_tracking if index == 0 else controller.step_tracking
        step = operation(
            paired=_paired(
                feedback, 1.0 if reverse else 0.4, 1.0 if reverse else 0.4, counter=index + 1
            ),
            target=_target(0.2 if reverse else 1.2),
            time_s=(index + 1) * 0.01,
            dt=0.01,
        )
        command = step.command
        assert (
            command_config.position_min_rad
            <= command.position_rad
            <= command_config.position_max_rad
        )
        assert (
            abs(command.position_rad - previous_position)
            <= command_config.velocity_limit_rad_s * 0.01 + 1e-12
        )
        torque = (
            command_config.kp * (command.position_rad - feedback.position_rad)
            + command_config.kd * (command.velocity_rad_s - feedback.velocity_rad_s)
            + command.feedforward_torque_nm
        )
        assert abs(torque) <= command_config.torque_limit_nm + 1e-12
        if index == 599 and constraint != "velocity":
            # 长时间顶住行程或力矩边界，积分不得把请求继续推到远离可执行值的位置。
            assert abs(step.position_adjustment) < 0.05
        previous_position = command.position_rad
    assert step.command.position_rad < feedback.position_rad
    assert step.position_adjustment < 0.0


@pytest.mark.parametrize("kind", ["adrc", "admittance"])
def test_disabled_pid_bias_limit_preserves_other_controllers(kind):
    """PID 的 None 上限不改变其他控制器的原有有限位移策略。"""
    steps = []
    for config in (_pid_feedforward_config(0.0, kind=kind), _pid_unbounded_config(kind=kind)):
        controller = GripController(config, kinematics=KINEMATICS, command_config=COMMAND_CONFIG)
        feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
        trajectory = []
        for index in range(20):
            operation = (
                controller.begin_contact_tracking if index == 0 else controller.step_tracking
            )
            trajectory.append(
                operation(
                    paired=_paired(feedback, 0.4, 0.4, counter=index + 1),
                    target=_target(1.0),
                    time_s=(index + 1) * 0.01,
                    dt=0.01,
                )
            )
        steps.append(trajectory)
    assert steps[0] == steps[1]


@pytest.mark.parametrize("estimation", [False, True])
def test_pid_model_feedforward_tracks_target_and_real_position_without_stiffness(estimation):
    """无有效刚度时仍按实测位置与当前目标输出模型前馈，且不增加位置修正。"""
    config = _pid_feedforward_config(0.7, estimation=estimation)
    controller = GripController(config, kinematics=KINEMATICS, command_config=COMMAND_CONFIG)
    baseline = GripController(
        _pid_feedforward_config(0.0, estimation=estimation),
        kinematics=KINEMATICS,
        command_config=COMMAND_CONFIG,
    )
    for index, (position, target) in enumerate(((0.4, 0.7), (0.45, 1.1), (0.5, 0.6))):
        feedback = MotorFeedback(position, 0.0, 0.0, STATUS_ENABLED)
        arguments = dict(
            paired=_paired(feedback, 0.4, 0.4, counter=index + 1),
            target=_target(target),
            time_s=(index + 1) * 0.01,
            dt=0.01,
        )
        operation = controller.begin_contact_tracking if index == 0 else controller.step_tracking
        reference_operation = (
            baseline.begin_contact_tracking if index == 0 else baseline.step_tracking
        )
        step = operation(**arguments)
        reference = reference_operation(**arguments)
        assert step.command.feedforward_torque_nm == pytest.approx(
            0.7 * KINEMATICS.closure_jacobian(position) * target
        )
        assert step.command.position_rad == pytest.approx(reference.command.position_rad)
        assert step.position_adjustment == pytest.approx(reference.position_adjustment)
        assert step.stiffness_estimate_n_per_m is None


@pytest.mark.parametrize("gain, expected_gain", [(None, 0.0), (0.0, 0.0), (0.4, 0.4), (1.0, 1.0)])
def test_pid_model_feedforward_is_controlled_only_by_explicit_gain(gain, expected_gain):
    """机构模型力矩前馈仅由独立增益控制，不消费刚度估计。"""
    config = _pid_feedforward_config(gain)
    controller = GripController(config, kinematics=KINEMATICS, command_config=COMMAND_CONFIG)
    feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
    step = controller.begin_contact_tracking(
        paired=_paired(feedback, 0.4, 0.4),
        target=_target(1.0),
        time_s=0.01,
        dt=0.01,
    )
    jacobian = KINEMATICS.closure_jacobian(feedback.position_rad)
    assert step.command.feedforward_torque_nm == pytest.approx(expected_gain * jacobian)
    assert step.position_adjustment == pytest.approx((0.016 + 0.2 * 0.01) * 0.6)


def test_pid_legacy_default_without_stiffness_has_no_feedforward():
    """未设置独立增益且未消费刚度时保持原来的零前馈。"""
    controller = GripController(
        _pid_feedforward_config(None), kinematics=KINEMATICS, command_config=COMMAND_CONFIG
    )
    step = controller.begin_contact_tracking(
        paired=_paired(MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED), 0.4, 0.4),
        target=_target(1.0),
        time_s=0.01,
        dt=0.01,
    )
    assert step.command.feedforward_torque_nm == 0.0


@pytest.mark.parametrize("kind", ["adrc", "admittance"])
def test_pid_feedforward_setting_does_not_change_other_controllers(kind):
    """PID 专属前馈字段不改变 LADRC 或导纳指令。"""
    steps = []
    for gain in (None, 1.0):
        controller = GripController(
            _pid_feedforward_config(gain, kind=kind),
            kinematics=KINEMATICS,
            command_config=COMMAND_CONFIG,
        )
        steps.append(
            controller.begin_contact_tracking(
                paired=_paired(MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED), 0.4, 0.4),
                target=_target(1.0),
                time_s=0.01,
                dt=0.01,
            )
        )
    assert steps[0] == steps[1]


def test_pid_feedforward_still_obeys_total_mit_torque_limit():
    """独立前馈与位置项合成后仍遵守 MIT 总力矩限制。"""
    command_config = replace(COMMAND_CONFIG, torque_limit_nm=0.01, feedforward_torque_limit_nm=0.01)
    controller = GripController(
        _pid_feedforward_config(1.0), kinematics=KINEMATICS, command_config=command_config
    )
    feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
    step = controller.begin_contact_tracking(
        paired=_paired(feedback, 0.4, 0.4),
        target=_target(1.5),
        time_s=0.01,
        dt=0.01,
    )
    assert KINEMATICS.closure_jacobian(feedback.position_rad) * 1.5 > command_config.torque_limit_nm
    torque = (
        command_config.kp * (step.command.position_rad - feedback.position_rad)
        + command_config.kd * (step.command.velocity_rad_s - feedback.velocity_rad_s)
        + step.command.feedforward_torque_nm
    )
    assert abs(torque) <= command_config.torque_limit_nm + 1e-12


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
        max_target_force_n=config.safety.max_target_force_n,
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

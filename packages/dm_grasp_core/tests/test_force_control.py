"""法向力跟踪核心算法的纯计算单元测试。"""

from __future__ import annotations

import dataclasses
import math

import pytest

from dm_grasp_core import (
    AdrcConfig,
    ContactStiffnessConfig,
    ContactStiffnessEstimator,
    CrankSliderKinematics,
    ForceControlObservation,
    ForceControlReference,
    MITControlConfig,
    MITTorqueModel,
    NormalForceConfig,
    NormalForceController,
    SecondOrderTorqueLADRC,
    TorqueAdrcConfig,
)
from dm_grasp_core.control.mit import _roundtrip_unsigned

GEOMETRY = CrankSliderKinematics(
    theta0_rad=math.pi / 4.0,
    crank_radius_m=0.03,
    link_length_m=0.06,
    offset_m=0.0,
)

MIT_CONFIG = MITControlConfig(
    p_min=-1.7,
    p_max=1.7,
    v_max=8.0,
    t_max=4.0,
    kp=10.0,
    kd=0.5,
)

STIFFNESS_CONFIG = ContactStiffnessConfig(
    enabled=True,
    method="window_linear",
    initial_n_per_m=2000.0,
    min_n_per_m=100.0,
    max_n_per_m=100_000.0,
    filter_alpha=0.15,
    min_delta_closure_m=1e-5,
    min_delta_force_n=1e-3,
)


class FakeInner:
    """记录命令并把测量位置回喂给外环的最小 MIT 内环替身。"""

    torque_limit_n_m = 4.0

    def __init__(self, *, position: float = 0.0, velocity: float = 0.0) -> None:
        """以固定实测位置与速度初始化替身。"""
        self.position_value = position
        self.velocity_value = velocity
        self.commands: list[dict[str, float | None]] = []

    def position(self) -> float:
        """返回固定的实测关节位置。"""
        return self.position_value

    def apply(
        self,
        *,
        target_position: float,
        target_velocity: float = 0.0,
        feedforward_torque: float | None = None,
        stiffness_override: float | None = None,
        damping_override: float | None = None,
    ):
        """记录命令并返回真实 MIT 模型的量化计算结果。"""
        self.commands.append(
            {
                "target_position": target_position,
                "target_velocity": target_velocity,
                "feedforward_torque": feedforward_torque,
                "stiffness_override": stiffness_override,
                "damping_override": damping_override,
            }
        )
        return MITTorqueModel(MIT_CONFIG).apply(
            position=self.position_value,
            velocity=self.velocity_value,
            target_position=target_position,
            target_velocity=target_velocity,
            feedforward_torque=feedforward_torque,
            stiffness_override=stiffness_override,
            damping_override=damping_override,
        )


def test_mit_model_quantizes_and_saturates_commands() -> None:
    """MIT 模型按达妙协议量化目标并饱和合成力矩。"""
    model = MITTorqueModel(MIT_CONFIG)
    command = model.apply(position=0.0, velocity=0.0, target_position=100.0)

    assert command.target_position == MIT_CONFIG.p_max
    assert command.torque == MIT_CONFIG.t_max

    position = _roundtrip_unsigned(0.123456, MIT_CONFIG.p_min, MIT_CONFIG.p_max, 16)
    velocity = _roundtrip_unsigned(0.0, -MIT_CONFIG.v_max, MIT_CONFIG.v_max, 12)
    feedforward = _roundtrip_unsigned(0.4567, -MIT_CONFIG.t_max, MIT_CONFIG.t_max, 12)
    stiffness = _roundtrip_unsigned(MIT_CONFIG.kp, 0.0, 500.0, 12)
    damping = _roundtrip_unsigned(MIT_CONFIG.kd, 0.0, 5.0, 12)
    command = model.apply(
        position=0.0,
        velocity=0.0,
        target_position=0.123456,
        feedforward_torque=0.4567,
    )
    assert command.target_position == pytest.approx(position)
    assert command.feedforward_torque == pytest.approx(feedforward)
    assert command.torque == pytest.approx(stiffness * position + damping * velocity + feedforward)


def test_mit_model_gain_overrides_act_for_one_command_only() -> None:
    """kp/kd 覆盖只在单次命令生效，下一命令恢复配置增益。"""
    model = MITTorqueModel(MIT_CONFIG)
    bypassed = model.apply(
        position=0.1,
        velocity=0.0,
        target_position=0.2,
        feedforward_torque=0.5,
        stiffness_override=0.0,
        damping_override=0.0,
    )
    # 增益置零后只剩量化后的前馈力矩。
    assert bypassed.torque == pytest.approx(_roundtrip_unsigned(0.5, -4.0, 4.0, 12))

    restored = model.apply(position=0.1, velocity=0.0, target_position=0.2, feedforward_torque=0.5)
    expected = (
        _roundtrip_unsigned(MIT_CONFIG.kp, 0.0, 500.0, 12)
        * (_roundtrip_unsigned(0.2, MIT_CONFIG.p_min, MIT_CONFIG.p_max, 16) - 0.1)
        + _roundtrip_unsigned(MIT_CONFIG.kd, 0.0, 5.0, 12)
        * (_roundtrip_unsigned(0.0, -MIT_CONFIG.v_max, MIT_CONFIG.v_max, 12) - 0.0)
        + _roundtrip_unsigned(0.5, -4.0, 4.0, 12)
    )
    assert restored.torque == pytest.approx(expected)


def test_second_order_torque_ladrc_keeps_bumpless_switch_and_gain_scheduling() -> None:
    """二阶 LADRC 稳态切换与 b0 调度前后保持力矩命令连续。"""
    config = TorqueAdrcConfig(
        equivalent_inertia_kg_m2=0.0021617741125,
        input_gain_scale=2.0,
        controller_bandwidth_rad_s=60.0,
        observer_bandwidth_rad_s=240.0,
        measurement_filter_cutoff_hz=40.0,
        min_input_gain_n_per_n_m_s2=5000.0,
        max_input_gain_n_per_n_m_s2=200000.0,
        max_torque_rate_n_m_s=100.0,
    )
    controller = SecondOrderTorqueLADRC(config)
    controller.reset(
        measured_force_n=2.0,
        applied_torque_n_m=0.3,
        input_gain_n_per_n_m_s2=1000.0,
    )
    first = controller.step(
        measured_force_n=2.0,
        target_force_n=2.0,
        target_force_rate_n_s=0.0,
        target_force_acceleration_n_s2=0.0,
        model_feedforward_torque_n_m=0.0,
        input_gain_n_per_n_m_s2=1000.0,
        dt=0.002,
        min_torque_n_m=-4.0,
        max_torque_n_m=4.0,
    )
    controller.set_applied_torque(first.requested_torque_n_m, model_feedforward_torque_n_m=0.0)
    second = controller.step(
        measured_force_n=2.0,
        target_force_n=2.0,
        target_force_rate_n_s=0.0,
        target_force_acceleration_n_s2=0.0,
        model_feedforward_torque_n_m=0.0,
        input_gain_n_per_n_m_s2=2000.0,
        dt=0.002,
        min_torque_n_m=-4.0,
        max_torque_n_m=4.0,
    )

    assert first.requested_torque_n_m == pytest.approx(0.3)
    assert first.estimated_disturbance_n_s2 == pytest.approx(-300.0)
    assert second.requested_torque_n_m == pytest.approx(0.3)
    assert second.estimated_disturbance_n_s2 == pytest.approx(-600.0)


def test_contact_stiffness_estimator_secant_updates_on_valid_samples() -> None:
    """割线方法用有效相邻样本按滤波系数更新估计。"""
    estimator = ContactStiffnessEstimator(
        dataclasses.replace(STIFFNESS_CONFIG, method="secant_ewma"),
        GEOMETRY,
    )
    q0 = 0.4
    q1 = 0.5
    estimator.reset(position_rad=q0, normal_force_n=1.0)
    delta_closure = GEOMETRY.closure(q1) - GEOMETRY.closure(q0)
    estimate = estimator.update(
        position_rad=q1,
        normal_force_n=1.0 + 10000.0 * delta_closure,
    )

    expected = STIFFNESS_CONFIG.initial_n_per_m + 0.15 * (
        10000.0 - STIFFNESS_CONFIG.initial_n_per_m
    )
    assert estimate == pytest.approx(expected)


def test_normal_force_controller_switches_to_tracking_after_bilateral_contact() -> None:
    """双侧力连续超阈值后进入力跟踪，并从接触位置开始位置修正。"""
    config = NormalForceConfig(
        target_n=5.0,
        contact_threshold_n=1.0,
        contact_confirm_steps=3,
        release_threshold_n=0.2,
        release_confirm_steps=5,
        kp=0.02,
        ki=0.4,
        kd=0.0,
        max_position_adjustment=0.3,
        filter_cutoff_hz=20.0,
        geometry=GEOMETRY,
        stiffness=STIFFNESS_CONFIG,
    )
    controller = NormalForceController(config)
    inner = FakeInner(position=0.3)
    reference = ForceControlReference(target_force_n=5.0)

    for _ in range(2):
        command = controller.step(
            inner,
            observation=ForceControlObservation(
                time_s=0.0,
                approach_position=0.3,
                total_normal_force_n=0.4,
                left_normal_force_n=0.2,
                right_normal_force_n=0.2,
                dt=0.002,
            ),
            reference=reference,
        )
        assert command.state == "approach"

    for _ in range(3):
        command = controller.step(
            inner,
            observation=ForceControlObservation(
                time_s=0.0,
                approach_position=0.3,
                total_normal_force_n=4.0,
                left_normal_force_n=2.0,
                right_normal_force_n=2.0,
                dt=0.002,
            ),
            reference=reference,
        )
    assert command.state == "force_tracking"
    assert inner.commands[-1]["stiffness_override"] is None
    assert command.measured_force_n == pytest.approx(2.0)

    command = controller.step(
        inner,
        observation=ForceControlObservation(
            time_s=0.008,
            approach_position=0.3,
            total_normal_force_n=4.0,
            left_normal_force_n=2.0,
            right_normal_force_n=2.0,
            dt=0.002,
        ),
        reference=reference,
    )
    assert command.force_error_n == pytest.approx(5.0 - command.filtered_force_n)
    assert command.position_adjustment != 0.0


def test_normal_force_controller_resets_after_release() -> None:
    """双侧力持续低于释放阈值后复位回接近模式。"""
    config = NormalForceConfig(
        target_n=5.0,
        contact_threshold_n=1.0,
        contact_confirm_steps=1,
        release_threshold_n=0.2,
        release_confirm_steps=2,
        kp=0.02,
        ki=0.4,
        kd=0.0,
        max_position_adjustment=0.3,
        filter_cutoff_hz=20.0,
    )
    controller = NormalForceController(config)
    inner = FakeInner(position=0.3)

    def observe(left: int, right: int, tick: int):
        return ForceControlObservation(
            time_s=tick * 0.002,
            approach_position=0.3,
            total_normal_force_n=left + right,
            left_normal_force_n=left,
            right_normal_force_n=right,
            dt=0.002,
        )

    reference = ForceControlReference(target_force_n=5.0)
    controller.step(inner, observation=observe(2, 2, 0), reference=reference)
    assert controller.state == "force_tracking"

    controller.step(inner, observation=observe(0, 0, 1), reference=reference)
    assert controller.state == "force_tracking"
    controller.step(inner, observation=observe(0, 0, 2), reference=reference)
    assert controller.state == "approach"


def test_first_order_adrc_outer_loop_replaces_pid_position_correction() -> None:
    """一阶 LADRC 路径下 PID 修正置零，位置修正由闭合速度积分产生。"""
    config = NormalForceConfig(
        target_n=5.0,
        contact_threshold_n=1.0,
        contact_confirm_steps=1,
        release_threshold_n=0.2,
        release_confirm_steps=5,
        kp=0.02,
        ki=0.4,
        kd=0.0,
        max_position_adjustment=0.3,
        filter_cutoff_hz=20.0,
        geometry=GEOMETRY,
        adrc=AdrcConfig(
            b0_n_per_m=780.0,
            controller_bandwidth_rad_s=10.0,
            observer_bandwidth_rad_s=30.0,
            max_closing_velocity_m_s=0.02,
        ),
    )
    controller = NormalForceController(config)
    inner = FakeInner(position=0.3)
    reference = ForceControlReference(target_force_n=5.0)
    controller.step(
        inner,
        observation=ForceControlObservation(
            time_s=0.0,
            approach_position=0.3,
            total_normal_force_n=4.0,
            left_normal_force_n=2.0,
            right_normal_force_n=2.0,
            dt=0.002,
        ),
        reference=reference,
    )
    assert controller.state == "force_tracking"
    command = controller.step(
        inner,
        observation=ForceControlObservation(
            time_s=0.002,
            approach_position=0.3,
            total_normal_force_n=4.0,
            left_normal_force_n=2.0,
            right_normal_force_n=2.0,
            dt=0.002,
        ),
        reference=reference,
    )

    assert command.pid_position_adjustment == 0.0
    assert command.position_adjustment > 0.0
    assert inner.commands[-1]["stiffness_override"] is None


def test_normal_force_config_rejects_inconsistent_paths() -> None:
    """互斥外环在控制器构造时被拒绝，配置层拒绝阈值倒置与缺失几何。"""
    base = dict(
        target_n=5.0,
        contact_threshold_n=1.0,
        contact_confirm_steps=1,
        release_threshold_n=0.2,
        release_confirm_steps=5,
        kp=0.02,
        ki=0.4,
        kd=0.0,
        max_position_adjustment=0.3,
        filter_cutoff_hz=20.0,
    )
    mixed = NormalForceConfig(
        **base,
        geometry=GEOMETRY,
        torque_feedback_gain=1.0,
        adrc=AdrcConfig(
            b0_n_per_m=780.0,
            controller_bandwidth_rad_s=10.0,
            observer_bandwidth_rad_s=30.0,
            max_closing_velocity_m_s=0.02,
        ),
    )
    with pytest.raises(ValueError, match="mutually exclusive"):
        NormalForceController(mixed)
    with pytest.raises(ValueError, match="release_threshold_n"):
        NormalForceConfig(**{**base, "contact_threshold_n": 0.1})
    with pytest.raises(ValueError, match="geometry is required"):
        NormalForceConfig(**base, stiffness=STIFFNESS_CONFIG)

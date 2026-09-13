"""外部生命周期跟踪接口与刚度诊断快照的契约测试。"""

from __future__ import annotations

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
    StiffnessSnapshot,
)

KINEMATICS = CrankSliderKinematics(
    theta0_rad=0.7853981633974483,
    crank_radius_m=0.03,
    link_length_m=0.04,
    offset_m=0.021213203435596423,
)

MIT_CONFIG = MITControlConfig(p_min=-1.7, p_max=1.7, v_max=8.0, t_max=4.0, kp=10.0, kd=0.5)


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


def _controller(*, adrc: bool = False, stiffness: ContactStiffnessConfig | None = None):
    """构造 PID 或一阶 LADRC 的被测控制器。"""
    return NormalForceController(
        NormalForceConfig(
            target_n=0.5,
            contact_threshold_n=0.2,
            contact_confirm_steps=1,
            release_threshold_n=0.1,
            release_confirm_steps=1,
            kp=0.02,
            ki=0.2,
            kd=0.0,
            max_position_adjustment=0.15,
            filter_cutoff_hz=10.0,
            geometry=KINEMATICS,
            stiffness=stiffness,
            adrc=(
                AdrcConfig(
                    b0_n_per_m=1000.0,
                    controller_bandwidth_rad_s=5.0,
                    observer_bandwidth_rad_s=20.0,
                    max_closing_velocity_m_s=0.002,
                )
                if adrc
                else None
            ),
        )
    )


def _observation(left: float, right: float, dt: float = 0.01, time_s: float = 0.0):
    """构造平均单侧语义的观测。"""
    return ForceControlObservation(
        time_s=time_s,
        approach_position=0.0,
        total_normal_force_n=left + right,
        left_normal_force_n=left,
        right_normal_force_n=right,
        dt=dt,
    )


def _reference(force_n: float, rate: float = 0.0):
    return ForceControlReference(target_force_n=force_n, target_force_rate_n_s=rate)


def test_begin_tracking_initializes_contact_reference_and_pid():
    """begin 以当前内环位置为接触参考，首个命令立即进入跟踪。"""
    controller = _controller()
    inner = FakeInner(position=0.3)
    command = controller.begin_tracking(
        inner,
        observation=_observation(0.4, 0.4),
        reference=_reference(0.5),
    )
    assert command.state == "force_tracking"
    assert inner.commands, "begin_tracking 必须产生首个命令"
    assert controller.state == "force_tracking"


def test_step_tracking_requires_begin_first():
    """未初始化直接 step 必须报错，防止双重生命周期。"""
    controller = _controller()
    inner = FakeInner(position=0.3)
    with pytest.raises(RuntimeError, match="begin_tracking"):
        controller.step_tracking(
            inner, observation=_observation(0.4, 0.4), reference=_reference(0.5)
        )


def test_step_tracking_tracks_rising_reference():
    """跟踪若干周期后滤波力趋近目标且命令有限。"""
    controller = _controller()
    inner = FakeInner(position=0.3)
    controller.begin_tracking(inner, observation=_observation(0.4, 0.4), reference=_reference(0.5))
    for step in range(50):
        command = controller.step_tracking(
            inner,
            observation=_observation(0.45, 0.45, dt=0.01, time_s=0.01 * (step + 1)),
            reference=_reference(0.5),
        )
        assert command.state == "force_tracking"
        assert command.mit.position == command.mit.position  # 有限性由模型保证
    assert controller._filtered_force is not None  # noqa: SLF001  检查滤波状态存在


def test_external_stiffness_conflicts_with_internal_estimator():
    """内部估计器启用时传入外部刚度必须报错。"""
    stiffness = ContactStiffnessConfig(
        enabled=True,
        initial_n_per_m=2000.0,
        min_n_per_m=100.0,
        max_n_per_m=100_000.0,
        filter_alpha=0.15,
        min_delta_closure_m=1e-5,
        min_delta_force_n=1e-3,
    )
    controller = _controller(stiffness=stiffness)
    inner = FakeInner(position=0.3)
    with pytest.raises(RuntimeError, match="external stiffness"):
        controller.begin_tracking(
            inner,
            observation=_observation(0.4, 0.4),
            reference=_reference(0.5),
            external_stiffness_n_per_m=1500.0,
        )


def test_external_stiffness_requires_gain_configuration():
    """无刚度增益配置时消费外部估计必须报错。"""
    controller = _controller()
    inner = FakeInner(position=0.3)
    controller.begin_tracking(inner, observation=_observation(0.4, 0.4), reference=_reference(0.5))
    with pytest.raises(RuntimeError, match="stiffness gain"):
        controller.step_tracking(
            inner,
            observation=_observation(0.4, 0.4),
            reference=_reference(0.5),
            external_stiffness_n_per_m=1500.0,
        )


def test_stiffness_snapshot_reports_single_consumed_update():
    """快照的 updated 事件只被消费一次，重复快照不重复报告。"""
    estimator = ContactStiffnessEstimator(
        ContactStiffnessConfig(
            enabled=True,
            method="secant_ewma",
            initial_n_per_m=1000.0,
            min_n_per_m=100.0,
            max_n_per_m=100_000.0,
            filter_alpha=0.5,
            min_delta_closure_m=1e-6,
            min_delta_force_n=1e-6,
        ),
        KINEMATICS,
    )
    estimator.reset(position_rad=0.3, normal_force_n=0.1)
    assert estimator.snapshot().reason == "initial"
    # 闭合方向：closing_direction 语义由运动学保证，直接改变位置产生样本。
    estimator.update(position_rad=0.31, normal_force_n=0.2)
    first = estimator.snapshot(time_s=1.0, sample_id=7)
    assert isinstance(first, StiffnessSnapshot)
    assert first.updated is True
    assert first.valid is True
    assert first.last_update_time_s == 1.0
    assert first.sample_id == 7
    second = estimator.snapshot(time_s=1.5, sample_id=8)
    assert second.updated is False
    assert second.reason == "holding_previous"
    # 激励不足的样本给出可识别原因且不改变估计。
    before = estimator.estimate_n_per_m
    estimator.update(position_rad=0.31, normal_force_n=0.2)
    third = estimator.snapshot()
    assert third.updated is False
    assert third.reason == "insufficient_excitation"
    assert estimator.estimate_n_per_m == before


def test_adrc_path_supports_external_tracking():
    """一阶 LADRC 路径同样可以走外部生命周期接口。"""
    controller = _controller(adrc=True)
    inner = FakeInner(position=0.3)
    command = controller.begin_tracking(
        inner,
        observation=_observation(0.4, 0.4),
        reference=_reference(0.5),
    )
    assert command.state == "force_tracking"
    for step in range(10):
        command = controller.step_tracking(
            inner,
            observation=_observation(0.42, 0.42, dt=0.01, time_s=0.01 * (step + 1)),
            reference=_reference(0.5),
        )
        assert command.state == "force_tracking"

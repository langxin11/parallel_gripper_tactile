"""独立数学期望和迁移前节点固定观测回放。"""

from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
import pytest
import dm_grasp_core.command as legacy_command
import dm_grasp_core.control as legacy_control
from dm_grasp_core.control import admittance, kinematics
from dm_grasp_core.grasp import command, motion
from dm_grasp_core.tactile import contact
from dm_grasp_core import (
    CrankSliderKinematics,
    SecondOrderAdmittance,
    MITCommand,
    MITCommandConfig,
    build_mit_command,
    step_admittance,
    ContactDetector,
    MinimumJerkTrajectory,
    ContactTransition,
    quintic_blend,
    within_zero_window,
    limit_mit_position_for_torque,
)

K = CrankSliderKinematics(math.pi / 4, 0.03, 0.04, 0.021213203435596423)
C = MITCommandConfig(0.0, 0.9, 0.4, 1, 8.0, 0.2, 0.7, 0.2, 0.6)


def test_domain_packages_preserve_top_level_and_legacy_object_identity():
    """新子域、顶层和旧子模块均指向同一算法对象。"""
    assert CrankSliderKinematics is kinematics.CrankSliderKinematics
    assert SecondOrderAdmittance is admittance.SecondOrderAdmittance
    assert limit_mit_position_for_torque is admittance.limit_mit_position_for_torque
    assert MinimumJerkTrajectory is motion.MinimumJerkTrajectory
    assert ContactTransition is motion.ContactTransition
    assert quintic_blend is motion.quintic_blend
    assert within_zero_window is contact.within_zero_window
    assert ContactDetector is contact.ContactDetector
    assert MITCommand is command.MITCommand
    assert MITCommandConfig is command.MITCommandConfig
    assert build_mit_command is command.build_mit_command
    assert step_admittance is command.step_admittance

    for name in (
        "CrankSliderKinematics",
        "MinimumJerkTrajectory",
        "quintic_blend",
        "within_zero_window",
        "ContactTransition",
        "SecondOrderAdmittance",
        "limit_mit_position_for_torque",
        "ContactDetector",
    ):
        assert getattr(legacy_control, name) is globals()[name]
    for name in ("MITCommand", "MITCommandConfig", "build_mit_command", "step_admittance"):
        assert getattr(legacy_command, name) is globals()[name]
    assert legacy_command.dataclass is dataclass


def test_legacy_control_star_import_keeps_original_non_private_exports():
    """旧 ``control`` 路径的星号导入继续包含原先可见的辅助对象。"""
    namespace: dict[str, object] = {}
    exec("from dm_grasp_core.control import *", namespace)

    assert namespace["math"] is math
    assert namespace["dataclass"] is dataclass
    assert namespace["ContactDetector"] is ContactDetector


def test_legacy_command_keeps_original_non_private_exports():
    """旧 ``command`` 路径完整转发原实现曾公开的名称。"""
    implementation_names = {name for name in dir(command) if not name.startswith("_")}
    legacy_names = {name for name in dir(legacy_command) if not name.startswith("_")}

    assert legacy_names == implementation_names
    for name in legacy_names:
        assert getattr(legacy_command, name) is getattr(command, name)


def test_geometry_derivative_and_inverse():
    """验证 geometry derivative and inverse。"""
    for q in (0.01, 0.2, 0.5, 0.89):
        numerical = (K.closure(q + 1e-6) - K.closure(q - 1e-6)) / 2e-6
        assert K.closure_jacobian(q) == pytest.approx(numerical, rel=1e-8)
        assert K.position_for_closure(K.closure(q), 0.0, 0.9) == pytest.approx(q, abs=1e-14)
    assert K.aperture(0.0) == pytest.approx(2 * (0.03 / math.sqrt(2) + 0.04))


def test_semi_implicit_euler():
    """验证 semi implicit euler。"""
    a = SecondOrderAdmittance(2.0, 3.0, 4.0, 0.1, 0.2)
    x, v = a.step(5.0, 0.002)
    assert v == pytest.approx(0.2 + (5.0 - 3.0 * 0.2 - 4.0 * 0.1) / 2.0 * 0.002)
    assert x == pytest.approx(0.1 + v * 0.002)
    a.reset()
    assert (a.displacement_m, a.velocity_m_s) == (0.0, 0.0)


def test_admittance_clips_velocity_before_displacement_integration():
    """虚拟速度限幅必须在更新位移前生效，避免冲击造成单步位移跳变。"""
    a = SecondOrderAdmittance(1.0, 0.0, 0.0)

    displacement, velocity = a.step(100.0, 0.1, maximum_velocity_m_s=0.2)

    assert velocity == pytest.approx(0.2)
    assert displacement == pytest.approx(0.02)


def test_step_admittance_limits_single_step_displacement_rate():
    """共享导纳步骤以实测雅可比换算虚拟速度上限后再积分位移。"""
    admittance = SecondOrderAdmittance(1.0, 0.0, 0.0)
    measured_position = 0.3
    dt_s = 0.1
    maximum_velocity = C.velocity_limit_rad_s * K.closure_jacobian(measured_position)

    step_admittance(
        admittance,
        K,
        replace(C, torque_limit_nm=100.0),
        reference_position_rad=measured_position,
        measured_position_rad=measured_position,
        measured_velocity_rad_s=0.0,
        left_force_n=0.0,
        right_force_n=0.0,
        target_force_n=1_000.0,
        dt_s=dt_s,
    )

    assert admittance.velocity_m_s == pytest.approx(maximum_velocity)
    assert abs(admittance.displacement_m) <= maximum_velocity * dt_s


def test_state_saturation():
    """验证 state saturation。"""
    a = SecondOrderAdmittance(1.0, 0.0, 0.0, 10.0, 10.0)
    assert a.limit_state(-0.1, 0.2, 0.3) == (0.2, 0.0)
    a.displacement_m = -10.0
    a.velocity_m_s = -10.0
    assert a.limit_state(-0.1, 0.2, 0.3) == (-0.1, 0.0)
    a.displacement_m = 0.0
    a.velocity_m_s = 10.0
    assert a.limit_state(-0.1, 0.2, 0.3) == (0.0, 0.3)


def test_jacobians_and_feedforward_are_distinct():
    """验证 jacobians and feedforward are distinct。"""
    cfg = replace(C, torque_limit_nm=100.0, feedforward_torque_limit_nm=100.0)
    result = build_mit_command(
        K,
        cfg,
        reference_position_rad=0.2,
        displacement_m=K.closure(0.5) - K.closure(0.2),
        velocity_m_s=0.005,
        measured_position_rad=0.3,
        measured_velocity_rad_s=0.0,
        feedforward_force_n=3.0,
    )
    assert result.position_rad == pytest.approx(0.5, abs=1e-14)
    assert result.velocity_rad_s == pytest.approx(0.005 / K.closure_jacobian(0.5))
    assert result.feedforward_torque_nm == pytest.approx(0.7 * K.closure_jacobian(0.3) * 3.0)


def test_composite_torque_and_infeasible_mechanical_bound():
    """验证 composite torque and infeasible mechanical bound。"""
    assert limit_mit_position_for_torque(
        target_position_rad=2.0,
        target_velocity_rad_s=0.5,
        measured_position_rad=0.1,
        measured_velocity_rad_s=0.2,
        kp=10.0,
        kd=2.0,
        feedforward_torque_nm=0.3,
        torque_limit_nm=1.0,
    ) == pytest.approx(0.11)
    with pytest.raises(ValueError, match="机械角限位"):
        build_mit_command(
            K,
            C,
            reference_position_rad=0.3,
            displacement_m=0.0,
            velocity_m_s=0.0,
            measured_position_rad=0.3,
            measured_velocity_rad_s=100.0,
            feedforward_force_n=0.0,
        )


@pytest.mark.parametrize(
    "scenario",
    json.loads(Path(__file__).with_name("golden_tracking.json").read_text())["scenarios"],
)
def test_original_node_golden(scenario):
    """验证 original node golden。"""
    a = SecondOrderAdmittance(0.8, 40.0, 100.0)
    cfg = replace(C, closing_direction=scenario["direction"])
    for row in scenario["rows"]:
        actual = step_admittance(a, K, cfg, **row["inputs"])
        assert asdict(actual) == row["command"]
        assert [a.displacement_m, a.velocity_m_s] == row["state"]


@pytest.mark.parametrize("dt", [0.0, -0.002, float("nan"), float("inf")])
def test_invalid_dt(dt):
    """验证 invalid dt。"""
    with pytest.raises(ValueError):
        SecondOrderAdmittance(1.0, 0.0, 0.0).step(1.0, dt)


@pytest.mark.parametrize("force", [float("nan"), float("inf"), -float("inf")])
def test_invalid_observation(force):
    """验证 invalid observation。"""
    with pytest.raises(ValueError):
        step_admittance(
            SecondOrderAdmittance(1.0, 0.0, 0.0),
            K,
            C,
            reference_position_rad=0.3,
            measured_position_rad=0.3,
            measured_velocity_rad_s=0.0,
            left_force_n=force,
            right_force_n=0.0,
            target_force_n=3.0,
            dt_s=0.002,
        )


def test_trajectory_contact_and_zero():
    """验证 trajectory contact and zero。"""
    t = MinimumJerkTrajectory.from_limits(0.0, 1.0, 1.0, 2.0, 10.0)
    assert t.sample(-1.0) == (0.0, 0.0, 0.0)
    assert t.sample(t.duration_s) == (1.0, 0.0, 0.0)
    assert t.sample(t.duration_s / 2)[0] == pytest.approx(0.5)
    assert ContactTransition(2.0, 1.0).velocity_at(0.5) == pytest.approx(1.0)
    assert quintic_blend(-1.0) == 0.0 and quintic_blend(2.0) == 1.0
    assert within_zero_window(-0.1, 0.1, 0.1)
    d = ContactDetector(0.5, 0.1)
    assert not d.update(0.6, 0.6, 0.0)
    assert d.update(0.6, 0.6, 0.11)
    assert not d.update(0.4, 0.6, 0.12)
    d.reset()
    assert not d.update(0.6, 0.6, 0.2)


@pytest.mark.parametrize(
    "operation",
    [
        lambda: K.closure(float("nan")),
        lambda: K.position_for_closure(0.0, 1.0, 0.0),
        lambda: quintic_blend(float("nan")),
        lambda: within_zero_window(0.0, 0.0, -1.0),
        lambda: SecondOrderAdmittance(0.0, 0.0, 0.0).step(1.0, 0.002),
        lambda: SecondOrderAdmittance(1.0, 0.0, 0.0).limit_state(1.0, 0.0, 1.0),
    ],
)
def test_invalid_math_inputs(operation):
    """验证 invalid math inputs。"""
    with pytest.raises(ValueError):
        operation()


def test_command_overrides_and_saturation():
    """验证显式覆盖前馈、速度上限与收紧合成力矩。"""
    result = build_mit_command(
        K,
        C,
        reference_position_rad=0.3,
        displacement_m=0.01,
        velocity_m_s=10.0,
        measured_position_rad=0.3,
        measured_velocity_rad_s=0.0,
        feedforward_force_n=100.0,
        feedforward_ratio=0.0,
        torque_limit_nm=0.1,
    )
    assert result.feedforward_torque_nm == 0.0
    assert result.velocity_rad_s == C.velocity_limit_rad_s
    torque = C.kp * (result.position_rad - 0.3) + C.kd * result.velocity_rad_s
    assert torque == pytest.approx(0.1)
    result = build_mit_command(
        K,
        C,
        reference_position_rad=0.3,
        displacement_m=0.0,
        velocity_m_s=0.0,
        measured_position_rad=0.3,
        measured_velocity_rad_s=0.0,
        feedforward_force_n=100.0,
    )
    assert result.feedforward_torque_nm == C.feedforward_torque_limit_nm


def test_step_stops_at_mechanical_bound():
    """验证闭合端点处清零向外速度而保留端点位移。"""
    a = SecondOrderAdmittance(1.0, 0.0, 0.0)
    step_admittance(
        a,
        K,
        replace(C, torque_limit_nm=100.0, velocity_limit_rad_s=1000.0),
        reference_position_rad=0.3,
        measured_position_rad=0.89,
        measured_velocity_rad_s=0.0,
        left_force_n=0.0,
        right_force_n=0.0,
        target_force_n=100.0,
        dt_s=1.0,
    )
    assert a.displacement_m == K.closure(0.9) - K.closure(0.3)
    assert a.velocity_m_s == 0.0

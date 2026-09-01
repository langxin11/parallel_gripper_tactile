"""验证 MIT 力矩控制律、命令限幅与 MJCF 执行器契约。"""

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from parallel_gripper_tactile import (
    ContactStiffnessEstimator,
    CrankSliderKinematics,
    ForceControlObservation,
    ForceControlReference,
    MITTorqueController,
    NormalForceController,
    load_profile,
)


ROOT = Path(__file__).resolve().parents[1]


def _roundtrip_unsigned(value: float, lower: float, upper: float, bits: int) -> float:
    levels = (1 << bits) - 1
    clipped = min(max(value, lower), upper)
    encoded = round((clipped - lower) / (upper - lower) * levels)
    return lower + encoded / levels * (upper - lower)


def test_mit_controller_clamps_position_velocity_and_torque() -> None:
    """MIT 控制器按 profile 限制 P/V/T 命令并向 motor 写入力矩。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    controller = MITTorqueController.from_profile(model, profile)

    command = controller.apply(
        data,
        target_position=100.0,
        target_velocity=100.0,
        feedforward_torque=100.0,
    )

    assert profile.mit is not None
    assert command.target_position == profile.mit.p_max
    assert command.target_velocity == profile.mit.v_max
    assert command.feedforward_torque == profile.mit.t_max
    assert command.torque == profile.mit.t_max
    assert data.ctrl[controller.actuator_id] == profile.mit.t_max
    np.testing.assert_allclose(model.actuator_ctrlrange[controller.actuator_id], (-4.0, 4.0))
    np.testing.assert_allclose(model.actuator_forcerange[controller.actuator_id], (-4.0, 4.0))
    np.testing.assert_allclose(model.actuator_gainprm[controller.actuator_id, :3], (1.0, 0.0, 0.0))
    np.testing.assert_allclose(model.actuator_biasprm[controller.actuator_id, :3], (0.0, 0.0, 0.0))


def test_mit_controller_roundtrips_damiao_protocol_quantization() -> None:
    """MIT 控制器先模拟达妙 CAN 帧量化，再计算输出轴力矩。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    controller = MITTorqueController.from_profile(model, profile)

    command = controller.apply(
        data,
        target_position=0.123456,
        target_velocity=1.2345,
        feedforward_torque=0.4567,
    )

    assert profile.mit is not None
    position = _roundtrip_unsigned(0.123456, profile.mit.p_min, profile.mit.p_max, 16)
    velocity = _roundtrip_unsigned(1.2345, -profile.mit.v_max, profile.mit.v_max, 12)
    feedforward = _roundtrip_unsigned(0.4567, -profile.mit.t_max, profile.mit.t_max, 12)
    stiffness = _roundtrip_unsigned(profile.mit.kp, 0.0, 500.0, 12)
    damping = _roundtrip_unsigned(profile.mit.kd, 0.0, 5.0, 12)
    expected_torque = stiffness * position + damping * velocity + feedforward

    assert command.target_position == pytest.approx(position)
    assert command.target_velocity == pytest.approx(velocity)
    assert command.feedforward_torque == pytest.approx(feedforward)
    assert command.torque == pytest.approx(expected_torque)
    assert data.ctrl[controller.actuator_id] == pytest.approx(expected_torque)


def test_crank_slider_kinematics_matches_gripper_aperture_formula() -> None:
    """曲柄滑块模型复现夹爪开度公式和闭合雅可比。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    assert profile.normal_force is not None
    assert profile.normal_force.geometry is not None
    kinematics = CrankSliderKinematics.from_config(profile.normal_force.geometry)

    assert kinematics.aperture(0.0) == pytest.approx(0.122426407, abs=1e-9)
    assert kinematics.aperture(np.pi / 4.0) == pytest.approx(0.078045940, abs=1e-9)
    assert kinematics.closure_jacobian(np.pi / 4.0) == pytest.approx(0.06, abs=1e-9)


def test_contact_stiffness_estimator_tracks_force_over_closure() -> None:
    """接触刚度估计器用总闭合行程上的 dF/dc 样本更新。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    assert profile.normal_force is not None
    assert profile.normal_force.geometry is not None
    assert profile.normal_force.stiffness is not None
    kinematics = CrankSliderKinematics.from_config(profile.normal_force.geometry)
    config = profile.normal_force.stiffness.model_copy(update={"method": "secant_ewma"})
    estimator = ContactStiffnessEstimator(config, kinematics)

    q0 = 0.4
    q1 = 0.5
    estimator.reset(position_rad=q0, normal_force_n=1.0)
    delta_closure = kinematics.closure(q1) - kinematics.closure(q0)
    estimate = estimator.update(
        position_rad=q1,
        normal_force_n=1.0 + 10000.0 * delta_closure,
    )

    initial = profile.normal_force.stiffness.initial_n_per_m
    expected = initial + 0.15 * (10000.0 - initial)
    assert estimate == pytest.approx(expected)


def test_contact_stiffness_estimator_window_linear_uses_all_samples() -> None:
    """滑动窗口一次拟合用多个样本恢复线性接触刚度。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    assert profile.normal_force is not None
    assert profile.normal_force.geometry is not None
    assert profile.normal_force.stiffness is not None
    kinematics = CrankSliderKinematics.from_config(profile.normal_force.geometry)
    config = profile.normal_force.stiffness.model_copy(
        update={"method": "window_linear", "filter_alpha": 1.0, "min_samples": 4}
    )
    estimator = ContactStiffnessEstimator(config, kinematics)

    q_values = [0.4 + 0.02 * index for index in range(7)]
    reference_closure = kinematics.closure(q_values[0])
    estimator.reset(position_rad=q_values[0], normal_force_n=1.0)
    for q in q_values:
        closure = kinematics.closure(q)
        estimate = estimator.update(
            position_rad=q,
            normal_force_n=1.0 + 10000.0 * (closure - reference_closure),
        )

    assert estimate == pytest.approx(10000.0, rel=1e-6)


def test_contact_stiffness_estimator_window_quadratic_returns_current_slope() -> None:
    """滑动窗口二次拟合返回当前闭合量处的局部导数。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    assert profile.normal_force is not None
    assert profile.normal_force.geometry is not None
    assert profile.normal_force.stiffness is not None
    kinematics = CrankSliderKinematics.from_config(profile.normal_force.geometry)
    config = profile.normal_force.stiffness.model_copy(
        update={"method": "window_quadratic", "filter_alpha": 1.0, "min_samples": 5}
    )
    estimator = ContactStiffnessEstimator(config, kinematics)

    q_values = [0.4 + 0.02 * index for index in range(8)]
    initial_closure = kinematics.closure(q_values[0])
    estimator.reset(
        position_rad=q_values[0],
        normal_force_n=1.0 + 6000.0 * initial_closure + 200000.0 * initial_closure**2,
    )
    final_closure = kinematics.closure(q_values[-1])
    for q in q_values:
        closure = kinematics.closure(q)
        estimator.update(
            position_rad=q,
            normal_force_n=1.0 + 6000.0 * closure + 200000.0 * closure**2,
        )

    expected = 6000.0 + 400000.0 * final_closure
    assert estimator.estimate_n_per_m == pytest.approx(expected, rel=1e-6)


def test_contact_stiffness_estimator_window_keeps_last_estimate_for_invalid_fit() -> None:
    """样本不足或闭合/力跨度不足时，窗口估计保持上次结果。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    assert profile.normal_force is not None
    assert profile.normal_force.geometry is not None
    assert profile.normal_force.stiffness is not None
    kinematics = CrankSliderKinematics.from_config(profile.normal_force.geometry)
    config = profile.normal_force.stiffness.model_copy(
        update={"method": "window_linear", "filter_alpha": 1.0, "min_samples": 4}
    )
    estimator = ContactStiffnessEstimator(config, kinematics)
    estimator.reset(position_rad=0.4, normal_force_n=1.0)

    for _ in range(3):
        assert estimator.update(position_rad=0.4, normal_force_n=1.0) == pytest.approx(3000.0)

    assert estimator.update(position_rad=0.4, normal_force_n=1.1) == pytest.approx(3000.0)
    assert estimator.update(position_rad=0.5, normal_force_n=math.nan) == pytest.approx(3000.0)


def test_normal_force_controller_switches_after_bilateral_contact() -> None:
    """双侧接触确认后 simple-pid 外环接管，并在完全脱离后恢复接近。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    controller = NormalForceController.from_profile(model, profile)

    for _ in range(4):
        command = controller.apply(
            data,
            approach_position=0.5,
            total_normal_force_n=0.4,
            left_normal_force_n=0.2,
            right_normal_force_n=0.2,
            dt=0.002,
        )
        assert command.state == "approach"

    command = controller.apply(
        data,
        approach_position=0.5,
        total_normal_force_n=0.4,
        left_normal_force_n=0.2,
        right_normal_force_n=0.2,
        dt=0.002,
    )
    assert command.state == "force_tracking"
    assert command.measured_force_n == pytest.approx(0.2)
    assert command.filtered_force_n == pytest.approx(0.2)
    assert command.position_adjustment > 0
    assert command.stiffness_position_adjustment > 0
    assert profile.normal_force.stiffness is not None
    assert command.estimated_contact_stiffness_n_per_m == pytest.approx(
        profile.normal_force.stiffness.initial_n_per_m
    )
    assert command.closure_jacobian_m_per_rad == pytest.approx(0.042426407, abs=1e-9)
    assert command.force_feedforward_torque == pytest.approx(
        8.0 * command.closure_jacobian_m_per_rad
    )
    assert command.aperture_m == pytest.approx(0.122426407, abs=1e-9)
    assert command.mit.target_position < 0.5

    assert profile.normal_force is not None
    for _ in range(profile.normal_force.release_confirm_steps):
        command = controller.apply(
            data,
            approach_position=0.5,
            total_normal_force_n=0.0,
            left_normal_force_n=0.0,
            right_normal_force_n=0.0,
            dt=0.002,
        )
    assert command.state == "approach"


def test_total_force_semantics_preserves_legacy_measurement_and_feedforward() -> None:
    """旧总力语义使用双侧和，并以 ``F_sum / 2`` 映射总闭合雅可比。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    controller = NormalForceController.from_profile(model, profile, force_semantics="total")

    for _ in range(5):
        command = controller.apply(
            data,
            approach_position=0.5,
            total_normal_force_n=0.4,
            left_normal_force_n=0.2,
            right_normal_force_n=0.2,
            dt=0.002,
        )

    assert command.state == "force_tracking"
    assert command.measured_force_n == pytest.approx(0.4)
    assert command.closure_jacobian_m_per_rad is not None
    assert command.force_feedforward_torque == pytest.approx(
        0.5 * 8.0 * command.closure_jacobian_m_per_rad
    )


def test_normal_force_controller_exposes_force_tracking_interface() -> None:
    """统一 step 接口可供 force tracking 任务替换不同控制器实现。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    controller = NormalForceController.from_profile(model, profile)

    command = controller.step(
        data,
        observation=ForceControlObservation(
            time_s=0.0,
            approach_position=0.5,
            total_normal_force_n=0.0,
            left_normal_force_n=0.0,
            right_normal_force_n=0.0,
            dt=0.002,
        ),
        reference=ForceControlReference(
            target_force_n=4.0,
            approach_feedforward_force_n=2.0,
        ),
    )

    assert command.state == "approach"
    assert command.target_force_n == pytest.approx(4.0)
    assert command.closure_jacobian_m_per_rad == pytest.approx(0.042426407, abs=1e-9)
    assert command.force_feedforward_torque == pytest.approx(
        2.0 * command.closure_jacobian_m_per_rad
    )

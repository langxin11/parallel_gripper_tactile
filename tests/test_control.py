"""验证 MIT 力矩控制律、命令限幅与 MJCF 执行器契约。"""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from parallel_gripper_tactile import (
    ContactStiffnessEstimator,
    CrankSliderKinematics,
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
    estimator = ContactStiffnessEstimator(profile.normal_force.stiffness, kinematics)

    q0 = 0.4
    q1 = 0.5
    estimator.reset(position_rad=q0, normal_force_n=1.0)
    delta_closure = kinematics.closure(q1) - kinematics.closure(q0)
    estimate = estimator.update(
        position_rad=q1,
        normal_force_n=1.0 + 10000.0 * delta_closure,
    )

    expected = 6000.0 + 0.15 * (10000.0 - 6000.0)
    assert estimate == pytest.approx(expected)


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
    assert command.estimated_contact_stiffness_n_per_m == pytest.approx(6000.0)
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

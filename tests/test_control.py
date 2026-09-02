"""验证 MIT 力矩控制律、命令限幅与 MJCF 执行器契约。"""

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from parallel_gripper_tactile import (
    AdrcControl,
    ContactStiffnessEstimator,
    CrankSliderKinematics,
    ForceControlObservation,
    ForceControlReference,
    GripperProfile,
    MITTorqueController,
    NormalForceController,
    load_profile,
)
from parallel_gripper_tactile.control import (
    DAMIAO_DAMPING_RANGE,
    DAMIAO_GAIN_BITS,
    DAMIAO_STIFFNESS_RANGE,
    _roundtrip_unsigned,
)


ROOT = Path(__file__).resolve().parents[1]


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


def test_damiao_gain_quantization_roundtrips_exact_zero() -> None:
    """直接力矩路径依赖 0 增益经达妙无符号量化往返后仍精确为 0。"""
    for lower, upper in (DAMIAO_STIFFNESS_RANGE, DAMIAO_DAMPING_RANGE):
        assert _roundtrip_unsigned(0.0, lower, upper, DAMIAO_GAIN_BITS) == 0.0


def test_mit_controller_gain_overrides_act_for_one_command_only() -> None:
    """单周期 kp/kd 覆盖参与达妙量化，之后的命令仍沿用 profile 增益。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    controller = MITTorqueController.from_profile(model, profile)

    overridden = controller.apply(
        data,
        target_position=0.1,
        target_velocity=1.0,
        feedforward_torque=0.2,
        stiffness_override=0.0,
        damping_override=0.0,
    )

    # kp/kd 覆盖为 0 后，输出力矩只剩量化前馈项。
    assert overridden.torque == pytest.approx(overridden.feedforward_torque)
    assert data.ctrl[controller.actuator_id] == pytest.approx(overridden.feedforward_torque)

    baseline = controller.apply(
        data,
        target_position=0.1,
        target_velocity=1.0,
        feedforward_torque=0.2,
    )

    # 不传覆盖时恢复 profile kp/kd，位置弹簧项重新出现。
    assert baseline.torque > baseline.feedforward_torque
    assert baseline.torque == pytest.approx(
        _roundtrip_unsigned(float(profile.mit.kp), *DAMIAO_STIFFNESS_RANGE, DAMIAO_GAIN_BITS)
        * (baseline.target_position - baseline.position)
        + _roundtrip_unsigned(float(profile.mit.kd), *DAMIAO_DAMPING_RANGE, DAMIAO_GAIN_BITS)
        * (baseline.target_velocity - baseline.velocity)
        + baseline.feedforward_torque
    )


def _profile_with_torque_feedback_gain(profile: GripperProfile, value: float) -> GripperProfile:
    """返回仅覆盖 force.torque_feedback_gain 的不可变 profile 副本。"""
    assert profile.normal_force is not None
    force = profile.normal_force.model_copy(update={"torque_feedback_gain": value})
    return profile.model_copy(
        update={"control": profile.control.model_copy(update={"force": force})}
    )


def test_normal_force_controller_direct_torque_branch_assembles_feedforward() -> None:
    """torque_feedback_gain>0 时跟踪阶段走直接力矩路径，位置修正与 MIT kp/kd 置零。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    direct_profile = _profile_with_torque_feedback_gain(profile, 1.0)
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    controller = NormalForceController.from_profile(model, direct_profile)

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
    # PID 与刚度位置修正不进入命令。
    assert command.pid_position_adjustment == 0.0
    assert command.stiffness_position_adjustment == 0.0
    assert command.position_adjustment == 0.0
    assert command.mit.target_position == pytest.approx(command.mit.position, abs=1e-9)
    # t_ff = 1.0 * (8.0-0.2) * Jc + 1.0 * 8.0 * Jc（average_side 语义缩放为 1）。
    assert command.closure_jacobian_m_per_rad is not None
    jacobian = command.closure_jacobian_m_per_rad
    assert command.force_feedforward_torque == pytest.approx((7.8 + 8.0) * jacobian)
    # MIT kp/kd 逐周期覆盖为 0，输出力矩只剩量化前馈。
    assert command.mit.torque == pytest.approx(command.mit.feedforward_torque)
    assert data.ctrl[controller.actuator_id] == pytest.approx(command.mit.feedforward_torque)
    # 刚度估计器照常运行，trace 中刚度曲线保持可比。
    assert profile.normal_force is not None
    assert profile.normal_force.stiffness is not None
    assert command.estimated_contact_stiffness_n_per_m == pytest.approx(
        profile.normal_force.stiffness.initial_n_per_m
    )


def test_default_torque_feedback_gain_keeps_positional_tracking_path() -> None:
    """torque_feedback_gain 默认 0；显式 0.0 与未设置的输出完全一致。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    zeroed_profile = _profile_with_torque_feedback_gain(source, 0.0)
    model = mujoco.MjModel.from_xml_path(str(source.model_path))
    default_controller = NormalForceController.from_profile(model, source)
    zeroed_controller = NormalForceController.from_profile(model, zeroed_profile)
    default_data = mujoco.MjData(model)
    zeroed_data = mujoco.MjData(model)

    forces = [(0.2, 0.2)] * 8 + [(3.0, 2.6), (4.2, 3.6), (4.0, 4.0)]
    default_command = None
    zeroed_command = None
    for left, right in forces:
        default_command = default_controller.apply(
            default_data,
            approach_position=0.5,
            total_normal_force_n=left + right,
            left_normal_force_n=left,
            right_normal_force_n=right,
            dt=0.002,
        )
        zeroed_command = zeroed_controller.apply(
            zeroed_data,
            approach_position=0.5,
            total_normal_force_n=left + right,
            left_normal_force_n=left,
            right_normal_force_n=right,
            dt=0.002,
        )

    assert default_command is not None
    assert zeroed_command is not None
    assert default_command == zeroed_command
    # 位置式路径未被旁路：PID 修正非零，MIT 力矩仍含位置弹簧项。
    assert default_command.pid_position_adjustment > 0.0
    assert default_command.mit.torque != pytest.approx(default_command.mit.feedforward_torque)


def _profile_with_adrc(profile: GripperProfile, adrc: AdrcControl | None) -> GripperProfile:
    """返回仅覆盖 force.adrc 的不可变 profile 副本。"""
    assert profile.normal_force is not None
    force = profile.normal_force.model_copy(update={"adrc": adrc})
    return profile.model_copy(
        update={"control": profile.control.model_copy(update={"force": force})}
    )


_ADRC_TEST_CONFIG = AdrcControl(
    b0_n_per_m=780.0,
    controller_bandwidth_rad_s=10.0,
    observer_bandwidth_rad_s=30.0,
    max_closing_velocity_m_s=0.02,
)


def test_normal_force_controller_adrc_branch_integrates_velocity_into_adjustment() -> None:
    """adrc 分支：LESO 闭合速度裁剪后积分进位置修正，MIT kp/kd 不被覆盖。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    adrc_profile = _profile_with_adrc(source, _ADRC_TEST_CONFIG)
    model = mujoco.MjModel.from_xml_path(str(source.model_path))
    data = mujoco.MjData(model)
    controller = NormalForceController.from_profile(model, adrc_profile)

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
    # 接触确认时 LADRC 复位：z1 对齐滤波力 0.2，z2 与积分修正清零。
    assert controller._adrc_z1 == pytest.approx(0.2)
    assert controller._adrc_z2 == 0.0
    # 未裁剪控制律 u = ω_c·(f_ref−z1)/b0 = 10·(8−0.2)/780 ≈ 0.1，超出 ±0.02 被裁剪。
    assert controller._adrc_u == pytest.approx(0.02)
    # 闭合速度经闭合雅可比换算为电机角速度后积分：u_max/J_c·dt。
    jacobian = command.closure_jacobian_m_per_rad
    assert jacobian is not None and jacobian > 0.0
    assert command.position_adjustment == pytest.approx(0.02 / jacobian * 0.002)
    assert command.pid_position_adjustment == 0.0
    assert command.stiffness_position_adjustment == 0.0
    # MIT 内环增益未被 override：输出力矩仍含位置弹簧项（目标位置高于当前位形）。
    assert command.mit.target_position > command.mit.position
    assert command.mit.torque != pytest.approx(command.mit.feedforward_torque)
    assert data.ctrl[controller.actuator_id] == pytest.approx(command.mit.torque)
    # 模型力矩前馈照常走 torque_feedforward_gain 路径（average_side 语义缩放为 1）。
    assert command.closure_jacobian_m_per_rad is not None
    assert command.force_feedforward_torque == pytest.approx(
        8.0 * command.closure_jacobian_m_per_rad
    )
    # 刚度估计器照常运行，trace 中刚度曲线保持可比。
    assert source.normal_force is not None
    assert source.normal_force.stiffness is not None
    assert command.estimated_contact_stiffness_n_per_m == pytest.approx(
        source.normal_force.stiffness.initial_n_per_m
    )

    # 第二个跟踪周期：滤波力不变（innovation 为 0，z2 保持 0），积分修正继续累加。
    command = controller.apply(
        data,
        approach_position=0.5,
        total_normal_force_n=0.4,
        left_normal_force_n=0.2,
        right_normal_force_n=0.2,
        dt=0.002,
    )
    assert controller._adrc_z2 == 0.0
    assert controller._adrc_u == pytest.approx(0.02)
    assert command.position_adjustment == pytest.approx(2 * 0.02 / jacobian * 0.002)

    # 力升高后 innovation 非零，LESO 的扰动估计 z2 开始积累。
    controller.apply(
        data,
        approach_position=0.5,
        total_normal_force_n=0.8,
        left_normal_force_n=0.4,
        right_normal_force_n=0.4,
        dt=0.002,
    )
    assert controller._adrc_z2 > 0.0


def test_normal_force_controller_resets_adrc_state_on_release_and_recontact() -> None:
    """释放退出跟踪与重新确认接触都会把 LADRC 状态复位回跟踪起点。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    adrc_profile = _profile_with_adrc(source, _ADRC_TEST_CONFIG)
    model = mujoco.MjModel.from_xml_path(str(source.model_path))
    data = mujoco.MjData(model)
    controller = NormalForceController.from_profile(model, adrc_profile)

    for _ in range(8):
        controller.apply(
            data,
            approach_position=0.5,
            total_normal_force_n=0.4,
            left_normal_force_n=0.2,
            right_normal_force_n=0.2,
            dt=0.002,
        )
    assert controller.state == "force_tracking"
    assert controller._adrc_adjustment > 0.0

    assert source.normal_force is not None
    for _ in range(source.normal_force.release_confirm_steps):
        command = controller.apply(
            data,
            approach_position=0.5,
            total_normal_force_n=0.0,
            left_normal_force_n=0.0,
            right_normal_force_n=0.0,
            dt=0.002,
        )
    assert command.state == "approach"
    # 释放复位：z1/z2 与积分位置修正全部清零。
    assert controller._adrc_z1 == 0.0
    assert controller._adrc_z2 == 0.0
    assert controller._adrc_adjustment == 0.0

    # 重新确认接触后，LADRC 再次从当前滤波力起步，积分修正从零重新累积。
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
    assert controller._adrc_z1 == pytest.approx(0.2)
    assert controller._adrc_z2 == 0.0
    jacobian = command.closure_jacobian_m_per_rad
    assert jacobian is not None and jacobian > 0.0
    assert command.position_adjustment == pytest.approx(0.02 / jacobian * 0.002)


def test_adrc_none_keeps_pid_tracking_path() -> None:
    """adrc=None（默认与显式置空）时跟踪输出仍走原 PID 路径且完全一致。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    assert source.normal_force is not None
    assert source.normal_force.adrc is None
    explicit_none_profile = _profile_with_adrc(source, None)
    model = mujoco.MjModel.from_xml_path(str(source.model_path))
    default_controller = NormalForceController.from_profile(model, source)
    explicit_controller = NormalForceController.from_profile(model, explicit_none_profile)
    default_data = mujoco.MjData(model)
    explicit_data = mujoco.MjData(model)

    forces = [(0.2, 0.2)] * 8 + [(3.0, 2.6), (4.2, 3.6), (4.0, 4.0)]
    default_command = None
    explicit_command = None
    for left, right in forces:
        default_command = default_controller.apply(
            default_data,
            approach_position=0.5,
            total_normal_force_n=left + right,
            left_normal_force_n=left,
            right_normal_force_n=right,
            dt=0.002,
        )
        explicit_command = explicit_controller.apply(
            explicit_data,
            approach_position=0.5,
            total_normal_force_n=left + right,
            left_normal_force_n=left,
            right_normal_force_n=right,
            dt=0.002,
        )

    assert default_command is not None
    assert explicit_command is not None
    assert default_command == explicit_command
    # PID 外环照常参与：PID 修正非零，位置弹簧项保留在 MIT 力矩中。
    assert default_command.pid_position_adjustment > 0.0
    assert default_command.mit.torque != pytest.approx(default_command.mit.feedforward_torque)


def test_adrc_and_direct_torque_are_mutually_exclusive() -> None:
    """adrc 与 torque_feedback_gain>0 同时启用时，控制器构造直接拒绝。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    assert source.normal_force is not None
    force = source.normal_force.model_copy(
        update={"adrc": _ADRC_TEST_CONFIG, "torque_feedback_gain": 1.0}
    )
    conflicting = source.model_copy(
        update={"control": source.control.model_copy(update={"force": force})}
    )
    model = mujoco.MjModel.from_xml_path(str(source.model_path))

    with pytest.raises(ValueError, match="mutually exclusive"):
        NormalForceController.from_profile(model, conflicting)

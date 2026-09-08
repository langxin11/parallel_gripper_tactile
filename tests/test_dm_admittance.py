"""验证共享 DM 导纳适配、阶段转换和独立仿真入口。"""

from pathlib import Path

from dm_grasp_core import SecondOrderAdmittance, step_admittance
import mujoco
import pytest

from parallel_gripper_tactile.control import ForceControlObservation, ForceControlReference
from parallel_gripper_tactile.dm_admittance import DMAdmittanceController
from parallel_gripper_tactile.experiments.force_tracking import (
    ForceTrackingTask,
    configure_force_controller,
    run_force_tracking,
)
from parallel_gripper_tactile.profiles import DMAdmittanceControl, load_profile
from parallel_gripper_tactile.scenes.custom import GRIPPER_PREFIX, build_custom_grasp_model

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/custom_parallel_gripper_admittance.yaml"


def _controller():
    """创建不推进物理时钟的真实 MuJoCo 执行器适配器。"""
    profile = load_profile(PROFILE)
    model = build_custom_grasp_model(profile)
    data = mujoco.MjData(model)
    controller = DMAdmittanceController.from_profile(model, profile, name_prefix=GRIPPER_PREFIX)
    return controller, data


def _observation(now=0.0, left=0.4, right=0.6):
    """提供有限的平均单侧力输入。"""
    return ForceControlObservation(now, 0.0, left + right, left, right, 0.004)


def test_admittance_rejects_zero_mit_gain_before_first_step():
    """导纳依赖位置增益修正合成力矩，构造时拒绝零增益而非运行后失败。"""
    controller, _ = _controller()
    profile = load_profile(PROFILE)
    with pytest.raises(ValueError, match="positive MIT kp"):
        DMAdmittanceController(
            controller.motor, profile.normal_force, profile.mit.model_copy(update={"kp": 0.0})
        )


def test_tracking_matches_shared_step_before_protocol_quantization():
    """适配器跟踪请求必须与独立共享步骤相等，实际写入保留仿真量化。"""
    controller, data = _controller()
    controller._start_approach(0.0, 0.0)
    controller.state = "force_tracking"
    expected_admittance = SecondOrderAdmittance(0.02, 0.2, 1.0)
    for i in range(10):
        target = 0.6
        expected = step_admittance(
            expected_admittance,
            controller.kinematics,
            controller.command_config,
            reference_position_rad=0.0,
            measured_position_rad=0.0,
            measured_velocity_rad_s=0.0,
            left_force_n=0.4,
            right_force_n=0.6,
            target_force_n=target,
            dt_s=0.004,
        )
        output = controller.step(
            data, observation=_observation(i * 0.004), reference=ForceControlReference(target)
        )
        assert controller.last_requested_command == expected
        assert controller.admittance == expected_admittance
        assert data.ctrl[controller.actuator_id] == output.mit.torque
        assert output.filtered_force_n == 0.5


def test_approach_transition_contact_loss_and_reset():
    """接触确认后经过速度过渡，单侧脱离重新接近，复位不残留状态。"""
    controller, data = _controller()
    reference = ForceControlReference(0.5)
    for now in (0.0, 0.05, 0.101):
        controller.step(data, observation=_observation(now), reference=reference)
    assert controller.state == "contact_transition"
    controller.step(data, observation=_observation(0.252), reference=reference)
    assert controller.state == "force_tracking"
    output = controller.step(data, observation=_observation(0.256, left=0.1), reference=reference)
    assert output.state == "approach"
    assert controller.admittance.displacement_m == 0.0
    controller.reset()
    assert controller.trajectory is None
    assert controller.last_requested_command is None


@pytest.mark.parametrize(
    "update",
    [
        {"mass_kg": 0},
        {"damping_ns_m": -1},
        {"position_max_rad": 0},
        {"closing_direction": 0},
        {"approach_velocity_rad_s": 1},
        {"feedforward_ratio": float("nan")},
    ],
)
def test_invalid_admittance_config_is_rejected(update):
    """质量、符号、范围与有限性约束在配置入口拒绝。"""
    with pytest.raises(ValueError):
        DMAdmittanceControl(**update)


def test_admittance_variant_is_explicit_and_average_side_only():
    """带导纳的 profile 不能静默进入旧 PID 入口，总力语义被拒绝。"""
    profile = load_profile(PROFILE)
    with pytest.raises(ValueError, match="controller-variant"):
        configure_force_controller(profile)
    controller, _data = _controller()
    with pytest.raises(ValueError, match="average_side"):
        DMAdmittanceController(
            controller.motor, profile.normal_force, profile.mit, force_semantics="total"
        )


def test_mujoco_force_tracking_admittance_smoke(tmp_path):
    """4 ms 仿真入口完成接近并输出有限跟踪指标，测试不评价硬件稳定性。"""
    task = ForceTrackingTask.load(ROOT / "configs/force_tracking/dm_admittance.yaml")
    result = run_force_tracking(
        PROFILE, task=task, controller_variant="admittance", output_csv=tmp_path / "trace.csv"
    )
    assert result.simulation_stable
    assert result.passed
    assert result.tracking_duration_s > 0


def test_held_command_recomputes_inner_torque_without_advancing_outer_state():
    """物理步只重算 MIT 内环，不额外积分导纳或重新采样接近轨迹。"""
    controller, data = _controller()
    first = controller.step(data, observation=_observation(), reference=ForceControlReference(0.5))
    requested = controller.last_requested_command
    state = (controller.admittance.displacement_m, controller.admittance.velocity_m_s)
    data.qvel[:] = 0.1
    held = controller.apply_held_command(data)
    assert held.torque != first.mit.torque
    assert controller.last_requested_command == requested
    assert (controller.admittance.displacement_m, controller.admittance.velocity_m_s) == state
    assert data.ctrl[controller.actuator_id] == held.torque


def test_explicit_variant_injects_config_without_old_feedback_or_default_matrix():
    """显式变体注入可复现参数并关闭旧反馈，历史默认矩阵不增加条件。"""
    from parallel_gripper_tactile.experiments.force_tracking import CONTROLLER_VARIANTS

    baseline = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(baseline, variant="admittance")
    assert configured.normal_force.admittance is not None
    assert configured.normal_force.kp == configured.normal_force.ki == 0.0
    assert not configured.normal_force.stiffness.enabled
    assert baseline.normal_force.admittance is None
    assert "admittance" not in CONTROLLER_VARIANTS

"""验证 DMgripper PID 与导纳使用同一实验语义和公共接触状态机。"""

import mujoco

from parallel_gripper_tactile.control import (
    ForceControlObservation,
    ForceControlReference,
    NormalForceController,
)
from parallel_gripper_tactile.dm_admittance import DMAdmittanceController
from parallel_gripper_tactile.research import compose_research_run
from parallel_gripper_tactile.scenes.custom import GRIPPER_PREFIX, build_custom_grasp_model


def _resolved(experiment: str):
    """组合一条不推进仿真的统一 DMgripper 实验。"""
    return compose_research_run(
        experiment=experiment,
        overrides=("execution=plan", "seed=0"),
    )


def test_all_dm_force_controllers_enable_shared_contact_supervisor() -> None:
    """全部 DMgripper 力控制器必须启用同一组公共接触阶段参数。"""
    controllers = {
        "pid_only": "none",
        "pid_torque_ff": "window_linear",
        "pid_stiffness_limit": "window_linear",
        "adrc_torque": "window_linear",
        "admittance": "none",
    }

    for controller, estimator in controllers.items():
        resolved = compose_research_run(
            experiment="dm_gripper/force_tracking",
            overrides=(
                f"controller=dm_gripper/{controller}",
                f"estimator={estimator}",
                "execution=plan",
                "seed=0",
            ),
        )
        supervisor = resolved.profile.normal_force.supervisor
        assert supervisor is not None, controller
        assert supervisor.contact_stable_time_s == 0.0
        assert supervisor.contact_transition_time_s == 0.05
        assert supervisor.release_policy == "any_side"


def test_unified_pid_and_admittance_share_task_inner_loop_and_supervisor() -> None:
    """两种控制律之外的任务、MIT 内环与接触阶段参数必须逐项一致。"""
    pid = compose_research_run(
        experiment="dm_gripper/force_tracking_admittance",
        overrides=("controller=dm_gripper/pid_only", "execution=plan", "seed=0"),
    )
    admittance = _resolved("dm_gripper/force_tracking_admittance")

    assert pid.task == admittance.task
    assert pid.task.name == "ramp_force_tracking"
    assert pid.task.approach.duration_s == 1.0
    assert pid.task.approach.timeout_s == 3.0
    assert [point.force_n for point in pid.task.reference.waypoints] == [1.0, 3.0, 6.0, 1.0, 1.0]
    assert pid.task.control_period_s == 0.004
    assert pid.profile.mit == admittance.profile.mit
    assert pid.profile.normal_force.contact_threshold_n == 0.15
    assert pid.profile.normal_force.contact_threshold_n == (
        admittance.profile.normal_force.contact_threshold_n
    )
    assert pid.profile.normal_force.release_threshold_n == (
        admittance.profile.normal_force.release_threshold_n
    )
    assert pid.profile.normal_force.supervisor == admittance.profile.normal_force.supervisor
    assert pid.profile.normal_force.supervisor.release_policy == "any_side"
    assert pid.profile.normal_force.supervisor.contact_transition_time_s == 0.05
    assert pid.profile.normal_force.admittance is None
    assert admittance.profile.normal_force.admittance is not None


def test_unified_pid_and_admittance_emit_equal_approach_command() -> None:
    """公共接近阶段在相同观测下生成逐字段相同的 MIT 命令。"""
    pid = compose_research_run(
        experiment="dm_gripper/force_tracking_admittance",
        overrides=("controller=dm_gripper/pid_only", "execution=plan", "seed=0"),
    )
    admittance = _resolved("dm_gripper/force_tracking_admittance")
    pid_model = build_custom_grasp_model(pid.profile)
    admittance_model = build_custom_grasp_model(admittance.profile)
    pid_data = mujoco.MjData(pid_model)
    admittance_data = mujoco.MjData(admittance_model)
    pid_controller = NormalForceController.from_profile(
        pid_model,
        pid.profile,
        name_prefix=GRIPPER_PREFIX,
    )
    admittance_controller = DMAdmittanceController.from_profile(
        admittance_model,
        admittance.profile,
        name_prefix=GRIPPER_PREFIX,
    )
    observation = ForceControlObservation(
        time_s=0.0,
        approach_position=0.1,
        total_normal_force_n=0.0,
        left_normal_force_n=0.0,
        right_normal_force_n=0.0,
        dt=0.004,
        approach_velocity=0.05,
    )
    reference = ForceControlReference(
        target_force_n=1.0,
        approach_feedforward_force_n=1.0,
    )

    pid_command = pid_controller.step(
        pid_data,
        observation=observation,
        reference=reference,
    )
    admittance_command = admittance_controller.step(
        admittance_data,
        observation=observation,
        reference=reference,
    )

    assert pid_command.state == admittance_command.state == "approach"
    assert pid_command.mit == admittance_command.mit

    for index in range(1, 6):
        contact_observation = ForceControlObservation(
            time_s=index * 0.004,
            approach_position=0.1,
            total_normal_force_n=0.4,
            left_normal_force_n=0.2,
            right_normal_force_n=0.2,
            dt=0.004,
            approach_velocity=0.05,
        )
        pid_command = pid_controller.step(
            pid_data,
            observation=contact_observation,
            reference=reference,
        )
        admittance_command = admittance_controller.step(
            admittance_data,
            observation=contact_observation,
            reference=reference,
        )
        assert pid_command.state == admittance_command.state
        assert pid_command.mit == admittance_command.mit
    assert pid_command.state == "contact_transition"

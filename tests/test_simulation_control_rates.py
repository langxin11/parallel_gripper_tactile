"""验证两类夹爪的仿真控制频率默认契约。"""

from inspect import signature

import pytest

from parallel_gripper_tactile.experiments.force_scheduling import ForceSchedulingTask
from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
from parallel_gripper_tactile.experiments.friction_estimation import FrictionEstimationTask
from parallel_gripper_tactile.experiments.grasp import run_acceptance
from parallel_gripper_tactile.experiments.grasp_video import record_custom_grasp_video
from parallel_gripper_tactile.experiments.robotiq_discrete_force import (
    RobotiqDiscreteForceTask,
)
from parallel_gripper_tactile.experiments.stiffness_calibration import (
    StiffnessCalibrationTask,
)
from parallel_gripper_tactile.experiments.tangential_disturbance import (
    TangentialDisturbanceTask,
)


@pytest.mark.parametrize(
    "task_type",
    [
        ForceSchedulingTask,
        ForceTrackingTask,
        FrictionEstimationTask,
        StiffnessCalibrationTask,
        TangentialDisturbanceTask,
    ],
)
def test_dm_task_models_default_to_250hz(task_type: type[object]) -> None:
    """所有 DM 任务模型默认使用 4 ms 控制周期。"""
    assert task_type.model_fields["control_period_s"].default == pytest.approx(0.004)


def test_dm_grasp_helpers_default_to_250hz() -> None:
    """DM 抓取验收与录制入口默认使用 250 Hz。"""
    assert signature(run_acceptance).parameters["control_period_s"].default == pytest.approx(0.004)
    assert signature(record_custom_grasp_video).parameters[
        "control_period_s"
    ].default == pytest.approx(0.004)


def test_robotiq_discrete_task_defaults_to_30hz() -> None:
    """Robotiq 离散力任务保持独立的 30 Hz 默认值。"""
    assert RobotiqDiscreteForceTask.model_fields["control_period_s"].default == pytest.approx(
        1.0 / 30.0
    )

"""验证 Robotiq 离散力任务配置和场景参数。"""

from pathlib import Path

import pytest
from pydantic import ValidationError

from parallel_gripper_tactile.experiments.robotiq_discrete_force import (
    RobotiqDiscreteForceTask,
)
from parallel_gripper_tactile.profiles import load_profile
from parallel_gripper_tactile.scenes.robotiq import load_grasp_model


ROOT = Path(__file__).resolve().parents[1]


def test_recommended_task_maps_to_integer_controller_config() -> None:
    """推荐 YAML 完整映射方案中的整数命令和自适应参数。"""
    task = RobotiqDiscreteForceTask.load(ROOT / "configs/discrete_force/robotiq_delta_f_tick.yaml")
    control = task.controller_config()

    assert (control.command_min, control.command_max) == (0, 255)
    assert control.approach_step == 3
    assert control.max_dynamic_step == 3
    assert control.target_force_n == pytest.approx(2.0)
    assert task.reference.duration_s == pytest.approx(18.5)
    assert task.control_period_s == pytest.approx(1.0 / 30.0)
    assert [interval[2] for interval in task.reference.platform_intervals()] == [
        2.0,
        4.0,
        6.0,
        8.0,
        6.0,
        4.0,
        2.0,
    ]
    assert control.delta_f_ema_alpha == pytest.approx(0.3)
    assert control.tick_deadband_factor == pytest.approx(0.6)
    assert control.reactivate_tick_factor == pytest.approx(0.7)
    assert task.controller_variant == "dynamic-step"


def test_task_rejects_unknown_fields_and_unsafe_force_limit() -> None:
    """配置拼写错误和低于目标的安全力上限不能进入仿真。"""
    with pytest.raises(ValidationError):
        RobotiqDiscreteForceTask.model_validate(
            {
                "schema_version": 1,
                "name": "invalid",
                "reference": {
                    "waypoints": [
                        {"t_s": 0.0, "force_n": 2.0},
                        {"t_s": 1.0, "force_n": 5.0},
                    ]
                },
                "control": {"max_force_n": 4.0},
            }
        )
    with pytest.raises(ValidationError):
        RobotiqDiscreteForceTask.model_validate(
            {
                "schema_version": 1,
                "name": "invalid",
                "reference": {
                    "waypoints": [
                        {"t_s": 0.0, "force_n": 2.0},
                        {"t_s": 1.0, "force_n": 2.0},
                    ]
                },
                "typo": True,
            }
        )


@pytest.mark.parametrize("material", ["soft", "medium", "hard", "stiff"])
def test_robotiq_material_scenes_compile_with_full_command_range(material: str) -> None:
    """四种刚度场景均保留 0～255 actuator 命令范围。"""
    profile = load_profile(ROOT / "configs/robotiq_2f85.yaml")
    model = load_grasp_model(None, profile.model_path, object_material=material)
    actuator = model.actuator("gripper/fingers_actuator").id

    assert model.actuator_ctrlrange[actuator].tolist() == [0.0, 255.0]
    assert model.npair >= 18
    assert model.opt.timestep == pytest.approx(0.002)

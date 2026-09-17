"""验证 Robotiq 离散力任务配置和场景参数。"""

from pathlib import Path

import pytest
from pydantic import ValidationError

from parallel_gripper_tactile.experiments.robotiq_discrete_force import (
    _TRACE_COLUMNS,
    _equivalent_actuator_position,
    _plot_trace,
    _should_keep_trace_row,
    RobotiqDiscreteForceTask,
)
from parallel_gripper_tactile.config.profiles import load_profile
from parallel_gripper_tactile.scenes.robotiq import load_grasp_model


ROOT = Path(__file__).resolve().parents[1]


def test_trace_plot_renders_command_steps_with_action_markers(
    tmp_path: Path, fast_png_render: None
) -> None:
    """命令面板以同一坐标轴上的阶梯和动作标记呈现。"""
    rows = [
        {
            "time_s": time_s,
            "force_filtered_n": force,
            "force_target_n": 2.0,
            "u": command,
            "delta_u": delta,
            "delta_f_tick_est_n": 0.12,
            "hold_deadband_n": 0.08,
            "reactivate_threshold_n": 0.18,
        }
        for time_s, force, command, delta in (
            (0.0, 0.2, 10, 1),
            (0.1, 1.8, 11, 0),
            (0.2, 2.0, 10, -1),
        )
    ]
    output = tmp_path / "trace.png"

    _plot_trace(rows, output)

    assert output.is_file()
    assert output.stat().st_size > 0


def test_equivalent_actuator_position_uses_mujoco_affine_scale() -> None:
    """actuator length 应按增益与偏置换算到同向整数命令坐标。"""
    assert _equivalent_actuator_position(0.8, 0.3137255, -100.0) == pytest.approx(255.0)
    with pytest.raises(ValueError, match="finite and positive"):
        _equivalent_actuator_position(0.8, 0.0, -100.0)
    with pytest.raises(ValueError, match="finite and positive"):
        _equivalent_actuator_position(0.8, 1.0, 100.0)
    assert _equivalent_actuator_position(0.9, 0.3137255, -100.0) == 255.0


def test_trace_columns_cover_position_prediction_and_settled_action_diagnostics() -> None:
    """压缩 trace 保留旧列并公开位置、候选动作和稳定动作诊断。"""
    assert {
        "u",
        "delta_u",
        "requested_delta_u",
        "force_filtered_n",
        "u_request",
        "u_actual_command",
        "p_actual",
        "e_position",
        "selected_action",
        "candidate_cost_m3",
        "candidate_cost_m2",
        "candidate_cost_m1",
        "candidate_cost_hold",
        "candidate_cost_p1",
        "candidate_cost_p2",
        "candidate_cost_p3",
        "model_valid",
        "model_sample_count",
        "settled_action_id",
        "settled_delta_u",
        "delta_p",
        "delta_e_position",
        "delta_f",
        "q_tick_measured",
        "rho_p",
    }.issubset(_TRACE_COLUMNS)


@pytest.mark.parametrize(
    ("event", "expected"),
    [
        ({}, False),
        ({"is_first": True}, True),
        ({"is_last": True}, True),
        ({"record_tick": True}, True),
        ({"action_applied": True}, True),
        ({"settled_action_changed": True}, True),
        ({"state_changed": True}, True),
    ],
)
def test_trace_sampling_keeps_record_ticks_and_key_events(
    event: dict[str, bool], expected: bool
) -> None:
    """普通物理步不落完整行，独立记录 tick 和关键事件必须留样。"""
    flags = {
        "is_first": False,
        "is_last": False,
        "record_tick": False,
        "action_applied": False,
        "settled_action_changed": False,
        "state_changed": False,
    }
    flags.update(event)
    assert _should_keep_trace_row(**flags) is expected


def test_recommended_task_maps_to_integer_controller_config() -> None:
    """推荐 YAML 完整映射方案中的整数命令和自适应参数。"""
    task = RobotiqDiscreteForceTask.load(
        ROOT / "configs/task/discrete_force/robotiq_delta_f_tick.yaml"
    )
    control = task.controller_config()

    assert (control.command_min, control.command_max) == (0, 255)
    assert control.approach_step == 3
    assert control.max_dynamic_step == 3
    assert control.target_force_n == pytest.approx(2.0)
    assert task.reference.duration_s == pytest.approx(18.5)
    assert task.control_period_s == pytest.approx(1.0 / 30.0)
    assert task.record_period_s == pytest.approx(0.01)
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
    assert control.tick_deadband_factor == pytest.approx(0.5)
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
    assert model.opt.timestep == pytest.approx(0.001)

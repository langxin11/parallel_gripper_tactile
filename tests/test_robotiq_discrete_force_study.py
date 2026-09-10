"""验证 Robotiq 离散力控制 study 的矩阵与正式协议计划。"""

from pathlib import Path

from parallel_gripper_tactile.studies.protocols import robotiq_discrete_force as protocol
from parallel_gripper_tactile.studies.robotiq_discrete_force import (
    load_robotiq_discrete_force_study_config,
)


ROOT = Path(__file__).resolve().parents[1]


def test_default_study_expands_complete_ablation_matrix() -> None:
    """默认矩阵以单条主曲线覆盖五种控制器、四种刚度和三种噪声。"""
    config = load_robotiq_discrete_force_study_config(
        ROOT / "configs/studies/robotiq_discrete_force.yaml"
    )
    conditions = config.conditions()

    assert len(conditions) == 5 * 4 * 3
    assert conditions[0] == ("quantized-pi", "soft", 0.0, 0)
    assert conditions[-1] == ("dynamic-step", "stiff", 0.1, 0)
    assert config.profile == (ROOT / "configs/robotiq_2f85.yaml").resolve()
    assert config.task == (ROOT / "configs/discrete_force/robotiq_delta_f_tick.yaml").resolve()


def test_protocol_plan_preserves_matrix_and_baseline_role() -> None:
    """协议计划展开 60 条条件，且仅量化 PI 对照行登记基线角色。"""
    config = load_robotiq_discrete_force_study_config(
        ROOT / "configs/studies/robotiq_discrete_force.yaml"
    )
    plan = protocol.build_plan(config)

    assert len(plan.conditions) == len(config.conditions())
    assert plan.conditions[0].condition_id == "quantized-pi-soft-noise0.0-seed000"
    assert plan.conditions[-1].condition_id == "dynamic-step-stiff-noise0.1-seed000"
    assert plan.conditions[-1].pair_key == "stiff:noise0.1:seed000"
    assert all(
        (condition.baseline_role == "quantized_pi")
        == (condition.parameters["controller_variant"] == "quantized-pi")
        for condition in plan.conditions
    )

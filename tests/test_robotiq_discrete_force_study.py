"""验证 Robotiq 离散力控制 study 的矩阵和聚合。"""

from pathlib import Path

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

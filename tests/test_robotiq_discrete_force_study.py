"""验证 Robotiq 离散力控制 study 的矩阵、聚合与并行度。"""

import importlib.util
from pathlib import Path
import sys

from parallel_gripper_tactile.studies.robotiq_discrete_force import (
    load_robotiq_discrete_force_study_config,
)


ROOT = Path(__file__).resolve().parents[1]


def _protocol_module() -> object:
    """加载 Robotiq 离散力 study 入口脚本。"""
    path = ROOT / "scripts/experiments/robotiq_discrete_force.py"
    spec = importlib.util.spec_from_file_location("robotiq_discrete_force_protocol", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


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


def test_default_worker_count_uses_bounded_parallelism() -> None:
    """默认并行度受 CPU、条件数量和十二进程上限共同约束。"""
    protocol = _protocol_module()

    assert protocol._resolve_worker_count(None, 60, available_cpus=32) == 12
    assert protocol._resolve_worker_count(None, 60, available_cpus=4) == 4
    assert protocol._resolve_worker_count(None, 3, available_cpus=32) == 3
    assert protocol._resolve_worker_count(1, 60, available_cpus=32) == 1

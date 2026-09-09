"""验证 Robotiq 离散控制器的旧导入兼容性。"""

from parallel_gripper_tactile.discrete_force_control import (
    DiscreteControlState as LegacyState,
    DiscreteForceControlConfig as LegacyConfig,
    DiscreteForceController as LegacyController,
)
from robotiq_grasp_core import (
    DiscreteControlState,
    DiscreteForceControlConfig,
    DiscreteForceController,
)


def test_legacy_module_reexports_workspace_core_objects() -> None:
    """旧模块应导出 workspace 核心中的同一组对象。"""
    assert LegacyState is DiscreteControlState
    assert LegacyConfig is DiscreteForceControlConfig
    assert LegacyController is DiscreteForceController


def test_legacy_controller_keeps_basic_behavior() -> None:
    """旧导入路径创建的控制器应保留基本初始行为。"""
    controller = LegacyController(LegacyConfig())
    assert controller.snapshot().state == DiscreteControlState.APPROACH
    assert controller.decide(0.0) == controller.config.approach_step

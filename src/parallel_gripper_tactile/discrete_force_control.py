"""Robotiq 离散力控制器的兼容导出，将于 0.4.0 移除。"""

import warnings

from robotiq_grasp_core.discrete_force_control import (
    DiscreteControllerVariant,
    DiscreteControlSnapshot,
    DiscreteControlState,
    DiscreteForceControlConfig,
    DiscreteForceController,
)

warnings.warn(
    "parallel_gripper_tactile.discrete_force_control 已弃用，将于 0.4.0 移除；"
    "请改用 robotiq_grasp_core.discrete_force_control。",
    DeprecationWarning,
    stacklevel=2,
)

del warnings

__all__ = [
    "DiscreteControllerVariant",
    "DiscreteControlSnapshot",
    "DiscreteControlState",
    "DiscreteForceControlConfig",
    "DiscreteForceController",
]

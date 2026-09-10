"""兼容入口：请从 ``parallel_gripper_tactile.perception.friction`` 导入，将于 0.4.0 移除。"""

import warnings

from .perception import friction as _implementation

warnings.warn(
    "parallel_gripper_tactile.friction_estimation 已弃用，将于 0.4.0 移除；"
    "请改用 parallel_gripper_tactile.perception.friction。",
    DeprecationWarning,
    stacklevel=2,
)

del warnings

# 原模块没有声明 ``__all__``；完整转发以保持历史直接与星号导入行为。
__all__ = [name for name in dir(_implementation) if not name.startswith("_")]
globals().update({name: getattr(_implementation, name) for name in __all__})

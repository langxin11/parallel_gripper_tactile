"""兼容入口：请从 ``parallel_gripper_tactile.perception.slip`` 导入。"""

from .perception import slip as _implementation


# 原模块没有声明 ``__all__``；完整转发以保持历史直接与星号导入行为。
__all__ = [name for name in dir(_implementation) if not name.startswith("_")]
globals().update({name: getattr(_implementation, name) for name in __all__})

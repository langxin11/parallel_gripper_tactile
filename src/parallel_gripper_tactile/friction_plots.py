"""兼容旧摩擦估计绘图模块导入路径，将于 0.4.0 移除。"""

import warnings

from .visualization import friction as _implementation

warnings.warn(
    "parallel_gripper_tactile.friction_plots 已弃用，将于 0.4.0 移除；"
    "请改用 parallel_gripper_tactile.visualization.friction。",
    DeprecationWarning,
    stacklevel=2,
)

del warnings

# 原模块未声明 ``__all__``；完整转发以保持历史直接与星号导入行为。
for _name in dir(_implementation):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_implementation, _name)

del _name
del _implementation

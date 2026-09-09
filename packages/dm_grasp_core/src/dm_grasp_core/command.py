"""兼容旧 MIT 命令模块导入路径。"""

from .grasp import command as _implementation


# 原模块未声明 ``__all__``；完整转发以保持历史直接与星号导入行为。
for _name in dir(_implementation):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_implementation, _name)

del _name
del _implementation

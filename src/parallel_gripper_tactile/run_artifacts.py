"""兼容旧运行产物模块导入路径。"""

from .artifacts import run_artifacts as _implementation

# 旧模块没有定义 ``__all__``，其历史调用方可能直接导入任意非私有名称。
# 完整转发这些名称，避免迁移改变既有导入对象、可见性或星号导入行为。
for _name in dir(_implementation):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_implementation, _name)

del _name
del _implementation

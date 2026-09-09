"""触觉力轨迹的读取与基础分析接口。"""

from . import force_csv as _implementation


# 原 ``analysis.py`` 未声明 ``__all__``；完整转发其非私有名称，保持旧路径的
# 直接与星号导入行为。实现位于 ``force_csv``，避免文件与包同名冲突。
for _name in dir(_implementation):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_implementation, _name)

del _name
del _implementation

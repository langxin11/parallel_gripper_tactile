"""兼容旧 profile 配置导入路径，将于 0.4.0 移除。"""

import warnings

from .config.profiles import *  # noqa: F403
from .config.profiles import __all__ as __all__

warnings.warn(
    "parallel_gripper_tactile.profiles 已弃用，将于 0.4.0 移除；请改用 parallel_gripper_tactile.config.profiles。",
    DeprecationWarning,
    stacklevel=2,
)

del warnings

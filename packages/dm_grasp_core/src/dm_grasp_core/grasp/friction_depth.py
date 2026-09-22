"""由接触压缩深度建立经验摩擦先验，不解释为物理摩擦下界。"""

from dataclasses import dataclass, fields
import math
from numbers import Real

import numpy as np


@dataclass(frozen=True, slots=True)
class DepthFrictionPriorConfig:
    """对数曲线、接触确认及采用速率；深度统一使用米。

    关节总压缩量由调用方按双侧接触基线换算。兼容逐点位移入口须先
    完成空载零点标定，compression_sign 指定其 dz 压缩方向，不取绝对值。
    """

    min_friction: float = 0.1
    max_friction: float = 0.6
    onset_depth_m: float = 0.0002
    saturation_depth_m: float = 0.002
    curvature: float = 2.0
    compression_sign: float = 1.0
    min_contact_taxels: int = 4
    stable_time_s: float = 0.2
    depth_tolerance_m: float = 0.0001
    max_increase_per_s: float | None = 0.05

    def __post_init__(self) -> None:
        """拒绝非有限参数、反向区间及错误触点计数。"""
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name == "max_increase_per_s" and value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
                raise ValueError(f"{item.name} 必须为有限数值")
            if item.name not in {"compression_sign", "onset_depth_m"} and value <= 0:
                raise ValueError(f"{item.name} 必须为正数")
        if not 0 <= self.onset_depth_m < self.saturation_depth_m:
            raise ValueError("深度区间必须非负且严格递增")
        if not self.min_friction < self.max_friction:
            raise ValueError("深度摩擦先验区间必须严格递增")
        if self.compression_sign not in (-1.0, 1.0):
            raise ValueError("compression_sign 必须为正一或负一")
        if not isinstance(self.min_contact_taxels, int) or not 1 <= self.min_contact_taxels <= 9:
            raise ValueError("min_contact_taxels 必须为一至九的整数")

    def friction_at(self, depth_m: float) -> float:
        """返回带上下限的对数先验；负深度按尚未压入处理。"""
        if isinstance(depth_m, bool) or not isinstance(depth_m, Real) or not math.isfinite(depth_m):
            raise ValueError("压入深度必须有限")
        x = min(
            1.0,
            max(
                0.0, (depth_m - self.onset_depth_m) / (self.saturation_depth_m - self.onset_depth_m)
            ),
        )
        return self.min_friction + (self.max_friction - self.min_friction) * (
            math.log1p(self.curvature * x) / math.log1p(self.curvature)
        )


class DepthFrictionPrior:
    """每侧在可靠接触窗口后锁定先验，避免深度变化反复覆盖后验。"""

    def __init__(self, config: DepthFrictionPriorConfig) -> None:
        """保存配置并初始化接触段。"""
        self.config = config
        self.reset()

    def reset(self) -> None:
        """清除旧接触段的压入深度和锁定值。"""
        self.depth_m: float | None = None
        self.candidate = self.config.min_friction
        self.value = self.config.min_friction
        self.locked = False
        self.reason = "awaiting_depth"
        self._anchor: float | None = None
        self._since: float | None = None

    def observe(
        self,
        displacements_m,
        forces,
        valid_mask,
        *,
        eligible: bool,
        time_s: float,
        contact_depth_m: float | None = None,
    ) -> bool:
        """优先采用关节总压缩量，否则用逐点加权深度；返回是否首次锁定。"""
        c = self.config
        weights = np.asarray(forces, dtype=float)[:, 2]
        mask = np.asarray(valid_mask, dtype=bool) & (weights > 0)
        if contact_depth_m is not None:
            if isinstance(contact_depth_m, bool) or not math.isfinite(contact_depth_m):
                raise ValueError("关节推算压缩量必须有限")
            depth = max(0.0, contact_depth_m)
        else:
            values = np.asarray(displacements_m, dtype=float)
            if values.shape != (9, 3) or not np.isfinite(values).all():
                self.depth_m = None
                self._since = None
                self.reason = "missing_depth"
                return False
            depths = c.compression_sign * values[:, 2]
            mask &= depths >= 0
            depth = None
        if np.count_nonzero(mask) < c.min_contact_taxels:
            self.depth_m = None
            self._since = None
            self.reason = "insufficient_depth_coverage"
            return False
        self.depth_m = (
            depth if depth is not None else float(np.average(depths[mask], weights=weights[mask]))
        )
        self.candidate = c.friction_at(self.depth_m)
        if self.locked:
            self.reason = "locked"
            return False
        if not eligible:
            self._since = None
            self.reason = "awaiting_stable_contact"
            return False
        # 饱和区内候选不再随深度变化，继续压缩不应无限重启确认。
        # 使用同一曲线裁剪区间，斜坡区仍按物理深度容差确认。
        effective_depth = min(c.saturation_depth_m, max(c.onset_depth_m, self.depth_m))
        if self._since is None or abs(effective_depth - self._anchor) > c.depth_tolerance_m:
            self._since, self._anchor = time_s, effective_depth
        if time_s - self._since < c.stable_time_s:
            self.reason = "confirming_depth"
            return False
        self.value = self.candidate
        self.locked = True
        self.reason = "locked"
        return True

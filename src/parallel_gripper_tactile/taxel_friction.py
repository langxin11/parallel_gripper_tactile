"""从逐 taxel 三轴力生成带接触滞回的局部摩擦利用率观测。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True, slots=True)
class TaxelFrictionConfig:
    """逐 taxel 接触筛选与局部摩擦比参数。"""

    contact_enter_force_n: float = 0.05
    contact_exit_force_n: float = 0.025
    transition_confirm_s: float = 0.01

    def __post_init__(self) -> None:
        """校验接触阈值、滞回关系和确认时间。"""
        values = (
            self.contact_enter_force_n,
            self.contact_exit_force_n,
            self.transition_confirm_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("taxel friction parameters must be finite")
        if self.contact_enter_force_n <= 0:
            raise ValueError("contact_enter_force_n must be positive")
        if not 0 <= self.contact_exit_force_n < self.contact_enter_force_n:
            raise ValueError("contact_exit_force_n must be in [0, contact_enter_force_n)")
        if self.transition_confirm_s <= 0:
            raise ValueError("transition_confirm_s must be positive")


@dataclass(frozen=True, slots=True)
class TaxelFrictionObservation:
    """一次逐 taxel 局部摩擦利用率观测。"""

    left_contact_mask: np.ndarray
    right_contact_mask: np.ndarray
    left_ratio: np.ndarray
    right_ratio: np.ndarray
    active_count: int
    weighted_ratio: float
    ratio_p90: float
    maximum_ratio: float
    ratio_spread: float


class TaxelFrictionObserver:
    """以进入/退出滞回和持续时间筛选有效 taxel。"""

    def __init__(self, config: TaxelFrictionConfig | None = None) -> None:
        """创建尚未绑定触觉网格形状的观测器。"""
        self._config = config or TaxelFrictionConfig()
        self._shape: tuple[int, int] | None = None
        self._active: np.ndarray | None = None
        self._transition_duration_s: np.ndarray | None = None

    @property
    def config(self) -> TaxelFrictionConfig:
        """返回不可变配置。"""
        return self._config

    def reset(self) -> None:
        """清除接触状态但保留已绑定的网格形状。"""
        if self._active is not None:
            self._active.fill(False)
        if self._transition_duration_s is not None:
            self._transition_duration_s.fill(0.0)

    def update(
        self,
        left_force: np.ndarray,
        right_force: np.ndarray,
        *,
        dt: float,
    ) -> TaxelFrictionObservation:
        """更新双侧接触状态并返回局部摩擦利用率。

        Args:
            left_force: 左侧局部三轴力，形状为 ``(3, rows, cols)``。
            right_force: 右侧局部三轴力，形状为 ``(3, rows, cols)``。
            dt: 当前观测周期，单位为秒。

        Returns:
            接触掩码、逐点比值与汇总诊断量。

        Raises:
            ValueError: 输入形状、数值或时间步无效时抛出。
        """
        left = np.asarray(left_force, dtype=np.float64)
        right = np.asarray(right_force, dtype=np.float64)
        if left.ndim != 3 or left.shape[0] != 3 or right.shape != left.shape:
            raise ValueError("taxel forces must share shape (3, rows, cols)")
        if not np.isfinite(left).all() or not np.isfinite(right).all():
            raise ValueError("taxel forces must be finite")
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")

        shape = left.shape[1:]
        if self._shape is None:
            self._shape = shape
            self._active = np.zeros((2, *shape), dtype=bool)
            self._transition_duration_s = np.zeros((2, *shape), dtype=np.float64)
        elif shape != self._shape:
            raise ValueError("taxel grid shape must remain constant")
        assert self._active is not None
        assert self._transition_duration_s is not None

        normal = np.maximum(0.0, np.stack((left[2], right[2])))
        shear = np.hypot(np.stack((left[0], right[0])), np.stack((left[1], right[1])))
        transition_requested = np.where(
            self._active,
            normal <= self._config.contact_exit_force_n,
            normal >= self._config.contact_enter_force_n,
        )
        self._transition_duration_s = np.where(
            transition_requested,
            self._transition_duration_s + dt,
            0.0,
        )
        confirmed = self._transition_duration_s + 1e-12 >= self._config.transition_confirm_s
        self._active[confirmed] = ~self._active[confirmed]
        self._transition_duration_s[confirmed] = 0.0

        ratios = np.full(normal.shape, np.nan, dtype=np.float64)
        ratio_valid = self._active & (normal > 0.0)
        ratios[ratio_valid] = shear[ratio_valid] / normal[ratio_valid]
        active_count = int(np.count_nonzero(self._active))
        if np.any(ratio_valid):
            active_normal = normal[ratio_valid]
            active_shear = shear[ratio_valid]
            active_ratios = ratios[ratio_valid]
            weighted_ratio = float(active_shear.sum() / active_normal.sum())
            ratio_p90 = float(np.quantile(active_ratios, 0.9))
            maximum_ratio = float(active_ratios.max())
            ratio_spread = float(active_ratios.max() - active_ratios.min())
        else:
            weighted_ratio = math.nan
            ratio_p90 = math.nan
            maximum_ratio = math.nan
            ratio_spread = math.nan
        return TaxelFrictionObservation(
            left_contact_mask=self._active[0].copy(),
            right_contact_mask=self._active[1].copy(),
            left_ratio=ratios[0].copy(),
            right_ratio=ratios[1].copy(),
            active_count=active_count,
            weighted_ratio=weighted_ratio,
            ratio_p90=ratio_p90,
            maximum_ratio=maximum_ratio,
            ratio_spread=ratio_spread,
        )


__all__ = ["TaxelFrictionConfig", "TaxelFrictionObservation", "TaxelFrictionObserver"]

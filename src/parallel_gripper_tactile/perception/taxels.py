"""从逐 taxel 三轴力生成带接触滞回的局部摩擦利用率观测。"""

from __future__ import annotations

from collections import deque
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
    left_normal_force_n: np.ndarray
    right_normal_force_n: np.ndarray
    left_shear_force_n: np.ndarray
    right_shear_force_n: np.ndarray
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
            left_normal_force_n=normal[0].copy(),
            right_normal_force_n=normal[1].copy(),
            left_shear_force_n=shear[0].copy(),
            right_shear_force_n=shear[1].copy(),
            active_count=active_count,
            weighted_ratio=weighted_ratio,
            ratio_p90=ratio_p90,
            maximum_ratio=maximum_ratio,
            ratio_spread=ratio_spread,
        )


@dataclass(frozen=True, slots=True)
class ForceOnlySlipConfig:
    """纯力局部起滑检测参数。"""

    window_size: int = 200
    min_ratio: float = 0.1
    arming_ratio_increase: float = 0.05
    saturation_ratio_increase: float = 0.005
    redistribution_share_drop: float = 0.05
    min_side_shear_increase_n: float = 0.03
    confirm_s: float = 0.03
    estimate_quantile: float = 0.8
    safety_discount: float = 0.8

    def __post_init__(self) -> None:
        """校验窗口、判据阈值和保守估计参数。"""
        if not isinstance(self.window_size, int) or isinstance(self.window_size, bool):
            raise ValueError("window_size must be an integer")
        if self.window_size < 4 or self.window_size % 2:
            raise ValueError("window_size must be an even integer of at least four")
        values = (
            self.min_ratio,
            self.arming_ratio_increase,
            self.saturation_ratio_increase,
            self.redistribution_share_drop,
            self.min_side_shear_increase_n,
            self.confirm_s,
            self.estimate_quantile,
            self.safety_discount,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("force-only slip parameters must be finite")
        if self.min_ratio < 0 or self.arming_ratio_increase <= 0:
            raise ValueError("ratio thresholds must be non-negative with positive arming")
        if self.saturation_ratio_increase < 0:
            raise ValueError("saturation_ratio_increase must be non-negative")
        if self.redistribution_share_drop <= 0 or self.min_side_shear_increase_n < 0:
            raise ValueError("redistribution thresholds must be positive or zero as specified")
        if self.confirm_s <= 0:
            raise ValueError("confirm_s must be positive")
        if not 0 < self.estimate_quantile <= 1 or not 0 < self.safety_discount <= 1:
            raise ValueError("estimate quantile and safety discount must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class ForceOnlySlipObservation:
    """一次纯力局部起滑检测结果。"""

    left_armed_mask: np.ndarray
    right_armed_mask: np.ndarray
    left_candidate_mask: np.ndarray
    right_candidate_mask: np.ndarray
    left_detected_mask: np.ndarray
    right_detected_mask: np.ndarray
    event_count: int
    detected_count: int
    left_friction_estimate: float | None
    right_friction_estimate: float | None


class ForceOnlyTaxelSlipDetector:
    """用局部力比趋势和剪切重分配检测局部起滑。"""

    def __init__(self, config: ForceOnlySlipConfig | None = None) -> None:
        """创建尚未绑定网格形状的检测器。"""
        self._config = config or ForceOnlySlipConfig()
        self._ratio_history: deque[np.ndarray] = deque(maxlen=self._config.window_size)
        self._shear_history: deque[np.ndarray] = deque(maxlen=self._config.window_size)
        self._active_history: deque[np.ndarray] = deque(maxlen=self._config.window_size)
        self._armed: np.ndarray | None = None
        self._candidate_duration_s: np.ndarray | None = None
        self._detected: np.ndarray | None = None
        self._candidate_estimate: np.ndarray | None = None
        self._estimate: np.ndarray | None = None

    def reset(self) -> None:
        """清除窗口、候选事件和已锁存估计。"""
        self._ratio_history.clear()
        self._shear_history.clear()
        self._active_history.clear()
        for array in (
            self._armed,
            self._candidate_duration_s,
            self._detected,
        ):
            if array is not None:
                array.fill(False if array.dtype == bool else 0.0)
        if self._candidate_estimate is not None:
            self._candidate_estimate.fill(np.nan)
        if self._estimate is not None:
            self._estimate.fill(np.nan)

    def update(
        self,
        observation: TaxelFrictionObservation,
        *,
        dt: float,
    ) -> ForceOnlySlipObservation:
        """用一个周期的逐点力观测更新局部起滑状态。"""
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")
        ratio = np.stack((observation.left_ratio, observation.right_ratio))
        shear = np.stack((observation.left_shear_force_n, observation.right_shear_force_n))
        active = np.stack((observation.left_contact_mask, observation.right_contact_mask))
        if self._armed is None:
            self._armed = np.zeros(ratio.shape, dtype=bool)
            self._candidate_duration_s = np.zeros(ratio.shape)
            self._detected = np.zeros(ratio.shape, dtype=bool)
            self._candidate_estimate = np.full(ratio.shape, np.nan)
            self._estimate = np.full(ratio.shape, np.nan)
        elif ratio.shape != self._armed.shape:
            raise ValueError("taxel grid shape must remain constant")
        assert self._candidate_duration_s is not None
        assert self._detected is not None
        assert self._candidate_estimate is not None
        assert self._estimate is not None

        self._ratio_history.append(ratio.copy())
        self._shear_history.append(shear.copy())
        self._active_history.append(active.copy())
        candidate = np.zeros(ratio.shape, dtype=bool)
        event = np.zeros(ratio.shape, dtype=bool)
        if len(self._ratio_history) == self._config.window_size:
            ratio_window = np.stack(self._ratio_history)
            shear_window = np.stack(self._shear_history)
            active_window = np.stack(self._active_history)
            half = self._config.window_size // 2
            valid_window = active_window & np.isfinite(ratio_window)
            window_valid = valid_window.all(axis=0)
            safe_ratio = np.where(valid_window, ratio_window, 0.0)
            early_ratio = safe_ratio[:half].mean(axis=0)
            recent_ratio = safe_ratio[half:].mean(axis=0)
            ratio_increase = recent_ratio - early_ratio
            self._armed |= (
                window_valid
                & (recent_ratio >= self._config.min_ratio)
                & (ratio_increase >= self._config.arming_ratio_increase)
            )

            side_total = shear_window.sum(axis=(2, 3))
            side_increase = side_total[half:].mean(axis=0) - side_total[:half].mean(axis=0)
            shares = shear_window / np.maximum(side_total[:, :, None, None], 1e-12)
            share_drop = shares[:half].mean(axis=0) - shares[half:].mean(axis=0)
            loading = side_increase[:, None, None] >= self._config.min_side_shear_increase_n
            saturated = ratio_increase <= self._config.saturation_ratio_increase
            redistributed = share_drop >= self._config.redistribution_share_drop
            candidate = (
                self._armed & ~self._detected & window_valid & loading & (saturated | redistributed)
            )
            starting = candidate & (self._candidate_duration_s == 0.0)
            for side, row, col in np.argwhere(starting):
                samples = ratio_window[:half, side, row, col]
                self._candidate_estimate[side, row, col] = self._config.safety_discount * float(
                    np.quantile(samples, self._config.estimate_quantile)
                )
            self._candidate_duration_s = np.where(
                candidate,
                self._candidate_duration_s + dt,
                0.0,
            )
            confirmed = candidate & (self._candidate_duration_s + 1e-12 >= self._config.confirm_s)
            event = confirmed & ~self._detected
            self._detected |= confirmed
            self._estimate[event] = self._candidate_estimate[event]

        side_estimates: list[float | None] = []
        for side in range(2):
            values = self._estimate[side][np.isfinite(self._estimate[side])]
            side_estimates.append(float(np.min(values)) if values.size else None)
        return ForceOnlySlipObservation(
            left_armed_mask=self._armed[0].copy(),
            right_armed_mask=self._armed[1].copy(),
            left_candidate_mask=candidate[0].copy(),
            right_candidate_mask=candidate[1].copy(),
            left_detected_mask=self._detected[0].copy(),
            right_detected_mask=self._detected[1].copy(),
            event_count=int(np.count_nonzero(event)),
            detected_count=int(np.count_nonzero(self._detected)),
            left_friction_estimate=side_estimates[0],
            right_friction_estimate=side_estimates[1],
        )


__all__ = [
    "ForceOnlySlipConfig",
    "ForceOnlySlipObservation",
    "ForceOnlyTaxelSlipDetector",
    "TaxelFrictionConfig",
    "TaxelFrictionObservation",
    "TaxelFrictionObserver",
]

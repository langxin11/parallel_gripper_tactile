"""用接触后的 ``dF/dc`` 样本在线估计等效接触刚度。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Literal

import numpy as np

from .kinematics import CrankSliderKinematics


StiffnessEstimatorMethod = Literal["secant_ewma", "window_linear", "window_quadratic"]


@dataclass(frozen=True, slots=True)
class ContactStiffnessConfig:
    """在线接触刚度估计、前馈与刚度感知位置限幅参数。

    Attributes:
        enabled: 是否启用刚度估计；关闭时控制器不得读取估计值路径。
        method: 估计方法（割线滑动平均或窗口一次／二次拟合）。
        initial_n_per_m: 初始等效接触刚度估计 (N/m)。
        min_n_per_m: 单次样本刚度下限 (N/m)。
        max_n_per_m: 单次样本刚度上限 (N/m)。
        filter_alpha: 估计值的一阶平滑系数。
        min_delta_closure_m: 有效样本要求的最小闭合行程增量 (m)。
        min_delta_force_n: 有效样本要求的最小力增量 (N)。
        window_size: 滑动窗口容量（样本数）。
        min_samples: 触发拟合的最少样本数。
        position_feedforward_gain: 刚度前馈位置修正增益。
        torque_feedforward_gain: 目标力到输出轴力矩的前馈增益。
        position_limit_enabled: 是否启用刚度感知的 PID 位置目标限幅。
        position_limit_force_rate_n_s: 限幅路径允许的法向力变化率 (N/s)。
        position_limit_stiffness_safety_factor: 限幅路径的刚度安全系数。
    """

    enabled: bool
    initial_n_per_m: float
    min_n_per_m: float
    max_n_per_m: float
    filter_alpha: float
    min_delta_closure_m: float
    min_delta_force_n: float
    method: StiffnessEstimatorMethod = "window_linear"
    window_size: int = 25
    min_samples: int = 8
    position_feedforward_gain: float = 0.25
    torque_feedforward_gain: float = 1.0
    position_limit_enabled: bool = False
    position_limit_force_rate_n_s: float = 10.0
    position_limit_stiffness_safety_factor: float = 1.0

    def __post_init__(self) -> None:
        """要求刚度估计范围与窗口参数有效。"""
        if self.min_n_per_m >= self.max_n_per_m:
            raise ValueError("min_n_per_m must be smaller than max_n_per_m")
        if not self.min_n_per_m <= self.initial_n_per_m <= self.max_n_per_m:
            raise ValueError("initial_n_per_m must lie within stiffness limits")
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("filter_alpha must lie within (0, 1]")
        if self.window_size <= 0 or self.min_samples <= 0:
            raise ValueError("window_size and min_samples must be positive")
        if self.min_samples > self.window_size:
            raise ValueError("min_samples must not exceed window_size")
        degree = {"window_linear": 1, "window_quadratic": 2}.get(self.method)
        if degree is not None and self.min_samples < degree + 1:
            raise ValueError("min_samples must provide enough samples for the selected method")
        if self.position_limit_enabled and self.position_feedforward_gain > 0:
            raise ValueError(
                "stiffness position feedforward and position limit are mutually exclusive"
            )


class ContactStiffnessEstimator:
    """用接触后的 ``dF/dc`` 样本在线估计等效接触刚度。"""

    def __init__(
        self,
        config: ContactStiffnessConfig,
        kinematics: CrankSliderKinematics,
    ) -> None:
        """保存估计参数和几何模型，并初始化估计状态。"""
        self._config = config
        self._kinematics = kinematics
        self._samples: deque[tuple[float, float]] = deque(maxlen=config.window_size)
        self.reset()

    @property
    def estimate_n_per_m(self) -> float:
        """返回当前滤波后的等效接触刚度估计。"""
        return self._estimate_n_per_m

    def reset(
        self,
        *,
        position_rad: float | None = None,
        normal_force_n: float | None = None,
    ) -> None:
        """重置估计，并可选记录新的接触参考点。"""
        self._estimate_n_per_m = float(self._config.initial_n_per_m)
        self._samples.clear()
        if position_rad is None or normal_force_n is None:
            self._last_closure_m = None
            self._last_force_n = None
            return
        closure_m = self._kinematics.closure(float(position_rad))
        force_n = max(0.0, float(normal_force_n))
        self._last_closure_m = closure_m
        self._last_force_n = force_n
        if (
            self._config.method != "secant_ewma"
            and math.isfinite(closure_m)
            and math.isfinite(force_n)
        ):
            self._samples.append((closure_m, force_n))

    def update(self, *, position_rad: float, normal_force_n: float) -> float:
        """用新的接触样本更新刚度估计，并返回当前估计值。"""
        if self._config.method != "secant_ewma":
            return self._update_window(position_rad=position_rad, normal_force_n=normal_force_n)

        return self._update_secant(position_rad=position_rad, normal_force_n=normal_force_n)

    def _update_secant(self, *, position_rad: float, normal_force_n: float) -> float:
        """用相邻有效样本的割线更新刚度，保留历史算法行为。"""
        closure_m = self._kinematics.closure(float(position_rad))
        force_n = max(0.0, float(normal_force_n))
        if self._last_closure_m is None or self._last_force_n is None:
            self._last_closure_m = closure_m
            self._last_force_n = force_n
            return self._estimate_n_per_m

        delta_closure = closure_m - self._last_closure_m
        delta_force = force_n - self._last_force_n
        if (
            abs(delta_closure) < self._config.min_delta_closure_m
            or abs(delta_force) < self._config.min_delta_force_n
        ):
            return self._estimate_n_per_m

        if delta_closure * delta_force > 0:
            sample = abs(delta_force / delta_closure)
            sample = float(np.clip(sample, self._config.min_n_per_m, self._config.max_n_per_m))
            alpha = float(self._config.filter_alpha)
            self._estimate_n_per_m += alpha * (sample - self._estimate_n_per_m)

        self._last_closure_m = closure_m
        self._last_force_n = force_n
        return self._estimate_n_per_m

    def _update_window(self, *, position_rad: float, normal_force_n: float) -> float:
        """用最近窗口的局部一次或二次拟合更新刚度。"""
        closure_m = self._kinematics.closure(float(position_rad))
        raw_force_n = float(normal_force_n)
        if not math.isfinite(closure_m) or not math.isfinite(raw_force_n):
            return self._estimate_n_per_m
        force_n = max(0.0, raw_force_n)

        self._samples.append((closure_m, force_n))
        if len(self._samples) < self._config.min_samples:
            return self._estimate_n_per_m

        closures = np.asarray([sample[0] for sample in self._samples], dtype=float)
        forces = np.asarray([sample[1] for sample in self._samples], dtype=float)
        closure_span = float(np.ptp(closures))
        force_span = float(np.ptp(forces))
        if (
            not math.isfinite(closure_span)
            or not math.isfinite(force_span)
            or closure_span < self._config.min_delta_closure_m
            or force_span < self._config.min_delta_force_n
        ):
            return self._estimate_n_per_m

        degree = 1 if self._config.method == "window_linear" else 2
        scale = max(closure_span, float(self._config.min_delta_closure_m))
        coordinate = (closures - closures[-1]) / scale
        design = np.column_stack([coordinate**power for power in range(degree + 1)])
        try:
            coefficients, _, rank, _ = np.linalg.lstsq(design, forces, rcond=None)
        except (np.linalg.LinAlgError, ValueError):
            return self._estimate_n_per_m
        if rank < degree + 1:
            return self._estimate_n_per_m

        slope = float(coefficients[1] / scale)
        if not math.isfinite(slope) or slope <= 0.0:
            return self._estimate_n_per_m

        sample = float(np.clip(slope, self._config.min_n_per_m, self._config.max_n_per_m))
        alpha = float(self._config.filter_alpha)
        self._estimate_n_per_m += alpha * (sample - self._estimate_n_per_m)
        return self._estimate_n_per_m

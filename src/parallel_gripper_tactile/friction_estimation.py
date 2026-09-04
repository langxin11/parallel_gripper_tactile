"""用切向探测载荷与双侧触觉合力估计保守摩擦系数。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Literal


FrictionEstimationState = Literal[
    "no_probe",
    "insufficient_contact",
    "collecting",
    "suspected_slip",
    "detected_fallback",
    "finalized_fallback",
    "estimated",
]


@dataclass(frozen=True, slots=True)
class FrictionEstimatorConfig:
    """微滑移检测与保守摩擦系数估计参数。"""

    window_size: int = 20
    min_samples: int = 5
    estimate_quantile: float = 0.9
    safety_discount: float = 0.8
    fallback_friction_coefficient: float = 0.2
    min_friction_coefficient: float = 0.05
    max_friction_coefficient: float = 2.0
    min_probe_load_n: float = 0.2
    min_total_normal_force_n: float = 0.2
    support_residual_threshold_n: float = 0.1
    support_utilization_threshold: float = 0.8
    mismatch_confirm_s: float = 0.006
    ratio_trend_enabled: bool = False
    trend_window_size: int = 20
    arming_slope_threshold_per_s: float = 0.09
    saturation_slope_threshold_per_s: float = 0.055
    max_side_ratio_difference: float = 0.1

    def __post_init__(self) -> None:
        """拒绝非有限值以及不一致的窗口和阈值。"""
        if (
            not isinstance(self.window_size, int)
            or isinstance(self.window_size, bool)
            or self.window_size <= 0
        ):
            raise ValueError("window_size must be a positive integer")
        if (
            not isinstance(self.min_samples, int)
            or isinstance(self.min_samples, bool)
            or not 1 <= self.min_samples <= self.window_size
        ):
            raise ValueError("min_samples must be a positive integer not exceeding window_size")
        if not isinstance(self.ratio_trend_enabled, bool):
            raise ValueError("ratio_trend_enabled must be a boolean")
        if (
            not isinstance(self.trend_window_size, int)
            or isinstance(self.trend_window_size, bool)
            or self.trend_window_size < 2
        ):
            raise ValueError("trend_window_size must be an integer of at least two")
        values = (
            self.estimate_quantile,
            self.safety_discount,
            self.fallback_friction_coefficient,
            self.min_friction_coefficient,
            self.max_friction_coefficient,
            self.min_probe_load_n,
            self.min_total_normal_force_n,
            self.support_residual_threshold_n,
            self.support_utilization_threshold,
            self.mismatch_confirm_s,
            self.arming_slope_threshold_per_s,
            self.saturation_slope_threshold_per_s,
            self.max_side_ratio_difference,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("friction estimator parameters must be finite")
        if not 0.0 < self.estimate_quantile <= 1.0:
            raise ValueError("estimate_quantile must be in (0, 1]")
        if not 0.0 < self.safety_discount <= 1.0:
            raise ValueError("safety_discount must be in (0, 1]")
        if self.min_friction_coefficient <= 0:
            raise ValueError("min_friction_coefficient must be positive")
        if self.max_friction_coefficient < self.min_friction_coefficient:
            raise ValueError(
                "max_friction_coefficient must not be smaller than min_friction_coefficient"
            )
        if not (
            self.min_friction_coefficient
            <= self.fallback_friction_coefficient
            <= self.max_friction_coefficient
        ):
            raise ValueError("fallback_friction_coefficient must be within estimator limits")
        if self.min_probe_load_n <= 0:
            raise ValueError("min_probe_load_n must be positive")
        if self.min_total_normal_force_n <= 0:
            raise ValueError("min_total_normal_force_n must be positive")
        if self.support_residual_threshold_n < 0:
            raise ValueError("support_residual_threshold_n must be non-negative")
        if not 0.0 <= self.support_utilization_threshold < 1.0:
            raise ValueError("support_utilization_threshold must be in [0, 1)")
        if self.mismatch_confirm_s <= 0:
            raise ValueError("mismatch_confirm_s must be positive")
        if self.arming_slope_threshold_per_s <= 0:
            raise ValueError("arming_slope_threshold_per_s must be positive")
        if self.saturation_slope_threshold_per_s < 0:
            raise ValueError("saturation_slope_threshold_per_s must be non-negative")
        if self.saturation_slope_threshold_per_s >= self.arming_slope_threshold_per_s:
            raise ValueError("saturation slope threshold must be smaller than arming threshold")
        if self.max_side_ratio_difference < 0:
            raise ValueError("max_side_ratio_difference must be non-negative")


@dataclass(frozen=True, slots=True)
class FrictionEstimate:
    """一个微滑移估计周期的结果和可记录诊断量。"""

    state: FrictionEstimationState
    slip_detected: bool
    consecutive_mismatch_count: int
    mismatch_duration_s: float
    support_residual_n: float
    support_utilization: float
    sample_count: int
    friction_coefficient: float
    raw_friction_coefficient: float | None
    using_fallback: bool
    ratio_slope_per_s: float
    trend_armed: bool
    side_ratio_difference: float


@dataclass(frozen=True, slots=True)
class FrictionProbeObservation:
    """一次切向探测周期可用的命令与双侧触觉合力。"""

    tangential_demand_n: float
    probe_excitation_n: float
    left_normal_force_n: float
    left_shear_force_n: float
    right_normal_force_n: float
    right_shear_force_n: float
    dt: float


def _linear_quantile(samples: tuple[float, ...], quantile: float) -> float:
    """返回排序样本的线性插值分位数。"""
    ordered = sorted(samples)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower_index = math.floor(position)
    upper_index = math.ceil(position)
    fraction = position - lower_index
    return ordered[lower_index] + fraction * (ordered[upper_index] - ordered[lower_index])


class ConservativeFrictionEstimator:
    """根据触觉摩擦比饱和或剪切支撑失配检测微滑移。

    探测期间，估计器比较已知切向载荷与左右触觉剪切合力，也可检测触觉
    摩擦比先增长再饱和的趋势。任一判据持续指定时间后才确认微滑移。确认
    时使用候选起滑开始前固定窗口内的
    ``(F_t_left + F_t_right) / (F_n_left + F_n_right)`` 高分位数，并乘安全
    折减系数。未探测、接触不足或样本不足时返回配置的保守回退值。

    输入均可由实机探测命令和触觉传感器获得；本类不接收真实摩擦系数、
    物体位移或速度。
    """

    def __init__(self, config: FrictionEstimatorConfig | None = None) -> None:
        """创建估计器并清空探测历史。"""
        self._config = config or FrictionEstimatorConfig()
        self._samples: deque[float] = deque(maxlen=self._config.window_size)
        self._trend_samples: deque[tuple[float, float]] = deque(
            maxlen=self._config.trend_window_size
        )
        self.reset()

    @property
    def config(self) -> FrictionEstimatorConfig:
        """返回不可变的估计器配置。"""
        return self._config

    def reset(self) -> None:
        """清除微滑移锁存、连续失配计数和起滑前样本。"""
        self._samples.clear()
        self._trend_samples.clear()
        self._slip_detected = False
        self._consecutive_mismatch_count = 0
        self._mismatch_duration_s = 0.0
        self._friction_coefficient = self._config.fallback_friction_coefficient
        self._raw_friction_coefficient: float | None = None
        self._using_fallback = True
        self._last_state: FrictionEstimationState = "no_probe"
        self._last_support_residual_n = 0.0
        self._last_support_utilization = 0.0
        self._elapsed_time_s = 0.0
        self._last_probe_load_n: float | None = None
        self._ratio_slope_per_s = 0.0
        self._trend_armed = False
        self._side_ratio_difference = math.inf

    def _result(
        self,
        *,
        state: FrictionEstimationState,
        support_residual_n: float,
        support_utilization: float,
    ) -> FrictionEstimate:
        """用当前内部状态构造不可变诊断结果。"""
        self._last_state = state
        self._last_support_residual_n = support_residual_n
        self._last_support_utilization = support_utilization
        return FrictionEstimate(
            state=state,
            slip_detected=self._slip_detected,
            consecutive_mismatch_count=self._consecutive_mismatch_count,
            mismatch_duration_s=self._mismatch_duration_s,
            support_residual_n=support_residual_n,
            support_utilization=support_utilization,
            sample_count=len(self._samples),
            friction_coefficient=self._friction_coefficient,
            raw_friction_coefficient=self._raw_friction_coefficient,
            using_fallback=self._using_fallback,
            ratio_slope_per_s=self._ratio_slope_per_s,
            trend_armed=self._trend_armed,
            side_ratio_difference=self._side_ratio_difference,
        )

    def update(
        self,
        observation: FrictionProbeObservation,
    ) -> FrictionEstimate:
        """用一个探测周期的双侧触觉合力更新摩擦估计。

        所有力输入均表示非负合力大小，而不是带符号的坐标分量。

        Args:
            observation: 当前已知探测载荷、双侧触觉法向/剪切合力和时间步。

        Returns:
            当前估计值、回退状态和微滑移诊断量。

        Raises:
            ValueError: 任一输入非有限、力为负数或时间步非正时抛出。
        """
        tangential_demand_n = observation.tangential_demand_n
        probe_excitation_n = observation.probe_excitation_n
        left_normal_force_n = observation.left_normal_force_n
        left_shear_force_n = observation.left_shear_force_n
        right_normal_force_n = observation.right_normal_force_n
        right_shear_force_n = observation.right_shear_force_n
        dt = observation.dt
        values = (
            tangential_demand_n,
            probe_excitation_n,
            left_normal_force_n,
            left_shear_force_n,
            right_normal_force_n,
            right_shear_force_n,
            dt,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("friction estimator inputs must be finite")
        if any(value < 0 for value in values[:-1]):
            raise ValueError("force inputs must be non-negative")
        if dt <= 0:
            raise ValueError("dt must be positive")

        total_normal_force_n = left_normal_force_n + right_normal_force_n
        total_shear_force_n = left_shear_force_n + right_shear_force_n
        support_residual_n = tangential_demand_n - total_shear_force_n
        support_utilization = (
            total_shear_force_n / tangential_demand_n if tangential_demand_n > 0 else 0.0
        )

        if self._slip_detected:
            state: FrictionEstimationState = (
                "detected_fallback" if self._using_fallback else "estimated"
            )
            return self._result(
                state=state,
                support_residual_n=support_residual_n,
                support_utilization=support_utilization,
            )

        if probe_excitation_n < self._config.min_probe_load_n:
            self._clear_probe_sequence()
            return self._result(
                state="no_probe",
                support_residual_n=support_residual_n,
                support_utilization=support_utilization,
            )
        if total_normal_force_n < self._config.min_total_normal_force_n:
            self._clear_probe_sequence()
            return self._result(
                state="insufficient_contact",
                support_residual_n=support_residual_n,
                support_utilization=support_utilization,
            )

        ratio = total_shear_force_n / total_normal_force_n
        left_ratio = left_shear_force_n / max(left_normal_force_n, 1e-12)
        right_ratio = right_shear_force_n / max(right_normal_force_n, 1e-12)
        self._side_ratio_difference = abs(left_ratio - right_ratio)
        probe_increasing = (
            self._last_probe_load_n is not None
            and probe_excitation_n > self._last_probe_load_n + 1e-12
        )
        self._elapsed_time_s += dt
        self._last_probe_load_n = probe_excitation_n
        self._trend_samples.append((self._elapsed_time_s, ratio))
        self._ratio_slope_per_s = self._trend_slope()
        trend_ready = len(self._trend_samples) == self._config.trend_window_size
        if (
            self._config.ratio_trend_enabled
            and trend_ready
            and probe_increasing
            and self._ratio_slope_per_s >= self._config.arming_slope_threshold_per_s
        ):
            self._trend_armed = True

        residual_mismatch = (
            support_residual_n >= self._config.support_residual_threshold_n
            and support_utilization <= self._config.support_utilization_threshold
        )
        trend_mismatch = (
            self._config.ratio_trend_enabled
            and self._trend_armed
            and trend_ready
            and probe_increasing
            and self._ratio_slope_per_s <= self._config.saturation_slope_threshold_per_s
            and self._side_ratio_difference <= self._config.max_side_ratio_difference
        )
        mismatching = residual_mismatch or trend_mismatch
        if not mismatching:
            self._consecutive_mismatch_count = 0
            self._mismatch_duration_s = 0.0
            self._samples.append(ratio)
            return self._result(
                state="collecting",
                support_residual_n=support_residual_n,
                support_utilization=support_utilization,
            )

        self._consecutive_mismatch_count += 1
        self._mismatch_duration_s += dt
        if self._mismatch_duration_s + 1e-12 < self._config.mismatch_confirm_s:
            return self._result(
                state="suspected_slip",
                support_residual_n=support_residual_n,
                support_utilization=support_utilization,
            )

        self._slip_detected = True
        if len(self._samples) >= self._config.min_samples:
            raw_estimate = _linear_quantile(tuple(self._samples), self._config.estimate_quantile)
            self._raw_friction_coefficient = raw_estimate
            discounted_estimate = raw_estimate * self._config.safety_discount
            self._friction_coefficient = min(
                self._config.max_friction_coefficient,
                max(self._config.min_friction_coefficient, discounted_estimate),
            )
            self._using_fallback = False
            state = "estimated"
        else:
            state = "detected_fallback"
        return self._result(
            state=state,
            support_residual_n=support_residual_n,
            support_utilization=support_utilization,
        )

    def finalize(self) -> FrictionEstimate:
        """结束当前探测并返回冻结估计或显式回退结果。

        未检测到微滑移时，静摩擦区样本只表示当前利用率，不能作为极限摩擦
        系数，因此 ``finalize`` 保留回退值而不从窗口强行生成估计。

        Returns:
            当前已锁存的保守估计；尚未检测微滑移时返回回退状态。
        """
        state: FrictionEstimationState
        if self._slip_detected:
            state = "detected_fallback" if self._using_fallback else "estimated"
        else:
            state = "finalized_fallback"
        return self._result(
            state=state,
            support_residual_n=self._last_support_residual_n,
            support_utilization=self._last_support_utilization,
        )

    def _clear_probe_sequence(self) -> None:
        """清除不连续探测留下的窗口与候选失配状态。"""
        self._samples.clear()
        self._trend_samples.clear()
        self._consecutive_mismatch_count = 0
        self._mismatch_duration_s = 0.0
        self._elapsed_time_s = 0.0
        self._last_probe_load_n = None
        self._ratio_slope_per_s = 0.0
        self._trend_armed = False
        self._side_ratio_difference = math.inf

    def _trend_slope(self) -> float:
        """返回当前触觉摩擦比窗口相对时间的一阶最小二乘斜率。"""
        if len(self._trend_samples) < 2:
            return 0.0
        mean_time = sum(sample[0] for sample in self._trend_samples) / len(self._trend_samples)
        mean_ratio = sum(sample[1] for sample in self._trend_samples) / len(self._trend_samples)
        denominator = sum((sample[0] - mean_time) ** 2 for sample in self._trend_samples)
        if denominator <= 1e-18:
            return 0.0
        numerator = sum(
            (sample[0] - mean_time) * (sample[1] - mean_ratio) for sample in self._trend_samples
        )
        return numerator / denominator


__all__ = [
    "ConservativeFrictionEstimator",
    "FrictionEstimate",
    "FrictionEstimatorConfig",
    "FrictionEstimationState",
    "FrictionProbeObservation",
]

"""基于粘着不等式和微滑移事件的单侧摩擦粒子后验。"""

from __future__ import annotations

from dataclasses import dataclass, fields
import math

import numpy as np

from ..tactile.risk import TAXELS_PER_SIDE


@dataclass(frozen=True, slots=True)
class ParticleFrictionConfig:
    """一维摩擦粒子滤波候选参数；默认不由任何生产配置启用。"""

    particle_count: int = 256
    min_friction: float = 0.05
    max_friction: float = 2.0
    prior_mean: float = 0.6
    prior_std: float = 0.25
    process_std_per_sqrt_s: float = 0.01
    incipient_utilization: float = 0.85
    transition_width: float = 0.08
    stable_evidence_time_s: float = 0.2
    event_strength: float = 4.0
    minimum_event_utilization: float = 0.15
    control_quantile: float = 0.1
    resample_ratio: float = 0.5
    # 直接支持局部重分配证据的最少受影响触点数；不是统计置信度。
    minimum_event_taxels: int = 4
    seed: int = 0

    def __post_init__(self) -> None:
        """拒绝无效范围、概率和随机种子。"""
        if not isinstance(self.particle_count, int) or isinstance(self.particle_count, bool):
            raise ValueError("particle_count 必须为整数")
        if self.particle_count < 32:
            raise ValueError("particle_count 至少为 32")
        if not isinstance(self.seed, int) or isinstance(self.seed, bool) or self.seed < 0:
            raise ValueError("seed 必须为非负整数")
        if not isinstance(self.minimum_event_taxels, int) or isinstance(
            self.minimum_event_taxels, bool
        ):
            raise ValueError("minimum_event_taxels 必须为整数")
        if not 1 <= self.minimum_event_taxels <= TAXELS_PER_SIDE:
            raise ValueError("事件触点门槛必须位于 1 与单侧触点数之间")
        for item in fields(self):
            if item.name in {"particle_count", "seed", "minimum_event_taxels"}:
                continue
            value = getattr(self, item.name)
            if isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"{item.name} 必须为有限数")
        if not 0 < self.min_friction < self.max_friction:
            raise ValueError("摩擦范围无效")
        if not self.min_friction <= self.prior_mean <= self.max_friction:
            raise ValueError("先验均值必须位于摩擦范围内")
        if (
            min(
                self.prior_std,
                self.process_std_per_sqrt_s,
                self.transition_width,
                self.stable_evidence_time_s,
                self.event_strength,
            )
            <= 0
        ):
            raise ValueError("粒子扩散和似然尺度必须为正")
        if not 0 < self.incipient_utilization < 1:
            raise ValueError("微滑移利用率必须位于 (0, 1)")
        if not 0 < self.control_quantile < 0.5:
            raise ValueError("控制分位数必须位于 (0, 0.5)")
        if not 0 < self.resample_ratio <= 1:
            raise ValueError("重采样比例必须位于 (0, 1]")
        if not self.min_friction <= self.minimum_event_utilization <= self.max_friction:
            raise ValueError("事件利用率阈值必须位于摩擦范围内")


@dataclass(frozen=True, slots=True)
class ParticleFrictionSnapshot:
    """摩擦后验摘要；控制值使用保守低分位而不是均值。"""

    mean: float
    control_value: float
    lower: float
    upper: float
    effective_sample_size: float
    updated: bool
    reason: str


class ParticleFrictionEstimator:
    """用摩擦利用率与微滑移事件递推一维摩擦后验。"""

    def __init__(self, config: ParticleFrictionConfig) -> None:
        """创建确定种子的粒子集合。"""
        self.config = config
        self._rng = np.random.default_rng(config.seed)
        self.reset()

    def reset(self) -> ParticleFrictionSnapshot:
        """从截断高斯先验重建本次连续接触段的粒子。"""
        c = self.config
        self._particles = np.clip(
            self._rng.normal(c.prior_mean, c.prior_std, c.particle_count),
            c.min_friction,
            c.max_friction,
        )
        self._weights = np.full(c.particle_count, 1.0 / c.particle_count)
        return self.snapshot(updated=False, reason="reset")

    @staticmethod
    def _sigmoid(value: np.ndarray) -> np.ndarray:
        """稳定计算 logistic 函数。"""
        clipped = np.clip(value, -60.0, 60.0)
        return 1.0 / (1.0 + np.exp(-clipped))

    def _quantile(self, probability: float) -> float:
        """返回带权粒子分位数。"""
        order = np.argsort(self._particles)
        values = self._particles[order]
        cumulative = np.cumsum(self._weights[order])
        index = min(int(np.searchsorted(cumulative, probability, side="left")), len(values) - 1)
        return float(values[index])

    def snapshot(self, *, updated: bool, reason: str) -> ParticleFrictionSnapshot:
        """返回当前后验的均值、区间和有效粒子数。"""
        effective = 1.0 / float(np.sum(self._weights**2))
        return ParticleFrictionSnapshot(
            mean=float(np.sum(self._particles * self._weights)),
            control_value=self._quantile(self.config.control_quantile),
            lower=self._quantile(0.05),
            upper=self._quantile(0.95),
            effective_sample_size=effective,
            updated=updated,
            reason=reason,
        )

    def _resample(self) -> None:
        """系统重采样并施加小幅粗化，防止事件后粒子贫化。"""
        count = self.config.particle_count
        cumulative = np.cumsum(self._weights)
        positions = (self._rng.random() + np.arange(count)) / count
        indices = np.searchsorted(cumulative, positions, side="left")
        self._particles = self._particles[indices]
        roughening = self.config.process_std_per_sqrt_s / math.sqrt(count)
        self._particles += self._rng.normal(0.0, roughening, count)
        self._particles = np.clip(
            self._particles,
            self.config.min_friction,
            self.config.max_friction,
        )
        self._weights.fill(1.0 / count)

    def update(
        self,
        utilization: float,
        *,
        dt_s: float,
        stable: bool,
        event: bool,
        event_taxels: int = 0,
    ) -> ParticleFrictionSnapshot:
        """粘着帧提供单边约束，可信微滑移事件提供局部锚点。"""
        if (
            not all(math.isfinite(value) for value in (utilization, dt_s))
            or utilization < 0
            or dt_s <= 0
            or not isinstance(event_taxels, int)
            or isinstance(event_taxels, bool)
            or not 0 <= event_taxels <= TAXELS_PER_SIDE
            or not isinstance(stable, bool)
            or not isinstance(event, bool)
        ):
            raise ValueError("粒子摩擦观测无效")
        c = self.config
        diffusion = c.process_std_per_sqrt_s * math.sqrt(dt_s)
        self._particles += self._rng.normal(0.0, diffusion, c.particle_count)
        self._particles = np.clip(self._particles, c.min_friction, c.max_friction)

        quality_qualified = event and event_taxels >= c.minimum_event_taxels
        qualified_event = quality_qualified and utilization >= c.minimum_event_utilization
        if quality_qualified and not qualified_event:
            return self.snapshot(updated=False, reason="event_below_utilization")
        if not stable and not qualified_event:
            return self.snapshot(updated=False, reason="insufficient_evidence")

        normalized = utilization / np.maximum(self._particles, c.min_friction)
        if qualified_event:
            # 已确认的微滑移起始不是“摩擦越小越可能”的普通二元事件，
            # 而是摩擦利用率抵达临界带的局部观测，因此用钟形似然定位 μ。
            standardized = (normalized - c.incipient_utilization) / c.transition_width
            likelihood = np.exp(-0.5 * standardized**2)
            power = c.event_strength * event_taxels / TAXELS_PER_SIDE
            reason = "event_update"
        else:
            event_probability = self._sigmoid(
                (normalized - c.incipient_utilization) / c.transition_width
            )
            likelihood = 1.0 - event_probability
            power = min(1.0, dt_s / c.stable_evidence_time_s)
            reason = "stick_update"
        log_weights = np.log(np.maximum(self._weights, 1e-300))
        log_weights += power * np.log(np.maximum(likelihood, 1e-12))
        log_weights -= float(np.max(log_weights))
        weights = np.exp(log_weights)
        total = float(np.sum(weights))
        if not math.isfinite(total) or total <= 0:
            return self.reset()
        self._weights = weights / total
        effective = 1.0 / float(np.sum(self._weights**2))
        if effective < c.resample_ratio * c.particle_count:
            self._resample()
            reason += "_resampled"
        return self.snapshot(updated=True, reason=reason)


__all__ = [
    "ParticleFrictionConfig",
    "ParticleFrictionEstimator",
    "ParticleFrictionSnapshot",
]

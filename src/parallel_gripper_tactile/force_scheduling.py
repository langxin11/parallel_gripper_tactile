"""根据切向载荷与摩擦系数生成保守的法向抓取目标力。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal


TargetForceLimit = Literal["minimum", "maximum", "rate"]


@dataclass(frozen=True, slots=True)
class TargetForceSchedulerConfig:
    """Oracle 目标力调度器的安全限值。"""

    safety_factor: float = 1.5
    min_force_n: float = 0.5
    max_force_n: float = 8.0
    max_force_rate_n_s: float = 4.0
    friction_floor: float = 0.05

    def __post_init__(self) -> None:
        """拒绝非有限值和不满足物理范围的配置。"""
        values = (
            self.safety_factor,
            self.min_force_n,
            self.max_force_n,
            self.max_force_rate_n_s,
            self.friction_floor,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target force scheduler parameters must be finite")
        if self.safety_factor < 1.0:
            raise ValueError("safety_factor must be at least 1")
        if self.min_force_n < 0:
            raise ValueError("min_force_n must be non-negative")
        if self.max_force_n < self.min_force_n:
            raise ValueError("max_force_n must not be smaller than min_force_n")
        if self.max_force_rate_n_s <= 0:
            raise ValueError("max_force_rate_n_s must be positive")
        if self.friction_floor <= 0:
            raise ValueError("friction_floor must be positive")


@dataclass(frozen=True, slots=True)
class TargetForceCommand:
    """一个目标力调度周期的输出及限幅诊断量。"""

    target_force_n: float
    target_force_rate_n_s: float
    raw_target_force_n: float
    limited_by: tuple[TargetForceLimit, ...]


class OracleTargetForceScheduler:
    """用已知摩擦系数生成平均单侧法向抓取目标力。

    对称二指抓取的理想摩擦容量为 ``2 * mu * f_n``，其中 ``f_n``
    是平均单侧法向力。因此未经上限和变化率限制的目标为
    ``safety_factor * tangential_demand / (2 * mu)``。摩擦系数下限只
    防止分母过小；本类不会估计摩擦系数。
    """

    def __init__(self, config: TargetForceSchedulerConfig | None = None) -> None:
        """使用给定配置创建调度器，并从最小目标力开始。"""
        self._config = config or TargetForceSchedulerConfig()
        self._target_force_n = self._config.min_force_n

    @property
    def config(self) -> TargetForceSchedulerConfig:
        """返回不可变的调度器配置。"""
        return self._config

    @property
    def target_force_n(self) -> float:
        """返回最近一次调度后的平均单侧目标力。"""
        return self._target_force_n

    def reset(self, initial_target_force_n: float | None = None) -> None:
        """重置变化率限制状态，并可指定新的初始目标力。

        Args:
            initial_target_force_n: 新的平均单侧目标力。省略时使用配置的
                最小目标力；显式值必须位于配置的力范围内。

        Raises:
            ValueError: 初始目标力非有限或超出配置范围时抛出。
        """
        target = (
            self._config.min_force_n if initial_target_force_n is None else initial_target_force_n
        )
        if not math.isfinite(target):
            raise ValueError("initial_target_force_n must be finite")
        if not self._config.min_force_n <= target <= self._config.max_force_n:
            raise ValueError("initial_target_force_n must be within configured force limits")
        self._target_force_n = float(target)

    def update(
        self,
        tangential_demand_n: float,
        friction_coefficient: float,
        dt: float,
    ) -> TargetForceCommand:
        """根据当前切向载荷和摩擦系数推进一个调度周期。

        Args:
            tangential_demand_n: 需要由双侧摩擦共同抵抗的切向合力大小。
            friction_coefficient: Oracle 提供的非负滑动摩擦系数。
            dt: 距离上一次调度的时间间隔，单位为秒。

        Returns:
            经过力上限和对称变化率限制后的目标力命令。

        Raises:
            ValueError: 任一输入非有限，或载荷、摩擦系数、时间步不在有效范围时抛出。
        """
        values = (tangential_demand_n, friction_coefficient, dt)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target force scheduler inputs must be finite")
        if tangential_demand_n < 0:
            raise ValueError("tangential_demand_n must be non-negative")
        if friction_coefficient < 0:
            raise ValueError("friction_coefficient must be non-negative")
        if dt <= 0:
            raise ValueError("dt must be positive")

        effective_friction = max(friction_coefficient, self._config.friction_floor)
        friction_target = (
            self._config.safety_factor * tangential_demand_n / (2.0 * effective_friction)
        )
        raw_target = max(self._config.min_force_n, friction_target)
        limited_by: list[TargetForceLimit] = []
        if friction_target < self._config.min_force_n:
            limited_by.append("minimum")

        bounded_target = min(raw_target, self._config.max_force_n)
        if raw_target > self._config.max_force_n:
            limited_by.append("maximum")

        maximum_delta = self._config.max_force_rate_n_s * dt
        target = min(
            self._target_force_n + maximum_delta,
            max(self._target_force_n - maximum_delta, bounded_target),
        )
        if not math.isclose(target, bounded_target, rel_tol=0.0, abs_tol=1e-15):
            limited_by.append("rate")

        previous_target = self._target_force_n
        self._target_force_n = float(target)
        return TargetForceCommand(
            target_force_n=self._target_force_n,
            target_force_rate_n_s=(self._target_force_n - previous_target) / dt,
            raw_target_force_n=float(raw_target),
            limited_by=tuple(limited_by),
        )


__all__ = [
    "OracleTargetForceScheduler",
    "TargetForceCommand",
    "TargetForceLimit",
    "TargetForceSchedulerConfig",
]

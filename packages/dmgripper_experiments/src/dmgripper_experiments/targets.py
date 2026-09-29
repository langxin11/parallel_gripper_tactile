"""固定预载与人工冻结分侧摩擦先验的目标力来源。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from dm_grasp_core.grasp.adaptive import AdaptiveLoadConfig, AdaptiveLoadScheduler

from .config import AdaptiveReferenceConfig, ExperimentConfig
from .observation import PairedObservation


@dataclass(frozen=True, slots=True)
class ForceTarget:
    """一个控制周期的受限目标力与诊断。"""

    force_n: float
    rate_n_s: float | None
    acceleration_n_s2: float | None
    source: Literal["fixed", "adaptive"]
    raw_force_n: float | None = None
    measured_tangential_force_n: float | None = None


class TargetSource:
    """阶段目标力来源。"""

    def __init__(self, config: AdaptiveReferenceConfig) -> None:
        """初始化目标来源。"""
        self._config = config
        self._activated = False

    @property
    def kind(self) -> str:
        """返回目标来源类型。"""
        return "fixed"

    @property
    def duration_s(self) -> float | None:
        """返回任务时长。"""
        return self._config.duration_s

    def set_execution_limited(self, limited: bool) -> None:
        """登记执行受限状态。"""
        pass

    def trace_fields(self) -> dict[str, object]:
        """返回调度诊断。"""
        return {}

    def preload_target(self, task_time_s: float) -> float:
        """返回固定预载目标。"""
        return self._config.initial_force_n

    def stabilize_preload(self) -> None:
        """清除激活状态。"""
        self._activated = False

    def observe(self, paired: PairedObservation, policy_dt_s: float) -> None:
        """消费触觉观测。"""
        pass

    def activate(self) -> None:
        """启用阶段目标。"""
        self._activated = True

    def active_reference(self, task_time_s: float) -> ForceTarget:
        """返回当前受限目标。"""
        return ForceTarget(self._config.initial_force_n, 0.0, 0.0, "fixed")


class AdaptiveTargetSource(TargetSource):
    """仅以冻结 μ 和分侧切向合力调用共享调度器。"""

    def __init__(self, config: AdaptiveReferenceConfig, load: AdaptiveLoadConfig) -> None:
        """以冻结参数创建共享调度器。"""
        super().__init__(config)
        self.scheduler = AdaptiveLoadScheduler(load)
        self._latest = None
        self._execution_limited = False

    @property
    def kind(self) -> str:
        """返回目标来源类型。"""
        return "adaptive"

    def set_execution_limited(self, limited: bool) -> None:
        """登记执行受限状态。"""
        self._execution_limited = limited

    def observe(self, paired: PairedObservation, policy_dt_s: float) -> None:
        """消费触觉观测。"""
        if not self._activated:
            return
        left = paired.snapshot.left_taxel_forces_n
        right = paired.snapshot.right_taxel_forces_n
        left_tx = sum(t[0] for t in left)
        left_ty = sum(t[1] for t in left)
        right_tx = sum(t[0] for t in right)
        right_ty = sum(t[1] for t in right)
        self._latest = self.scheduler.update(
            left_tangential_n=math.hypot(left_tx, left_ty),
            right_tangential_n=math.hypot(right_tx, right_ty),
            dt=policy_dt_s,
            pause_increase=self._execution_limited,
        )

    def trace_fields(self) -> dict[str, object]:
        """返回调度诊断。"""
        latest = self._latest
        if latest is None:
            return {}
        return {
            "adaptive_left_mu": self.scheduler.config.left_friction,
            "adaptive_right_mu": self.scheduler.config.right_friction,
            "adaptive_load_target_n": latest.load_force_n,
            "adaptive_schedule_gap_n": latest.schedule_gap_n,
            "adaptive_capacity_limited": latest.capacity_limited,
            "adaptive_execution_limited": self._execution_limited,
        }

    def active_reference(self, task_time_s: float) -> ForceTarget:
        """返回当前受限目标。"""
        latest = self._latest
        if latest is None:
            return ForceTarget(self._config.initial_force_n, 0.0, 0.0, "adaptive")
        return ForceTarget(
            latest.target_force_n,
            latest.target_force_rate_n_s,
            None,
            "adaptive",
            raw_force_n=latest.raw_target_force_n,
            measured_tangential_force_n=latest.measured_tangential_force_n,
        )


def build_target_source(config: ExperimentConfig) -> TargetSource:
    """按显式阶段建立目标来源。"""
    if config.stage == "adaptive":
        return AdaptiveTargetSource(config.reference, config.adaptive_load_config)
    return TargetSource(config.reference)

"""逐触点观测、分侧摩擦状态及受限目标调度的共享组合。"""

from dataclasses import dataclass, field, fields, replace
import math
from numbers import Real

import numpy as np

from ..tactile.risk import TaxelRiskConfig, TaxelRiskObserver
from .adaptive import AdaptiveLoadCommand, AdaptiveLoadConfig, AdaptiveLoadScheduler


@dataclass(frozen=True, slots=True)
class UnifiedAdaptiveConfig:
    """统一策略权限和边界；检测阈值与质量分数须经实机独立验收。"""

    load: AdaptiveLoadConfig = field(default_factory=AdaptiveLoadConfig)
    observer: TaxelRiskConfig = field(default_factory=TaxelRiskConfig)
    risk_enabled: bool = False
    friction_update_enabled: bool = False
    risk_step_n: float = 0.15
    risk_rate_n_s: float = 1.0
    risk_budget_n: float = 0.6
    risk_max_events: int = 4
    risk_duration_s: float = 10.0
    friction_quality_min: float = 0.8
    friction_discount: float = 0.8
    friction_min: float = 0.05
    friction_max: float = 2.0
    friction_expiry_s: float = 5.0
    friction_raise_events: int = 3
    friction_consistency: float = 0.15
    tracking_error_n: float = 0.5
    failure_timeout_s: float = 0.5
    max_sample_gap_s: float = 0.1

    def __post_init__(self) -> None:
        """验证权限类型、数值范围和计数上限。"""
        if not isinstance(self.load, AdaptiveLoadConfig) or not isinstance(
            self.observer, TaxelRiskConfig
        ):
            raise ValueError("统一策略必须提供有效的承载和观测配置")
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name in {"load", "observer"}:
                continue
            if item.name in {"risk_enabled", "friction_update_enabled"}:
                if not isinstance(value, bool):
                    raise ValueError("策略权限必须为布尔值")
            elif (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{item.name} 必须为正有限数值")
        for value in (self.risk_max_events, self.friction_raise_events):
            if not isinstance(value, int):
                raise ValueError("事件计数必须为整数")
        if self.friction_update_enabled and not self.risk_enabled:
            raise ValueError("摩擦更新要求先开放局部风险权限")
        if not 0 < self.friction_quality_min <= 1 or not 0 < self.friction_discount <= 1:
            raise ValueError("质量阈值和折减必须位于 (0, 1]")
        if not self.friction_min < self.friction_max or self.friction_consistency >= 1:
            raise ValueError("摩擦范围或一致性容差无效")
        if not all(
            self.friction_min <= v <= self.friction_max
            for v in (self.load.left_friction, self.load.right_friction)
        ):
            raise ValueError("摩擦先验必须位于允许范围内")


@dataclass(slots=True)
class _FrictionState:
    """一侧摩擦历史；提高估计需要多个独立一致事件。"""

    value: float
    updated_s: float | None = None
    pending: float | None = None
    pending_s: float | None = None
    count: int = 0


@dataclass(frozen=True, slots=True)
class UnifiedAdaptiveCommand:
    """本次调度及可解释诊断，不包含硬件动作。"""

    load: AdaptiveLoadCommand
    risk: float
    event_id: int
    increase_count: int
    risk_budget_exhausted: bool
    left_friction: float
    right_friction: float
    left_candidate: float | None
    right_candidate: float | None
    left_quality: float
    right_quality: float
    left_update_reason: str
    right_update_reason: str
    observation_reason: str
    left_valid_mask: tuple[bool, ...]
    right_valid_mask: tuple[bool, ...]
    tracking_error_n: float
    execution_limited: bool
    failure_reason: str | None

    def trace_fields(self) -> dict[str, object]:
        """生成仿真、真机和离线回放共享的平面诊断字段。"""
        return {
            "adaptive_risk": self.risk,
            "adaptive_event_id": self.event_id,
            "adaptive_increase_count": self.increase_count,
            "adaptive_risk_budget_exhausted": self.risk_budget_exhausted,
            "adaptive_left_mu": self.left_friction,
            "adaptive_right_mu": self.right_friction,
            "adaptive_left_candidate": self.left_candidate,
            "adaptive_right_candidate": self.right_candidate,
            "adaptive_left_quality": self.left_quality,
            "adaptive_right_quality": self.right_quality,
            "adaptive_left_update_reason": self.left_update_reason,
            "adaptive_right_update_reason": self.right_update_reason,
            "adaptive_observation_reason": self.observation_reason,
            "adaptive_left_valid_mask": "".join("1" if v else "0" for v in self.left_valid_mask),
            "adaptive_right_valid_mask": "".join("1" if v else "0" for v in self.right_valid_mask),
            "adaptive_load_target_n": self.load.load_force_n,
            "adaptive_schedule_gap_n": self.load.schedule_gap_n,
            "adaptive_track_error_n": self.tracking_error_n,
            "adaptive_capacity_limited": self.load.capacity_limited,
            "adaptive_execution_limited": self.execution_limited,
            "adaptive_failure_reason": self.failure_reason,
        }


class UnifiedAdaptivePolicy:
    """每次仅消费一个新样本；状态重置由新抓取实例实现。"""

    def __init__(self, config: UnifiedAdaptiveConfig) -> None:
        """创建观察器、摩擦先验和调度状态。"""
        self.config = config
        self.observer = TaxelRiskObserver(config.observer)
        self.scheduler = AdaptiveLoadScheduler(config.load)
        self._friction = [
            _FrictionState(config.load.left_friction),
            _FrictionState(config.load.right_friction),
        ]
        self._time: float | None = None
        self._risk_started: float | None = None
        self._risk_goal = config.load.min_force_n
        self._risk_budget = 0.0
        self._events = 0
        self._last_event = 0
        self._failure_since: dict[str, float] = {}
        self._failure: str | None = None
        self.latest: UnifiedAdaptiveCommand | None = None

    def _update_friction(
        self,
        side: int,
        candidate: float | None,
        quality: float,
        event: bool,
        changed: bool,
        now: float,
    ) -> str:
        """接触变化或过期回退，可信低值即时使用，高值需重复证据。"""
        c, state = self.config, self._friction[side]
        prior = (c.load.left_friction, c.load.right_friction)[side]
        if state.pending_s is not None and now - state.pending_s > c.friction_expiry_s:
            state.pending, state.pending_s, state.count = None, None, 0
        if changed or (state.updated_s is not None and now - state.updated_s > c.friction_expiry_s):
            state.value, state.updated_s, state.pending, state.count = prior, None, None, 0
            return "contact_reset" if changed else "expired_prior"
        if not c.friction_update_enabled:
            return "diagnostic_only"
        if not event:
            return "unchanged"
        if candidate is None or not math.isfinite(candidate) or quality < c.friction_quality_min:
            state.value, state.updated_s = prior, None
            state.pending, state.count = None, 0
            return "insufficient_quality"
        value = candidate * c.friction_discount
        if not c.friction_min <= value <= c.friction_max:
            state.value, state.updated_s = prior, None
            state.pending, state.count = None, 0
            return "out_of_range"
        if value <= state.value:
            state.value, state.updated_s, state.pending, state.count = value, now, None, 0
            return "lower_accepted"
        consistent = (
            state.pending is not None
            and abs(value - state.pending) <= c.friction_consistency * state.pending
        )
        state.count = state.count + 1 if consistent else 1
        state.pending = (
            value if state.pending is None or not consistent else min(value, state.pending)
        )
        state.pending_s = now
        if state.count < c.friction_raise_events:
            return "raise_pending"
        state.value, state.updated_s = state.pending, now
        state.pending, state.count = None, 0
        return "raise_accepted"

    def update(
        self,
        left,
        right,
        *,
        time_s: float,
        measured_force_n: float,
        execution_limited: bool = False,
        enabled: bool = True,
    ) -> UnifiedAdaptiveCommand:
        """消费规范为每侧九点 Fx/Fy/Fn 的新观测；重复样本不积分或重发事件。"""
        if not math.isfinite(time_s) or not math.isfinite(measured_force_n):
            raise ValueError("时间与实际法向力必须有限")
        arrays = [np.asarray(v, dtype=float) for v in (left, right)]
        if any(a.shape != (9, 3) or not np.isfinite(a).all() for a in arrays):
            raise ValueError("统一策略要求双侧各九点有限三轴力")
        if self._time is not None:
            if time_s < self._time:
                raise ValueError("观测时间不得回退")
            if time_s == self._time:
                assert self.latest is not None
                return replace(
                    self.latest,
                    event_id=0,
                    left_candidate=None,
                    right_candidate=None,
                    left_quality=0.0,
                    right_quality=0.0,
                    observation_reason="duplicate_timestamp",
                    load=replace(self.latest.load, target_force_rate_n_s=0.0),
                )
        dt = 0.0 if self._time is None else time_s - self._time
        gap = dt > self.config.max_sample_gap_s
        if gap:
            self.observer.reset()
        observation = self.observer.update(*arrays, time_s=time_s)
        self._time = time_s
        c = self.config
        event = observation.event_id > self._last_event
        if event:
            self._last_event = observation.event_id
        reasons = [
            self._update_friction(
                side,
                candidate,
                quality,
                event and enabled and observation.valid,
                observation.contact_changed or gap or not observation.valid,
                time_s,
            )
            for side, candidate, quality in (
                (0, observation.left_candidate, observation.left_quality),
                (1, observation.right_candidate, observation.right_quality),
            )
        ]
        if event and enabled and c.risk_enabled and observation.valid:
            if self._risk_started is None:
                self._risk_started = time_s
            if (
                self._events < c.risk_max_events
                and self._risk_budget < c.risk_budget_n
                and time_s - self._risk_started <= c.risk_duration_s
            ):
                increment = min(c.risk_step_n, c.risk_budget_n - self._risk_budget)
                self._risk_goal = min(
                    c.load.max_force_n,
                    max(self._risk_goal, self.scheduler.target_force_n) + increment,
                )
                self._risk_budget += increment
                self._events += 1
        track_error = self.scheduler.target_force_n - measured_force_n
        blocked = execution_limited and track_error > c.tracking_error_n
        load = self.scheduler.update(
            left_tangential_n=float(np.linalg.norm(arrays[0][:, :2].sum(axis=0))),
            right_tangential_n=float(np.linalg.norm(arrays[1][:, :2].sum(axis=0))),
            dt=dt if 0 < dt <= c.max_sample_gap_s else min(0.004, c.max_sample_gap_s),
            left_friction=self._friction[0].value,
            right_friction=self._friction[1].value,
            goal_floor_n=self._risk_goal,
            risk_rate_n_s=c.risk_rate_n_s * observation.risk if c.risk_enabled else 0.0,
            pause_increase=not enabled
            or not observation.valid
            or blocked
            or gap
            or dt == 0
            or self._failure is not None,
        )
        exhausted = (
            self._events >= c.risk_max_events
            or self._risk_budget >= c.risk_budget_n
            or (self._risk_started is not None and time_s - self._risk_started > c.risk_duration_s)
        )
        conditions = {
            "capacity_limited": load.capacity_limited,
            "tracking_limited": blocked,
            "invalid_contact": not observation.valid,
            "sample_gap": gap,
            "risk_budget_exhausted": exhausted and observation.risk > 0,
        }
        for reason, active in conditions.items():
            if not enabled or not active:
                self._failure_since.pop(reason, None)
            else:
                started = self._failure_since.setdefault(reason, time_s)
                if time_s - started >= c.failure_timeout_s or reason == "sample_gap":
                    self._failure = self._failure or reason
        self.latest = UnifiedAdaptiveCommand(
            load,
            observation.risk,
            observation.event_id if event else 0,
            self._events,
            exhausted,
            self._friction[0].value,
            self._friction[1].value,
            observation.left_candidate,
            observation.right_candidate,
            observation.left_quality,
            observation.right_quality,
            *reasons,
            observation.reason,
            observation.left_valid_mask,
            observation.right_valid_mask,
            track_error,
            execution_limited,
            self._failure,
        )
        return self.latest

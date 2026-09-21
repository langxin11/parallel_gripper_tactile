"""逐触点观测、分侧摩擦状态及受限目标调度的共享组合。"""

from dataclasses import dataclass, field, fields, replace
import math
from numbers import Real

import numpy as np

from ..tactile.risk import TaxelRiskConfig, TaxelRiskObservation, TaxelRiskObserver
from ..tactile.multirate import FilteredTangentialLoad, TactileState
from .adaptive import AdaptiveLoadCommand, AdaptiveLoadConfig, AdaptiveLoadScheduler
from .friction_depth import DepthFrictionPrior, DepthFrictionPriorConfig
from .friction_particle import (
    ParticleFrictionConfig,
    ParticleFrictionEstimator,
    ParticleFrictionSnapshot,
)


@dataclass(frozen=True, slots=True)
class UnifiedAdaptiveConfig:
    """统一策略权限和边界；检测阈值与质量分数须经实机独立验收。"""

    load: AdaptiveLoadConfig = field(default_factory=AdaptiveLoadConfig)
    observer: TaxelRiskConfig = field(default_factory=TaxelRiskConfig)
    particle_friction: ParticleFrictionConfig | None = None
    depth_friction_prior: DepthFrictionPriorConfig | None = None
    risk_enabled: bool = False
    risk_step_enabled: bool = True
    friction_update_enabled: bool = False
    risk_step_n: float = 0.15
    risk_rate_n_s: float = 1.0
    risk_budget_n: float = 0.6
    risk_max_events: int = 4
    risk_duration_s: float = 10.0
    risk_repeat_interval_s: float | None = None
    friction_quality_min: float = 0.8
    friction_discount: float = 0.8
    friction_min: float = 0.05
    friction_max: float = 2.0
    friction_expiry_s: float | None = 5.0
    friction_raise_events: int = 3
    friction_consistency: float = 0.15
    friction_lower_bound_tolerance: float = 0.02
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
            if item.name in {"load", "observer", "particle_friction", "depth_friction_prior"}:
                continue
            if item.name in {"risk_repeat_interval_s", "friction_expiry_s"} and value is None:
                continue
            if item.name in {"risk_enabled", "risk_step_enabled", "friction_update_enabled"}:
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
        if self.depth_friction_prior is not None:
            depth = self.depth_friction_prior
            if not isinstance(depth, DepthFrictionPriorConfig):
                raise ValueError("深度摩擦先验配置类型无效")
            if not self.friction_update_enabled:
                raise ValueError("深度摩擦先验要求开放摩擦更新权限")
            if (
                not self.friction_min
                <= depth.min_friction
                < depth.max_friction
                <= self.friction_max
            ):
                raise ValueError("深度先验必须位于允许摩擦范围内")
            if (self.load.left_friction, self.load.right_friction) != (
                depth.min_friction,
                depth.min_friction,
            ):
                raise ValueError("深度先验下限必须与双侧启动回退值一致")
            if self.particle_friction is not None and not (
                self.particle_friction.min_friction <= depth.min_friction
                and depth.max_friction <= self.particle_friction.max_friction
            ):
                raise ValueError("深度先验必须位于粒子摩擦范围内")
        if self.particle_friction is not None:
            if not isinstance(self.particle_friction, ParticleFrictionConfig):
                raise ValueError("粒子摩擦配置类型无效")
            if not self.friction_update_enabled:
                raise ValueError("粒子摩擦估计要求开放摩擦更新权限")
            if (
                self.particle_friction.min_friction < self.friction_min
                or self.particle_friction.max_friction > self.friction_max
            ):
                raise ValueError("粒子摩擦范围必须位于统一策略允许范围内")
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
    """一侧摩擦历史；整侧下界可提高采用值，事件候选另需一致确认。"""

    value: float
    updated_s: float | None = None
    pending: float | None = None
    pending_s: float | None = None
    count: int = 0
    lower_bound: float = 0.0
    taxel_lower_bounds: tuple[float, ...] = (0.0,) * 9


@dataclass(frozen=True, slots=True)
class UnifiedAdaptiveCommand:
    """本次调度及可解释诊断，不包含硬件动作。

    整侧下界来自切向合力与法向合力比；局部下界是连续有效触点的
    历史力比最大值。仅在共同摩擦系数及非滑移假设下，局部最大值
    才能约束同一个系数；异质接触时保留为诊断，不否决整侧候选。
    """

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
    left_lower_bound: float
    right_lower_bound: float
    left_update_reason: str
    right_update_reason: str
    observation_reason: str
    left_valid_mask: tuple[bool, ...]
    right_valid_mask: tuple[bool, ...]
    tracking_error_n: float
    execution_limited: bool
    failure_reason: str | None
    left_taxel_lower_bound: float = 0.0
    right_taxel_lower_bound: float = 0.0
    left_particle_friction: ParticleFrictionSnapshot | None = None
    right_particle_friction: ParticleFrictionSnapshot | None = None

    depth_prior_diagnostics: tuple[tuple[float | None, float, float, bool, str], ...] = ()

    def trace_fields(self) -> dict[str, object]:
        """生成仿真、真机和离线回放共享的平面诊断字段。"""
        result: dict[str, object] = {
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
            "adaptive_left_mu_lower_bound": self.left_lower_bound,
            "adaptive_right_mu_lower_bound": self.right_lower_bound,
            "adaptive_left_taxel_mu_lower_bound": self.left_taxel_lower_bound,
            "adaptive_right_taxel_mu_lower_bound": self.right_taxel_lower_bound,
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
        for side, diagnostic in zip(("left", "right"), self.depth_prior_diagnostics):
            for name, value in zip(
                ("depth_m", "candidate", "value", "locked", "reason"), diagnostic, strict=True
            ):
                result[f"adaptive_{side}_depth_prior_{name}"] = value
        for side, snapshot in (
            ("left", self.left_particle_friction),
            ("right", self.right_particle_friction),
        ):
            if snapshot is None:
                continue
            prefix = f"adaptive_{side}_particle_mu"
            result.update(
                {
                    f"{prefix}_mean": snapshot.mean,
                    f"{prefix}_control": snapshot.control_value,
                    f"{prefix}_lower": snapshot.lower,
                    f"{prefix}_upper": snapshot.upper,
                    f"{prefix}_ess": snapshot.effective_sample_size,
                    f"{prefix}_updated": snapshot.updated,
                    f"{prefix}_reason": snapshot.reason,
                }
            )
        return result


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
        if config.particle_friction is None:
            self._particle_filters: (
                tuple[ParticleFrictionEstimator, ParticleFrictionEstimator] | None
            ) = None
            self._particle_snapshots: (
                tuple[ParticleFrictionSnapshot, ParticleFrictionSnapshot] | None
            ) = None
        else:
            particle = config.particle_friction
            self._particle_filters = (
                ParticleFrictionEstimator(
                    replace(
                        particle,
                        prior_mean=config.load.left_friction,
                        seed=particle.seed,
                    )
                ),
                ParticleFrictionEstimator(
                    replace(
                        particle,
                        prior_mean=config.load.right_friction,
                        seed=particle.seed + 1,
                    )
                ),
            )
            self._particle_snapshots = tuple(
                estimator.snapshot(updated=False, reason="initialized")
                for estimator in self._particle_filters
            )
        self._applied_friction = [config.load.left_friction, config.load.right_friction]
        self._depth_priors = (
            None
            if config.depth_friction_prior is None
            else tuple(DepthFrictionPrior(config.depth_friction_prior) for _ in range(2))
        )
        self._time: float | None = None
        self._sample_sequence = -1
        self._invalid_samples = 0
        self._risk_started: float | None = None
        self._risk_goal = config.load.min_force_n
        self._risk_budget = 0.0
        self._events = 0
        self._last_event = 0
        self._last_risk_step_s = -math.inf
        self._risk_confirmed_since: float | None = None
        self._failure_since: dict[str, float] = {}
        self._failure: str | None = None
        self.latest: UnifiedAdaptiveCommand | None = None
        self.contact_floor_n = config.load.min_force_n

    def establish_contact_floor(self, force_n: float) -> None:
        """预载结束时衔接已达到的参考目标，不重置摩擦、风险及承载滤波历史。"""
        if (
            not math.isfinite(force_n)
            or not self.config.load.min_force_n <= force_n <= self.config.load.max_force_n
        ):
            raise ValueError("接触底力必须位于调度范围内")
        self.contact_floor_n = force_n
        self.scheduler.target_force_n = force_n
        if self.latest is not None:
            self.latest = replace(
                self.latest,
                load=replace(self.latest.load, target_force_n=force_n, target_force_rate_n_s=0.0),
            )

    def _update_friction(
        self,
        side: int,
        candidate: float | None,
        quality: float,
        event: bool,
        changed: bool,
        utilization: float,
        taxel_utilization: tuple[float, ...],
        valid_mask: tuple[bool, ...],
        lower_bound_eligible: bool,
        now: float,
    ) -> str:
        """接触变化或过期回退，可信低值即时使用，高值需重复证据。"""
        c, state = self.config, self._friction[side]
        prior = (
            (c.load.left_friction, c.load.right_friction)[side]
            if self._depth_priors is None
            else self._depth_priors[side].value
        )
        if (
            c.friction_expiry_s is not None
            and state.pending_s is not None
            and now - state.pending_s > c.friction_expiry_s
        ):
            state.pending, state.pending_s, state.count = None, None, 0
        expired = (
            c.friction_expiry_s is not None
            and state.updated_s is not None
            and now - state.updated_s > c.friction_expiry_s
        )
        if changed or expired:
            state.value, state.updated_s, state.pending, state.count = prior, None, None, 0
            state.lower_bound = 0.0
            state.taxel_lower_bounds = (0.0,) * 9
            return "contact_reset" if changed else "expired_prior"
        # 非滑移是下界解释的前提；低风险仅为工程筛选，不能证明静摩擦。
        # 逐触点历史在触点失效时清空，稳定子集模式下也不得跨触点重入复用。
        state.taxel_lower_bounds = tuple(
            max(previous, ratio)
            if lower_bound_eligible and math.isfinite(ratio) and valid
            else previous
            if valid
            else 0.0
            for previous, ratio, valid in zip(
                state.taxel_lower_bounds, taxel_utilization, valid_mask, strict=True
            )
        )
        if lower_bound_eligible and math.isfinite(utilization):
            state.lower_bound = max(state.lower_bound, utilization)
        if not c.friction_update_enabled:
            return "diagnostic_only"
        lower_bound_value = min(state.lower_bound, c.friction_max)
        lower_bound_accepted = lower_bound_value > state.value
        if lower_bound_accepted:
            state.value, state.updated_s = lower_bound_value, now
            state.pending, state.pending_s, state.count = None, None, 0
        if not event:
            return "lower_bound_accepted" if lower_bound_accepted else "unchanged"
        if candidate is None or not math.isfinite(candidate) or quality < c.friction_quality_min:
            state.pending, state.count = None, 0
            return "insufficient_quality"
        value = candidate * c.friction_discount
        if not c.friction_min <= value <= c.friction_max:
            state.pending, state.count = None, 0
            return "out_of_range"
        if candidate + c.friction_lower_bound_tolerance < state.lower_bound:
            state.pending, state.count = None, 0
            return "below_observed_lower_bound"
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

    def _update_particle_friction(
        self,
        observation: TaxelRiskObservation,
        *,
        dt_s: float,
        event: bool,
        actionable: bool,
        stable: bool,
        reset: bool,
        now: float,
        reasons: list[str],
    ) -> None:
        """用后验低分位保守地下调摩擦；无滑移证据只收缩后验。"""
        if self._particle_filters is None:
            return
        if reset:
            if self._depth_priors is not None:
                for side, estimator in enumerate(self._particle_filters):
                    estimator.config = replace(
                        estimator.config, prior_mean=self._depth_priors[side].value
                    )
            self._particle_snapshots = tuple(
                estimator.reset() for estimator in self._particle_filters
            )
            return
        if dt_s <= 0:
            return
        snapshots: list[ParticleFrictionSnapshot] = []
        for side, estimator, utilization, candidate, quality in (
            (
                0,
                self._particle_filters[0],
                observation.left_utilization,
                observation.left_candidate,
                observation.left_quality,
            ),
            (
                1,
                self._particle_filters[1],
                observation.right_utilization,
                observation.right_candidate,
                observation.right_quality,
            ),
        ):
            physics_event = event and actionable and candidate is not None
            qualified_event = physics_event
            event_observation = candidate if physics_event else utilization
            event_quality = quality if physics_event else 0.0
            snapshot = estimator.update(
                event_observation if qualified_event else utilization,
                dt_s=dt_s,
                stable=stable,
                event=qualified_event,
                event_quality=event_quality if qualified_event else 0.0,
            )
            snapshots.append(snapshot)
            # 粘着帧只能说明摩擦下界，不能据此乐观地减小夹持力；只有经质量
            # 门控的微滑移事件才允许将控制摩擦向保守低分位下调。
            if qualified_event and snapshot.reason.startswith("event_update"):
                state = self._friction[side]
                conservative = max(
                    snapshot.control_value,
                    state.lower_bound - self.config.friction_lower_bound_tolerance,
                    self.config.friction_min,
                )
                if conservative < state.value:
                    state.value = conservative
                    state.updated_s = now
                    state.pending, state.pending_s, state.count = None, None, 0
                    reasons[side] = f"particle_{snapshot.reason}"
        self._particle_snapshots = (snapshots[0], snapshots[1])

    def update(
        self,
        left,
        right,
        *,
        time_s: float,
        measured_force_n: float,
        execution_limited: bool = False,
        enabled: bool = True,
        observation: TaxelRiskObservation | None = None,
        filtered_load: FilteredTangentialLoad | None = None,
        freeze_increase: bool = False,
        left_displacements_m=None,
        right_displacements_m=None,
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
        if observation is None:
            observation = self.observer.update(*arrays, time_s=time_s)
        self._time = time_s
        c = self.config
        track_error = self.scheduler.target_force_n - measured_force_n
        blocked = execution_limited and track_error > c.tracking_error_n
        actionable = (
            enabled
            and observation.valid
            and not freeze_increase
            and not blocked
            and not gap
            and dt > 0
            and self._failure is None
        )
        event = observation.event_id > self._last_event
        if event:
            self._last_event = observation.event_id
        lower_bound_eligible = (
            enabled
            and observation.valid
            and observation.reason == "observing"
            and observation.risk < c.observer.release_ratio
            and not freeze_increase
            and not blocked
            and not gap
        )
        previous_friction = tuple(self._applied_friction)
        depth_reset = observation.contact_changed or gap or not observation.valid
        depth_ready = [True, True]
        if self._depth_priors is not None:
            for side, prior in enumerate(self._depth_priors):
                state = self._friction[side]
                expired = (
                    c.friction_expiry_s is not None
                    and state.updated_s is not None
                    and time_s - state.updated_s > c.friction_expiry_s
                )
                if depth_reset or expired:
                    prior.reset()
                locked_now = prior.observe(
                    (left_displacements_m, right_displacements_m)[side],
                    arrays[side],
                    (observation.left_valid_mask, observation.right_valid_mask)[side],
                    eligible=lower_bound_eligible and not depth_reset and not expired,
                    time_s=time_s,
                )
                depth_ready[side] = prior.locked and prior.depth_m is not None
                # 先验只初始化无证据状态；已接受的低摩擦不能被压入深度抬高。
                if locked_now and state.updated_s is None and self._particle_filters is not None:
                    estimator = self._particle_filters[side]
                    estimator.config = replace(estimator.config, prior_mean=prior.value)
                    estimator.reset()
                if depth_ready[side] and state.updated_s is None and lower_bound_eligible:
                    state.value = prior.value
        reasons = [
            self._update_friction(
                side,
                candidate,
                quality,
                event and actionable and c.particle_friction is None,
                observation.contact_changed or gap or not observation.valid,
                utilization,
                taxel_utilization,
                valid_mask,
                lower_bound_eligible and depth_ready[side],
                time_s,
            )
            for side, candidate, quality, utilization, taxel_utilization, valid_mask in (
                (
                    0,
                    observation.left_candidate,
                    observation.left_quality,
                    observation.left_utilization,
                    observation.left_taxel_utilization,
                    observation.left_valid_mask,
                ),
                (
                    1,
                    observation.right_candidate,
                    observation.right_quality,
                    observation.right_utilization,
                    observation.right_taxel_utilization,
                    observation.right_valid_mask,
                ),
            )
        ]
        self._update_particle_friction(
            observation,
            dt_s=dt,
            event=event,
            actionable=actionable,
            stable=lower_bound_eligible and all(depth_ready),
            reset=(
                observation.contact_changed
                or gap
                or not observation.valid
                or "expired_prior" in reasons
            ),
            now=time_s,
            reasons=reasons,
        )
        self._applied_friction = [state.value for state in self._friction]
        if self._depth_priors is not None:
            for side, state in enumerate(self._friction):
                if state.value > previous_friction[side]:
                    # 接触可靠才允许提高采用值；所有上调统一限速，负向证据立即保留。
                    allowance = (
                        c.depth_friction_prior.max_increase_per_s * dt
                        if lower_bound_eligible and depth_ready[side] and not depth_reset
                        else 0.0
                    )
                    self._applied_friction[side] = min(
                        state.value, previous_friction[side] + allowance
                    )
        if not enabled or not observation.valid or blocked or gap or observation.risk < 1:
            self._risk_confirmed_since = None
        elif actionable and self._risk_confirmed_since is None:
            self._risk_confirmed_since = time_s
        repeat = (
            c.risk_repeat_interval_s is not None
            and observation.confirmed_risk
            and self._risk_confirmed_since is not None
            and time_s - self._risk_confirmed_since >= c.observer.confirmation_s
            and time_s - self._last_risk_step_s >= c.risk_repeat_interval_s
        )
        if (event or repeat) and actionable and c.risk_enabled and c.risk_step_enabled:
            if self._risk_started is None:
                self._risk_started = time_s
            if (
                self._events < c.risk_max_events
                and self._risk_budget < c.risk_budget_n
                and time_s - self._risk_started <= c.risk_duration_s
                and (
                    c.risk_repeat_interval_s is None
                    or time_s - self._last_risk_step_s >= c.risk_repeat_interval_s
                )
            ):
                base = max(self._risk_goal, self.scheduler.target_force_n)
                increment = min(
                    c.risk_step_n,
                    c.risk_budget_n - self._risk_budget,
                    c.load.max_force_n - base,
                )
                self._risk_goal = base + increment
                self._risk_budget += increment
                self._events += int(increment > 0)
                self._last_risk_step_s = time_s
        load = self.scheduler.update(
            left_tangential_n=float(np.linalg.norm(arrays[0][:, :2].sum(axis=0))),
            right_tangential_n=float(np.linalg.norm(arrays[1][:, :2].sum(axis=0))),
            dt=dt if 0 < dt <= c.max_sample_gap_s else min(0.004, c.max_sample_gap_s),
            left_friction=self._applied_friction[0],
            right_friction=self._applied_friction[1],
            goal_floor_n=max(
                self._risk_goal,
                self.contact_floor_n,
                self.scheduler.target_force_n
                if self._depth_priors is not None
                and not (lower_bound_eligible and all(depth_ready) and not depth_reset)
                else 0.0,
            ),
            risk_rate_n_s=(
                c.risk_rate_n_s
                if c.risk_enabled and self._risk_goal > self.scheduler.target_force_n
                else 0.0
            ),
            filtered_load=filtered_load,
            pause_increase=not enabled
            or freeze_increase
            or not observation.valid
            or blocked
            or gap
            or dt == 0
            or self._failure is not None,
        )
        exhausted = c.risk_step_enabled and (
            self._events >= c.risk_max_events
            or self._risk_budget >= c.risk_budget_n
            or (c.risk_enabled and self._risk_goal >= c.load.max_force_n)
            or (self._risk_started is not None and time_s - self._risk_started > c.risk_duration_s)
        )
        conditions = {
            "capacity_limited": load.capacity_limited,
            "tracking_limited": blocked,
            "invalid_contact": not observation.valid and observation.reason != "stale_sample",
            "stale_tactile": observation.reason == "stale_sample",
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
            self._applied_friction[0],
            self._applied_friction[1],
            observation.left_candidate,
            observation.right_candidate,
            observation.left_quality,
            observation.right_quality,
            self._friction[0].lower_bound,
            self._friction[1].lower_bound,
            *reasons,
            observation.reason,
            observation.left_valid_mask,
            observation.right_valid_mask,
            track_error,
            execution_limited,
            self._failure,
            max(self._friction[0].taxel_lower_bounds),
            max(self._friction[1].taxel_lower_bounds),
            None if self._particle_snapshots is None else self._particle_snapshots[0],
            None if self._particle_snapshots is None else self._particle_snapshots[1],
            ()
            if self._depth_priors is None
            else tuple(
                (prior.depth_m, prior.candidate, prior.value, prior.locked, prior.reason)
                for prior in self._depth_priors
            ),
        )
        return self.latest

    def update_tactile_state(
        self,
        state: TactileState,
        *,
        control_time_s: float,
        stale_after_s: float,
        measured_force_n: float,
        execution_limited: bool = False,
        enabled: bool = True,
        left_displacements_m=None,
        right_displacements_m=None,
    ) -> UnifiedAdaptiveCommand:
        """控制侧消费最新状态及短期锁存事件，按序号去重，不回补历史增力。

        采样与控制时间须来自同一时基。硬件适配需先处理设备与主机时基，
        不得直接把设备时间与主机单调时钟相减。重复或陈旧快照禁止增力，
        持续陈旧沿用既有失败超时；故障动作仍由后端负责。
        """
        if not math.isfinite(stale_after_s) or stale_after_s <= 0:
            raise ValueError("新鲜度阈值必须为正有限数")
        previous_sequence = self._sample_sequence
        if state.sequence_id < previous_sequence:
            raise ValueError("控制侧触觉序号不得回退")
        fresh = state.valid and state.is_fresh(control_time_s, stale_after_s)
        unseen_invalid = state.invalid_samples > self._invalid_samples
        self._invalid_samples = state.invalid_samples
        new = state.sequence_id > previous_sequence
        self._sample_sequence = state.sequence_id
        observation = state.observation
        # 锁存候选避免 500/250 Hz 抽取丢失单帧事件；样本和事件时间
        # 在硬件适配处一起映射到主机时基，计入接收后等待造成的事件年龄。
        event_fresh = (
            state.event_time_s is not None
            and 0 <= control_time_s - state.event_time_s <= stale_after_s
        )
        if new and fresh and not unseen_invalid and event_fresh and state.latest_event is not None:
            observation = replace(
                observation,
                event_id=state.latest_event.event_id,
                left_candidate=state.latest_event.left_candidate,
                right_candidate=state.latest_event.right_candidate,
                left_quality=state.latest_event.left_quality,
                right_quality=state.latest_event.right_quality,
            )
        if unseen_invalid:
            observation = replace(observation, valid=False, reason="invalid_sample")
        elif not fresh:
            observation = replace(
                observation,
                valid=False,
                reason="invalid_sample" if not state.valid else "stale_sample",
            )
        elif not new:
            observation = replace(
                observation,
                event_id=0,
                left_candidate=None,
                right_candidate=None,
                left_quality=0.0,
                right_quality=0.0,
                contact_changed=False,
                reason="repeated_snapshot",
            )
        # 每个控制 tick 推进时钟，即便冻结也不在恢复时补算历史增力。
        return self.update(
            state.left_taxels if state.valid else np.zeros((9, 3)),
            state.right_taxels if state.valid else np.zeros((9, 3)),
            time_s=control_time_s,
            measured_force_n=measured_force_n,
            execution_limited=execution_limited,
            enabled=enabled,
            observation=observation,
            filtered_load=state.load,
            freeze_increase=not fresh or not new or unseen_invalid,
            left_displacements_m=left_displacements_m,
            right_displacements_m=right_displacements_m,
        )

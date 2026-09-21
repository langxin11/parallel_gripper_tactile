"""可信刚度驱动的预载目标与导纳参数调度，不访问硬件。"""

from collections import deque
from dataclasses import dataclass, fields
import math

from ..control.admittance import SecondOrderAdmittance
from ..control.stiffness import StiffnessSnapshot


def _validate_positive(config) -> None:
    """拒绝非有限或非正的实验边界。"""
    for item in fields(config):
        value = getattr(config, item.name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{item.name} 必须为正有限数值")


@dataclass(frozen=True, slots=True)
class StiffnessPreloadConfig:
    """预载辨识边界；压入量相对局部等效零力点，力为平均单侧力。"""

    indentation_m: float = 0.002
    min_force_n: float = 1.0
    max_force_n: float = 4.0
    force_ceiling_n: float = 5.0
    probe_force_n: float = 1.0
    max_closure_m: float = 0.003
    timeout_s: float = 3.0
    consistency: float = 0.15
    stable_s: float = 0.1
    min_updates: int = 3
    max_estimate_age_s: float = 0.2

    def __post_init__(self) -> None:
        """验证探测、接触目标及可信更新门槛。"""
        _validate_positive(self)
        if not isinstance(self.min_updates, int) or self.min_updates < 2:
            raise ValueError("min_updates 必须为至少 2 的整数")
        if not self.min_force_n <= self.probe_force_n <= self.max_force_n:
            raise ValueError("预载探测力必须位于目标力上下限之间")
        if self.force_ceiling_n <= self.max_force_n:
            raise ValueError("预载原始过力上限必须高于目标力上限")
        if self.consistency >= 1 or self.stable_s >= min(self.timeout_s, self.max_estimate_age_s):
            raise ValueError("刚度一致性或确认时间无效")


@dataclass(frozen=True, slots=True)
class StiffnessAdmittanceConfig:
    """局部闭环频率调度的实验参数；不代表真机带宽验收。"""

    bandwidth_rad_s: float = 10.0
    damping_ratio: float = 1.0
    filter_tau_s: float = 0.2
    relative_rate_per_s: float = 1.0
    min_mass_kg: float = 0.2
    max_mass_kg: float = 300.0
    max_estimate_age_s: float = 0.2

    def __post_init__(self) -> None:
        """验证参数范围与离散调度输入。"""
        _validate_positive(self)
        if self.min_mass_kg > self.max_mass_kg:
            raise ValueError("虚拟质量上下限倒置")


def fresh_stiffness(snapshot: StiffnessSnapshot | None, now_s: float, max_age_s: float) -> bool:
    """valid 只表示曾成功估计，必须另行检查最近更新时间。"""
    return bool(
        snapshot is not None
        and snapshot.valid
        and math.isfinite(snapshot.value_n_per_m)
        and snapshot.value_n_per_m > 0
        and snapshot.last_update_time_s is not None
        and 0 <= now_s - snapshot.last_update_time_s <= max_age_s
    )


class StiffnessPreload:
    """先受限探测，再一次性锁定接触目标；斜坡与确认均在 preload 内。"""

    def __init__(
        self,
        config: StiffnessPreloadConfig,
        *,
        now_s: float,
        closure_m: float,
        force_n: float,
        rate_n_s: float,
    ) -> None:
        """保存接触参考和有限探测预算。"""
        if (
            not all(math.isfinite(v) for v in (now_s, closure_m, force_n, rate_n_s))
            or rate_n_s <= 0
        ):
            raise ValueError("预载输入必须有限且速率为正")
        self.config = config
        self.started_s = now_s
        self.closure_m = closure_m
        self.rate_n_s = rate_n_s
        self.stage = "identifying"
        self.reason = "waiting_estimate"
        self.accepted_stiffness: float | None = None
        self._updates: deque[tuple[float, float]] = deque()
        self._sample_id = None
        self._ramp(now_s, min(config.probe_force_n, max(0.0, force_n)), config.probe_force_n)

    def _ramp(self, now_s: float, start: float, goal: float) -> None:
        """五次斜坡保证峰值目标速率不超过预载限制。"""
        self.ramp_started_s, self.start_force_n, self.goal_n = now_s, start, goal
        self.duration_s = 1.875 * abs(goal - start) / self.rate_n_s

    def sample(self, now_s: float) -> tuple[float, float, float]:
        """返回预载目标及其一、二阶导数。"""
        if self.duration_s == 0:
            return self.goal_n, 0.0, 0.0
        u = min(1.0, max(0.0, (now_s - self.ramp_started_s) / self.duration_s))
        delta = self.goal_n - self.start_force_n
        return (
            self.start_force_n + delta * (10 * u**3 - 15 * u**4 + 6 * u**5),
            delta * 30 * u**2 * (1 - u) ** 2 / self.duration_s,
            delta * (60 * u - 180 * u**2 + 120 * u**3) / self.duration_s**2,
        )

    def ready(self, now_s: float) -> bool:
        """锁定后完成最终斜坡才允许累计预载稳定时间。"""
        return self.stage == "establishing" and now_s - self.ramp_started_s >= self.duration_s

    def update(self, snapshot: StiffnessSnapshot | None, *, now_s: float, closure_m: float) -> bool:
        """消费新估计；返回是否刚锁定目标，位移超界交给故障保持。"""
        c = self.config
        if closure_m - self.closure_m > c.max_closure_m:
            raise RuntimeError("刚度预载超过实测闭合增量上限")
        if self.stage != "identifying":
            return False
        while self._updates and now_s - self._updates[0][0] > c.max_estimate_age_s:
            self._updates.popleft()
        fresh = fresh_stiffness(snapshot, now_s, c.max_estimate_age_s)
        if not fresh:
            self._updates.clear()
        elif snapshot.updated and snapshot.sample_id != self._sample_id:
            self._sample_id = snapshot.sample_id
            value = snapshot.value_n_per_m
            if self._updates and max(value, *(v for _, v in self._updates)) > (
                1 + c.consistency
            ) * min(value, *(v for _, v in self._updates)):
                self._updates.clear()
            self._updates.append((snapshot.last_update_time_s, value))
        trusted = (
            len(self._updates) >= c.min_updates
            and self._updates[-1][0] - self._updates[0][0] >= c.stable_s
        )
        if trusted:
            self.accepted_stiffness = sum(v for _, v in self._updates) / len(self._updates)
            goal = min(c.max_force_n, max(c.min_force_n, self.accepted_stiffness * c.indentation_m))
            self.reason = "stiffness_locked"
        elif now_s - self.started_s >= c.timeout_s:
            goal = c.probe_force_n
            self.reason = "fallback_insufficient_estimate"
        else:
            return False
        start = self.sample(now_s)[0]
        self._ramp(now_s, start, goal)
        self.stage = "establishing"
        return True

    def trace_fields(self) -> dict[str, object]:
        """记录回退与接触底力，避免把初值当成有效估计。"""
        return {
            "stiffness_preload_stage": self.stage,
            "stiffness_preload_reason": self.reason,
            "contact_force_goal_n": self.goal_n,
            "preload_stiffness_used_n_per_m": self.accepted_stiffness,
        }


class StiffnessAdmittance:
    """对虚拟质量和阻尼作连续调度，保持 MIT 增益不变。"""

    def __init__(self, config: StiffnessAdmittanceConfig) -> None:
        """新接触段不复用上段刚度。"""
        self.config = config
        self.filtered_k: float | None = None
        self.reason = "waiting_estimate"

    def seed(
        self,
        admittance: SecondOrderAdmittance,
        *,
        stiffness_n_per_m: float,
        dt_s: float,
    ) -> None:
        """在生命周期边界直接建立匹配参数，并以零速度连续交接位置。"""
        if (
            not math.isfinite(stiffness_n_per_m)
            or stiffness_n_per_m <= 0
            or not math.isfinite(dt_s)
            or dt_s <= 0
        ):
            raise ValueError("导纳初始化刚度与周期必须为正有限数值")
        c = self.config
        mass = min(
            c.max_mass_kg,
            max(
                c.min_mass_kg,
                (admittance.stiffness_n_m + stiffness_n_per_m) / c.bandwidth_rad_s**2,
            ),
        )
        damping = 2 * c.damping_ratio * c.bandwidth_rad_s * mass
        self.filtered_k = stiffness_n_per_m
        self._check_discrete(mass, damping, admittance.stiffness_n_m, dt_s)
        admittance.mass_kg = mass
        admittance.damping_ns_m = damping
        # 预载已满足稳定窗口；交接时保留位移以保证 q_des 连续，并清除残余动能，
        # 避免质量突变把旧速度状态映射为不同的虚拟动能。
        admittance.velocity_m_s = 0.0
        admittance.acceleration_m_s2 = 0.0
        self.reason = "seeded_from_preload"

    def update(
        self,
        admittance: SecondOrderAdmittance,
        snapshot: StiffnessSnapshot | None,
        *,
        now_s: float,
        dt_s: float,
    ) -> None:
        """有限速率调整参数；陈旧估计冻结参数，不重置运动状态。"""
        c = self.config
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("导纳调度周期必须为正有限数值")
        if admittance.damping_ns_m <= 0:
            raise ValueError("自适应导纳要求正阻尼")
        if not fresh_stiffness(snapshot, now_s, c.max_estimate_age_s):
            self.reason = "waiting_estimate" if self.filtered_k is None else "frozen_stale_estimate"
            if self.filtered_k is not None:
                self._check_discrete(
                    admittance.mass_kg, admittance.damping_ns_m, admittance.stiffness_n_m, dt_s
                )
            return
        value = snapshot.value_n_per_m
        alpha = -math.expm1(-dt_s / c.filter_tau_s)
        self.filtered_k = (
            value
            if self.filtered_k is None
            else self.filtered_k + alpha * (value - self.filtered_k)
        )
        desired = min(
            c.max_mass_kg,
            max(c.min_mass_kg, (admittance.stiffness_n_m + self.filtered_k) / c.bandwidth_rad_s**2),
        )
        factor = math.exp(min(c.relative_rate_per_s * dt_s, 1.0))
        mass = min(admittance.mass_kg * factor, max(admittance.mass_kg / factor, desired))
        desired_b = 2 * c.damping_ratio * c.bandwidth_rad_s * mass
        # 质量、阻尼分别限速，初次启用也不使阻尼突跳。
        old_b = admittance.damping_ns_m
        damping = min(old_b * factor, max(old_b / factor, desired_b))
        # 半隐式欧拉的局部接触模型要求 2*B*dt+(K+k)*dt² < 4*M。
        self._check_discrete(mass, damping, admittance.stiffness_n_m, dt_s)
        admittance.mass_kg, admittance.damping_ns_m = mass, damping
        self.reason = "adapting"

    def _check_discrete(self, mass: float, damping: float, stiffness: float, dt_s: float) -> None:
        """冻结参数也不能跳过周期变长时的局部数值边界。"""
        if 2 * damping * dt_s + (stiffness + self.filtered_k) * dt_s**2 >= 4 * mass:
            raise RuntimeError("自适应导纳超出局部离散稳定边界")

    def trace_fields(self, admittance: SecondOrderAdmittance) -> dict[str, object]:
        """记录实际消费的刚度与参数，而非未执行的候选。"""
        return {
            "admittance_stiffness_used_n_per_m": self.filtered_k,
            "admittance_adaptation_reason": self.reason,
            "admittance_mass_kg": admittance.mass_kg,
            "admittance_damping_ns_m": admittance.damping_ns_m,
            "admittance_stiffness_n_m": admittance.stiffness_n_m,
        }

"""固定分侧摩擦先验下的绝对承载需求与单调目标调度。"""

from dataclasses import dataclass, fields
import math

from ..tactile.multirate import FilteredTangentialLoad


@dataclass(frozen=True, slots=True)
class AdaptiveLoadConfig:
    """初步验证参数；先验与速率上限仍需在目标材料和真机上标定。"""

    left_friction: float = 0.6
    right_friction: float = 0.6
    safety_factor: float = 1.5
    min_force_n: float = 0.5
    max_force_n: float = 8.0
    max_force_rate_n_s: float = 1.0
    gap_gain_per_s: float = 8.0
    load_rate_gain: float = 1.0
    filter_tau_s: float = 0.05

    def __post_init__(self) -> None:
        """拒绝无效先验和不满足范围约束的参数。"""
        for item in fields(self):
            value = getattr(self, item.name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{item.name} 必须为非负有限数值")
        if (
            min(
                self.left_friction,
                self.right_friction,
                self.max_force_rate_n_s,
                self.gap_gain_per_s,
                self.filter_tau_s,
            )
            <= 0
        ):
            raise ValueError("摩擦先验、速率上限、缺口增益和滤波时间常数必须为正")
        if self.safety_factor < 1 or self.max_force_n < self.min_force_n:
            raise ValueError("安全系数至少为 1，目标力上下限必须有序")


@dataclass(frozen=True, slots=True)
class AdaptiveLoadCommand:
    """平均单侧目标、未裁剪承载需求及本次调度诊断。"""

    target_force_n: float
    target_force_rate_n_s: float
    raw_target_force_n: float
    limited_by: tuple[str, ...]
    load_force_n: float
    schedule_gap_n: float
    measured_tangential_force_n: float
    load_rate_n_s: float
    capacity_limited: bool


class AdaptiveLoadScheduler:
    """只消费分侧实测切向合力模，不接收重量、外加载荷或真实摩擦。

    每次 update 对应一个新观测，dt 为实际样本间隔。生命周期负责新鲜度
    门禁与激活时刻；预载真实承载不作为零偏扣除。第一版没有风险或摩擦更新权限。
    """

    def __init__(self, config: AdaptiveLoadConfig) -> None:
        """从最低抓力开始，首样本直接播种滤波以保留已有载荷。"""
        self.config = config
        self.target_force_n = config.min_force_n
        self._filtered: tuple[float, float] | None = None

    def update(
        self,
        *,
        left_tangential_n: float,
        right_tangential_n: float,
        dt: float,
        left_friction: float | None = None,
        right_friction: float | None = None,
        goal_floor_n: float = 0.0,
        risk_rate_n_s: float = 0.0,
        pause_increase: bool = False,
        filtered_load: FilteredTangentialLoad | None = None,
    ) -> AdaptiveLoadCommand:
        """以新样本推进目标，返回调度后的剩余缺口。

        Args:
            left_tangential_n: 左侧先求切向合力再取模的结果，单位 N。
            right_tangential_n: 右侧先求切向合力再取模的结果，单位 N。
            dt: 相邻观测间隔，单位 s，必须为正。
            left_friction: 质量门禁后使用的左侧摩擦；省略时使用先验。
            right_friction: 质量门禁后使用的右侧摩擦；省略时使用先验。
            goal_floor_n: 已消费风险事件留下的目标下界。
            risk_rate_n_s: 有界风险附加增长率。
            pause_increase: 执行受限时暂停目标增长，仍更新需求诊断。
            filtered_load: 采样侧已滤波承载；提供时不重复低通，使用采样侧变化率。

        Returns:
            只增不减、满足幅值和速率限制的平均单侧目标与诊断。
        """
        if not all(
            not isinstance(v, bool) and math.isfinite(v)
            for v in (left_tangential_n, right_tangential_n, dt)
        ):
            raise ValueError("观测与时间间隔必须有限")
        if min(left_tangential_n, right_tangential_n) < 0 or dt <= 0:
            raise ValueError("切向合力模不得为负，时间间隔必须为正")
        c = self.config
        mu_left = c.left_friction if left_friction is None else left_friction
        mu_right = c.right_friction if right_friction is None else right_friction
        if not all(math.isfinite(v) for v in (mu_left, mu_right, goal_floor_n, risk_rate_n_s)):
            raise ValueError("摩擦与风险请求必须有限")
        if min(mu_left, mu_right) <= 0 or min(goal_floor_n, risk_rate_n_s) < 0:
            raise ValueError("摩擦必须为正，风险请求不得为负")
        sample = (left_tangential_n, right_tangential_n)
        previous = self._filtered
        alpha = -math.expm1(-dt / c.filter_tau_s)
        filtered = (
            sample
            if previous is None
            else tuple(old + alpha * (new - old) for old, new in zip(previous, sample, strict=True))
        )
        total = sum(filtered)
        # 使用滤波状态增量估计载荷趋势，首样本不制造突变导数。
        load_rate = 0.0 if previous is None else (total - sum(previous)) / dt
        if filtered_load is not None:
            filtered = (filtered_load.left_n, filtered_load.right_n)
            total = sum(filtered)
            load_rate = filtered_load.rate_n_s
        raw = c.safety_factor * max(filtered[0] / mu_left, filtered[1] / mu_right)
        if not all(math.isfinite(v) for v in (total, load_rate, raw)):
            raise ValueError("承载需求或载荷趋势计算溢出")
        load = min(c.max_force_n, max(c.min_force_n, raw))
        goal = max(self.target_force_n, load, min(c.max_force_n, goal_floor_n))
        gap = goal - self.target_force_n
        rate = min(
            c.max_force_rate_n_s,
            c.gap_gain_per_s * gap + c.load_rate_gain * max(0, load_rate) + risk_rate_n_s,
        )
        increment = 0.0 if pause_increase else min(gap, rate * dt)
        self._filtered = filtered
        self.target_force_n += increment
        limited = []
        if raw < c.min_force_n:
            limited.append("minimum")
        if raw > c.max_force_n:
            limited.append("maximum")
        if increment < gap:
            limited.append("rate")
        if pause_increase:
            limited.append("execution")
        return AdaptiveLoadCommand(
            self.target_force_n,
            increment / dt,
            raw,
            tuple(limited),
            load,
            goal - self.target_force_n,
            total,
            load_rate,
            raw > c.max_force_n,
        )

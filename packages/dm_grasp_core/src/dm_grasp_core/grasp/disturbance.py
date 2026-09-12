"""仿真与 DMgripper 真机共用的纯触觉增力计算。"""

from collections import deque
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class DisturbancePolicyParameters:
    """不依赖配置框架的增力参数。"""

    strategy: str = "dynamic_step"
    detector: str = "shear_increase"
    initial_force_n: float = 0.8
    max_force_n: float = 5.0
    max_force_rate_n_s: float = 30.0
    update_period_s: float = 0.01
    filter_tau_s: float = 0.005
    confirm_time_s: float = 0.006
    shear_threshold_n: float = 0.06
    shear_gain: float = 1.0
    fixed_step_n: float = 0.15
    min_step_n: float = 0.03
    max_step_n: float = 0.3
    ratio_window_s: float = 0.06
    normal_delta_floor_n: float = 0.02
    ratio_threshold: float = -0.1
    closure_scale_m: float = 0.01
    contact_floor_n: float = 0.05

    def __post_init__(self):
        """验证纯计算参数的有限性、正值和边界。"""
        if self.strategy not in {"constant", "fixed_step", "dynamic_step"}:
            raise ValueError("unknown disturbance strategy")
        if self.detector not in {"shear_increase", "force_ratio"}:
            raise ValueError("unknown disturbance detector")
        for field in fields(self):
            if field.name in {"strategy", "detector"}:
                continue
            value = getattr(self, field.name)
            if isinstance(value, bool) or not math.isfinite(value):
                raise ValueError("disturbance parameters must be finite numbers")
            if field.name != "ratio_threshold" and value <= 0:
                raise ValueError("disturbance scales must be positive")
        if self.max_force_n < self.initial_force_n or self.max_step_n < self.min_step_n:
            raise ValueError("force and step limits must be ordered")


@dataclass(frozen=True)
class DisturbanceCommand:
    """一次触觉更新的目标和检测诊断。"""

    target_force_n: float
    target_force_rate_n_s: float
    measured_tangential_force_n: float
    trigger_score: float
    trigger_active: bool
    ratio: float | None
    ratio_valid: bool
    increase_count: int


class TactileDisturbancePolicy:
    """从固定保持基线产生只增不减的目标力。

    shear_increase 是工程扩展：切向载荷增长对应有限目标包络，避免
    持续载荷在每个周期无休止地加力。force_ratio 为独立的论文判据对照，
    不把无效比值替换成零，也不将物体运动输入控制。
    """

    def __init__(self, config: DisturbancePolicyParameters):
        """初始化固定目标和物理时间窗口。"""
        self.config = config
        self.target = float(config.initial_force_n)
        self.requested = self.target
        self.time = 0.0
        self.filtered = None
        self.baseline = None
        self.initial_closure = None
        self.history = deque()
        self.confirmed_s = 0.0
        self.action_elapsed = 0.0
        self.increase_count = 0

    def update(
        self,
        *,
        left_normal_n: float,
        right_normal_n: float,
        tangential_force_n: float,
        signed_tangential_force_n: float,
        closure_m: float,
        dt: float,
        enabled: bool,
    ) -> DisturbanceCommand:
        """用两侧触觉和夹爪闭合量更新，不接收摩擦、外载或物体状态。

        Args:
            left_normal_n: 左侧压缩力。
            right_normal_n: 右侧压缩力。
            tangential_force_n: 两侧各自切向合力模的和。
            signed_tangential_force_n: 左侧传感器固定局部轴的带符号切向合力。
            closure_m: 由夹爪自身编码器换算的闭合距离。
            dt: 当前触觉更新的物理间隔。
            enabled: 是否已完成初始稳定保持。

        Returns:
            单调目标力、有效性和触发诊断。
        """
        values = (
            left_normal_n,
            right_normal_n,
            tangential_force_n,
            signed_tangential_force_n,
            closure_m,
            dt,
        )
        if not all(math.isfinite(v) for v in values) or dt <= 0:
            raise ValueError("tactile inputs must be finite and dt positive")
        if tangential_force_n < 0:
            raise ValueError("tangential magnitude must be nonnegative")
        c = self.config
        self.time += dt
        sample = (
            tangential_force_n,
            left_normal_n,
            signed_tangential_force_n,
        )
        a = -math.expm1(-dt / c.filter_tau_s)
        self.filtered = (
            sample
            if self.filtered is None
            else tuple(old + a * (new - old) for old, new in zip(self.filtered, sample))
        )
        shear, normal, signed = self.filtered
        self.history.append((self.time, normal, signed))
        cutoff = self.time - c.ratio_window_s
        while len(self.history) > 1 and self.history[1][0] <= cutoff:
            self.history.popleft()
        first_t, first_n, first_s = self.history[0]
        dn = normal - first_n
        valid = self.time - first_t >= c.ratio_window_s - 1e-9 and abs(dn) >= c.normal_delta_floor_n
        ratio = (signed - first_s) / dn if valid else None
        contact = min(left_normal_n, right_normal_n) >= c.contact_floor_n
        if not enabled:
            self.baseline = shear
            self.initial_closure = closure_m
            self.confirmed_s = 0.0
            self.action_elapsed = 0.0
        if self.baseline is None or self.initial_closure is None:
            raise ValueError("policy needs a disabled baseline sample before activation")
        rise = max(0.0, shear - self.baseline)
        envelope = min(c.max_force_n, c.initial_force_n + c.shear_gain * rise)
        if c.detector == "shear_increase":
            score = rise / c.shear_threshold_n
            evidence = score >= 1 and self.requested < envelope - 1e-9
            alpha = min(1.0, max(0.0, 1 - 1 / max(score, 1.0)))
        else:
            score = abs(ratio) if ratio is not None else 0.0
            evidence = ratio is not None and ratio < c.ratio_threshold
            alpha = min(1.0, score)
            envelope = c.max_force_n
        evidence = enabled and contact and evidence
        self.confirmed_s = self.confirmed_s + dt if evidence else 0.0
        active = evidence and self.confirmed_s + 1e-12 >= c.confirm_time_s
        previous = self.target
        if enabled:
            self.action_elapsed += dt
        if self.action_elapsed + 1e-12 >= c.update_period_s:
            # 每次实际更新只发出一次动作，低速采样不得补发过时动作。
            periods = max(1, math.floor((self.action_elapsed + 1e-12) / c.update_period_s))
            self.action_elapsed = max(0.0, self.action_elapsed - periods * c.update_period_s)
            if active and c.strategy != "constant":
                pe = min(1.0, max(0.0, closure_m - self.initial_closure) / c.closure_scale_m)
                raw = math.exp(alpha) + math.exp(-pe)
                low, high = 1 + math.exp(-1), math.e + 1
                dynamic = c.min_step_n + (c.max_step_n - c.min_step_n) * (raw - low) / (high - low)
                step = c.fixed_step_n if c.strategy == "fixed_step" else dynamic
                previous_request = self.requested
                self.requested = max(self.requested, min(envelope, self.requested + step))
                self.increase_count += int(self.requested > previous_request + 1e-12)
        if enabled and contact:
            self.target = min(self.requested, previous + c.max_force_rate_n_s * dt)
        return DisturbanceCommand(
            self.target,
            (self.target - previous) / dt,
            shear,
            score,
            active,
            ratio,
            valid,
            self.increase_count,
        )

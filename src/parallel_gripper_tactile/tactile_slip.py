"""仅从触觉时序识别接触状态变化并冻结摩擦能力候选。"""

from collections import deque
from dataclasses import asdict, dataclass
import math

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .friction_estimation import FrictionEstimate, FrictionEstimatorConfig
from .taxel_friction import TaxelFrictionObservation


class TactileSlipConfig(BaseModel):
    """触觉变化评分参数；阈值为待实验验证的工程初值。"""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
    smoothing_s: float = Field(default=0.01, gt=0)
    trend_window_s: float = Field(default=0.06, gt=0)
    arming_increase: float = Field(default=0.03, gt=0)
    minimum_ratio_coherence: float = Field(default=0.8, gt=0, le=1)
    arming_slope: float = Field(default=0.08, gt=0)
    distribution_scale: float = Field(default=0.08, gt=0)
    asymmetry_scale: float = Field(default=0.15, gt=0)
    high_ratio_fraction: float = Field(default=0.85, gt=0, le=1)
    weight_saturation: float = Field(default=0.7, ge=0)
    weight_distribution: float = Field(default=0.15, ge=0)
    weight_asymmetry: float = Field(default=0.05, ge=0)
    weight_high_utilization: float = Field(default=0.1, ge=0)
    candidate_threshold: float = Field(default=0.55, gt=0, le=1)
    confirm_threshold: float = Field(default=0.65, gt=0, le=1)
    release_threshold: float = Field(default=0.4, ge=0, lt=1)
    confirm_duration_s: float = Field(default=0.012, gt=0)

    @model_validator(mode="after")
    def validate_thresholds(self):
        """约束滞回关系与评分权重。"""
        if not self.release_threshold < self.candidate_threshold <= self.confirm_threshold:
            raise ValueError("require release < candidate <= confirm")
        if sum(self.weights) <= 0:
            raise ValueError("score weights must have positive sum")
        return self

    @property
    def weights(self):
        """返回各归一化证据的权重。"""
        return (
            self.weight_saturation,
            self.weight_distribution,
            self.weight_asymmetry,
            self.weight_high_utilization,
        )


@dataclass(frozen=True)
class TactileFeatures:
    """检测器允许读取的全部输入，均由触觉生成。"""

    rho_global: float = 0.0
    rho_q95: float = 0.0
    high_utilization_fraction: float = 0.0
    distribution_change: float = 0.0
    d_rho_global_dt: float = 0.0
    d_rho_q95_dt: float = 0.0
    d_distribution_dt: float = 0.0
    shear_asymmetry: float = 0.0
    normal_asymmetry: float = 0.0
    total_normal_n: float = 0.0
    valid: bool = False


class TactileFeatureComputer:
    """平滑有效触点的力，并以短窗拟合计算变化率。"""

    def __init__(self, config: TactileSlipConfig):
        """保存配置并初始化时序。"""
        self.config = config
        self.reset()

    def reset(self):
        """清空滤波状态和分布参考。"""
        self.time = 0.0
        self.history = deque()
        self.smoothed = None
        self.mask = None

    def compute_tactile_features(self, observation: TaxelFrictionObservation, dt: float):
        """接触集合变化时重建窗口，避免把触点进入误作重分布。"""
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")
        normal = np.stack((observation.left_normal_force_n, observation.right_normal_force_n))
        shear = np.stack((observation.left_shear_force_n, observation.right_shear_force_n))
        mask = np.stack((observation.left_contact_mask, observation.right_contact_mask))
        if not np.isfinite(normal).all() or not np.isfinite(shear).all():
            raise ValueError("tactile forces must be finite")
        if self.mask is None or not np.array_equal(mask, self.mask):
            self.history.clear()
            self.smoothed = None
        self.mask = mask.copy()
        self.time += dt
        values = np.stack((np.where(mask, normal, 0), np.where(mask, shear, 0)))
        alpha = 1 - math.exp(-dt / self.config.smoothing_s)
        self.smoothed = (
            values if self.smoothed is None else self.smoothed + alpha * (values - self.smoothed)
        )
        normal, shear = self.smoothed
        valid = mask & (normal > 1e-12)
        if not valid.any() or not all(np.any(side) for side in valid):
            self.history.clear()
            return TactileFeatures()
        ratios = shear[valid] / normal[valid]
        rho = float(shear.sum() / normal.sum())
        q95 = float(np.quantile(ratios, 0.95))
        weights = shear.ravel() / max(float(shear.sum()), 1e-12)
        distribution = (
            0.0 if not self.history else float(np.abs(weights - self.history[0][4]).sum())
        )
        self.history.append((self.time, rho, q95, distribution, weights.copy()))
        while len(self.history) > 2 and self.time - self.history[1][0] > self.config.trend_window_s:
            self.history.popleft()
        slopes = np.zeros(3)
        if len(self.history) > 2:
            samples = np.array([entry[:4] for entry in self.history])
            times = samples[:, 0] - samples[:, 0].mean()
            slopes = times @ samples[:, 1:4] / max(float(times @ times), 1e-12)
        side_n = normal.sum(axis=(1, 2))
        side_t = shear.sum(axis=(1, 2))
        return TactileFeatures(
            rho,
            q95,
            float(np.mean(ratios >= self.config.high_ratio_fraction * q95)),
            distribution,
            *map(float, slopes),
            float(abs(side_t[0] - side_t[1]) / max(side_t.sum(), 1e-12)),
            float(abs(side_n[0] - side_n[1]) / max(side_n.sum(), 1e-12)),
            float(normal.sum()),
            self.time - self.history[0][0] >= self.config.trend_window_s * 0.9,
        )


class TactileFrictionEstimator:
    """仅接受触觉特征；确认表示起滑代理事件，而非微滑真值。"""

    def __init__(self, config: FrictionEstimatorConfig, detector: TactileSlipConfig):
        """分离估计窗口参数和检测评分参数。"""
        self.config = config
        self.detector = detector
        self.reset()

    def reset(self):
        """重置候选状态与冻结估计。"""
        self.samples = deque(maxlen=self.config.window_size)
        self.state = "stable"
        self.duration = 0.0
        self.armed = False
        self.minimum_ratio = math.inf
        self.mu_raw = None
        self.mu_hat = self.config.fallback_friction_coefficient
        self.features = TactileFeatures()
        self.score = 0.0
        self.components = (0.0, 0.0, 0.0, 0.0)

    def update_slip_detector(self, features: TactileFeatures, dt: float):
        """以评分滞回和连续确认时间更新状态，候选期间停止估计采样。"""
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")
        if any(not math.isfinite(v) for v in asdict(features).values()):
            raise ValueError("features must be finite")
        if self.state == "incipient_slip_confirmed":
            return self.finalize()
        self.features = features
        c = self.detector
        if not features.valid or features.total_normal_n < self.config.min_total_normal_force_n:
            self.reset()
            self.features = features
            return self.finalize()
        self.minimum_ratio = min(self.minimum_ratio, features.rho_global)
        if (
            features.rho_global - self.minimum_ratio >= c.arming_increase
            and features.d_rho_global_dt >= c.arming_slope
        ):
            self.armed = True
        saturation = float(np.clip(1 - max(features.d_rho_global_dt, 0) / c.arming_slope, 0, 1))
        self.components = (
            saturation,
            min(1.0, features.distribution_change / c.distribution_scale),
            min(1.0, max(features.shear_asymmetry, features.normal_asymmetry) / c.asymmetry_scale),
            features.high_utilization_fraction,
        )
        self.score = (
            float(np.dot(c.weights, self.components) / sum(c.weights)) if self.armed else 0.0
        )
        # 局部与整体比值相差过大时，不把局部载荷变化当作总体摩擦极限。
        if features.rho_global / max(features.rho_q95, 1e-12) < c.minimum_ratio_coherence:
            self.score = 0.0
        if self.state == "stable" and self.score >= c.candidate_threshold:
            self.state = "incipient_slip_candidate"
        elif self.score < c.release_threshold:
            self.state = "stable"
        if self.state == "incipient_slip_candidate":
            self.duration = self.duration + dt if self.score >= c.confirm_threshold else 0.0
            if self.duration + 1e-12 >= c.confirm_duration_s:
                self.state = "incipient_slip_confirmed"
                self.estimate_friction()
        else:
            self.duration = 0.0
            self.samples.append(features.rho_global)
        return self.finalize()

    def estimate_friction(self):
        """仅使用候选开始前窗口，按高分位折减后锁存。"""
        if len(self.samples) >= self.config.min_samples:
            self.mu_raw = float(np.quantile(self.samples, self.config.estimate_quantile))
            self.mu_hat = float(
                np.clip(
                    self.mu_raw * self.config.safety_discount,
                    self.config.min_friction_coefficient,
                    self.config.max_friction_coefficient,
                )
            )

    def finalize(self):
        """兼容实验结果结构；残差旧字段不再作为在线量。"""
        confirmed = self.state == "incipient_slip_confirmed"
        return FrictionEstimate(
            state="estimated" if confirmed and self.mu_raw is not None else "finalized_fallback",
            slip_detected=confirmed,
            consecutive_mismatch_count=0,
            mismatch_duration_s=self.duration,
            support_residual_n=0.0,
            support_utilization=0.0,
            sample_count=len(self.samples),
            friction_coefficient=self.mu_hat,
            raw_friction_coefficient=self.mu_raw,
            using_fallback=self.mu_raw is None,
            ratio_slope_per_s=self.features.d_rho_global_dt,
            trend_armed=self.armed,
            side_ratio_difference=0.0,
        )

    def trace_fields(self):
        """输出全部特征及各评分分量，便于独立复核。"""
        return {
            **asdict(self.features),
            "slip_state": self.state,
            "incipient_slip_score": self.score,
            "candidate_threshold": self.detector.candidate_threshold,
            "confirm_threshold": self.detector.confirm_threshold,
            "release_threshold": self.detector.release_threshold,
            **dict(
                zip(
                    (
                        "score_saturation",
                        "score_distribution",
                        "score_asymmetry",
                        "score_high_utilization",
                    ),
                    self.components,
                    strict=True,
                )
            ),
        }

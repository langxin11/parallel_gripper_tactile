"""由接触刚度映射的力变化率 PID 外环。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class StiffnessRateConfig:
    """力误差到期望力变化率、关节速度和位置修正的参数。

    Attributes:
        kp_s_inv: 比例增益，单位 1/s。
        ki_s_inv2: 积分增益，单位 1/s²。
        kd: 测量微分增益，无量纲。
        max_force_rate_n_s: 期望法向力变化率绝对值上限 (N/s)。
        max_joint_velocity_rad_s: 电机侧目标速度绝对值上限 (rad/s)。
    """

    kp_s_inv: float = 20.0
    ki_s_inv2: float = 0.0
    kd: float = 0.0
    max_force_rate_n_s: float = 50.0
    max_joint_velocity_rad_s: float = 0.5

    def __post_init__(self) -> None:
        """拒绝负增益和非正安全边界。"""
        if self.kp_s_inv < 0 or self.ki_s_inv2 < 0 or self.kd < 0:
            raise ValueError("stiffness-rate PID gains must be non-negative")
        if self.max_force_rate_n_s <= 0 or self.max_joint_velocity_rad_s <= 0:
            raise ValueError("stiffness-rate limits must be positive")


@dataclass(frozen=True, slots=True)
class StiffnessRateStep:
    """一次刚度映射速度控制更新的结果。"""

    position_adjustment_rad: float
    force_rate_command_n_s: float
    joint_velocity_command_rad_s: float
    force_rate_limited: bool
    joint_velocity_limited: bool


class StiffnessRateController:
    """将力变化率 PID 输出经刚度映射后积分为持久位置修正。"""

    def __init__(self, config: StiffnessRateConfig, *, max_position_adjustment_rad: float) -> None:
        """保存配置并初始化连续时间状态。"""
        if max_position_adjustment_rad <= 0:
            raise ValueError("max_position_adjustment_rad must be positive")
        self.config = config
        self.max_position_adjustment_rad = float(max_position_adjustment_rad)
        self.reset()

    def reset(self, *, measured_force_n: float | None = None) -> None:
        """清除积分状态，并可用当前测量初始化微分历史。"""
        self.integral_error_n_s = 0.0
        self.position_adjustment_rad = 0.0
        self.previous_force_n = measured_force_n

    def step(
        self,
        *,
        force_error_n: float,
        measured_force_n: float,
        stiffness_n_per_m: float,
        closure_jacobian_m_per_rad: float,
        dt_s: float,
    ) -> StiffnessRateStep:
        """计算力变化率，映射为关节速度并按实际周期积分。"""
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        if stiffness_n_per_m <= 0 or closure_jacobian_m_per_rad <= 0:
            raise ValueError("stiffness and closure jacobian must be positive")

        derivative_n_s = (
            0.0
            if self.previous_force_n is None
            else (measured_force_n - self.previous_force_n) / dt_s
        )
        candidate_integral = self.integral_error_n_s + force_error_n * dt_s
        raw_force_rate = (
            self.config.kp_s_inv * force_error_n
            + self.config.ki_s_inv2 * candidate_integral
            - self.config.kd * derivative_n_s
        )
        force_rate = float(
            np.clip(
                raw_force_rate,
                -self.config.max_force_rate_n_s,
                self.config.max_force_rate_n_s,
            )
        )
        force_rate_limited = not np.isclose(force_rate, raw_force_rate, rtol=0.0, atol=1e-12)

        raw_joint_velocity = force_rate / (stiffness_n_per_m * closure_jacobian_m_per_rad)
        joint_velocity = float(
            np.clip(
                raw_joint_velocity,
                -self.config.max_joint_velocity_rad_s,
                self.config.max_joint_velocity_rad_s,
            )
        )
        joint_velocity_limited = not np.isclose(
            joint_velocity,
            raw_joint_velocity,
            rtol=0.0,
            atol=1e-12,
        )

        # 饱和且误差仍推动输出继续进入同向饱和时冻结积分，防止 windup。
        saturated = force_rate_limited or joint_velocity_limited
        effective_force_rate = joint_velocity * stiffness_n_per_m * closure_jacobian_m_per_rad
        saturation_direction = raw_force_rate - effective_force_rate
        if not saturated or force_error_n * saturation_direction <= 0:
            self.integral_error_n_s = candidate_integral

        raw_adjustment = self.position_adjustment_rad + joint_velocity * dt_s
        adjustment = float(
            np.clip(
                raw_adjustment,
                -self.max_position_adjustment_rad,
                self.max_position_adjustment_rad,
            )
        )
        if not np.isclose(adjustment, raw_adjustment, rtol=0.0, atol=1e-12):
            joint_velocity_limited = True
        self.position_adjustment_rad = adjustment
        self.previous_force_n = measured_force_n
        return StiffnessRateStep(
            position_adjustment_rad=adjustment,
            force_rate_command_n_s=force_rate,
            joint_velocity_command_rad_s=joint_velocity,
            force_rate_limited=force_rate_limited,
            joint_velocity_limited=joint_velocity_limited,
        )


__all__ = ["StiffnessRateConfig", "StiffnessRateController", "StiffnessRateStep"]

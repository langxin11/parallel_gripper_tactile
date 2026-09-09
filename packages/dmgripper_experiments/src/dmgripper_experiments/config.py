"""最小真机力跟踪实验配置。"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ForceDemoConfig:
    """沿用 ROS 2 已验证参数的单次真机实验配置。"""

    target_force_n: float = 0.5
    tracking_duration_s: float = 10.0
    control_rate_hz: float = 100.0
    tactile_startup_timeout_s: float = 5.0
    tactile_timeout_s: float = 0.2
    tactile_bias_settle_s: float = 2.0
    tactile_cutoff_hz: float = 10.0
    tactile_filter_reset_gap_s: float = 0.1
    contact_on_n: float = 0.2
    contact_on_stable_s: float = 0.1
    contact_off_n: float = 0.1
    contact_off_stable_s: float = 0.15
    contact_transition_s: float = 0.15
    force_ceiling_n: float = 2.0
    max_force_imbalance_n: float = 1.0
    zero_force_threshold_n: float = 0.1
    zero_force_stable_s: float = 0.5
    zero_force_timeout_s: float = 5.0
    approach_closure_velocity_m_s: float = 0.01
    approach_closure_acceleration_m_s2: float = 0.025
    approach_closure_jerk_m_s3: float = 0.1
    approach_feedforward_force_n: float = 2.0
    approach_feedforward_ratio: float = 0.5
    approach_endpoint_hold_s: float = 0.5
    return_closure_velocity_m_s: float = 0.012
    return_closure_acceleration_m_s2: float = 0.025
    return_closure_jerk_m_s3: float = 0.1
    return_timeout_s: float = 5.0
    return_settle_timeout_s: float = 2.0
    return_position_tolerance_rad: float = 0.02
    mit_kp: float = 2.0
    mit_kd: float = 0.5
    return_mit_kp: float = 10.0
    return_mit_kd: float = 0.5
    velocity_limit_rad_s: float = 0.3
    torque_limit_nm: float = 4.0
    return_torque_limit_nm: float = 2.0
    admittance_mass_kg: float = 0.02
    admittance_damping_ns_m: float = 0.2
    admittance_stiffness_n_m: float = 1.0

    def __post_init__(self) -> None:
        """拒绝明显无效或彼此矛盾的实验参数。"""
        values = tuple(getattr(self, name) for name in self.__dataclass_fields__)
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
            raise ValueError("真机实验参数必须全部为有限数值")
        positive = (
            self.tracking_duration_s,
            self.control_rate_hz,
            self.tactile_startup_timeout_s,
            self.tactile_timeout_s,
            self.tactile_bias_settle_s,
            self.tactile_cutoff_hz,
            self.tactile_filter_reset_gap_s,
            self.contact_on_stable_s,
            self.contact_off_stable_s,
            self.contact_transition_s,
            self.force_ceiling_n,
            self.zero_force_timeout_s,
            self.approach_closure_velocity_m_s,
            self.approach_closure_acceleration_m_s2,
            self.approach_closure_jerk_m_s3,
            self.return_closure_velocity_m_s,
            self.return_closure_acceleration_m_s2,
            self.return_closure_jerk_m_s3,
            self.return_timeout_s,
            self.return_settle_timeout_s,
            self.return_position_tolerance_rad,
            self.mit_kp,
            self.return_mit_kp,
            self.velocity_limit_rad_s,
            self.torque_limit_nm,
            self.return_torque_limit_nm,
            self.admittance_mass_kg,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("真机实验的时间、频率、上限与正增益必须大于 0")
        if not 0.0 <= self.contact_off_n < self.contact_on_n:
            raise ValueError("接触释放阈值必须非负且小于接触确认阈值")
        if not 0.0 <= self.target_force_n <= self.force_ceiling_n:
            raise ValueError("目标力必须位于 0 与力上限之间")
        if self.max_force_imbalance_n < 0.0 or self.zero_force_threshold_n < 0.0:
            raise ValueError("力差与零力阈值不得为负")
        if (
            not 0.0 <= self.mit_kd <= 5.0
            or not 0.0 <= self.return_mit_kd <= 5.0
            or self.mit_kp > 500.0
            or self.return_mit_kp > 500.0
        ):
            raise ValueError("MIT 增益超出协议范围")
        if self.admittance_damping_ns_m < 0.0 or self.admittance_stiffness_n_m < 0.0:
            raise ValueError("导纳阻尼与刚度不得为负")

"""DMgripper 的导纳状态积分与 MIT 力矩约束。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(slots=True)
class SecondOrderAdmittance:
    """沿夹爪闭合方向积分的二阶导纳。

    Attributes:
        mass_kg: 虚拟质量 (kg)，必须为正数。
        damping_ns_m: 虚拟阻尼 (N·s/m)，不得为负数。
        stiffness_n_m: 虚拟刚度 (N/m)，不得为负数。
        displacement_m: 当前虚拟闭合位移 (m)。
        velocity_m_s: 当前虚拟闭合速度 (m/s)。
        deadband_active: 最近一步是否因力误差位于死区而冻结。
        unloading_blocked: 最近一步是否阻止了反向卸载。
    """

    mass_kg: float
    damping_ns_m: float
    stiffness_n_m: float
    displacement_m: float = 0.0
    velocity_m_s: float = 0.0
    deadband_active: bool = field(default=False, init=False)
    unloading_blocked: bool = field(default=False, init=False)

    def reset(self) -> None:
        """清零虚拟位移和速度，以当前电机位置作为新的参考点。"""
        self.displacement_m = 0.0
        self.velocity_m_s = 0.0
        self.deadband_active = False
        self.unloading_blocked = False

    def step(
        self,
        force_error_n: float,
        dt_s: float,
        maximum_velocity_m_s: float | None = None,
    ) -> tuple[float, float]:
        """以半隐式欧拉法更新二阶导纳状态。

        Args:
            force_error_n: 目标力减测得力，正值推动夹爪继续闭合 (N)。
            dt_s: 本次离散积分步长 (s)。
            maximum_velocity_m_s: 可选的虚拟闭合速度绝对值上限 (m/s)。给定时在
                位移积分前裁剪更新后的速度，确保本步虚拟位移增量不超过
                ``maximum_velocity_m_s * dt_s``；省略时保持既有积分语义。

        Returns:
            tuple[float, float]: 更新后的虚拟位移 (m) 与速度 (m/s)。

        Raises:
            ValueError: 参数或输入不满足有限性、物理符号约束时抛出。
        """
        self.deadband_active = False
        self.unloading_blocked = False
        values = (
            self.mass_kg,
            self.damping_ns_m,
            self.stiffness_n_m,
            force_error_n,
            dt_s,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or self.mass_kg <= 0.0
            or self.damping_ns_m < 0.0
            or self.stiffness_n_m < 0.0
            or dt_s <= 0.0
        ):
            raise ValueError("二阶导纳参数或积分输入无效")
        if maximum_velocity_m_s is not None and (
            not math.isfinite(maximum_velocity_m_s) or maximum_velocity_m_s <= 0.0
        ):
            raise ValueError("二阶导纳虚拟速度上限无效")
        acceleration_m_s2 = (
            force_error_n
            - self.damping_ns_m * self.velocity_m_s
            - self.stiffness_n_m * self.displacement_m
        ) / self.mass_kg
        self.velocity_m_s += acceleration_m_s2 * dt_s
        if maximum_velocity_m_s is not None:
            self.velocity_m_s = min(
                max(self.velocity_m_s, -maximum_velocity_m_s),
                maximum_velocity_m_s,
            )
        self.displacement_m += self.velocity_m_s * dt_s
        if not all(math.isfinite(value) for value in (self.displacement_m, self.velocity_m_s)):
            raise ValueError("二阶导纳积分结果非有限")
        return self.displacement_m, self.velocity_m_s

    def limit_state(
        self,
        minimum_displacement_m: float,
        maximum_displacement_m: float,
        maximum_velocity_m_s: float,
    ) -> tuple[float, float]:
        """约束导纳状态并阻止位置限位处继续积分。

        Args:
            minimum_displacement_m: 相对使能参考点允许的最小闭合位移 (m)。
            maximum_displacement_m: 相对使能参考点允许的最大闭合位移 (m)。
            maximum_velocity_m_s: 允许的最大闭合速度绝对值 (m/s)。

        Returns:
            tuple[float, float]: 约束后的虚拟位移 (m) 与速度 (m/s)。

        Raises:
            ValueError: 约束非有限、次序错误或速度上限不是正数时抛出。
        """
        values = (
            minimum_displacement_m,
            maximum_displacement_m,
            maximum_velocity_m_s,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or minimum_displacement_m > maximum_displacement_m
            or maximum_velocity_m_s <= 0.0
        ):
            raise ValueError("二阶导纳状态约束无效")
        self.displacement_m = min(
            max(self.displacement_m, minimum_displacement_m),
            maximum_displacement_m,
        )
        self.velocity_m_s = min(
            max(self.velocity_m_s, -maximum_velocity_m_s),
            maximum_velocity_m_s,
        )
        if (self.displacement_m <= minimum_displacement_m and self.velocity_m_s < 0.0) or (
            self.displacement_m >= maximum_displacement_m and self.velocity_m_s > 0.0
        ):
            self.velocity_m_s = 0.0
        return self.displacement_m, self.velocity_m_s


def limit_mit_position_for_torque(
    *,
    target_position_rad: float,
    target_velocity_rad_s: float,
    measured_position_rad: float,
    measured_velocity_rad_s: float,
    kp: float,
    kd: float,
    feedforward_torque_nm: float,
    torque_limit_nm: float,
) -> float:
    """按 MIT 合成力矩上限修正目标位置。

    Args:
        target_position_rad: 未约束的目标位置 (rad)。
        target_velocity_rad_s: 目标速度 (rad/s)。
        measured_position_rad: 当前反馈位置 (rad)。
        measured_velocity_rad_s: 当前反馈速度 (rad/s)。
        kp: MIT 位置刚度。
        kd: MIT 速度阻尼。
        feedforward_torque_nm: 前馈力矩 (N·m)。
        torque_limit_nm: 合成力矩绝对值上限 (N·m)。

    Returns:
        float: 保证预测 MIT 合成力矩不超过上限的目标位置 (rad)。

    Raises:
        ValueError: 输入非有限、增益无效或 ``kp`` 不为正时抛出。
    """
    values = (
        target_position_rad,
        target_velocity_rad_s,
        measured_position_rad,
        measured_velocity_rad_s,
        kp,
        kd,
        feedforward_torque_nm,
        torque_limit_nm,
    )
    if (
        not all(math.isfinite(value) for value in values)
        or kp <= 0.0
        or kd < 0.0
        or torque_limit_nm <= 0.0
    ):
        raise ValueError("MIT 合成力矩约束参数无效")
    non_position_torque_nm = (
        kd * (target_velocity_rad_s - measured_velocity_rad_s) + feedforward_torque_nm
    )
    requested_torque_nm = (
        kp * (target_position_rad - measured_position_rad) + non_position_torque_nm
    )
    limited_torque_nm = min(max(requested_torque_nm, -torque_limit_nm), torque_limit_nm)
    return measured_position_rad + (limited_torque_nm - non_position_torque_nm) / kp

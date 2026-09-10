"""二阶直接力矩 LADRC：current observer、临界阻尼 TD 与工程限幅。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True, slots=True)
class TorqueAdrcConfig:
    """二阶直接力矩 LADRC 的控制导向模型与工程约束。

    被控假设为 ``d²F/dt² = f + b0·τ``。名义输入增益按
    ``b0 = scale·K_hat·J(q)/I_eq`` 在线调度，其中 ``K_hat`` 为整体等效
    接触刚度，``J(q)`` 为总闭合行程雅可比，``I_eq`` 为折算到电机输出轴的
    等效惯量，``scale`` 用小信号辨识校准未建模的输入增益。可选线性 TD
    仅安排参考过渡过程；``None`` 保持无显式 TD 结构。

    Attributes:
        equivalent_inertia_kg_m2: 折算到电机输出轴的等效惯量 (kg·m²)。
        input_gain_scale: 输入增益校准比例（无量纲）。
        controller_bandwidth_rad_s: 控制律带宽 (rad/s)。
        observer_bandwidth_rad_s: LESO 带宽 (rad/s)。
        measurement_filter_cutoff_hz: 独立轻度预处理的测量截止频率 (Hz)。
        tracking_differentiator_bandwidth_rad_s: 可选线性 TD 带宽 (rad/s)。
        min_input_gain_n_per_n_m_s2: 调度输入增益下限 (N/(N·m·s²))。
        max_input_gain_n_per_n_m_s2: 调度输入增益上限 (N/(N·m·s²))。
        max_torque_rate_n_m_s: 输出力矩变化率上限 (N·m/s)。
    """

    equivalent_inertia_kg_m2: float
    input_gain_scale: float
    controller_bandwidth_rad_s: float
    observer_bandwidth_rad_s: float
    measurement_filter_cutoff_hz: float
    min_input_gain_n_per_n_m_s2: float
    max_input_gain_n_per_n_m_s2: float
    max_torque_rate_n_m_s: float
    tracking_differentiator_bandwidth_rad_s: float | None = None

    def __post_init__(self) -> None:
        """要求名义输入增益裁剪范围严格递增。"""
        if self.min_input_gain_n_per_n_m_s2 >= self.max_input_gain_n_per_n_m_s2:
            raise ValueError(
                "min_input_gain_n_per_n_m_s2 must be smaller than max_input_gain_n_per_n_m_s2"
            )


@dataclass(frozen=True, slots=True)
class TorqueAdrcStep:
    """二阶直接力矩 LADRC 的一次控制输出与内部诊断量。"""

    requested_torque_n_m: float
    raw_torque_n_m: float
    model_feedforward_torque_n_m: float
    residual_torque_n_m: float
    input_gain_n_per_n_m_s2: float
    estimated_force_n: float
    estimated_force_rate_n_s: float
    estimated_disturbance_n_s2: float
    reference_force_n: float
    reference_force_rate_n_s: float
    reference_force_acceleration_n_s2: float
    rate_limited: bool
    amplitude_limited: bool


class SecondOrderTorqueLADRC:
    """使用 current observer 和实际受限输入的二阶直接力矩 LADRC。"""

    def __init__(self, config: TorqueAdrcConfig) -> None:
        """保存参数并初始化观测器。"""
        self._config = config
        self.reset()

    def reset(
        self,
        *,
        measured_force_n: float = 0.0,
        applied_torque_n_m: float = 0.0,
        model_feedforward_torque_n_m: float = 0.0,
        input_gain_n_per_n_m_s2: float | None = None,
    ) -> None:
        """按当前力与实际力矩初始化，保证稳态切换时控制量连续。

        Args:
            measured_force_n: 切换时的滤波法向力，单位 N。
            applied_torque_n_m: 切换前实际施加的电机力矩，单位 N·m。
            model_feedforward_torque_n_m: 切换时机构模型给出的名义力矩，单位 N·m。
            input_gain_n_per_n_m_s2: 当前名义输入增益；省略时等待第一次控制步设置。
        """
        self._z1 = float(measured_force_n)
        self._z2 = 0.0
        self._input_gain = (
            None if input_gain_n_per_n_m_s2 is None else float(input_gain_n_per_n_m_s2)
        )
        self._applied_torque = float(applied_torque_n_m)
        self._model_feedforward_torque = float(model_feedforward_torque_n_m)
        residual_torque = self._applied_torque - self._model_feedforward_torque
        self._z3 = 0.0 if self._input_gain is None else -self._input_gain * residual_torque
        self._td_force = float(measured_force_n)
        self._td_force_rate = 0.0
        self._td_active = False
        self._td_last_target: float | None = None

    def update_reference(
        self,
        *,
        target_force_n: float,
        target_force_rate_n_s: float,
        target_force_acceleration_n_s2: float,
        dt: float,
    ) -> tuple[float, float, float]:
        """更新可选的临界阻尼线性 TD，并返回控制律使用的参考状态。"""
        bandwidth = self._config.tracking_differentiator_bandwidth_rad_s
        if bandwidth is None:
            return (
                float(target_force_n),
                float(target_force_rate_n_s),
                float(target_force_acceleration_n_s2),
            )
        if dt <= 0:
            raise ValueError("dt must be positive")
        raw_target = float(target_force_n)
        raw_rate = float(target_force_rate_n_s)
        raw_acceleration = float(target_force_acceleration_n_s2)
        has_analytic_motion = not math.isclose(raw_rate, 0.0, abs_tol=1e-12) or not math.isclose(
            raw_acceleration, 0.0, abs_tol=1e-12
        )
        if has_analytic_motion:
            # Linear 与 smoothstep 已由轨迹发生器提供解析导数；再次经过 TD 只会
            # 增加相位滞后，因此同步内部状态并原样使用解析参考。
            self._td_force = raw_target
            self._td_force_rate = raw_rate
            self._td_active = False
            self._td_last_target = raw_target
            return raw_target, raw_rate, raw_acceleration
        target_changed = self._td_last_target is None or not math.isclose(
            raw_target, self._td_last_target, abs_tol=1e-12
        )
        if target_changed and not math.isclose(raw_target, self._td_force, abs_tol=1e-12):
            self._td_active = True
        self._td_last_target = raw_target
        if not self._td_active:
            self._td_force = raw_target
            self._td_force_rate = 0.0
            return raw_target, 0.0, 0.0
        omega = float(bandwidth)
        error = self._td_force - raw_target
        coupling = self._td_force_rate + omega * error
        decay = math.exp(-omega * dt)
        self._td_force = raw_target + (error + coupling * dt) * decay
        self._td_force_rate = (self._td_force_rate - omega * coupling * dt) * decay
        acceleration = omega**2 * (raw_target - self._td_force) - 2.0 * omega * (
            self._td_force_rate
        )
        if math.isclose(self._td_force, raw_target, abs_tol=1e-6) and math.isclose(
            self._td_force_rate, 0.0, abs_tol=1e-5
        ):
            self._td_force = raw_target
            self._td_force_rate = 0.0
            self._td_active = False
            acceleration = 0.0
        return self._td_force, self._td_force_rate, acceleration

    def set_applied_torque(
        self,
        applied_torque_n_m: float,
        *,
        model_feedforward_torque_n_m: float,
    ) -> None:
        """记录实际总力矩及同周期名义模型前馈。"""
        self._applied_torque = float(applied_torque_n_m)
        self._model_feedforward_torque = float(model_feedforward_torque_n_m)

    def step(
        self,
        *,
        measured_force_n: float,
        target_force_n: float,
        target_force_rate_n_s: float,
        target_force_acceleration_n_s2: float,
        model_feedforward_torque_n_m: float,
        input_gain_n_per_n_m_s2: float,
        dt: float,
        min_torque_n_m: float,
        max_torque_n_m: float,
    ) -> TorqueAdrcStep:
        """更新离散 LESO，并生成经过力矩变化率和幅值限制的命令。"""
        if dt <= 0:
            raise ValueError("dt must be positive")
        if input_gain_n_per_n_m_s2 <= 0:
            raise ValueError("input_gain_n_per_n_m_s2 must be positive")
        if min_torque_n_m >= max_torque_n_m:
            raise ValueError("min_torque_n_m must be smaller than max_torque_n_m")

        input_gain = float(input_gain_n_per_n_m_s2)
        if self._input_gain is None:
            self._input_gain = input_gain
            residual_applied_torque = self._applied_torque - self._model_feedforward_torque
            self._z3 = -input_gain * residual_applied_torque
        elif not math.isclose(input_gain, self._input_gain, rel_tol=1e-12, abs_tol=0.0):
            # 调度 b0 时同步缩放扰动状态，使 -z3/b0 对应的补偿力矩连续。
            self._z3 *= input_gain / self._input_gain
            self._input_gain = input_gain

        # 精确离散积分链加 current observer：最新测量 y(k) 在本周期直接校正
        # 预测状态，上一周期输入使用实际总力矩扣除同周期名义模型前馈后的
        # 残差力矩；由此让 ESO 只补偿模型没有解释的部分。
        residual_applied_torque = self._applied_torque - self._model_feedforward_torque
        dt_sq = dt * dt
        predicted_z1 = (
            self._z1
            + dt * self._z2
            + 0.5 * dt_sq * (self._z3 + input_gain * residual_applied_torque)
        )
        predicted_z2 = self._z2 + dt * (self._z3 + input_gain * residual_applied_torque)
        predicted_z3 = self._z3
        observer_pole = math.exp(-float(self._config.observer_bandwidth_rad_s) * dt)
        one_minus_pole = 1.0 - observer_pole
        l1 = 1.0 - observer_pole**3
        l2 = 3.0 * one_minus_pole**2 * (1.0 + observer_pole) / (2.0 * dt)
        l3 = one_minus_pole**3 / dt_sq
        innovation = float(measured_force_n) - predicted_z1
        self._z1 = predicted_z1 + l1 * innovation
        self._z2 = predicted_z2 + l2 * innovation
        self._z3 = predicted_z3 + l3 * innovation

        bandwidth = float(self._config.controller_bandwidth_rad_s)
        virtual_force_acceleration = (
            float(target_force_acceleration_n_s2)
            + 2.0 * bandwidth * (float(target_force_rate_n_s) - self._z2)
            + bandwidth**2 * (float(target_force_n) - self._z1)
        )
        residual_torque = (virtual_force_acceleration - self._z3) / input_gain
        model_feedforward_torque = float(model_feedforward_torque_n_m)
        raw_torque = model_feedforward_torque + residual_torque
        max_delta = float(self._config.max_torque_rate_n_m_s) * dt
        rate_limited_torque = float(
            np.clip(
                raw_torque,
                self._applied_torque - max_delta,
                self._applied_torque + max_delta,
            )
        )
        requested_torque = float(np.clip(rate_limited_torque, min_torque_n_m, max_torque_n_m))
        return TorqueAdrcStep(
            requested_torque_n_m=requested_torque,
            raw_torque_n_m=float(raw_torque),
            model_feedforward_torque_n_m=model_feedforward_torque,
            residual_torque_n_m=float(residual_torque),
            input_gain_n_per_n_m_s2=input_gain,
            estimated_force_n=self._z1,
            estimated_force_rate_n_s=self._z2,
            estimated_disturbance_n_s2=self._z3,
            reference_force_n=float(target_force_n),
            reference_force_rate_n_s=float(target_force_rate_n_s),
            reference_force_acceleration_n_s2=float(target_force_acceleration_n_s2),
            rate_limited=not math.isclose(rate_limited_torque, raw_torque, abs_tol=1e-12),
            amplitude_limited=not math.isclose(
                requested_torque, rate_limited_torque, abs_tol=1e-12
            ),
        )

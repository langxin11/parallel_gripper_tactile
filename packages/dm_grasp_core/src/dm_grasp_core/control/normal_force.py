"""法向力外环跟踪：接触状态机与 PID、一阶 LADRC、直接力矩、二阶 LADRC 路径。

本模块是纯算法：电机侧的实测位置读取与 MIT 命令下发通过
:class:`MITTorqueInner` 协议注入，仿真与真机适配层各自实现该协议。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal, Protocol

import numpy as np
from simple_pid import PID

from .adrc import SecondOrderTorqueLADRC, TorqueAdrcConfig, TorqueAdrcStep
from .kinematics import CrankSliderKinematics
from .mit import MITControlCommand
from .stiffness import ContactStiffnessConfig, ContactStiffnessEstimator


ForceSemantics = Literal["average_side", "total"]


@dataclass(frozen=True, slots=True)
class AdrcConfig:
    """一阶线性自抗扰（LADRC）外环参数。

    被控假设为 ``df/dt = f + b0·u``：f 为滤波后的法向力 [N]，u 为闭合速度
    [m/s]，b0 为名义输入增益 [N/m]，量级约等于接触等效刚度。

    Attributes:
        b0_n_per_m: 名义输入增益 (N/m)。
        controller_bandwidth_rad_s: 控制律带宽 (rad/s)。
        observer_bandwidth_rad_s: 扩张状态观测器带宽 (rad/s)。
        max_closing_velocity_m_s: 闭合速度绝对值上限 (m/s)。
    """

    b0_n_per_m: float
    controller_bandwidth_rad_s: float
    observer_bandwidth_rad_s: float
    max_closing_velocity_m_s: float


@dataclass(frozen=True, slots=True)
class NormalForceConfig:
    """外环法向力跟踪参数（不含传感器噪声与导纳等调用方专属字段）。

    Attributes:
        target_n: 默认目标法向力 (N)。
        contact_threshold_n: 双侧接触确认的每侧力阈值 (N)。
        contact_confirm_steps: 接触确认所需连续周期数。
        release_threshold_n: 判定释放的每侧力阈值 (N)。
        release_confirm_steps: 释放确认所需连续周期数。
        kp: 外环 PID 比例增益。
        ki: 外环 PID 积分增益。
        kd: 外环 PID 微分增益。
        max_position_adjustment: 跟踪阶段位置修正绝对值上限 (rad)。
        filter_cutoff_hz: 公共法向力一阶低通截止频率 (Hz)。
        geometry: 曲柄滑块运动学；刚度前馈、限幅与两条 ADRC 路径必需。
        stiffness: 在线接触刚度估计配置；``None`` 表示不启用。
        torque_feedback_gain: 直接力矩路径增益；大于 0 时替代 PID 位置修正。
        adrc: 一阶 LADRC 外环配置；与 PID 及二阶路径互斥。
        torque_adrc: 二阶直接力矩 LADRC 配置；与前两条路径互斥。
    """

    target_n: float
    contact_threshold_n: float
    contact_confirm_steps: int
    release_threshold_n: float
    release_confirm_steps: int
    kp: float
    ki: float
    kd: float
    max_position_adjustment: float
    filter_cutoff_hz: float
    geometry: CrankSliderKinematics | None = None
    stiffness: ContactStiffnessConfig | None = None
    torque_feedback_gain: float = 0.0
    adrc: AdrcConfig | None = None
    torque_adrc: TorqueAdrcConfig | None = None

    def __post_init__(self) -> None:
        """要求释放阈值和可选刚度估计配置相互一致。"""
        if self.release_threshold_n > self.contact_threshold_n:
            raise ValueError("release_threshold_n must not exceed contact_threshold_n")
        if self.stiffness is not None and self.stiffness.enabled and self.geometry is None:
            raise ValueError("geometry is required when contact stiffness estimation is enabled")


class MITTorqueInner(Protocol):
    """法向力外环依赖的最小电机内环接口。

    实现者负责访问自身后端状态（MuJoCo ``MjData`` 或真机反馈），
    并把量化后的力矩写入执行器。
    """

    @property
    def torque_limit_n_m(self) -> float:
        """返回二阶力矩 LADRC 路径允许的对称输出力矩上限 (N·m)。"""
        ...

    def position(self) -> float:
        """返回当前受控关节位置 (rad)。"""
        ...

    def apply(
        self,
        *,
        target_position: float,
        target_velocity: float = 0.0,
        feedforward_torque: float | None = None,
        stiffness_override: float | None = None,
        damping_override: float | None = None,
    ) -> MITControlCommand:
        """下发一个 MIT 力矩命令并返回量化后的命令记录。"""
        ...


@dataclass(frozen=True, slots=True)
class ForceControlObservation:
    """力控器在一个控制周期内可用的仿真或硬件观测。"""

    time_s: float
    approach_position: float
    total_normal_force_n: float
    left_normal_force_n: float
    right_normal_force_n: float
    dt: float
    approach_velocity: float = 0.0


@dataclass(frozen=True, slots=True)
class ForceControlReference:
    """力控器在一个控制周期内需要跟踪的参考量。"""

    target_force_n: float
    approach_feedforward_force_n: float = 0.0
    target_force_rate_n_s: float = 0.0
    target_force_acceleration_n_s2: float = 0.0


@dataclass(frozen=True, slots=True)
class NormalForceControlCommand:
    """一个混合的接近/力控制命令及其观察到的力状态。"""

    state: str
    target_force_n: float
    measured_force_n: float
    filtered_force_n: float
    force_error_n: float
    position_adjustment: float
    mit: MITControlCommand
    pid_position_adjustment: float = 0.0
    stiffness_position_adjustment: float = 0.0
    force_feedforward_torque: float = 0.0
    estimated_contact_stiffness_n_per_m: float | None = None
    closure_jacobian_m_per_rad: float | None = None
    aperture_m: float | None = None
    stiffness_position_limit_rad: float | None = None
    stiffness_position_limited: bool = False
    torque_adrc_measurement_n: float | None = None
    torque_adrc_estimated_force_n: float | None = None
    torque_adrc_estimated_force_rate_n_s: float | None = None
    torque_adrc_estimated_disturbance_n_s2: float | None = None
    torque_adrc_reference_force_n: float | None = None
    torque_adrc_reference_force_rate_n_s: float | None = None
    torque_adrc_reference_force_acceleration_n_s2: float | None = None
    torque_adrc_raw_torque_n_m: float | None = None
    torque_adrc_limited_torque_n_m: float | None = None
    torque_adrc_residual_torque_n_m: float | None = None
    torque_adrc_input_gain_n_per_n_m_s2: float | None = None
    torque_adrc_rate_limited: bool = False
    torque_adrc_amplitude_limited: bool = False


@dataclass(frozen=True, slots=True)
class _ForceTrackingStep:
    """力跟踪阶段的一次控制输出及其诊断量。"""

    mit: MITControlCommand
    position_adjustment: float
    pid_position_adjustment: float
    stiffness_position_adjustment: float
    force_feedforward_torque: float
    estimated_contact_stiffness_n_per_m: float | None
    closure_jacobian_m_per_rad: float | None
    aperture_m: float | None
    stiffness_position_limit_rad: float | None = None
    stiffness_position_limited: bool = False
    torque_adrc: TorqueAdrcStep | None = None


class NormalForceController:
    """先以位置控制接近，再跟踪指定语义的双指 taxel 法向力。"""

    def __init__(
        self,
        config: NormalForceConfig,
        *,
        force_semantics: ForceSemantics = "average_side",
    ) -> None:
        """创建可切换的法向力外环与接触状态机。

        Args:
            config: 外环法向力跟踪配置；``config.adrc`` 非 ``None`` 时跟踪阶段
                以一阶 LADRC 外环替换 PID 位置修正；``config.torque_adrc`` 非
                ``None`` 时改用二阶直接力矩 LADRC。
            force_semantics: 目标力的语义（平均单侧力或总力）。

        Raises:
            ValueError: 多条可选力控路径同时启用，或 ADRC 缺少所需机构几何、
                在线刚度估计时抛出。
        """
        enabled_outer_loops = sum(
            (
                config.torque_feedback_gain > 0,
                config.adrc is not None,
                config.torque_adrc is not None,
            )
        )
        if enabled_outer_loops > 1:
            raise ValueError(
                "adrc, torque_adrc and torque_feedback_gain are mutually exclusive; "
                "enable at most one force-tracking outer loop"
            )
        if config.adrc is not None and config.geometry is None:
            raise ValueError("adrc requires crank-slider geometry for closure-jacobian conversion")
        if config.torque_adrc is not None and config.geometry is None:
            raise ValueError("torque_adrc requires crank-slider geometry for input-gain scheduling")
        if config.torque_adrc is not None and (
            config.stiffness is None or not config.stiffness.enabled
        ):
            raise ValueError("torque_adrc requires enabled contact stiffness estimation")
        self._config = config
        self._force_semantics = force_semantics
        self._kinematics = config.geometry
        self._stiffness_estimator = (
            ContactStiffnessEstimator(config.stiffness, self._kinematics)
            if config.stiffness is not None
            and config.stiffness.enabled
            and self._kinematics is not None
            else None
        )
        self._torque_adrc = (
            SecondOrderTorqueLADRC(config.torque_adrc) if config.torque_adrc is not None else None
        )
        adjustment = config.max_position_adjustment
        self._pid = PID(
            config.kp,
            config.ki,
            config.kd,
            setpoint=config.target_n,
            sample_time=None,
            output_limits=(-adjustment, adjustment),
            auto_mode=False,
        )
        # LADRC 外环状态（z1/z2 为扩张状态观测器，u 为上一周期闭合速度，
        # adjustment 为 u 逐周期积分出的持久位置修正）；adrc 为 None 时不被读取。
        self._adrc_z1 = 0.0
        self._adrc_z2 = 0.0
        self._adrc_u = 0.0
        self._adrc_adjustment = 0.0
        self._position_adjustment = 0.0
        self.reset()

    @property
    def state(self) -> str:
        """返回 ``approach`` 或 ``force_tracking``。"""
        return self._state

    def _reset_adrc(self, filtered_force_n: float | None = None) -> None:
        """把 LADRC 外环状态复位到跟踪起点。

        ``z1`` 初始化为当前滤波法向力（缺失时置 0，待接触确认时再对齐），
        ``z2``（总扰动估计）与积分位置修正清零，上一周期闭合速度一并复位。

        Args:
            filtered_force_n: 跟踪起点处的滤波法向力，单位 N。
        """
        self._adrc_z1 = 0.0 if filtered_force_n is None else float(filtered_force_n)
        self._adrc_z2 = 0.0
        self._adrc_u = 0.0
        self._adrc_adjustment = 0.0

    def _torque_adrc_input_gain(
        self,
        *,
        position_rad: float,
        stiffness_n_per_m: float,
    ) -> float:
        """由在线刚度、闭合雅可比和名义惯量计算受限的 ``b0``。"""
        if self._kinematics is None or self._config.torque_adrc is None:
            raise RuntimeError("torque_adrc input gain requested without torque_adrc configuration")
        adrc = self._config.torque_adrc
        jacobian = self._kinematics.closure_jacobian(position_rad)
        raw_gain = (
            max(0.0, float(stiffness_n_per_m))
            * jacobian
            * float(adrc.input_gain_scale)
            / float(adrc.equivalent_inertia_kg_m2)
        )
        return float(
            np.clip(
                raw_gain,
                adrc.min_input_gain_n_per_n_m_s2,
                adrc.max_input_gain_n_per_n_m_s2,
            )
        )

    def reset(self) -> None:
        """返回接近模式，并清除滤波器、计数器和 PID 历史。"""
        self._state = "approach"
        self._filtered_force: float | None = None
        self._torque_adrc_measurement: float | None = None
        self._contact_position = 0.0
        self._contact_steps = 0
        self._release_steps = 0
        self._position_adjustment = 0.0
        if self._stiffness_estimator is not None:
            self._stiffness_estimator.reset()
        self._pid.set_auto_mode(False)
        self._pid.reset()
        self._reset_adrc()
        if self._torque_adrc is not None:
            self._torque_adrc.reset()

    def step(
        self,
        inner: MITTorqueInner,
        *,
        observation: ForceControlObservation,
        reference: ForceControlReference,
    ) -> NormalForceControlCommand:
        """实现 force tracking 实验使用的统一控制器接口。"""
        return self.apply(
            inner,
            approach_position=observation.approach_position,
            total_normal_force_n=observation.total_normal_force_n,
            left_normal_force_n=observation.left_normal_force_n,
            right_normal_force_n=observation.right_normal_force_n,
            dt=observation.dt,
            approach_velocity=observation.approach_velocity,
            target_force_n=reference.target_force_n,
            approach_feedforward_force_n=reference.approach_feedforward_force_n,
            target_force_rate_n_s=reference.target_force_rate_n_s,
            target_force_acceleration_n_s2=reference.target_force_acceleration_n_s2,
        )

    def _force_feedforward_torque(
        self,
        *,
        position_rad: float,
        target_force_n: float,
        gain_override: float | None = None,
    ) -> tuple[float, float | None, float | None]:
        """将当前语义的目标力转换为与总闭合量功共轭的输出轴力矩。"""
        if self._kinematics is None:
            return 0.0, None, None
        aperture = self._kinematics.aperture(position_rad)
        closure_jacobian = self._kinematics.closure_jacobian(position_rad)
        gain = gain_override
        if gain is None:
            gain = (
                float(self._config.stiffness.torque_feedforward_gain)
                if self._config.stiffness is not None
                else 1.0
            )
        semantic_scale = 1.0 if self._force_semantics == "average_side" else 0.5
        torque = gain * max(0.0, float(target_force_n)) * semantic_scale * closure_jacobian
        return torque, closure_jacobian, aperture

    def _tracking_command(
        self,
        inner: MITTorqueInner,
        *,
        measured_force_n: float,
        torque_adrc_measurement_n: float | None,
        target_force_n: float,
        target_force_rate_n_s: float,
        target_force_acceleration_n_s2: float,
        dt: float,
    ) -> _ForceTrackingStep:
        """生成力跟踪阶段的组合位置修正和力矩前馈。

        当 ``config.torque_feedback_gain > 0`` 时改走直接力矩路径：PID 与刚度
        位置修正置零，力误差经 ``torque_feedback_gain`` 放大后与既有模型前馈
        （``torque_feedforward_gain`` 路径、以目标力计算）合并为 MIT 前馈力矩，
        并仅在本周期以 override 把 MIT kp、kd 覆盖为 0。刚度估计器两条路径都
        照常更新，保持 trace 中刚度曲线可比。

        当 ``config.adrc`` 非 ``None`` 时改走一阶 LADRC 路径（与直接力矩路径
        互斥，构造时已校验）：PID 与刚度位置修正均不参与，扩张状态观测器
        （LESO）估计滤波力 ``z1`` 与总扰动 ``z2``，控制律输出闭合速度 ``u``
        并逐周期积分成持久位置修正（裁剪到 ``±max_position_adjustment``）。
        每周期先用上一周期的 ``u`` 更新 LESO，再计算本周期 ``u``。MIT 内环
        kp/kd 不做 override，位置弹簧阻尼保留——这是与直接力矩路径的本质
        区别；模型力矩前馈照常经 ``torque_feedforward_gain`` 路径进入 MIT
        前馈力矩，刚度估计器照常更新。

        当 ``config.torque_adrc`` 非 ``None`` 时，二阶 current LESO 直接以实际
        受限电机力矩为输入，按在线刚度、机构雅可比和名义惯量调度 ``b0``。
        测量端使用独立的轻度一阶低通，不复用 PID、刚度估计和指标使用的
        公共低通，使 LESO 承担主要状态估计而不过度放大原始触觉噪声。
        控制律自身提供名义 PD 动态并输出力矩，因此仅在跟踪周期旁路 MIT
        ``kp/kd``；接近阶段的阻抗控制保持不变。

        当刚度感知位置限幅启用时，在线刚度不作为额外位置前馈叠加，
        而是把允许的法向力变化率换算为 PID 位置目标的周期增量边界。
        动态更新 PID 输出上下界同时限制其积分项，避免限幅期间 windup。

        Returns:
            ``_ForceTrackingStep``；三条路径的 ``force_feedforward_torque`` 均填
            送入 MIT 内环的前馈力矩总值（直接力矩路径下为力误差项与模型前馈之和）。
        """
        config = self._config
        current_position = inner.position()
        force_error = target_force_n - measured_force_n
        stiffness_adjustment = 0.0
        force_feedforward_torque = 0.0
        stiffness_estimate = None
        closure_jacobian = None
        aperture = None
        stiffness_position_limit = None
        stiffness_position_limited = False

        if self._kinematics is not None:
            aperture = self._kinematics.aperture(current_position)
            closure_jacobian = self._kinematics.closure_jacobian(current_position)

        if (
            self._stiffness_estimator is not None
            and self._kinematics is not None
            and closure_jacobian is not None
            and closure_jacobian > 1e-12
            and config.stiffness is not None
        ):
            stiffness_estimate = self._stiffness_estimator.update(
                position_rad=current_position,
                normal_force_n=measured_force_n,
            )
            joint_stiffness = stiffness_estimate * closure_jacobian
            stiffness_adjustment = (
                float(config.stiffness.position_feedforward_gain)
                * force_error
                / max(joint_stiffness, 1e-12)
            )
            force_feedforward_torque, closure_jacobian, aperture = self._force_feedforward_torque(
                position_rad=current_position,
                target_force_n=target_force_n,
            )

        if config.torque_adrc is not None:
            if self._torque_adrc is None or stiffness_estimate is None:
                raise RuntimeError("torque_adrc requires an active stiffness estimator")
            if torque_adrc_measurement_n is None:
                raise RuntimeError("torque_adrc requires its lightly filtered force measurement")
            reference_force, reference_rate, reference_acceleration = (
                self._torque_adrc.update_reference(
                    target_force_n=target_force_n,
                    target_force_rate_n_s=target_force_rate_n_s,
                    target_force_acceleration_n_s2=target_force_acceleration_n_s2,
                    dt=dt,
                )
            )
            # TD 启用时模型前馈也必须使用整形后的参考，否则原始阶跃会绕过 TD
            # 直接进入力矩命令，破坏参考过渡过程。
            force_feedforward_torque, closure_jacobian, aperture = self._force_feedforward_torque(
                position_rad=current_position,
                target_force_n=reference_force,
            )
            input_gain = self._torque_adrc_input_gain(
                position_rad=current_position,
                stiffness_n_per_m=stiffness_estimate,
            )
            torque_limit = inner.torque_limit_n_m
            torque_adrc = self._torque_adrc.step(
                measured_force_n=torque_adrc_measurement_n,
                target_force_n=reference_force,
                target_force_rate_n_s=reference_rate,
                target_force_acceleration_n_s2=reference_acceleration,
                model_feedforward_torque_n_m=force_feedforward_torque,
                input_gain_n_per_n_m_s2=input_gain,
                dt=dt,
                min_torque_n_m=-torque_limit,
                max_torque_n_m=torque_limit,
            )
            mit = inner.apply(
                target_position=self._contact_position,
                feedforward_torque=torque_adrc.requested_torque_n_m,
                stiffness_override=0.0,
                damping_override=0.0,
            )
            self._torque_adrc.set_applied_torque(
                mit.torque,
                model_feedforward_torque_n_m=force_feedforward_torque,
            )
            return _ForceTrackingStep(
                mit=mit,
                position_adjustment=0.0,
                pid_position_adjustment=0.0,
                stiffness_position_adjustment=0.0,
                force_feedforward_torque=force_feedforward_torque,
                estimated_contact_stiffness_n_per_m=stiffness_estimate,
                closure_jacobian_m_per_rad=closure_jacobian,
                aperture_m=aperture,
                torque_adrc=torque_adrc,
            )

        if config.torque_feedback_gain > 0:
            # 直接力矩式力控：力误差直接进入 MIT 前馈力矩，位置修正不进入命令，
            # 且仅在本跟踪周期把 MIT 位置环 kp/kd 覆盖为 0（接近阶段不受影响）。
            if self._kinematics is not None:
                assert closure_jacobian is not None
                semantic_scale = 1.0 if self._force_semantics == "average_side" else 0.5
                force_error_torque = (
                    float(config.torque_feedback_gain)
                    * force_error
                    * semantic_scale
                    * closure_jacobian
                )
                model_feedforward, closure_jacobian, aperture = self._force_feedforward_torque(
                    position_rad=current_position,
                    target_force_n=target_force_n,
                )
            else:
                force_error_torque = 0.0
                model_feedforward = 0.0
            force_feedforward_torque = force_error_torque + model_feedforward
            mit = inner.apply(
                target_position=self._contact_position,
                feedforward_torque=force_feedforward_torque,
                stiffness_override=0.0,
                damping_override=0.0,
            )
            return _ForceTrackingStep(
                mit=mit,
                position_adjustment=0.0,
                pid_position_adjustment=0.0,
                stiffness_position_adjustment=0.0,
                force_feedforward_torque=force_feedforward_torque,
                estimated_contact_stiffness_n_per_m=stiffness_estimate,
                closure_jacobian_m_per_rad=closure_jacobian,
                aperture_m=aperture,
            )

        if config.adrc is not None:
            # 一阶 LADRC 外环：被控假设 df/dt = f + b0·u（f 为滤波法向力，
            # u 为闭合速度，b0 为名义增益，量级约等于接触等效刚度）。
            adrc = config.adrc
            # 先用上一周期的闭合速度 u_prev 更新 LESO；innovation 为滤波力
            # 相对观测值的残差，β1 = 2ω_o、β2 = ω_o²。
            innovation = measured_force_n - self._adrc_z1
            beta_1 = 2.0 * adrc.observer_bandwidth_rad_s
            beta_2 = adrc.observer_bandwidth_rad_s**2
            self._adrc_z1 += dt * (
                self._adrc_z2 + adrc.b0_n_per_m * self._adrc_u + beta_1 * innovation
            )
            self._adrc_z2 += dt * (beta_2 * innovation)
            # 控制律：带宽比例误差项扣除扰动估计后除以 b0，得到本周期闭合速度并裁剪。
            raw_closing_velocity = (
                adrc.controller_bandwidth_rad_s * (target_force_n - self._adrc_z1) - self._adrc_z2
            ) / adrc.b0_n_per_m
            self._adrc_u = float(
                np.clip(
                    raw_closing_velocity,
                    -adrc.max_closing_velocity_m_s,
                    adrc.max_closing_velocity_m_s,
                )
            )
            # 闭合速度经闭合雅可比换算为电机侧角速度后，逐周期积分成持久位置
            # 修正（u 单位 m/s，closure_jacobian 单位 m/rad），并裁剪到
            # ±max_position_adjustment。
            motor_rate_rad_s = self._adrc_u / max(closure_jacobian, 1e-12)
            self._adrc_adjustment = float(
                np.clip(
                    self._adrc_adjustment + motor_rate_rad_s * dt,
                    -config.max_position_adjustment,
                    config.max_position_adjustment,
                )
            )
            force_feedforward_torque, closure_jacobian, aperture = self._force_feedforward_torque(
                position_rad=current_position,
                target_force_n=target_force_n,
            )
            mit = inner.apply(
                target_position=self._contact_position + self._adrc_adjustment,
                feedforward_torque=force_feedforward_torque,
            )
            return _ForceTrackingStep(
                mit=mit,
                position_adjustment=self._adrc_adjustment,
                pid_position_adjustment=0.0,
                stiffness_position_adjustment=0.0,
                force_feedforward_torque=force_feedforward_torque,
                estimated_contact_stiffness_n_per_m=stiffness_estimate,
                closure_jacobian_m_per_rad=closure_jacobian,
                aperture_m=aperture,
            )

        self._pid.setpoint = target_force_n
        stiffness = config.stiffness
        if (
            stiffness is not None
            and stiffness.position_limit_enabled
            and stiffness_estimate is not None
            and closure_jacobian is not None
        ):
            safe_joint_stiffness = (
                float(stiffness.position_limit_stiffness_safety_factor)
                * stiffness_estimate
                * closure_jacobian
            )
            maximum_force_step = min(
                abs(force_error),
                float(stiffness.position_limit_force_rate_n_s) * dt,
            )
            stiffness_position_limit = maximum_force_step / max(safe_joint_stiffness, 1e-12)
            lower = max(
                -float(config.max_position_adjustment),
                self._position_adjustment - stiffness_position_limit,
            )
            upper = min(
                float(config.max_position_adjustment),
                self._position_adjustment + stiffness_position_limit,
            )
            self._pid.output_limits = (lower, upper)
        else:
            self._pid.output_limits = (
                -float(config.max_position_adjustment),
                float(config.max_position_adjustment),
            )
        pid_adjustment = float(self._pid(measured_force_n, dt=dt))
        if stiffness_position_limit is not None:
            raw_pid_adjustment = float(sum(self._pid.components))
            stiffness_position_limited = not math.isclose(
                pid_adjustment,
                raw_pid_adjustment,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        adjustment = float(
            np.clip(
                stiffness_adjustment + pid_adjustment,
                -config.max_position_adjustment,
                config.max_position_adjustment,
            )
        )
        self._position_adjustment = adjustment
        mit = inner.apply(
            target_position=self._contact_position + adjustment,
            feedforward_torque=force_feedforward_torque,
        )
        return _ForceTrackingStep(
            mit=mit,
            position_adjustment=adjustment,
            pid_position_adjustment=pid_adjustment,
            stiffness_position_adjustment=stiffness_adjustment,
            force_feedforward_torque=force_feedforward_torque,
            estimated_contact_stiffness_n_per_m=stiffness_estimate,
            closure_jacobian_m_per_rad=closure_jacobian,
            aperture_m=aperture,
            stiffness_position_limit_rad=stiffness_position_limit,
            stiffness_position_limited=stiffness_position_limited,
        )

    def apply(
        self,
        inner: MITTorqueInner,
        *,
        approach_position: float,
        total_normal_force_n: float,
        left_normal_force_n: float,
        right_normal_force_n: float,
        dt: float,
        approach_velocity: float = 0.0,
        target_force_n: float | None = None,
        approach_feedforward_force_n: float = 0.0,
        target_force_rate_n_s: float = 0.0,
        target_force_acceleration_n_s2: float = 0.0,
    ) -> NormalForceControlCommand:
        """推进接触检测/力跟踪，并经 ``inner`` 下发一个电机力矩。

        接触要求两个指尖都保持在配置阈值之上。一旦确认，simple-pid 会
        调整检测到的接触位置，MIT 内环再将该位置目标转换为电机力矩。
        """
        if dt <= 0:
            raise ValueError("dt must be positive")
        config = self._config
        total_force = max(0.0, float(left_normal_force_n)) + max(0.0, float(right_normal_force_n))
        measured_force = (
            0.5 * total_force if self._force_semantics == "average_side" else total_force
        )
        stiffness_adjustment = 0.0
        pid_adjustment = 0.0
        force_feedforward_torque = 0.0
        stiffness_estimate = None
        closure_jacobian = None
        aperture = None
        stiffness_position_limit = None
        stiffness_position_limited = False
        torque_adrc_step = None
        if self._filtered_force is None:
            self._filtered_force = measured_force
        else:
            alpha = 1.0 - np.exp(-2.0 * np.pi * config.filter_cutoff_hz * dt)
            self._filtered_force += float(alpha) * (measured_force - self._filtered_force)
        if config.torque_adrc is not None:
            # ADRC 只做独立的轻度预处理；公共滤波器继续服务于 PID、刚度估计
            # 和评价指标，不能把它的额外相位滞后带入 LESO。
            if self._torque_adrc_measurement is None:
                self._torque_adrc_measurement = measured_force
            else:
                adrc_alpha = 1.0 - np.exp(
                    -2.0 * np.pi * config.torque_adrc.measurement_filter_cutoff_hz * dt
                )
                self._torque_adrc_measurement += float(adrc_alpha) * (
                    measured_force - self._torque_adrc_measurement
                )
        active_target_force = (
            float(config.target_n) if target_force_n is None else max(0.0, float(target_force_n))
        )

        if self._state == "approach":
            both_contacting = min(left_normal_force_n, right_normal_force_n)
            self._contact_steps = (
                self._contact_steps + 1 if both_contacting >= config.contact_threshold_n else 0
            )
            force_feedforward_torque, closure_jacobian, aperture = self._force_feedforward_torque(
                position_rad=inner.position(),
                target_force_n=approach_feedforward_force_n,
                # 控制器消融只作用于接触后的跟踪阶段；所有方案共享相同接近轨迹。
                gain_override=1.0,
            )
            mit = inner.apply(
                target_position=approach_position,
                target_velocity=approach_velocity,
                feedforward_torque=force_feedforward_torque,
            )
            adjustment = 0.0
            if self._contact_steps >= config.contact_confirm_steps:
                self._state = "force_tracking"
                self._contact_position = mit.position
                if self._stiffness_estimator is not None:
                    self._stiffness_estimator.reset(
                        position_rad=mit.position,
                        normal_force_n=self._filtered_force,
                    )
                # LADRC 外环与刚度估计器同一处复位：z1 对齐当前滤波力，
                # z2 与积分位置修正清零。
                self._reset_adrc(self._filtered_force)
                if self._torque_adrc is not None:
                    if self._stiffness_estimator is None:
                        raise RuntimeError("torque_adrc requires an active stiffness estimator")
                    input_gain = self._torque_adrc_input_gain(
                        position_rad=mit.position,
                        stiffness_n_per_m=self._stiffness_estimator.estimate_n_per_m,
                    )
                    # 以接近阶段最后一个实际力矩初始化 z3；当 r=y 时，首次直接
                    # 力矩命令会延续该力矩，实现阻抗到 ADRC 的无扰切换。
                    self._torque_adrc.reset(
                        measured_force_n=self._torque_adrc_measurement,
                        applied_torque_n_m=mit.torque,
                        model_feedforward_torque_n_m=force_feedforward_torque,
                        input_gain_n_per_n_m_s2=input_gain,
                    )
                self._pid.reset()
                self._pid.set_auto_mode(True, last_output=0.0)
                tracking = self._tracking_command(
                    inner,
                    measured_force_n=self._filtered_force,
                    torque_adrc_measurement_n=self._torque_adrc_measurement,
                    target_force_n=active_target_force,
                    target_force_rate_n_s=target_force_rate_n_s,
                    target_force_acceleration_n_s2=target_force_acceleration_n_s2,
                    dt=dt,
                )
                mit = tracking.mit
                adjustment = tracking.position_adjustment
                pid_adjustment = tracking.pid_position_adjustment
                stiffness_adjustment = tracking.stiffness_position_adjustment
                force_feedforward_torque = tracking.force_feedforward_torque
                stiffness_estimate = tracking.estimated_contact_stiffness_n_per_m
                closure_jacobian = tracking.closure_jacobian_m_per_rad
                aperture = tracking.aperture_m
                stiffness_position_limit = tracking.stiffness_position_limit_rad
                stiffness_position_limited = tracking.stiffness_position_limited
                torque_adrc_step = tracking.torque_adrc
        else:
            both_released = max(left_normal_force_n, right_normal_force_n)
            self._release_steps = (
                self._release_steps + 1 if both_released <= config.release_threshold_n else 0
            )
            if self._release_steps >= config.release_confirm_steps:
                self.reset()
                mit = inner.apply(
                    target_position=approach_position,
                    target_velocity=approach_velocity,
                )
                adjustment = 0.0
            else:
                tracking = self._tracking_command(
                    inner,
                    measured_force_n=self._filtered_force,
                    torque_adrc_measurement_n=self._torque_adrc_measurement,
                    target_force_n=active_target_force,
                    target_force_rate_n_s=target_force_rate_n_s,
                    target_force_acceleration_n_s2=target_force_acceleration_n_s2,
                    dt=dt,
                )
                mit = tracking.mit
                adjustment = tracking.position_adjustment
                pid_adjustment = tracking.pid_position_adjustment
                stiffness_adjustment = tracking.stiffness_position_adjustment
                force_feedforward_torque = tracking.force_feedforward_torque
                stiffness_estimate = tracking.estimated_contact_stiffness_n_per_m
                closure_jacobian = tracking.closure_jacobian_m_per_rad
                aperture = tracking.aperture_m
                stiffness_position_limit = tracking.stiffness_position_limit_rad
                stiffness_position_limited = tracking.stiffness_position_limited
                torque_adrc_step = tracking.torque_adrc

        reported_filtered_force = (
            measured_force if self._filtered_force is None else self._filtered_force
        )
        return NormalForceControlCommand(
            state=self._state,
            target_force_n=active_target_force,
            measured_force_n=measured_force,
            filtered_force_n=reported_filtered_force,
            force_error_n=active_target_force - reported_filtered_force,
            position_adjustment=adjustment,
            mit=mit,
            pid_position_adjustment=pid_adjustment,
            stiffness_position_adjustment=stiffness_adjustment,
            force_feedforward_torque=force_feedforward_torque,
            estimated_contact_stiffness_n_per_m=stiffness_estimate,
            closure_jacobian_m_per_rad=closure_jacobian,
            aperture_m=aperture,
            stiffness_position_limit_rad=stiffness_position_limit,
            stiffness_position_limited=stiffness_position_limited,
            torque_adrc_measurement_n=self._torque_adrc_measurement,
            torque_adrc_estimated_force_n=(
                None if torque_adrc_step is None else torque_adrc_step.estimated_force_n
            ),
            torque_adrc_estimated_force_rate_n_s=(
                None if torque_adrc_step is None else torque_adrc_step.estimated_force_rate_n_s
            ),
            torque_adrc_estimated_disturbance_n_s2=(
                None if torque_adrc_step is None else torque_adrc_step.estimated_disturbance_n_s2
            ),
            torque_adrc_reference_force_n=(
                None if torque_adrc_step is None else torque_adrc_step.reference_force_n
            ),
            torque_adrc_reference_force_rate_n_s=(
                None if torque_adrc_step is None else torque_adrc_step.reference_force_rate_n_s
            ),
            torque_adrc_reference_force_acceleration_n_s2=(
                None
                if torque_adrc_step is None
                else torque_adrc_step.reference_force_acceleration_n_s2
            ),
            torque_adrc_raw_torque_n_m=(
                None if torque_adrc_step is None else torque_adrc_step.raw_torque_n_m
            ),
            torque_adrc_limited_torque_n_m=(
                None if torque_adrc_step is None else torque_adrc_step.requested_torque_n_m
            ),
            torque_adrc_residual_torque_n_m=(
                None if torque_adrc_step is None else torque_adrc_step.residual_torque_n_m
            ),
            torque_adrc_input_gain_n_per_n_m_s2=(
                None if torque_adrc_step is None else torque_adrc_step.input_gain_n_per_n_m_s2
            ),
            torque_adrc_rate_limited=(
                False if torque_adrc_step is None else torque_adrc_step.rate_limited
            ),
            torque_adrc_amplitude_limited=(
                False if torque_adrc_step is None else torque_adrc_step.amplitude_limited
            ),
        )

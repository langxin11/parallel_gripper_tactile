"""用于 MuJoCo 电机执行器的 MIT 风格力矩控制。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Literal, Protocol

import numpy as np
from simple_pid import PID

from .profiles import (
    ContactStiffnessControl,
    CrankSliderGeometry,
    GripperProfile,
    MITControl,
    NormalForceControl,
)


DAMIAO_POSITION_BITS = 16
DAMIAO_VELOCITY_BITS = 12
DAMIAO_TORQUE_BITS = 12
DAMIAO_GAIN_BITS = 12
DAMIAO_STIFFNESS_RANGE = (0.0, 500.0)
DAMIAO_DAMPING_RANGE = (0.0, 5.0)


def _roundtrip_unsigned(value: float, lower: float, upper: float, bits: int) -> float:
    """按达妙 MIT 无符号整数编码后再解码回物理量。"""
    if lower >= upper:
        raise ValueError("encoding lower bound must be smaller than upper bound")
    levels = (1 << bits) - 1
    clipped = float(np.clip(value, lower, upper))
    encoded = int(round((clipped - lower) / (upper - lower) * levels))
    return lower + encoded / levels * (upper - lower)


@dataclass(frozen=True, slots=True)
class CrankSliderKinematics:
    """曲柄滑块夹爪的总开度与闭合行程雅可比。"""

    theta0_rad: float
    crank_radius_m: float
    link_length_m: float
    offset_m: float

    @classmethod
    def from_config(cls, config: CrankSliderGeometry) -> "CrankSliderKinematics":
        """从 profile 几何配置创建运动学模型。"""
        return cls(
            theta0_rad=float(config.theta0_rad),
            crank_radius_m=float(config.crank_radius_m),
            link_length_m=float(config.link_length_m),
            offset_m=float(config.offset_m),
        )

    def aperture(self, position_rad: float) -> float:
        """返回两指总开度 ``omega(q)``，单位 m。"""
        alpha = position_rad + self.theta0_rad
        radial_offset = self.crank_radius_m * math.sin(alpha) - self.offset_m
        radicand = self.link_length_m**2 - radial_offset**2
        if radicand <= 0:
            raise ValueError("crank-slider geometry is outside its valid aperture domain")
        return 2.0 * (self.crank_radius_m * math.cos(alpha) + math.sqrt(radicand))

    def closure(self, position_rad: float) -> float:
        """返回从 ``q=0`` 起算的总闭合行程，单位 m。"""
        return self.aperture(0.0) - self.aperture(position_rad)

    def closure_jacobian(self, position_rad: float) -> float:
        """返回 ``d(omega(0)-omega(q))/dq``，单位 m/rad。"""
        alpha = position_rad + self.theta0_rad
        radial_offset = self.crank_radius_m * math.sin(alpha) - self.offset_m
        radicand = self.link_length_m**2 - radial_offset**2
        if radicand <= 0:
            raise ValueError("crank-slider geometry is outside its valid Jacobian domain")
        return (
            2.0
            * self.crank_radius_m
            * (math.sin(alpha) + radial_offset * math.cos(alpha) / math.sqrt(radicand))
        )


class ContactStiffnessEstimator:
    """用接触后的 ``dF/dc`` 样本在线估计等效接触刚度。"""

    def __init__(
        self,
        config: ContactStiffnessControl,
        kinematics: CrankSliderKinematics,
    ) -> None:
        """保存估计参数和几何模型，并初始化估计状态。"""
        self._config = config
        self._kinematics = kinematics
        self._samples: deque[tuple[float, float]] = deque(maxlen=config.window_size)
        self.reset()

    @property
    def estimate_n_per_m(self) -> float:
        """返回当前滤波后的等效接触刚度估计。"""
        return self._estimate_n_per_m

    def reset(
        self,
        *,
        position_rad: float | None = None,
        normal_force_n: float | None = None,
    ) -> None:
        """重置估计，并可选记录新的接触参考点。"""
        self._estimate_n_per_m = float(self._config.initial_n_per_m)
        self._samples.clear()
        if position_rad is None or normal_force_n is None:
            self._last_closure_m = None
            self._last_force_n = None
            return
        closure_m = self._kinematics.closure(float(position_rad))
        force_n = max(0.0, float(normal_force_n))
        self._last_closure_m = closure_m
        self._last_force_n = force_n
        if (
            self._config.method != "secant_ewma"
            and math.isfinite(closure_m)
            and math.isfinite(force_n)
        ):
            self._samples.append((closure_m, force_n))

    def update(self, *, position_rad: float, normal_force_n: float) -> float:
        """用新的接触样本更新刚度估计，并返回当前估计值。"""
        if self._config.method != "secant_ewma":
            return self._update_window(position_rad=position_rad, normal_force_n=normal_force_n)

        return self._update_secant(position_rad=position_rad, normal_force_n=normal_force_n)

    def _update_secant(self, *, position_rad: float, normal_force_n: float) -> float:
        """用相邻有效样本的割线更新刚度，保留历史算法行为。"""
        closure_m = self._kinematics.closure(float(position_rad))
        force_n = max(0.0, float(normal_force_n))
        if self._last_closure_m is None or self._last_force_n is None:
            self._last_closure_m = closure_m
            self._last_force_n = force_n
            return self._estimate_n_per_m

        delta_closure = closure_m - self._last_closure_m
        delta_force = force_n - self._last_force_n
        if (
            abs(delta_closure) < self._config.min_delta_closure_m
            or abs(delta_force) < self._config.min_delta_force_n
        ):
            return self._estimate_n_per_m

        if delta_closure * delta_force > 0:
            sample = abs(delta_force / delta_closure)
            sample = float(np.clip(sample, self._config.min_n_per_m, self._config.max_n_per_m))
            alpha = float(self._config.filter_alpha)
            self._estimate_n_per_m += alpha * (sample - self._estimate_n_per_m)

        self._last_closure_m = closure_m
        self._last_force_n = force_n
        return self._estimate_n_per_m

    def _update_window(self, *, position_rad: float, normal_force_n: float) -> float:
        """用最近窗口的局部一次或二次拟合更新刚度。"""
        closure_m = self._kinematics.closure(float(position_rad))
        raw_force_n = float(normal_force_n)
        if not math.isfinite(closure_m) or not math.isfinite(raw_force_n):
            return self._estimate_n_per_m
        force_n = max(0.0, raw_force_n)

        self._samples.append((closure_m, force_n))
        if len(self._samples) < self._config.min_samples:
            return self._estimate_n_per_m

        closures = np.asarray([sample[0] for sample in self._samples], dtype=float)
        forces = np.asarray([sample[1] for sample in self._samples], dtype=float)
        closure_span = float(np.ptp(closures))
        force_span = float(np.ptp(forces))
        if (
            not math.isfinite(closure_span)
            or not math.isfinite(force_span)
            or closure_span < self._config.min_delta_closure_m
            or force_span < self._config.min_delta_force_n
        ):
            return self._estimate_n_per_m

        degree = 1 if self._config.method == "window_linear" else 2
        scale = max(closure_span, float(self._config.min_delta_closure_m))
        coordinate = (closures - closures[-1]) / scale
        design = np.column_stack([coordinate**power for power in range(degree + 1)])
        try:
            coefficients, _, rank, _ = np.linalg.lstsq(design, forces, rcond=None)
        except (np.linalg.LinAlgError, ValueError):
            return self._estimate_n_per_m
        if rank < degree + 1:
            return self._estimate_n_per_m

        slope = float(coefficients[1] / scale)
        if not math.isfinite(slope) or slope <= 0.0:
            return self._estimate_n_per_m

        sample = float(np.clip(slope, self._config.min_n_per_m, self._config.max_n_per_m))
        alpha = float(self._config.filter_alpha)
        self._estimate_n_per_m += alpha * (sample - self._estimate_n_per_m)
        return self._estimate_n_per_m


@dataclass(frozen=True, slots=True)
class MITControlCommand:
    """一个经过饱和处理的 MIT 力矩命令，以及用于计算它的状态。"""

    target_position: float
    target_velocity: float
    position: float
    velocity: float
    feedforward_torque: float
    torque: float


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


ForceSemantics = Literal["average_side", "total"]


class MITTorqueController:
    """将 ``kp*(p_des-p) + kd*(v_des-v) + t_ff`` 施加到电机执行器。"""

    def __init__(
        self,
        model,
        *,
        actuator_name: str,
        joint_name: str,
        config: MITControl,
    ) -> None:
        """解析模型索引，并拒绝与 MJCF 不一致的限值。"""
        self._actuator_id = model.actuator(actuator_name).id
        joint_id = model.joint(joint_name).id
        self._qpos_address = int(model.jnt_qposadr[joint_id])
        self._dof_address = int(model.jnt_dofadr[joint_id])
        self._config = config
        actuator_force_limit = float(np.min(np.abs(model.actuator_forcerange[self._actuator_id])))
        actuator_control_limit = float(np.min(np.abs(model.actuator_ctrlrange[self._actuator_id])))
        if config.t_max > actuator_force_limit or config.t_max > actuator_control_limit:
            raise ValueError(
                f"MIT t_max={config.t_max:g} exceeds actuator limits "
                f"ctrl={actuator_control_limit:g}, force={actuator_force_limit:g}"
            )

    @classmethod
    def from_profile(
        cls,
        model,
        profile: GripperProfile,
        *,
        name_prefix: str = "",
    ) -> "MITTorqueController":
        """基于 profile 和一个可选的附加模型前缀构建控制器。"""
        if profile.control_mode != "mit_torque" or profile.mit is None:
            raise ValueError("profile does not define MIT torque control")
        return cls(
            model,
            actuator_name=f"{name_prefix}{profile.actuator}",
            joint_name=f"{name_prefix}{profile.actuator}",
            config=profile.mit,
        )

    @property
    def actuator_id(self) -> int:
        """返回此实例所控制的编译后执行器索引。"""
        return self._actuator_id

    def position(self, data) -> float:
        """返回当前受控关节位置。"""
        return float(data.qpos[self._qpos_address])

    def velocity(self, data) -> float:
        """返回当前受控关节速度。"""
        return float(data.qvel[self._dof_address])

    def apply(
        self,
        data,
        *,
        target_position: float,
        target_velocity: float = 0.0,
        feedforward_torque: float | None = None,
        stiffness_override: float | None = None,
        damping_override: float | None = None,
    ) -> MITControlCommand:
        """计算、按达妙协议量化、饱和、写入并返回一个 MIT 力矩命令。

        Args:
            data: 当前 ``MjData``，最终力矩会写入其 ``ctrl`` 数组。
            target_position: 目标位置，会先按达妙位置编码量化。
            target_velocity: 目标速度，默认 0。
            feedforward_torque: 本周期前馈力矩；``None`` 时沿用 ``config.t_ff``。
            stiffness_override: 仅本周期生效的 MIT kp 覆盖；``None`` 时沿用
                ``config.kp``。直接力矩式力控在跟踪阶段传 0.0 以旁路位置弹簧。
            damping_override: 仅本周期生效的 MIT kd 覆盖；``None`` 时沿用
                ``config.kd``。语义与 ``stiffness_override`` 一致。

        Returns:
            量化与饱和后的 ``MITControlCommand``。
        """
        config = self._config
        desired_position = _roundtrip_unsigned(
            target_position,
            config.p_min,
            config.p_max,
            DAMIAO_POSITION_BITS,
        )
        desired_velocity = _roundtrip_unsigned(
            target_velocity,
            -config.v_max,
            config.v_max,
            DAMIAO_VELOCITY_BITS,
        )
        feedforward = _roundtrip_unsigned(
            config.t_ff if feedforward_torque is None else feedforward_torque,
            -config.t_max,
            config.t_max,
            DAMIAO_TORQUE_BITS,
        )
        stiffness = _roundtrip_unsigned(
            config.kp if stiffness_override is None else stiffness_override,
            DAMIAO_STIFFNESS_RANGE[0],
            DAMIAO_STIFFNESS_RANGE[1],
            DAMIAO_GAIN_BITS,
        )
        damping = _roundtrip_unsigned(
            config.kd if damping_override is None else damping_override,
            DAMIAO_DAMPING_RANGE[0],
            DAMIAO_DAMPING_RANGE[1],
            DAMIAO_GAIN_BITS,
        )
        position = float(data.qpos[self._qpos_address])
        velocity = float(data.qvel[self._dof_address])
        torque = stiffness * (desired_position - position)
        torque += damping * (desired_velocity - velocity) + feedforward
        torque = float(np.clip(torque, -config.t_max, config.t_max))
        data.ctrl[self._actuator_id] = torque
        return MITControlCommand(
            target_position=desired_position,
            target_velocity=desired_velocity,
            position=position,
            velocity=velocity,
            feedforward_torque=feedforward,
            torque=torque,
        )


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


class ForceTrackingController(Protocol):
    """force tracking 实验调用的最小控制器接口。"""

    @property
    def actuator_id(self) -> int:
        """返回由控制器写入命令的执行器索引。"""
        ...

    def reset(self) -> None:
        """重置控制器内部状态。"""
        ...

    def step(
        self,
        data,
        *,
        observation: ForceControlObservation,
        reference: ForceControlReference,
    ) -> NormalForceControlCommand:
        """使用当前观测和参考量推进一个控制周期。"""
        ...


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


class NormalForceController:
    """先以位置控制接近，再跟踪指定语义的双指 taxel 法向力。"""

    def __init__(
        self,
        inner: MITTorqueController,
        config: NormalForceControl,
        *,
        force_semantics: ForceSemantics = "average_side",
    ) -> None:
        """创建 simple-pid 外环与接触状态机。

        Args:
            inner: MIT 力矩内环控制器。
            config: 外环法向力跟踪配置；``config.adrc`` 非 ``None`` 时跟踪阶段
                以一阶 LADRC 外环替换 PID 位置修正。
            force_semantics: 目标力的语义（平均单侧力或总力）。

        Raises:
            ValueError: ``config.adrc`` 与 ``config.torque_feedback_gain > 0``
                同时启用时抛出；两条路径互斥，本文假设不同时配置。``config.adrc``
                非 ``None`` 但未配置曲柄滑块几何时抛出（速度到位置修正的换算
                依赖闭合雅可比）。
        """
        if config.adrc is not None and config.torque_feedback_gain > 0:
            raise ValueError(
                "adrc and torque_feedback_gain are mutually exclusive; "
                "enable at most one force-tracking outer loop"
            )
        if config.adrc is not None and config.geometry is None:
            raise ValueError("adrc requires crank-slider geometry for closure-jacobian conversion")
        self._inner = inner
        self._config = config
        self._force_semantics = force_semantics
        self._kinematics = (
            CrankSliderKinematics.from_config(config.geometry)
            if config.geometry is not None
            else None
        )
        self._stiffness_estimator = (
            ContactStiffnessEstimator(config.stiffness, self._kinematics)
            if config.stiffness is not None
            and config.stiffness.enabled
            and self._kinematics is not None
            else None
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
        self.reset()

    @classmethod
    def from_profile(
        cls,
        model,
        profile: GripperProfile,
        *,
        name_prefix: str = "",
        force_semantics: ForceSemantics = "average_side",
    ) -> "NormalForceController":
        """基于一个 profile 构建外环力环路及其 MIT 内环。"""
        if profile.normal_force is None:
            raise ValueError("profile does not define normal-force control")
        return cls(
            MITTorqueController.from_profile(model, profile, name_prefix=name_prefix),
            profile.normal_force,
            force_semantics=force_semantics,
        )

    @property
    def actuator_id(self) -> int:
        """返回由内环 MIT 环路控制的电机执行器。"""
        return self._inner.actuator_id

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

    def reset(self) -> None:
        """返回接近模式，并清除滤波器、计数器和 PID 历史。"""
        self._state = "approach"
        self._filtered_force: float | None = None
        self._contact_position = 0.0
        self._contact_steps = 0
        self._release_steps = 0
        if self._stiffness_estimator is not None:
            self._stiffness_estimator.reset()
        self._pid.set_auto_mode(False)
        self._pid.reset()
        self._reset_adrc()

    def step(
        self,
        data,
        *,
        observation: ForceControlObservation,
        reference: ForceControlReference,
    ) -> NormalForceControlCommand:
        """实现 force tracking 实验使用的统一控制器接口。"""
        return self.apply(
            data,
            approach_position=observation.approach_position,
            total_normal_force_n=observation.total_normal_force_n,
            left_normal_force_n=observation.left_normal_force_n,
            right_normal_force_n=observation.right_normal_force_n,
            dt=observation.dt,
            approach_velocity=observation.approach_velocity,
            target_force_n=reference.target_force_n,
            approach_feedforward_force_n=reference.approach_feedforward_force_n,
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
        data,
        *,
        measured_force_n: float,
        target_force_n: float,
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

        Returns:
            ``_ForceTrackingStep``；三条路径的 ``force_feedforward_torque`` 均填
            送入 MIT 内环的前馈力矩总值（直接力矩路径下为力误差项与模型前馈之和）。
        """
        config = self._config
        current_position = self._inner.position(data)
        force_error = target_force_n - measured_force_n
        stiffness_adjustment = 0.0
        force_feedforward_torque = 0.0
        stiffness_estimate = None
        closure_jacobian = None
        aperture = None

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
            mit = self._inner.apply(
                data,
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
            mit = self._inner.apply(
                data,
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
        pid_adjustment = float(self._pid(measured_force_n, dt=dt))
        adjustment = float(
            np.clip(
                stiffness_adjustment + pid_adjustment,
                -config.max_position_adjustment,
                config.max_position_adjustment,
            )
        )
        mit = self._inner.apply(
            data,
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
        )

    def apply(
        self,
        data,
        *,
        approach_position: float,
        total_normal_force_n: float,
        left_normal_force_n: float,
        right_normal_force_n: float,
        dt: float,
        approach_velocity: float = 0.0,
        target_force_n: float | None = None,
        approach_feedforward_force_n: float = 0.0,
    ) -> NormalForceControlCommand:
        """推进接触检测/力跟踪，并写入一个电机力矩。

        接触要求两个指尖都保持在配置阈值之上。一旦确认，simple-pid 会
        调整检测到的接触位置，现有 MIT 控制器再将该位置目标转换为电机力矩。
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
        if self._filtered_force is None:
            self._filtered_force = measured_force
        else:
            alpha = 1.0 - np.exp(-2.0 * np.pi * config.filter_cutoff_hz * dt)
            self._filtered_force += float(alpha) * (measured_force - self._filtered_force)
        active_target_force = (
            float(config.target_n) if target_force_n is None else max(0.0, float(target_force_n))
        )

        if self._state == "approach":
            both_contacting = min(left_normal_force_n, right_normal_force_n)
            self._contact_steps = (
                self._contact_steps + 1 if both_contacting >= config.contact_threshold_n else 0
            )
            force_feedforward_torque, closure_jacobian, aperture = self._force_feedforward_torque(
                position_rad=self._inner.position(data),
                target_force_n=approach_feedforward_force_n,
                # 控制器消融只作用于接触后的跟踪阶段；所有方案共享相同接近轨迹。
                gain_override=1.0,
            )
            mit = self._inner.apply(
                data,
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
                self._pid.reset()
                self._pid.set_auto_mode(True, last_output=0.0)
                tracking = self._tracking_command(
                    data,
                    measured_force_n=self._filtered_force,
                    target_force_n=active_target_force,
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
        else:
            both_released = max(left_normal_force_n, right_normal_force_n)
            self._release_steps = (
                self._release_steps + 1 if both_released <= config.release_threshold_n else 0
            )
            if self._release_steps >= config.release_confirm_steps:
                self.reset()
                mit = self._inner.apply(
                    data,
                    target_position=approach_position,
                    target_velocity=approach_velocity,
                )
                adjustment = 0.0
            else:
                tracking = self._tracking_command(
                    data,
                    measured_force_n=self._filtered_force,
                    target_force_n=active_target_force,
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
        )

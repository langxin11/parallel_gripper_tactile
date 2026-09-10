"""DM 力控算法的仿真适配层。

算法本体位于核心包 ``dm_grasp_core.control``（纯 Python，不依赖
MuJoCo 与模型路径）；本模块负责三件事：把 profile 配置转换为核心
dataclass、把 MIT 力矩命令写入 MuJoCo 执行器、并向既有调用方
再导出全部公共名称。导入路径、类签名与数值行为与迁移前一致。
"""

from __future__ import annotations

from typing import Protocol

import numpy as np
from dm_grasp_core import (
    AdrcConfig as _AdrcConfig,
    ContactStiffnessConfig,
    CrankSliderKinematics as _CoreCrankSliderKinematics,
    ForceControlObservation as ForceControlObservation,
    ForceControlReference as ForceControlReference,
    ForceSemantics as ForceSemantics,
    MITControlCommand as MITControlCommand,
    MITControlConfig,
    MITTorqueModel,
    NormalForceConfig as _NormalForceConfig,
    NormalForceControlCommand as NormalForceControlCommand,
    SecondOrderTorqueLADRC as _CoreSecondOrderTorqueLADRC,
    TorqueAdrcConfig as _TorqueAdrcConfig,
    TorqueAdrcStep as TorqueAdrcStep,
)
from dm_grasp_core.control.mit import _roundtrip_unsigned
from dm_grasp_core.control.normal_force import (
    NormalForceController as _CoreNormalForceController,
)
from dm_grasp_core.control.stiffness import (
    ContactStiffnessEstimator as _CoreContactStiffnessEstimator,
)

from .config.profiles import (
    ContactStiffnessControl,
    CrankSliderGeometry,
    GripperProfile,
    MITControl,
    NormalForceControl,
    TorqueAdrcControl,
)

__all__ = [
    "DAMIAO_DAMPING_RANGE",
    "DAMIAO_GAIN_BITS",
    "DAMIAO_POSITION_BITS",
    "DAMIAO_STIFFNESS_RANGE",
    "DAMIAO_TORQUE_BITS",
    "DAMIAO_VELOCITY_BITS",
    "ContactStiffnessEstimator",
    "CrankSliderKinematics",
    "ForceControlObservation",
    "ForceControlReference",
    "ForceSemantics",
    "ForceTrackingController",
    "MITControlCommand",
    "MITTorqueController",
    "NormalForceControlCommand",
    "NormalForceController",
    "SecondOrderTorqueLADRC",
    "TorqueAdrcStep",
    "_roundtrip_unsigned",
]

DAMIAO_POSITION_BITS = 16
DAMIAO_VELOCITY_BITS = 12
DAMIAO_TORQUE_BITS = 12
DAMIAO_GAIN_BITS = 12
DAMIAO_STIFFNESS_RANGE = (0.0, 500.0)
DAMIAO_DAMPING_RANGE = (0.0, 5.0)


class CrankSliderKinematics(_CoreCrankSliderKinematics):
    """曲柄滑块夹爪运动学；公式维护在核心包，此处仅补充 profile 构造。"""

    @classmethod
    def from_config(cls, config: CrankSliderGeometry) -> "CrankSliderKinematics":
        """从 profile 几何配置创建运动学模型。"""
        return cls(
            theta0_rad=float(config.theta0_rad),
            crank_radius_m=float(config.crank_radius_m),
            link_length_m=float(config.link_length_m),
            offset_m=float(config.offset_m),
        )


def _mit_config(config: MITControl) -> MITControlConfig:
    """把 profile MIT 配置转换为核心 dataclass。"""
    return MITControlConfig(
        p_min=config.p_min,
        p_max=config.p_max,
        v_max=config.v_max,
        t_max=config.t_max,
        kp=config.kp,
        kd=config.kd,
        t_ff=config.t_ff,
    )


def _stiffness_config(config: ContactStiffnessControl) -> ContactStiffnessConfig:
    """把 profile 刚度估计配置转换为核心 dataclass。"""
    return ContactStiffnessConfig(
        enabled=config.enabled,
        method=config.method,
        initial_n_per_m=config.initial_n_per_m,
        min_n_per_m=config.min_n_per_m,
        max_n_per_m=config.max_n_per_m,
        filter_alpha=config.filter_alpha,
        min_delta_closure_m=config.min_delta_closure_m,
        min_delta_force_n=config.min_delta_force_n,
        window_size=config.window_size,
        min_samples=config.min_samples,
        position_feedforward_gain=config.position_feedforward_gain,
        torque_feedforward_gain=config.torque_feedforward_gain,
        position_limit_enabled=config.position_limit_enabled,
        position_limit_force_rate_n_s=config.position_limit_force_rate_n_s,
        position_limit_stiffness_safety_factor=config.position_limit_stiffness_safety_factor,
    )


def _adrc_config(config: TorqueAdrcControl) -> _TorqueAdrcConfig:
    """把 profile 二阶 LADRC 配置转换为核心 dataclass。"""
    return _TorqueAdrcConfig(
        equivalent_inertia_kg_m2=config.equivalent_inertia_kg_m2,
        input_gain_scale=config.input_gain_scale,
        controller_bandwidth_rad_s=config.controller_bandwidth_rad_s,
        observer_bandwidth_rad_s=config.observer_bandwidth_rad_s,
        measurement_filter_cutoff_hz=config.measurement_filter_cutoff_hz,
        min_input_gain_n_per_n_m_s2=config.min_input_gain_n_per_n_m_s2,
        max_input_gain_n_per_n_m_s2=config.max_input_gain_n_per_n_m_s2,
        max_torque_rate_n_m_s=config.max_torque_rate_n_m_s,
        tracking_differentiator_bandwidth_rad_s=config.tracking_differentiator_bandwidth_rad_s,
    )


def _normal_force_config(config: NormalForceControl) -> _NormalForceConfig:
    """把 profile 外环配置转换为核心 dataclass。

    传感器噪声与导纳字段属于实验编排与导纳适配层，不进入核心控制器。
    """
    return _NormalForceConfig(
        target_n=config.target_n,
        contact_threshold_n=config.contact_threshold_n,
        contact_confirm_steps=config.contact_confirm_steps,
        release_threshold_n=config.release_threshold_n,
        release_confirm_steps=config.release_confirm_steps,
        kp=config.kp,
        ki=config.ki,
        kd=config.kd,
        max_position_adjustment=config.max_position_adjustment,
        filter_cutoff_hz=config.filter_cutoff_hz,
        geometry=(
            CrankSliderKinematics.from_config(config.geometry)
            if config.geometry is not None
            else None
        ),
        stiffness=(_stiffness_config(config.stiffness) if config.stiffness is not None else None),
        torque_feedback_gain=config.torque_feedback_gain,
        adrc=(
            _AdrcConfig(
                b0_n_per_m=config.adrc.b0_n_per_m,
                controller_bandwidth_rad_s=config.adrc.controller_bandwidth_rad_s,
                observer_bandwidth_rad_s=config.adrc.observer_bandwidth_rad_s,
                max_closing_velocity_m_s=config.adrc.max_closing_velocity_m_s,
            )
            if config.adrc is not None
            else None
        ),
        torque_adrc=_adrc_config(config.torque_adrc) if config.torque_adrc is not None else None,
    )


class MITTorqueController:
    """把核心 MIT 力矩命令写入 MuJoCo 电机执行器的适配层。"""

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
        self._core = MITTorqueModel(_mit_config(config))
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

    @property
    def torque_limit_n_m(self) -> float:
        """返回 MIT 命令允许的对称输出轴力矩上限。"""
        return float(self._config.t_max)

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
        command = self._core.apply(
            position=self.position(data),
            velocity=self.velocity(data),
            target_position=target_position,
            target_velocity=target_velocity,
            feedforward_torque=feedforward_torque,
            stiffness_override=stiffness_override,
            damping_override=damping_override,
        )
        data.ctrl[self._actuator_id] = command.torque
        return command


class ContactStiffnessEstimator:
    """在线接触刚度估计；算法维护在核心包，此处做 profile 配置转换。"""

    def __init__(
        self,
        config: ContactStiffnessControl,
        kinematics: CrankSliderKinematics,
    ) -> None:
        """保存估计参数和几何模型，并初始化估计状态。"""
        self._core = _CoreContactStiffnessEstimator(_stiffness_config(config), kinematics)

    @property
    def estimate_n_per_m(self) -> float:
        """返回当前滤波后的等效接触刚度估计。"""
        return self._core.estimate_n_per_m

    def reset(
        self,
        *,
        position_rad: float | None = None,
        normal_force_n: float | None = None,
    ) -> None:
        """重置估计，并可选记录新的接触参考点。"""
        self._core.reset(position_rad=position_rad, normal_force_n=normal_force_n)

    def update(self, *, position_rad: float, normal_force_n: float) -> float:
        """用新的接触样本更新刚度估计，并返回当前估计值。"""
        return self._core.update(position_rad=position_rad, normal_force_n=normal_force_n)


class SecondOrderTorqueLADRC:
    """二阶直接力矩 LADRC；算法维护在核心包，此处做 profile 配置转换。"""

    def __init__(self, config: TorqueAdrcControl) -> None:
        """保存参数并初始化观测器。"""
        self._core = _CoreSecondOrderTorqueLADRC(_adrc_config(config))

    def reset(
        self,
        *,
        measured_force_n: float = 0.0,
        applied_torque_n_m: float = 0.0,
        model_feedforward_torque_n_m: float = 0.0,
        input_gain_n_per_n_m_s2: float | None = None,
    ) -> None:
        """按当前力与实际力矩初始化，保证稳态切换时控制量连续。"""
        self._core.reset(
            measured_force_n=measured_force_n,
            applied_torque_n_m=applied_torque_n_m,
            model_feedforward_torque_n_m=model_feedforward_torque_n_m,
            input_gain_n_per_n_m_s2=input_gain_n_per_n_m_s2,
        )

    def update_reference(
        self,
        *,
        target_force_n: float,
        target_force_rate_n_s: float,
        target_force_acceleration_n_s2: float,
        dt: float,
    ) -> tuple[float, float, float]:
        """更新可选的临界阻尼线性 TD，并返回控制律使用的参考状态。"""
        return self._core.update_reference(
            target_force_n=target_force_n,
            target_force_rate_n_s=target_force_rate_n_s,
            target_force_acceleration_n_s2=target_force_acceleration_n_s2,
            dt=dt,
        )

    def set_applied_torque(
        self,
        applied_torque_n_m: float,
        *,
        model_feedforward_torque_n_m: float,
    ) -> None:
        """记录实际总力矩及同周期名义模型前馈。"""
        self._core.set_applied_torque(
            applied_torque_n_m,
            model_feedforward_torque_n_m=model_feedforward_torque_n_m,
        )

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
        return self._core.step(
            measured_force_n=measured_force_n,
            target_force_n=target_force_n,
            target_force_rate_n_s=target_force_rate_n_s,
            target_force_acceleration_n_s2=target_force_acceleration_n_s2,
            model_feedforward_torque_n_m=model_feedforward_torque_n_m,
            input_gain_n_per_n_m_s2=input_gain_n_per_n_m_s2,
            dt=dt,
            min_torque_n_m=min_torque_n_m,
            max_torque_n_m=max_torque_n_m,
        )


class _DataBoundInner:
    """把 ``MITTorqueController`` 与当期 ``MjData`` 绑定为核心内环协议。"""

    def __init__(self, inner: MITTorqueController, data) -> None:
        """记录当期执行器适配器与仿真状态。"""
        self._inner = inner
        self._data = data

    @property
    def torque_limit_n_m(self) -> float:
        """返回内环允许的对称输出力矩上限。"""
        return self._inner.torque_limit_n_m

    def position(self) -> float:
        """返回当前受控关节位置。"""
        return self._inner.position(self._data)

    def apply(self, *, target_position: float, **overrides) -> MITControlCommand:
        """把请求转发给执行器适配器并写入 ``ctrl``。"""
        return self._inner.apply(self._data, target_position=target_position, **overrides)


class NormalForceController:
    """法向力外环与接触状态机；算法维护在核心包，此处绑定 MuJoCo 状态。"""

    def __init__(
        self,
        inner: MITTorqueController,
        config: NormalForceControl,
        *,
        force_semantics: ForceSemantics = "average_side",
    ) -> None:
        """创建可切换的法向力外环与接触状态机。

        Args:
            inner: MIT 力矩内环控制器。
            config: 外环法向力跟踪配置；``config.adrc`` 非 ``None`` 时跟踪阶段
                以一阶 LADRC 外环替换 PID 位置修正；``config.torque_adrc`` 非
                ``None`` 时改用二阶直接力矩 LADRC。
            force_semantics: 目标力的语义（平均单侧力或总力）。
        """
        self._inner = inner
        self._core = _CoreNormalForceController(
            _normal_force_config(config),
            force_semantics=force_semantics,
        )

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
        return self._core.state

    def reset(self) -> None:
        """返回接近模式，并清除滤波器、计数器和 PID 历史。"""
        self._core.reset()

    def step(
        self,
        data,
        *,
        observation: ForceControlObservation,
        reference: ForceControlReference,
    ) -> NormalForceControlCommand:
        """使用当前观测和参考量推进一个控制周期。"""
        return self._core.step(
            _DataBoundInner(self._inner, data),
            observation=observation,
            reference=reference,
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
        target_force_rate_n_s: float = 0.0,
        target_force_acceleration_n_s2: float = 0.0,
    ) -> NormalForceControlCommand:
        """推进接触检测/力跟踪，并写入一个电机力矩。

        接触要求两个指尖都保持在配置阈值之上。一旦确认，simple-pid 会
        调整检测到的接触位置，现有 MIT 控制器再将该位置目标转换为电机力矩。
        """
        return self._core.apply(
            _DataBoundInner(self._inner, data),
            approach_position=approach_position,
            total_normal_force_n=total_normal_force_n,
            left_normal_force_n=left_normal_force_n,
            right_normal_force_n=right_normal_force_n,
            dt=dt,
            approach_velocity=approach_velocity,
            target_force_n=target_force_n,
            approach_feedforward_force_n=approach_feedforward_force_n,
            target_force_rate_n_s=target_force_rate_n_s,
            target_force_acceleration_n_s2=target_force_acceleration_n_s2,
        )


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

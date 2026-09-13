"""共享核控制器到真机 MIT 请求的适配。

生命周期归运行时所有：本模块只提供 ``begin_contact_tracking``／
``step_tracking``／``reset``，不自行切回接近或决定释放。导纳路径消费
外层已滤波的触觉力；PID／一阶 LADRC 路径把原始力交给共享核，由核心
内部做唯一一次低通，避免双重滤波。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from dmgripper_hardware import MotorFeedback

from dm_grasp_core import (
    AdrcConfig,
    ContactStiffnessConfig,
    ForceControlObservation,
    ForceControlReference,
    MITCommand,
    MITCommandConfig,
    MITControlCommand,
    NormalForceConfig,
    NormalForceController,
    SecondOrderAdmittance,
    build_mit_command,
    step_admittance,
)

from .config import ExperimentConfig
from .observation import PairedObservation
from .targets import ForceTarget


@dataclass(frozen=True, slots=True)
class TrackingStep:
    """一次跟踪计算产出的受限 MIT 请求与控制器诊断。

    Attributes:
        command: 发给电机的受限 MIT 请求。
        force_deadband_active: 导纳死区本周期是否生效（其他控制器恒 False）。
        unloading_blocked: 导纳单向闭合本周期是否阻止卸载（其他控制器恒 False）。
        filtered_force_n: 控制实际使用的滤波力（导纳路径为 ``None``，
            其控制输入即外层滤波力，已记录在触觉字段中）。
        position_adjustment: 本周期位置修正量（导纳路径为 ``None``）。
        stiffness_estimate_n_per_m: 控制消费的刚度估计值；未消费为 ``None``。
        force_error_n: 目标力与滤波力之差；不可得时为 ``None``。
    """

    command: MITCommand
    force_deadband_active: bool
    unloading_blocked: bool
    filtered_force_n: float | None
    position_adjustment: float | None
    stiffness_estimate_n_per_m: float | None
    force_error_n: float | None


class _MitRequestAdapter:
    """把共享核外环请求转换为受限 MIT 请求的内环适配。

    限幅规则沿用已验证的 cup 实现：每周期位置变化不超过
    ``velocity_limit × dt``，最终命令经机械行程与 MIT 合成力矩限幅。
    """

    def __init__(
        self,
        *,
        kinematics,
        command_config: MITCommandConfig,
        max_control_gap_s: float,
    ) -> None:
        """保存限幅所需参数。"""
        self._kinematics = kinematics
        self._command_config = command_config
        self._max_control_gap_s = max_control_gap_s
        self.feedback: MotorFeedback | None = None
        self.dt = 0.0
        self.previous_target: float | None = None
        self.last_command: MITCommand | None = None

    @property
    def torque_limit_n_m(self) -> float:
        """提供共享法向力核心所需的力矩上限。"""
        return self._command_config.torque_limit_nm

    def position(self) -> float:
        """只返回当前真机反馈位置。"""
        if self.feedback is None:
            raise RuntimeError("内环适配尚未收到电机反馈")
        return self.feedback.position_rad

    def reset(self, position_rad: float) -> None:
        """接触建立后重置速度限幅参考。"""
        self.previous_target = position_rad

    def bind(self, feedback: MotorFeedback, dt: float) -> None:
        """登记本周期反馈与真实控制间隔。"""
        if not math.isfinite(dt) or not 0 < dt <= self._max_control_gap_s:
            raise RuntimeError("force control cycle exceeded its valid interval")
        self.feedback = feedback
        self.dt = dt

    def apply(
        self,
        *,
        target_position: float,
        target_velocity: float = 0.0,
        feedforward_torque: float | None = None,
        stiffness_override: float | None = None,
        damping_override: float | None = None,
    ) -> MITControlCommand:
        """输出一个受限 MIT 请求；运行时增益覆盖不受支持。"""
        if stiffness_override is not None or damping_override is not None:
            raise ValueError("真机 MIT 适配不支持运行时增益覆盖")
        config = self._command_config
        if self.feedback is None or self.previous_target is None:
            raise RuntimeError("内环适配尚未初始化")
        maximum = config.velocity_limit_rad_s * self.dt
        position = max(
            config.position_min_rad,
            min(
                config.position_max_rad,
                max(
                    self.previous_target - maximum,
                    min(self.previous_target + maximum, target_position),
                ),
            ),
        )
        velocity = max(
            -config.velocity_limit_rad_s, min(config.velocity_limit_rad_s, target_velocity)
        )
        ff = 0.0 if feedforward_torque is None else feedforward_torque
        jacobian = self._kinematics.closure_jacobian(self.feedback.position_rad)
        if not math.isfinite(jacobian) or abs(jacobian) < 1e-9:
            raise ValueError("invalid gripper Jacobian")
        command = build_mit_command(
            self._kinematics,
            config,
            reference_position_rad=position,
            displacement_m=0.0,
            velocity_m_s=velocity * self._kinematics.closure_jacobian(position),
            measured_position_rad=self.feedback.position_rad,
            measured_velocity_rad_s=self.feedback.velocity_rad_s,
            feedforward_force_n=ff / jacobian,
            feedforward_ratio=1.0,
        )
        self.previous_target = command.position_rad
        self.last_command = command
        torque = (
            config.kp * (command.position_rad - self.feedback.position_rad)
            + config.kd * (command.velocity_rad_s - self.feedback.velocity_rad_s)
            + command.feedforward_torque_nm
        )
        return MITControlCommand(
            command.position_rad,
            command.velocity_rad_s,
            self.feedback.position_rad,
            self.feedback.velocity_rad_s,
            command.feedforward_torque_nm,
            torque,
        )


class GripController:
    """导纳、PID 与一阶 LADRC 的统一真机跟踪适配。

    只生成受限请求，不打开串口，也不重复定义外环控制律；三种控制器
    面对同一有效观测时共享运行时提供的刚度快照来源。
    """

    def __init__(
        self,
        config: ExperimentConfig,
        *,
        kinematics,
        command_config: MITCommandConfig,
    ) -> None:
        """以启动时固定的控制器与 MIT 参数创建适配器。"""
        self._config = config
        self._kinematics = kinematics
        self._command_config = command_config
        controller = config.controller
        self.admittance = SecondOrderAdmittance(
            controller.admittance.mass_kg,
            controller.admittance.damping_ns_m,
            controller.admittance.stiffness_n_m,
        )
        self._admittance_params = controller.admittance
        stiffness_config = None
        if controller.stiffness_consumption == "feedforward":
            # 外部估计模式：内部估计器关闭，仅保留前馈增益；估计由
            # 运行时的唯一所有者更新并经快照传入。
            stiffness_config = ContactStiffnessConfig(
                enabled=False,
                initial_n_per_m=config.estimation.initial_n_per_m,
                min_n_per_m=config.estimation.min_n_per_m,
                max_n_per_m=config.estimation.max_n_per_m,
                filter_alpha=config.estimation.filter_alpha,
                min_delta_closure_m=config.estimation.min_delta_closure_m,
                min_delta_force_n=config.estimation.min_delta_force_n,
            )
        self.normal = NormalForceController(
            NormalForceConfig(
                target_n=config.reference.initial_force_n,
                contact_threshold_n=config.lifecycle.contact_on_n,
                contact_confirm_steps=1,
                release_threshold_n=config.lifecycle.contact_off_n,
                release_confirm_steps=1,
                kp=controller.pid.kp,
                ki=controller.pid.ki,
                kd=controller.pid.kd,
                max_position_adjustment=controller.pid.max_position_adjustment_rad,
                filter_cutoff_hz=config.timing.tactile_cutoff_hz,
                geometry=kinematics,
                stiffness=stiffness_config,
                adrc=(
                    AdrcConfig(
                        b0_n_per_m=controller.adrc.b0_n_per_m,
                        controller_bandwidth_rad_s=controller.adrc.controller_bandwidth_rad_s,
                        observer_bandwidth_rad_s=controller.adrc.observer_bandwidth_rad_s,
                        max_closing_velocity_m_s=controller.adrc.max_closing_velocity_m_s,
                    )
                    if controller.kind == "adrc"
                    else None
                ),
            )
        )
        self._stiffness_consumption = controller.stiffness_consumption
        self._inner = _MitRequestAdapter(
            kinematics=kinematics,
            command_config=command_config,
            max_control_gap_s=config.timing.max_control_gap_s,
        )
        self._contact_reference_rad: float | None = None

    def reset(self, position_rad: float) -> None:
        """接触边沿重置导纳状态、核心跟踪状态与速度限幅参考。"""
        self.admittance.reset()
        self.normal.reset()
        self._inner.reset(position_rad)
        self._contact_reference_rad = position_rad

    def begin_contact_tracking(
        self,
        *,
        paired: PairedObservation,
        target: ForceTarget,
        time_s: float,
        dt: float,
        stiffness_value: float | None = None,
    ) -> TrackingStep:
        """接触建立后初始化跟踪控制律并生成首个命令。"""
        if self._contact_reference_rad is None:
            self.reset(paired.feedback.position_rad)
        self._inner.bind(paired.feedback, dt)
        return self._dispatch(
            paired=paired,
            target=target,
            time_s=time_s,
            dt=dt,
            begin=True,
            stiffness_value=stiffness_value,
        )

    def step_tracking(
        self,
        *,
        paired: PairedObservation,
        target: ForceTarget,
        time_s: float,
        dt: float,
        stiffness_value: float | None = None,
    ) -> TrackingStep:
        """执行一次跟踪计算。"""
        self._inner.bind(paired.feedback, dt)
        return self._dispatch(
            paired=paired,
            target=target,
            time_s=time_s,
            dt=dt,
            begin=False,
            stiffness_value=stiffness_value,
        )

    def _require_contact_reference(self) -> float:
        """返回导纳围绕的接触参考位置。"""
        if self._contact_reference_rad is None:
            raise RuntimeError("跟踪尚未以接触参考初始化")
        return self._contact_reference_rad

    def _dispatch(
        self,
        *,
        paired: PairedObservation,
        target: ForceTarget,
        time_s: float,
        dt: float,
        begin: bool,
        stiffness_value: float | None,
    ) -> TrackingStep:
        """按控制器类型生成受限命令与诊断。"""
        _check_target_bounds(target.force_n, self._config)
        if self._config.controller.kind == "admittance":
            return self._step_admittance(paired=paired, target=target, dt=dt)
        return self._step_normal(
            paired=paired,
            target=target,
            time_s=time_s,
            dt=dt,
            begin=begin,
            stiffness_value=stiffness_value,
        )

    def _step_admittance(
        self,
        *,
        paired: PairedObservation,
        target: ForceTarget,
        dt: float,
    ) -> TrackingStep:
        """导纳路径：输入外层滤波力，死区与单向闭合约定为导纳参数。"""
        grip = self._admittance_params
        command = step_admittance(
            self.admittance,
            self._kinematics,
            self._command_config,
            reference_position_rad=self._require_contact_reference(),
            measured_position_rad=paired.feedback.position_rad,
            measured_velocity_rad_s=paired.feedback.velocity_rad_s,
            left_force_n=paired.snapshot.left_force_n,
            right_force_n=paired.snapshot.right_force_n,
            target_force_n=target.force_n,
            dt_s=dt,
            force_deadband_n=grip.force_deadband_n,
            prevent_unloading=grip.prevent_unloading,
        )
        self._inner.previous_target = command.position_rad
        return TrackingStep(
            command=command,
            force_deadband_active=self.admittance.deadband_active,
            unloading_blocked=self.admittance.unloading_blocked,
            filtered_force_n=None,
            position_adjustment=None,
            stiffness_estimate_n_per_m=None,
            force_error_n=(
                target.force_n - (paired.snapshot.left_force_n + paired.snapshot.right_force_n) / 2
            ),
        )

    def _step_normal(
        self,
        *,
        paired: PairedObservation,
        target: ForceTarget,
        time_s: float,
        dt: float,
        begin: bool,
        stiffness_value: float | None,
    ) -> TrackingStep:
        """PID／LADRC 路径：原始力交给共享核，由核心内部唯一滤波。"""
        observation = ForceControlObservation(
            time_s=time_s,
            approach_position=paired.feedback.position_rad,
            total_normal_force_n=paired.snapshot.raw_left_fz_n + paired.snapshot.raw_right_fz_n,
            left_normal_force_n=paired.snapshot.raw_left_fz_n,
            right_normal_force_n=paired.snapshot.raw_right_fz_n,
            dt=dt,
        )
        reference = ForceControlReference(
            target_force_n=target.force_n,
            target_force_rate_n_s=target.rate_n_s if target.rate_n_s is not None else 0.0,
            target_force_acceleration_n_s2=(
                target.acceleration_n_s2 if target.acceleration_n_s2 is not None else 0.0
            ),
        )
        external_stiffness = (
            stiffness_value
            if self._stiffness_consumption == "feedforward" and stiffness_value is not None
            else None
        )
        if begin:
            result = self.normal.begin_tracking(
                self._inner,
                observation=observation,
                reference=reference,
                external_stiffness_n_per_m=external_stiffness,
            )
        else:
            result = self.normal.step_tracking(
                self._inner,
                observation=observation,
                reference=reference,
                external_stiffness_n_per_m=external_stiffness,
            )
        assert self._inner.last_command is not None, "跟踪计算必须产生 MIT 请求"
        return TrackingStep(
            command=self._inner.last_command,
            force_deadband_active=False,
            unloading_blocked=False,
            filtered_force_n=result.filtered_force_n,
            position_adjustment=result.position_adjustment,
            stiffness_estimate_n_per_m=external_stiffness,
            force_error_n=result.force_error_n,
        )


def _check_target_bounds(force_n: float, config: ExperimentConfig) -> None:
    """拒绝越界目标力。"""
    if not math.isfinite(force_n) or not 0 <= force_n <= config.safety.max_target_force_n:
        raise ValueError("force target outside experiment limits")

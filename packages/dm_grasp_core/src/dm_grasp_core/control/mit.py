"""达妙 MIT 协议量化与 MIT 风格力矩命令的纯计算。

本模块不含任何后端状态访问：调用者传入实测位置与速度，
返回量化、饱和后的命令数据；把力矩写入执行器由仿真适配层或
真机适配层完成。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


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
class MITControlConfig:
    """MIT 风格输出轴的位置、速度和力矩限值与增益。

    Attributes:
        p_min: 命令位置下限 (rad)。
        p_max: 命令位置上限 (rad)。
        v_max: 命令速度绝对值上限 (rad/s)。
        t_max: 输出力矩绝对值上限 (N·m)。
        kp: 位置刚度 (N·m/rad)。
        kd: 速度阻尼 (N·m·s/rad)。
        t_ff: 常值前馈力矩 (N·m)。
    """

    p_min: float
    p_max: float
    v_max: float
    t_max: float
    kp: float
    kd: float
    t_ff: float = 0.0

    def __post_init__(self) -> None:
        """确保命令位置与前馈范围相互一致。"""
        if self.p_min >= self.p_max:
            raise ValueError("p_min must be smaller than p_max")
        if self.v_max <= 0.0:
            raise ValueError("v_max must be positive")
        if self.t_max <= 0.0:
            raise ValueError("t_max must be positive")
        if not 0.0 <= self.kp <= 500.0:
            raise ValueError("kp must lie within [0, 500]")
        if not 0.0 <= self.kd <= 5.0:
            raise ValueError("kd must lie within [0, 5]")
        if abs(self.t_ff) > self.t_max:
            raise ValueError("t_ff must not exceed t_max")


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
class MITTorqueModel:
    """把 ``kp*(p_des-p) + kd*(v_des-v) + t_ff`` 计算为量化后的输出轴力矩。

    命令位置、速度、前馈与增益先按达妙协议量化再参与计算，与真机
    CAN 帧的数值语义一致。
    """

    config: MITControlConfig

    def apply(
        self,
        *,
        position: float,
        velocity: float,
        target_position: float,
        target_velocity: float = 0.0,
        feedforward_torque: float | None = None,
        stiffness_override: float | None = None,
        damping_override: float | None = None,
    ) -> MITControlCommand:
        """计算、按达妙协议量化、饱和并返回一个 MIT 力矩命令。

        Args:
            position: 实测关节位置 (rad)。
            velocity: 实测关节速度 (rad/s)。
            target_position: 目标位置，会先按达妙位置编码量化。
            target_velocity: 目标速度，默认 0。
            feedforward_torque: 本周期前馈力矩；``None`` 时沿用
                ``config.t_ff``。
            stiffness_override: 仅本周期生效的 MIT kp 覆盖；``None`` 时沿用
                ``config.kp``。直接力矩式力控在跟踪阶段传 0.0 以旁路位置弹簧。
            damping_override: 仅本周期生效的 MIT kd 覆盖；``None`` 时沿用
                ``config.kd``。语义与 ``stiffness_override`` 一致。

        Returns:
            量化与饱和后的 ``MITControlCommand``。
        """
        config = self.config
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
        torque = stiffness * (desired_position - position)
        torque += damping * (desired_velocity - velocity) + feedforward
        torque = float(np.clip(torque, -config.t_max, config.t_max))
        return MITControlCommand(
            target_position=desired_position,
            target_velocity=desired_velocity,
            position=position,
            velocity=velocity,
            feedforward_torque=feedforward,
            torque=torque,
        )

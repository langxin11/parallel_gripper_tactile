"""从 force_tracking_node.py 提取 MIT 映射与跟踪步骤，保持运算顺序。

修改：用显式配置、输入和纯 Python 返回值替代 ROS 参数、消息及节点状态。
"""

from __future__ import annotations

from dataclasses import dataclass

from ..control.admittance import SecondOrderAdmittance, limit_mit_position_for_torque
from ..control.kinematics import CrankSliderKinematics


@dataclass(frozen=True, slots=True)
class MITCommandConfig:
    """MIT 映射参数；位置 rad，速度 rad/s，力矩 N·m，方向为 ±1。

    与原节点相同，由调用者在启动时验证设备参数；这里不新增安全策略。

    Attributes:
        position_min_rad: 机械角下限 (rad)。
        position_max_rad: 机械角上限 (rad)。
        velocity_limit_rad_s: 角速度绝对值上限 (rad/s)。
        closing_direction: 闭合方向符号，无量纲，±1。
        kp: 位置刚度 (N·m/rad)。
        kd: 速度阻尼 (N·m·s/rad)。
        feedforward_ratio: 力前馈比例，无量纲。
        feedforward_torque_limit_nm: 前馈力矩绝对值上限 (N·m)。
        torque_limit_nm: 合成力矩绝对值上限 (N·m)。
    """

    position_min_rad: float
    position_max_rad: float
    velocity_limit_rad_s: float
    closing_direction: int
    kp: float
    kd: float
    feedforward_ratio: float
    feedforward_torque_limit_nm: float
    torque_limit_nm: float


@dataclass(frozen=True, slots=True)
class MITCommand:
    """未经过协议量化的 MIT 命令。

    Attributes:
        position_rad: 目标角位置 (rad)。
        velocity_rad_s: 目标角速度 (rad/s)。
        kp: 位置刚度 (N·m/rad)。
        kd: 速度阻尼 (N·m·s/rad)。
        feedforward_torque_nm: 前馈力矩 (N·m)。
    """

    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    feedforward_torque_nm: float


def build_mit_command(
    kinematics: CrankSliderKinematics,
    config: MITCommandConfig,
    *,
    reference_position_rad: float,
    displacement_m: float,
    velocity_m_s: float,
    measured_position_rad: float,
    measured_velocity_rad_s: float,
    feedforward_force_n: float,
    feedforward_ratio: float | None = None,
    torque_limit_nm: float | None = None,
) -> MITCommand:
    """把导纳闭合位移映射为带限幅的 MIT 命令。

    目标构型雅可比映射速度，实测构型雅可比映射平均单侧力前馈。
    合成力矩限幅后再裁剪机械角；两种约束不相容时抛出 ValueError。
    不负责协议量化、设备参数验证或观测有效性判定。

    Args:
        kinematics: 曲柄滑块模型。
        config: 经过调用者验证的 MIT 参数。
        reference_position_rad: 接触参考角 (rad)。
        displacement_m: 相对参考点的虚拟闭合位移 (m)。
        velocity_m_s: 虚拟闭合速度 (m/s)。
        measured_position_rad: 实测角位置 (rad)。
        measured_velocity_rad_s: 实测角速度 (rad/s)。
        feedforward_force_n: 前馈平均单侧力 (N)。
        feedforward_ratio: 可选的无量纲前馈比例覆盖值。
        torque_limit_nm: 可选的合成力矩上限覆盖值 (N·m)。

    Returns:
        MITCommand: 满足机械角与合成力矩约束的未量化命令。

    Raises:
        ValueError: 运动学或力矩约束输入无效，或两种限位不可兼容。
    """
    closing_direction = config.closing_direction
    position_min_rad = config.position_min_rad
    position_max_rad = config.position_max_rad
    reference_closure_m = kinematics.closure(reference_position_rad)
    position_rad = kinematics.position_for_closure(
        reference_closure_m + closing_direction * displacement_m,
        position_min_rad,
        position_max_rad,
    )
    target_jacobian_m_per_rad = kinematics.closure_jacobian(position_rad)
    velocity_rad_s = closing_direction * velocity_m_s / target_jacobian_m_per_rad
    position_rad = min(max(position_rad, position_min_rad), position_max_rad)
    velocity_limit_rad_s = config.velocity_limit_rad_s
    velocity_rad_s = min(max(velocity_rad_s, -velocity_limit_rad_s), velocity_limit_rad_s)
    if feedforward_ratio is None:
        feedforward_ratio = config.feedforward_ratio
    measured_jacobian_m_per_rad = kinematics.closure_jacobian(measured_position_rad)
    torque_nm = (
        closing_direction * feedforward_ratio * measured_jacobian_m_per_rad * feedforward_force_n
    )
    if torque_limit_nm is None:
        torque_limit_nm = config.torque_limit_nm
    kp = config.kp
    kd = config.kd
    feedforward_limit_nm = config.feedforward_torque_limit_nm
    feedforward_torque_nm = min(max(torque_nm, -feedforward_limit_nm), feedforward_limit_nm)
    position_rad = limit_mit_position_for_torque(
        target_position_rad=position_rad,
        target_velocity_rad_s=velocity_rad_s,
        measured_position_rad=measured_position_rad,
        measured_velocity_rad_s=measured_velocity_rad_s,
        kp=kp,
        kd=kd,
        feedforward_torque_nm=feedforward_torque_nm,
        torque_limit_nm=torque_limit_nm,
    )
    position_rad = min(max(position_rad, position_min_rad), position_max_rad)
    predicted_torque_nm = (
        kp * (position_rad - measured_position_rad)
        + kd * (velocity_rad_s - measured_velocity_rad_s)
        + feedforward_torque_nm
    )
    if abs(predicted_torque_nm) > torque_limit_nm + 1e-9:
        raise ValueError("机械角限位内无法满足 MIT 合成力矩上限")
    return MITCommand(position_rad, velocity_rad_s, kp, kd, feedforward_torque_nm)


def step_admittance(
    admittance: SecondOrderAdmittance,
    kinematics: CrankSliderKinematics,
    config: MITCommandConfig,
    *,
    reference_position_rad: float,
    measured_position_rad: float,
    measured_velocity_rad_s: float,
    left_force_n: float,
    right_force_n: float,
    target_force_n: float,
    dt_s: float,
) -> MITCommand:
    """按平均单侧力误差积分、限制导纳状态并构建 MIT 命令。

    原地更新 admittance；沿用原节点错误时可能已更新状态的语义。
    调用者负责接触状态、时间步裁剪、目标力上限、复位及设备保护。

    Args:
        admittance: 原地更新的二阶导纳状态。
        kinematics: 曲柄滑块模型。
        config: 经过调用者验证的 MIT 参数。
        reference_position_rad: 接触参考角 (rad)。
        measured_position_rad: 实测角位置 (rad)。
        measured_velocity_rad_s: 实测角速度 (rad/s)。
        left_force_n: 左侧法向力 (N)。
        right_force_n: 右侧法向力 (N)。
        target_force_n: 目标平均单侧力 (N)。
        dt_s: 积分步长 (s)，本函数不裁剪。

    Returns:
        MITCommand: 此步生成的未量化命令。

    Raises:
        ValueError: 导纳积分、运动学或命令约束失败。
    """
    measured_force_n = 0.5 * (float(left_force_n) + float(right_force_n))
    jacobian_m_per_rad = kinematics.closure_jacobian(float(measured_position_rad))
    maximum_velocity_m_s = config.velocity_limit_rad_s * jacobian_m_per_rad
    displacement_m, velocity_m_s = admittance.step(
        target_force_n - measured_force_n,
        dt_s,
        maximum_velocity_m_s=maximum_velocity_m_s,
    )
    closing_direction = config.closing_direction
    reference_closure_m = kinematics.closure(reference_position_rad)
    position_bounds_m = sorted(
        (
            (kinematics.closure(config.position_min_rad) - reference_closure_m) / closing_direction,
            (kinematics.closure(config.position_max_rad) - reference_closure_m) / closing_direction,
        )
    )
    displacement_m, velocity_m_s = admittance.limit_state(
        position_bounds_m[0],
        position_bounds_m[1],
        maximum_velocity_m_s,
    )
    return build_mit_command(
        kinematics,
        config,
        reference_position_rad=reference_position_rad,
        displacement_m=displacement_m,
        velocity_m_s=velocity_m_s,
        measured_position_rad=measured_position_rad,
        measured_velocity_rad_s=measured_velocity_rad_s,
        feedforward_force_n=target_force_n,
    )

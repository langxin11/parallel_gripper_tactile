"""DMgripper 纯算法；从 dm_gripper_control/control.py 抽取，移除端点搜索类。"""

from __future__ import annotations
import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class CrankSliderKinematics:
    """曲柄滑块夹爪的总开度、闭合行程和雅可比。

    Attributes:
        theta0_rad: 电机零点对应的曲柄初始角 (rad)。
        crank_radius_m: 曲柄半径 (m)。
        link_length_m: 连杆长度 (m)。
        offset_m: 滑块法向偏置 (m)。
    """

    theta0_rad: float
    crank_radius_m: float
    link_length_m: float
    offset_m: float

    def _geometry_terms(self, position_rad: float) -> tuple[float, float, float]:
        """返回曲柄角、径向偏置和连杆根式项。"""
        values = (
            self.theta0_rad,
            self.crank_radius_m,
            self.link_length_m,
            self.offset_m,
            position_rad,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or self.crank_radius_m <= 0.0
            or self.link_length_m <= 0.0
            or self.offset_m < 0.0
        ):
            raise ValueError("曲柄滑块几何参数无效")
        alpha = position_rad + self.theta0_rad
        radial_offset = self.crank_radius_m * math.sin(alpha) - self.offset_m
        radicand = self.link_length_m**2 - radial_offset**2
        if radicand <= 0.0:
            raise ValueError("曲柄滑块超出有效运动学域")
        return alpha, radial_offset, radicand

    def aperture(self, position_rad: float) -> float:
        """返回两指总开度 (m)。"""
        alpha, _radial_offset, radicand = self._geometry_terms(position_rad)
        return 2.0 * (self.crank_radius_m * math.cos(alpha) + math.sqrt(radicand))

    def closure(self, position_rad: float) -> float:
        """返回从电机 ``q=0`` 起算的总闭合行程 (m)。"""
        return self.aperture(0.0) - self.aperture(position_rad)

    def closure_jacobian(self, position_rad: float) -> float:
        """返回总闭合行程对电机角的导数 ``dc/dq`` (m/rad)。"""
        alpha, radial_offset, radicand = self._geometry_terms(position_rad)
        jacobian = (
            2.0
            * self.crank_radius_m
            * (math.sin(alpha) + radial_offset * math.cos(alpha) / math.sqrt(radicand))
        )
        if not math.isfinite(jacobian) or jacobian <= 0.0:
            raise ValueError("工作区间内的曲柄滑块闭合雅可比必须为正")
        return jacobian

    def position_for_closure(
        self,
        closure_m: float,
        position_min_rad: float,
        position_max_rad: float,
    ) -> float:
        """在机械范围内反解给定总闭合行程对应的电机角。

        Args:
            closure_m: 目标总闭合行程 (m)。
            position_min_rad: 电机角下限 (rad)。
            position_max_rad: 电机角上限 (rad)。

        Returns:
            float: 经机械范围裁剪后的电机角 (rad)。

        Raises:
            ValueError: 范围无效或区间内闭合行程不单调时抛出。
        """
        if (
            not math.isfinite(closure_m)
            or not math.isfinite(position_min_rad)
            or not math.isfinite(position_max_rad)
            or position_min_rad >= position_max_rad
        ):
            raise ValueError("曲柄滑块反解参数无效")
        lower_closure = self.closure(position_min_rad)
        upper_closure = self.closure(position_max_rad)
        if lower_closure >= upper_closure:
            raise ValueError("机械范围内的曲柄滑块闭合行程必须单调递增")
        target = min(max(closure_m, lower_closure), upper_closure)
        lower = position_min_rad
        upper = position_max_rad
        for _ in range(48):
            middle = 0.5 * (lower + upper)
            if self.closure(middle) < target:
                lower = middle
            else:
                upper = middle
        return 0.5 * (lower + upper)


@dataclass(frozen=True, slots=True)
class MinimumJerkTrajectory:
    """满足速度、加速度和加加速度上限的五次位置轨迹。

    Attributes:
        start_position_rad: 轨迹起点 (rad)。
        goal_position_rad: 轨迹终点 (rad)。
        duration_s: 由三项运动学上限共同确定的持续时间 (s)。
    """

    start_position_rad: float
    goal_position_rad: float
    duration_s: float

    @classmethod
    def from_limits(
        cls,
        start_position_rad: float,
        goal_position_rad: float,
        max_velocity_rad_s: float,
        max_acceleration_rad_s2: float,
        max_jerk_rad_s3: float,
    ) -> MinimumJerkTrajectory:
        """根据五次曲线的解析峰值计算不违反限制的最短持续时间。

        Args:
            start_position_rad: 起点 (rad)。
            goal_position_rad: 终点 (rad)。
            max_velocity_rad_s: 最大速度绝对值 (rad/s)。
            max_acceleration_rad_s2: 最大加速度绝对值 (rad/s²)。
            max_jerk_rad_s3: 最大加加速度绝对值 (rad/s³)。

        Returns:
            MinimumJerkTrajectory: 可按任意时刻采样的位置轨迹。

        Raises:
            ValueError: 输入非有限或运动学上限不是正数时抛出。
        """
        values = (
            start_position_rad,
            goal_position_rad,
            max_velocity_rad_s,
            max_acceleration_rad_s2,
            max_jerk_rad_s3,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or max_velocity_rad_s <= 0.0
            or max_acceleration_rad_s2 <= 0.0
            or max_jerk_rad_s3 <= 0.0
        ):
            raise ValueError("五次轨迹参数无效")
        distance_rad = abs(goal_position_rad - start_position_rad)
        if distance_rad <= 1e-12:
            return cls(start_position_rad, goal_position_rad, 0.0)
        duration_s = max(
            1.875 * distance_rad / max_velocity_rad_s,
            math.sqrt((10.0 / math.sqrt(3.0)) * distance_rad / max_acceleration_rad_s2),
            math.cbrt(60.0 * distance_rad / max_jerk_rad_s3),
        )
        return cls(start_position_rad, goal_position_rad, duration_s)

    def sample(self, elapsed_s: float) -> tuple[float, float, float]:
        """采样轨迹位置、速度和加速度，超出持续时间后保持终点。

        Args:
            elapsed_s: 从轨迹启动起累计的时间 (s)。

        Returns:
            tuple[float, float, float]: 目标位置 (rad)、目标速度 (rad/s) 与
                目标加速度 (rad/s²)。起终点处速度与加速度均为 0，满足
                minimum-jerk 边界条件。

        Raises:
            ValueError: 时间不是有限数时抛出。
        """
        if not math.isfinite(elapsed_s):
            raise ValueError("轨迹采样时间必须是有限数")
        if self.duration_s <= 0.0 or elapsed_s >= self.duration_s:
            return self.goal_position_rad, 0.0, 0.0
        if elapsed_s <= 0.0:
            return self.start_position_rad, 0.0, 0.0
        phase = elapsed_s / self.duration_s
        blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
        blend_rate = (30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4) / self.duration_s
        blend_accel = (60.0 * phase - 180.0 * phase**2 + 120.0 * phase**3) / (
            self.duration_s * self.duration_s
        )
        displacement_rad = self.goal_position_rad - self.start_position_rad
        return (
            self.start_position_rad + displacement_rad * blend,
            displacement_rad * blend_rate,
            displacement_rad * blend_accel,
        )


def quintic_blend(time_ratio: float) -> float:
    """返回 minimum-jerk 的 0→1 缓动值，起终点的速度与加速度均为 0。

    Args:
        time_ratio: 归一化时间，小于 0 或大于 1 时沿端点截断。

    Returns:
        float: 位于 [0, 1] 的五次缓动值。
    """
    if not math.isfinite(time_ratio):
        raise ValueError("缓动时间必须是有限数")
    phase = min(max(time_ratio, 0.0), 1.0)
    return 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5


def within_zero_window(left_force_n: float, right_force_n: float, threshold_n: float) -> bool:
    """判断双侧法向力是否同时落在零力窗口内。

    Args:
        left_force_n: 左侧法向力 (N)，可为负数（bias 后的残余读数）。
        right_force_n: 右侧法向力 (N)。
        threshold_n: 零窗口阈值 (N)，必须为非负数。

    Returns:
        bool: 两侧绝对值均不超过阈值时为 True。

    Raises:
        ValueError: 输入非有限或阈值为负时抛出。
    """
    values = (left_force_n, right_force_n, threshold_n)
    if not all(math.isfinite(value) for value in values) or threshold_n < 0.0:
        raise ValueError("零力窗口参数无效")
    return abs(left_force_n) <= threshold_n and abs(right_force_n) <= threshold_n


@dataclass(frozen=True, slots=True)
class ContactTransition:
    """接触后把期望速度从接触速度平滑衰减到 0 的速度过渡。

    位置保持接触点附近的参考位置，速度按 minimum-jerk 缓动从 ``start_velocity``
    递减到 0，避免高速接触后阻尼项 ``Kd(vd-v)`` 突然归零造成力矩冲击。

    Attributes:
        start_velocity_rad_s: 进入过渡瞬间的目标速度 (rad/s)。
        duration_s: 过渡持续时间 (s)，必须为正数。
    """

    start_velocity_rad_s: float
    duration_s: float

    def velocity_at(self, elapsed_s: float) -> float:
        """返回过渡过程中距离其初始速度衰减后的目标速度。

        Args:
            elapsed_s: 从过渡启动起累计的时间 (s)。

        Returns:
            float: 目标速度 (rad/s)，到达 ``duration_s`` 后为 0。
        """
        if not math.isfinite(elapsed_s):
            raise ValueError("过渡采样时间必须是有限数")
        if self.duration_s <= 0.0:
            return 0.0
        if elapsed_s <= 0.0:
            return self.start_velocity_rad_s
        if elapsed_s >= self.duration_s:
            return 0.0
        blend = quintic_blend(elapsed_s / self.duration_s)
        return self.start_velocity_rad_s * (1.0 - blend)


@dataclass(slots=True)
class SecondOrderAdmittance:
    """沿夹爪闭合方向积分的二阶导纳。

    Attributes:
        mass_kg: 虚拟质量 (kg)，必须为正数。
        damping_ns_m: 虚拟阻尼 (N·s/m)，不得为负数。
        stiffness_n_m: 虚拟刚度 (N/m)，不得为负数。
        displacement_m: 当前虚拟闭合位移 (m)。
        velocity_m_s: 当前虚拟闭合速度 (m/s)。
    """

    mass_kg: float
    damping_ns_m: float
    stiffness_n_m: float
    displacement_m: float = 0.0
    velocity_m_s: float = 0.0

    def reset(self) -> None:
        """清零虚拟位移和速度，以当前电机位置作为新的参考点。"""
        self.displacement_m = 0.0
        self.velocity_m_s = 0.0

    def step(self, force_error_n: float, dt_s: float) -> tuple[float, float]:
        """以半隐式欧拉法更新二阶导纳状态。

        Args:
            force_error_n: 目标力减测得力，正值推动夹爪继续闭合 (N)。
            dt_s: 本次离散积分步长 (s)。

        Returns:
            tuple[float, float]: 更新后的虚拟位移 (m) 与速度 (m/s)。

        Raises:
            ValueError: 参数或输入不满足有限性、物理符号约束时抛出。
        """
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
        acceleration_m_s2 = (
            force_error_n
            - self.damping_ns_m * self.velocity_m_s
            - self.stiffness_n_m * self.displacement_m
        ) / self.mass_kg
        self.velocity_m_s += acceleration_m_s2 * dt_s
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


class ContactDetector:
    """要求左右触觉力同时越过阈值并保持稳定时间。"""

    def __init__(self, threshold_n: float, stable_s: float) -> None:
        """创建双侧接触判定器。

        Args:
            threshold_n: 每侧接触力阈值 (N)。
            stable_s: 双侧同时越过阈值的持续时间 (s)。
        """
        self._threshold_n = threshold_n
        self._stable_s = stable_s
        self._started_s: float | None = None

    def reset(self) -> None:
        """清空已累计的稳定时间，使下一次接触重新计时。"""
        self._started_s = None

    def update(self, left_force_n: float, right_force_n: float, now_s: float) -> bool:
        """更新双侧力并返回稳定接触结果。

        Args:
            left_force_n: 左侧法向力 (N)。
            right_force_n: 右侧法向力 (N)。
            now_s: 单调时钟时间 (s)。

        Returns:
            bool: 已形成稳定双侧接触时为 True。
        """
        if min(left_force_n, right_force_n) < self._threshold_n:
            self._started_s = None
            return False
        if self._started_s is None:
            self._started_s = now_s
            return False
        return now_s - self._started_s >= self._stable_s

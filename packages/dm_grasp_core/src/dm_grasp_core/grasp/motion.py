"""DMgripper 抓取阶段的平滑运动与接触速度过渡。"""

from __future__ import annotations

import math
from dataclasses import dataclass


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

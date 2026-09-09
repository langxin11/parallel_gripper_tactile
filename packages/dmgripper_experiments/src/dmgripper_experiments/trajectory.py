"""夹爪闭合量空间的 minimum-jerk 轨迹。"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ClosureTrajectory:
    """满足速度、加速度和加加速度上限的五次闭合量轨迹。"""

    start_closure_m: float
    goal_closure_m: float
    duration_s: float

    @classmethod
    def from_limits(
        cls,
        start_closure_m: float,
        goal_closure_m: float,
        max_velocity_m_s: float,
        max_acceleration_m_s2: float,
        max_jerk_m_s3: float,
    ) -> ClosureTrajectory:
        """根据解析峰值生成不违反闭合量运动学上限的最短轨迹。"""
        values = (
            start_closure_m,
            goal_closure_m,
            max_velocity_m_s,
            max_acceleration_m_s2,
            max_jerk_m_s3,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or max_velocity_m_s <= 0.0
            or max_acceleration_m_s2 <= 0.0
            or max_jerk_m_s3 <= 0.0
        ):
            raise ValueError("闭合量轨迹参数无效")
        distance_m = abs(goal_closure_m - start_closure_m)
        if distance_m <= 1e-12:
            return cls(start_closure_m, goal_closure_m, 0.0)
        duration_s = max(
            1.875 * distance_m / max_velocity_m_s,
            math.sqrt((10.0 / math.sqrt(3.0)) * distance_m / max_acceleration_m_s2),
            math.cbrt(60.0 * distance_m / max_jerk_m_s3),
        )
        return cls(start_closure_m, goal_closure_m, duration_s)

    def sample(self, elapsed_s: float) -> tuple[float, float, float]:
        """返回目标闭合量、闭合速度和闭合加速度。"""
        if not math.isfinite(elapsed_s):
            raise ValueError("闭合量轨迹采样时间必须是有限数")
        if self.duration_s <= 0.0 or elapsed_s >= self.duration_s:
            return self.goal_closure_m, 0.0, 0.0
        if elapsed_s <= 0.0:
            return self.start_closure_m, 0.0, 0.0
        phase = elapsed_s / self.duration_s
        blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
        blend_rate = (30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4) / self.duration_s
        blend_accel = (60.0 * phase - 180.0 * phase**2 + 120.0 * phase**3) / (
            self.duration_s * self.duration_s
        )
        distance_m = self.goal_closure_m - self.start_closure_m
        return (
            self.start_closure_m + distance_m * blend,
            distance_m * blend_rate,
            distance_m * blend_accel,
        )

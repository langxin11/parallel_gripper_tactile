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

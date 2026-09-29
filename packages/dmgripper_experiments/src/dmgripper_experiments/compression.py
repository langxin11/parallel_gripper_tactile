"""由关节反馈约束首次接触后的总闭合行程。

此保护只覆盖主机观测与命令，不保证电机惯性、反馈延迟下的硬件绝对边界。
"""

from __future__ import annotations

import math

from dm_grasp_core import CrankSliderKinematics


def _finite(value: float, name: str, *, positive: bool = False) -> None:
    """拒绝非有限数值、布尔值和不合法正参数。"""
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or (positive and value <= 0.0)
    ):
        raise ValueError(f"{name} 必须为有限{'正' if positive else ''}数值")


class ContactCompressionGuard:
    """以任一侧首次有效接触为基线，保护两指总压缩行程。

    Args:
        kinematics: 关节角到两指总闭合量的非线性运动学。
        max_compression_m: 总压缩行程上限（米），None 关闭限制。
        contact_on_n: 任一侧建立接触所需的法向力（牛顿）。
    """

    def __init__(
        self,
        kinematics: CrankSliderKinematics,
        max_compression_m: float | None,
        contact_on_n: float,
    ) -> None:
        """校验参数并初始化接触段。"""
        if max_compression_m is not None:
            _finite(max_compression_m, "max_compression_m", positive=True)
        _finite(contact_on_n, "contact_on_n", positive=True)
        kinematics.closure(0.0)
        self._kinematics = kinematics
        self._max_compression_m = max_compression_m
        self._contact_on_n = contact_on_n
        self.reset()

    def reset(self) -> None:
        """显式开始新接触段，丢弃旧基线与反馈。"""
        self._contact_closure_m: float | None = None
        self._position_rad: float | None = None
        self._compression_m: float | None = None

    @property
    def contact_closure_m(self) -> float | None:
        """返回首次接触时的总闭合基线。"""
        return self._contact_closure_m

    @property
    def compression_m(self) -> float | None:
        """返回首次接触后的总闭合增量，无接触基线时返回 None。"""
        return self._compression_m

    def observe(self, position_rad: float, left_normal_n: float, right_normal_n: float) -> None:
        """更新反馈；测量到达最大压缩时立即拒绝继续运行。

        短暂失去接触不会重置基线，避免反复闭合绕过累积行程限制。
        """
        for name, value in (
            ("position_rad", position_rad),
            ("left_normal_n", left_normal_n),
            ("right_normal_n", right_normal_n),
        ):
            _finite(value, name)
        closure_m = self._kinematics.closure(position_rad)
        self._position_rad = position_rad
        if (
            self._contact_closure_m is None
            and max(left_normal_n, right_normal_n) >= self._contact_on_n
        ):
            self._contact_closure_m = closure_m
        if self._contact_closure_m is not None:
            self._compression_m = max(0.0, closure_m - self._contact_closure_m)
            if (
                self._max_compression_m is not None
                and self._compression_m >= self._max_compression_m
            ):
                raise RuntimeError("实测闭合行程已达到最大压缩深度")

    def check_command(self, position_rad: float, velocity_rad_s: float, dt_s: float) -> None:
        """校验目标位置及从最近反馈出发的一步速度预测。

        预测使用完整非线性运动学，而非固定雅可比近似。
        """
        _finite(position_rad, "position_rad")
        _finite(velocity_rad_s, "velocity_rad_s")
        _finite(dt_s, "dt_s", positive=True)
        target_closure_m = self._kinematics.closure(position_rad)
        origin_rad = self._position_rad if self._position_rad is not None else position_rad
        predicted_rad = origin_rad + velocity_rad_s * dt_s
        _finite(predicted_rad, "predicted_position_rad")
        predicted_closure_m = self._kinematics.closure(predicted_rad)
        if self._contact_closure_m is None or self._max_compression_m is None:
            return
        limit_m = self._contact_closure_m + self._max_compression_m
        if max(target_closure_m, predicted_closure_m) > limit_m:
            raise RuntimeError("命令目标或一步速度预测超过最大压缩深度")

    def trace_fields(self) -> dict[str, float | None]:
        """返回接触基线、实测总压缩和限值，单位均为米。"""
        return {
            "contact_closure_m": self._contact_closure_m,
            "contact_compression_m": self._compression_m,
            "contact_compression_limit_m": self._max_compression_m,
        }

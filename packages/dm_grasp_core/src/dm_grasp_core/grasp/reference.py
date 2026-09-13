"""目标力曲线的纯插值计算，供仿真与真机共用。

本模块只做 waypoint 插值与求导，不依赖配置框架、仿真主包或设备 I/O。
插值语义（含 ``hold`` 的精确切换边界、末点保持与区间外行为）以迁移前
仿真 ``ForceReference`` 的既有测试为准，不得在本模块中顺手改写。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Literal

ForceInterpolation = Literal["hold", "linear", "smoothstep"]


@dataclass(frozen=True, slots=True)
class ForceWaypoint:
    """目标力曲线上的一个时间—力 waypoint。

    Attributes:
        t_s: 相对曲线起点的时刻（s），非负。
        force_n: 该时刻的目标力（N），非负。
    """

    t_s: float
    force_n: float

    def __post_init__(self) -> None:
        """拒绝非有限或负值 waypoint。"""
        if isinstance(self.t_s, bool) or isinstance(self.force_n, bool):
            raise ValueError("waypoint fields must be numbers, not booleans")
        if not math.isfinite(self.t_s) or not math.isfinite(self.force_n):
            raise ValueError("waypoint fields must be finite")
        if self.t_s < 0.0:
            raise ValueError("waypoint time must be non-negative")
        if self.force_n < 0.0:
            raise ValueError("waypoint force must be non-negative")


def sample_force_reference(
    interpolation: ForceInterpolation,
    waypoints: Iterable[ForceWaypoint],
    time_s: float,
) -> tuple[float, float, float]:
    """对 waypoint 序列采样目标力及其一、二阶时间导数。

    任何提供 ``t_s`` 与 ``force_n`` 属性的对象都可以作为 waypoint，
    因此仿真侧的 Pydantic waypoint 可以直接复用本实现。

    Args:
        interpolation: 插值方式（hold／linear／smoothstep）。
        waypoints: 按时间升序的 waypoint 序列，至少两个且时间严格递增。
        time_s: 相对曲线起点的采样时刻；早于首点返回首点值，
            晚于末点保持末点值。

    Returns:
        ``(目标力 N, 一阶导 N/s, 二阶导 N/s²)``；阶跃区间不伪造有限
        冲击导数，导数为零表示解析导数不存在或恒为零。

    Raises:
        ValueError: waypoint 序列不满足数量或单调性要求，或插值方式未知。
    """
    points = tuple(waypoints)
    if len(points) < 2:
        raise ValueError("force reference requires at least two waypoints")
    previous = -math.inf
    for waypoint in points:
        if waypoint.t_s <= previous:
            raise ValueError("force reference waypoint times must be strictly increasing")
        previous = waypoint.t_s
    moment = max(0.0, float(time_s))
    if moment <= points[0].t_s:
        return (float(points[0].force_n), 0.0, 0.0)
    for start, end in zip(points, points[1:], strict=False):
        if moment <= end.t_s:
            if interpolation == "hold":
                return (float(start.force_n), 0.0, 0.0)
            span = end.t_s - start.t_s
            u = (moment - start.t_s) / span
            delta = float(end.force_n - start.force_n)
            rate = delta / span
            acceleration = 0.0
            if interpolation == "smoothstep":
                rate *= 6.0 * u * (1.0 - u)
                acceleration = delta * (6.0 - 12.0 * u) / span**2
                u = u * u * (3.0 - 2.0 * u)
            return (float(start.force_n + u * delta), rate, acceleration)
    return (float(points[-1].force_n), 0.0, 0.0)


@dataclass(frozen=True, slots=True)
class ForceReferenceCurve:
    """由 waypoint 定义的目标法向力曲线（共享核纯数据版本）。

    Attributes:
        interpolation: 插值方式。
        waypoints: 有序 waypoint 元组。
    """

    interpolation: ForceInterpolation = "smoothstep"
    waypoints: tuple[ForceWaypoint, ...] = ()

    def __post_init__(self) -> None:
        """在构造期完成与采样函数一致的序列校验。"""
        if self.interpolation not in {"hold", "linear", "smoothstep"}:
            raise ValueError(f"unknown force reference interpolation: {self.interpolation}")
        sample_force_reference(self.interpolation, self.waypoints, 0.0)

    @property
    def duration_s(self) -> float:
        """返回曲线持续时间（末 waypoint 时刻）。"""
        return float(self.waypoints[-1].t_s)

    def target_at(self, time_s: float) -> float:
        """返回指定时刻的目标力。"""
        return self.sample_at(time_s)[0]

    def sample_at(self, time_s: float) -> tuple[float, float, float]:
        """返回指定时刻的目标力及其一、二阶时间导数。"""
        return sample_force_reference(self.interpolation, self.waypoints, time_s)

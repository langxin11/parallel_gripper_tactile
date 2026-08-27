"""用于解耦控制与物理循环的仿真时间调度器。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass(slots=True)
class SimulationTimer:
    """针对仿真时间而非墙钟时间发出周期性嘀嗒。

    当控制更新到期时，对 :meth:`pop_due` 的一次调用会返回已过去的仿真时间，
    否则返回 ``None``。物理积分可能以比 ``period_s`` 更小的步长运行；调用方
    在嘀嗒之间保持其最新命令。当某个物理步跳过一个时间间隔时，此计时器
    不会在完全相同的仿真状态下多次运行控制器。
    """

    period_s: float
    start_time_s: float = 0.0
    _last_tick_time_s: float | None = field(default=None, init=False, repr=False)
    _next_tick_time_s: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """校验调度，并在 ``start_time_s`` 处启动其第一次嘀嗒。"""
        if not math.isfinite(self.period_s) or self.period_s <= 0:
            raise ValueError("period_s must be finite and positive")
        if not math.isfinite(self.start_time_s):
            raise ValueError("start_time_s must be finite")
        self._next_tick_time_s = self.start_time_s

    def pop_due(self, simulation_time_s: float) -> float | None:
        """当嘀嗒到期时返回已过去的仿真时间，否则返回 ``None``。"""
        if not math.isfinite(simulation_time_s):
            raise ValueError("simulation_time_s must be finite")
        if self._last_tick_time_s is not None and simulation_time_s < self._last_tick_time_s:
            raise ValueError("simulation time must not move backwards")
        tolerance_s = 1e-12
        if simulation_time_s + tolerance_s < self._next_tick_time_s:
            return None
        elapsed_s = (
            self.period_s
            if self._last_tick_time_s is None
            else simulation_time_s - self._last_tick_time_s
        )
        self._last_tick_time_s = simulation_time_s
        while self._next_tick_time_s <= simulation_time_s + tolerance_s:
            self._next_tick_time_s += self.period_s
        return elapsed_s


@dataclass(frozen=True, slots=True)
class RealtimePacer:
    """将墙钟时间映射到期望的仿真时间播放位置。"""

    realtime_factor: float = 1.0
    simulation_start_s: float = 0.0
    wall_start_s: float = 0.0

    def __post_init__(self) -> None:
        """拒绝无效的播放速率与时间原点。"""
        if not math.isfinite(self.realtime_factor) or self.realtime_factor <= 0:
            raise ValueError("realtime_factor must be finite and positive")
        if not math.isfinite(self.simulation_start_s) or not math.isfinite(self.wall_start_s):
            raise ValueError("time origins must be finite")

    def target_simulation_time(self, wall_time_s: float) -> float:
        """返回当前墙钟时间所请求的仿真时间戳。"""
        if not math.isfinite(wall_time_s):
            raise ValueError("wall_time_s must be finite")
        return self.simulation_start_s + self.realtime_factor * (wall_time_s - self.wall_start_s)

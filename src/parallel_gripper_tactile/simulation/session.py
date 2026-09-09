"""一个拥有单一所有权的、带解耦实验时钟的 MuJoCo 步进循环。"""

from __future__ import annotations

from collections.abc import Callable
import math
from typing import Protocol, TypeVar

import mujoco

from ..timing import SimulationTimer

SampleT = TypeVar("SampleT")


class Experiment(Protocol[SampleT]):
    """在每次物理步进中由 :class:`SimulationSession` 消费的钩子。"""

    def before_step(self, time_s: float) -> None:
        """在积分之前立即更新与阶段相关的世界状态。"""

    def control(self, dt_s: float) -> None:
        """仅当独立控制时钟走时更新控制量。"""

    def sample(self) -> SampleT:
        """当采样时钟走时，在积分步后返回一个采样。"""


class SimulationSession:
    """拥有一个模型/数据对，并独占地使用 ``mj_step`` 推进它。

    物理、控制与采样调度刻意彼此独立。采样总是在使其状态可用的物理步
    *之后* 采集，而 ``before_step`` 与控制则在步进前的时间执行。
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        control_period_s: float | None = None,
        sample_period_s: float | None = None,
    ) -> None:
        """创建会话，其控制与采样时钟独立于物理。"""
        physics_timestep_s = float(model.opt.timestep)
        if not math.isfinite(physics_timestep_s) or physics_timestep_s <= 0:
            raise ValueError("model.opt.timestep must be finite and positive")
        self.model = model
        self.data = data
        self.physics_timestep_s = physics_timestep_s
        control_period = physics_timestep_s if control_period_s is None else control_period_s
        sample_period = physics_timestep_s if sample_period_s is None else sample_period_s
        self._validate_period("control_period_s", control_period)
        self._validate_period("sample_period_s", sample_period)
        now = float(data.time)
        self._control_timer = SimulationTimer(control_period, now)
        self._sample_timer = SimulationTimer(sample_period, now)
        self.steps = 0

    def _validate_period(self, name: str, period_s: float) -> None:
        """拒绝无法在物理步边界上被观测到的调度。"""
        if not math.isfinite(period_s) or period_s <= 0:
            raise ValueError(f"{name} must be finite and positive")
        if period_s + 1e-12 < self.physics_timestep_s:
            raise ValueError(f"{name} must not be shorter than model.opt.timestep")

    def step(self, experiment: Experiment[SampleT]) -> SampleT | None:
        """恰好执行一个 MuJoCo 步，并返回到期的实验采样。"""
        time_s = float(self.data.time)
        experiment.before_step(time_s)
        control_dt_s = self._control_timer.pop_due(time_s)
        if control_dt_s is not None:
            experiment.control(control_dt_s)
        mujoco.mj_step(self.model, self.data)
        self.steps += 1
        if self._sample_timer.pop_due(float(self.data.time)) is not None:
            return experiment.sample()
        return None

    def run_steps(
        self,
        steps: int,
        experiment: Experiment[SampleT],
        *,
        on_sample: Callable[[SampleT], None] | None = None,
    ) -> list[SampleT]:
        """推进固定数量的物理步并收集到期采样。"""
        if steps < 0:
            raise ValueError("steps must be nonnegative")
        samples: list[SampleT] = []
        for _ in range(steps):
            sample = self.step(experiment)
            if sample is not None:
                samples.append(sample)
                if on_sample is not None:
                    on_sample(sample)
        return samples

    def advance_to(
        self,
        target_time_s: float,
        experiment: Experiment[SampleT],
        *,
        on_sample: Callable[[SampleT], None] | None = None,
    ) -> list[SampleT]:
        """推进直到下一个固定物理边界到达 ``target_time_s``。

        MuJoCo 使用固定积分步长，因此未与 ``model.opt.timestep`` 对齐的
        目标会在其之后的第一个边界处被到达。
        """
        if not math.isfinite(target_time_s):
            raise ValueError("target_time_s must be finite")
        current_time_s = float(self.data.time)
        if target_time_s + 1e-12 < current_time_s:
            raise ValueError("target_time_s must not precede the current simulation time")
        samples: list[SampleT] = []
        while float(self.data.time) + 1e-12 < target_time_s:
            sample = self.step(experiment)
            if sample is not None:
                samples.append(sample)
                if on_sample is not None:
                    on_sample(sample)
        return samples

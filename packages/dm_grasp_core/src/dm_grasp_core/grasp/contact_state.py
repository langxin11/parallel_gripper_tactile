"""DMgripper 双侧接触阶段的公共状态机。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from .motion import ContactTransition


ContactState = Literal["approach", "contact_transition", "force_tracking"]
ReleasePolicy = Literal["any_side", "both_sides"]


@dataclass(frozen=True, slots=True)
class BilateralContactConfig:
    """双侧接触确认、速度过渡与掉力恢复参数。"""

    contact_threshold_n: float
    contact_confirm_steps: int
    release_threshold_n: float
    release_confirm_steps: int
    contact_transition_time_s: float
    contact_stable_time_s: float = 0.0
    release_policy: ReleasePolicy = "any_side"

    def __post_init__(self) -> None:
        """拒绝非有限、倒置或无效的状态机参数。"""
        numeric = (
            self.contact_threshold_n,
            self.release_threshold_n,
            self.contact_transition_time_s,
            self.contact_stable_time_s,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("双侧接触状态机参数必须为有限数值")
        if self.contact_threshold_n <= 0.0:
            raise ValueError("contact_threshold_n must be positive")
        if not 0.0 <= self.release_threshold_n <= self.contact_threshold_n:
            raise ValueError("release_threshold_n must lie within contact threshold")
        if self.contact_confirm_steps <= 0 or self.release_confirm_steps <= 0:
            raise ValueError("contact and release confirmation steps must be positive")
        if self.contact_transition_time_s < 0.0 or self.contact_stable_time_s < 0.0:
            raise ValueError("contact transition and stable durations must be non-negative")
        if self.release_policy not in {"any_side", "both_sides"}:
            raise ValueError("release_policy must be any_side or both_sides")


@dataclass(frozen=True, slots=True)
class ContactStateUpdate:
    """一次状态机更新后的阶段与边沿事件。"""

    state: ContactState
    entered_transition: bool = False
    entered_tracking: bool = False
    reentered_approach: bool = False


class BilateralContactStateMachine:
    """为所有 DM 力控制律统一管理接触建立与丢失。"""

    def __init__(self, config: BilateralContactConfig) -> None:
        """保存冻结配置并初始化到接近阶段。"""
        self.config = config
        self.reset()

    def reset(self) -> None:
        """返回接近阶段并清除确认计数与速度过渡。"""
        self.state: ContactState = "approach"
        self._contact_steps = 0
        self._contact_started_s: float | None = None
        self._release_steps = 0
        self._transition_started_s = 0.0
        self._transition: ContactTransition | None = None

    def transition_velocity(self, now_s: float) -> float:
        """返回接触过渡阶段的期望关节速度。"""
        if self.state != "contact_transition" or self._transition is None:
            return 0.0
        return self._transition.velocity_at(now_s - self._transition_started_s)

    def _released(self, left_force_n: float, right_force_n: float) -> bool:
        """按配置策略判断本周期是否失去有效双侧接触。"""
        if self.config.release_policy == "any_side":
            return min(left_force_n, right_force_n) <= self.config.release_threshold_n
        return max(left_force_n, right_force_n) <= self.config.release_threshold_n

    def update(
        self,
        *,
        left_force_n: float,
        right_force_n: float,
        now_s: float,
        approach_velocity_rad_s: float,
    ) -> ContactStateUpdate:
        """消费一次原始双侧力观测并推进公共阶段。"""
        values = (left_force_n, right_force_n, now_s, approach_velocity_rad_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("双侧接触状态机输入必须为有限数值")
        config = self.config
        if self.state == "approach":
            if min(left_force_n, right_force_n) >= config.contact_threshold_n:
                self._contact_steps += 1
                if self._contact_started_s is None:
                    self._contact_started_s = now_s
            else:
                self._contact_steps = 0
                self._contact_started_s = None
            stable_duration = (
                0.0 if self._contact_started_s is None else now_s - self._contact_started_s
            )
            if (
                self._contact_steps >= config.contact_confirm_steps
                and stable_duration >= config.contact_stable_time_s
            ):
                self._contact_steps = 0
                self._contact_started_s = None
                if config.contact_transition_time_s == 0.0:
                    self.state = "force_tracking"
                    return ContactStateUpdate(self.state, entered_tracking=True)
                self.state = "contact_transition"
                self._transition_started_s = now_s
                self._transition = ContactTransition(
                    approach_velocity_rad_s,
                    config.contact_transition_time_s,
                )
                return ContactStateUpdate(self.state, entered_transition=True)
            return ContactStateUpdate(self.state)

        self._release_steps = (
            self._release_steps + 1 if self._released(left_force_n, right_force_n) else 0
        )
        if self._release_steps >= config.release_confirm_steps:
            self.reset()
            return ContactStateUpdate(self.state, reentered_approach=True)
        if (
            self.state == "contact_transition"
            and now_s - self._transition_started_s >= config.contact_transition_time_s
        ):
            self.state = "force_tracking"
            self._release_steps = 0
            return ContactStateUpdate(self.state, entered_tracking=True)
        return ContactStateUpdate(self.state)

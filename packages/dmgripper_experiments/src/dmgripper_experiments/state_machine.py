"""闭合接近与力跟踪状态机。"""

from __future__ import annotations

import math
from enum import StrEnum

from .config import ForceDemoConfig


class ForceTrackingState(StrEnum):
    """一次真机闭合实验的有限状态。"""

    PREPARE = "prepare"
    APPROACH = "approach"
    CONTACT_TRANSITION = "contact_transition"
    FORCE_TRACKING = "force_tracking"
    RETURN = "return"
    COMPLETE = "complete"
    FAULT = "fault"


class ForceTrackingStateMachine:
    """用双阈值与双向持续确认抑制接触边界抖动。"""

    def __init__(self, config: ForceDemoConfig) -> None:
        """创建处于准备状态的状态机。"""
        self.config = config
        self.state = ForceTrackingState.PREPARE
        self.reason = "尚未使能"
        self.entered_at_s = 0.0
        self._contact_started_s: float | None = None
        self._release_started_s: float | None = None

    def begin_approach(self, now_s: float, reason: str) -> None:
        """进入接近状态并重新累计稳定接触时间。"""
        self._enter(ForceTrackingState.APPROACH, now_s, reason)
        self._contact_started_s = None
        self._release_started_s = None

    def observe_forces(self, left_force_n: float, right_force_n: float, now_s: float) -> None:
        """检查力保护，并按持续接触或持续失接触推进状态。"""
        if not all(math.isfinite(value) for value in (left_force_n, right_force_n, now_s)):
            self.fault(now_s, "触觉力或时间不是有限数值")
            return
        if max(left_force_n, right_force_n) > self.config.force_ceiling_n:
            self.fault(now_s, "触觉力超过上限")
            return
        if self.state is ForceTrackingState.APPROACH:
            if min(left_force_n, right_force_n) >= self.config.contact_on_n:
                if self._contact_started_s is None:
                    self._contact_started_s = now_s
                elif now_s - self._contact_started_s >= self.config.contact_on_stable_s:
                    self._enter(ForceTrackingState.CONTACT_TRANSITION, now_s, "双侧稳定接触")
                    self._release_started_s = None
            else:
                self._contact_started_s = None
            return
        if self.state not in {
            ForceTrackingState.CONTACT_TRANSITION,
            ForceTrackingState.FORCE_TRACKING,
        }:
            return
        if abs(left_force_n - right_force_n) > self.config.max_force_imbalance_n:
            self.fault(now_s, "双侧触觉力差超过上限")
            return
        if max(left_force_n, right_force_n) <= self.config.contact_off_n:
            if self._release_started_s is None:
                self._release_started_s = now_s
            elif now_s - self._release_started_s >= self.config.contact_off_stable_s:
                self.begin_approach(now_s, "双侧持续失去接触")
        else:
            self._release_started_s = None

    def finish_contact_transition(self, now_s: float) -> None:
        """完成接触速度衰减并进入力跟踪。"""
        if self.state is not ForceTrackingState.CONTACT_TRANSITION:
            raise RuntimeError("仅接触过渡状态可以进入力跟踪")
        self._enter(ForceTrackingState.FORCE_TRACKING, now_s, "接触速度过渡完成")

    def begin_return(self, now_s: float, reason: str) -> None:
        """进入回位状态。"""
        self._enter(ForceTrackingState.RETURN, now_s, reason)

    def complete(self, now_s: float) -> None:
        """标记回位流程完成。"""
        self._enter(ForceTrackingState.COMPLETE, now_s, "回位完成")

    def fault(self, now_s: float, reason: str) -> None:
        """锁定故障状态。"""
        self._enter(ForceTrackingState.FAULT, now_s, reason)

    def _enter(self, state: ForceTrackingState, now_s: float, reason: str) -> None:
        """统一记录状态、进入时间和原因。"""
        if not math.isfinite(now_s):
            raise ValueError("状态时间必须是有限数值")
        self.state = state
        self.entered_at_s = now_s
        self.reason = reason

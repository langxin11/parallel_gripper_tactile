"""通用抓取实验的生命周期阶段与合法转换。

生命周期只拥有阶段切换与阶段进入时间；接触确认、稳定判定与命令生成
由运行时按新观测驱动，再调用这里的显式转换方法。任何非法转换都会
立即抛错，避免状态被静默改写。
"""

from __future__ import annotations

from enum import StrEnum

import math


class LifecyclePhase(StrEnum):
    """一次通用抓取实验的运行阶段。"""

    PREPARING = "preparing"
    READY = "ready"
    APPROACH = "approach"
    CONTACT_TRANSITION = "contact_transition"
    PRELOAD = "preload"
    ACTIVE = "active"
    HOLDING = "holding"
    RETURNING = "returning"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    FAULT = "fault"


TERMINAL_PHASES = frozenset(
    {
        LifecyclePhase.COMPLETED,
        LifecyclePhase.CANCELLED,
        LifecyclePhase.FAULT,
    }
)

_TRACKING_PHASES = frozenset(
    {
        LifecyclePhase.PRELOAD,
        LifecyclePhase.ACTIVE,
        LifecyclePhase.HOLDING,
    }
)


class Lifecycle:
    """记录当前阶段并强制执行通用生命周期转换表。

    转换表（箭头表示允许的显式转换）：

    ```text
    preparing → ready → approach ⇄ preload → active → holding
                       ↓            ↓           ↓        ↓
                    cancelled   returning ← release（active／holding）
    approach → contact_transition → preload
    任意非终态 → fault
    ```

    接触段编号在每次确认新的双侧接触时递增，用于对齐控制器、估计器与
    目标策略在同一接触边沿上的重置。
    """

    def __init__(self) -> None:
        """创建处于准备阶段的 lifecycle。"""
        self.phase = LifecyclePhase.PREPARING
        self.phase_started_s = 0.0
        self.reason = "配置已验证，等待预检"
        self.contact_segment = 0
        self.fault_reason: str | None = None

    @property
    def is_terminal(self) -> bool:
        """返回是否已进入终态。"""
        return self.phase in TERMINAL_PHASES

    @property
    def is_tracking(self) -> bool:
        """返回是否处于闭环抓握阶段（preload／active／holding）。"""
        return self.phase in _TRACKING_PHASES

    def elapsed_s(self, now_s: float) -> float:
        """返回当前阶段已经持续的时间。"""
        return now_s - self.phase_started_s

    def mark_ready(self, now_s: float, reason: str = "预检完成") -> None:
        """完成预检，进入等待启动的 ready 阶段。"""
        self._require(LifecyclePhase.PREPARING)
        self._enter(LifecyclePhase.READY, now_s, reason)

    def start(self, now_s: float) -> None:
        """响应用户 start，进入接近阶段。"""
        self._require(LifecyclePhase.READY)
        self._enter(LifecyclePhase.APPROACH, now_s, "收到 start，电机已确认使能")

    def cancel_before_enable(self, now_s: float) -> None:
        """使能前收到取消，直接进入 cancelled 终态。"""
        self._require(LifecyclePhase.READY)
        self._enter(LifecyclePhase.CANCELLED, now_s, "使能前收到取消，未发送运动命令")

    def confirm_contact(self, now_s: float) -> None:
        """双侧接触持续确认，进入速度过渡阶段并递增接触段编号。"""
        self._require(LifecyclePhase.APPROACH)
        self.contact_segment += 1
        self._enter(LifecyclePhase.CONTACT_TRANSITION, now_s, "双侧接触持续确认")

    def finish_contact_transition(self, now_s: float) -> None:
        """接近速度过渡完成，进入初始抓力稳定阶段。"""
        self._require(LifecyclePhase.CONTACT_TRANSITION)
        self._enter(LifecyclePhase.PRELOAD, now_s, "接触速度过渡完成")

    def activate(self, now_s: float) -> None:
        """初始抓力稳定达标，进入正式运行阶段。"""
        self._require(LifecyclePhase.PRELOAD)
        self._enter(LifecyclePhase.ACTIVE, now_s, "初始抓力稳定，目标策略启用")

    def finish_task(self, now_s: float) -> None:
        """任务计时完成，保持闭环抓握进入 holding。"""
        self._require(LifecyclePhase.ACTIVE)
        self._enter(LifecyclePhase.HOLDING, now_s, "任务计时完成，保持抓握")

    def begin_return(self, now_s: float, reason: str) -> None:
        """响应用户 release，进入受限张开回位。"""
        if self.phase not in {LifecyclePhase.ACTIVE, LifecyclePhase.HOLDING}:
            raise RuntimeError(f"阶段 {self.phase.value} 不允许开始回位")
        self._enter(LifecyclePhase.RETURNING, now_s, reason)

    def complete_return(self, now_s: float) -> None:
        """回位轨迹结束且实际位置达标，进入 completed 终态。"""
        self._require(LifecyclePhase.RETURNING)
        self._enter(LifecyclePhase.COMPLETED, now_s, "回位完成")

    def reenter_approach(self, now_s: float, reason: str) -> None:
        """失接触后按配置重新接近；接触段编号由下次确认递增。"""
        if not self.is_tracking:
            raise RuntimeError(f"阶段 {self.phase.value} 不允许重新接近")
        self._enter(LifecyclePhase.APPROACH, now_s, reason)

    def fault(self, now_s: float, reason: str) -> None:
        """进入故障终态并保留失败原因。"""
        if self.is_terminal:
            raise RuntimeError(f"终态 {self.phase.value} 不能再次转入故障")
        self._enter(LifecyclePhase.FAULT, now_s, reason)
        self.fault_reason = reason

    def _require(self, expected: LifecyclePhase) -> None:
        """校验当前阶段是转换的唯一来源。"""
        if self.phase is not expected:
            raise RuntimeError(
                f"生命周期转换非法：当前 {self.phase.value}，期望来源 {expected.value}"
            )

    def _enter(self, phase: LifecyclePhase, now_s: float, reason: str) -> None:
        """统一记录阶段、进入时间与原因。"""
        if not math.isfinite(now_s):
            raise ValueError("阶段时间必须是有限数值")
        self.phase = phase
        self.phase_started_s = now_s
        self.reason = reason

"""由采集线程执行的滑移检测会话与设备确认。"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading
from typing import Literal, Protocol

NativeSlipPhase = Literal["idle", "queued", "starting", "active", "stopping", "stopped", "failed"]


class SlipClient(Protocol):
    """仅向串口所有者暴露滑移启停命令。"""

    def start_slip_detection(self) -> None:
        """请求设备开始检测。"""

    def stop_slip_detection(self) -> None:
        """请求设备停止检测。"""


@dataclass(frozen=True, slots=True)
class NativeSlipStatus:
    """设备会话状态；发送停止与设备确认停止是不同状态。"""

    session_id: int = 0
    phase: NativeSlipPhase = "idle"
    reason: str = "not_requested"
    started_s: float | None = None
    stop_sent_s: float | None = None


class NativeSlipLease:
    """跨线程只提交请求；tick 和 close 必须由唯一串口所有者调用。"""

    def __init__(self) -> None:
        """初始化未启动会话与互斥锁。"""
        self._lock = threading.Lock()
        self._status = NativeSlipStatus()
        self._start: tuple[int, float | None, float] | None = None
        self._stop: str | None = None
        self._duration: float | None = None
        self._confirmation = 0.0
        self._may_be_active = False

    @property
    def status(self) -> NativeSlipStatus:
        """返回不可变状态快照。"""
        with self._lock:
            return self._status

    def request_start(
        self,
        session_id: int,
        *,
        max_duration_s: float | None,
        confirmation_timeout_s: float,
    ) -> None:
        """排队一个检测会话；None 表示仅接受显式停止。"""
        if isinstance(session_id, bool) or not isinstance(session_id, int) or session_id <= 0:
            raise ValueError("会话编号必须为正整数")
        if (
            isinstance(confirmation_timeout_s, bool)
            or not math.isfinite(confirmation_timeout_s)
            or confirmation_timeout_s <= 0
        ):
            raise ValueError("确认时限必须为正有限数")
        if max_duration_s is not None and (
            isinstance(max_duration_s, bool)
            or not math.isfinite(max_duration_s)
            or max_duration_s <= 0
        ):
            raise ValueError("检测租期必须为正有限数或 None")
        if max_duration_s is not None and confirmation_timeout_s >= max_duration_s:
            raise ValueError("确认时限必须小于检测租期")
        with self._lock:
            if self._status.phase not in {"idle", "stopped"}:
                raise RuntimeError("上一次检测尚未确认停止")
            if session_id <= self._status.session_id:
                raise ValueError("检测会话编号必须严格递增")
            self._start = (session_id, max_duration_s, confirmation_timeout_s)
            self._stop = None
            self._status = NativeSlipStatus(session_id, "queued", "waiting_owner")

    def request_stop(self, reason: str = "cancelled") -> None:
        """排队停止请求；停止优先于尚未发出的启动请求。"""
        with self._lock:
            if self._status.phase in {"queued", "starting", "active"}:
                self._stop = reason

    def read_timeout_s(self, now_s: float, default_s: float) -> float | None:
        """把单包等待预算缩短至下一个租期或确认期限。"""
        with self._lock:
            status = self._status
            if status.phase == "starting":
                assert status.started_s is not None
                deadline = status.started_s + self._confirmation
            elif status.phase == "active":
                assert status.started_s is not None
                if self._duration is None:
                    return None
                deadline = status.started_s + self._duration
            elif status.phase == "stopping":
                assert status.stop_sent_s is not None
                deadline = status.stop_sent_s + self._confirmation
            else:
                return None
            return max(1e-6, min(default_s, deadline - now_s))

    @property
    def shutdown_timeout_s(self) -> float:
        """退出等待需要包含设备停止确认预算。"""
        with self._lock:
            return self._confirmation

    def tick(
        self, client: SlipClient, now_s: float, active: tuple[bool, ...] | None = None
    ) -> None:
        """检查租期并处理请求；active 仅来自本次新读取的完整双侧状态。"""
        command = self._advance(now_s, active)
        # 串口写入或 flush 不持有状态锁，避免反馈查询阻塞电机控制线程。
        if command == "start":
            client.start_slip_detection()
        elif command == "stop":
            client.stop_slip_detection()

    def _advance(self, now_s: float, active: tuple[bool, ...] | None) -> str | None:
        """在短临界区内推进状态并返回串口动作。"""
        with self._lock:
            status = self._status
            if self._stop is not None and status.phase == "queued":
                self._start = None
                self._status = replace(status, phase="stopped", reason=self._stop)
                self._stop = None
                return
            if self._start is not None:
                _, self._duration, self._confirmation = self._start
                self._start = None
                # 写命令失败也可能已部分送达；退出时仍须尝试停止。
                self._may_be_active = True
                self._status = replace(
                    status, phase="starting", reason="awaiting_active", started_s=now_s
                )
                return "start"
            status = self._status
            if status.phase in {"starting", "active"}:
                assert status.started_s is not None
                elapsed = now_s - status.started_s
                reason = self._stop
                if self._duration is not None and elapsed >= self._duration:
                    reason = "duration_limit"
                elif status.phase == "starting" and elapsed >= self._confirmation:
                    reason = "start_unconfirmed"
                elif (
                    reason is None
                    and status.phase == "active"
                    and active is not None
                    and not all(active)
                ):
                    reason = "detector_inactive"
                if reason is not None:
                    self._stop = None
                    self._status = replace(
                        status, phase="stopping", reason=reason, stop_sent_s=now_s
                    )
                    # 此包先于停止请求读取，不能用作停止确认。
                    return "stop"
                if status.phase == "starting" and active == (True, True):
                    self._status = replace(status, phase="active", reason="confirmed_active")
            elif status.phase == "stopping":
                assert status.stop_sent_s is not None
                if active == (False, False):
                    self._may_be_active = False
                    self._status = replace(status, phase="stopped")
                elif now_s - status.stop_sent_s >= self._confirmation:
                    self._status = replace(status, phase="failed", reason="stop_unconfirmed")
                    raise RuntimeError("原厂滑移检测停止未获双侧设备确认")
        return None

    def close(self, client: SlipClient) -> None:
        """退出时尽力停用；无后续设备反馈时不伪造停止确认。"""
        with self._lock:
            stop_needed = self._may_be_active
            if stop_needed:
                if self._status.phase != "failed":
                    self._status = replace(
                        self._status, phase="failed", reason="closed_without_confirmation"
                    )
        if stop_needed:
            client.stop_slip_detection()

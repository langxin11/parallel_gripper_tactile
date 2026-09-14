"""通用抓取实验的终端展示：Rich／纯文本／JSON。

终端独立于控制线程消费不可变快照：控制线程只做非阻塞发布，显示
刷新合并或有界丢弃；UI 慢写或失败不阻塞控制，也不吞掉记录层的
数据写入。JSON 模式不夹杂 ANSI 或进度表。
"""

from __future__ import annotations

import json
import queue
import sys
import threading
import time
from dataclasses import dataclass, fields

from .lifecycle import LifecyclePhase

_MESSAGES = {
    LifecyclePhase.PREPARING: "正在预检：建立采集并验证空载零力。",
    LifecyclePhase.READY: ("预检完成。电机未使能；输入 start 后可能先自动回零，再开始闭合。"),
    LifecyclePhase.HOMING: "正在执行受限自动回零。",
    LifecyclePhase.APPROACH: "正在受限闭合接近，等待双侧接触。",
    LifecyclePhase.CONTACT_TRANSITION: "双侧接触已确认，正在平滑衰减接近速度。",
    LifecyclePhase.PRELOAD: "正在建立初始抓力并学习基线。",
    LifecyclePhase.ACTIVE: "目标策略已启用，运行中。",
    LifecyclePhase.HOLDING: "任务计时完成，保持抓握；输入 release 后按 Enter 结束。",
    LifecyclePhase.FAULT_HOLDING: "实验已失败并保持当前位置；承接物体后输入 release。",
    LifecyclePhase.RETURNING: "已收到 release，正在受限张开回位。",
    LifecyclePhase.COMPLETED: "回位完成，实验结束。",
    LifecyclePhase.CANCELLED: "使能前取消，未发送运动命令。",
    LifecyclePhase.FAULT: "发生故障，已执行退出清理。",
}


@dataclass(frozen=True, slots=True)
class RunSnapshot:
    """控制线程发布、终端消费的不可变运行快照。

    Attributes:
        phase: 当前生命周期阶段。
        phase_elapsed_s: 当前阶段持续时间。
        task_time_s: 任务时间（active 累计，holding 固定为结束值）。
        task_name: 任务显示名。
        object_name: 物体显示名。
        left_normal_n: 左侧滤波法向力。
        right_normal_n: 右侧滤波法向力。
        target_force_n: 当前受限目标力。
        tangential_force_n: 两侧切向合力模之和。
        stiffness_n_per_m: 刚度估计值；无估计为 ``None``。
        stiffness_valid: 刚度估计是否有效。
        stiffness_updated: 本周期刚度是否有新更新。
        aperture_m: 当前夹爪开度。
        position_limited: 命令是否触及位置限幅。
        control_dt_s: 本周期实际控制间隔。
        tactile_age_s: 触觉快照年龄。
        output_directory: 运行输出目录。
        force_deadband_active: 导纳死区是否生效。
        unloading_blocked: 导纳单向闭合是否阻止卸载。
    """

    phase: LifecyclePhase
    phase_elapsed_s: float
    task_time_s: float | None
    task_name: str
    object_name: str
    left_normal_n: float
    right_normal_n: float
    target_force_n: float
    tangential_force_n: float | None
    stiffness_n_per_m: float | None
    stiffness_valid: bool | None
    stiffness_updated: bool | None
    aperture_m: float | None
    position_limited: bool
    control_dt_s: float
    tactile_age_s: float
    output_directory: str
    force_deadband_active: bool = False
    unloading_blocked: bool = False

    def to_record(self) -> dict[str, object]:
        """返回可 JSON 序列化的普通记录。"""
        record: dict[str, object] = {}
        for item in fields(self):
            value = getattr(self, item.name)
            if isinstance(value, LifecyclePhase):
                value = value.value
            record[item.name] = value
        return record


class TerminalDisplay:
    """按配置模式展示运行快照；刷新线程拥有全部终端写入。

    Args:
        mode: ``auto``（兼容的交互终端用 Rich，其余用纯文本）、
            ``rich``、``plain`` 或 ``json``。
        refresh_hz: 显示刷新频率；不参与控制时钟。
        stream: 文本输出流（默认标准输出）。
    """

    def __init__(
        self,
        mode: str = "auto",
        *,
        refresh_hz: float = 5.0,
        stream=None,
    ) -> None:
        """创建显示；不启动线程。"""
        if mode not in {"auto", "rich", "plain", "json"}:
            raise ValueError("terminal.mode 必须是 auto、rich、plain 或 json")
        self._mode = mode
        self._refresh_hz = refresh_hz
        self._stream = stream if stream is not None else sys.stdout
        self._queue: queue.Queue[RunSnapshot | None] = queue.Queue(maxsize=64)
        self._latest: RunSnapshot | None = None
        self._events: list[str] = []
        self._event_revision = 0
        self._event_phase = ""
        self._event_kind = ""
        self._event_action = ""
        self._command_input = ""
        self._input_revision = 0
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stopped = threading.Event()
        self._rich_failed = False
        self._rich_console = None

    @property
    def uses_rich(self) -> bool:
        """返回实际是否会使用 Rich。"""
        if self._rich_failed:
            return False
        if self._mode == "json" or self._mode == "plain":
            return False
        if self._mode == "rich":
            return True
        try:
            if not bool(getattr(self._stream, "isatty", lambda: False)()):
                return False
            console = self._get_rich_console()
            return bool(console.is_terminal and not console.is_dumb_terminal)
        except Exception:  # noqa: BLE001
            return False

    def start(self) -> None:
        """启动刷新线程。"""
        if self._thread is not None:
            return
        self._stopped.clear()
        self._thread = threading.Thread(target=self._run, name="terminal-display", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """停止刷新线程并等待退出。"""
        self._stopped.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def publish(self, snapshot: RunSnapshot) -> None:
        """非阻塞发布最新快照；队列满时丢弃旧帧，绝不让控制等待。"""
        self._latest = snapshot
        try:
            self._queue.put_nowait(snapshot)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(snapshot)
            except queue.Full:
                pass

    def show_event(
        self,
        text: str,
        *,
        phase: str = "",
        event: str = "",
        action: str = "",
    ) -> None:
        """登记一条最近事件行，随快照一起展示。"""
        with self._lock:
            self._events.append(text)
            del self._events[:-5]
            self._event_revision += 1
            if phase:
                self._event_phase = phase
            if event:
                self._event_kind = event
            if action:
                self._event_action = action

    def set_command_input(self, text: str) -> None:
        """更新 Rich 面板底部的固定命令输入区。"""
        with self._lock:
            self._command_input = text
            self._input_revision += 1

    def _run(self) -> None:
        """刷新循环：按配置模式消费快照。"""
        if self._mode == "json":
            self._run_json()
            return
        if self.uses_rich and not self._rich_failed:
            try:
                self._run_rich()
                return
            except Exception as error:  # noqa: BLE001
                # Rich 失效时退化为纯文本，不中断控制。
                self._rich_failed = True
                self._write_line(f"[warning] Rich 终端显示不可用，已切换为纯文本：{error}")
                self._write_cached_events()
                self._write_latest_plain_snapshot()
        self._run_plain()

    def _run_json(self) -> None:
        """JSON 行模式：每帧一条快照记录，无 ANSI；按哨兵退出不丢帧。"""
        while True:
            snapshot = self._next_snapshot()
            if snapshot is None:
                break
            self._write_line(json.dumps(snapshot.to_record(), ensure_ascii=False, allow_nan=False))

    def _run_plain(self) -> None:
        """纯文本模式：按显示频率合并快照，退出前补写最后一帧。"""
        interval_s = 1.0 / self._refresh_hz
        next_write_s = 0.0
        pending: RunSnapshot | None = None
        while True:
            now_s = time.monotonic()
            timeout_s = 0.2 if pending is None else max(0.0, min(0.2, next_write_s - now_s))
            try:
                item = self._queue.get(timeout=timeout_s)
            except queue.Empty:
                item = None
                if self._stopped.is_set() and pending is None:
                    break
            if item is None and self._stopped.is_set():
                if pending is not None:
                    self._write_line(self._format_plain(pending))
                break
            if item is not None:
                pending = item
            now_s = time.monotonic()
            if pending is not None and now_s >= next_write_s:
                self._write_line(self._format_plain(pending))
                pending = None
                next_write_s = now_s + interval_s

    def _run_rich(self) -> None:
        """由单一 Live 区域同时维护状态面板与固定输入行。"""
        from rich.live import Live
        from rich.text import Text

        console = self._get_rich_console()
        rendered_event_revision = -1
        rendered_input_revision = -1
        latest_snapshot: RunSnapshot | None = None
        with Live(
            renderable=self._with_command_input(Text("正在初始化…")),
            console=console,
            auto_refresh=False,
            redirect_stdout=False,
            redirect_stderr=False,
            screen=False,
            transient=False,
            vertical_overflow="ellipsis",
        ) as live:
            while True:
                snapshot = self._drain_latest()
                if snapshot is not None:
                    latest_snapshot = snapshot
                with self._lock:
                    event_revision = self._event_revision
                    input_revision = self._input_revision
                if (
                    snapshot is not None
                    or event_revision != rendered_event_revision
                    or input_revision != rendered_input_revision
                ):
                    body = (
                        self._render_rich(latest_snapshot)
                        if latest_snapshot is not None
                        else self._render_preflight_events() or Text("正在初始化…")
                    )
                    live.update(self._with_command_input(body), refresh=True)
                    rendered_event_revision = event_revision
                    rendered_input_revision = input_revision
                if self._stopped.is_set():
                    break
                self._stopped.wait(1.0 / self._refresh_hz)

    def _with_command_input(self, body):
        """把状态内容与不受重绘干扰的底部输入行组合。"""
        from rich.console import Group
        from rich.text import Text

        with self._lock:
            command_input = self._command_input
        prompt = Text.assemble(("命令> ", "bold cyan"), command_input, ("█", "bold white"))
        return Group(body, prompt)

    def _get_rich_console(self):
        """返回用于能力判断与 Live 的同一个 Rich Console。"""
        if self._rich_console is None:
            from rich.console import Console

            self._rich_console = Console(file=self._stream)
        return self._rich_console

    def _render_preflight_events(self):
        """在尚无控制快照时渲染预检与告警事件。"""
        from rich.panel import Panel
        from rich.text import Text

        with self._lock:
            events = list(self._events)
            phase = self._event_phase
            event = self._event_kind
            action = self._event_action
        if not events:
            return None
        if event == "command_received" and action in {"start", "auto_start"}:
            title = "DMgripper 已收到命令"
            subtitle = "正在连接、检查并使能电机；请勿重复输入"
        elif phase == LifecyclePhase.READY.value:
            title = "DMgripper 等待命令"
            subtitle = "命令：s/start 开始 · status 状态 · r/release 取消（按 Enter）"
        elif phase == LifecyclePhase.CANCELLED.value:
            title = "DMgripper 已取消"
            subtitle = "使能前取消，未发送运动命令"
        elif phase == LifecyclePhase.COMPLETED.value:
            title = "DMgripper 已完成"
            subtitle = "实验已结束"
        elif phase == LifecyclePhase.FAULT.value:
            title = "DMgripper 故障"
            subtitle = "退出清理已尝试完成"
        elif phase == LifecyclePhase.HOMING.value:
            title = "DMgripper 自动回零"
            subtitle = "仅接受 status；紧急停止请按 Ctrl+C"
        elif phase == LifecyclePhase.FAULT_HOLDING.value:
            title = "DMgripper 故障保持"
            subtitle = "请承接物体后输入 release；紧急停止请按 Ctrl+C"
        elif phase == LifecyclePhase.PREPARING.value or not phase:
            title = "DMgripper 正在准备"
            subtitle = "请等待预检完成"
        else:
            title = "DMgripper 正在启动"
            subtitle = "请等待预检完成"
        return Panel(
            Text("\n".join(events[-3:])),
            title=title,
            subtitle=subtitle,
        )

    def _next_snapshot(self) -> RunSnapshot | None:
        """阻塞等待下一帧或停止哨兵。"""
        while True:
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                if self._stopped.is_set():
                    return None
                continue
            if item is None:
                return None
            return item

    def _drain_latest(self) -> RunSnapshot | None:
        """合并队列中的帧，只保留最新。"""
        latest = None
        try:
            while True:
                item = self._queue.get_nowait()
                if item is None:
                    self._stopped.set()
                    break
                latest = item
        except queue.Empty:
            pass
        return latest

    def _write_cached_events(self) -> None:
        """Rich 运行时失败后补写已经缓存的事件，避免 ready 阶段静默。"""
        with self._lock:
            events = list(self._events)
        for event in events:
            self._write_line(event)

    def _write_latest_plain_snapshot(self) -> None:
        """Rich 消费帧后失败时，以纯文本补写最后快照。"""
        snapshot = self._drain_latest() or self._latest
        if snapshot is not None:
            self._write_line(self._format_plain(snapshot))

    def _write_line(self, text: str) -> None:
        """写一行文本；写入失败时静默降级，不影响控制。"""
        try:
            self._stream.write(text + "\n")
            self._stream.flush()
        except Exception:  # noqa: BLE001
            pass

    def _format_plain(self, snapshot: RunSnapshot) -> str:
        """格式化单行文本摘要；普通终端也能区分状态。"""
        stiffness = (
            f"{snapshot.stiffness_n_per_m:.0f}N/m"
            if snapshot.stiffness_n_per_m is not None
            else "n/a"
        )
        valid = (
            ""
            if snapshot.stiffness_valid is None
            else ("+valid" if snapshot.stiffness_valid else "+initial")
        )
        task_time = "n/a" if snapshot.task_time_s is None else f"{snapshot.task_time_s:.1f}s"
        return (
            f"[{snapshot.phase.value}] t_task={task_time} "
            f"F_L={snapshot.left_normal_n:.3f}N F_R={snapshot.right_normal_n:.3f}N "
            f"F_target={snapshot.target_force_n:.3f}N "
            f"K={stiffness}{valid} dt={snapshot.control_dt_s * 1000:.1f}ms "
            f"tactile_age={snapshot.tactile_age_s * 1000:.0f}ms"
        )

    def _render_rich(self, snapshot: RunSnapshot):
        """构造 Rich 面板。"""
        from rich.panel import Panel
        from rich.table import Table
        from rich.text import Text

        table = Table.grid(padding=(0, 2))
        table.add_column(justify="right", style="cyan", no_wrap=True)
        table.add_column(no_wrap=True)
        title = Text(f"{snapshot.task_name} / {snapshot.object_name}", style="bold")
        phase_text = Text(snapshot.phase.value, style="bold magenta")
        message = _MESSAGES.get(snapshot.phase, "")
        table.add_row("阶段", f"{phase_text} +{snapshot.phase_elapsed_s:.1f}s  {message}")
        task_time = "n/a" if snapshot.task_time_s is None else f"{snapshot.task_time_s:.2f} s"
        table.add_row("任务时间", task_time)
        table.add_row(
            "法向力",
            f"L {snapshot.left_normal_n:+.3f} N   R {snapshot.right_normal_n:+.3f} N",
        )
        table.add_row("目标力", f"{snapshot.target_force_n:.3f} N")
        tangential = (
            "n/a" if snapshot.tangential_force_n is None else f"{snapshot.tangential_force_n:.3f} N"
        )
        table.add_row("切向力", tangential)
        if snapshot.stiffness_n_per_m is None:
            stiffness_text = "未启用"
        else:
            state = "有效" if snapshot.stiffness_valid else "初始"
            updated = " 新更新" if snapshot.stiffness_updated else ""
            stiffness_text = f"{snapshot.stiffness_n_per_m:.0f} N/m（{state}{updated}）"
        table.add_row("刚度估计", stiffness_text)
        aperture = "n/a" if snapshot.aperture_m is None else f"{snapshot.aperture_m * 1000:.1f} mm"
        table.add_row("开度", aperture)
        flags = []
        if snapshot.position_limited:
            flags.append("位置限幅")
        if snapshot.force_deadband_active:
            flags.append("死区")
        if snapshot.unloading_blocked:
            flags.append("单向闭合")
        table.add_row("限制状态", "、".join(flags) if flags else "无")
        table.add_row(
            "控制周期",
            f"{snapshot.control_dt_s * 1000:.1f} ms   触觉年龄 {snapshot.tactile_age_s * 1000:.0f} ms",
        )
        table.add_row("交互命令", _command_hint(snapshot.phase))
        table.add_row("运行目录", snapshot.output_directory)
        with self._lock:
            events = list(self._events)
        if events:
            table.add_row("最近事件", "\n".join(events[-3:]))
        return Panel(table, title=title, subtitle="DMgripper 实时控制快照")


def _command_hint(phase: LifecyclePhase) -> str:
    """返回与当前生命周期实际允许动作一致的命令提示。"""
    if phase in {
        LifecyclePhase.ACTIVE,
        LifecyclePhase.HOLDING,
        LifecyclePhase.FAULT_HOLDING,
    }:
        return "status 查看状态 · r/release 回位（按 Enter）"
    if phase in {
        LifecyclePhase.HOMING,
        LifecyclePhase.APPROACH,
        LifecyclePhase.CONTACT_TRANSITION,
        LifecyclePhase.PRELOAD,
    }:
        return "status 查看状态 · r/release 暂不可用 · 紧急停止 Ctrl+C"
    if phase is LifecyclePhase.RETURNING:
        return "正在回位 · status 查看状态 · 紧急停止 Ctrl+C"
    return "实验已结束"


def format_event_line(event: dict[str, object]) -> str:
    """把结构化事件格式化为单行文本，供终端与日志复用。"""
    parts = []
    for key in ("event", "state", "phase", "action"):
        if key in event and event[key] is not None:
            parts.append(str(event[key]))
    for key in ("message", "reason", "error"):
        if key in event and event[key] is not None:
            parts.append(str(event[key]))
    return " | ".join(parts) if parts else json.dumps(event, ensure_ascii=False)

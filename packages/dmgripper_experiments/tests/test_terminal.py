"""终端展示：快照记录、模式选择与非阻塞发布。"""

from __future__ import annotations

import json
import os
import time

import pytest

from dmgripper_experiments.lifecycle import LifecyclePhase
from dmgripper_experiments.terminal import RunSnapshot, TerminalDisplay, format_event_line


def _snapshot(**overrides) -> RunSnapshot:
    values = dict(
        phase=LifecyclePhase.ACTIVE,
        phase_elapsed_s=1.5,
        task_time_s=2.0,
        task_name="force-curve",
        object_name="cube",
        left_normal_n=0.52,
        right_normal_n=0.48,
        target_force_n=0.5,
        tangential_force_n=0.12,
        stiffness_n_per_m=1500.0,
        stiffness_valid=True,
        stiffness_updated=False,
        aperture_m=0.03,
        position_limited=False,
        control_dt_s=0.01,
        tactile_age_s=0.004,
        output_directory="/tmp/run",
    )
    values.update(overrides)
    return RunSnapshot(**values)


def test_snapshot_serializes_phase_as_plain_string():
    """快照 JSON 记录把阶段序列化为字符串。"""
    record = _snapshot().to_record()
    assert record["phase"] == "active"
    encoded = json.dumps(record, allow_nan=False)
    assert "force-curve" in encoded


def _wait_until(predicate, timeout_s: float = 1.0) -> None:
    """短时等待终端后台线程达到断言状态。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("终端后台线程未在时限内达到预期状态")


def test_mode_selection_auto_uses_rich_terminal_capability(
    monkeypatch: pytest.MonkeyPatch,
):
    """auto 仅在非 dumb 的真实终端启用 Rich。"""

    class FakeTty:
        def isatty(self) -> bool:
            return True

        def write(self, text: str) -> int:
            return len(text)

        def flush(self) -> None:
            return None

    class FakePipe:
        def isatty(self) -> bool:
            return False

        def write(self, _text: str) -> int:
            return 0

        def flush(self) -> None:
            return None

    monkeypatch.setenv("TERM", "xterm-256color")
    monkeypatch.setenv("FORCE_COLOR", "1")
    assert TerminalDisplay("auto", stream=FakePipe()).uses_rich is False
    monkeypatch.delenv("FORCE_COLOR")
    assert TerminalDisplay("auto", stream=FakeTty()).uses_rich is True
    assert TerminalDisplay("auto", stream=FakePipe()).uses_rich is False
    monkeypatch.setenv("TERM", "dumb")
    assert TerminalDisplay("auto", stream=FakeTty()).uses_rich is False
    assert TerminalDisplay("json", stream=FakeTty()).uses_rich is False
    assert TerminalDisplay("plain", stream=FakeTty()).uses_rich is False
    assert TerminalDisplay("rich", stream=FakePipe()).uses_rich is True


def test_publish_never_blocks_when_queue_full():
    """队列满时发布丢弃旧帧而不阻塞。"""
    display = TerminalDisplay("plain")
    for _ in range(200):
        display.publish(_snapshot())
    assert display._latest is not None  # noqa: SLF001  直接检查替身状态


def test_json_mode_emits_plain_lines_without_ansi():
    """JSON 模式逐帧输出无 ANSI 的记录行。"""
    import io

    stream = io.StringIO()
    display = TerminalDisplay("json", stream=stream)
    display.start()
    display.publish(_snapshot())
    display.publish(_snapshot(phase=LifecyclePhase.HOLDING))
    display.stop()
    lines = [line for line in stream.getvalue().splitlines() if line.strip()]
    assert len(lines) >= 2
    for line in lines:
        payload = json.loads(line)
        assert "\x1b" not in line
        assert payload["phase"] in {"active", "holding"}


def test_plain_mode_formats_phase_and_forces():
    """纯文本模式单行摘要包含阶段与关键力值。"""
    import io

    stream = io.StringIO()
    display = TerminalDisplay("plain", stream=stream)
    display.start()
    display.publish(_snapshot())
    display.stop()
    text = stream.getvalue()
    assert "[active]" in text
    assert "F_target=0.500N" in text


def test_plain_mode_coalesces_high_rate_snapshots() -> None:
    """纯文本回退遵守显示刷新率，不按控制频率刷屏。"""
    import io

    stream = io.StringIO()
    display = TerminalDisplay("plain", refresh_hz=5.0, stream=stream)
    display.start()
    for index in range(50):
        display.publish(_snapshot(target_force_n=index / 100.0))
    display.stop()
    lines = [line for line in stream.getvalue().splitlines() if line]
    assert 1 <= len(lines) <= 2
    assert "F_target=0.490N" in lines[-1]


def test_rich_preflight_panel_keeps_recent_warning_without_snapshot():
    """Rich 在控制快照前也要展示预检事件与零力告警。"""
    import io

    from rich.console import Console

    stream = io.StringIO()
    display = TerminalDisplay("rich", stream=stream)
    display.show_event("state | preparing | 正在预检", phase="preparing")
    display.show_event("warning | 检测到峰值")
    display.show_event("state | ready | 等待 start", phase="ready")
    panel = display._render_preflight_events()  # noqa: SLF001  验证无快照渲染契约
    assert panel is not None
    Console(file=stream, force_terminal=False, width=100).print(panel)
    rendered = stream.getvalue()
    assert "检测到峰值" in rendered
    assert "等待 start" in rendered
    assert "输入后按 Enter" in rendered


def test_rich_command_hint_matches_phase_permissions() -> None:
    """Rich 控制面板不得在不支持的阶段邀请用户输入 release。"""
    import io

    from rich.console import Console

    stream = io.StringIO()
    display = TerminalDisplay("rich", stream=stream)
    Console(file=stream, force_terminal=False, width=120).print(
        display._render_rich(_snapshot(phase=LifecyclePhase.APPROACH))  # noqa: SLF001
    )
    rendered = stream.getvalue()
    assert "release 暂不可用" in rendered

    stream.seek(0)
    stream.truncate(0)
    Console(file=stream, force_terminal=False, width=120).print(
        display._render_rich(_snapshot(phase=LifecyclePhase.HOLDING))  # noqa: SLF001
    )
    rendered = stream.getvalue()
    assert "release 回位" in rendered


def test_rich_preflight_title_tracks_command_and_cancellation() -> None:
    """start 确认和使能前取消使用各自状态标题，不再继续邀请输入。"""
    import io

    from rich.console import Console

    stream = io.StringIO()
    display = TerminalDisplay("rich", stream=stream)
    display.show_event(
        "command_received | ready | start | 已收到 start",
        phase="ready",
        event="command_received",
        action="start",
    )
    Console(file=stream, force_terminal=False, width=100).print(
        display._render_preflight_events()  # noqa: SLF001
    )
    assert "DMgripper 已收到命令" in stream.getvalue()
    assert "请勿重复输入" in stream.getvalue()

    stream.seek(0)
    stream.truncate(0)
    display.show_event("state | cancelled | 已取消", phase="cancelled", event="state")
    Console(file=stream, force_terminal=False, width=100).print(
        display._render_preflight_events()  # noqa: SLF001
    )
    assert "DMgripper 已取消" in stream.getvalue()


def test_rich_live_has_one_refresh_owner_and_idles_without_redraw(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """控制快照 Live 禁用内建刷新线程，内容未变化时不重复重画。"""
    import io

    import rich.live

    updates: list[object] = []
    options: dict[str, object] = {}

    class FakeLive:
        def __init__(self, **kwargs) -> None:
            options.update(kwargs)

        def __enter__(self):
            return self

        def __exit__(self, *_args) -> None:
            return None

        def update(self, renderable, *, refresh: bool = False) -> None:
            assert refresh is True
            updates.append(renderable)

    monkeypatch.setattr(rich.live, "Live", FakeLive)
    display = TerminalDisplay("rich", refresh_hz=50.0, stream=io.StringIO())
    display.show_event("state | ready | 等待 start", phase="ready")
    display.start()
    display.publish(_snapshot())
    _wait_until(lambda: len(updates) == 1)
    time.sleep(0.08)
    assert len(updates) == 1
    display.show_event("status | active | 仍在运行", phase="active")
    _wait_until(lambda: len(updates) == 2)
    display.stop()

    assert options["auto_refresh"] is False
    assert options["redirect_stdout"] is False
    assert options["redirect_stderr"] is False
    assert options["screen"] is False
    assert options["transient"] is False


def test_rich_failure_switches_property_and_replays_ready_event(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Rich 更新异常后标记为纯文本，并补写已缓存的 ready 提示。"""
    import io

    import rich.live

    class BrokenLive:
        def __init__(self, **_kwargs) -> None:
            pass

        def __enter__(self):
            raise OSError("模拟 Rich 输出失败")

        def __exit__(self, *_args) -> None:
            return None

    monkeypatch.setattr(rich.live, "Live", BrokenLive)
    stream = io.StringIO()
    display = TerminalDisplay("rich", stream=stream)
    display.show_event("state | ready | 输入 start 后按 Enter", phase="ready")
    display.start()
    display.publish(_snapshot())
    _wait_until(lambda: not display.uses_rich)
    display.stop()
    rendered = stream.getvalue()
    assert "已切换为纯文本" in rendered
    assert "输入 start 后按 Enter" in rendered


def test_rich_update_failure_replays_consumed_snapshot_as_plain(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Rich 消费快照后更新失败时，纯文本回退仍保留该快照。"""
    import io

    import rich.live

    class BrokenUpdateLive:
        def __init__(self, **_kwargs) -> None:
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args) -> None:
            return None

        def update(self, _renderable, *, refresh: bool = False) -> None:
            assert refresh is True
            raise OSError("模拟 Rich 更新失败")

    monkeypatch.setattr(rich.live, "Live", BrokenUpdateLive)
    stream = io.StringIO()
    display = TerminalDisplay("rich", refresh_hz=50.0, stream=stream)
    display.start()
    display.publish(_snapshot(target_force_n=0.73))
    _wait_until(lambda: not display.uses_rich)
    display.stop()
    rendered = stream.getvalue()
    assert "已切换为纯文本" in rendered
    assert "F_target=0.730N" in rendered


def test_rich_stop_flushes_an_event_without_waiting() -> None:
    """事件登记后立即停止时，Rich 仍执行最后一次渲染。"""
    import io

    stream = io.StringIO()
    display = TerminalDisplay("rich", stream=stream)
    display.show_event("command_received | ready | start | 已收到 start", phase="ready")
    display.start()
    display.stop()
    assert "已收到 start" in stream.getvalue()


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="需要 POSIX 伪终端")
def test_rich_ready_accepts_start_without_periodic_frame_appends(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """真实伪终端中 ready 可收 start，静止期间不会按刷新频率追加面板。"""
    from dmgripper_experiments import cli

    monkeypatch.setenv("TERM", "xterm-256color")
    master_fd, slave_fd = os.openpty()
    input_stream = os.fdopen(os.dup(slave_fd), "r", encoding="utf-8", buffering=1)
    output_stream = os.fdopen(os.dup(slave_fd), "w", encoding="utf-8", buffering=1)
    display = TerminalDisplay("auto", refresh_hz=20.0, stream=output_stream)
    try:
        assert display.uses_rich is True
        action_source = cli._action_source(input_stream)
        display.show_event(
            "state | ready | 输入 start 后按 Enter 开始闭合",
            phase="ready",
        )
        display.start()
        time.sleep(0.2)
        os.write(master_fd, b"start\n")
        deadline = time.monotonic() + 1.0
        action = None
        while time.monotonic() < deadline and action is None:
            action = action_source()
            time.sleep(0.005)
        assert action == "start"
        display.show_event(
            "command_received | ready | start | 已收到 start，正在检查电机",
            phase="ready",
            event="command_received",
            action="start",
        )
        time.sleep(0.1)
        display.stop()
        output_stream.flush()
        os.set_blocking(master_fd, False)
        chunks: list[bytes] = []
        while True:
            try:
                chunk = os.read(master_fd, 65_536)
            except BlockingIOError:
                break
            if not chunk:
                break
            chunks.append(chunk)
        rendered = b"".join(chunks).decode("utf-8", errors="replace")
        assert "输入 start 后按 Enter" in rendered
        assert "已收到 start，正在检查电机" in rendered
        assert rendered.count("DMgripper 等待命令") == 1
        assert rendered.count("DMgripper 已收到命令") == 1
    finally:
        display.stop()
        os.close(master_fd)
        os.close(slave_fd)
        input_stream.close()
        output_stream.close()


def test_stopped_display_swallows_publish_without_threads():
    """停止后的显示仍可安全接收发布（控制线程不感知 UI 状态）。"""
    display = TerminalDisplay("plain")
    display.stop()
    display.publish(_snapshot())


def test_format_event_line_combines_fields():
    """事件行格式化合并事件与消息字段。"""
    line = format_event_line({"event": "state", "phase": "active", "message": "运行中"})
    assert "state" in line and "active" in line and "运行中" in line

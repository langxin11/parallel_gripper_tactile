"""DMgripper 摇杆面板的输入互锁测试。"""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

from dmgripper_hardware.teleop_ui import run_panel


class _Snapshot:
    """提供固定状态的显示快照。"""

    def __init__(self, state: str = "ready") -> None:
        """保存控制器状态。"""
        self.state = state
        self.position_rad = 0.12
        self.velocity_rad_s = -0.03
        self.torque_nm = 0.08
        self.target_rad = 0.15
        self.feedforward_torque_nm = 0.06
        self.motion_phase = "holding"
        self.command = SimpleNamespace(
            position_rad=0.15,
            velocity_rad_s=0.0,
            kp=2.0,
            kd=0.5,
            feedforward_torque_nm=0.06,
        )
        self.axis = 0.0
        self.error = None


class _Controller:
    """记录面板命令的控制器替身。"""

    def __init__(self, state: str = "ready") -> None:
        """初始化状态和调用记录。"""
        self.state = state
        self.config = SimpleNamespace(
            deadzone=0.12, max_speed_rad_s=0.2, feedforward_force_n=1.0, opening_force_n=1.0
        )
        self.demo = True
        self.axes: list[float] = []
        self.enables = 0
        self.disables = 0
        self.shutdown_called = False

    def snapshot(self) -> _Snapshot:
        """返回当前快照。"""
        return _Snapshot(self.state)

    def set_axis(self, raw_axis: float) -> None:
        """记录帧心跳。"""
        self.axes.append(raw_axis)

    def enable(self) -> None:
        """记录使能请求。"""
        self.enables += 1

    def disable(self) -> None:
        """记录失能请求。"""
        self.disables += 1

    def shutdown(self) -> None:
        """记录关闭请求。"""
        self.shutdown_called = True


class _StateSequenceController(_Controller):
    """按帧返回状态序列，并记录失能发生时机。"""

    def __init__(self, states: list[str]) -> None:
        """保存预定状态。"""
        super().__init__(states[0])
        self._states = states
        self.snapshot_count = 0
        self.disable_at_snapshots: list[int] = []

    def snapshot(self) -> _Snapshot:
        """返回下一帧状态。"""
        self.snapshot_count += 1
        index = min(self.snapshot_count - 1, len(self._states) - 1)
        return _Snapshot(self._states[index])

    def disable(self) -> None:
        """记录失能请求对应的状态帧。"""
        super().disable()
        self.disable_at_snapshots.append(self.snapshot_count)


class _RenderedText:
    """满足绘制文本所需的最小对象。"""

    def get_rect(self, *, center: tuple[int, int]) -> SimpleNamespace:
        """返回可居中的占位矩形。"""
        return SimpleNamespace(center=center)


class _Font:
    """满足布局和渲染所需的最小字体。"""

    def render(self, text: str, antialias: bool, color: tuple[int, int, int]) -> _RenderedText:
        """返回可绘制的占位文本。"""
        del text, antialias, color
        return _RenderedText()

    def size(self, text: str) -> tuple[int, int]:
        """提供固定字符宽度以支持截短与换行。"""
        return len(text) * 10, 20

    def get_linesize(self) -> int:
        """返回固定行高。"""
        return 20


class _Surface:
    """满足绘制所需的最小画布。"""

    def fill(self, color: tuple[int, int, int]) -> None:
        """接受背景色。"""
        del color

    def blit(self, rendered: _RenderedText, pos: object) -> None:
        """接受已渲染文字。"""
        del rendered, pos


class _Rect:
    """仅实现面板点击所需的矩形。"""

    def __init__(self, x: int, y: int, width: int, height: int) -> None:
        """保存边界。"""
        self.x, self.y, self.width, self.height = x, y, width, height
        self.center = x + width // 2, y + height // 2

    def collidepoint(self, point: tuple[int, int]) -> bool:
        """判断点是否位于矩形中。"""
        return (
            self.x <= point[0] < self.x + self.width and self.y <= point[1] < self.y + self.height
        )


class _Joystick:
    """可逐帧提供轴值的假手柄。"""

    def __init__(self) -> None:
        """初始化默认纵轴。"""
        self.axis = 0.0
        self.axes = 2
        self.identity = 7

    def init(self) -> None:
        """满足 pygame 接口。"""

    def get_instance_id(self) -> int:
        """返回稳定实例编号。"""
        return self.identity

    def get_numaxes(self) -> int:
        """返回可用轴数量。"""
        return self.axes

    def get_axis(self, index: int) -> float:
        """返回选定纵轴。"""
        assert index == 1
        return self.axis

    def get_name(self) -> str:
        """返回可显示的设备名。"""
        return "Fake stick"


class _Clock:
    """不会实际等待的时钟。"""

    def tick(self, rate: int) -> None:
        """接受帧率参数。"""
        del rate


class _FakePygame:
    """驱动面板循环的最小 pygame 替身。"""

    QUIT, KEYDOWN, MOUSEBUTTONDOWN, WINDOWFOCUSLOST, WINDOWFOCUSGAINED, JOYDEVICEREMOVED = range(
        1, 7
    )
    K_SPACE = 32

    class error(Exception):
        """模拟 pygame.error。"""

    def __init__(self, frames: list[tuple[float, list[SimpleNamespace]]]) -> None:
        """接收每帧的轴值和事件。"""
        self._frames = frames
        self._joystick = _Joystick()
        self.event = SimpleNamespace(get=self._get_events)
        self.joystick = SimpleNamespace(
            init=lambda: None,
            get_count=lambda: 1,
            Joystick=lambda index: self._joystick,
        )
        self.font = SimpleNamespace(match_font=lambda name: None, Font=lambda path, size: _Font())
        self.display = SimpleNamespace(
            set_mode=lambda size: _Surface(), set_caption=lambda title: None, flip=lambda: None
        )
        self.draw = SimpleNamespace(rect=lambda *args, **kwargs: None)
        self.time = SimpleNamespace(Clock=_Clock)
        self.Rect = _Rect

    def _get_events(self) -> list[SimpleNamespace]:
        """切换到下一帧。"""
        if not self._frames:
            return [SimpleNamespace(type=self.QUIT)]
        self._joystick.axis, events = self._frames.pop(0)
        return events

    def init(self) -> None:
        """满足 pygame 接口。"""

    def quit(self) -> None:
        """满足 pygame 接口。"""


def _event(event_type: int, **attrs: object) -> SimpleNamespace:
    """构造带任意属性的假事件。"""
    return SimpleNamespace(type=event_type, **attrs)


def test_axis_is_continuous_heartbeat_and_default_direction(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """纵轴负值直接作为打开方向的连续心跳。"""
    pygame = _FakePygame([(0.0, []), (-0.6, []), (-0.6, [_event(_FakePygame.QUIT)])])
    monkeypatch.setitem(sys.modules, "pygame", pygame)
    controller = _Controller()

    run_panel(controller)

    assert -0.6 in controller.axes
    assert controller.shutdown_called


def test_enable_requires_connected_centered_focused_gamepad(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """仅连接、聚焦且回中的手柄能触发使能。"""
    pygame = _FakePygame(
        [
            (0.4, [_event(_FakePygame.MOUSEBUTTONDOWN, button=1, pos=(40, 640))]),
            (0.0, [_event(_FakePygame.MOUSEBUTTONDOWN, button=1, pos=(40, 640))]),
            (0.0, [_event(_FakePygame.QUIT)]),
        ]
    )
    monkeypatch.setitem(sys.modules, "pygame", pygame)
    controller = _Controller("connected")

    run_panel(controller)

    assert controller.enables == 1


def test_connected_enabling_ready_does_not_cancel_valid_enable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """连接、使能中、就绪的正常过渡不会被界面误判为安全取消。"""
    pygame = _FakePygame(
        [
            (0.0, [_event(_FakePygame.MOUSEBUTTONDOWN, button=1, pos=(40, 640))]),
            (0.0, []),
            (-0.5, []),
            (-0.5, [_event(_FakePygame.QUIT)]),
        ]
    )
    monkeypatch.setitem(sys.modules, "pygame", pygame)
    controller = _StateSequenceController(["connected", "enabling", "ready", "ready"])

    run_panel(controller)

    assert controller.enables == 1
    assert -0.5 in controller.axes
    assert all(frame >= 4 for frame in controller.disable_at_snapshots)


def test_enabling_with_deflected_stick_requests_disable(monkeypatch: pytest.MonkeyPatch) -> None:
    """使能尚未完成时摇杆偏转必须取消，不得以零轴心跳掩盖。"""
    pygame = _FakePygame(
        [
            (0.0, [_event(_FakePygame.MOUSEBUTTONDOWN, button=1, pos=(40, 640))]),
            (-0.5, []),
            (-0.5, [_event(_FakePygame.QUIT)]),
        ]
    )
    monkeypatch.setitem(sys.modules, "pygame", pygame)
    controller = _StateSequenceController(["connected", "enabling", "enabling"])

    run_panel(controller)

    assert -0.5 not in controller.axes
    assert 2 in controller.disable_at_snapshots


def test_stop_cancels_same_frame_axis_before_any_nonzero_heartbeat(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """STOP 与非零轴同帧到达时只能提交零输入。"""
    pygame = _FakePygame(
        [
            (0.0, []),
            (-0.8, [_event(_FakePygame.MOUSEBUTTONDOWN, button=1, pos=(560, 640))]),
            (0.0, [_event(_FakePygame.QUIT)]),
        ]
    )
    monkeypatch.setitem(sys.modules, "pygame", pygame)
    controller = _Controller()

    run_panel(controller)

    assert -0.8 not in controller.axes
    assert controller.disables >= 2


def test_space_disables_ready_controller_before_nonzero_axis(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """就绪状态下按物理空格的事件优先于该帧轴值。"""
    pygame = _FakePygame(
        [
            (0.0, []),
            (-0.8, [_event(_FakePygame.KEYDOWN, key=_FakePygame.K_SPACE)]),
            (0.0, [_event(_FakePygame.QUIT)]),
        ]
    )
    monkeypatch.setitem(sys.modules, "pygame", pygame)
    controller = _Controller()

    run_panel(controller)

    assert -0.8 not in controller.axes
    assert controller.disables >= 2


def test_focus_recovery_requires_recenter_before_enable(monkeypatch: pytest.MonkeyPatch) -> None:
    """失焦恢复后，未回中不能以点击重新使能。"""
    pygame = _FakePygame(
        [
            (0.0, []),
            (0.0, [_event(_FakePygame.WINDOWFOCUSLOST)]),
            (
                0.4,
                [
                    _event(_FakePygame.WINDOWFOCUSGAINED),
                    _event(_FakePygame.MOUSEBUTTONDOWN, button=1, pos=(40, 640)),
                ],
            ),
            (0.0, [_event(_FakePygame.MOUSEBUTTONDOWN, button=1, pos=(40, 640))]),
            (0.0, [_event(_FakePygame.QUIT)]),
        ]
    )
    monkeypatch.setitem(sys.modules, "pygame", pygame)
    controller = _Controller("connected")

    run_panel(controller)

    assert controller.enables == 1
    assert controller.disables >= 2


def test_illegal_axis_configuration_disables_and_shuts_down() -> None:
    """非法配置在导入 pygame 前仍会失能并关闭控制器。"""
    controller = _Controller()

    with pytest.raises(TypeError):
        run_panel(controller, axis_index=True)

    assert controller.axes == [0.0]
    assert controller.disables == 1
    assert controller.shutdown_called

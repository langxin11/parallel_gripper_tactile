"""遥控面板的输入互锁与退出回归测试。"""

from __future__ import annotations

import sys
from types import SimpleNamespace

from robotiq_hardware.teleop_ui import run_panel


class _Snapshot:
    """提供固定的 ready 状态。"""

    state = "ready"
    position = 128
    error = None


class _Controller:
    """记录面板提交的控制器替身。"""

    def __init__(self) -> None:
        """初始化调用记录。"""
        self.directions: list[int] = []
        self.cancellations = 0
        self.shutdown_called = False

    def activate(self) -> None:
        """满足面板协议。"""

    def set_direction(self, direction: int) -> None:
        """记录一帧方向。"""
        self.directions.append(direction)

    def cancel_motion(self) -> None:
        """记录安全取消。"""
        self.cancellations += 1

    def shutdown(self) -> None:
        """记录关闭调用。"""
        self.shutdown_called = True

    def snapshot(self) -> _Snapshot:
        """返回固定快照。"""
        return _Snapshot()


class _RenderedText:
    """最小化的已渲染文字对象。"""

    def get_rect(self, *, center: tuple[int, int]) -> SimpleNamespace:
        """返回支持居中定位的占位矩形。"""
        return SimpleNamespace(center=center)


class _Font:
    """支持面板布局计算的假字体。"""

    def render(self, text: str, antialias: bool, color: tuple[int, int, int]) -> _RenderedText:
        """返回可放入假画布的文字对象。"""
        del text, antialias, color
        return _RenderedText()

    def size(self, text: str) -> tuple[int, int]:
        """以固定字符宽度模拟布局。"""
        return len(text) * 10, 20

    def get_linesize(self) -> int:
        """返回固定行高。"""
        return 20


class _Surface:
    """最小化的假画布。"""

    def fill(self, color: tuple[int, int, int]) -> None:
        """接受背景填充。"""
        del color

    def blit(self, rendered: _RenderedText, position: object) -> None:
        """接受渲染对象。"""
        del rendered, position


class _Rect:
    """仅实现按钮点击判断所需的矩形。"""

    def __init__(self, x: int, y: int, width: int, height: int) -> None:
        """保存矩形边界。"""
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.center = x + width // 2, y + height // 2

    def collidepoint(self, point: tuple[int, int]) -> bool:
        """判断坐标是否位于矩形内。"""
        return (
            self.x <= point[0] < self.x + self.width and self.y <= point[1] < self.y + self.height
        )


class _Joystick:
    """可在每一帧替换按键集合的假手柄。"""

    def __init__(self) -> None:
        """初始化空按键集合。"""
        self.pressed: set[int] = set()

    def init(self) -> None:
        """满足 pygame 接口。"""

    def get_instance_id(self) -> int:
        """返回稳定的设备实例编号。"""
        return 7

    def get_numbuttons(self) -> int:
        """声明四个可读按钮。"""
        return 4

    def get_button(self, button: int) -> bool:
        """读取当前帧的按钮状态。"""
        return button in self.pressed


class _Clock:
    """不等待的时钟。"""

    def tick(self, rate: int) -> None:
        """接受帧率限制。"""
        del rate


class _FakePygame:
    """驱动面板事件循环所需的 pygame 最小替身。"""

    QUIT = 1
    KEYDOWN = 2
    KEYUP = 3
    MOUSEBUTTONDOWN = 4
    MOUSEBUTTONUP = 5
    WINDOWFOCUSLOST = 6
    WINDOWFOCUSGAINED = 7
    JOYDEVICEREMOVED = 8
    K_SPACE = 32

    class error(Exception):
        """模拟 pygame.error。"""

    def __init__(self, frames: list[tuple[set[int], bool, list[SimpleNamespace]]]) -> None:
        """接收每帧的手柄、空格与事件状态。"""
        self._frames = frames
        self._joystick = _Joystick()
        self._space_down = False
        self.event = SimpleNamespace(get=self._get_events)
        self.joystick = SimpleNamespace(
            init=lambda: None,
            get_count=lambda: 1,
            Joystick=lambda index: self._joystick,
        )
        self.key = SimpleNamespace(get_pressed=self._get_pressed)
        self.font = SimpleNamespace(match_font=lambda name: None, Font=lambda path, size: _Font())
        self.display = SimpleNamespace(
            set_mode=lambda size: _Surface(),
            set_caption=lambda title: None,
            flip=lambda: None,
        )
        self.draw = SimpleNamespace(rect=lambda *args, **kwargs: None)
        self.time = SimpleNamespace(Clock=_Clock)
        self.Rect = _Rect

    def _get_events(self) -> list[SimpleNamespace]:
        """切换到下一帧的输入状态。"""
        if not self._frames:
            return [SimpleNamespace(type=self.QUIT)]
        buttons, self._space_down, events = self._frames.pop(0)
        self._joystick.pressed = buttons
        return events

    def _get_pressed(self) -> list[bool]:
        """返回可按索引读取的键盘状态。"""
        states = [False] * (self.K_SPACE + 1)
        states[self.K_SPACE] = self._space_down
        return states

    def init(self) -> None:
        """满足 pygame 接口。"""

    def quit(self) -> None:
        """满足 pygame 接口。"""


def _event(event_type: int, **attributes: object) -> SimpleNamespace:
    """创建一个带任意 pygame 事件属性的替身。"""
    return SimpleNamespace(type=event_type, **attributes)


def test_conflicting_joystick_buttons_latch_until_every_button_is_released(monkeypatch) -> None:
    """同一手柄的冲突按钮不会放行鼠标输入。"""
    fake_pygame = _FakePygame([])
    fake_pygame._frames = [
        (set(), False, []),
        ({0, 1}, False, [_event(fake_pygame.MOUSEBUTTONDOWN, button=1, pos=(40, 480))]),
        (set(), False, [_event(fake_pygame.MOUSEBUTTONUP, button=1)]),
        ({0}, False, []),
        ({0}, False, [_event(fake_pygame.QUIT)]),
    ]
    monkeypatch.setitem(sys.modules, "pygame", fake_pygame)
    controller = _Controller()

    run_panel(controller)

    assert -1 in controller.directions
    assert controller.cancellations >= 3
    assert controller.shutdown_called


def test_focus_loss_clears_stale_space_state_before_motion_resumes(monkeypatch) -> None:
    """失焦时丢失 KEYUP 后，重新聚焦仍能在释放后恢复操作。"""
    fake_pygame = _FakePygame([])
    fake_pygame._frames = [
        (set(), False, []),
        (set(), True, [_event(fake_pygame.KEYDOWN, key=fake_pygame.K_SPACE)]),
        (set(), False, [_event(fake_pygame.WINDOWFOCUSLOST)]),
        (set(), False, [_event(fake_pygame.WINDOWFOCUSGAINED)]),
        (set(), False, [_event(fake_pygame.MOUSEBUTTONDOWN, button=1, pos=(40, 480))]),
        (set(), False, [_event(fake_pygame.QUIT)]),
    ]
    monkeypatch.setitem(sys.modules, "pygame", fake_pygame)
    controller = _Controller()

    run_panel(controller)

    assert -1 in controller.directions
    assert controller.cancellations >= 4


def test_quit_stops_before_processing_prior_motion_event(monkeypatch) -> None:
    """QUIT 与运动事件同帧到达时不会再发送非零方向。"""
    fake_pygame = _FakePygame([])
    fake_pygame._frames = [
        (set(), False, []),
        (
            set(),
            False,
            [
                _event(fake_pygame.MOUSEBUTTONDOWN, button=1, pos=(40, 480)),
                _event(fake_pygame.QUIT),
            ],
        ),
    ]
    monkeypatch.setitem(sys.modules, "pygame", fake_pygame)
    controller = _Controller()

    run_panel(controller)

    assert controller.directions
    assert set(controller.directions) == {0}


def test_short_click_submits_edge_before_same_frame_release(monkeypatch) -> None:
    """按钮的同帧按下和松开仍登记一个单步边沿。"""
    fake_pygame = _FakePygame([])
    fake_pygame._frames = [
        (set(), False, []),
        (
            set(),
            False,
            [
                _event(fake_pygame.MOUSEBUTTONDOWN, button=1, pos=(40, 480)),
                _event(fake_pygame.MOUSEBUTTONUP, button=1),
            ],
        ),
        (set(), False, [_event(fake_pygame.QUIT)]),
    ]
    monkeypatch.setitem(sys.modules, "pygame", fake_pygame)
    controller = _Controller()

    run_panel(controller)

    assert controller.directions[:3] == [0, -1, 0]
    assert controller.cancellations >= 2


def test_safety_event_cancels_same_frame_click_without_submitting_edge(monkeypatch) -> None:
    """同帧 STOP 会取消点击，不能在安全事件后留下待发单步。"""
    fake_pygame = _FakePygame([])
    fake_pygame._frames = [
        (set(), False, []),
        (
            set(),
            False,
            [
                _event(fake_pygame.MOUSEBUTTONDOWN, button=1, pos=(40, 480)),
                _event(fake_pygame.MOUSEBUTTONUP, button=1),
                _event(fake_pygame.MOUSEBUTTONDOWN, button=1, pos=(530, 480)),
            ],
        ),
        (set(), False, [_event(fake_pygame.QUIT)]),
    ]
    monkeypatch.setitem(sys.modules, "pygame", fake_pygame)
    controller = _Controller()

    run_panel(controller)

    assert -1 not in controller.directions
    assert controller.cancellations >= 3


def test_tap_conflicting_with_held_joystick_is_cancelled(monkeypatch) -> None:
    """同帧短点击与反向手柄冲突时，不得登记点击边沿。"""
    fake_pygame = _FakePygame([])
    fake_pygame._frames = [
        (set(), False, []),
        (
            {1},
            False,
            [
                _event(fake_pygame.MOUSEBUTTONDOWN, button=1, pos=(40, 480)),
                _event(fake_pygame.MOUSEBUTTONUP, button=1),
            ],
        ),
        (set(), False, [_event(fake_pygame.QUIT)]),
    ]
    monkeypatch.setitem(sys.modules, "pygame", fake_pygame)
    controller = _Controller()

    run_panel(controller)

    assert -1 not in controller.directions
    assert 1 not in controller.directions
    assert controller.cancellations >= 3

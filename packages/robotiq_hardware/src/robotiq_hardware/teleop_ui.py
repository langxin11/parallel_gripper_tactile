"""Robotiq USB 真机遥控的 pygame 本地面板。"""

from __future__ import annotations

from typing import Protocol


class _SnapshotLike(Protocol):
    """面板显示所需的控制器状态快照。"""

    state: str
    position: int | None
    error: str | None


class _TeleopController(Protocol):
    """遥控面板使用的最小控制器协议。"""

    def activate(self) -> None:
        """异步提交夹爪激活请求。"""

    def set_direction(self, direction: int) -> None:
        """提交一帧方向心跳。"""

    def cancel_motion(self) -> None:
        """取消待发动作并请求安全停止。"""

    def shutdown(self) -> None:
        """停止并关闭控制器。"""

    def snapshot(self) -> _SnapshotLike:
        """返回当前控制器状态。"""


def _cancel_motion(controller: _TeleopController) -> None:
    """请求安全停止，并兼容尚未提供新接口的旧替身。"""
    cancel = getattr(controller, "cancel_motion", None)
    if callable(cancel):
        cancel()
    else:
        controller.set_direction(0)


_CHINESE_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Serif CJK SC",
    "Source Han Sans SC",
    "WenQuanYi Zen Hei",
    "Microsoft YaHei",
)


def _find_chinese_font(pygame: object) -> str | None:
    """查找能显示中文的系统字体。

    Args:
        pygame: 延迟导入后的 pygame 模块。

    Returns:
        可用字体路径；未找到时返回 `None`。
    """
    for name in _CHINESE_FONT_CANDIDATES:
        path = pygame.font.match_font(name)
        if path is not None:
            return path
    return None


def _draw_text(
    surface: object,
    font: object,
    text: str,
    color: tuple[int, int, int],
    position: tuple[int, int],
) -> None:
    """在面板上绘制一行左对齐文字。"""
    surface.blit(font.render(text, True, color), position)


def _draw_wrapped_text(
    surface: object,
    font: object,
    text: str,
    color: tuple[int, int, int],
    position: tuple[int, int],
    *,
    max_width: int,
    max_lines: int | None = None,
) -> int:
    """绘制不超过指定宽度的文字，并返回末行下方的纵坐标。"""
    x, y = position
    lines: list[str] = []
    for paragraph in text.split("\n"):
        line = ""
        for word in paragraph.split(" "):
            candidate = word if not line else f"{line} {word}"
            if font.size(candidate)[0] <= max_width:
                line = candidate
                continue
            if line:
                lines.append(line)
            line = word
            while font.size(line)[0] > max_width:
                split_at = 1
                while split_at < len(line) and font.size(line[: split_at + 1])[0] <= max_width:
                    split_at += 1
                lines.append(line[:split_at])
                line = line[split_at:]
        lines.append(line)
    surface_height = getattr(surface, "get_height", lambda: 2**31 - 1)()
    line_height = font.get_linesize()
    if max_lines is not None:
        surface_height = min(surface_height, y + max_lines * line_height)
    for index, line in enumerate(lines):
        if y + line_height > surface_height:
            break
        if index + 1 < len(lines) and y + 2 * line_height > surface_height:
            visible = line
            while visible and font.size(visible + "…")[0] > max_width:
                visible = visible[:-1]
            line = visible + "…"
        _draw_text(surface, font, line, color, (x, y))
        y += line_height
    return y


def _button(
    pygame: object,
    surface: object,
    rect: object,
    label: str,
    font: object,
    *,
    enabled: bool,
    pressed: bool,
    color: tuple[int, int, int],
) -> None:
    """绘制一个带禁用与按下状态的面板按钮。"""
    if not enabled:
        fill = (36, 44, 57)
    elif pressed:
        fill = tuple(max(channel - 35, 0) for channel in color)
    else:
        fill = color
    pygame.draw.rect(surface, fill, rect, border_radius=8)
    pygame.draw.rect(surface, (62, 78, 99), rect, width=1, border_radius=8)
    text = font.render(label, True, (239, 244, 251) if enabled else (112, 127, 147))
    surface.blit(text, text.get_rect(center=rect.center))


def _selected_joystick(pygame: object, index: int, current: object | None) -> object | None:
    """取得当前编号对应的已初始化手柄，并处理拔出后的旧对象。"""
    count = pygame.joystick.get_count()
    if not 0 <= index < count:
        return None
    try:
        candidate = pygame.joystick.Joystick(index)
        candidate.init()
    except pygame.error:
        return None
    if current is not None:
        try:
            if current.get_instance_id() == candidate.get_instance_id():
                return current
        except pygame.error:
            pass
    return candidate


def _fit_text(font, text: str, width: int) -> str:
    """截短设备名等单行文本，避免覆盖相邻信息。"""
    if font.size(text)[0] <= width:
        return text
    while text and font.size(text + "…")[0] > width:
        text = text[:-1]
    return text + "…"


def _render_panel(
    pygame,
    screen,
    fonts,
    *,
    chinese,
    demo,
    snapshot,
    joystick_name,
    open_button,
    close_button,
    pressed_buttons,
    enabled,
    direction,
    release_required,
    focused,
    open_rect,
    close_rect,
    stop_rect,
    activate_rect,
):
    """绘制设备控制台；只读取状态，不产生任何控制命令。"""
    ink, muted = (232, 237, 245), (144, 158, 179)
    blue, green, amber, red = (100, 164, 255), (78, 204, 157), (243, 183, 85), (255, 119, 125)
    title, body, small, number = fonts
    state = snapshot.state

    def tr(zh, en):
        """按实际字体能力选择界面语言。"""
        return zh if chinese else en

    def text(value, x, y, font=body, color=ink):
        """在当前画布绘制一行文本。"""
        _draw_text(screen, font, value, color, (x, y))

    def card(x, y, width, height, color=(26, 34, 47)):
        """绘制统一圆角信息区。"""
        pygame.draw.rect(screen, color, pygame.Rect(x, y, width, height), border_radius=12)

    screen.fill((16, 22, 32))
    text("Robotiq 2F85", 32, 23, title)
    text(tr("夹爪控制台", "GRIPPER CONTROL"), 34, 62, small, muted)
    card(568, 26, 160, 28, (48, 42, 28) if demo else (27, 44, 66))
    text(
        tr("模拟 · 无硬件动作", "DEMO · NO HARDWARE")
        if demo
        else tr("真机 · USB / RS485", "LIVE · USB / RS485"),
        579,
        30,
        small,
        amber if demo else blue,
    )
    states = {
        "disconnected": (tr("连接中", "Connecting"), muted),
        "connected": (tr("已连接 · 待激活", "Awaiting activation"), blue),
        "activating": (tr("激活中", "Activating"), amber),
        "ready": (tr("可操作", "Ready"), green),
        "fault": (tr("设备故障", "Device fault"), red),
        "closed": (tr("已断开", "Disconnected"), muted),
    }
    state_label, state_color = states.get(state, (tr("未知状态", "Unknown state"), amber))
    if state == "ready" and (release_required or not focused):
        state_label, state_color = tr("输入暂停", "Input paused"), amber
    text(state_label, 568, 65, small, state_color)

    card(32, 112, 696, 72)
    name = joystick_name or tr("未连接手柄 · 可使用鼠标", "No gamepad · mouse available")
    text(_fit_text(body, name, 480), 50, 124)
    pressed = ", ".join(map(str, pressed_buttons)) or "—"
    text(_fit_text(small, tr("按键：", "Pressed: ") + pressed, 158), 550, 126, small, blue)
    mapping = tr(
        f"手柄映射   打开 {open_button}  /  闭合 {close_button}",
        f"Gamepad mapping   Open {open_button}  /  Close {close_button}",
    )
    text(mapping, 50, 153, small, muted)

    card(32, 204, 696, 166)
    text(tr("当前位置", "CURRENT POSITION"), 52, 216, small, muted)
    position = snapshot.position
    target = getattr(snapshot, "target", None)
    text("—" if position is None else f"{position:03d}", 48, 242, number)
    text(tr("目标位置", "TARGET POSITION"), 320, 216, small, muted)
    text("—" if target is None else f"{target:03d}", 320, 259, title, blue)
    text(tr("位置编码  0–255", "POSITION CODE  0–255"), 514, 222, small, muted)
    text(tr("非毫米值", "Not millimeters"), 514, 250, small, muted)
    card(52, 330, 656, 6, (45, 56, 73))
    if position is not None:
        fraction = max(0, min(255, position)) / 255
        if fraction:
            card(52, 330, max(4, int(656 * fraction)), 6, blue)
    if target is not None:
        marker = 52 + int(652 * max(0, min(255, target)) / 255)
        pygame.draw.rect(screen, amber, pygame.Rect(marker, 326, 3, 14), border_radius=1)
    text(tr("打开  0", "OPEN  0"), 52, 344, small, muted)
    text(tr("255  闭合", "255  CLOSED"), 624, 344, small, muted)

    if state == "connected":
        _button(
            pygame,
            screen,
            activate_rect,
            tr("激活夹爪", "Activate gripper"),
            body,
            enabled=True,
            pressed=False,
            color=(47, 104, 186),
        )
        hint = tr(
            "激活会产生开闭运动。\n请先清空夹爪运动范围。",
            "Activation moves the fingers.\nKeep the gripper's travel clear.",
        )
        _draw_wrapped_text(screen, small, hint, amber, (306, 397), max_width=406, max_lines=2)
    else:
        card(32, 390, 696, 60, (23, 43, 38) if enabled else (32, 37, 47))
        if state == "ready":
            hint = (
                tr("已就绪 · 短按微调，长按开闭", "Ready · tap to nudge, hold to move")
                if enabled
                else tr(
                    "请释放全部输入并回到窗口后继续",
                    "Release all inputs and focus this window to resume",
                )
            )
        elif state == "activating":
            hint = tr(
                "正在激活，请保持运动范围畅通", "Activating · keep the gripper's travel clear"
            )
        elif state == "fault":
            hint = tr("控制已禁用，请检查下方错误信息", "Controls disabled · check the error below")
        else:
            hint = tr("等待设备连接", "Waiting for device connection")
        text(_fit_text(body, hint, 650), 52, 408, color=green if enabled else amber)

    _button(
        pygame,
        screen,
        open_rect,
        tr("打开  −", "Open  −"),
        title,
        enabled=enabled,
        pressed=enabled and direction == -1,
        color=(39, 70, 100),
    )
    _button(
        pygame,
        screen,
        close_rect,
        tr("闭合  +", "Close  +"),
        title,
        enabled=enabled,
        pressed=enabled and direction == 1,
        color=(45, 99, 154),
    )
    _button(
        pygame,
        screen,
        stop_rect,
        tr("停止", "STOP"),
        title,
        enabled=True,
        pressed=False,
        color=(146, 49, 60),
    )
    text(
        tr(
            "短按一步 · 长按 0.4 秒后连续运动 · 松开停止",
            "Tap: one step · Hold 0.4 s: continuous · Release: stop",
        ),
        34,
        565,
        small,
        muted,
    )
    text(tr("空格停止", "Space: stop"), 630, 565, small, red)
    if snapshot.error:
        card(32, 602, 696, 82, (48, 28, 36))
        _draw_wrapped_text(
            screen,
            small,
            tr("错误：", "Error: ") + snapshot.error,
            red,
            (48, 610),
            max_width=664,
            max_lines=3,
        )
    else:
        text(
            tr(
                "短按的一步可完成；停止按钮会取消待发动作。",
                "A tap can finish its step. STOP cancels pending motion.",
            ),
            34,
            609,
            small,
            muted,
        )


def run_panel(
    controller: _TeleopController,
    *,
    open_button: int = 0,
    close_button: int = 1,
    joystick_index: int = 0,
) -> None:
    """运行 Robotiq 真机遥控面板，直到用户关闭窗口。

    短按“打开”或“闭合”会移动一步；保持按住 0.4 秒后连续运动，松开会停止连续
    运动。可选手柄的两个按钮默认分别对应打开和闭合。面板不会自动激活夹爪，且在
    焦点、连接或激活状态变化后，必须先释放所有输入才会重新允许运动。

    Args:
        controller: 已启动后台线程的 Robotiq 遥控控制器。
        open_button: 手柄打开按钮编号。
        close_button: 手柄闭合按钮编号。
        joystick_index: 使用的 pygame 手柄编号。

    Raises:
        TypeError: 手柄按钮编号或手柄编号不是非负整数。
        ValueError: 打开与闭合使用了相同的手柄按钮编号。
    """
    if (
        isinstance(open_button, bool)
        or not isinstance(open_button, int)
        or open_button < 0
        or isinstance(close_button, bool)
        or not isinstance(close_button, int)
        or close_button < 0
        or isinstance(joystick_index, bool)
        or not isinstance(joystick_index, int)
        or joystick_index < 0
    ):
        raise TypeError("joystick indices must be non-negative integers, not bool")
    if open_button == close_button:
        raise ValueError("open_button and close_button must differ")

    # pygame 仅是可选的本地界面依赖；导入本模块不能要求真机环境安装它。
    import pygame

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((760, 710))
    pygame.display.set_caption("Robotiq 2F85 · 夹爪控制台")
    clock = pygame.time.Clock()
    chinese_font = _find_chinese_font(pygame)
    is_chinese = chinese_font is not None
    fonts = tuple(pygame.font.Font(chinese_font, size) for size in (26, 20, 16, 64))
    open_rect = pygame.Rect(32, 468, 220, 82)
    close_rect = pygame.Rect(268, 468, 220, 82)
    stop_rect = pygame.Rect(520, 468, 208, 82)
    activate_rect = pygame.Rect(32, 390, 250, 60)
    running = True
    focused = True
    mouse_direction = 0
    mouse_left_down = False
    space_down = False
    release_required = True
    joystick: object | None = None
    joystick_identity: int | None = None
    previous_state: str | None = None

    try:
        while running:
            cancel_requested = False
            new_joystick = _selected_joystick(pygame, joystick_index, joystick)
            try:
                new_identity = new_joystick.get_instance_id() if new_joystick is not None else None
            except pygame.error:
                new_joystick = None
                new_identity = None
            if new_identity != joystick_identity:
                # 手柄接入、拔出或换成另一个设备时，不能继续沿用先前的按键状态。
                release_required = True
                cancel_requested = True
                joystick_identity = new_identity
            joystick = new_joystick

            snapshot = controller.snapshot()
            state = snapshot.state
            if state != previous_state:
                if previous_state is not None:
                    # 连接、激活和故障恢复都要求操作者重新确认输入已释放。
                    release_required = True
                    mouse_direction = 0
                if state != "ready":
                    cancel_requested = True
                previous_state = state

            mouse_tap_direction = 0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    cancel_requested = True
                    running = False
                    break
                if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    space_down = True
                    release_required = True
                    cancel_requested = True
                elif event.type == pygame.KEYUP and event.key == pygame.K_SPACE:
                    space_down = False
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    mouse_left_down = True
                    if open_rect.collidepoint(event.pos):
                        mouse_direction = -1
                        mouse_tap_direction = -1
                    elif close_rect.collidepoint(event.pos):
                        mouse_direction = 1
                        mouse_tap_direction = 1
                    elif stop_rect.collidepoint(event.pos):
                        mouse_direction = 0
                        release_required = True
                        cancel_requested = True
                    elif activate_rect.collidepoint(event.pos) and state == "connected":
                        cancel_requested = True
                        release_required = True
                        mouse_direction = 0
                        controller.activate()
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    mouse_direction = 0
                    mouse_left_down = False
                elif event.type == getattr(pygame, "WINDOWFOCUSLOST", -1):
                    focused = False
                    mouse_direction = 0
                    mouse_left_down = False
                    space_down = False
                    release_required = True
                    cancel_requested = True
                elif event.type == getattr(pygame, "WINDOWFOCUSGAINED", -1):
                    focused = True
                    mouse_direction = 0
                    release_required = True
                    cancel_requested = True
                elif event.type == getattr(pygame, "JOYDEVICEREMOVED", -1):
                    release_required = True
                    joystick = None
                    joystick_identity = None
                    cancel_requested = True

            if not running:
                _cancel_motion(controller)
                break
            if focused:
                # 失焦期间可能收不到 KEYUP；重新聚焦后以当前物理状态为准。
                space_down = bool(pygame.key.get_pressed()[pygame.K_SPACE])

            joystick_direction = 0
            pressed_buttons: list[int] = []
            joystick_conflict = False
            if joystick is not None:
                try:
                    pressed_buttons = [
                        button
                        for button in range(joystick.get_numbuttons())
                        if joystick.get_button(button)
                    ]
                except pygame.error:
                    joystick = None
                    joystick_identity = None
                    release_required = True
                    cancel_requested = True
                joystick_open = open_button in pressed_buttons
                joystick_close = close_button in pressed_buttons
                joystick_conflict = joystick_open and joystick_close
                if joystick_open != joystick_close:
                    joystick_direction = -1 if joystick_open else 1

            inputs = (mouse_direction, joystick_direction)
            tap_conflict = mouse_tap_direction != 0 and joystick_direction == -mouse_tap_direction
            has_conflict = joystick_conflict or tap_conflict or (-1 in inputs and 1 in inputs)
            raw_direction = 0 if has_conflict else mouse_direction or joystick_direction
            if has_conflict:
                release_required = True
                cancel_requested = True
            # 映射外按键也必须释放，避免输入状态变化后意外恢复运动。
            all_released = (
                mouse_direction == 0
                and not pressed_buttons
                and not mouse_left_down
                and not space_down
            )
            if all_released:
                release_required = False

            enabled = focused and state == "ready" and not release_required and not space_down
            direction = raw_direction if enabled else 0
            if cancel_requested:
                _cancel_motion(controller)
                # 输入已确认释放时补发零心跳，解除控制器的安全锁存；有按住输入时保持锁存。
                if all_released:
                    controller.set_direction(0)
            else:
                # 同帧按下并松开会使最终方向为零，先登记按下沿以免丢失单击。
                if mouse_tap_direction and direction == 0 and enabled:
                    controller.set_direction(mouse_tap_direction)
                controller.set_direction(direction)

            joystick_name = None
            if joystick is not None:
                try:
                    joystick_name = getattr(joystick, "get_name", lambda: "Gamepad")()
                except pygame.error:
                    joystick_name = "Gamepad"
            _render_panel(
                pygame,
                screen,
                fonts,
                chinese=is_chinese,
                demo=getattr(controller, "demo", False),
                snapshot=snapshot,
                joystick_name=joystick_name,
                open_button=open_button,
                close_button=close_button,
                pressed_buttons=pressed_buttons,
                enabled=enabled,
                direction=direction,
                release_required=release_required,
                focused=focused,
                open_rect=open_rect,
                close_rect=close_rect,
                stop_rect=stop_rect,
                activate_rect=activate_rect,
            )
            pygame.display.flip()
            clock.tick(60)
    finally:
        # 窗口异常关闭时取消未发点动，再交给控制器完成后台线程收束。
        try:
            _cancel_motion(controller)
        finally:
            controller.shutdown()
            pygame.quit()

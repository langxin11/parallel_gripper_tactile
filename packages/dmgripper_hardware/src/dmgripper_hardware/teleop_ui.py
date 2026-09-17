"""DMgripper 摇杆遥控的 pygame 本地面板。"""

from __future__ import annotations

import math
from typing import Protocol


class _SnapshotLike(Protocol):
    """面板显示所需的控制器状态快照。"""

    state: str
    position_rad: float | None
    velocity_rad_s: float | None
    torque_nm: float | None
    target_rad: float | None
    feedforward_torque_nm: float | None
    command: object | None
    motion_phase: str
    axis: float
    error: str | None


class _TeleopConfig(Protocol):
    """面板显示所需的遥控配置。"""

    deadzone: float
    max_speed_rad_s: float
    feedforward_force_n: float
    opening_force_n: float


class _TeleopController(Protocol):
    """遥控面板使用的最小控制器协议。"""

    config: _TeleopConfig
    demo: bool

    def enable(self) -> None:
        """异步提交使能请求。"""

    def set_axis(self, raw_axis: float) -> None:
        """提交一帧摇杆心跳。"""

    def disable(self) -> None:
        """异步提交显式失能请求。"""

    def shutdown(self) -> None:
        """失能并收束控制器。"""

    def snapshot(self) -> _SnapshotLike:
        """返回当前控制器状态。"""


_CHINESE_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Serif CJK SC",
    "Source Han Sans SC",
    "WenQuanYi Zen Hei",
    "Microsoft YaHei",
)


def _find_chinese_font(pygame: object) -> str | None:
    """查找能显示中文的系统字体。"""
    for name in _CHINESE_FONT_CANDIDATES:
        path = pygame.font.match_font(name)
        if path is not None:
            return path
    return None


def _draw_text(
    surface: object, font: object, text: str, color: tuple[int, int, int], pos: tuple[int, int]
) -> None:
    """在面板上绘制左对齐文字。"""
    surface.blit(font.render(text, True, color), pos)


def _fit_text(font: object, text: str, width: int) -> str:
    """将单行设备名截短到给定宽度。"""
    if font.size(text)[0] <= width:
        return text
    while text and font.size(text + "…")[0] > width:
        text = text[:-1]
    return text + "…"


def _draw_wrapped_text(
    surface: object,
    font: object,
    text: str,
    color: tuple[int, int, int],
    pos: tuple[int, int],
    *,
    max_width: int,
    max_lines: int,
) -> None:
    """按字符换行绘制有限行数的诊断文字。"""
    lines: list[str] = []
    remaining = text
    while remaining and len(lines) < max_lines:
        end = 1
        while end <= len(remaining) and font.size(remaining[:end])[0] <= max_width:
            end += 1
        line = remaining[: end - 1]
        remaining = remaining[end - 1 :]
        if remaining and len(lines) + 1 == max_lines:
            line = _fit_text(font, line + "…", max_width)
            remaining = ""
        lines.append(line)
    x, y = pos
    line_height = font.get_linesize()
    for line in lines:
        _draw_text(surface, font, line, color, (x, y))
        y += line_height


def _button(
    pygame: object,
    surface: object,
    rect: object,
    label: str,
    font: object,
    *,
    enabled: bool,
    color: tuple[int, int, int],
) -> None:
    """绘制一个带禁用状态的按钮。"""
    fill = color if enabled else (25, 34, 47)
    pygame.draw.rect(surface, fill, rect, border_radius=14)
    pygame.draw.rect(surface, (65, 84, 107), rect, width=1, border_radius=14)
    rendered = font.render(label, True, (245, 248, 252) if enabled else (158, 174, 194))
    surface.blit(rendered, rendered.get_rect(center=rect.center))


def _selected_joystick(pygame: object, index: int, current: object | None) -> object | None:
    """取得当前编号对应的已初始化手柄。"""
    if not 0 <= index < pygame.joystick.get_count():
        return None
    try:
        candidate = pygame.joystick.Joystick(index)
        candidate.init()
        if current is not None and current.get_instance_id() == candidate.get_instance_id():
            return current
        return candidate
    except pygame.error:
        return None


def _valid_index(value: object) -> bool:
    """检查手柄编号是否为非负整数。"""
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _safe_disable(controller: _TeleopController) -> None:
    """先提交零输入，再请求失能，保证安全帧没有非零轴值。"""
    controller.set_axis(0.0)
    controller.disable()


def _render_panel(
    pygame: object,
    screen: object,
    fonts: tuple[object, object, object],
    *,
    chinese: bool,
    controller: _TeleopController,
    snapshot: _SnapshotLike,
    joystick_name: str | None,
    raw_axis: float | None,
    centered: bool,
    focused: bool,
    axis_valid: bool,
    input_ready: bool,
    invert_axis: bool,
    enable_rect: object,
    stop_rect: object,
) -> None:
    """绘制只读摇杆面板。"""
    title, body, small = fonts
    ink, muted = (236, 241, 248), (153, 170, 190)
    blue, green, amber, red = (111, 174, 255), (92, 213, 186), (244, 189, 107), (255, 133, 143)
    panel, border = (21, 30, 43), (37, 51, 69)

    def tr(zh: str, en: str) -> str:
        return zh if chinese else en

    def text(
        value: str, x: int, y: int, font: object = body, color: tuple[int, int, int] = ink
    ) -> None:
        _draw_text(screen, font, value, color, (x, y))

    def right(value: str, edge: int, y: int, color: tuple[int, int, int] = muted) -> None:
        """按实际字体宽度右对齐单位和辅助信息。"""
        text(value, edge - small.size(value)[0], y, small, color)

    def card(
        x: int,
        y: int,
        width: int,
        height: int,
        color: tuple[int, int, int] = panel,
        *,
        outlined: bool = True,
    ) -> None:
        """绘制圆角面板，细边框用于区分背景层次。"""
        rect = pygame.Rect(x, y, width, height)
        pygame.draw.rect(screen, color, rect, border_radius=14)
        if outlined:
            pygame.draw.rect(screen, border, rect, width=1, border_radius=14)

    def number(value: float | None, *, digits: int = 4, signed: bool = True) -> str:
        """数值保留统一精度，未知反馈不显示为零。"""
        if value is None:
            return "—"
        sign = "+" if signed else ""
        return f"{value:{sign}.{digits}f}"

    state_labels = {
        "disconnected": (tr("未连接", "Disconnected"), muted),
        "connected": (tr("待使能", "Awaiting enable"), blue),
        "enabling": (tr("使能中", "Enabling"), amber),
        "ready": (tr("已使能", "Enabled"), green),
        "fault": (tr("设备故障", "Device fault"), red),
        "closed": (tr("已关闭", "Closed"), muted),
    }
    state_label, state_color = state_labels.get(snapshot.state, (tr("未知状态", "Unknown"), amber))
    if snapshot.state == "ready" and not focused:
        state_label, state_color = tr("输入暂停", "Input paused"), amber

    screen.fill((11, 17, 26))
    card(32, 27, 48, 48, (28, 54, 83), outlined=False)
    # 两侧指爪的几何标记与产品对应，不依赖外部图片或字体图标。
    for x in (43, 63):
        pygame.draw.rect(screen, blue, pygame.Rect(x, 38, 6, 26), border_radius=2)
    pygame.draw.rect(screen, blue, pygame.Rect(43, 58, 12, 6), border_radius=2)
    pygame.draw.rect(screen, blue, pygame.Rect(57, 58, 12, 6), border_radius=2)
    text("DMgripper", 96, 15, title)
    text(tr("DM4310P · 手柄控制", "DM4310P · GAMEPAD CONTROL"), 98, 62, small, muted)
    card(664, 30, 184, 42, (35, 31, 27) if controller.demo else (22, 42, 64), outlined=False)
    text(
        tr("模拟 · 离线", "DEMO · OFFLINE") if controller.demo else "USB2CAN · LIVE",
        679,
        36,
        small,
        amber if controller.demo else blue,
    )
    card(860, 30, 148, 42, (22, 37, 47), outlined=False)
    text(_fit_text(small, state_label, 122), 873, 36, small, state_color)
    pygame.draw.rect(screen, border, pygame.Rect(32, 96, 976, 1))

    card(32, 112, 976, 94)
    text(tr("手柄", "GAMEPAD"), 52, 125, small, muted)
    name = joystick_name or tr("尚未连接", "Not connected")
    text(_fit_text(body, name, 524), 148, 120, body, ink if joystick_name else amber)
    text(tr("摇杆输入", "STICK INPUT"), 758, 125, small, muted)
    right("—" if raw_axis is None else f"{raw_axis:+.3f}", 984, 125, blue if centered else amber)
    hint = (
        tr("上推 闭合 / 下推 张开 · 回中保持", "UP Close / DOWN Open · CENTER Hold")
        if invert_axis
        else tr("上推 张开 / 下推 闭合 · 回中保持", "UP Open / DOWN Close · CENTER Hold")
    )
    text(hint, 52, 164, small, muted)
    pygame.draw.rect(screen, border, pygame.Rect(758, 175, 226, 6), border_radius=3)
    if axis_valid and raw_axis is not None:
        cursor = 871 + round(raw_axis * 109)
        pygame.draw.rect(
            screen,
            blue if centered else amber,
            pygame.Rect(min(871, cursor), 175, max(1, abs(cursor - 871)), 6),
            border_radius=3,
        )
        pygame.draw.rect(screen, ink, pygame.Rect(cursor - 2, 171, 4, 14), border_radius=2)
    pygame.draw.rect(screen, muted, pygame.Rect(870, 173, 2, 10), border_radius=1)

    text(tr("电机反馈", "MOTOR FEEDBACK"), 32, 222, body)
    right(
        tr(
            f"速率上限 {controller.config.max_speed_rad_s:.2f} rad/s  ·  前馈 闭 {controller.config.feedforward_force_n:.2f} / 开 {controller.config.opening_force_n:.2f} N/侧",
            f"MAX RATE {controller.config.max_speed_rad_s:.2f} rad/s  ·  FF CLOSE {controller.config.feedforward_force_n:.2f} / OPEN {controller.config.opening_force_n:.2f} N/side",
        ),
        1008,
        226,
    )
    for x, label, unit, value, accent in (
        (32, tr("关节位置", "POSITION"), "q / rad", snapshot.position_rad, blue),
        (364, tr("关节速度", "VELOCITY"), "v / rad/s", snapshot.velocity_rad_s, green),
        (696, tr("关节力矩", "TORQUE"), "τ / N·m", snapshot.torque_nm, amber),
    ):
        card(x, 266, 312, 136)
        pygame.draw.rect(screen, accent, pygame.Rect(x + 20, 283, 4, 20), border_radius=2)
        text(label, x + 34, 279, small, muted)
        right(unit, x + 292, 279)
        text(number(value), x + 20, 315, title, accent if value is not None else muted)

    card(32, 422, 976, 176)
    text(tr("最近控制命令", "LAST CONTROL COMMAND"), 52, 434, body)
    right(
        tr("故障前记录", "RECORDED BEFORE FAULT")
        if snapshot.state == "fault"
        else tr("发送值 · 量化前", "SENT · BEFORE QUANTIZATION"),
        988,
        438,
        red if snapshot.state == "fault" else muted,
    )
    command = snapshot.command
    for index, (label, field, unit, accent) in enumerate(
        (
            ("q_des", "position_rad", "rad", blue),
            ("v_des", "velocity_rad_s", "rad/s", green),
            ("τ_ff", "feedforward_torque_nm", "N·m", amber),
            ("Kp", "kp", "N·m/rad", ink),
            ("Kd", "kd", "N·m·s/rad", ink),
        )
    ):
        x = 52 + index * 192
        value = getattr(command, field, None) if command is not None else None
        if index:
            pygame.draw.rect(screen, border, pygame.Rect(x - 18, 486, 1, 87))
        text(label, x, 477, small, muted)
        text(
            number(value, digits=2 if index >= 3 else 4, signed=index < 3),
            x,
            503,
            title,
            accent if value is not None else muted,
        )
        text(unit, x, 555, small, muted)

    joystick_ready = joystick_name is not None
    can_enable = (
        snapshot.state == "connected" and focused and centered and joystick_ready and axis_valid
    )
    if not joystick_ready:
        enable_label = tr("先连接手柄", "CONNECT GAMEPAD FIRST")
    elif not axis_valid:
        enable_label = tr("请检查摇杆轴", "CHECK STICK AXIS")
    elif not focused:
        enable_label = tr("点击窗口后使能", "FOCUS WINDOW TO ENABLE")
    elif snapshot.state == "ready":
        enable_label = tr("电机已使能", "MOTOR ENABLED")
    elif snapshot.state == "enabling":
        enable_label = tr("正在使能…", "ENABLING…")
    elif snapshot.state != "connected":
        enable_label = tr("等待设备就绪", "WAITING FOR DEVICE")
    elif not centered:
        enable_label = tr("摇杆回中后使能", "CENTER STICK TO ENABLE")
    else:
        enable_label = tr("使能电机", "ENABLE MOTOR")
    _button(
        pygame,
        screen,
        enable_rect,
        enable_label,
        body,
        enabled=can_enable,
        color=(46, 109, 199),
    )
    _button(
        pygame,
        screen,
        stop_rect,
        tr("STOP · 失能", "STOP · DISABLE"),
        body,
        enabled=True,
        color=(132, 48, 62),
    )
    if not joystick_ready:
        notice = tr(
            "请连接手柄；本面板不提供鼠标运动控制。",
            "Connect a gamepad; mouse motion is unavailable.",
        )
    elif not axis_valid:
        notice = tr(
            "所选轴不存在，已请求失能。", "Selected axis is unavailable; disable requested."
        )
    elif not focused:
        notice = tr(
            "窗口失焦；回到窗口后请先让摇杆回中。",
            "Window unfocused; center the stick before resuming.",
        )
    elif snapshot.state == "ready" and not input_ready:
        notice = tr(
            "请先让摇杆回到死区中心，再使能。",
            "Center the stick within the deadzone before enabling.",
        )
    elif snapshot.state == "ready" and not centered and snapshot.motion_phase == "release_to_open":
        notice = tr(
            "切换到张开：正在释放原方向力矩。", "Switching to open: releasing previous effort."
        )
    elif snapshot.state == "ready" and not centered and snapshot.motion_phase == "release_to_close":
        notice = tr(
            "切换到闭合：正在释放原方向力矩。", "Switching to close: releasing previous effort."
        )
    elif snapshot.state == "ready" and not centered:
        effective_axis = -raw_axis if invert_axis and raw_axis is not None else raw_axis
        notice = (
            tr(
                "正在打开；摇杆回中后保持当前目标。",
                "Opening; center the stick to hold the target.",
            )
            if effective_axis is not None and effective_axis < 0.0
            else tr(
                "正在闭合；摇杆回中后保持当前目标。",
                "Closing; center the stick to hold the target.",
            )
        )
    elif snapshot.state == "ready":
        notice = tr("摇杆已回中；保持当前目标。", "Stick centered; holding the current target.")
    elif not centered:
        notice = tr(
            "请先让摇杆回到死区中心，再使能。",
            "Center the stick within the deadzone before enabling.",
        )
    elif snapshot.state == "connected":
        notice = tr(
            "已连接。确认摇杆回中后，可点击使能。",
            "Connected. Click enable after centering the stick.",
        )
    else:
        notice = tr("空格或 STOP 会请求失能。", "Space or STOP requests disable.")
    notice_color = green if snapshot.state == "ready" and centered and focused else amber
    pygame.draw.rect(screen, notice_color, pygame.Rect(32, 699, 4, 18), border_radius=2)
    text(notice, 48, 690, small, notice_color)
    if not controller.demo:
        text(
            tr(
                "真机失能可能释放负载，请保持运动范围安全。",
                "Live disable may release load; keep travel clear.",
            ),
            34,
            718,
            small,
            red,
        )
    if snapshot.error:
        card(32, 750, 976, 66, (50, 29, 37))
        _draw_wrapped_text(
            screen,
            small,
            tr("错误：", "ERROR: ") + snapshot.error,
            red,
            (48, 758),
            max_width=944,
            max_lines=2,
        )


def run_panel(
    controller: _TeleopController,
    *,
    axis_index: int = 1,
    joystick_index: int = 0,
    invert_axis: bool = False,
) -> None:
    """运行 DMgripper 摇杆遥控面板，直到用户关闭窗口。

    默认读取左摇杆纵轴。上推得到负值并打开夹爪，下推得到正值并闭合夹爪；
    `invert_axis=True` 可反转该方向。面板不会自动使能。启动、手柄替换及失焦恢复后，
    都必须先将摇杆回到死区中心，才能点击使能。

    Args:
        controller: 已启动后台线程的 DMgripper 遥控控制器。
        axis_index: 要读取的手柄轴编号，默认左摇杆纵轴 `1`。
        joystick_index: 使用的 pygame 手柄编号。
        invert_axis: 是否反转读取到的轴方向。

    Raises:
        TypeError: 轴编号、手柄编号或反转标志类型无效。
    """
    if (
        not _valid_index(axis_index)
        or not _valid_index(joystick_index)
        or not isinstance(invert_axis, bool)
    ):
        try:
            _safe_disable(controller)
        finally:
            controller.shutdown()
        raise TypeError(
            "axis_index and joystick_index must be non-negative integers; invert_axis must be bool"
        )

    # pygame 是可选的本地界面依赖，导入硬件包不应要求无头测试环境安装它。
    import pygame

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((1040, 830))
    pygame.display.set_caption("DMgripper · 摇杆遥控")
    clock = pygame.time.Clock()
    chinese_font = _find_chinese_font(pygame)
    fonts = tuple(pygame.font.Font(chinese_font, size) for size in (36, 24, 20))
    enable_rect = pygame.Rect(32, 618, 464, 62)
    stop_rect = pygame.Rect(544, 618, 464, 62)
    running = True
    focused = True
    joystick: object | None = None
    joystick_identity: int | None = None
    release_required = True
    previous_state: str | None = None
    previous_axis_invalid = False

    try:
        while running:
            disable_requested = False
            candidate = _selected_joystick(pygame, joystick_index, joystick)
            try:
                identity = candidate.get_instance_id() if candidate is not None else None
            except pygame.error:
                candidate, identity = None, None
            if identity != joystick_identity:
                # 设备接入、拔出或替换后，旧轴值不能继续驱动电机。
                if joystick_identity is not None:
                    disable_requested = True
                joystick_identity = identity
                release_required = True
            joystick = candidate

            snapshot = controller.snapshot()
            if snapshot.state != previous_state:
                unsafe_states = {"disconnected", "fault", "closed"}
                if previous_state in unsafe_states or snapshot.state in unsafe_states:
                    release_required = True
                if snapshot.state in unsafe_states:
                    disable_requested = True
                previous_state = snapshot.state

            enable_clicked = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    disable_requested = True
                    running = False
                    break
                if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    disable_requested = True
                    release_required = True
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if stop_rect.collidepoint(event.pos):
                        disable_requested = True
                        release_required = True
                    elif enable_rect.collidepoint(event.pos):
                        # 实际许可条件在本帧读取物理轴后统一判定。
                        enable_clicked = True
                elif event.type == getattr(pygame, "WINDOWFOCUSLOST", -1):
                    focused = False
                    release_required = True
                    disable_requested = True
                elif event.type == getattr(pygame, "WINDOWFOCUSGAINED", -1):
                    focused = True
                    release_required = True
                    disable_requested = True
                elif event.type == getattr(pygame, "JOYDEVICEREMOVED", -1):
                    joystick, joystick_identity = None, None
                    release_required = True
                    disable_requested = True

            if focused:
                # 某些窗口管理器会漏掉 KEYDOWN；就绪状态仍须以物理空格状态优先失能。
                try:
                    space_down = bool(pygame.key.get_pressed()[pygame.K_SPACE])
                except (AttributeError, IndexError):
                    space_down = False
                if space_down:
                    release_required = True
                    disable_requested = True

            if not running:
                _safe_disable(controller)
                break

            raw_axis: float | None = None
            axis_valid = joystick is not None
            if joystick is not None:
                try:
                    axis_valid = axis_index < joystick.get_numaxes()
                    if axis_valid:
                        raw_axis = float(joystick.get_axis(axis_index))
                        axis_valid = math.isfinite(raw_axis) and -1.0 <= raw_axis <= 1.0
                except pygame.error:
                    joystick, joystick_identity, axis_valid = None, None, False
                    disable_requested = True
            if not axis_valid and joystick is not None and not previous_axis_invalid:
                disable_requested = True
            previous_axis_invalid = not axis_valid and joystick is not None

            deadzone = float(getattr(controller.config, "deadzone", 0.12))
            centered = raw_axis is not None and abs(raw_axis) <= deadzone
            if snapshot.state == "enabling" and (not focused or not axis_valid or not centered):
                # 使能尚未完成时若输入不再安全，必须取消而不能用零心跳掩盖偏转。
                disable_requested = True
                release_required = True
            if centered:
                release_required = False

            if (
                enable_clicked
                and snapshot.state == "connected"
                and focused
                and centered
                and axis_valid
                and not release_required
                and not disable_requested
            ):
                controller.enable()

            command_axis = 0.0
            if (
                snapshot.state == "ready"
                and focused
                and axis_valid
                and not release_required
                and raw_axis is not None
            ):
                command_axis = -raw_axis if invert_axis else raw_axis
            if disable_requested:
                _safe_disable(controller)
            else:
                controller.set_axis(command_axis)

            joystick_name = None
            if joystick is not None:
                try:
                    joystick_name = joystick.get_name()
                except pygame.error:
                    joystick_name = "Gamepad"
            _render_panel(
                pygame,
                screen,
                fonts,
                chinese=chinese_font is not None,
                controller=controller,
                snapshot=snapshot,
                joystick_name=joystick_name,
                raw_axis=raw_axis,
                centered=centered,
                focused=focused,
                axis_valid=axis_valid,
                input_ready=not release_required,
                invert_axis=invert_axis,
                enable_rect=enable_rect,
                stop_rect=stop_rect,
            )
            pygame.display.flip()
            clock.tick(60)
    finally:
        try:
            _safe_disable(controller)
        finally:
            controller.shutdown()
            pygame.quit()

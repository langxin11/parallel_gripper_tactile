"""独立手柄遥控的连续角目标、输入失效与设备接管契约。"""

import math
from dataclasses import replace
import threading
import time

import pytest

from dmgripper_hardware.teleop import (
    DemoSession,
    DmTeleopController,
    JointTeleopTarget,
    TeleopConfig,
    main,
)


def until(predicate, timeout=2.0):
    """等待异步状态变化，避免测试无限等待。"""
    end = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < end, "设备线程未按期完成。"
        time.sleep(0.005)


class RecordingSession(DemoSession):
    """记录生命周期并可注入反馈异常的虚拟设备。"""

    def __init__(self):
        """初始化事件记录。"""
        super().__init__()
        self.calls = []
        self.command_positions = []
        self.fail_command = False
        self.fail_disable = False
        self.enable_gate = None

    def open(self):
        """记录显式打开。"""
        self.calls.append("open")

    def close(self):
        """记录资源清理。"""
        self.calls.append("close")

    def enable(self):
        """记录使能，可阻塞以验证异步取消。"""
        self.calls.append("enable")
        if self.enable_gate:
            assert self.enable_gate.wait(2)
        return super().enable()

    def command(self, command):
        """记录目标或模拟设备反馈失败。"""
        self.calls.append(command)
        self.command_positions.append(self.position)
        if self.fail_command:
            raise OSError("反馈超时")
        return super().command(command)

    def disable(self):
        """记录失能，可模拟失能确认失败。"""
        self.calls.append("disable")
        if self.fail_disable:
            raise OSError("失能回复丢失")
        return super().disable()


@pytest.fixture
def session():
    """提供未自动使能的设备线程。"""
    device = RecordingSession()
    controller = DmTeleopController(lambda: device)
    assert not device.calls
    controller.start()
    until(lambda: controller.snapshot().state == "connected")
    yield controller, device
    controller.shutdown()
    assert not controller._thread.is_alive()


def ready(controller):
    """先提交中心心跳，再显式使能。"""
    controller.set_axis(0)
    controller.enable()
    until(lambda: controller.snapshot().state == "ready")


def test_deadzone_and_proportional_speed():
    """中心漂移被抑制，剩余行程按比例映射到目标速度。"""
    config = TeleopConfig()
    assert config.normalized_axis(0.12) == 0
    assert config.normalized_axis(-0.1) == 0
    assert config.normalized_axis(0.56) == pytest.approx(0.5)
    assert config.normalized_axis(-1) == -1
    target = JointTeleopTarget(config)
    assert target.update(0, 0.5, 0) == 0.5
    assert target.update(0.56, 0.5, 0.1) == pytest.approx(0.51)
    assert target.update(0.56, 0.51, 0.1) == pytest.approx(0.52)


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("center_axis", [-0.12, 0.0, 0.12])
def test_center_preserves_last_target_despite_feedback_error(direction, center_axis):
    """双向运动回到死区后保留目标，反馈偏差及中心漂移不清除目标。"""
    target = JointTeleopTarget(TeleopConfig())
    held = 0.5 + direction * 0.02
    assert target.update(direction, 0.5, 0.1) == pytest.approx(held)
    assert target.update(center_axis, 0.507, 0.01) == pytest.approx(held)
    assert target.update(center_axis, 0.49, 0.01) == pytest.approx(held)
    assert target.update(direction, 0.5, 0.01) == pytest.approx(held + direction * 0.002)


@pytest.mark.parametrize("direction", [-1, 1])
def test_default_target_accumulates_error_and_holds_with_stalled_feedback(direction):
    """实测位置不动时仍可累积超过 0.05 rad 的误差，回中保留，同向继续积累。"""
    config = TeleopConfig()
    assert config.max_lead_rad is None
    target = JointTeleopTarget(config)
    for _ in range(100):
        target.update(direction, 0.5, 0.01)
    held = 0.5 + direction * 0.2
    assert target.target == pytest.approx(held)
    assert target.update(0, 0.5, 0.01) == pytest.approx(held)
    for _ in range(1000):
        target.update(direction, 0.5, 0.01)
    assert target.target == (math.pi / 2 if direction > 0 else 0)


def test_explicit_lead_limit_prevents_further_accumulation():
    """仅用户显式指定领先限制时，堵转目标才停止继续累积。"""
    target = JointTeleopTarget(TeleopConfig(max_lead_rad=0.05))
    for _ in range(100):
        target.update(1, 0.5, 0.01)
    assert target.target == pytest.approx(0.55)
    assert target.update(0, 0.5, 0.01) == pytest.approx(0.55)
    assert target.update(1, 0.5, 0.01) == pytest.approx(0.55)
    before = target.target
    assert target.update(1, 0.4, 0.01) == before


def test_target_stays_in_mechanical_range():
    """默认取消领先限制后，目标仍不得越过机械行程。"""
    target = JointTeleopTarget(TeleopConfig())
    assert target.update(-1, -0.02, 0.1) == 0
    target = JointTeleopTarget(TeleopConfig())
    assert target.update(1, math.pi / 2 - 0.001, 0.1) == math.pi / 2


@pytest.mark.parametrize("initial_direction", [-1, 1])
def test_reversal_unloads_then_moves_in_new_direction(initial_direction):
    """双向反转均先归零再渐入新方向前馈，不把原方向力矩直接反号。"""
    target = JointTeleopTarget(TeleopConfig())
    assert target.update(0, 0.7, 0.01) == 0.7
    assert target.feedforward_scale == 0
    for _ in range(50):
        target.update(initial_direction, 0.7, 0.01)
    initial_target = target.target
    initial_scale = target.feedforward_scale
    assert initial_scale == initial_direction
    assert target.update(-initial_direction, 0.7, 0.01) == initial_target
    assert target.feedforward_scale == initial_scale
    previous_error = abs(initial_target - 0.7)
    for _ in range(10):
        target.update(-initial_direction, 0.7, 0.025)
        error = abs(target.target - 0.7)
        assert error <= previous_error + 1e-12
        assert 0 <= initial_direction * target.feedforward_scale <= 1
        previous_error = error
    assert target.target == pytest.approx(0.7)
    assert target.feedforward_scale == pytest.approx(0)
    # 给浮点累计留一个零位移周期，接下来明确验证新方向推进。
    target.update(-initial_direction, 0.7, 0.001)
    before = target.target
    for _ in range(10):
        target.update(-initial_direction, 0.7, 0.025)
    assert target.target == pytest.approx(before - initial_direction * 0.05)
    assert target.feedforward_scale == pytest.approx(-initial_direction)


def test_center_pauses_release_and_reversal_restarts_from_current_output():
    """切换中回中冻结输出，快速再反转不恢复旧目标，终点使用最新反馈。"""
    target = JointTeleopTarget(TeleopConfig(switch_duration_s=0.2))
    for _ in range(50):
        target.update(1, 0.7, 0.01)
    target.update(-1, 0.7, 0.01)
    target.update(-1, 0.7, 0.1)
    held, scale = target.target, target.feedforward_scale
    assert 0.7 < held < 0.8
    assert 0 < scale < 1
    for _ in range(50):
        assert target.update(0, 0.6, 0.01) == held
        assert target.feedforward_scale == scale
    assert target.phase == "holding"
    target.update(1, 0.6, 0.01)
    assert target.target == held
    assert target.feedforward_scale == scale
    target.update(1, 0.65, 0.1)
    target.update(1, 0.66, 0.1)
    assert target.target == pytest.approx(0.66)
    assert target.feedforward_scale == 0
    target.update(1, 0.66, 0.01)
    assert target.target == pytest.approx(0.662)
    assert 0 < target.feedforward_scale < 1


@pytest.mark.parametrize("direction", [-1, 1])
def test_center_pauses_initial_feedforward_ramp_and_resumes_same_direction(direction):
    """双向初次动作均渐入力矩，松手不继续加力，恢复同向时继续渐变。"""
    target = JointTeleopTarget(TeleopConfig(switch_duration_s=0.2))
    target.update(direction, 0.5, 0.1)
    held = target.target
    assert target.feedforward_scale == pytest.approx(direction * 0.5)
    for _ in range(10):
        assert target.update(0, 0.4, 0.1) == held
        assert target.feedforward_scale == pytest.approx(direction * 0.5)
    target.update(direction, 0.4, 0.1)
    assert target.target == pytest.approx(held + direction * 0.02)
    assert target.feedforward_scale == direction


def test_blend_roundoff_does_not_cross_mechanical_or_feedforward_bounds():
    """渐变接近终点时的浮点消减误差不得产生负目标或超量前馈。"""
    target = JointTeleopTarget(TeleopConfig(switch_duration_s=0.1))
    target.update(1, 0.5, 0.1 * 0.999999)
    assert 0 <= target.feedforward_scale <= 1
    target.update(-1, 0, 0.01)
    target.update(-1, 0, 0.1 * 0.999999)
    assert 0 <= target.target <= math.pi / 2
    assert 0 <= target.feedforward_scale <= 1


@pytest.mark.parametrize("resume_dt", [0.0, 0.01])
def test_resume_release_after_feedback_moves_does_not_jump(resume_dt):
    """回中期间反馈移动后恢复同方向，入口仍保持暂停时输出。"""
    target = JointTeleopTarget(TeleopConfig(switch_duration_s=0.2))
    for _ in range(250):
        target.update(1, 0.5, 0.01)
    target.update(-1, 0.5, 0.01)
    target.update(-1, 0.5, 0.1)
    assert target.target == pytest.approx(0.75)
    assert target.feedforward_scale == pytest.approx(0.5)
    target.update(0, 0.2, 0.1)
    held, scale = target.target, target.feedforward_scale
    assert target.update(-1, 0.2, resume_dt) == held
    assert target.feedforward_scale == scale
    target.update(-1, 0.2, 0.1)
    assert 0.2 < target.target < held
    target.update(-1, 0.2, 0.1)
    assert target.target == pytest.approx(0.2)
    assert target.feedforward_scale == 0


@pytest.mark.parametrize("position", [0.0, math.pi / 4, math.pi / 2])
def test_force_feedforward_matches_geometry_and_side_semantics(position):
    """独立总开度差分验证雅可比及单侧力定义，防止多乘或少乘 2。"""

    def aperture(q):
        """按已知几何计算两指总开度，用差分独立核对力矩换算。"""
        alpha = q + math.pi / 4
        radial = 0.03 * math.sin(alpha) - 0.03 / math.sqrt(2)
        return 2 * (0.03 * math.cos(alpha) + math.sqrt(0.04**2 - radial**2))

    epsilon = 1e-6
    jacobian = (aperture(position - epsilon) - aperture(position + epsilon)) / (2 * epsilon)
    assert TeleopConfig().feedforward_force_n == 1.0
    assert TeleopConfig().feedforward_torque(position) == pytest.approx(jacobian)
    assert TeleopConfig(feedforward_force_n=2).feedforward_torque(position) == pytest.approx(
        2 * jacobian
    )
    assert TeleopConfig(feedforward_force_n=0).feedforward_torque(position) == 0
    assert TeleopConfig().feedforward_torque(position, opening=True) == pytest.approx(-jacobian)
    config = TeleopConfig(feedforward_force_n=1, opening_feedforward_force_n=2)
    assert config.feedforward_torque(position, opening=True) == pytest.approx(-2 * jacobian)
    assert config.feedforward_torque(position) == pytest.approx(jacobian)
    assert TeleopConfig(feedforward_force_n=0).feedforward_torque(position, opening=True) == 0
    assert (
        TeleopConfig(opening_feedforward_force_n=0).feedforward_torque(position, opening=True) == 0
    )


def test_force_feedforward_respects_protocol_limit():
    """高目标力的前馈不得突破电机协议力矩量程。"""
    assert TeleopConfig(feedforward_force_n=1000).feedforward_torque(math.pi / 4) == 4
    assert (
        TeleopConfig(feedforward_force_n=1000).feedforward_torque(math.pi / 4, opening=True) == -4
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"max_speed_rad_s": float("nan")},
        {"max_speed_rad_s": 9},
        {"deadzone": 1},
        {"kp": 501},
        {"kd": -1},
        {"max_lead_rad": 0},
        {"max_lead_rad": -0.05},
        {"max_lead_rad": float("nan")},
        {"max_lead_rad": float("inf")},
        {"feedforward_force_n": -1},
        {"feedforward_force_n": float("nan")},
        {"feedforward_force_n": float("inf")},
        {"feedforward_force_n": True},
        {"opening_feedforward_force_n": -1},
        {"opening_feedforward_force_n": float("nan")},
        {"opening_feedforward_force_n": float("inf")},
        {"opening_feedforward_force_n": True},
        {"switch_duration_s": 0},
        {"switch_duration_s": -1},
        {"switch_duration_s": float("nan")},
        {"switch_duration_s": float("inf")},
    ],
)
def test_invalid_config_rejected(kwargs):
    """非法增益、速度和死区不能创建控制器。"""
    with pytest.raises(ValueError):
        TeleopConfig(**kwargs)


def test_late_tick_rejected():
    """控制周期严重迟到时拒绝一次性积分大位移。"""
    with pytest.raises(ValueError, match="周期"):
        JointTeleopTarget(TeleopConfig()).update(1, 0.5, 0.101)


def test_connection_never_enables_and_rejects_off_center(session):
    """连接不接管电机，偏转或无心跳时不能使能。"""
    controller, device = session
    controller.enable()
    controller.set_axis(1)
    controller.enable()
    time.sleep(0.06)
    assert device.calls == ["open"]
    controller.shutdown()
    assert device.calls == ["open", "close"]


def test_feedback_and_last_command_are_distinct_and_disable_clears_command():
    """真实反馈通道保留非零速度和力矩，命令快照不冒充反馈，失能后清空命令。"""

    class TelemetrySession(RecordingSession):
        """提供与命令明显不同且可更新的反馈，用于验证面板数据来源。"""

        velocity = -0.37
        torque = 0.24

        def _feedback(self):
            """在角度演示反馈上注入速度和力矩。"""
            return replace(super()._feedback(), velocity_rad_s=self.velocity, torque_nm=self.torque)

        def command(self, command):
            """模拟受阻关节，接收命令但反馈位置不跟随目标。"""
            self.calls.append(command)
            return self._feedback()

    device = TelemetrySession()
    controller = DmTeleopController(
        lambda: device, TeleopConfig(kp=3.0, kd=0.7, switch_duration_s=0.05)
    )
    assert controller.snapshot().velocity_rad_s is None
    assert controller.snapshot().torque_nm is None
    controller.start()
    try:
        until(lambda: controller.snapshot().state == "connected")
        snapshot = controller.snapshot()
        assert snapshot.velocity_rad_s == -0.37
        assert snapshot.torque_nm == 0.24
        assert snapshot.command is None
        device.velocity = -0.21
        until(lambda: controller.snapshot().velocity_rad_s == -0.21)
        ready(controller)
        controller.set_axis(1)
        until(lambda: (controller.snapshot().target_rad or 0.0) > device.position)
        until(lambda: (controller.snapshot().feedforward_torque_nm or 0.0) > 0.05999)
        controller.set_axis(0)
        snapshot = controller.snapshot()
        assert snapshot.position_rad == device.position
        assert snapshot.velocity_rad_s == -0.21
        assert snapshot.torque_nm == 0.24
        assert snapshot.command in device.calls
        assert snapshot.command.position_rad == snapshot.target_rad
        assert snapshot.command.velocity_rad_s == 0
        assert snapshot.command.kp == 3.0
        assert snapshot.command.kd == 0.7
        assert snapshot.command.feedforward_torque_nm == snapshot.feedforward_torque_nm
        assert snapshot.command.feedforward_torque_nm == pytest.approx(0.06)
        controller.disable()
        until(lambda: controller.snapshot().state == "connected")
        snapshot = controller.snapshot()
        assert snapshot.command is None
        assert snapshot.target_rad is None
        assert snapshot.feedforward_torque_nm is None
        assert snapshot.velocity_rad_s == -0.21
        assert snapshot.torque_nm == 0.24
        ready(controller)
        until(lambda: controller.snapshot().command is not None)
        assert controller.snapshot().command.feedforward_torque_nm == 0
        assert controller.snapshot().target_rad == device.position
    finally:
        controller.shutdown()


@pytest.mark.parametrize("force", [0.0, 1.0, 2.0])
@pytest.mark.parametrize("direction", [-1, 1])
def test_continuous_commands_and_center_hold(session, force, direction):
    """运动及回中均按最新反馈角生成前馈，保持固定角目标并在退出失能。"""
    controller, device = session
    controller.config = TeleopConfig(feedforward_force_n=force, switch_duration_s=0.05)
    ready(controller)
    origin = device.position
    end = time.monotonic() + 0.08
    while time.monotonic() < end:
        controller.set_axis(direction)
        time.sleep(0.01)
    controller.set_axis(0)
    time.sleep(0.03)
    held = controller.snapshot().target_rad
    assert direction * (held - origin) > 0
    time.sleep(0.03)
    assert controller.snapshot().target_rad == held
    commands = [c for c in device.calls if not isinstance(c, str)]
    assert len(commands) > 3
    assert all(c.velocity_rad_s == 0 for c in commands)
    for command, measured_position in zip(commands, device.command_positions, strict=True):
        assert (
            0
            <= direction * command.feedforward_torque_nm
            <= abs(controller.config.feedforward_torque(measured_position, opening=direction < 0))
        )
    if force:
        assert commands[0].feedforward_torque_nm == 0
        assert commands[-1].feedforward_torque_nm == pytest.approx(
            controller.config.feedforward_torque(device.position, opening=direction < 0)
        )
    assert controller.snapshot().feedforward_torque_nm == pytest.approx(
        commands[-1].feedforward_torque_nm
    )
    controller.shutdown()
    assert device.calls[-2:] == ["disable", "close"]


def test_lost_input_disables_without_automatic_reenable(session):
    """失去手柄心跳后失能，新输入本身不自动恢复使能。"""
    controller, device = session
    ready(controller)
    controller.set_axis(1)
    until(lambda: "disable" in device.calls)
    assert controller.snapshot().state == "connected"
    controller.set_axis(1)
    time.sleep(0.04)
    assert device.calls.count("enable") == 1
    assert not device.enabled


def test_asymmetric_opening_feedforward_releases_before_closing(session):
    """开闭幅度不同时，先释放原负力矩到零，再渐入正力矩，避免切换幅度跳变。"""
    controller, device = session
    controller.config = TeleopConfig(
        feedforward_force_n=1, opening_feedforward_force_n=2, switch_duration_s=0.05
    )
    ready(controller)
    controller.set_axis(-1)
    until(lambda: (controller.snapshot().feedforward_torque_nm or 0) < -0.115)
    controller.set_axis(0)
    time.sleep(0.03)
    assert controller.snapshot().feedforward_torque_nm == pytest.approx(
        controller.config.feedforward_torque(device.position, opening=True)
    )
    beginning = len(device.calls)
    controller.set_axis(1)
    until(lambda: (controller.snapshot().feedforward_torque_nm or 0) > 0.05)
    controller.set_axis(0)
    commands = [c for c in device.calls[beginning:] if not isinstance(c, str)]
    first_positive = next(
        i for i, command in enumerate(commands) if command.feedforward_torque_nm > 0
    )
    assert commands[0].feedforward_torque_nm < 0
    assert any(command.feedforward_torque_nm == 0 for command in commands[:first_positive])
    assert all(command.feedforward_torque_nm <= 0 for command in commands[:first_positive])


def test_input_timeout_during_direction_release_disables(session):
    """反向释放阶段仍受输入超时约束，不会自主继续完成整个过渡。"""
    controller, device = session
    controller.config = TeleopConfig(switch_duration_s=1.0)
    ready(controller)
    controller.set_axis(1)
    until(lambda: controller.snapshot().motion_phase == "closing")
    controller.set_axis(-1)
    until(lambda: controller.snapshot().motion_phase == "release_to_open")
    until(lambda: "disable" in device.calls)
    assert controller.snapshot().state == "connected"
    assert controller.snapshot().command is None
    assert not device.enabled


def test_disable_during_enable_never_sends_command(session):
    """使能等待中收到失焦或断连请求，确认返回后也必须失能。"""
    controller, device = session
    device.enable_gate = threading.Event()
    controller.set_axis(0)
    controller.enable()
    until(lambda: "enable" in device.calls)
    controller.disable()
    device.enable_gate.set()
    until(lambda: "disable" in device.calls)
    assert not any(not isinstance(c, str) for c in device.calls)


def test_axis_moves_during_enable_cancels_enable(session):
    """使能过程中摇杆离开中心不能直接引发运动。"""
    controller, device = session
    device.enable_gate = threading.Event()
    controller.set_axis(0)
    controller.enable()
    until(lambda: "enable" in device.calls)
    controller.set_axis(1)
    device.enable_gate.set()
    until(lambda: "disable" in device.calls)
    assert not any(not isinstance(c, str) for c in device.calls)


def test_fault_cleanup_preserves_disable_failure(session):
    """通信失败后退出，并同时报告失能未确认。"""
    controller, device = session
    ready(controller)
    device.fail_disable = True
    device.fail_command = True
    until(lambda: not controller._thread.is_alive())
    assert controller.snapshot().state == "fault"
    assert "反馈超时" in controller.snapshot().error
    assert "失能未确认" in controller.snapshot().error
    assert device.calls[-1] == "close"


def test_already_enabled_device_is_not_taken_over():
    """初检发现已使能时拒绝接管，也不失能其他程序拥有的设备。"""
    device = RecordingSession()
    device.enabled = True
    controller = DmTeleopController(lambda: device)
    controller.start()
    until(lambda: not controller._thread.is_alive())
    assert controller.snapshot().state == "fault"
    assert device.calls == ["open", "close"]


@pytest.mark.parametrize(
    "args",
    [
        ["--axis", "-1"],
        ["--max-speed", "nan"],
        ["--deadzone", "1"],
        ["--mit-kp", "999"],
        ["--feedforward-force", "-1"],
        ["--feedforward-force", "nan"],
        ["--feedforward-force", "inf"],
        ["--switch-duration", "0"],
        ["--switch-duration", "nan"],
        ["--opening-feedforward-force", "-1"],
        ["--opening-feedforward-force", "nan"],
    ],
)
def test_invalid_cli_never_opens_device(args):
    """参数在启动设备线程之前校验。"""
    with pytest.raises(SystemExit) as exc:
        main(args)
    assert exc.value.code == 2


@pytest.mark.parametrize(
    "force_args, expected_force, expected_opening_force",
    [
        ([], 1.0, 1.0),
        (["--feedforward-force", "0"], 0.0, 0.0),
        (["--feedforward-force", "2"], 2.0, 2.0),
        (["--opening-feedforward-force", "0.5"], 1.0, 0.5),
        (["--opening-feedforward-force", "0"], 1.0, 0.0),
    ],
)
def test_default_cli_never_constructs_hardware(
    monkeypatch, force_args, expected_force, expected_opening_force
):
    """默认启动即使指定端口也只使用离线模拟。"""
    import sys
    from types import SimpleNamespace
    from dmgripper_hardware import teleop

    def forbidden(*args, **kwargs):
        """阻止意外构造真实串口会话。"""
        pytest.fail("默认启动不能构造 DmSession。")

    def panel(controller, **kwargs):
        """模拟面板先检查连接，再退出。"""
        until(lambda: controller.snapshot().state == "connected")
        assert controller.demo
        assert controller.config.max_lead_rad is None
        assert controller.config.feedforward_force_n == expected_force
        assert controller.config.opening_force_n == expected_opening_force
        assert kwargs == dict(axis_index=1, joystick_index=0, invert_axis=False)

    monkeypatch.setattr(teleop, "DmSession", forbidden)
    monkeypatch.setitem(
        sys.modules, "dmgripper_hardware.teleop_ui", SimpleNamespace(run_panel=panel)
    )
    main(["--port", "/dev/never-open", *force_args])


def test_invalid_axis_input_disables(session):
    """非法轴值立即请求失能，不保留上一次有效运动输入。"""
    controller, device = session
    ready(controller)
    with pytest.raises(ValueError):
        controller.set_axis(float("nan"))
    until(lambda: "disable" in device.calls)
    assert not device.enabled

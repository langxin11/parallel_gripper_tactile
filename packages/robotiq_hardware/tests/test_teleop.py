"""遥控会话的停止、故障和显式激活契约。"""

import threading
import time

import pytest

from robotiq_hardware.teleop import TeleopController, create_device, main


@pytest.mark.parametrize("error_type", [KeyboardInterrupt, RuntimeError])
def test_panel_interrupt_cleans_up_without_hiding_other_errors(monkeypatch, error_type):
    """Ctrl+C 正常退出、其他异常仍透传，两者均停止设备并释放串口。"""
    from contextlib import nullcontext

    from robotiq_hardware import teleop, teleop_ui

    device = FakeDevice()
    sessions = []

    def interrupted_panel(controller, **kwargs):
        """模拟已激活窗口收到中断或真正的程序异常。"""
        sessions.append(controller)
        until(lambda: controller.snapshot().state == "connected")
        ready(controller)
        raise error_type()

    monkeypatch.setattr(teleop, "DemoDevice", lambda: device)
    monkeypatch.setattr(teleop_ui, "run_panel", interrupted_panel)
    expected = nullcontext() if error_type is KeyboardInterrupt else pytest.raises(error_type)
    with expected:
        main(["--dry-run"])

    assert [call[0] for call in device.calls[-2:]] == ["stop", "disconnect"]
    assert not sessions[0]._thread.is_alive()


class FakeDevice:
    """记录设备操作，并允许注入反馈异常。"""

    def __init__(self):
        """初始化事件记录。"""
        self.calls = []
        self.fail_read = False
        self.fault = 0
        self.activation_gate = None

    def activate(self, **kwargs):
        """记录激活，可模拟延迟。"""
        self.calls.append(("activate", kwargs))
        if self.activation_gate:
            self.activation_gate.wait(2)

    def stop(self, **kwargs):
        """记录停止。"""
        self.calls.append(("stop", kwargs))

    def move(self, position, **kwargs):
        """记录运动请求。"""
        self.calls.append(("move", position, kwargs))

    def position(self):
        """模拟通信失败或正常反馈。"""
        if self.fail_read:
            raise OSError("串口断开")
        return 128

    def status(self, **kwargs):
        """返回设备故障码。"""
        return {"gFLT": self.fault, "gSTA": 3, "gACT": 1}

    def disconnect(self):
        """记录断开。"""
        self.calls.append(("disconnect",))


def until(predicate, timeout=2):
    """等待异步结果，超时给出明确断言失败。"""
    end = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < end, "工作线程未按时完成操作。"
        time.sleep(0.005)


@pytest.fixture
def session():
    """提供无真实串口的工作会话，并确保线程退出。"""
    device = FakeDevice()
    controller = TeleopController(lambda: device)
    assert device.calls == []
    assert controller.snapshot().state == "disconnected"
    controller.start()
    until(lambda: controller.snapshot().state == "connected")
    yield controller, device
    controller.shutdown()
    until(lambda: not controller._thread.is_alive())


def ready(controller):
    """显式激活并释放输入。"""
    controller.activate()
    until(lambda: controller.snapshot().state == "ready")
    controller.set_direction(0)


def hold_until_continuous(controller, device):
    """持续提供长按心跳，等待连续目标下发。"""
    end = time.monotonic() + 2
    while not any(c[0] == "move" and c[1] == 255 for c in device.calls):
        assert time.monotonic() < end
        controller.set_direction(1)
        time.sleep(0.02)


def test_connect_does_not_activate_or_move(session):
    """连接和按住输入都不能隐式激活或发送运动。"""
    controller, device = session
    controller.set_direction(1)
    time.sleep(0.06)
    assert device.calls == []
    controller.shutdown()
    assert device.calls == [("disconnect",)]


def test_release_stop_and_restart_atomic_target(session):
    """松开会停止，下一次运动原子写入启动位与新目标。"""
    controller, device = session
    ready(controller)
    controller.set_direction(1)
    hold_until_continuous(controller, device)
    assert device.calls[-1] == (
        "move",
        255,
        dict(speed=30, force=30, wait=False, readStatus=False, refreshStatus=False, start=True),
    )
    controller.set_direction(0)
    until(lambda: device.calls[-1][0] == "stop")
    controller.set_direction(-1)
    until(lambda: device.calls[-1][0] == "move")
    assert device.calls[-1][1] == 127
    controller.shutdown()
    assert [c[0] for c in device.calls[-2:]] == ["stop", "disconnect"]


def test_watchdog_requires_release(session):
    """失去窗口心跳后停止，重新出现的按住输入不能直接恢复运动。"""
    controller, device = session
    ready(controller)
    controller.set_direction(1)
    until(lambda: device.calls[-1][0] == "move")
    until(lambda: device.calls[-1][0] == "stop")
    count = len(device.calls)
    controller.set_direction(1)
    time.sleep(0.06)
    assert len(device.calls) == count
    controller.set_direction(0)
    controller.set_direction(1)
    until(lambda: device.calls[-1][0] == "move")


def test_activation_discards_held_input(session):
    """激活期间的按住输入必须释放后才生效。"""
    controller, device = session
    device.activation_gate = threading.Event()
    controller.activate()
    until(lambda: device.calls and device.calls[0][0] == "activate")
    controller.set_direction(1)
    device.activation_gate.set()
    until(lambda: controller.snapshot().state == "ready")
    controller.set_direction(1)
    time.sleep(0.06)
    assert not any(c[0] == "move" for c in device.calls)


@pytest.mark.parametrize("failure", ["read", "fault"])
def test_failure_stops_disconnects_and_latches(session, failure):
    """通信或设备故障后尝试停止，断开并禁止自动恢复。"""
    controller, device = session
    ready(controller)
    controller.set_direction(1)
    until(lambda: device.calls[-1][0] == "move")
    device.fail_read = failure == "read"
    device.fault = 14 if failure == "fault" else 0
    until(lambda: not controller._thread.is_alive())
    assert controller.snapshot().state == "fault"
    assert [c[0] for c in device.calls[-2:]] == ["stop", "disconnect"]
    count = len(device.calls)
    controller.activate()
    controller.set_direction(1)
    assert len(device.calls) == count


def test_driver_constructor_and_rtu_contract(monkeypatch):
    """用真实固定版本驱动配假 Modbus 客户端验证串口参数及清理。"""
    pymodbus = pytest.importorskip("pymodbus.client")
    pytest.importorskip("pyrobotiqgripper")
    clients = []

    class Client:
        """避免打开串口，模拟连接失败。"""

        def __init__(self, **kwargs):
            """记录串口设置。"""
            self.kwargs = kwargs
            self.closed = False
            clients.append(self)

        def connect(self):
            """模拟设备不可用。"""
            return False

        def close(self):
            """记录失败后的资源清理。"""
            self.closed = True

    monkeypatch.setattr(pymodbus, "ModbusSerialClient", Client)
    with pytest.raises(Exception, match="Failed to connect"):
        create_device("/dev/fake", 115200, 9)
    assert clients[0].closed
    assert clients[0].kwargs == dict(
        port="/dev/fake",
        baudrate=115200,
        parity="N",
        stopbits=1,
        bytesize=8,
        timeout=0.15,
        retries=0,
    )


@pytest.mark.parametrize(
    "args",
    [
        [],
        ["--port", "auto"],
        ["--dry-run", "--force", "256"],
        ["--dry-run", "--open-button", "1"],
        ["--dry-run", "--device-id", "0"],
    ],
)
def test_invalid_cli_never_connects(args):
    """非法参数在创建会话前被拒绝。"""
    with pytest.raises(SystemExit) as exc:
        main(args)
    assert exc.value.code == 2


def test_real_driver_writes_atomic_start_and_stop(monkeypatch):
    """在假串口上核对实际驱动的激活、启动及停止寄存器编码。"""
    from types import SimpleNamespace

    pymodbus = pytest.importorskip("pymodbus.client")
    pytest.importorskip("pyrobotiqgripper")
    writes = []

    class Client:
        """模拟未激活设备及其寄存器反馈。"""

        def __init__(self, **kwargs):
            """初始化设备状态。"""
            self.active = False

        def connect(self):
            """模拟连接成功。"""
            return True

        def close(self):
            """假串口无需释放设备。"""

        def read_input_registers(self, address, *, count, device_id):
            """报告由激活写入切换的状态。"""
            assert (address, count, device_id) == (2000, 3, 9)
            return SimpleNamespace(
                registers=[0x3100 if self.active else 0, 0, 128 << 8],
                isError=lambda: False,
            )

        def write_registers(self, address, values, *, device_id):
            """记录实际底层命令。"""
            assert (address, device_id) == (1000, 9)
            self.active = bool(values[0] & 0x0100)
            writes.append(values)
            return SimpleNamespace(isError=lambda: False, count=len(values))

    monkeypatch.setattr(pymodbus, "ModbusSerialClient", Client)
    device = create_device("/dev/fake", 115200, 9)
    assert writes == []
    device.activate(reset=False, start=False, refreshStatus=True)
    device.stop(refreshStatus=True, readStatus=False)
    device.move(
        255, speed=30, force=30, wait=False, readStatus=False, refreshStatus=False, start=True
    )
    device.stop(refreshStatus=False, readStatus=False)
    device.disconnect()
    assert writes == [[0x0100], [0x0100], [0x0900, 255, 0x1E1E], [0x0100]]


def test_quick_release_is_not_overwritten(session):
    """快速松开并重新按下时，工作线程仍必须提交中间的停止。"""
    controller, device = session
    ready(controller)
    hold_until_continuous(controller, device)
    start = len(device.calls)
    controller.set_direction(0)
    controller.set_direction(1)
    until(lambda: len(device.calls) >= start + 2)
    assert [c[0] for c in device.calls[start : start + 2]] == ["stop", "move"]


def test_quit_during_activation_never_moves(session):
    """退出请求在激活结束后优先执行停止，不能继续发送位置命令。"""
    controller, device = session
    device.activation_gate = threading.Event()
    controller.activate()
    until(lambda: device.calls and device.calls[0][0] == "activate")
    controller.set_direction(1)
    controller._quit.set()
    device.activation_gate.set()
    until(lambda: not controller._thread.is_alive())
    assert not any(c[0] == "move" for c in device.calls)
    assert [c[0] for c in device.calls[-2:]] == ["stop", "disconnect"]


def test_stop_failure_is_visible_and_disconnects(session, monkeypatch):
    """停止写入失败不会显示为正常关闭，也不会阻止串口清理。"""
    controller, device = session
    ready(controller)

    def fail_stop(**kwargs):
        """模拟停止命令无法送达。"""
        raise OSError("设备离线")

    monkeypatch.setattr(device, "stop", fail_stop)
    controller.shutdown()
    assert controller.snapshot().state == "fault"
    assert "停止未确认" in controller.snapshot().error
    assert device.calls[-1] == ("disconnect",)


@pytest.fixture
def jog_clock(monkeypatch):
    """提供确定时钟和未启动线程的点动会话，用于核对命令节拍。"""
    now = [10.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    device = FakeDevice()
    controller = TeleopController(lambda: device)
    controller._publish(state="ready")
    controller.set_direction(0)
    return controller, device, now


def test_jog_quick_click_is_exactly_one_command(jog_clock):
    """在串口采样前已松开的短点击仍发出单步，随后没有重复。"""
    controller, device, now = jog_clock
    controller.set_direction(1)
    controller.set_direction(0)
    assert controller._motion_step(device)
    assert device.calls[-1][1] == 129
    for delta in (0.1, 0.4, 1.0):
        now[0] += delta
        controller.set_direction(0)
        assert not controller._motion_step(device)
    assert len(device.calls) == 1
    assert controller.snapshot().target == 129


def test_hold_upgrades_once_after_delay(jog_clock):
    """长按阈值前仅单步，阈值后一次切换终点目标，不重复发送。"""
    controller, device, now = jog_clock
    controller.set_direction(1)
    assert controller._motion_step(device)
    for timestamp, expected in [
        (10.2, False),
        (10.399, False),
        (10.401, True),
        (10.5, False),
        (12.0, False),
    ]:
        now[0] = timestamp
        controller.set_direction(1)
        assert controller._motion_step(device) is expected
    assert [c[1] for c in device.calls] == [129, 255]
    controller.set_direction(0)
    assert controller._input()[2]


def test_jog_direction_reversal_and_limits(jog_clock):
    """反向一步相对上次成功目标递减，边界不溢出也不重复写入。"""
    controller, device, _ = jog_clock
    controller._target = 254
    controller.set_direction(1)
    assert controller._motion_step(device)
    controller.set_direction(0)
    controller.set_direction(1)
    assert not controller._motion_step(device)
    controller.set_direction(-1)
    assert controller._motion_step(device)
    assert [c[1] for c in device.calls] == [255, 254]
    controller._target = 0
    controller.set_direction(0)
    controller.set_direction(-1)
    assert not controller._motion_step(device)


@pytest.mark.parametrize("reason", ["cancel", "timeout", "shutdown"])
def test_jog_pending_click_cancelled_before_send(jog_clock, reason):
    """安全停止、失鲜和退出均取消尚未发送的短点击。"""
    controller, device, now = jog_clock
    controller.set_direction(1)
    controller.set_direction(0)
    if reason == "cancel":
        controller.cancel_motion()
    elif reason == "timeout":
        now[0] += 0.3
        controller._input()
    else:
        controller.shutdown()
    assert not controller._motion_step(device)
    assert device.calls == []


def test_jog_cancel_during_feedback_read_cannot_send(jog_clock, monkeypatch):
    """获取首步基准期间取消操作，读返回后也不能下发过期目标。"""
    controller, device, _ = jog_clock

    def read_and_cancel():
        """模拟串口读取期间收到 STOP。"""
        controller.cancel_motion()
        return 128

    monkeypatch.setattr(device, "position", read_and_cancel)
    controller.set_direction(1)
    assert not controller._motion_step(device)
    assert not device.calls


def test_jog_failed_send_does_not_advance_target(jog_clock, monkeypatch):
    """失败的写入不能记为已成功推进目标。"""
    controller, device, _ = jog_clock

    def fail(*args, **kwargs):
        """模拟写入失败。"""
        raise OSError("写入失败")

    monkeypatch.setattr(device, "move", fail)
    controller.set_direction(1)
    with pytest.raises(OSError, match="写入失败"):
        controller._motion_step(device)
    assert controller.snapshot().target is None
    assert controller._target is None


def test_jog_worker_stop_and_rebase_after_cancel():
    """工作线程处理显式停止，重新点动从反馈位置建立基准。"""
    device = FakeDevice()
    controller = TeleopController(lambda: device)
    controller.start()
    try:
        until(lambda: controller.snapshot().state == "connected")
        ready(controller)
        controller.set_direction(1)
        controller.set_direction(0)
        until(lambda: any(c[0] == "move" for c in device.calls))
        assert controller.snapshot().target == 129
        controller.cancel_motion()
        until(lambda: controller.snapshot().target is None)
        assert device.calls[-1][0] == "stop"
        controller.set_direction(0)
        controller.set_direction(-1)
        controller.set_direction(0)
        until(lambda: device.calls[-1][0] == "move")
        assert device.calls[-1][1] == 127
    finally:
        controller.shutdown()
    assert device.calls[-1][0] == "disconnect"


def test_release_before_hold_threshold_never_starts_continuous(jog_clock):
    """阈值前松开后，即使时间越过阈值也只能完成一次短按目标。"""
    controller, device, now = jog_clock
    controller.set_direction(-1)
    assert controller._motion_step(device)
    now[0] = 10.399
    controller.set_direction(0)
    assert not controller._input()[2]
    now[0] = 10.6
    controller.set_direction(0)
    assert not controller._motion_step(device)
    assert [c[1] for c in device.calls] == [127]


def test_release_during_continuous_write_latches_stop(jog_clock, monkeypatch):
    """终点命令尚在写入时松开，也必须保证下一周期停止。"""
    controller, device, now = jog_clock
    controller.set_direction(1)
    controller._motion_step(device)
    original = device.move

    def release_during_write(position, **kwargs):
        """模拟串口写入期间的鼠标释放事件。"""
        controller.set_direction(0)
        original(position, **kwargs)

    monkeypatch.setattr(device, "move", release_during_write)
    now[0] = 10.401
    controller.set_direction(1)
    assert controller._motion_step(device)
    assert controller._input()[2]
    assert device.calls[-1][1] == 255


def test_release_during_continuous_read_cancels_upgrade(jog_clock, monkeypatch):
    """长按升级取得反馈时松开，不得发出终点目标。"""
    controller, device, now = jog_clock
    controller.set_direction(1)
    controller._motion_step(device)
    controller._target = None

    def release_during_read():
        """模拟反馈等待期间松开。"""
        controller.set_direction(0)
        return 129

    monkeypatch.setattr(device, "position", release_during_read)
    now[0] = 10.401
    controller.set_direction(1)
    assert not controller._motion_step(device)
    assert [c[1] for c in device.calls] == [129]

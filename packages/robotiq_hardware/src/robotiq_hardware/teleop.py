"""USB／RS485 鼠标与手柄遥控入口，以及串行设备会话。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from importlib.metadata import version
import threading
import time
from typing import Callable

from .gripper import validate_position_command


@dataclass(frozen=True)
class TeleopStatus:
    """供窗口读取的不可变设备状态。"""

    state: str = "disconnected"
    position: int | None = None
    error: str | None = None
    target: int | None = None


class TeleopController:
    """在唯一工作线程中连接设备、处理输入和刷新反馈。

    窗口必须持续提交输入；超过 250 ms 未更新时停止并等待释放输入。
    软件停止仍依赖串口可用，不能替代硬件急停。
    """

    def __init__(
        self,
        factory: Callable,
        *,
        speed: int = 30,
        force: int = 30,
    ):
        """创建会话；显式调用 start 后才连接，不自动激活夹爪。"""
        self._pending_press = 0
        self._hold_deadline = 0.0
        self._target = None
        self._continuous_direction = 0
        self._cancel_generation = 0
        self.speed = validate_position_command(speed)
        self.force = validate_position_command(force)
        self._factory = factory
        self._lock = threading.Lock()
        self._quit = threading.Event()
        self._status = TeleopStatus()
        self._direction = 0
        self._updated = time.monotonic()
        self._activate = False
        self._stop_pending = False
        self._inhibit = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="robotiq-serial")

    def start(self) -> None:
        """显式启动唯一设备线程并连接串口。"""
        self._thread.start()

    def snapshot(self) -> TeleopStatus:
        """读取一致的状态快照，不访问串口。"""
        with self._lock:
            return self._status

    def _publish(self, **changes) -> None:
        """原子发布设备状态。"""
        with self._lock:
            self._status = replace(self._status, **changes)

    def activate(self) -> None:
        """提交一次显式激活请求，仅连接状态下接受。"""
        with self._lock:
            if self._status.state == "connected":
                self._activate = True
                self._status = replace(self._status, state="activating")
                self._direction = 0
                self._inhibit = True
                self._pending_press = 0

    def set_direction(self, direction: int) -> None:
        """提交方向心跳；短按单步，长按连续运动，松开停止连续运动。"""
        if type(direction) is not int or direction not in (-1, 0, 1):
            raise ValueError("方向只能为整数 -1、0 或 1。")
        with self._lock:
            self._updated = time.monotonic()
            if direction == 0:
                if self._continuous_direction:
                    self._stop_pending = True
                if self._status.state == "ready":
                    self._inhibit = False
            if self._status.state != "ready" or self._inhibit:
                self._direction = 0
            else:
                if direction and direction != self._direction:
                    if self._continuous_direction:
                        self._stop_pending = True
                    # 仅保留最新按下沿，不积压设备未来仍需执行的点击队列。
                    self._pending_press = direction
                    self._hold_deadline = self._updated + 0.4
                self._direction = direction

    def cancel_motion(self) -> None:
        """取消未发点动并请求停止；必须释放输入才允许恢复。"""
        with self._lock:
            self._stop_pending = True
            self._direction = 0
            self._pending_press = 0
            self._inhibit = True
            self._cancel_generation += 1

    def _motion_step(self, device) -> bool:
        """按下发出单步，持续按住 0.4 秒后只发送一次连续运动目标。"""
        with self._lock:
            now = time.monotonic()
            if self._inhibit or self._quit.is_set() or self._status.state != "ready":
                return False
            if now - self._updated > 0.25:
                return False
            direction, self._pending_press = self._pending_press, 0
            continuous = False
            if not direction and self._direction and now >= self._hold_deadline:
                direction = self._direction
                continuous = True
                if self._continuous_direction == direction:
                    return False
            generation = self._cancel_generation
            deadline = self._hold_deadline
        if not direction:
            return False
        baseline = self._target
        if baseline is None:
            baseline = validate_position_command(device.position())
        target = (
            (0 if direction < 0 else 255) if continuous else max(0, min(255, baseline + direction))
        )
        with self._lock:
            if (
                generation != self._cancel_generation
                or self._quit.is_set()
                or self._stop_pending
                or time.monotonic() - self._updated > 0.25
            ):
                return False
            if continuous:
                # 读反馈期间松开或重新按下时，不能执行先前的长按升级。
                if self._direction != direction or self._hold_deadline != deadline:
                    return False
                # 写入尚在进行时松开也必须锁存停止，不能等到写入返回才设置。
                self._continuous_direction = direction
        if target == baseline:
            return False
        device.move(
            target,
            speed=self.speed,
            force=self.force,
            wait=False,
            readStatus=False,
            refreshStatus=False,
            start=True,
        )
        self._target = target
        self._publish(target=target)
        return True

    def shutdown(self) -> None:
        """请求停止并释放串口；激活或串口超时期间不能保证立即停止。"""
        self.cancel_motion()
        self._quit.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=7.0)

    def _input(self) -> tuple[bool, int, bool]:
        """取出最新输入和停止锁存，陈旧输入必须先释放才能恢复。"""
        with self._lock:
            activate, self._activate = self._activate, False
            stop, self._stop_pending = self._stop_pending, False
            if time.monotonic() - self._updated > 0.25:
                self._direction = 0
                self._inhibit = True
                self._pending_press = 0
                self._cancel_generation += 1
                stop = True
            return activate, self._direction, stop

    def _run(self) -> None:
        """独占底层驱动，异常后停止并禁止继续下发运动。"""
        device = None
        owned = False
        moving = 0
        try:
            device = self._factory()
            self._publish(state="connected")
            next_read = 0.0
            while not self._quit.is_set():
                activate, _, stop = self._input()
                if activate:
                    # 激活内含设备自检运动，失败时也必须尝试停止。
                    owned = True
                    device.activate(reset=False, start=False, refreshStatus=True)
                    device.stop(refreshStatus=True, readStatus=False)
                    with self._lock:
                        self._direction = 0
                        self._inhibit = True
                        self._status = replace(self._status, state="ready")
                if self._quit.is_set():
                    break
                if self.snapshot().state == "ready":
                    if stop:
                        if moving:
                            device.stop(refreshStatus=False, readStatus=False)
                        with self._lock:
                            self._continuous_direction = 0
                        self._target = None
                        self._publish(target=None)
                        moving = 0
                    elif self._motion_step(device):
                        moving = 1
                if time.monotonic() >= next_read:
                    position = validate_position_command(device.position())
                    if owned:
                        status = device.status(refreshStatus=False)
                        if status["gFLT"] != 0 or status["gSTA"] != 3 or status["gACT"] != 1:
                            raise RuntimeError(f"夹爪状态异常：{status}")
                    self._publish(position=position)
                    next_read = time.monotonic() + 0.1
                self._quit.wait(0.02)
        except Exception as exc:
            self._publish(state="fault", error=str(exc))
        finally:
            errors = []
            if device is not None:
                if owned:
                    try:
                        device.stop(refreshStatus=False, readStatus=False)
                    except Exception as exc:
                        errors.append(f"停止未确认：{exc}")
                try:
                    device.disconnect()
                except Exception as exc:
                    errors.append(f"断开失败：{exc}")
            if errors:
                previous = self.snapshot().error
                self._publish(state="fault", error="；".join(filter(None, [previous, *errors])))
            elif self.snapshot().state != "fault":
                self._publish(state="closed")


def create_device(port: str, baudrate: int, device_id: int):
    """创建固定版本 RTU 驱动，并限制单次串口等待、不重试运动命令。"""
    if version("pyrobotiqgripper") != "3.3.12":
        raise RuntimeError("遥控入口需要 pyrobotiqgripper==3.3.12。")
    from pymodbus.client import ModbusSerialClient
    from pyrobotiqgripper import RobotiqGripper

    class SerialGripper(RobotiqGripper):
        """为固定驱动版本提供有界串口等待和失败清理。"""

        def _create_modbus_client(self):
            """驱动的构造钩子；禁止扫描或自动选择串口。"""
            return ModbusSerialClient(
                port=port,
                baudrate=baudrate,
                parity="N",
                stopbits=1,
                bytesize=8,
                timeout=0.15,
                retries=0,
            )

        def __init__(self):
            """构造期间发生读失败时也释放已打开的串口。"""
            try:
                super().__init__(com_port=port, baudrate=baudrate, device_id=device_id)
            except Exception:
                if hasattr(self, "_client"):
                    self.disconnect()
                raise

    return SerialGripper()


class DemoDevice:
    """仅用于检查鼠标和手柄的虚拟设备，不访问串口。"""

    def __init__(self):
        """初始化虚拟位置和运动状态。"""
        self._position = 128.0
        self._target = self._position
        self._time = time.monotonic()

    def activate(self, **kwargs):
        """模拟激活，不产生真实运动。"""

    def move(self, position, **kwargs):
        """设置虚拟目标。"""
        self.position()
        self._target = position

    def position(self):
        """按固定速度更新虚拟位置。"""
        now = time.monotonic()
        step = (now - self._time) * 60
        self._time = now
        self._position += max(-step, min(step, self._target - self._position))
        return round(self._position)

    def stop(self, **kwargs):
        """在当前虚拟位置停止。"""
        self.position()
        self._target = self._position

    def status(self, **kwargs):
        """返回正常的虚拟设备状态。"""
        return {"gFLT": 0, "gSTA": 3, "gACT": 1}

    def disconnect(self):
        """虚拟设备没有串口需要释放。"""


def main(argv=None) -> None:
    """解析启动参数，显示本地控制窗口。"""
    parser = argparse.ArgumentParser(description="Robotiq 2F85 鼠标／手柄开闭控制")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--port", help="显式串口路径，例如 /dev/ttyUSB0；不自动扫描")
    mode.add_argument("--dry-run", action="store_true", help="模拟设备，检查鼠标和手柄")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--device-id", type=int, default=9)
    parser.add_argument("--speed", type=int, default=30, help="速度寄存器，0～255")
    parser.add_argument("--force", type=int, default=30, help="力寄存器，0～255，不是牛顿")
    parser.add_argument("--open-button", type=int, default=0, help="打开按钮编号")
    parser.add_argument("--close-button", type=int, default=1, help="闭合按钮编号")
    parser.add_argument("--joystick-index", type=int, default=0)
    parser.add_argument(
        "--motion-mode",
        choices=("continuous", "jog"),
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args(argv)
    if args.port == "auto":
        parser.error("请指定具体串口，不能使用 auto。")
    if args.baudrate <= 0 or not 1 <= args.device_id <= 247:
        parser.error("波特率必须为正整数，设备地址必须在 1～247 内。")
    if min(args.open_button, args.close_button, args.joystick_index) < 0:
        parser.error("手柄索引和按钮编号不能为负数。")
    if args.open_button == args.close_button:
        parser.error("打开和闭合必须使用不同按钮。")
    try:
        validate_position_command(args.speed)
        validate_position_command(args.force)
    except ValueError as exc:
        parser.error(str(exc))
    from .teleop_ui import run_panel

    factory = (
        DemoDevice
        if args.dry_run
        else lambda: create_device(args.port, args.baudrate, args.device_id)
    )
    if args.motion_mode is not None:
        print("已合并为短按单步、长按连续模式，旧 --motion-mode 参数不再切换行为。")
    controller = TeleopController(factory, speed=args.speed, force=args.force)
    controller.demo = args.dry_run
    controller.start()
    print(
        "模拟模式：不访问硬件。" if args.dry_run else f"真机串口：{args.port}。等待窗口内手动激活。"
    )
    try:
        run_panel(
            controller,
            open_button=args.open_button,
            close_button=args.close_button,
            joystick_index=args.joystick_index,
        )
    except KeyboardInterrupt:
        # Ctrl+C 属于主动退出；设备清理由下方 finally 保证执行。
        pass
    finally:
        controller.shutdown()
        status = controller.snapshot()
        if controller._thread.is_alive():
            print("串口操作仍在结束中；尚未确认停止，请使用设备断电方式确保停止。")
        elif status.error:
            print(status.error)


if __name__ == "__main__":
    main()

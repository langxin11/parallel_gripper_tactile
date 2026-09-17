"""DM 独立摇杆遥控：连续角目标与显式使能的设备线程。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import math
import threading
import time
from typing import Callable

from dm_grasp_core import CrankSliderKinematics, MITCommand, quintic_blend

from .config import DEFAULT_USB2CAN_PORT
from .deployment import make_dm4310p_gripper_config
from .protocol import MotorFeedback, STATUS_DISABLED, STATUS_ENABLED
from .session import DmSession


# 与 docs/dm-shared-control.md 及真机实验使用相同几何，保持遥控不依赖实验运行时。
_KINEMATICS = CrankSliderKinematics(
    theta0_rad=math.pi / 4,
    crank_radius_m=0.03,
    link_length_m=0.04,
    offset_m=0.021213203435596423,
)
_TORQUE_LIMIT_NM = make_dm4310p_gripper_config(DEFAULT_USB2CAN_PORT).motor_limits.torque_max_nm


@dataclass(frozen=True)
class TeleopConfig:
    """独立位置遥控参数；这些默认值不代表真机验收结论。"""

    max_speed_rad_s: float = 0.2
    deadzone: float = 0.12
    kp: float = 2.0
    kd: float = 0.5
    max_lead_rad: float | None = None
    feedforward_force_n: float = 1.0
    switch_duration_s: float = 0.25
    opening_feedforward_force_n: float | None = None

    def __post_init__(self):
        """校验输入整形、目标速度、MIT 增益及平均单侧前馈目标力。"""
        for name, lower, upper in (
            ("max_speed_rad_s", 0.0, 8.0),
            ("deadzone", 0.0, 1.0),
            ("kp", 0.0, 500.0),
            ("kd", 0.0, 5.0),
            ("switch_duration_s", 0.0, float("inf")),
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or not lower < value <= upper:
                raise ValueError(f"{name} 必须位于 ({lower}, {upper}]。")
        if self.deadzone >= 1:
            raise ValueError("摇杆死区必须小于 1。")
        lead = self.max_lead_rad
        if lead is not None and (
            isinstance(lead, bool)
            or not isinstance(lead, (int, float))
            or not math.isfinite(lead)
            or not 0 < lead <= math.pi / 2
        ):
            raise ValueError("max_lead_rad 必须为 None 或 (0, pi/2] 内的有限数值。")
        for force in (self.feedforward_force_n, self.opening_force_n):
            if (
                isinstance(force, bool)
                or not isinstance(force, (int, float))
                or not math.isfinite(force)
                or force < 0
            ):
                raise ValueError("平均单侧等效前馈目标力必须是非负有限数值，单位 N。")

    @property
    def opening_force_n(self) -> float:
        """返回张开等效目标力；未单独指定时沿用闭合设定。"""
        return (
            self.feedforward_force_n
            if self.opening_feedforward_force_n is None
            else self.opening_feedforward_force_n
        )

    def feedforward_torque(self, position_rad: float, *, opening: bool = False) -> float:
        """按实测角映射并限幅：闭合取正，张开取负，不代表测得的接触力。"""
        magnitude = min(
            _TORQUE_LIMIT_NM,
            _KINEMATICS.closure_jacobian(position_rad)
            * (self.opening_force_n if opening else self.feedforward_force_n),
        )
        return -magnitude if opening else magnitude

    def normalized_axis(self, value: float) -> float:
        """去除中心死区，再线性映射剩余行程到最大速度比例。"""
        if isinstance(value, bool) or not math.isfinite(value) or not -1 <= value <= 1:
            raise ValueError("摇杆轴必须为 [-1, 1] 内的有限数值。")
        magnitude = max(0.0, (abs(value) - self.deadzone) / (1 - self.deadzone))
        return math.copysign(magnitude, value) if magnitude else 0.0


class JointTeleopTarget:
    """生成连续角目标；反向先释放原偏差，回中暂停并保持目标与前馈比例。"""

    def __init__(self, config: TeleopConfig):
        """初始化无设备访问的目标生成器。"""
        self.config = config
        self.target: float | None = None
        self.feedforward_scale = 0.0
        self.phase = "holding"
        self._direction = 0
        self._releasing = False
        self._release_start = 0.0
        self._release_scale = 0.0
        self._release_elapsed = 0.0
        self._ramp_elapsed = 0.0

    def update(self, raw_axis: float, position_rad: float, dt_s: float) -> float:
        """按方向释放旧偏差、推进新目标，并用五次曲线渐变有符号前馈。"""
        if not math.isfinite(position_rad) or not math.isfinite(dt_s) or not 0 <= dt_s <= 0.1:
            raise ValueError("位置或控制周期非法，周期不得超过 0.1 秒。")
        axis = self.config.normalized_axis(raw_axis)
        position = min(math.pi / 2, max(0.0, position_rad))
        if self.target is None:
            self.target = position
        if not axis:
            # 回中不追随反馈，也不在用户松手后继续卸力或建立前馈。
            self.phase = "holding"
            return self.target

        direction = 1 if axis > 0 else -1
        if direction != self._direction:
            if self._direction:
                self._releasing = True
                self._release_start = self.target
                self._release_scale = self.feedforward_scale
                self._release_elapsed = 0.0
                self._direction = direction
                self.phase = "release_to_close" if direction > 0 else "release_to_open"
                # 反向识别帧保持原输出，下一周期开始渐变，避免入口跳变。
                return self.target
            self._direction = direction
            self._ramp_elapsed = 0.0

        if self._releasing:
            if self.phase == "holding":
                # 暂停期间反馈可能移动；从保持输出重新起步，不把反馈差一次性带入旧曲线。
                self._release_start = self.target
                self._release_scale = self.feedforward_scale
                self._release_elapsed = 0.0
                self.phase = "release_to_close" if direction > 0 else "release_to_open"
                return self.target
            self.phase = "release_to_close" if direction > 0 else "release_to_open"
            self._release_elapsed = min(self.config.switch_duration_s, self._release_elapsed + dt_s)
            # 五次多项式在端点附近可能产生浮点越界，不能让目标越过机械边界。
            blend = min(
                1.0, max(0.0, quintic_blend(self._release_elapsed / self.config.switch_duration_s))
            )
            # 终点使用最新反馈，避免释放期间实际关节移动后残留旧方向误差。
            self.target = (1 - blend) * self._release_start + blend * position
            self.feedforward_scale = (1 - blend) * self._release_scale
            if self._release_elapsed >= self.config.switch_duration_s:
                self._releasing = False
                self._ramp_elapsed = 0.0
            return self.target

        self.phase = "closing" if direction > 0 else "opening"
        delta = abs(axis) * self.config.max_speed_rad_s * dt_s
        if self.config.max_lead_rad is not None:
            # 可选限制只抑制继续累积，不反向跳变来追随限幅带。
            remaining = max(0.0, self.config.max_lead_rad - direction * (self.target - position))
            delta = min(delta, remaining)
        self.target = min(math.pi / 2, max(0.0, self.target + direction * delta))
        self._ramp_elapsed = min(self.config.switch_duration_s, self._ramp_elapsed + dt_s)
        self.feedforward_scale = direction * min(
            1.0, max(0.0, quintic_blend(self._ramp_elapsed / self.config.switch_duration_s))
        )
        return self.target


@dataclass(frozen=True)
class TeleopStatus:
    """供面板读取的原子状态快照。"""

    state: str = "disconnected"
    position_rad: float | None = None
    target_rad: float | None = None
    feedforward_torque_nm: float | None = None
    axis: float = 0.0
    error: str | None = None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None
    command: MITCommand | None = None
    motion_phase: str = "holding"


class DmTeleopController:
    """在唯一工作线程中运行 DM 会话，输入失效后失能并等待再次手动使能。"""

    def __init__(self, factory: Callable, config: TeleopConfig = TeleopConfig()):
        """组装会话工厂；构造不访问设备，必须显式 start。"""
        self.config = config
        self._factory = factory
        self._lock = threading.Lock()
        self._quit = threading.Event()
        self._status = TeleopStatus()
        self._axis = 0.0
        self._updated = float("-inf")
        self._enable = False
        self._disable = False
        self._generation = 0
        self._thread = threading.Thread(target=self._run, name="dm-teleop", daemon=True)

    def start(self):
        """显式启动设备线程。"""
        self._thread.start()

    def snapshot(self) -> TeleopStatus:
        """读取状态，不访问串口。"""
        with self._lock:
            return self._status

    def _publish(self, **changes):
        """原子更新面板状态。"""
        with self._lock:
            self._status = replace(self._status, **changes)

    def set_axis(self, raw_axis: float):
        """提交原始摇杆轴及心跳，非法输入先触发失能。"""
        try:
            self.config.normalized_axis(raw_axis)
        except (TypeError, ValueError):
            self.disable()
            raise
        with self._lock:
            self._axis = raw_axis
            self._updated = time.monotonic()
            self._status = replace(self._status, axis=raw_axis)

    def enable(self):
        """仅在已连接、最新输入处于中心时接受手动使能。"""
        with self._lock:
            if (
                self._status.state == "connected"
                and not self._disable
                and time.monotonic() - self._updated <= 0.25
                and self.config.normalized_axis(self._axis) == 0
            ):
                self._enable = True
                self._status = replace(self._status, state="enabling")

    def disable(self):
        """取消待使能并锁存失能请求。"""
        with self._lock:
            self._disable = True
            self._enable = False
            self._axis = 0.0
            self._generation += 1

    def shutdown(self):
        """请求失能和释放串口，等待有界设备操作结束。"""
        self.disable()
        self._quit.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=2.0)

    def _read_input(self):
        """读取一致的输入与请求，陈旧输入不允许继续执行。"""
        with self._lock:
            enable, self._enable = self._enable, False
            disable, self._disable = self._disable, False
            stale = time.monotonic() - self._updated > 0.25
            return enable, disable or stale, self._axis, self._generation

    def _run(self):
        """驱动显式生命周期；故障后停止运行，并单独报告失能确认失败。"""
        session = None
        owned = False
        try:
            session = self._factory()
            session.open()
            session.inspect()
            feedback = session.require_disabled()
            self._publish(
                state="connected",
                position_rad=feedback.position_rad,
                velocity_rad_s=feedback.velocity_rad_s,
                torque_nm=feedback.torque_nm,
            )
            target = JointTeleopTarget(self.config)
            previous = time.monotonic()
            next_inspect = previous + 0.2
            while not self._quit.is_set():
                started = time.monotonic()
                enable, disable, axis, generation = self._read_input()
                if disable:
                    if owned:
                        feedback = session.disable()
                        owned = False
                    self._publish(
                        state="connected",
                        target_rad=None,
                        position_rad=feedback.position_rad,
                        velocity_rad_s=feedback.velocity_rad_s,
                        torque_nm=feedback.torque_nm,
                        feedforward_torque_nm=None,
                        command=None,
                        motion_phase="holding",
                    )
                    target = JointTeleopTarget(self.config)
                    enable = False
                if enable:
                    # 发送使能即视为接管，确认失败也需要尽力失能。
                    owned = True
                    feedback = session.enable()
                    target = JointTeleopTarget(self.config)
                    previous = time.monotonic()
                    with self._lock:
                        cancelled = (
                            self._generation != generation
                            or self._quit.is_set()
                            or time.monotonic() - self._updated > 0.25
                            or self.config.normalized_axis(self._axis) != 0
                        )
                        if cancelled:
                            self._disable = True
                    if cancelled:
                        continue
                    self._publish(state="ready")
                if owned and not disable:
                    now = time.monotonic()
                    # 串口延迟期间的失焦／断连不能继续发送旧摇杆请求。
                    with self._lock:
                        if (
                            self._generation != generation
                            or self._disable
                            or self._quit.is_set()
                            or now - self._updated > 0.25
                        ):
                            continue
                        axis = self._axis
                    position = target.update(axis, feedback.position_rad, now - previous)
                    previous = now
                    command = MITCommand(
                        position,
                        0.0,
                        self.config.kp,
                        self.config.kd,
                        abs(target.feedforward_scale)
                        * self.config.feedforward_torque(
                            feedback.position_rad, opening=target.feedforward_scale < 0
                        ),
                    )
                    feedback = session.command(command)
                    self._publish(
                        position_rad=feedback.position_rad,
                        velocity_rad_s=feedback.velocity_rad_s,
                        torque_nm=feedback.torque_nm,
                        target_rad=position,
                        feedforward_torque_nm=command.feedforward_torque_nm,
                        command=command,
                        motion_phase=target.phase,
                    )
                elif started >= next_inspect:
                    feedback = session.require_disabled()
                    self._publish(
                        position_rad=feedback.position_rad,
                        velocity_rad_s=feedback.velocity_rad_s,
                        torque_nm=feedback.torque_nm,
                    )
                    next_inspect = started + 0.2
                self._quit.wait(max(0.0, 0.01 - (time.monotonic() - started)))
        except Exception as exc:
            self._publish(state="fault", error=str(exc))
        finally:
            errors = []
            if session is not None:
                if owned:
                    try:
                        feedback = session.disable()
                        self._publish(
                            position_rad=feedback.position_rad,
                            velocity_rad_s=feedback.velocity_rad_s,
                            torque_nm=feedback.torque_nm,
                            target_rad=None,
                            feedforward_torque_nm=None,
                            command=None,
                            motion_phase="holding",
                        )
                    except Exception as exc:
                        errors.append(f"失能未确认：{exc}")
                try:
                    session.close()
                except Exception as exc:
                    errors.append(f"串口关闭失败：{exc}")
            if errors:
                self._publish(
                    state="fault", error="；".join(filter(None, [self.snapshot().error, *errors]))
                )
            elif self.snapshot().state != "fault":
                self._publish(state="closed")


class DemoSession:
    """不访问设备的角位置演示会话，仅验证交互和目标推进。"""

    def __init__(self):
        """在行程中点创建失能的虚拟关节。"""
        self.position = math.pi / 4
        self.enabled = False
        self.last_feedback = None

    def _feedback(self):
        """创建虚拟设备反馈。"""
        self.last_feedback = MotorFeedback(
            self.position,
            0.0,
            0.0,
            STATUS_ENABLED if self.enabled else STATUS_DISABLED,
        )
        return self.last_feedback

    def open(self):
        """演示会话不打开串口。"""

    def close(self):
        """演示会话不占用外部资源。"""

    def inspect(self):
        """返回演示反馈。"""
        return self._feedback()

    def require_disabled(self):
        """校验演示关节处于失能状态。"""
        if self.enabled:
            raise RuntimeError("开始前电机必须处于失能状态。")
        return self._feedback()

    def enable(self):
        """显式使能虚拟关节。"""
        self.enabled = True
        return self._feedback()

    def command(self, command):
        """直接跟随目标；这不是动力学或接触力仿真。"""
        if not self.enabled:
            raise RuntimeError("演示关节尚未使能。")
        self.position = command.position_rad
        return self._feedback()

    def disable(self):
        """失能虚拟关节并返回确认。"""
        self.enabled = False
        return self._feedback()


def main(argv=None):
    """运行默认不访问硬件的独立摇杆控制面板。"""
    parser = argparse.ArgumentParser(description="DMgripper 连续关节角手柄遥控", allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="连接真实设备；仍须在窗口手动使能")
    mode.add_argument("--dry-run", action="store_true", help="模拟交互，不打开串口（默认）")
    parser.add_argument("--port", default=DEFAULT_USB2CAN_PORT)
    parser.add_argument("--axis", type=int, default=1, help="摇杆纵轴编号，默认 1")
    parser.add_argument("--joystick-index", type=int, default=0)
    parser.add_argument("--invert-axis", action="store_true", help="反转开闭方向")
    parser.add_argument("--max-speed", type=float, default=0.2, help="最大目标角速度，rad/s")
    parser.add_argument("--deadzone", type=float, default=0.12)
    parser.add_argument("--mit-kp", type=float, default=2.0)
    parser.add_argument("--mit-kd", type=float, default=0.5)
    parser.add_argument(
        "--switch-duration",
        type=float,
        default=0.25,
        help="反向释放及新方向前馈渐变各自的时长，秒（默认 0.25）；回中暂停计时",
    )
    parser.add_argument(
        "--feedforward-force",
        type=float,
        default=1.0,
        help="平均单侧等效前馈目标力，N；闭合为正、张开为负，按当前角度渐入（默认 1）",
    )
    parser.add_argument(
        "--opening-feedforward-force",
        type=float,
        default=None,
        help="单独指定张开等效前馈目标力的大小，N；默认沿用闭合设定，0 关闭张开前馈",
    )
    parser.add_argument(
        "--max-lead",
        type=float,
        default=None,
        help="可选目标领先限制，rad；默认不限制目标与反馈的角度偏差",
    )
    args = parser.parse_args(argv)
    if args.axis < 0 or args.joystick_index < 0:
        parser.error("手柄和轴索引不能为负数。")
    try:
        config = TeleopConfig(
            max_speed_rad_s=args.max_speed,
            deadzone=args.deadzone,
            kp=args.mit_kp,
            kd=args.mit_kd,
            max_lead_rad=args.max_lead,
            feedforward_force_n=args.feedforward_force,
            switch_duration_s=args.switch_duration,
            opening_feedforward_force_n=args.opening_feedforward_force,
        )
        make_dm4310p_gripper_config(args.port, timeout_s=0.05)
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    from .teleop_ui import run_panel

    factory = (lambda: DmSession(args.port, 0.05)) if args.execute else DemoSession
    controller = DmTeleopController(factory, config)
    controller.demo = not args.execute
    controller.start()
    print("真机模式：等待窗口内手动使能。" if args.execute else "模拟模式：不访问设备。")
    try:
        run_panel(
            controller,
            axis_index=args.axis,
            joystick_index=args.joystick_index,
            invert_axis=args.invert_axis,
        )
    finally:
        controller.shutdown()
        if controller._thread.is_alive():
            raise RuntimeError("设备线程仍未退出，尚不能确认失能。")
        if controller.snapshot().error:
            raise RuntimeError(controller.snapshot().error)


if __name__ == "__main__":
    main()

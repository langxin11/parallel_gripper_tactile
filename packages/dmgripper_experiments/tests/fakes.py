"""通用抓取实验测试的共享假设施。

FakeClock／FakeTactile／FakeDmSession 延续已验证的假设备思想：时钟由
注入的 sleep 与设备耗时显式推进，触觉按阶段产力，DM 反馈跟随 MIT
请求；全部替身不触碰真实串口。
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

from dmgripper_hardware import MotorFeedback, STATUS_DISABLED, STATUS_ENABLED

from dmgripper_experiments.config import (
    AdaptiveReferenceConfig,
    CurveReferenceConfig,
    ExperimentConfig,
    LifecycleConfig,
    ReferenceConfig,
    TimingConfig,
    WaypointConfig,
)
from dmgripper_experiments.tactile import TactileSnapshot


class FakeClock:
    """由注入 sleep 和测试替身显式推进的单调时钟。"""

    def __init__(self) -> None:
        """从零时刻开始。"""
        self.now_s = 0.0

    def __call__(self) -> float:
        """返回当前单调时间。"""
        return self.now_s

    def sleep(self, duration_s: float) -> None:
        """推进正的请求睡眠时间。"""
        self.now_s += max(0.0, duration_s)

    def advance(self, duration_s: float) -> None:
        """由伪设备模拟调用本身的耗时。"""
        self.now_s += duration_s


class PhaseActions:
    """依据真实运行时发出的状态事件提供交互动作。"""

    def __init__(self) -> None:
        """初始化当前阶段和释放轮次。"""
        self.phase = "preparing"
        self.holding_cycles = 0
        self.states: list[str] = []

    def event(self, payload: dict[str, object]) -> None:
        """从运行时事件中观察真实阶段迁移。"""
        if payload.get("event") == "state":
            self.phase = str(payload["phase"])
            self.states.append(self.phase)

    def __call__(self) -> str | None:
        """ready 时返回 start；holding 数轮后返回 release。"""
        if self.phase == "ready":
            self.phase = "start_requested"
            return "start"
        if self.phase == "holding":
            self.holding_cycles += 1
            if self.holding_cycles >= 4:
                return "release"
        return None


class CancelActions:
    """使能前请求释放的取消场景。"""

    phase = "ready"

    def __init__(self) -> None:
        """准备一次性 release。"""
        self.released = False

    def event(self, payload: dict[str, object]) -> None:
        """接受事件以兼容事件 sink 签名。"""

    def __call__(self) -> str | None:
        """仅在第一次被询问时返回 release。"""
        if not self.released:
            self.released = True
            return "release"
        return None


class ClosedInputActions(PhaseActions):
    """在已使能阶段模拟交互输入关闭。"""

    def __call__(self) -> str | None:
        """接近开始后报告输入关闭。"""
        if self.phase == "approach":
            raise RuntimeError("交互输入已关闭")
        return super().__call__()


class FakeTactile:
    """按运行阶段产生新鲜双侧触觉快照的无 I/O 替身。"""

    def __init__(self, _serial_config, *, clock, sample_sink=None, phase, **_kwargs) -> None:
        """保存时钟、采样 sink 和状态观察器。"""
        self.clock = clock
        self.sample_sink = sample_sink
        self.phase = phase
        self.counter = 0
        self.stopped = False

    def start(self) -> None:
        """标记采集已启动。"""

    def stop(self) -> None:
        """标记采集已停止。"""
        self.stopped = True

    def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
        """为空载校验生成有递增设备时间的新快照。"""
        self.clock.advance(0.01)
        return self._snapshot(force_n=0.0)

    def latest(self) -> TactileSnapshot:
        """返回当前阶段的完整新鲜快照。"""
        phase = self.phase.phase
        idle = phase in {"preparing", "ready", "start_requested"}
        force_n = 0.0 if idle else 0.5
        tangential_n = 1.0 if phase == "active" else 0.0
        return self._snapshot(force_n=force_n, tangential_n=tangential_n)

    def _snapshot(self, *, force_n: float, tangential_n: float = 0.0) -> TactileSnapshot:
        """构造一帧完整三轴力，并写入原始采样 sink。"""
        self.counter += 1
        snapshot = TactileSnapshot(
            received_at_s=self.clock(),
            packet_counter=self.counter,
            timestamp_us=self.counter * 10_000,
            left_force_n=force_n,
            right_force_n=force_n,
            raw_left_fz_n=force_n,
            raw_right_fz_n=force_n,
            raw_left_fx_n=0.0,
            raw_left_fy_n=tangential_n,
            raw_right_fx_n=0.0,
            raw_right_fy_n=0.0,
        )
        if self.sample_sink is not None:
            self.sample_sink({"counter": self.counter, "timestamp_us": snapshot.timestamp_us})
        return snapshot


class BadTactile(FakeTactile):
    """在指定阶段注入触觉保护故障。"""

    mode = "stale"
    target_phase = "approach"

    def latest(self) -> TactileSnapshot:
        """仅在目标阶段返回指定保护故障。"""
        snapshot = super().latest()
        if self.phase.phase != self.target_phase:
            return snapshot
        from dataclasses import replace

        if self.mode == "stale":
            return replace(snapshot, received_at_s=self.clock() - 1.0)
        if self.mode == "nan":
            return replace(snapshot, raw_left_fx_n=math.nan)
        if self.mode == "overforce":
            return replace(snapshot, raw_left_fz_n=3.0)
        if self.mode == "rollback":
            return replace(snapshot, timestamp_us=1)
        if self.mode == "lost":
            return replace(snapshot, left_force_n=0.0, raw_left_fz_n=0.0)
        raise AssertionError(f"未知触觉故障模式：{self.mode}")


class TransientLostTactile(FakeTactile):
    """在跟踪阶段短暂失去双侧接触后自动恢复，用于重接近验证。"""

    def __init__(self, *args, **kwargs) -> None:
        """初始化丢失窗口计数。"""
        super().__init__(*args, **kwargs)
        self._lost_done = False

    def latest(self) -> TactileSnapshot:
        """active 阶段前若干周期返回零力，制造一次可恢复的失接触。"""
        snapshot = super().latest()
        if self.phase.phase == "active" and not self._lost_done:
            self._lost_done = True
            self._lost_cycles = 6
        if getattr(self, "_lost_cycles", 0) > 0:
            self._lost_cycles -= 1
            from dataclasses import replace

            return replace(
                snapshot,
                left_force_n=0.0,
                right_force_n=0.0,
                raw_left_fz_n=0.0,
                raw_right_fz_n=0.0,
            )
        return snapshot


class FakeDmSession:
    """跟随 MIT 目标的无硬件 DM 会话。"""

    def __init__(
        self,
        _port: str,
        *,
        timeout_s: float,
        clock: FakeClock | None = None,
        enable_error: bool = False,
        disable_error: bool = False,
    ) -> None:
        """保存部署数据、当前反馈和故障注入选项。"""
        del timeout_s
        self.clock = clock
        self.enable_error = enable_error
        self.disable_error = disable_error
        self.deployment = SimpleNamespace(
            joint_position_min_rad=0.0,
            joint_position_max_rad=math.pi / 2,
            closing_direction=1,
        )
        self.feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_DISABLED)
        self.opened = False
        self.closed = False
        self.disable_calls = 0
        self.command_count = 0

    def open(self) -> None:
        """标记串口打开。"""
        self.opened = True

    def close(self) -> None:
        """标记串口关闭。"""
        self.closed = True

    def inspect(self) -> MotorFeedback:
        """返回启动前反馈。"""
        return self.feedback

    def require_disabled(self) -> MotorFeedback:
        """确认失能初始状态。"""
        if self.feedback.status_code != STATUS_DISABLED:
            raise RuntimeError("开始前电机必须处于失能状态")
        return self.feedback

    def enable(self) -> MotorFeedback:
        """模拟使能确认或确认丢失。"""
        if self.enable_error:
            raise RuntimeError("使能确认丢失")
        self.feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
        return self.feedback

    def command(self, command) -> MotorFeedback:
        """让反馈跟随纯控制核的 MIT 请求。"""
        self.command_count += 1
        self.feedback = MotorFeedback(
            command.position_rad,
            command.velocity_rad_s,
            command.feedforward_torque_nm,
            STATUS_ENABLED,
        )
        return self.feedback

    def disable(self) -> MotorFeedback:
        """记录每一次安全失能请求。"""
        self.disable_calls += 1
        if self.disable_error:
            raise RuntimeError("失能确认失败")
        self.feedback = MotorFeedback(0.0, 0.0, 0.0, STATUS_DISABLED)
        return self.feedback


def curve_config(**lifecycle_overrides) -> ExperimentConfig:
    """返回可快速执行全部阶段的曲线模式配置。"""
    lifecycle = LifecycleConfig(
        zero_force_stable_s=0.01,
        zero_force_timeout_s=0.2,
        contact_on_stable_s=0.01,
        contact_transition_s=0.01,
        contact_off_stable_s=0.02,
        preload_stable_time_s=0.02,
        preload_timeout_s=1.0,
        approach_endpoint_hold_s=1.0,
        return_closure_velocity_m_s=0.1,
        return_closure_acceleration_m_s2=0.5,
        return_closure_jerk_m_s3=5.0,
        return_settle_timeout_s=1.0,
        **lifecycle_overrides,
    )
    return ExperimentConfig(
        timing=TimingConfig(
            control_rate_hz=100.0,
            max_control_gap_s=0.05,
            tactile_timeout_s=0.1,
        ),
        lifecycle=lifecycle,
        reference=ReferenceConfig(
            curve=CurveReferenceConfig(
                waypoints=(
                    WaypointConfig(t_s=0.0, force_n=0.5),
                    WaypointConfig(t_s=0.03, force_n=0.5),
                )
            )
        ),
    )


def adaptive_config(**lifecycle_overrides) -> ExperimentConfig:
    """返回可快速执行全部阶段的动态增力配置。"""
    return ExperimentConfig(
        timing=TimingConfig(
            control_rate_hz=100.0,
            max_control_gap_s=0.05,
            tactile_timeout_s=0.1,
        ),
        lifecycle=LifecycleConfig(
            zero_force_stable_s=0.01,
            zero_force_timeout_s=0.2,
            contact_on_stable_s=0.01,
            contact_transition_s=0.01,
            contact_off_stable_s=0.02,
            preload_stable_time_s=0.02,
            preload_timeout_s=1.0,
            approach_endpoint_hold_s=1.0,
            return_closure_velocity_m_s=0.1,
            return_closure_acceleration_m_s2=0.5,
            return_closure_jerk_m_s3=5.0,
            return_settle_timeout_s=1.0,
            **lifecycle_overrides,
        ),
        reference=ReferenceConfig(
            adaptive=AdaptiveReferenceConfig(
                initial_force_n=0.5,
                duration_s=0.03,
                max_force_n=1.5,
            )
        ),
    )


def make_config(kind: str = "curve", **lifecycle_overrides) -> ExperimentConfig:
    """按目标模式返回快速配置。"""
    if kind == "adaptive":
        return adaptive_config(**lifecycle_overrides)
    return curve_config(**lifecycle_overrides)


def run_fake_experiment(
    tmp_path: Path,
    config: ExperimentConfig,
    *,
    tactile_type=FakeTactile,
    actions_type: type[PhaseActions] = PhaseActions,
    enable_error: bool = False,
    disable_error: bool = False,
    command_delay_s: float = 0.0,
    plots: bool = False,
    terminal=None,
    event_observer=None,
):
    """以注入的纯内存会话和触觉工厂执行一次运行并返回关键替身。"""
    import dataclasses

    from dmgripper_experiments.recording import create_run_directory
    from dmgripper_experiments.runtime import run_experiment

    if not plots:
        config = dataclasses.replace(config, output=dataclasses.replace(config.output, plots=False))
    clock = FakeClock()
    actions = actions_type()
    session_holder: list[FakeDmSession] = []

    def session_factory(port: str, *, timeout_s: float) -> FakeDmSession:
        session = FakeDmSession(
            port,
            timeout_s=timeout_s,
            clock=clock,
            enable_error=enable_error,
            disable_error=disable_error,
        )
        session_holder.append(session)
        original_command = session.command

        def command_with_delay(command):
            result = original_command(command)
            clock.advance(command_delay_s)
            return result

        session.command = command_with_delay
        return session

    def tactile_factory(serial_config, **kwargs):
        return tactile_type(serial_config, phase=actions, **kwargs)

    def event_sink(payload: dict[str, object]) -> None:
        actions.event(payload)
        if event_observer is not None:
            event_observer(payload, session_holder[0])

    output_directory = create_run_directory(tmp_path, config)
    result = run_experiment(
        config,
        output_directory=output_directory,
        clear_bias=False,
        action_source=actions,
        event_sink=event_sink,
        clock=clock,
        sleep=clock.sleep,
        session_factory=session_factory,
        tactile_factory=tactile_factory,
        terminal=terminal,
    )
    assert session_holder, "必须创建 DM 会话"
    return result, session_holder[0], actions, output_directory

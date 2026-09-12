"""离线验证真机倒水运行时的阶段、记录和失能收尾。"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from dmgripper_hardware import MotorFeedback, STATUS_DISABLED, STATUS_ENABLED

from dmgripper_experiments.config import ForceDemoConfig
from dmgripper_experiments.cup_config import CupConfig, GripConfig
from dmgripper_experiments.cup_runtime import _verify_zero, run_cup
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
        """初始化当前状态和各阶段轮次。"""
        self.phase = "zero_check"
        self.takeover_cycles = 0
        self.await_release_cycles = 0
        self.states: list[str] = []

    def event(self, payload: dict[str, object]) -> None:
        """从运行时事件中观察真实状态迁移。"""
        if payload.get("event") == "state":
            self.phase = str(payload["state"])
            self.states.append(self.phase)

    def __call__(self) -> str | None:
        """按阶段返回 ready 或 release，其余时间保持等待。"""
        if self.phase == "place_cup":
            self.phase = "approach_requested"
            return "ready"
        if self.phase == "takeover":
            self.takeover_cycles += 1
            if self.takeover_cycles >= 3:
                return "ready"
        if self.phase == "await_release":
            self.await_release_cycles += 1
            if self.await_release_cycles >= 4:
                return "release"
        return None


class ClosedInputActions(PhaseActions):
    """在已使能的接管阶段模拟交互输入关闭。"""

    def __call__(self) -> str | None:
        """撤手前保持正常，在接管时报告输入关闭。"""
        if self.phase == "takeover":
            return "__input_closed__"
        return super().__call__()


class FakeTactile:
    """按运行阶段产生新鲜双侧触觉快照的无 I/O 替身。"""

    def __init__(
        self, _serial_config, *, clock, sample_sink=None, phase: PhaseActions, **_kwargs
    ) -> None:
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
        force_n = 0.0 if phase in {"zero_check", "place_cup"} else 0.5
        return self._snapshot(force_n=force_n, tangential_n=1.0 if phase == "takeover" else 0.0)

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


class FakeDmSession:
    """跟随 MIT 目标的无硬件 DM 会话。"""

    def __init__(
        self,
        _port: str,
        *,
        timeout_s: float,
        clock: FakeClock,
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
        """返回启动前失能反馈。"""
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


def _config() -> CupConfig:
    """返回可快速执行全部阶段的有效实验配置。"""
    control = ForceDemoConfig(
        tracking_duration_s=0.03,
        control_rate_hz=100.0,
        tactile_timeout_s=0.1,
        zero_force_stable_s=0.01,
        zero_force_timeout_s=0.2,
        contact_on_stable_s=0.01,
        contact_transition_s=0.01,
        contact_off_stable_s=0.02,
        approach_endpoint_hold_s=1.0,
        return_closure_velocity_m_s=0.1,
        return_closure_acceleration_m_s2=0.5,
        return_closure_jerk_m_s3=5.0,
        return_settle_timeout_s=1.0,
    )
    return CupConfig(
        control=control,
        grip=GripConfig(stable_time_s=0.02),
        max_control_gap_s=0.05,
    )


def _run(
    tmp_path: Path,
    *,
    tactile_type=FakeTactile,
    enable_error: bool = False,
    command_delay_s: float = 0.0,
    sessions: list[FakeDmSession] | None = None,
    actions_type: type[PhaseActions] = PhaseActions,
    actions_out: list[PhaseActions] | None = None,
    disable_error: bool = False,
) -> tuple[dict[str, object], FakeDmSession, PhaseActions]:
    """以注入的纯内存会话和触觉工厂执行一次运行。"""
    clock = FakeClock()
    actions = actions_type()
    if actions_out is not None:
        actions_out.append(actions)
    session: FakeDmSession | None = None

    def session_factory(port: str, *, timeout_s: float) -> FakeDmSession:
        nonlocal session
        session = FakeDmSession(
            port,
            timeout_s=timeout_s,
            clock=clock,
            enable_error=enable_error,
            disable_error=disable_error,
        )
        if sessions is not None:
            sessions.append(session)
        original_command = session.command

        def command_with_delay(command):
            result = original_command(command)
            clock.advance(command_delay_s)
            return result

        session.command = command_with_delay
        return session

    def tactile_factory(serial_config, **kwargs):
        return tactile_type(serial_config, phase=actions, **kwargs)

    result = run_cup(
        _config(),
        output_directory=tmp_path / "run",
        clear_bias=False,
        action_source=actions,
        event_sink=actions.event,
        clock=clock,
        sleep=clock.sleep,
        session_factory=session_factory,
        tactile_factory=tactile_factory,
    )
    assert session is not None
    return result, session, actions


def test_run_cup_completes_all_interactive_phases_and_records_trace(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """空载到 release 回位全链路应保持 await_release 并最终安全失能。"""
    monkeypatch.setattr("dmgripper_experiments.cup_runtime.plot_cup_run", lambda _directory: ())

    result, session, actions = _run(tmp_path)

    assert result["completed"] is True
    assert session.disable_calls == 1
    assert session.closed
    assert actions.states == [
        "zero_check",
        "place_cup",
        "approach",
        "contact_transition",
        "supported_hold",
        "takeover",
        "stabilizing",
        "pour",
        "await_release",
        "return",
        "complete",
    ]
    assert actions.await_release_cycles >= 4
    with (tmp_path / "run" / "trace.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    states = [row["state"] for row in rows]
    assert "takeover" in states and "await_release" in states and "return" in states
    assert max(float(row["target_force_n"]) for row in rows) > 0.5
    assert all(row["force_deadband_active"] in {"True", "False"} for row in rows)
    assert all(row["unloading_blocked"] in {"True", "False"} for row in rows)
    return_positions = [float(row["q_des_rad"]) for row in rows if row["state"] == "return"]
    assert return_positions[-1] < return_positions[0]
    assert all(row["unloading_blocked"] == "False" for row in rows if row["state"] == "return")
    manifest = json.loads((tmp_path / "run" / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "completed"


class BadTactile(FakeTactile):
    """在使能后的接近阶段注入指定触觉保护故障。"""

    mode = "stale"

    def latest(self) -> TactileSnapshot:
        """仅在接近阶段返回指定保护故障。"""
        snapshot = super().latest()
        applies_after_contact = self.mode == "lost" and self.phase.phase == "supported_hold"
        if self.mode == "lost" and not applies_after_contact:
            return snapshot
        if self.phase.phase != "approach" and not applies_after_contact:
            return snapshot
        if self.mode == "stale":
            return replace(snapshot, received_at_s=self.clock() - 1.0)
        if self.mode == "nan":
            return replace(snapshot, raw_left_fx_n=math.nan)
        if self.mode == "overforce":
            return replace(snapshot, raw_left_fz_n=3.0)
        if self.mode == "rollback":
            return replace(snapshot, timestamp_us=1)
        if applies_after_contact:
            return replace(snapshot, left_force_n=0.0, raw_left_fz_n=0.0)
        if self.mode == "repeated":
            if not hasattr(self, "_repeated"):
                self._repeated = snapshot
            return self._repeated
        raise AssertionError(f"未知触觉故障模式：{self.mode}")


@pytest.mark.parametrize(
    ("failure", "expected"),
    [
        ("enable", "使能确认丢失"),
        ("stale", "触觉快照过期"),
        ("nan", "触觉三轴力缺失或包含非有限数值"),
        ("latency", "命令反馈或控制记录超时"),
        ("overforce", "原始法向力超过保护上限"),
        ("rollback", "触觉设备时间未递增或设备发生重启"),
        ("lost", "已接触后至少一侧持续失去接触"),
    ],
)
def test_run_cup_failures_disable_and_write_failed_manifest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure: str, expected: str
) -> None:
    """关键使能、触觉与控制时序故障均应在收尾时尝试失能并保留失败记录。"""
    monkeypatch.setattr("dmgripper_experiments.cup_runtime.plot_cup_run", lambda _directory: ())
    tactile_type = FakeTactile
    if failure in {"stale", "nan", "overforce", "rollback", "lost"}:
        tactile_type = type(f"{failure.title()}Tactile", (BadTactile,), {"mode": failure})
    sessions: list[FakeDmSession] = []

    with pytest.raises(RuntimeError, match=expected):
        _run(
            tmp_path,
            tactile_type=tactile_type,
            enable_error=failure == "enable",
            command_delay_s=0.1 if failure == "latency" else 0.0,
            sessions=sessions,
        )

    assert len(sessions) == 1
    assert sessions[0].disable_calls == 1
    assert sessions[0].closed
    manifest = json.loads((tmp_path / "run" / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"


def test_repeated_snapshot_does_not_confirm_contact_before_staleness(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """重复快照不得被重复计入接触确认，最终应以新鲜度保护退出。"""
    monkeypatch.setattr("dmgripper_experiments.cup_runtime.plot_cup_run", lambda _directory: ())
    tactile_type = type("RepeatedTactile", (BadTactile,), {"mode": "repeated"})
    sessions: list[FakeDmSession] = []
    actions_out: list[PhaseActions] = []

    with pytest.raises(RuntimeError, match="触觉快照过期"):
        _run(tmp_path, tactile_type=tactile_type, sessions=sessions, actions_out=actions_out)

    assert sessions[0].disable_calls == 1
    assert "takeover" not in actions_out[0].states


def test_input_closed_and_disable_failure_preserve_original_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """输入关闭后的失能失败须同时保留原始错误和清理错误。"""
    monkeypatch.setattr("dmgripper_experiments.cup_runtime.plot_cup_run", lambda _directory: ())
    sessions: list[FakeDmSession] = []

    with pytest.raises(RuntimeError, match="原始错误：交互输入已关闭；失能失败：失能确认失败"):
        _run(
            tmp_path,
            sessions=sessions,
            actions_type=ClosedInputActions,
            disable_error=True,
        )

    assert sessions[0].disable_calls == 1
    manifest = json.loads((tmp_path / "run" / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert "交互输入已关闭" in manifest["error"]["message"]


def test_verify_zero_uses_filtered_fz_and_tolerates_raw_noise() -> None:
    """零力门禁与 ROS 一致使用滤波 Fz，原始三轴噪声不应反复清空窗口。"""
    clock = FakeClock()
    actions = PhaseActions()

    class NoisyTactile(FakeTactile):
        """持续返回滤波力为零、原始三轴合力超过零力阈值的快照。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            """推进时钟并构造含原始噪声尖峰的触觉帧。"""
            self.clock.advance(0.01)
            snapshot = self._snapshot(force_n=0.0)
            return replace(snapshot, raw_left_fz_n=0.2, raw_right_fz_n=0.2)

    tactile = NoisyTactile(None, clock=clock, phase=actions)
    _verify_zero(tactile, _config().control, False, clock, clock.sleep)


def test_verify_zero_reports_filtered_force_timeout_instead_of_snapshot_timeout() -> None:
    """持续滤波残余力应报告零力统计，期限末端不能误报触觉断流。"""
    clock = FakeClock()
    actions = PhaseActions()

    class LoadedTactile(FakeTactile):
        """持续返回略高于零力阈值的滤波法向力。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            """推进时钟并构造持续受载触觉帧。"""
            self.clock.advance(0.01)
            return self._snapshot(force_n=0.11)

    tactile = LoadedTactile(None, clock=clock, phase=actions)
    with pytest.raises(RuntimeError, match=r"滤波双侧 Fz.*LEFT=\+0\.110N.*最长稳定=0\.000"):
        _verify_zero(tactile, _config().control, False, clock, clock.sleep)

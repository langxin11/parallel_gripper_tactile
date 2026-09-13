"""通用运行时的端到端假设备验证：两类目标 × 三控制器、故障与清理。"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import pytest
from dmgripper_hardware import STATUS_DISABLED

from .fakes import (
    CancelActions,
    ClosedInputActions,
    FakeTactile,
    PhaseActions,
    adaptive_config,
    curve_config,
    make_config,
    run_fake_experiment,
)

from dmgripper_experiments.config import EstimationConfig


def _read_rows(directory: Path) -> list[dict[str, str]]:
    with (directory / "trace.csv").open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _read_events(directory: Path) -> list[dict[str, object]]:
    return [
        json.loads(line)
        for line in (directory / "events.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


@pytest.mark.parametrize("controller", ["admittance", "pid", "adrc"])
@pytest.mark.parametrize("kind", ["curve", "adaptive"])
def test_full_lifecycle_completes_for_all_six_combinations(
    tmp_path: Path, kind: str, controller: str
) -> None:
    """两类目标来源与三种控制器的组合都能走完全部阶段并安全失能。"""
    import dataclasses

    from dmgripper_experiments.config import ControllerConfig

    config = dataclasses.replace(
        make_config(kind), controller=dataclasses.replace(ControllerConfig(), kind=controller)
    )
    result, session, actions, directory = run_fake_experiment(tmp_path, config)
    assert result["status"] == "completed"
    assert result["disable_confirmed"] is True
    assert session.disable_calls == 1
    assert session.closed
    assert actions.states == [
        "preparing",
        "ready",
        "approach",
        "contact_transition",
        "preload",
        "active",
        "holding",
        "returning",
        "completed",
    ]
    rows = _read_rows(directory)
    phases = [row["phase"] for row in rows]
    assert "preload" in phases and "active" in phases and "returning" in phases
    assert all(math.isfinite(float(row["target_force_n"])) for row in rows if row["target_force_n"])
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["schema"] == "dmgripper-experiment/v1"
    assert manifest["status"] == "completed"
    assert manifest["disable_confirmed"] is True


def test_adaptive_target_increases_under_tangential_load(tmp_path: Path) -> None:
    """动态模式在 active 阶段因切向载荷增加目标力并记录触发。"""
    result, _session, _actions, directory = run_fake_experiment(tmp_path, adaptive_config())
    assert result["status"] == "completed"
    rows = _read_rows(directory)
    active_targets = [float(row["target_force_n"]) for row in rows if row["phase"] == "active"]
    assert active_targets, "active 阶段必须有 trace 记录"
    assert max(active_targets) > 0.5, "切向载荷应当驱动目标力增长"
    assert any(row["target_trigger_active"] == "True" for row in rows if row["phase"] == "active")


def test_preload_timeout_reports_force_and_blocked_unloading(tmp_path: Path) -> None:
    """预载过冲且禁止卸载时，超时错误直接报告力与相容性线索。"""
    from dataclasses import replace

    class HighPreloadTactile(FakeTactile):
        """进入 preload 后持续返回高于稳定容差的法向力。"""

        def latest(self):
            snapshot = super().latest()
            if self.phase.phase != "preload":
                return snapshot
            return replace(
                snapshot,
                left_force_n=0.8,
                right_force_n=0.7,
                raw_left_fz_n=0.8,
                raw_right_fz_n=0.7,
            )

    with pytest.raises(
        RuntimeError,
        match=(
            r"目标=0\.500N，当前均值=0\.750N.*允许区间=0\.350–0\.650N；"
            r"导纳 prevent_unloading=true"
        ),
    ):
        config = adaptive_config()
        config = replace(config, lifecycle=replace(config.lifecycle, preload_timeout_s=0.05))
        run_fake_experiment(tmp_path, config, tactile_type=HighPreloadTactile)


def test_asymmetric_preload_tolerance_accepts_safe_overforce(tmp_path: Path) -> None:
    """只放宽高侧容差时，0.75 N 预载可稳定进入 active。"""
    from dataclasses import replace

    class HighPreloadTactile(FakeTactile):
        """仅在 preload 阶段返回 0.75 N 平均力。"""

        def latest(self):
            snapshot = super().latest()
            if self.phase.phase != "preload":
                return snapshot
            return replace(
                snapshot,
                left_force_n=0.8,
                right_force_n=0.7,
                raw_left_fz_n=0.8,
                raw_right_fz_n=0.7,
            )

    config = adaptive_config()
    config = replace(
        config,
        lifecycle=replace(config.lifecycle, preload_overforce_tolerance_n=0.3),
    )
    result, _session, actions, _directory = run_fake_experiment(
        tmp_path,
        config,
        tactile_type=HighPreloadTactile,
    )
    assert result["status"] == "completed"
    assert "active" in actions.states


def test_curve_mode_holds_final_target_in_holding(tmp_path: Path) -> None:
    """曲线模式任务结束后保持末目标等待释放。"""
    result, _session, _actions, directory = run_fake_experiment(tmp_path, curve_config())
    assert result["status"] == "completed"
    rows = _read_rows(directory)
    holding = [row for row in rows if row["phase"] == "holding"]
    assert holding
    assert all(float(row["target_force_n"]) == pytest.approx(0.5) for row in holding)
    task_times = [float(row["task_time_s"]) for row in rows if row["task_time_s"]]
    assert task_times == sorted(task_times)


def test_release_before_enable_cancels_without_motion(tmp_path: Path) -> None:
    """使能前 release 记为 cancelled：不开电机、不发运动命令。"""
    result, session, actions, directory = run_fake_experiment(
        tmp_path, curve_config(), actions_type=CancelActions
    )
    assert result["status"] == "cancelled"
    assert result["disable_confirmed"] == "not_applicable"
    assert session.disable_calls == 0
    assert session.command_count == 0
    assert not session.opened, "使能前取消不应打开 DM 串口"
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "cancelled"
    assert manifest["disable_confirmed"] == "not_applicable"


def test_start_command_is_recorded_before_motor_approach(tmp_path: Path) -> None:
    """收到 start 后先留存确认事件，再连接使能并进入 approach。"""
    device_state_at_acknowledgement: list[tuple[bool, int]] = []

    def observe_event(event: dict[str, object], session) -> None:
        if event.get("event") == "command_received" and event.get("action") == "start":
            device_state_at_acknowledgement.append((session.opened, session.feedback.status_code))

    result, session, _actions, directory = run_fake_experiment(
        tmp_path,
        curve_config(),
        event_observer=observe_event,
    )
    assert result["status"] == "completed"
    assert session.command_count > 0
    assert device_state_at_acknowledgement == [(False, STATUS_DISABLED)]
    events = _read_events(directory)
    acknowledgement = next(
        index
        for index, event in enumerate(events)
        if event.get("event") == "command_received" and event.get("action") == "start"
    )
    approach = next(
        index
        for index, event in enumerate(events)
        if event.get("event") == "state" and event.get("phase") == "approach"
    )
    assert acknowledgement < approach
    assert "正在连接、检查并使能电机" in str(events[acknowledgement]["message"])


def test_ready_reports_status_and_unknown_command_before_start(tmp_path: Path) -> None:
    """ready 中 status 与拼错命令都有反馈，且随后仍可正常 start。"""

    class ReportingActions(PhaseActions):
        def __init__(self) -> None:
            super().__init__()
            self.pending = ["__input_closed__", "statuz", "status", "start"]

        def __call__(self) -> str | None:
            if self.phase == "ready" and self.pending:
                action = self.pending.pop(0)
                if action == "start":
                    self.phase = "start_requested"
                return action
            return super().__call__()

    result, _session, _actions, directory = run_fake_experiment(
        tmp_path,
        curve_config(),
        actions_type=ReportingActions,
    )
    assert result["status"] == "completed"
    events = _read_events(directory)
    warnings = [event for event in events if event.get("code") == "unknown_command"]
    assert [warning["action"] for warning in warnings] == ["__input_closed__", "statuz"]
    assert all("start、status、release" in str(warning["message"]) for warning in warnings)
    assert any(event.get("event") == "status" and event.get("phase") == "ready" for event in events)


def test_auto_start_and_return_complete_without_interactive_actions(tmp_path: Path) -> None:
    """无人值守组合不依赖标准输入，自动启动并在任务结束后回位。"""

    class PassiveActions(PhaseActions):
        def __call__(self) -> None:
            return None

    result, session, actions, directory = run_fake_experiment(
        tmp_path,
        curve_config(auto_start=True, on_finished="return"),
        actions_type=PassiveActions,
    )
    assert result["status"] == "completed"
    assert session.disable_calls == 1
    assert actions.states[-3:] == ["holding", "returning", "completed"]
    events = _read_events(directory)
    assert any(
        event.get("event") == "command_received" and event.get("action") == "auto_start"
        for event in events
    )


def test_zero_force_peak_warning_is_recorded_without_blocking_start(tmp_path: Path) -> None:
    """零力均值合格时，接触级峰值只记录告警且仍可进入完整生命周期。"""
    from .fakes import FakeTactile

    class PeakThenQuietTactile(FakeTactile):
        """零力验证首帧有峰值，后续帧保持空载。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s):
            self.clock.advance(0.01)
            force_n = 0.2 if self.counter == 0 else 0.0
            return self._snapshot(force_n=force_n)

    result, _session, _actions, directory = run_fake_experiment(
        tmp_path,
        curve_config(),
        tactile_type=PeakThenQuietTactile,
    )
    assert result["status"] == "completed"
    events = _read_events(directory)
    warnings = [event for event in events if event["event"] == "warning"]
    assert len(warnings) == 1
    assert warnings[0]["code"] == "zero_force_peak"
    assert warnings[0]["peak_side"] == "both"
    assert warnings[0]["maximum_peak_n"] == pytest.approx(0.2)
    assert events.index(warnings[0]) < next(
        index
        for index, event in enumerate(events)
        if event.get("event") == "state" and event.get("phase") == "ready"
    )


@pytest.mark.parametrize(
    ("failure", "expected"),
    [
        ("enable", "使能确认丢失"),
        ("stale", "触觉快照过期"),
        ("nan", "触觉三轴力缺失或包含非有限数值"),
        ("latency", "命令反馈或控制记录超时"),
        ("overforce", "原始法向力超过保护上限"),
        ("rollback", "触觉设备时间未递增或设备发生重启"),
        ("lost", "跟踪阶段持续失去接触"),
    ],
)
def test_failures_disable_and_write_failed_manifest(
    tmp_path: Path, failure: str, expected: str
) -> None:
    """关键使能、触觉与控制时序故障均在收尾时尽力失能并保留失败记录。"""
    from .fakes import BadTactile

    tactile_type = FakeTactile
    if failure == "lost":
        tactile_type = type(
            "LostTactile", (BadTactile,), {"mode": "lost", "target_phase": "active"}
        )
    elif failure in {"stale", "nan", "overforce", "rollback"}:
        tactile_type = type(
            f"{failure.title()}Tactile",
            (BadTactile,),
            {"mode": failure, "target_phase": "approach"},
        )
    with pytest.raises(RuntimeError, match=expected):
        run_fake_experiment(
            tmp_path,
            curve_config(),
            tactile_type=tactile_type,
            enable_error=failure == "enable",
            command_delay_s=0.1 if failure == "latency" else 0.0,
        )


def test_input_closed_and_disable_failure_preserve_original_error(tmp_path: Path) -> None:
    """输入关闭与失能失败同时发生时保留原始错误与清理错误。"""
    with pytest.raises(RuntimeError, match="交互输入已关闭") as exc_info:
        run_fake_experiment(
            tmp_path,
            curve_config(),
            actions_type=ClosedInputActions,
            disable_error=True,
        )
    notes = getattr(exc_info.value, "__notes__", ())
    assert any("退出清理未完全成功" in note and "失能失败" in note for note in notes)
    assert any("运行记录：" in note for note in notes)
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["error"]["message"] == "交互输入已关闭"
    assert manifest["disable_confirmed"] is False
    assert any("失能失败" in error for error in manifest["cleanup_errors"])


def test_disable_failure_after_completed_control_is_a_run_failure(tmp_path: Path) -> None:
    """控制完成后的失能失败必须写 failed manifest 并向调用者返回失败。"""
    with pytest.raises(RuntimeError, match="退出清理失败.*失能失败"):
        run_fake_experiment(tmp_path, curve_config(), disable_error=True)
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["disable_confirmed"] is False
    assert "失能失败" in manifest["error"]["message"]


def test_terminal_stop_failure_does_not_skip_manifest(tmp_path: Path) -> None:
    """终端收尾异常作为清理故障上报，且不得阻断 manifest 落盘。"""

    class BrokenTerminal:
        def start(self) -> None:
            pass

        def stop(self) -> None:
            raise OSError("模拟终端退出失败")

        def publish(self, _snapshot) -> None:
            pass

        def show_event(self, _text: str, **_kwargs) -> None:
            pass

    with pytest.raises(RuntimeError, match="退出清理失败.*终端显示退出失败"):
        run_fake_experiment(tmp_path, curve_config(), terminal=BrokenTerminal())
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["disable_confirmed"] is True
    assert any("终端显示退出失败" in error for error in manifest["cleanup_errors"])


def test_contact_loss_reapproach_resumes_task_time(tmp_path: Path) -> None:
    """曲线模式失接触重接近：接触段递增、任务时间从暂停点继续。"""
    from .fakes import TransientLostTactile

    config = curve_config(
        lost_contact_action="reapproach",
        reapproach_max_attempts=2,
    )
    result, _session, actions, directory = run_fake_experiment(
        tmp_path, config, tactile_type=TransientLostTactile
    )
    assert result["status"] == "completed"
    assert actions.states.count("approach") >= 2, "应当出现重接近"
    events = _read_events(directory)
    confirmed = [event for event in events if event["event"] == "contact_confirmed"]
    assert len(confirmed) >= 2
    assert confirmed[1]["contact_segment"] == 2
    rows = _read_rows(directory)
    segments = {int(row["contact_segment"]) for row in rows if row["contact_segment"]}
    assert segments == {0, 1, 2}, "重接近后接触段编号应递增"
    task_times = [
        (row["phase"], float(row["task_time_s"] or 0.0)) for row in rows if row["task_time_s"]
    ]
    assert task_times == sorted(task_times, key=lambda item: item[1])


def test_repeated_snapshot_does_not_confirm_contact_before_staleness(tmp_path: Path) -> None:
    """重复快照不能推进接触确认窗口。"""
    from dataclasses import replace

    from dmgripper_experiments.observation import pair_observation
    from dmgripper_experiments.runtime import _KINEMATICS
    from dmgripper_experiments.tactile import TactileSnapshot
    from dmgripper_hardware import MotorFeedback, STATUS_ENABLED

    feedback = MotorFeedback(0.4, 0.0, 0.0, STATUS_ENABLED)
    snapshot = TactileSnapshot(
        received_at_s=1.0,
        packet_counter=1,
        timestamp_us=10_000,
        left_force_n=0.5,
        right_force_n=0.5,
        raw_left_fz_n=0.5,
        raw_right_fz_n=0.5,
        raw_left_fx_n=0.0,
        raw_left_fy_n=0.0,
        raw_right_fx_n=0.0,
        raw_right_fy_n=0.0,
    )
    first = pair_observation(
        snapshot=snapshot, feedback=feedback, previous=None, now_s=1.0, kinematics=_KINEMATICS
    )
    assert first.is_new_tactile is True
    repeated = pair_observation(
        snapshot=snapshot, feedback=feedback, previous=snapshot, now_s=1.05, kinematics=_KINEMATICS
    )
    assert repeated.is_new_tactile is False
    with pytest.raises(RuntimeError, match="未递增"):
        pair_observation(
            snapshot=replace(snapshot, timestamp_us=9_999, received_at_s=1.06),
            feedback=feedback,
            previous=snapshot,
            now_s=1.06,
            kinematics=_KINEMATICS,
        )


def test_stiffness_diagnostics_do_not_change_control_commands(tmp_path: Path) -> None:
    """只诊断模式下，刚度估计开关不改变同一观测序列上的控制命令。"""
    import dataclasses

    disabled = dataclasses.replace(
        curve_config(), estimation=dataclasses.replace(EstimationConfig(), enabled=False)
    )
    enabled = curve_config()
    result_a, _session_a, _actions_a, directory_a = run_fake_experiment(tmp_path, disabled)
    result_b, _session_b, _actions_b, directory_b = run_fake_experiment(tmp_path, enabled)
    assert result_a["status"] == result_b["status"] == "completed"
    rows_a = _read_rows(directory_a)
    rows_b = _read_rows(directory_b)
    assert len(rows_a) == len(rows_b)
    for row_a, row_b in zip(rows_a, rows_b, strict=True):
        assert row_a["q_des_rad"] == row_b["q_des_rad"]
        assert row_a["tau_ff_nm"] == row_b["tau_ff_nm"]
    tracking_b = [row for row in rows_b if row["phase"] in {"preload", "active", "holding"}]
    assert tracking_b
    diagnosed = [row for row in tracking_b if row["stiffness_reason"]]
    assert diagnosed, "启用估计后跟踪阶段必须产生刚度诊断"
    assert all(row["stiffness_n_per_m"] for row in diagnosed)
    # 未收到新观测的首个跟踪周期允许留空，其余必须有诊断原因。
    assert len(diagnosed) >= len(tracking_b) - 1
    assert all(row["stiffness_n_per_m"] == "" for row in rows_a)


def test_pid_records_control_force_and_raw_inputs(tmp_path: Path) -> None:
    """PID 路径记录原始力、控制使用力与外层滤波力三层信息。"""
    import dataclasses

    from dmgripper_experiments.config import ControllerConfig

    config = dataclasses.replace(
        curve_config(), controller=dataclasses.replace(ControllerConfig(), kind="pid")
    )
    _result, _session, _actions, directory = run_fake_experiment(tmp_path, config)
    rows = _read_rows(directory)
    tracking = [row for row in rows if row["phase"] in {"preload", "active", "holding"}]
    assert tracking
    assert all(row["control_force_n"] for row in tracking), "控制使用力必须被记录"
    assert all(row["raw_left_fz_n"] for row in tracking)
    assert all(row["left_fz_n"] for row in tracking)


def test_holding_phase_keeps_responding_for_adaptive(tmp_path: Path) -> None:
    """动态模式进入 holding 后策略时钟不冻结，仍按新观测更新。"""
    _result, _session, _actions, directory = run_fake_experiment(tmp_path, adaptive_config())
    rows = _read_rows(directory)
    holding = [row for row in rows if row["phase"] == "holding"]
    assert holding
    # FakeTactile 在 holding 仍施加切向 0（active 才有）；目标保持不超过上限。
    assert all(0.0 <= float(row["target_force_n"]) <= 1.5 + 1e-9 for row in holding)
    assert all(row["measured_tangential_force_n"] for row in holding)

"""通用运行时的端到端假设备验证：统一自适应目标、故障与清理。"""

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
    TransientLostTactile,
    adaptive_config,
    make_config,
    run_fake_experiment,
)


def _read_rows(directory: Path) -> list[dict[str, str]]:
    with (directory / "trace.csv").open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _read_events(directory: Path) -> list[dict[str, object]]:
    return [
        json.loads(line)
        for line in (directory / "events.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def test_full_lifecycle_completes(tmp_path: Path) -> None:
    """统一自适应配置能走完全部阶段并安全失能。"""
    result, session, actions, directory = run_fake_experiment(tmp_path, make_config())
    assert result["status"] == "completed"
    assert result["disable_confirmed"] is True
    assert session.disable_calls == 1
    assert session.closed
    assert actions.states == [
        "preparing",
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


def test_online_plots_are_registered_in_final_manifest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """在线绘图发生在 manifest 最终化前，产物仍必须登记到最终清单。"""
    from dmgripper_experiments import runtime

    def fake_plot(directory: Path) -> tuple[Path, Path]:
        paths = (directory / "plot.pdf", directory / "plot.png")
        for path in paths:
            path.write_bytes(b"fake plot")
        return paths

    monkeypatch.setattr(runtime, "plot_experiment_run", fake_plot)
    result, _session, _actions, directory = run_fake_experiment(tmp_path, make_config())
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert result["status"] == "completed"
    assert set(result["plots"]) == {str(directory / "plot.pdf"), str(directory / "plot.png")}
    assert manifest["files"]["plot.pdf"] == "plot.pdf"
    assert manifest["files"]["plot.png"] == "plot.png"


def test_failed_run_still_generates_and_registers_diagnostic_plots(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """控制失败后的已记录轨迹仍生成图表，且不掩盖原始故障。"""
    from dmgripper_experiments import runtime

    def fake_plot(directory: Path) -> tuple[Path, Path]:
        paths = (directory / "plot.pdf", directory / "plot.png")
        for path in paths:
            path.write_bytes(b"failed-run plot")
        return paths

    monkeypatch.setattr(runtime, "plot_experiment_run", fake_plot)
    with pytest.raises(RuntimeError, match="交互输入已关闭"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            actions_type=ClosedInputActions,
        )
    directory = next(tmp_path.rglob("manifest.json")).parent
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["primary_error"]["message"] == "交互输入已关闭"
    assert manifest["files"]["plot.pdf"] == "plot.pdf"
    assert manifest["files"]["plot.png"] == "plot.png"


def test_online_plot_failure_does_not_replace_control_result(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """后处理绘图失败只记入 manifest，不覆盖已完成的控制结果。"""
    from dmgripper_experiments import runtime

    def fail_plot(_directory: Path):
        raise OSError("模拟绘图失败")

    monkeypatch.setattr(runtime, "plot_experiment_run", fail_plot)
    result, _session, _actions, directory = run_fake_experiment(tmp_path, make_config())
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert result["status"] == "completed"
    assert result["plots"] == []
    assert manifest["status"] == "completed"
    assert manifest["post_processing_error"]["message"] == "模拟绘图失败"


def test_adaptive_target_increases_under_tangential_load(tmp_path: Path) -> None:
    """active 阶段因切向载荷增加目标力并记录触发。"""
    result, _session, _actions, directory = run_fake_experiment(tmp_path, adaptive_config())
    assert result["status"] == "completed"
    rows = _read_rows(directory)
    active_targets = [float(row["target_force_n"]) for row in rows if row["phase"] == "active"]
    assert active_targets, "active 阶段必须有 trace 记录"
    assert max(active_targets) > 0.5, "切向载荷应当驱动目标力增长"
    # 静态切向不产生重分配风险；承载调度增长由摩擦比目标直接体现。
    assert any(
        float(row["adaptive_load_target_n"] or 0.0) > 0.5
        for row in rows
        if row["phase"] == "active"
    )


def _nine_taxel_config():
    """构造九触点链路专用的短时限统一配置。"""
    from dataclasses import replace

    from dmgripper_experiments.config import UnifiedHardwareConfig, UnifiedObserverConfig

    config = adaptive_config()
    unified = UnifiedHardwareConfig(
        estimator="classic",
        observer=UnifiedObserverConfig(window_s=0.01),
        failure_timeout_s=0.01,
    )
    return replace(
        config,
        reference=replace(config.reference, duration_s=0.08, unified=unified),
        safety=replace(
            config.safety,
            max_target_force_n=1.5,
            force_ceiling_n=2.0,
            max_force_rate_n_s=0.5,
        ),
    )


class NineTaxelTactile(FakeTactile):
    """提供九点真机形状，设备采样间隔与接收间隔刻意不同。"""

    mode = "healthy"

    def latest(self):
        """按模式注入通信丢失，否则返回正常快照。"""
        if (self.mode == "communication" and self.phase.phase == "active") or (
            self.mode == "hold_communication" and self.phase.phase == "fault_holding"
        ):
            raise OSError("模拟统一模式触觉通信丢失")
        return super().latest()

    def _snapshot(self, *, force_n, tangential_n=0.0):
        from dataclasses import replace

        sink = self.sample_sink
        self.sample_sink = None
        try:
            snapshot = super()._snapshot(force_n=force_n, tangential_n=tangential_n)
        finally:
            self.sample_sink = sink
        # 粘着下界会把采用摩擦抬到量程上限；容量类场景的切向须在 μ=2.0 下
        # 仍越过目标上限，才能形成持续的 capacity_limited 证据。
        heavy = self.mode in {"capacity", "hold_communication"}
        shear = (0.3 if self.mode == "healthy" else 3.0 if heavy else 1.0) if force_n else 0.0
        if self.mode == "overforce" and self.phase.phase == "active":
            force_n = 3.0
        snapshot = replace(
            snapshot,
            timestamp_us=self.counter * 5000,
            raw_left_fy_n=shear,
            raw_left_fz_n=force_n,
            left_taxel_forces_n=((0.0, shear / 3, force_n / 3),) * 3 + ((0.0, 0.0, 0.0),) * 6,
            right_taxel_forces_n=((0.0, 0.0, force_n / 3),) * 3 + ((0.0, 0.0, 0.0),) * 6,
        )
        if self.mode == "taxel_saturation" and self.phase.phase == "active":
            snapshot = replace(snapshot, left_taxel_forces_n=((4.0, 0.0, force_n / 9),) * 9)
        if self.mode == "invalid_timestamp" and self.phase.phase == "active":
            snapshot = replace(snapshot, timestamp_us=math.nan)
        if sink is not None:
            sink(
                {
                    "timestamp_us": snapshot.timestamp_us,
                    "left_taxel_forces_n": snapshot.left_taxel_forces_n,
                    "right_taxel_forces_n": snapshot.right_taxel_forces_n,
                }
            )
        return snapshot


@pytest.mark.parametrize(
    "mode",
    [
        "healthy",
        "capacity",
        "overforce",
        "communication",
        "hold_communication",
        "taxel_saturation",
        "invalid_timestamp",
    ],
)
def test_unified_nine_taxel_lifecycle_and_fault_health_gate(tmp_path: Path, mode: str) -> None:
    """九点链路按设备时间增力，任务与触觉故障均持位等待人工释放。"""
    config = _nine_taxel_config()
    if mode == "healthy":
        result, session, actions, directory = run_fake_experiment(
            tmp_path, config, tactile_type=type("Healthy", (NineTaxelTactile,), {"mode": mode})
        )
        assert result["disable_confirmed"] is True
        assert session.disable_calls == 1
    else:
        with pytest.raises(RuntimeError, match="统一"):
            run_fake_experiment(
                tmp_path,
                config,
                tactile_type=type(mode.title(), (NineTaxelTactile,), {"mode": mode}),
            )
        directory = next(tmp_path.rglob("manifest.json")).parent
    manifest = json.loads((directory / "manifest.json").read_text())
    events = _read_events(directory)
    if mode == "healthy":
        assert result["status"] == "completed"
        assert "active" in actions.states and "holding" in actions.states
        rows = _read_rows(directory)
        active = [row for row in rows if row["phase"] == "active"]
        targets = [float(row["target_force_n"]) for row in active]
        assert max(targets) > 0.5
        assert all(0 <= b - a <= 0.5 * 0.005 + 1e-12 for a, b in zip(targets, targets[1:]))
        assert all(row["adaptive_left_valid_mask"] == "111000000" for row in active)
        assert all(row["adaptive_left_update_reason"] for row in active)
        assert any(
            float(row["adaptive_load_target_n"]) > 0.5
            for row in rows
            if row["phase"] == "preload" and row["adaptive_load_target_n"]
        )
        assert any(event["event"] == "adaptive_observation" for event in events)
        raw = [json.loads(line) for line in (directory / "tactile.jsonl").read_text().splitlines()]
        assert raw and all(len(row["left_taxel_forces_n"]) == 9 for row in raw)
    else:
        assert manifest["status"] == "failed"
        assert manifest["disable_confirmed"] is True
        assert manifest["fault_holding_entered"] is True
        if mode in {"capacity", "hold_communication"}:
            assert "capacity_limited" in manifest["primary_error"]["message"]
        assert manifest["fault_resolution"] == "released"


def test_preload_accepts_force_above_floor_without_imbalance_fault(tmp_path: Path) -> None:
    """双侧均接触且均值高于下限时，力差不阻止进入 active。"""
    from dataclasses import replace

    class HighPreloadTactile(FakeTactile):
        """仅在 preload 阶段返回明显不对称的双侧力。"""

        def latest(self):
            snapshot = super().latest()
            if self.phase.phase != "preload":
                return snapshot
            return replace(
                snapshot,
                left_force_n=2.0,
                right_force_n=0.2,
                raw_left_fz_n=2.0,
                raw_right_fz_n=0.2,
            )

    result, _session, actions, _directory = run_fake_experiment(
        tmp_path,
        adaptive_config(),
        tactile_type=HighPreloadTactile,
    )
    assert result["status"] == "completed"
    assert "active" in actions.states


def test_release_before_enable_cancels_without_motion(tmp_path: Path) -> None:
    """使能前 release 记为 cancelled：不使能、不发运动命令。

    DM 串口已在预检阶段打开并确认失能，但不产生任何使能或运动命令。
    """
    result, session, actions, directory = run_fake_experiment(
        tmp_path, make_config(), actions_type=CancelActions
    )
    assert result["status"] == "cancelled"
    assert result["disable_confirmed"] == "not_applicable"
    assert session.disable_calls == 0
    assert session.command_count == 0
    assert session.opened, "预检已打开 DM 串口；取消只禁止使能与运动"
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "cancelled"
    assert manifest["disable_confirmed"] == "not_applicable"


def test_start_command_is_recorded_before_motor_approach(tmp_path: Path) -> None:
    """收到 start 后先留存确认事件，再使能并进入 approach。"""
    device_state_at_acknowledgement: list[tuple[bool, int]] = []

    def observe_event(event: dict[str, object], session) -> None:
        if event.get("event") == "command_received" and event.get("action") == "start":
            device_state_at_acknowledgement.append((session.opened, session.feedback.status_code))

    result, session, _actions, directory = run_fake_experiment(
        tmp_path,
        make_config(),
        event_observer=observe_event,
    )
    assert result["status"] == "completed"
    assert session.command_count > 0
    assert device_state_at_acknowledgement == [(True, STATUS_DISABLED)]
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
    assert "正在使能电机并进入控制" in str(events[acknowledgement]["message"])


@pytest.mark.parametrize("initial_position_rad", [0.2, -0.04])
def test_start_outside_home_runs_limited_homing_before_approach(
    tmp_path: Path, initial_position_rad: float
) -> None:
    """预检阶段对不在 home 的残留位置先受限回零再进 ready，目标都在工作范围。"""
    result, session, actions, _directory = run_fake_experiment(
        tmp_path,
        make_config(),
        initial_position_rad=initial_position_rad,
    )
    assert result["status"] == "completed"
    assert actions.states.index("homing") < actions.states.index("ready")
    assert actions.states.index("homing") < actions.states.index("approach")
    assert session.commands
    assert all(0.0 <= command.position_rad <= math.pi / 2 for command in session.commands)


@pytest.mark.parametrize("initial_position_rad", [0.02, -0.02])
def test_start_inside_home_skips_homing(tmp_path: Path, initial_position_rad: float) -> None:
    """启动位置已在 home 容差内时不发送回零阶段命令。"""
    result, _session, actions, _directory = run_fake_experiment(
        tmp_path,
        make_config(),
        initial_position_rad=initial_position_rad,
    )
    assert result["status"] == "completed"
    assert "homing" not in actions.states


def test_feedback_outside_extended_safety_range_never_moves(tmp_path: Path) -> None:
    """初始反馈越出扩展安全范围时不得使能或自动运动。"""
    observed_sessions = []

    def observe(_event: dict[str, object], session) -> None:
        observed_sessions.append(session)

    with pytest.raises(ValueError, match="扩展安全范围"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            initial_position_rad=-0.051,
            event_observer=observe,
        )
    session = observed_sessions[-1]
    assert session.command_count == 0
    assert session.disable_calls == 0


def test_precheck_homing_timeout_forces_disable_before_ready(tmp_path: Path) -> None:
    """预检回零未到位时直接失能失败退出，不进入故障保持。"""
    with pytest.raises(RuntimeError, match="预检回零轨迹执行超时"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            initial_position_rad=0.2,
            freeze_phase="homing",
        )
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["fault_phase"] == "homing"
    assert manifest["fault_holding_entered"] is False
    assert manifest["fault_resolution"] == "forced_disable"
    assert manifest["disable_confirmed"] is True


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
        make_config(),
        actions_type=ReportingActions,
    )
    assert result["status"] == "completed"
    events = _read_events(directory)
    warnings = [event for event in events if event.get("code") == "unknown_command"]
    assert [warning["action"] for warning in warnings] == ["__input_closed__", "statuz"]
    assert all("start、status、release" in str(warning["message"]) for warning in warnings)
    assert any(event.get("event") == "status" and event.get("phase") == "ready" for event in events)


def test_on_finished_return_completes_without_interactive_actions(tmp_path: Path) -> None:
    """on_finished=return 的有限任务不依赖交互输入，任务计时后自动回位。"""

    class PassiveActions(PhaseActions):
        def __call__(self) -> None:
            return None

    result, session, actions, directory = run_fake_experiment(
        tmp_path,
        make_config(on_finished="return"),
        actions_type=PassiveActions,
    )
    assert result["status"] == "completed"
    assert session.disable_calls == 1
    assert actions.states[-3:] == ["holding", "returning", "completed"]
    events = _read_events(directory)
    assert any(event.get("event") == "task_finished" for event in events)


@pytest.mark.parametrize("release_phase", ["approach", "contact_transition", "preload"])
def test_release_before_active_uses_limited_return(tmp_path: Path, release_phase: str) -> None:
    """建立抓力的各子阶段都能直接请求受限回位。"""

    class EarlyReleaseActions(PhaseActions):
        def __init__(self) -> None:
            super().__init__()
            self.released = False

        def __call__(self) -> str | None:
            if self.phase == release_phase and not self.released:
                self.released = True
                return "release"
            return super().__call__()

    result, session, actions, directory = run_fake_experiment(
        tmp_path,
        make_config(),
        actions_type=EarlyReleaseActions,
    )
    assert result["status"] == "completed"
    assert session.disable_calls == 1
    assert actions.released is True
    assert actions.states[-2:] == ["returning", "completed"]
    assert "active" not in actions.states
    events = _read_events(directory)
    assert any(
        event.get("event") == "command_received"
        and event.get("phase") == release_phase
        and event.get("action") == "release"
        for event in events
    )


def test_zero_force_peak_warning_is_recorded_without_blocking_start(tmp_path: Path) -> None:
    """零力均值合格时，接触级峰值只记录告警且仍可进入完整生命周期。"""

    class PeakThenQuietTactile(FakeTactile):
        """零力验证首帧有峰值，后续帧保持空载。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s):
            self.clock.advance(0.01)
            force_n = 0.2 if self.counter == 0 else 0.0
            return self._snapshot(force_n=force_n)

    result, _session, _actions, directory = run_fake_experiment(
        tmp_path,
        make_config(),
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
            make_config(),
            tactile_type=tactile_type,
            enable_error=failure == "enable",
            command_delay_s=0.1 if failure == "latency" else 0.0,
        )


@pytest.mark.parametrize(
    ("failure", "expected"),
    [
        ("stale", "触觉快照过期"),
        ("lost", "持续失去接触"),
        ("preload", "初始抓力在等待上限内未达到稳定"),
        ("overforce", "原始法向力超过保护上限"),
    ],
)
def test_holdable_fault_waits_for_release_then_keeps_failed_result(
    tmp_path: Path, failure: str, expected: str
) -> None:
    """四类实验故障先保持并接受 status/release，回位失能后结果仍为 failed。"""
    from dataclasses import replace

    from .fakes import BadTactile

    class StatusThenReleaseActions(PhaseActions):
        def __init__(self) -> None:
            super().__init__()
            self.fault_actions = ["status", None, "release"]

        def __call__(self):
            if self.phase == "fault_holding" and self.fault_actions:
                return self.fault_actions.pop(0)
            return super().__call__()

    if failure == "lost":
        tactile_type = type(
            "LostTactile",
            (BadTactile,),
            {"mode": "lost", "target_phase": "active"},
        )
        config = make_config()
    elif failure == "preload":

        class PreloadTimeoutTactile(FakeTactile):
            def latest(self):
                snapshot = super().latest()
                if self.phase.phase == "preload":
                    return replace(
                        snapshot,
                        left_force_n=0.2,
                        right_force_n=0.2,
                        raw_left_fz_n=0.2,
                        raw_right_fz_n=0.2,
                    )
                return snapshot

        tactile_type = PreloadTimeoutTactile
        config = replace(
            make_config(),
            lifecycle=replace(make_config().lifecycle, preload_timeout_s=0.05),
        )
    else:
        tactile_type = type(
            f"{failure.title()}Tactile",
            (BadTactile,),
            {"mode": failure, "target_phase": "approach"},
        )
        config = make_config()

    observations: list[tuple[dict[str, object], object]] = []

    def observe(event: dict[str, object], session) -> None:
        observations.append((event, session))
        if event.get("phase") == "fault_holding" and event.get("event") == "state":
            assert session.disable_calls == 0
            assert session.hold_calls >= 1
            hold = session.commands[-1]
            assert hold.velocity_rad_s == pytest.approx(0.0)
            assert hold.feedforward_torque_nm == pytest.approx(0.0)

    with pytest.raises(RuntimeError, match=expected):
        run_fake_experiment(
            tmp_path,
            config,
            tactile_type=tactile_type,
            actions_type=StatusThenReleaseActions,
            event_observer=observe,
        )
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert expected in manifest["primary_error"]["message"]
    assert manifest["fault_holding_entered"] is True
    assert manifest["fault_resolution"] == "released"
    assert manifest["disable_confirmed"] is True
    events = _read_events(next(tmp_path.rglob("manifest.json")).parent)
    phases = [event.get("phase") for event in events]
    assert "fault_holding" in phases
    assert any(
        event.get("event") == "status" and event.get("phase") == "fault_holding" for event in events
    )
    assert any(
        event.get("event") == "command_received" and event.get("action") == "release"
        for event in events
    )
    assert "returning" in phases


def test_dm_command_failure_forces_immediate_disable_without_claiming_hold(
    tmp_path: Path,
) -> None:
    """DM 命令通道丢失属于不可保持故障，立即尽力失能且不记录 holding。"""
    sessions = []

    def observe(_event: dict[str, object], session) -> None:
        sessions.append(session)

    with pytest.raises(RuntimeError, match="DM 命令或反馈失败"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            command_error_at=1,
            event_observer=observe,
        )
    session = sessions[-1]
    assert session.hold_calls == 0
    assert session.disable_calls == 1
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["fault_holding_entered"] is False
    assert manifest["fault_resolution"] == "forced_disable"
    assert manifest["disable_confirmed"] is True


@pytest.mark.parametrize(
    "hardware_error",
    [
        RuntimeError("DM 电机故障：status_code=8"),
        RuntimeError("DM 运行中失能：status_code=0"),
        ValueError("机械关节反馈角超出扩展安全范围"),
    ],
)
def test_unsafe_dm_feedback_forces_immediate_disable(
    tmp_path: Path,
    hardware_error: BaseException,
) -> None:
    """电机故障、意外失能与反馈越界均不得进入故障保持。"""
    sessions = []

    def observe(_event: dict[str, object], session) -> None:
        sessions.append(session)

    with pytest.raises(RuntimeError, match=str(hardware_error)):
        run_fake_experiment(
            tmp_path,
            make_config(),
            command_error_at=1,
            command_error=hardware_error,
            event_observer=observe,
        )
    session = sessions[-1]
    assert session.hold_calls == 0
    assert session.disable_calls == 1
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["fault_holding_entered"] is False
    assert manifest["fault_resolution"] == "forced_disable"
    assert manifest["disable_confirmed"] is True


def test_precheck_recovers_leftover_enabled_state_before_zero_force(tmp_path: Path) -> None:
    """上次异常退出的残留使能态由预检显式失能，不影响后续流程。"""
    result, session, _actions, _directory = run_fake_experiment(
        tmp_path,
        make_config(),
        initial_enabled=True,
    )
    assert result["status"] == "completed"
    # 预检恢复失能一次，正常收尾清理失能一次。
    assert session.disable_calls == 2
    assert result["disable_confirmed"] is True


def test_enable_confirmation_failure_is_recorded_as_forced_disable(tmp_path: Path) -> None:
    """使能确认丢失后必须尽力失能，并明确记录 forced_disable 处置。"""
    with pytest.raises(RuntimeError, match="使能确认丢失"):
        run_fake_experiment(tmp_path, make_config(), enable_error=True)
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["fault_holding_entered"] is False
    assert manifest["fault_resolution"] == "forced_disable"
    assert manifest["disable_confirmed"] is True


def test_precheck_enable_failure_outside_home_fails_before_ready(tmp_path: Path) -> None:
    """需要预检回零时使能失败在 ready 前暴露，清理仍尽力失能。"""
    with pytest.raises(RuntimeError, match="使能确认丢失"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            initial_position_rad=0.2,
            enable_error=True,
        )
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["fault_phase"] == "homing"
    assert manifest["fault_resolution"] == "forced_disable"
    assert manifest["disable_confirmed"] is True


def test_fault_holding_command_failure_preserves_primary_and_marks_hold_lost(
    tmp_path: Path,
) -> None:
    """保持建立后再次写入失败时立即失能，但首个触觉故障仍是 primary。"""
    from .fakes import BadTactile

    stale_type = type(
        "StaleTactile",
        (BadTactile,),
        {"mode": "stale", "target_phase": "approach"},
    )
    with pytest.raises(RuntimeError, match="触觉快照过期"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            tactile_type=stale_type,
            hold_error_at=2,
        )
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert "触觉快照过期" in manifest["primary_error"]["message"]
    assert manifest["fault_holding_entered"] is True
    assert manifest["fault_resolution"] == "hold_lost"
    assert manifest["disable_confirmed"] is True
    assert any("故障保持丢失" in error for error in manifest["cleanup_errors"])


def test_fault_release_planning_failure_preserves_primary_and_holding_metadata(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """释放规划异常保留首故障与位置保持，下一次 release 可重试。"""
    from .fakes import BadTactile
    from dmgripper_experiments import runtime

    stale_type = type(
        "StaleTactile",
        (BadTactile,),
        {"mode": "stale", "target_phase": "approach"},
    )

    original = runtime._home_trajectory
    failures = []

    def fail_home_trajectory(*args, **kwargs):
        if not failures:
            failures.append(True)
            raise ValueError("模拟故障释放规划失败")
        return original(*args, **kwargs)

    monkeypatch.setattr(runtime, "_home_trajectory", fail_home_trajectory)
    with pytest.raises(RuntimeError, match="触觉快照过期"):
        run_fake_experiment(tmp_path, make_config(), tactile_type=stale_type)
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert "触觉快照过期" in manifest["primary_error"]["message"]
    assert manifest["fault_holding_entered"] is True
    assert manifest["fault_resolution"] == "released"
    assert any("模拟故障释放规划失败" in error for error in manifest["cleanup_errors"])


def test_keyboard_interrupt_is_emergency_exit_and_disables_immediately(tmp_path: Path) -> None:
    """Ctrl+C 不进入普通 release 流程，而是立即尽力失能并关闭设备。"""

    class InterruptActions(PhaseActions):
        def __call__(self):
            if self.phase == "approach":
                raise KeyboardInterrupt
            return super().__call__()

    sessions = []

    def observe(_event: dict[str, object], session) -> None:
        sessions.append(session)

    with pytest.raises(KeyboardInterrupt):
        run_fake_experiment(
            tmp_path,
            make_config(),
            actions_type=InterruptActions,
            event_observer=observe,
        )
    assert sessions[-1].disable_calls == 1
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["fault_holding_entered"] is False
    assert manifest["fault_resolution"] == "forced_disable"
    assert manifest["disable_confirmed"] is True


def test_keyboard_interrupt_during_fault_holding_preserves_first_fault(tmp_path: Path) -> None:
    """故障保持中的 Ctrl+C 立即失能，但不得覆盖先发生的实验故障。"""
    from .fakes import BadTactile

    stale_type = type(
        "StaleTactile",
        (BadTactile,),
        {"mode": "stale", "target_phase": "approach"},
    )

    class InterruptHoldingActions(PhaseActions):
        def __call__(self):
            if self.phase == "fault_holding":
                raise KeyboardInterrupt
            return super().__call__()

    sessions = []

    def observe(_event: dict[str, object], session) -> None:
        sessions.append(session)

    with pytest.raises(RuntimeError, match="触觉快照过期"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            tactile_type=stale_type,
            actions_type=InterruptHoldingActions,
            event_observer=observe,
        )
    assert sessions[-1].disable_calls == 1
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert "触觉快照过期" in manifest["primary_error"]["message"]
    assert manifest["fault_holding_entered"] is True
    assert manifest["fault_resolution"] == "forced_disable"
    assert manifest["disable_confirmed"] is True
    assert any("Ctrl+C" in error for error in manifest["cleanup_errors"])


def test_keyboard_interrupt_during_fault_holding_sleep_preserves_first_fault(
    tmp_path: Path,
) -> None:
    """故障保持等待被 Ctrl+C 打断时同样保留首故障并立即失能。"""
    from .fakes import BadTactile

    stale_type = type(
        "StaleTactile",
        (BadTactile,),
        {"mode": "stale", "target_phase": "approach"},
    )

    def interrupt_holding_sleep(_duration_s: float, actions: PhaseActions) -> None:
        if actions.phase == "fault_holding":
            raise KeyboardInterrupt

    sessions = []

    def observe(_event: dict[str, object], session) -> None:
        sessions.append(session)

    with pytest.raises(RuntimeError, match="触觉快照过期"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            tactile_type=stale_type,
            event_observer=observe,
            sleep_observer=interrupt_holding_sleep,
        )
    assert sessions[-1].disable_calls == 1
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert "触觉快照过期" in manifest["primary_error"]["message"]
    assert manifest["fault_holding_entered"] is True
    assert manifest["fault_resolution"] == "forced_disable"
    assert any("Ctrl+C" in error for error in manifest["cleanup_errors"])


def test_recorder_failure_enters_safe_hold_before_release(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """控制记录失败时，只要 DM 通道健康就先保持而不是立即失能。"""
    from dmgripper_experiments import runtime
    from dmgripper_experiments.recording import ExperimentRecorder

    class OneShotFailingRecorder(ExperimentRecorder):
        failed = False

        def write(self, row):
            if not self.failed:
                self.failed = True
                raise OSError("模拟控制记录失败")
            return super().write(row)

    monkeypatch.setattr(runtime, "ExperimentRecorder", OneShotFailingRecorder)
    holding_observed = []

    def observe(event: dict[str, object], session) -> None:
        if event.get("event") == "state" and event.get("phase") == "fault_holding":
            holding_observed.append((session.disable_calls, session.hold_calls))

    with pytest.raises(OSError, match="模拟控制记录失败"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            event_observer=observe,
        )
    assert holding_observed and holding_observed[0][0] == 0
    assert holding_observed[0][1] >= 1
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["fault_resolution"] == "released"
    assert "模拟控制记录失败" in manifest["primary_error"]["message"]


def test_first_emit_error_is_not_overwritten_by_later_output_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """同一事件的记录与外部输出同时失败时，先发生的记录错误仍是 primary。"""
    from dmgripper_experiments import runtime
    from dmgripper_experiments.recording import ExperimentRecorder

    class ApproachFailingRecorder(ExperimentRecorder):
        def append(self, event):
            if event.get("event") == "state" and event.get("phase") == "approach":
                raise OSError("首个记录错误")
            return super().append(event)

    monkeypatch.setattr(runtime, "ExperimentRecorder", ApproachFailingRecorder)

    def fail_same_event(event: dict[str, object], _session) -> None:
        if event.get("event") == "state" and event.get("phase") == "approach":
            raise RuntimeError("后续显示错误")

    with pytest.raises(OSError, match="首个记录错误") as exc_info:
        run_fake_experiment(
            tmp_path,
            make_config(),
            event_observer=fail_same_event,
        )
    assert any("后续显示错误" in note for note in getattr(exc_info.value, "__notes__", ()))
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["primary_error"]["message"] == "首个记录错误"
    assert manifest["fault_resolution"] == "released"


def test_session_factory_failure_still_finalizes_pre_enable_manifest(tmp_path: Path) -> None:
    """会话构造失败仍在清理边界内，不能留下无 manifest 的半成品目录。"""
    with pytest.raises(RuntimeError, match="模拟会话构造失败"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            session_factory_error=RuntimeError("模拟会话构造失败"),
        )
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["primary_error"]["message"] == "模拟会话构造失败"
    assert manifest["disable_confirmed"] == "not_applicable"


def test_input_closed_and_disable_failure_preserve_original_error(tmp_path: Path) -> None:
    """输入关闭与失能失败同时发生时保留原始错误与清理错误。"""
    with pytest.raises(RuntimeError, match="交互输入已关闭") as exc_info:
        run_fake_experiment(
            tmp_path,
            make_config(),
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
        run_fake_experiment(tmp_path, make_config(), disable_error=True)
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
        run_fake_experiment(tmp_path, make_config(), terminal=BrokenTerminal())
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["disable_confirmed"] is True
    assert any("终端显示退出失败" in error for error in manifest["cleanup_errors"])


def test_contact_loss_enters_fault_holding_and_waits_for_release(tmp_path: Path) -> None:
    """失接触不再重接近：立即进故障保持，人工释放后保留失败结果。"""
    phases: list[str] = []

    def observe(event: dict[str, object], _session) -> None:
        if event.get("event") == "state":
            phases.append(str(event.get("phase")))

    with pytest.raises(RuntimeError, match="跟踪阶段持续失去接触"):
        run_fake_experiment(
            tmp_path,
            make_config(),
            tactile_type=TransientLostTactile,
            event_observer=observe,
        )
    assert phases.count("approach") == 1, "失接触不得触发重新接近"
    assert "fault_holding" in phases
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["fault_holding_entered"] is True
    assert manifest["fault_resolution"] == "released"
    events = _read_events(next(tmp_path.rglob("manifest.json")).parent)
    confirmed = [event for event in events if event["event"] == "contact_confirmed"]
    assert len(confirmed) == 1
    rows = _read_rows(next(tmp_path.rglob("manifest.json")).parent)
    segments = {int(row["contact_segment"]) for row in rows if row["contact_segment"]}
    assert segments == {0, 1}, "接触段编号不应在失接触后继续递增"


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

    from dmgripper_experiments.config import EstimationConfig

    disabled = dataclasses.replace(
        make_config(), estimation=dataclasses.replace(EstimationConfig(), enabled=False)
    )
    enabled = make_config()
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


def test_tracking_records_raw_and_filtered_force_layers(tmp_path: Path) -> None:
    """跟踪阶段记录原始力与外层滤波力；控制使用力仅由控制器语义决定。"""
    _result, _session, _actions, directory = run_fake_experiment(tmp_path, make_config())
    rows = _read_rows(directory)
    tracking = [row for row in rows if row["phase"] in {"preload", "active", "holding"}]
    assert tracking
    assert all(row["raw_left_fz_n"] for row in tracking)
    assert all(row["left_fz_n"] for row in tracking)


def test_holding_phase_keeps_responding_for_adaptive(tmp_path: Path) -> None:
    """进入 holding 后策略时钟不冻结，仍按新观测更新且不越界。"""
    result, _session, _actions, directory = run_fake_experiment(tmp_path, adaptive_config())
    assert result["status"] == "completed"
    rows = _read_rows(directory)
    holding = [row for row in rows if row["phase"] == "holding"]
    assert holding
    # FakeTactile 在 holding 仍施加切向 0（active 才有）；目标保持不超过安全上限。
    assert all(0.0 <= float(row["target_force_n"]) <= 30.0 + 1e-9 for row in holding)
    assert all(row["measured_tangential_force_n"] for row in holding)


def test_unlimited_adaptive_runs_until_explicit_release(tmp_path: Path) -> None:
    """不限时目标超过原任务时长仍停留 active，仅人工释放结束。"""
    from dataclasses import replace

    class DelayedReleaseActions(PhaseActions):
        def __init__(self):
            super().__init__()
            self.active_cycles = 0

        def __call__(self):
            if self.phase == "active":
                self.active_cycles += 1
                if self.active_cycles >= 100:
                    return "release"
            return super().__call__()

    config = adaptive_config()
    config = replace(config, reference=replace(config.reference, duration_s=None))
    result, session, actions, directory = run_fake_experiment(
        tmp_path, config, actions_type=DelayedReleaseActions
    )
    assert result["status"] == "completed"
    assert actions.active_cycles == 100
    assert "holding" not in actions.states
    assert session.disable_calls == 1
    assert not any(event["event"] == "task_finished" for event in _read_events(directory))


def test_input_errors_during_fault_keep_motor_held_until_recovered_release(tmp_path: Path):
    """输入通道暂时丢失不失能退出，恢复后的 release 仍可完成回位。"""

    class RecoveringInput(ClosedInputActions):
        def __init__(self):
            super().__init__()
            self.failures = 0

        def __call__(self):
            if self.phase == "fault_holding" and self.failures < 3:
                self.failures += 1
                raise OSError("模拟输入读取异常")
            return super().__call__()

    def observe(event, session):
        if event.get("code") == "fault_holding_input_lost":
            assert session.disable_calls == 0
            assert session.hold_calls >= 1

    with pytest.raises(RuntimeError, match="交互输入已关闭"):
        run_fake_experiment(
            tmp_path, make_config(), actions_type=RecoveringInput, event_observer=observe
        )
    manifest_path = next(tmp_path.rglob("manifest.json"))
    manifest = json.loads(manifest_path.read_text())
    assert manifest["fault_holding_entered"] is True
    assert manifest["fault_resolution"] == "released"
    warnings = [
        event
        for event in _read_events(manifest_path.parent)
        if event.get("code") == "fault_holding_input_lost"
    ]
    assert len(warnings) == 1


def test_closing_torque_mode_keeps_transition_feedforward_and_release(tmp_path):
    """接触过渡保留前馈，单向跟踪不影响人工释放回位。"""
    result, session, _, directory = run_fake_experiment(tmp_path, adaptive_config())
    rows = _read_rows(directory)
    transition = [row for row in rows if row["phase"] == "contact_transition"]
    returning = [row for row in rows if row["phase"] == "returning"]
    assert transition and returning
    assert all(float(row["tau_ff_nm"]) > 0 for row in transition)
    assert result["status"] == "completed"
    assert session.disable_calls == 1


def test_zero_tracking_velocity_applies_after_contact_only(tmp_path):
    """接近轨迹保留速度，预载与 active 的 MIT 目标速度固定为零。"""

    class LateContactTactile(FakeTactile):
        """approach 前若干周期无接触，保留带非零速度的接近样本。"""

        def __init__(self, *args, **kwargs) -> None:
            super().__init__(*args, **kwargs)
            self._approach_cycles = 0

        def latest(self):
            from dataclasses import replace

            snapshot = super().latest()
            if self.phase.phase == "approach" and self._approach_cycles < 20:
                self._approach_cycles += 1
                return replace(
                    snapshot,
                    left_force_n=0.0,
                    right_force_n=0.0,
                    raw_left_fz_n=0.0,
                    raw_right_fz_n=0.0,
                )
            return snapshot

    result, _, _, directory = run_fake_experiment(
        tmp_path, adaptive_config(), tactile_type=LateContactTactile
    )
    rows = _read_rows(directory)
    approach = [row for row in rows if row["phase"] == "approach"]
    tracking = [row for row in rows if row["phase"] in {"preload", "active"}]

    assert any(abs(float(row["dq_des_rad_s"])) > 0.0 for row in approach)
    assert all(float(row["dq_des_rad_s"]) == 0.0 for row in tracking)
    assert result["status"] == "completed"


def test_run_echoes_effective_limits_and_records_config_delta(tmp_path: Path) -> None:
    """启动即回显最终限幅；config.json 同时保存全量与净差异留档。"""
    result, _session, _actions, directory = run_fake_experiment(tmp_path, adaptive_config())
    assert result["status"] == "completed"
    events = _read_events(directory)
    limits = next(event for event in events if event["event"] == "safety_limits")
    assert limits["max_target_force_n"] == pytest.approx(30.0)
    assert limits["force_ceiling_n"] == pytest.approx(40.0)
    assert limits["unified_min_force_n"] == pytest.approx(0.5)
    assert limits["unified_max_force_n"] == pytest.approx(30.0)
    record = json.loads((directory / "config.json").read_text(encoding="utf-8"))
    assert record["delta"]["reference"]["initial_force_n"] == pytest.approx(0.5)
    assert record["delta"]["reference"]["duration_s"] == pytest.approx(0.03)
    assert record["effective"]["safety"]["max_target_force_n"] == pytest.approx(30.0)


def test_preload_ramp_waits_until_final_target_before_activation(tmp_path):
    """即使实测力提前达标，也必须等升力完成后重新累计稳定时间。"""

    class GentleContact(FakeTactile):
        def latest(self):
            if self.phase.phase == "contact_transition":
                return self._snapshot(force_n=0.3)
            return super().latest()

    config = adaptive_config(preload_force_rate_n_s=1.0)
    result, _, _, directory = run_fake_experiment(tmp_path, config, tactile_type=GentleContact)
    rows = _read_rows(directory)
    preload = [r for r in rows if r["phase"] == "preload"]
    targets = [float(r["target_force_n"]) for r in preload]
    assert targets[0] == pytest.approx(0.3)
    assert targets[-1] == pytest.approx(0.5)
    assert targets == sorted(targets)
    assert max(float(r["target_force_rate_n_s"]) for r in preload) <= 1.0 + 1e-9
    active = next(r for r in rows if r["phase"] == "active")
    assert float(active["time_s"]) - float(preload[0]["time_s"]) >= 1.875 * 0.2 + 0.02
    assert result["status"] == "completed"


def test_preload_min_force_ratio_allows_coarse_preload_completion(tmp_path):
    """按比例的预载门槛允许达到最低抓力后尽早进入 active。"""

    class CoarsePreload(FakeTactile):
        def latest(self):
            from dataclasses import replace

            snapshot = super().latest()
            if self.phase.phase == "preload":
                return replace(
                    snapshot,
                    left_force_n=0.3,
                    right_force_n=0.3,
                    raw_left_fz_n=0.3,
                    raw_right_fz_n=0.3,
                )
            return snapshot

    config = adaptive_config(preload_min_force_ratio=0.6)
    result, _, _, directory = run_fake_experiment(tmp_path, config, tactile_type=CoarsePreload)
    rows = _read_rows(directory)

    assert any(row["phase"] == "preload" for row in rows)
    assert any(row["phase"] == "active" for row in rows)
    assert result["status"] == "completed"


def test_recording_duration_limit_finishes_like_release(tmp_path: Path) -> None:
    """时长上限触发后按 release 语义受限回位，manifest 写明 stop_reason。"""
    from dataclasses import replace

    from dmgripper_experiments.config import RecordingConfig

    config = replace(
        make_config(on_finished="return"),
        reference=replace(make_config().reference, duration_s=None),
        recording=RecordingConfig(max_duration_s=0.5),
    )
    result, session, _actions, directory = run_fake_experiment(tmp_path, config)
    assert result["status"] == "completed"
    assert result["stop_reason"] == "max_duration_s=0.5"
    assert session.disable_calls == 1
    events = _read_events(directory)
    assert any(event.get("event") == "recording_limit" for event in events)
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "completed"
    assert manifest["stop_reason"] == "max_duration_s=0.5"


def test_recording_tactile_size_limit_finishes_like_release(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """触觉体积上限触发后同样优雅收尾并写明 stop_reason。

    写线程置位上限是真实时序，FakeClock 驱动的假实验可能先结束；
    这里注入首个样本后即置位的记录器，确定性覆盖控制循环的轮询、
    受限回位与留档链路。真实置位语义由记录器级测试覆盖。
    """
    from dmgripper_experiments import runtime
    from dmgripper_experiments.recording import ExperimentRecorder

    class SizeLimitRecorder(ExperimentRecorder):
        limit_seen = False

        def sample(self, record) -> None:
            super().sample(record)
            self.limit_seen = True

        @property
        def limit_stop_reason(self) -> str | None:
            return "max_tactile_mib=0.000488281" if self.limit_seen else None

    monkeypatch.setattr(runtime, "ExperimentRecorder", SizeLimitRecorder)
    result, _session, _actions, directory = run_fake_experiment(tmp_path, make_config())
    assert result["status"] == "completed"
    assert result["stop_reason"] == "max_tactile_mib=0.000488281"
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["stop_reason"] == result["stop_reason"]
    assert any(event.get("event") == "recording_limit" for event in _read_events(directory))

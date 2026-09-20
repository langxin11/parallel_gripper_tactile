"""独立记录模式的稳定接触门禁与逐触点证据测试。"""

from dataclasses import replace
import io
import json

import pytest

from papillarray_hardware import TactileSnapshot
from papillarray_hardware.native_slip import NativeSlipStatus
from papillarray_hardware.record_cli import (
    JsonlRunRecorder,
    _manual_stop_requested,
    build_parser,
)
from papillarray_hardware.standalone import StandaloneSlipConfig, StandaloneSlipSession


class _Worker:
    """记录独立策略提交的启停请求。"""

    def __init__(self) -> None:
        self.native_slip_status = NativeSlipStatus()
        self.starts: list[tuple[int, float | None, float]] = []
        self.stops: list[str] = []

    def request_native_slip(
        self,
        session_id: int,
        *,
        max_duration_s: float | None,
        confirmation_timeout_s: float,
    ) -> None:
        self.starts.append((session_id, max_duration_s, confirmation_timeout_s))
        self.native_slip_status = NativeSlipStatus(session_id, "queued", "waiting_owner")

    def stop_native_slip(self, reason: str) -> None:
        self.stops.append(reason)


def _config(**changes: object) -> StandaloneSlipConfig:
    """构造使用精确时间步的短窗口配置。"""
    return replace(
        StandaloneSlipConfig(
            max_duration_s=None,
            native_slip_enabled=True,
            stable_duration_s=0.2,
            max_force_rate_n_s=1.0,
            sample_timeout_s=0.2,
            confirmation_timeout_s=0.1,
        ),
        **changes,
    )


def _sample(t: float, **changes: object) -> TactileSnapshot:
    """构造双侧各两个接触触点的快照。"""
    return replace(
        TactileSnapshot(
            received_at_s=t,
            packet_counter=round(t * 10),
            timestamp_us=round(t * 1e6),
            left_force_n=1.0,
            right_force_n=1.0,
            raw_left_fz_n=1.0,
            raw_right_fz_n=1.0,
            left_taxel_forces_n=((0.0, 0.0, 0.5), (0.0, 0.0, 0.5)),
            right_taxel_forces_n=((0.0, 0.0, 0.5), (0.0, 0.0, 0.5)),
        ),
        **changes,
    )


def test_starts_once_after_bilateral_contact_is_stable() -> None:
    """无目标力输入时，仅连续低变化率双侧接触能够启动一次。"""
    session, worker = StandaloneSlipSession(_config()), _Worker()
    for t in (0.0, 0.1):
        session.update(_sample(t), now_s=t, worker=worker)
    assert worker.starts == []
    session.update(_sample(0.2), now_s=0.2, worker=worker)
    assert worker.starts == [(1, None, 0.1)]
    worker.native_slip_status = NativeSlipStatus(1, "stopped", "duration_limit")
    for t in (0.4, 0.5, 0.6, 0.7):
        session.update(_sample(t), now_s=t, worker=worker)
    assert len(worker.starts) == 1


def test_own_estimation_starts_without_requesting_native_service() -> None:
    """自主模式通过同一稳定接触门禁，但不得向原厂服务发送启停请求。"""
    session, worker = StandaloneSlipSession(_config(native_slip_enabled=False)), _Worker()
    events = []
    for t in (0.0, 0.1, 0.2):
        events.extend(session.update(_sample(t), now_s=t, worker=worker))
    assert worker.starts == []
    assert worker.stops == []
    assert [event for event in events if event["event"] == "native_slip_state"] == []
    own_states = [event for event in events if event["event"] == "own_friction_state"]
    assert [(event["phase"], event["reason"]) for event in own_states] == [
        ("active", "stable_contact")
    ]


def test_stale_sample_stops_own_estimation_without_native_command() -> None:
    """自主模式失鲜时结束当前估计，但不得误发原厂停止命令。"""
    session, worker = StandaloneSlipSession(_config(native_slip_enabled=False)), _Worker()
    for t in (0.0, 0.1, 0.2):
        session.update(_sample(t), now_s=t, worker=worker)
    events = session.update(_sample(0.3), now_s=0.6, worker=worker)
    assert worker.starts == []
    assert worker.stops == []
    assert [
        (event["phase"], event["reason"])
        for event in events
        if event["event"] == "own_friction_state"
    ] == [("stopped", "stale_sample")]


def test_own_estimation_requires_sustained_contact_loss() -> None:
    """单包逐点跌破门槛不结束自主估计，持续失接触才停止。"""
    session, worker = StandaloneSlipSession(_config(native_slip_enabled=False)), _Worker()
    for t in (0.0, 0.1, 0.2):
        session.update(_sample(t), now_s=t, worker=worker)
    lost = dict(right_taxel_forces_n=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    first = session.update(_sample(0.21, **lost), now_s=0.21, worker=worker)
    recovered = session.update(_sample(0.22), now_s=0.22, worker=worker)
    assert [event for event in first + recovered if event["event"] == "own_friction_state"] == []
    session.update(_sample(0.23, **lost), now_s=0.23, worker=worker)
    stopped = session.update(_sample(0.27, **lost), now_s=0.27, worker=worker)
    assert [
        (event["phase"], event["reason"])
        for event in stopped
        if event["event"] == "own_friction_state"
    ] == [("stopped", "contact_lost")]


def test_force_ramp_and_contact_change_reset_stability() -> None:
    """夹爪仍在加力或触点集合变化时不能把接触误判为稳定。"""
    session, worker = StandaloneSlipSession(_config()), _Worker()
    session.update(_sample(0.0, left_force_n=0.5), now_s=0.0, worker=worker)
    session.update(_sample(0.1, left_force_n=1.0), now_s=0.1, worker=worker)
    session.update(_sample(0.2), now_s=0.2, worker=worker)
    session.update(
        _sample(0.3, left_taxel_forces_n=((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))),
        now_s=0.3,
        worker=worker,
    )
    session.update(_sample(0.4), now_s=0.4, worker=worker)
    assert worker.starts == []


def test_new_bilateral_slip_estimates_are_recorded_without_automatic_stop() -> None:
    """默认只记录双侧新 SLIPPED 正摩擦估计，把正常停止时机交给操作者。"""
    session, worker = StandaloneSlipSession(_config()), _Worker()
    for t in (0.0, 0.1, 0.2, 0.3):
        session.update(_sample(t), now_s=t, worker=worker)
    worker.native_slip_status = NativeSlipStatus(1, "active", "confirmed_active")
    events = session.update(
        _sample(
            0.4,
            native_session_id=1,
            native_session_phase="active",
            native_slip_active=(True, True),
            native_pillar_states=((3, 1), (3, 1)),
            native_pillar_friction=((0.4, None), (0.5, None)),
        ),
        now_s=0.4,
        worker=worker,
    )
    estimates = [event for event in events if event["event"] == "native_slip_estimate"]
    assert [(event["side"], event["native_mu"]) for event in estimates] == [
        ("left", 0.4),
        ("right", 0.5),
    ]
    assert worker.stops == []


def test_new_bilateral_slip_estimates_can_stop_automatically() -> None:
    """显式启用自动停止时，双侧取得所需估计后结束服务。"""
    session, worker = StandaloneSlipSession(_config(stop_when_estimates_ready=True)), _Worker()
    for t in (0.0, 0.1, 0.2, 0.3):
        session.update(_sample(t), now_s=t, worker=worker)
    worker.native_slip_status = NativeSlipStatus(1, "active", "confirmed_active")
    session.update(
        _sample(
            0.4,
            native_session_id=1,
            native_session_phase="active",
            native_slip_active=(True, True),
            native_pillar_states=((3, 1), (3, 1)),
            native_pillar_friction=((0.4, None), (0.5, None)),
        ),
        now_s=0.4,
        worker=worker,
    )
    assert worker.stops[-1] == "estimates_ready"


def test_running_session_keeps_reference_when_edge_contact_changes() -> None:
    """扰动期边缘触点变化不终止会话，辨识仍使用启动时冻结的参考集合。"""
    session, worker = StandaloneSlipSession(_config()), _Worker()
    for t in (0.0, 0.1, 0.2, 0.3):
        session.update(_sample(t), now_s=t, worker=worker)
    worker.native_slip_status = NativeSlipStatus(1, "active", "confirmed_active")
    session.update(
        _sample(
            0.4,
            native_session_id=1,
            native_session_phase="active",
            native_slip_active=(True, True),
            left_taxel_forces_n=((0.0, 0.0, 0.5), (0.0, 0.0, 0.05)),
            native_pillar_states=((1, 1), (1, 1)),
            native_pillar_friction=((None, None), (None, None)),
        ),
        now_s=0.4,
        worker=worker,
    )
    assert worker.stops == []


def test_running_session_stops_when_one_side_loses_contact() -> None:
    """边缘变化允许，但任一侧不再具有有效接触时必须结束原厂服务。"""
    session, worker = StandaloneSlipSession(_config()), _Worker()
    for t in (0.0, 0.1, 0.2, 0.3):
        session.update(_sample(t), now_s=t, worker=worker)
    worker.native_slip_status = NativeSlipStatus(1, "active", "confirmed_active")
    session.update(
        _sample(0.4, right_taxel_forces_n=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))),
        now_s=0.4,
        worker=worker,
    )
    assert worker.stops[-1] == "contact_lost"


@pytest.mark.parametrize(
    "changes",
    [
        {"max_duration_s": float("inf")},
        {"stable_duration_s": 0.0},
        {"contact_off_n": 0.15},
        {"contact_loss_confirm_s": 0.0},
        {"max_duration_s": 1.0, "confirmation_timeout_s": 1.0},
        {"min_estimates_per_side": True},
        {"stop_when_estimates_ready": 1},
        {"own_friction": None},
        {"native_slip_enabled": False, "own_friction_enabled": False},
    ],
)
def test_invalid_configuration_is_rejected(changes: dict[str, object]) -> None:
    """拒绝非有限时长、矛盾滞回和无效逐侧数量。"""
    with pytest.raises(ValueError):
        _config(**changes)


def test_recorder_writes_independent_run_artifacts(tmp_path) -> None:
    """独立记录目录保存快照、事件、配置和最终状态且拒绝覆盖。"""
    directory = tmp_path / "run"
    recorder = JsonlRunRecorder(directory, {"bias": True})
    recorder.sample({"packet_counter": 1})
    recorder.event({"event": "native_slip_state", "phase": "active"})
    recorder.close(status="completed")
    assert json.loads((directory / "config.json").read_text())["bias"] is True
    assert json.loads((directory / "tactile.jsonl").read_text())["packet_counter"] == 1
    assert json.loads((directory / "events.jsonl").read_text())["phase"] == "active"
    assert json.loads((directory / "manifest.json").read_text())["status"] == "completed"
    with pytest.raises(FileExistsError):
        JsonlRunRecorder(directory, {})


def test_cli_requires_explicit_bias_and_estimation_modes() -> None:
    """清零与两条估计路径都保持显式授权，原厂模式不隐式设置时长。"""
    parser = build_parser()
    plain = parser.parse_args([])
    assert plain.bias is False
    assert plain.estimate_friction is False
    assert plain.native_slip is False
    assert plain.native_slip_duration is None
    assert plain.stop_on_estimates_ready is False
    assert StandaloneSlipConfig().native_slip_enabled is False
    enabled = parser.parse_args(["--bias", "--native-slip"])
    assert enabled.bias is True
    assert enabled.native_slip is True
    assert enabled.native_slip_duration is None
    own = parser.parse_args(["--bias", "--estimate-friction"])
    assert own.bias is True
    assert own.estimate_friction is True
    assert own.native_slip is False
    assert own.native_slip_duration is None


def test_manual_stop_input_only_consumes_interactive_enter(monkeypatch) -> None:
    """只从真实交互终端非阻塞消费 Enter，避免管道输入误停服务。"""

    class _InteractiveInput(io.StringIO):
        def isatty(self) -> bool:
            return True

    interactive = _InteractiveInput("\n")
    monkeypatch.setattr(
        "papillarray_hardware.record_cli.select.select",
        lambda readers, _writers, _errors, _timeout: (readers, [], []),
    )
    assert _manual_stop_requested(interactive) is True
    assert _manual_stop_requested(io.StringIO("\n")) is False

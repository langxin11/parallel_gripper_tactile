"""原厂旁路辨识的稳定接触门禁、有限会话与逐触点证据测试。"""

from dataclasses import replace

import pytest

from dmgripper_experiments.native_slip import NativeSlipConfig, NativeSlipSession
from papillarray_hardware import TactileSnapshot
from papillarray_hardware.native_slip import NativeSlipStatus


class _Worker:
    """记录请求并由测试显式确认设备状态，不连接硬件。"""

    def __init__(self) -> None:
        """初始化待机状态与调用记录。"""
        self.native_slip_status = NativeSlipStatus()
        self.starts: list[tuple[int, float, float]] = []
        self.stops: list[str] = []

    def request_native_slip(
        self, session_id: int, *, max_duration_s: float, confirmation_timeout_s: float
    ) -> None:
        """保存传给采集层的会话与硬截止参数。"""
        self.starts.append((session_id, max_duration_s, confirmation_timeout_s))
        self.native_slip_status = NativeSlipStatus(session_id, "queued", "waiting_owner")

    def stop_native_slip(self, reason: str) -> None:
        """保留停止原因，但不冒充设备已确认停用。"""
        self.stops.append(reason)
        if self.native_slip_status.phase in {"queued", "starting", "active"}:
            self.native_slip_status = replace(
                self.native_slip_status, phase="stopping", reason=reason
            )


def _config(**changes: object) -> NativeSlipConfig:
    """用可精确表示的时间步构造短周期离线测试配置。"""
    return replace(
        NativeSlipConfig(
            max_duration_s=2.0,
            stable_duration_s=0.25,
            sample_timeout_s=0.3,
            confirmation_timeout_s=0.25,
            cooldown_s=0.5,
            estimate_expiry_s=0.5,
        ),
        **changes,
    )


def _sample(t: float, **changes: object) -> TactileSnapshot:
    """构造双侧各两个已接触触点以及同会话原厂扩展。"""
    return replace(
        TactileSnapshot(
            received_at_s=t,
            packet_counter=round(t * 8),
            timestamp_us=round(t * 1e6),
            left_force_n=1.0,
            right_force_n=1.0,
            raw_left_fz_n=1.0,
            raw_right_fz_n=1.0,
            left_taxel_forces_n=((0.0, 0.0, 0.5), (0.0, 0.0, 0.5)),
            right_taxel_forces_n=((0.0, 0.0, 0.5), (0.0, 0.0, 0.5)),
            native_session_id=1,
            native_session_phase="active",
            native_slip_active=(True, True),
            native_pillar_states=((1, 1), (1, 1)),
            native_pillar_friction=((0.3, 0.4), (0.5, 0.6)),
        ),
        **changes,
    )


def _update(
    session: NativeSlipSession,
    worker: _Worker,
    t: float,
    *,
    target: float = 1.0,
    permitted: bool = True,
    now: float | None = None,
    **changes: object,
) -> list[dict[str, object]]:
    """以可控设备时间和主机时钟处理一个包。"""
    return session.update(
        _sample(t, **changes),
        target_force_n=target,
        permitted=permitted,
        now_s=t if now is None else now,
        worker=worker,
    )


def _running(config: NativeSlipConfig | None = None) -> tuple[NativeSlipSession, _Worker]:
    """满足连续稳定门槛，再模拟双侧设备确认启动。"""
    session, worker = NativeSlipSession(config or _config()), _Worker()
    for t in (0.0, 0.125, 0.25, 0.375):
        _update(session, worker, t)
    assert len(worker.starts) == 1
    worker.native_slip_status = NativeSlipStatus(1, "active", "confirmed_active")
    return session, worker


def _estimates(events: list[dict[str, object]]) -> list[dict[str, object]]:
    """只取逐触点估计事件，排除生命周期事件。"""
    return [event for event in events if event["event"] == "native_slip_estimate"]


def test_start_requires_continuous_bilateral_stability() -> None:
    """稳定窗口未满不启动，左右任一侧偏离目标都阻断启动。"""
    session, worker = NativeSlipSession(_config()), _Worker()
    for t in (0.0, 0.125, 0.25):
        _update(session, worker, t, right_force_n=0.5)
    assert worker.starts == []
    for t in (0.375, 0.5, 0.625):
        _update(session, worker, t)
    assert worker.starts == []
    _update(session, worker, 0.75)
    assert worker.starts == [(1, 2.0, 0.25)]


def test_old_packets_cannot_accumulate_stable_time() -> None:
    """重复包的主机等待时间不能充当连续的新触觉证据。"""
    session, worker = NativeSlipSession(_config()), _Worker()
    _update(session, worker, 0.0)
    _update(session, worker, 0.125)
    for now in (0.25, 0.375):
        _update(session, worker, 0.125, now=now)
    assert worker.starts == []
    _update(session, worker, 0.25)
    assert worker.starts == []
    _update(session, worker, 0.375)
    assert len(worker.starts) == 1


def test_target_change_during_repeated_packet_resets_stability() -> None:
    """没有新触觉包时发生的目标突变也会破坏连续稳定条件。"""
    session, worker = NativeSlipSession(_config()), _Worker()
    _update(session, worker, 0.0)
    _update(session, worker, 0.125)
    _update(session, worker, 0.125, now=0.2, target=2.0)
    _update(session, worker, 0.25)
    _update(session, worker, 0.375)
    assert worker.starts == []


@pytest.mark.parametrize("kind", ["target", "stale", "counter_gap", "contact", "phase"])
def test_start_window_resets_on_interruption(kind: str) -> None:
    """目标突变、失鲜、丢包、接触变化及退出允许阶段均清空稳定窗口。"""
    session, worker = NativeSlipSession(_config()), _Worker()
    _update(session, worker, 0.0)
    _update(session, worker, 0.125)
    options = {
        "target": {"target": 1.1},
        "stale": {"now": 0.625},
        "counter_gap": {"counter_gap": 1},
        "contact": {"left_taxel_forces_n": ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))},
        "phase": {"permitted": False},
    }[kind]
    _update(session, worker, 0.25, **options)
    _update(session, worker, 0.375)
    assert worker.starts == []


def test_only_new_slipped_transitions_with_positive_finite_mu_are_evidence() -> None:
    """负值和非有限估计不成功，同一持续滑移状态也不重复产生估计。"""
    session, worker = _running(_config(min_estimates_per_side=2))
    events = _update(
        session,
        worker,
        0.5,
        native_pillar_states=((3, 3), (3, 3)),
        native_pillar_friction=((0.3, -1.0), (float("nan"), 0.0)),
    )
    assert [(event["side"], event["pillar_id"]) for event in _estimates(events)] == [("left", 0)]
    assert "estimates_ready" not in worker.stops
    assert _estimates(_update(session, worker, 0.625, native_pillar_states=((3, 3), (3, 3)))) == []
    assert session.trace_fields(worker)["native_left_estimate_count"] == 1
    assert session.trace_fields(worker)["native_right_estimate_count"] == 0
    _update(session, worker, 0.75)
    events = _update(session, worker, 0.875, native_pillar_states=((3, 3), (3, 3)))
    assert len(_estimates(events)) == 4
    assert worker.stops[-1] == "estimates_ready"


def test_missing_extension_does_not_fabricate_a_new_slipped_transition() -> None:
    """扩展缺失没有提供离开滑移状态的证据，恢复后的持续状态不能重新计数。"""
    session, worker = _running(_config(min_estimates_per_side=2))
    first = _update(session, worker, 0.5, native_pillar_states=((3, 1), (1, 1)))
    assert len(_estimates(first)) == 1
    _update(session, worker, 0.625, native_pillar_states=(), native_pillar_friction=())
    resumed = _update(session, worker, 0.75, native_pillar_states=((3, 1), (1, 1)))
    assert _estimates(resumed) == []


@pytest.mark.parametrize(
    "changes",
    [
        {"native_pillar_states": (), "native_pillar_friction": ()},
        {"native_pillar_states": ((-1, -1), (-1, -1))},
        {"native_slip_active": ()},
        {"native_session_id": 0},
        {"native_session_phase": "starting"},
    ],
)
def test_missing_invalid_or_other_session_extensions_never_succeed(
    changes: dict[str, object],
) -> None:
    """缺失扩展、无效状态和非当前活动会话的数据不能产生辨识结果。"""
    session, worker = _running()
    fields = {"native_pillar_states": ((3, 3), (3, 3)), **changes}
    assert _estimates(_update(session, worker, 0.5, **fields)) == []
    assert "estimates_ready" not in worker.stops


def test_each_side_must_reach_required_estimate_count_before_stop() -> None:
    """左侧数量充足不能替代右侧证据，逐侧满足数量后才提前结束。"""
    session, worker = _running(_config(min_estimates_per_side=2))
    _update(session, worker, 0.5, native_pillar_states=((3, 3), (3, 1)))
    assert "estimates_ready" not in worker.stops
    _update(session, worker, 0.625, native_pillar_states=((3, 3), (3, 3)))
    assert worker.stops[-1] == "estimates_ready"
    assert worker.native_slip_status.phase == "stopping"


def test_stopped_session_preserves_results_but_does_not_consume_new_evidence() -> None:
    """停止后继续保留已有结果，新的滑移状态不能刷新摩擦或有效期。"""
    session, worker = _running()
    _update(session, worker, 0.5, native_pillar_states=((3, 1), (3, 1)))
    worker.native_slip_status = NativeSlipStatus(1, "stopped", "estimates_ready")
    events = _update(session, worker, 0.625, native_pillar_states=((3, 3), (3, 3)))
    assert _estimates(events) == []
    assert session.trace_fields(worker)["native_left_estimate_count"] == 1
    for t in (0.75, 0.875, 1.0):
        _update(session, worker, t)
    assert session.trace_fields(worker)["native_left_estimate_count"] == 1
    _update(session, worker, 1.125)
    assert session.trace_fields(worker)["native_left_estimate_count"] == 0
    assert session.trace_fields(worker)["native_right_estimate_count"] == 0


def test_contact_change_invalidates_retained_results() -> None:
    """接触集合改变后旧结果失效，即使估计尚未到期。"""
    session, worker = _running()
    _update(session, worker, 0.5, native_pillar_states=((3, 1), (3, 1)))
    worker.native_slip_status = NativeSlipStatus(1, "stopped", "estimates_ready")
    _update(session, worker, 0.625, right_taxel_forces_n=((0.0, 0.0, 1.0), (0.0, 0.0, 0.0)))
    assert session.trace_fields(worker)["native_left_estimate_count"] == 0
    assert session.trace_fields(worker)["native_right_estimate_count"] == 0


@pytest.mark.parametrize("max_sessions,expected", [(1, 1), (2, 2)])
def test_restart_obeys_cooldown_new_stability_and_session_limit(
    max_sessions: int, expected: int
) -> None:
    """停止后的冷却与新的完整稳定窗口独立满足，会话总数不可超过上限。"""
    session, worker = _running(_config(max_sessions=max_sessions))
    worker.native_slip_status = NativeSlipStatus(1, "stopped", "duration_limit")
    for t in (0.5, 0.625, 0.75, 0.875, 1.0, 1.125):
        _update(session, worker, t)
        assert len(worker.starts) == 1
    _update(session, worker, 1.25)
    assert len(worker.starts) == expected
    if expected == 2:
        assert worker.starts[-1][0] == 2


def test_device_failure_is_propagated() -> None:
    """未确认停止等设备失败必须中断会话，而不能继续采集辨识证据。"""
    session, worker = _running()
    worker.native_slip_status = NativeSlipStatus(1, "failed", "stop_unconfirmed")
    with pytest.raises(RuntimeError, match="stop_unconfirmed"):
        _update(session, worker, 0.5)


@pytest.mark.parametrize(
    "changes",
    [
        {"max_duration_s": float("inf")},
        {"stable_duration_s": 0.0},
        {"max_sessions": True},
        {"min_estimates_per_side": 1.5},
        {"contact_off_n": 0.05},
        {"confirmation_timeout_s": 2.0},
    ],
)
def test_invalid_configuration_is_rejected(changes: dict[str, object]) -> None:
    """拒绝不可结束会话、非整数数量和矛盾的阈值配置。"""
    with pytest.raises(ValueError):
        _config(**changes)

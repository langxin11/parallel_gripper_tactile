"""有限原厂检测会话的离线命令、租期和双侧确认契约。"""

from __future__ import annotations

import pytest

from papillarray_hardware.native_slip import NativeSlipLease


class _Client:
    """仅记录启停命令，不访问串口或真实设备。"""

    def __init__(self, *, fail_start: bool = False) -> None:
        """初始化命令记录及可注入的启动故障。"""
        self.commands: list[str] = []
        self.fail_start = fail_start

    def start_slip_detection(self) -> None:
        """记录启动，模拟命令可能部分发送后发生故障。"""
        self.commands.append("start")
        if self.fail_start:
            raise OSError("模拟串口启动写入失败")

    def stop_slip_detection(self) -> None:
        """记录停止。"""
        self.commands.append("stop")


def _start(lease: NativeSlipLease, client: _Client, *, confirmed: bool = True) -> None:
    """在固定时钟建立五秒租期，便于直接检验边界。"""
    lease.request_start(1, max_duration_s=5.0, confirmation_timeout_s=1.0)
    lease.tick(client, 10.0)
    if confirmed:
        lease.tick(client, 10.5, (True, True))


def test_unrequested_lease_never_sends_commands() -> None:
    """普通采集和退出不隐式启动检测服务。"""
    lease, client = NativeSlipLease(), _Client()
    lease.request_stop()
    lease.tick(client, 10.0)
    lease.tick(client, 20.0, (False, False))
    lease.close(client)
    assert client.commands == []
    assert lease.status.phase == "idle"


def test_queued_stop_cancels_without_starting_device() -> None:
    """采集线程尚未发送启动时，取消无需启停设备。"""
    lease, client = NativeSlipLease(), _Client()
    lease.request_start(1, max_duration_s=5.0, confirmation_timeout_s=1.0)
    lease.request_stop("contact_lost")
    lease.tick(client, 10.0, (True, True))
    lease.close(client)
    assert client.commands == []
    assert lease.status.phase == "stopped"
    assert lease.status.reason == "contact_lost"


@pytest.mark.parametrize("feedback", [None, (), (True,), (True, False), (False, True)])
def test_start_requires_new_complete_bilateral_confirmation(
    feedback: tuple[bool, ...] | None,
) -> None:
    """启动前读取的状态与不完整状态均不能充当启动确认。"""
    lease, client = NativeSlipLease(), _Client()
    lease.request_start(1, max_duration_s=5.0, confirmation_timeout_s=1.0)
    lease.tick(client, 10.0, (True, True))
    assert lease.status.phase == "starting"
    lease.tick(client, 10.2, feedback)
    assert lease.status.phase == "starting"
    lease.tick(client, 10.5, (True, True))
    assert lease.status.phase == "active"
    assert lease.status.started_s == 10.0
    assert client.commands == ["start"]


def test_total_duration_includes_starting_and_expires_without_feedback() -> None:
    """总租期从发出启动起计算，即使后续完全没有包也发送停止。"""
    lease, client = NativeSlipLease(), _Client()
    _start(lease, client)
    lease.tick(client, 14.99)
    assert lease.status.phase == "active"
    lease.tick(client, 15.0)
    assert client.commands == ["start", "stop"]
    assert lease.status.phase == "stopping"
    assert lease.status.reason == "duration_limit"
    assert lease.status.stop_sent_s == 15.0


def test_manual_session_has_no_duration_limit_and_stops_on_request() -> None:
    """无租期会话保持活动，直到操作者显式请求停止。"""
    lease, client = NativeSlipLease(), _Client()
    lease.request_start(1, max_duration_s=None, confirmation_timeout_s=1.0)
    lease.tick(client, 10.0)
    lease.tick(client, 10.5, (True, True))
    lease.tick(client, 10_000.0, (True, True))
    assert lease.status.phase == "active"
    assert lease.read_timeout_s(10_000.0, 0.2) is None
    lease.request_stop("operator_requested")
    lease.tick(client, 10_000.1, (True, True))
    assert lease.status.phase == "stopping"
    assert lease.status.reason == "operator_requested"
    assert client.commands == ["start", "stop"]


def test_start_confirmation_timeout_stops_even_without_feedback() -> None:
    """设备未确认启动时也必须停止可能已开启的服务。"""
    lease, client = NativeSlipLease(), _Client()
    _start(lease, client, confirmed=False)
    lease.tick(client, 11.0)
    assert client.commands == ["start", "stop"]
    assert lease.status.phase == "stopping"
    assert lease.status.reason == "start_unconfirmed"


@pytest.mark.parametrize("phase", ["queued", "starting", "active", "stopping", "failed"])
def test_existing_session_cannot_be_overwritten_or_renewed(phase: str) -> None:
    """新编号和相同编号都不能覆盖尚未确认停止的会话。"""
    lease, client = NativeSlipLease(), _Client()
    lease.request_start(1, max_duration_s=5.0, confirmation_timeout_s=1.0)
    if phase != "queued":
        lease.tick(client, 10.0)
    if phase in {"active", "stopping", "failed"}:
        lease.tick(client, 10.5, (True, True))
    if phase in {"stopping", "failed"}:
        lease.request_stop()
        lease.tick(client, 11.0)
    if phase == "failed":
        with pytest.raises(RuntimeError, match="停止未获双侧"):
            lease.tick(client, 12.0)
    before = lease.status
    for session_id in (1, 2):
        with pytest.raises(RuntimeError, match="尚未确认停止"):
            lease.request_start(session_id, max_duration_s=20.0, confirmation_timeout_s=2.0)
    assert lease.status == before


def test_stop_needs_feedback_from_a_later_tick() -> None:
    """停止命令同帧的全关闭状态不能确认，下一完整双侧包才可以。"""
    lease, client = NativeSlipLease(), _Client()
    _start(lease, client)
    lease.request_stop("estimate_ready")
    lease.tick(client, 11.0, (False, False))
    assert lease.status.phase == "stopping"
    lease.tick(client, 11.1, (False, False))
    assert lease.status.phase == "stopped"
    assert lease.status.reason == "estimate_ready"
    lease.close(client)
    assert client.commands == ["start", "stop"]


@pytest.mark.parametrize(
    "feedback", [None, (), (False,), (False, True), (True, False), (True, True)]
)
def test_missing_or_partial_stop_confirmation_times_out(feedback: tuple[bool, ...] | None) -> None:
    """缺失、单侧和仍开启的反馈都不能被误判为停止成功。"""
    lease, client = NativeSlipLease(), _Client()
    _start(lease, client)
    lease.request_stop()
    lease.tick(client, 11.0)
    lease.tick(client, 11.5, feedback)
    assert lease.status.phase == "stopping"
    with pytest.raises(RuntimeError, match="停止未获双侧"):
        lease.tick(client, 12.0, feedback)
    assert lease.status.phase == "failed"
    assert lease.status.reason == "stop_unconfirmed"


@pytest.mark.parametrize("confirmed", [False, True])
def test_finally_close_stops_possible_active_service_without_faking_confirmation(
    confirmed: bool,
) -> None:
    """异常退出仍发送停止，但没有后续包就只能记录未确认。"""
    lease, client = NativeSlipLease(), _Client()
    try:
        _start(lease, client, confirmed=confirmed)
    finally:
        lease.close(client)
    assert client.commands == ["start", "stop"]
    assert lease.status.phase == "failed"
    assert lease.status.reason == "closed_without_confirmation"


def test_start_write_error_still_requires_finally_stop() -> None:
    """启动写入异常可能已经送达设备，退出时仍须尝试停止。"""
    lease, client = NativeSlipLease(), _Client(fail_start=True)
    with pytest.raises(OSError, match="模拟串口"):
        try:
            _start(lease, client)
        finally:
            lease.close(client)
    assert client.commands == ["start", "stop"]
    assert lease.status.phase == "failed"


def test_restart_requires_stopped_state_and_strictly_increasing_id() -> None:
    """成功停止后仍拒绝旧会话编号，新会话获得独立租期。"""
    lease, client = NativeSlipLease(), _Client()
    _start(lease, client)
    lease.request_stop()
    lease.tick(client, 11.0)
    lease.tick(client, 11.1, (False, False))
    with pytest.raises(ValueError, match="严格递增"):
        lease.request_start(1, max_duration_s=5.0, confirmation_timeout_s=1.0)
    lease.request_start(2, max_duration_s=3.0, confirmation_timeout_s=0.5)
    lease.tick(client, 20.0)
    lease.tick(client, 20.1, (True, True))
    lease.tick(client, 23.0)
    assert lease.status.session_id == 2
    assert lease.status.started_s == 20.0
    assert lease.status.reason == "duration_limit"
    assert client.commands == ["start", "stop", "start", "stop"]


@pytest.mark.parametrize("session_id", [0, -1, True, 1.5, "1", None])
def test_invalid_session_id_is_rejected(session_id: object) -> None:
    """会话编号必须是严格正整数，布尔值不能冒充编号。"""
    lease = NativeSlipLease()
    with pytest.raises(ValueError, match="正整数"):
        lease.request_start(session_id, max_duration_s=5.0, confirmation_timeout_s=1.0)
    assert lease.status.phase == "idle"


@pytest.mark.parametrize("invalid", [0.0, -1.0, float("inf"), float("-inf"), float("nan"), True])
@pytest.mark.parametrize("field", ["max_duration_s", "confirmation_timeout_s"])
def test_invalid_time_limits_are_rejected(field: str, invalid: float) -> None:
    """租期和确认时限拒绝非正、非有限及布尔输入。"""
    lease = NativeSlipLease()
    limits = {"max_duration_s": 5.0, "confirmation_timeout_s": 1.0}
    limits[field] = invalid
    with pytest.raises(ValueError, match="正有限数"):
        lease.request_start(1, **limits)
    assert lease.status.phase == "idle"


@pytest.mark.parametrize("confirmation", [5.0, 6.0])
def test_confirmation_must_be_shorter_than_total_duration(confirmation: float) -> None:
    """确认时间不能占满或超过全部检测租期。"""
    with pytest.raises(ValueError, match="小于检测租期"):
        NativeSlipLease().request_start(1, max_duration_s=5.0, confirmation_timeout_s=confirmation)

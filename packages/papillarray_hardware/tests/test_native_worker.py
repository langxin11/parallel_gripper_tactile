"""原厂短会话在真实采集线程中的离线生命周期与串口所有权测试。"""

from __future__ import annotations

import json
import queue
import threading

import numpy as np
import pytest

from papillarray_hardware import (
    PapillArraySerialConfig,
    PtsPacket,
    PtsReadDiagnostics,
    PtsReadTimeout,
    TactileWorker,
)


def _timeout() -> PtsReadTimeout:
    """构造假串口的正常空读取，不借助设备或真实时间推进租期。"""
    return PtsReadTimeout(
        "离线队列暂时为空",
        PtsReadDiagnostics(0, 0, 0, 0, 0, 0, 0, None, ""),
    )


class _QueuedClient:
    """用队列投递包，用事件观测命令及串口关闭。"""

    def __init__(self, *, start_error: bool = False) -> None:
        self.items: queue.Queue[PtsPacket | BaseException] = queue.Queue()
        self.commands: list[tuple[str, int]] = []
        self.started = threading.Event()
        self.stopped = threading.Event()
        self.closed = threading.Event()
        self.reading = threading.Event()
        self.start_error = start_error
        self.read_budgets: queue.Queue[tuple[float, float | None]] = queue.Queue()
        self.budget_clock = lambda: 0.0

    def __enter__(self):
        return self

    def __exit__(self, *_args: object) -> None:
        self.closed.set()

    def configure_stream(self) -> None:
        self.commands.append(("configure", threading.get_ident()))

    def start_slip_detection(self) -> None:
        self.commands.append(("start", threading.get_ident()))
        self.started.set()
        if self.start_error:
            raise OSError("启动写入失败")

    def stop_slip_detection(self) -> None:
        self.commands.append(("stop", threading.get_ident()))
        self.stopped.set()

    def read_packet(self, *, packet_timeout_s: float | None = None) -> PtsPacket:
        self.read_budgets.put((self.budget_clock(), packet_timeout_s))
        self.reading.set()
        try:
            item = self.items.get(timeout=min(0.01, packet_timeout_s or 0.01))
        except queue.Empty:
            raise _timeout() from None
        if isinstance(item, BaseException):
            raise item
        return item


class _Session:
    """管理假设备、人工时钟及采集记录，避免依靠 sleep 判断状态。"""

    def __init__(self, *, start_error: bool = False) -> None:
        self.client = _QueuedClient(start_error=start_error)
        self.now_s = 0.0
        self.client.budget_clock = lambda: self.now_s
        self.counter = 0
        self.records: queue.Queue[dict[str, object]] = queue.Queue()
        self.worker = TactileWorker(
            PapillArraySerialConfig(timeout_s=0.01, packet_timeout_s=0.02),
            clear_bias=False,
            client_factory=lambda _config: self.client,
            clock=lambda: self.now_s,
            sample_sink=self.records.put,
        )

    def request(self) -> None:
        self.worker.request_native_slip(7, max_duration_s=10.0, confirmation_timeout_s=1.0)

    def feed(self, active: tuple[bool, ...]) -> dict[str, object]:
        self.counter += 1
        self.client.items.put(
            PtsPacket(
                packet_counter=self.counter,
                timestamp_us=self.counter * 1000,
                pillar_forces=[np.array([[0.1, 0.2, 1.0]]), np.array([[0.2, 0.3, 2.0]])],
                pillar_displacements=[np.zeros((1, 3)), np.zeros((1, 3))],
                global_forces=[np.array([0.1, 0.2, 1.0]), np.array([0.2, 0.3, 2.0])],
                global_torques=[np.zeros(3), np.zeros(3)],
                pillar_slip_states=[np.array([3], dtype=np.int8), np.array([2], dtype=np.int8)],
                pillar_friction_estimates=[np.array([0.25]), np.array([0.5])],
                slip_detection_active=list(active),
            )
        )
        return self.records.get(timeout=1.0)

    def activate(self) -> None:
        self.request()
        self.worker.start()
        assert self.client.started.wait(1.0)
        assert self.feed((True, True))["native_session_phase"] == "active"

    def finish(self) -> None:
        if not self.client.closed.is_set():
            if self.worker.native_slip_status.phase in {"starting", "active", "stopping"}:
                self.worker.stop_native_slip()
                assert self.client.stopped.wait(1.0)
                self.feed((False, False))
            self.worker.stop()

    def read_budget_at(self, at_s: float) -> float | None:
        """等待人工时钟推进后的首次读取预算，跳过已开始读取的旧记录。"""
        while True:
            when, budget = self.client.read_budgets.get(timeout=1.0)
            if when >= at_s:
                return budget


def test_default_worker_never_sends_native_commands() -> None:
    """默认采集仅配置数据流，快照中没有虚构的原厂会话。"""
    session = _Session()
    session.worker.start()
    try:
        record = session.feed(())
        assert record["native_session_id"] == 0
        assert record["native_session_phase"] == "idle"
    finally:
        session.finish()
    assert [name for name, _ in session.client.commands] == ["configure"]
    assert session.read_budget_at(0.0) is None


def test_native_commands_use_owner_thread_and_require_bilateral_feedback() -> None:
    """排队不写串口，单侧启动或停止反馈均不能完成双侧会话确认。"""
    session = _Session()
    caller_id = threading.get_ident()
    session.request()
    assert session.client.commands == []
    session.worker.start()
    try:
        assert session.client.started.wait(1.0)
        assert session.feed((True, False))["native_session_phase"] == "starting"
        active_record = session.feed((True, True))
        assert active_record["native_session_phase"] == "active"
        assert active_record["native_session_id"] == 7
        assert active_record["native_pillar_states"] == ((3,), (2,))
        assert active_record["native_pillar_friction"] == ((0.25,), (0.5,))
        assert json.loads(json.dumps(active_record, allow_nan=False))["native_session_id"] == 7

        session.worker.stop_native_slip("estimate_complete")
        assert session.client.stopped.wait(1.0)
        partial_record = session.feed((False, True))
        assert partial_record["native_session_phase"] == "stopping"
        assert partial_record["native_session_id"] == 7
        stopped_record = session.feed((False, False))
        assert stopped_record["native_session_phase"] == "stopped"
        assert stopped_record["native_session_reason"] == "estimate_complete"
        assert stopped_record["native_session_id"] == 7
    finally:
        session.finish()
    owners = {owner for _, owner in session.client.commands}
    assert len(owners) == 1
    assert caller_id not in owners
    assert [name for name, _ in session.client.commands] == ["configure", "start", "stop"]


def test_owner_enforces_deadline_without_any_control_thread_call() -> None:
    """控制调用暂停且串口暂时无新包时，采集线程仍依据租期发出停止。"""
    session = _Session()
    session.activate()
    try:
        session.now_s = 10.0
        session.client.items.put(_timeout())
        assert session.client.stopped.wait(1.0)
        assert session.worker.native_slip_status.phase == "stopping"
        assert session.worker.native_slip_status.reason == "duration_limit"
        assert session.feed((False, False))["native_session_phase"] == "stopped"
    finally:
        session.finish()


def test_each_native_phase_limits_packet_wait_to_remaining_deadline() -> None:
    """启动、运行和停止确认均收紧读包预算，断流不能等到普通读包超时。"""
    session = _Session()
    session.request()
    session.worker.start()
    try:
        assert session.client.started.wait(1.0)
        session.now_s = 0.999
        session.client.items.put(_timeout())
        starting_budget = session.read_budget_at(0.999)
        assert starting_budget is not None and 0 < starting_budget <= 0.001001
        assert session.feed((True, True))["native_session_phase"] == "active"

        session.now_s = 9.999
        session.client.items.put(_timeout())
        active_budget = session.read_budget_at(9.999)
        assert active_budget is not None and 0 < active_budget <= 0.001001

        session.worker.stop_native_slip()
        assert session.client.stopped.wait(1.0)
        session.now_s = 10.998
        session.client.items.put(_timeout())
        stopping_budget = session.read_budget_at(10.998)
        assert stopping_budget is not None and 0 < stopping_budget <= 0.001001
        assert session.feed((False, False))["native_session_phase"] == "stopped"
    finally:
        session.finish()


def test_worker_shutdown_keeps_reading_until_both_sides_are_inactive() -> None:
    """退出等待设备确认，收到单侧停止时仍保留串口和读取循环。"""
    session = _Session()
    session.activate()
    finished = threading.Event()
    failures: list[BaseException] = []

    def stop_worker() -> None:
        """从独立调用线程等待停止，以便测试继续提供设备反馈。"""
        try:
            session.worker.stop()
        except BaseException as error:  # noqa: BLE001
            failures.append(error)
        finally:
            finished.set()

    stopper = threading.Thread(target=stop_worker)
    stopper.start()
    try:
        assert session.client.stopped.wait(1.0)
        assert session.feed((False, True))["native_session_phase"] == "stopping"
        assert not finished.is_set()
        assert not session.client.closed.is_set()
        assert session.feed((False, False))["native_session_phase"] == "stopped"
        assert finished.wait(1.0)
        assert not failures
        assert session.client.closed.is_set()
    finally:
        session.finish()
        stopper.join(timeout=1.0)


@pytest.mark.parametrize("failure", ["start", "read"])
def test_native_or_stream_failure_attempts_stop_and_reports_missing_confirmation(
    failure: str,
) -> None:
    """启动写入或采集异常退出都尽力停止，缺少设备回执时不能报告正常结束。"""
    session = _Session(start_error=failure == "start")
    if failure == "start":
        session.request()
        session.worker.start()
    else:
        session.activate()
        session.client.items.put(OSError("读取断开"))
    assert session.client.closed.wait(1.0)
    # 用不可能被更新的接收时间阻塞，直到线程经条件变量锁存采集错误后抛出。
    with pytest.raises(RuntimeError, match="启动写入失败|读取断开"):
        session.worker.wait_for_update(float("inf"), 1.0)
    assert session.client.stopped.is_set()
    assert session.worker.native_slip_status.phase == "failed"
    assert session.worker.native_slip_status.reason == "closed_without_confirmation"
    with pytest.raises(RuntimeError, match="停止未确认"):
        session.worker.stop()
    assert [name for name, _ in session.client.commands] == ["configure", "start", "stop"]


def test_stop_confirmation_timeout_retries_on_exit_and_remains_failure() -> None:
    """停止回执超时会退出采集并重试停止，退出不能抹去未确认故障。"""
    session = _Session()
    session.activate()
    session.worker.stop_native_slip()
    assert session.client.stopped.wait(1.0)
    session.now_s = 1.0
    session.client.items.put(_timeout())
    assert session.client.closed.wait(1.0)
    # 用不可能被更新的接收时间阻塞，直到线程经条件变量锁存采集错误后抛出。
    with pytest.raises(RuntimeError, match="停止未获双侧设备确认"):
        session.worker.wait_for_update(float("inf"), 1.0)
    assert session.worker.native_slip_status.phase == "failed"
    assert session.worker.native_slip_status.reason == "stop_unconfirmed"
    with pytest.raises(RuntimeError, match="停止未确认"):
        session.worker.stop()
    assert [name for name, _ in session.client.commands] == ["configure", "start", "stop", "stop"]

"""验证触觉后台采集的首次连接重试。"""

import time
from types import SimpleNamespace

from papillarray_hardware import PapillArraySerialConfig, PtsReadDiagnostics, PtsReadTimeout

from dmgripper_experiments.tactile import TactileWorker


class _RetryClient:
    """第一次读取超时、后续持续返回有效包的假客户端。"""

    def __init__(self, _config: PapillArraySerialConfig) -> None:
        self.calls = 0
        self.commands: list[str] = []

    def __enter__(self):
        return self

    def __exit__(self, *_args: object) -> None:
        return None

    def configure_stream(self) -> None:
        self.commands.append("configure")

    def clear_bias(self) -> None:
        self.commands.append("bias")

    def read_packet(self):
        self.calls += 1
        if self.calls == 1:
            diagnostics = PtsReadDiagnostics(0, 0, 0, 0, 0, 0, 0, None, "")
            raise PtsReadTimeout("首次启动暂无数据", diagnostics)
        time.sleep(0.001)
        return SimpleNamespace(
            packet_counter=self.calls,
            timestamp_us=self.calls * 2_000,
            global_forces=([0.0, 0.0, 0.01], [0.0, 0.0, 0.02]),
        )


def test_worker_retries_transient_packet_timeout() -> None:
    """首次单包超时不会终止后台线程。"""
    worker = TactileWorker(
        PapillArraySerialConfig(timeout_s=0.05, packet_timeout_s=0.2),
        clear_bias=True,
        client_factory=_RetryClient,
    )
    worker.start()
    try:
        snapshot = worker.wait_for_update(None, 0.5)
        assert snapshot.left_force_n == 0.01
        assert snapshot.right_force_n == 0.02
    finally:
        worker.stop()


def test_bias_is_sent_after_stream_produces_first_packet() -> None:
    """先确认数据流，再按 ROS 2 运行顺序发送清零命令。"""
    client = _RetryClient(PapillArraySerialConfig())
    worker = TactileWorker(
        PapillArraySerialConfig(timeout_s=0.05, packet_timeout_s=0.2),
        clear_bias=True,
        client_factory=lambda _config: client,
    )
    worker.start()
    try:
        worker.wait_for_update(None, 0.5)
        assert client.commands == ["configure", "bias"]
    finally:
        worker.stop()


class _TimeoutClient(_RetryClient):
    """始终以带诊断的 PTS 超时结束读取。"""

    def read_packet(self):
        diagnostics = PtsReadDiagnostics(12, 0, 0, 0, 0, 0, 0, None, "abcd", empty_reads=2)
        raise PtsReadTimeout("暂无数据", diagnostics)


def test_overall_timeout_keeps_last_protocol_diagnostics() -> None:
    """重复尝试耗尽总时限时仍报告底层接收证据。"""
    worker = TactileWorker(
        PapillArraySerialConfig(timeout_s=0.01, packet_timeout_s=0.02),
        clear_bias=False,
        client_factory=_TimeoutClient,
    )
    worker.start()
    try:
        try:
            worker.wait_for_update(None, 0.03)
        except TimeoutError as error:
            assert "接收字节=12" in str(error)
            assert "原始预览=abcd" in str(error)
        else:
            raise AssertionError("预期触觉总等待超时")
    finally:
        worker.stop()


def test_worker_preserves_all_axes_in_snapshot_and_record() -> None:
    """控制快照和原始记录必须保留有符号三轴力，不能伪造零切向力。"""
    from dataclasses import asdict

    class ThreeAxisClient(_RetryClient):
        def read_packet(self):
            packet = super().read_packet()
            packet.global_forces = ([0.12, -0.34, 0.56], [-0.78, 0.9, -0.12])
            return packet

    records = []
    worker = TactileWorker(
        PapillArraySerialConfig(timeout_s=0.05, packet_timeout_s=0.2),
        clear_bias=False,
        client_factory=ThreeAxisClient,
        sample_sink=records.append,
    )
    worker.start()
    try:
        snapshot = worker.wait_for_update(None, 0.5)
        assert snapshot.raw_left_fx_n == 0.12
        assert snapshot.raw_left_fy_n == -0.34
        assert snapshot.raw_right_fx_n == -0.78
        assert snapshot.raw_right_fy_n == 0.9
        assert snapshot.raw_right_fz_n == -0.12
        assert snapshot.right_force_n == 0.0
        assert asdict(snapshot) in records
    finally:
        worker.stop()

"""PapillArray 后台采集与最新完整快照。"""

from __future__ import annotations

import threading
import time
import math
from collections.abc import Callable
from dataclasses import dataclass

from papillarray_hardware import PapillArraySerialClient, PapillArraySerialConfig, PtsReadTimeout


@dataclass(frozen=True, slots=True)
class TactileSnapshot:
    """一次双侧法向力快照。"""

    received_at_s: float
    packet_counter: int
    timestamp_us: int
    left_force_n: float
    right_force_n: float
    raw_left_fz_n: float
    raw_right_fz_n: float


class _FirstOrderLowPassFilter:
    """与 ROS 2 接触处理器一致的精确极点一阶低通。"""

    def __init__(self, cutoff_hz: float, reset_gap_s: float) -> None:
        """保存滤波参数并清空状态。"""
        self._cutoff_hz = cutoff_hz
        self._reset_gap_s = reset_gap_s
        self.reset()

    def reset(self) -> None:
        """让下一个样本直接播种输出。"""
        self._value: float | None = None
        self._time_s: float | None = None

    def filter(self, sample: float, sample_time_s: float) -> float:
        """按设备时间戳更新滤波值。"""
        if self._value is None or self._time_s is None:
            return self._seed(sample, sample_time_s)
        dt_s = sample_time_s - self._time_s
        if dt_s <= 0.0 or dt_s > self._reset_gap_s:
            return self._seed(sample, sample_time_s)
        alpha = -math.expm1(-2.0 * math.pi * self._cutoff_hz * dt_s)
        self._value += alpha * (sample - self._value)
        self._time_s = sample_time_s
        return self._value

    def _seed(self, sample: float, sample_time_s: float) -> float:
        """用当前样本播种。"""
        self._value = sample
        self._time_s = sample_time_s
        return sample


ClientFactory = Callable[[PapillArraySerialConfig], PapillArraySerialClient]


class TactileWorker:
    """由单一后台线程持有触觉串口并发布最新快照。"""

    def __init__(
        self,
        config: PapillArraySerialConfig,
        *,
        clear_bias: bool,
        client_factory: ClientFactory = PapillArraySerialClient,
        clock: Callable[[], float] = time.monotonic,
        cutoff_hz: float = 10.0,
        filter_reset_gap_s: float = 0.1,
    ) -> None:
        """保存配置，不打开设备。"""
        self._config = config
        self._clear_bias = clear_bias
        self._client_factory = client_factory
        self._clock = clock
        self._left_filter = _FirstOrderLowPassFilter(cutoff_hz, filter_reset_gap_s)
        self._right_filter = _FirstOrderLowPassFilter(cutoff_hz, filter_reset_gap_s)
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._snapshot: TactileSnapshot | None = None
        self._error: BaseException | None = None
        self._last_timeout: PtsReadTimeout | None = None

    def start(self) -> None:
        """启动唯一的触觉串口所有者线程。"""
        if self._thread is not None:
            raise RuntimeError("触觉采集线程已经启动")
        self._thread = threading.Thread(target=self._run, name="papillarray-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """请求采集结束并等待串口关闭。"""
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=self._config.packet_timeout_s + self._config.timeout_s + 1.0)
            if thread.is_alive():
                raise RuntimeError("触觉采集线程未能在时限内退出")

    def wait_for_update(
        self, previous_received_at_s: float | None, timeout_s: float
    ) -> TactileSnapshot:
        """等待首个或比给定接收时间更新的触觉快照。"""
        deadline = self._clock() + timeout_s
        with self._condition:
            while True:
                if self._error is not None:
                    raise RuntimeError(f"触觉采集失败：{self._error}") from self._error
                if self._snapshot is not None and (
                    previous_received_at_s is None
                    or self._snapshot.received_at_s > previous_received_at_s
                ):
                    return self._snapshot
                remaining = deadline - self._clock()
                if remaining <= 0.0:
                    detail = _timeout_detail(self._last_timeout)
                    raise TimeoutError(f"等待新的触觉快照超时{detail}")
                self._condition.wait(remaining)

    def latest(self) -> TactileSnapshot:
        """返回最新快照，并传播后台采集异常。"""
        with self._condition:
            if self._error is not None:
                raise RuntimeError(f"触觉采集失败：{self._error}") from self._error
            if self._snapshot is None:
                raise RuntimeError("尚未收到触觉快照")
            return self._snapshot

    def _run(self) -> None:
        """打开、配置并持续读取双侧全局 Fz。"""
        try:
            with self._client_factory(self._config) as client:
                client.configure_stream()
                bias_pending = self._clear_bias
                while not self._stop.is_set():
                    try:
                        packet = client.read_packet()
                    except PtsReadTimeout as error:
                        # Controller 首次收到采样率或清零命令后可能短暂无输出。
                        # 单包超时只结束本次尝试；首包总等待和运行中新鲜度由上层分别约束。
                        with self._condition:
                            self._last_timeout = error
                            self._condition.notify_all()
                        continue
                    if bias_pending:
                        # ROS 2 驱动先建立稳定数据流，再由同一 I/O 所有者发送 bias。
                        # 首个有效包证明采样配置已经生效；该包属于 bias 前数据，直接丢弃。
                        client.clear_bias()
                        self._left_filter.reset()
                        self._right_filter.reset()
                        bias_pending = False
                        continue
                    sample_time_s = packet.timestamp_us * 1e-6
                    raw_left_fz_n = float(packet.global_forces[0][2])
                    raw_right_fz_n = float(packet.global_forces[1][2])
                    snapshot = TactileSnapshot(
                        received_at_s=self._clock(),
                        packet_counter=packet.packet_counter,
                        timestamp_us=packet.timestamp_us,
                        left_force_n=max(
                            0.0, self._left_filter.filter(raw_left_fz_n, sample_time_s)
                        ),
                        right_force_n=max(
                            0.0, self._right_filter.filter(raw_right_fz_n, sample_time_s)
                        ),
                        raw_left_fz_n=raw_left_fz_n,
                        raw_right_fz_n=raw_right_fz_n,
                    )
                    with self._condition:
                        self._snapshot = snapshot
                        self._last_timeout = None
                        self._condition.notify_all()
        except BaseException as error:  # noqa: BLE001
            if not self._stop.is_set():
                with self._condition:
                    self._error = error
                    self._condition.notify_all()


def _timeout_detail(error: PtsReadTimeout | None) -> str:
    """把最后一次 PTS 超时压缩为可操作的诊断。"""
    if error is None:
        return ""
    diagnostics = error.diagnostics
    return (
        "；最后一次 PTS 诊断："
        f"接收字节={diagnostics.received_bytes}，"
        f"空读取={diagnostics.empty_reads}，"
        f"候选帧={diagnostics.candidate_frames}，"
        f"校验失败={diagnostics.checksum_failures}，"
        f"结构失败={diagnostics.protocol_failures}，"
        f"原始预览={diagnostics.raw_hex_preview or '无'}"
    )

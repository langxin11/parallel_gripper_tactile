"""PapillArray 双侧采集会话、完整性诊断与最新快照。"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from typing import Literal

import numpy as np

from .client import PapillArraySerialClient, PapillArraySerialConfig
from .protocol import PtsPacket, PtsReadTimeout

CounterEvent = Literal["first", "consecutive", "gap", "wrap"]
TaxelForces = tuple[tuple[float, float, float], ...]
_COUNTER_MODULUS = 2**32


class PacketIntegrityError(RuntimeError):
    """表示 PTS 包的值、拓扑或连续性不适合继续用于实时采集。"""


@dataclass(frozen=True, slots=True)
class PacketIntegrityDiagnostics:
    """一个完整 PTS 包的拓扑与时序诊断。

    Args:
        sensor_count: 当前包的传感器数。
        taxel_counts: 各传感器当前包的 taxel 数。
        counter_event: 包计数器相对上一有效包的变化类型。
        counter_gap: 计数器前进时可解释的缺包数。
    """

    sensor_count: int
    taxel_counts: tuple[int, ...]
    counter_event: CounterEvent
    counter_gap: int | None


class PacketIntegrityTracker:
    """验证 PTS 包内容，并在一次采集会话内跟踪拓扑与时序。"""

    def __init__(self, expected_sensors: int) -> None:
        """保存期望传感器数，尚不锁定 taxel 拓扑。"""
        if (
            isinstance(expected_sensors, bool)
            or not isinstance(expected_sensors, int)
            or expected_sensors <= 0
        ):
            raise ValueError("期望传感器数必须是正整数")
        self._expected_sensors = expected_sensors
        self._taxel_counts: tuple[int, ...] | None = None
        self._previous_counter: int | None = None
        self._previous_timestamp_us: int | None = None

    @property
    def taxel_counts(self) -> tuple[int, ...] | None:
        """返回本次会话已经锁定的各传感器 taxel 数。"""
        return self._taxel_counts

    def reset_stream_sequence(self) -> None:
        """重置设备时序基线，同时保留本次会话已经确认的 taxel 拓扑。"""
        self._previous_counter = None
        self._previous_timestamp_us = None

    def inspect(self, packet: PtsPacket) -> PacketIntegrityDiagnostics:
        """验证一个包，并在全部检查通过后推进完整性基线。

        计数器缺包和合法回绕属于可记录诊断，不会拒绝包；重复、乱序、设备时间重复或回退
        会拒绝包。

        Args:
            packet: 待验证的原始 PTS 包。

        Returns:
            当前有效包的结构化诊断。

        Raises:
            PacketIntegrityError: 包内容、拓扑或设备时序不满足约束。
        """
        pillar_forces = getattr(packet, "pillar_forces", None)
        global_forces = getattr(packet, "global_forces", None)
        if not isinstance(pillar_forces, (list, tuple)):
            raise PacketIntegrityError("PTS 包缺少逐 taxel 力数组")
        sensor_count = len(pillar_forces)
        reported_sensor_count = getattr(packet, "n_sensors", sensor_count)
        if reported_sensor_count != sensor_count or sensor_count != self._expected_sensors:
            raise PacketIntegrityError(
                "PapillArray 传感器数与部署配置不一致："
                f"期望 {self._expected_sensors}，实际 {sensor_count}"
            )
        if not isinstance(global_forces, (list, tuple)) or len(global_forces) != sensor_count:
            raise PacketIntegrityError("PTS 包的全局力传感器数不一致")

        taxel_counts: list[int] = []
        for sensor_index in range(sensor_count):
            try:
                taxels = np.asarray(pillar_forces[sensor_index], dtype=np.float64)
            except (TypeError, ValueError) as error:
                raise PacketIntegrityError(
                    f"sensor {sensor_index} 逐 taxel 力无法转换为数值数组"
                ) from error
            if taxels.ndim != 2 or taxels.shape[1:] != (3,):
                raise PacketIntegrityError(f"sensor {sensor_index} 逐 taxel 力 shape 必须为 (N, 3)")
            if taxels.shape[0] == 0:
                raise PacketIntegrityError(f"sensor {sensor_index} 不包含 taxel")
            nonfinite = np.argwhere(~np.isfinite(taxels))
            if nonfinite.size:
                taxel_index, axis_index = (int(value) for value in nonfinite[0])
                axis = "xyz"[axis_index]
                raise PacketIntegrityError(
                    f"sensor {sensor_index} taxel {taxel_index} F{axis} 包含非有限数值"
                )
            try:
                global_force = np.asarray(global_forces[sensor_index], dtype=np.float64)
            except (TypeError, ValueError) as error:
                raise PacketIntegrityError(
                    f"sensor {sensor_index} 全局力无法转换为数值数组"
                ) from error
            if global_force.shape != (3,):
                raise PacketIntegrityError(f"sensor {sensor_index} 全局力 shape 必须为 (3,)")
            nonfinite = np.flatnonzero(~np.isfinite(global_force))
            if nonfinite.size:
                axis = "xyz"[int(nonfinite[0])]
                raise PacketIntegrityError(f"sensor {sensor_index} 全局 F{axis} 包含非有限数值")
            taxel_counts.append(int(taxels.shape[0]))

        counts = tuple(taxel_counts)
        if self._taxel_counts is not None and counts != self._taxel_counts:
            raise PacketIntegrityError(
                "PapillArray taxel 数在同一次采集会话中发生变化："
                f"期望 {self._taxel_counts}，实际 {counts}"
            )

        counter = _unsigned_integer(packet.packet_counter, 32, "packet_counter")
        timestamp_us = _unsigned_integer(packet.timestamp_us, 64, "timestamp_us")
        counter_event, counter_gap = _counter_status(self._previous_counter, counter)
        if self._previous_timestamp_us is not None and timestamp_us <= self._previous_timestamp_us:
            raise PacketIntegrityError("PapillArray 设备时间未严格递增")

        if self._taxel_counts is None:
            self._taxel_counts = counts
        self._previous_counter = counter
        self._previous_timestamp_us = timestamp_us
        return PacketIntegrityDiagnostics(
            sensor_count=sensor_count,
            taxel_counts=counts,
            counter_event=counter_event,
            counter_gap=counter_gap,
        )


def _unsigned_integer(value: object, bits: int, name: str) -> int:
    """验证设备无符号整数字段并返回普通 Python 整数。"""
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise PacketIntegrityError(f"{name} 必须是无符号 {bits} 位整数")
    number = int(value)
    if not 0 <= number < 2**bits:
        raise PacketIntegrityError(f"{name} 超出无符号 {bits} 位范围")
    return number


def _counter_status(previous: int | None, current: int) -> tuple[CounterEvent, int | None]:
    """按无符号 32 位半模规则分类计数器前进、缺包与回绕。"""
    if previous is None:
        return "first", None
    delta = (current - previous) % _COUNTER_MODULUS
    if delta == 0:
        raise PacketIntegrityError("PapillArray 包计数器重复")
    if delta >= _COUNTER_MODULUS // 2:
        raise PacketIntegrityError("PapillArray 包计数器乱序或异常回退")
    gap = delta - 1
    if current < previous:
        return "wrap", gap
    if gap:
        return "gap", gap
    return "consecutive", 0


@dataclass(frozen=True, slots=True)
class TactileSnapshot:
    """一次双侧全局力和逐 taxel 三轴原始力快照。"""

    received_at_s: float
    packet_counter: int
    timestamp_us: int
    left_force_n: float
    right_force_n: float
    raw_left_fz_n: float
    raw_right_fz_n: float
    raw_left_fx_n: float | None = None
    raw_left_fy_n: float | None = None
    raw_right_fx_n: float | None = None
    raw_right_fy_n: float | None = None
    left_taxel_forces_n: TaxelForces = ()
    right_taxel_forces_n: TaxelForces = ()
    counter_event: CounterEvent = "first"
    counter_gap: int | None = None


class FirstOrderLowPassFilter:
    """按设备时间戳更新的精确极点一阶低通滤波器。"""

    def __init__(self, cutoff_hz: float, reset_gap_s: float) -> None:
        """保存现有采集链路的滤波参数并清空状态。"""
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
    """由单一后台线程持有双侧触觉串口并发布最新完整快照。"""

    def __init__(
        self,
        config: PapillArraySerialConfig,
        *,
        clear_bias: bool,
        client_factory: ClientFactory = PapillArraySerialClient,
        clock: Callable[[], float] = time.monotonic,
        cutoff_hz: float = 10.0,
        filter_reset_gap_s: float = 0.1,
        sample_sink: Callable[[dict[str, object]], None] | None = None,
        snapshot_transform: Callable[[TactileSnapshot], TactileSnapshot] | None = None,
    ) -> None:
        """保存配置，不打开设备。"""
        if config.expected_sensors != 2:
            raise ValueError("双侧 TactileWorker 要求 expected_sensors=2")
        self._sample_sink = sample_sink
        self._snapshot_transform = snapshot_transform
        self._config = config
        self._clear_bias = clear_bias
        self._client_factory = client_factory
        self._clock = clock
        self._left_filter = FirstOrderLowPassFilter(cutoff_hz, filter_reset_gap_s)
        self._right_filter = FirstOrderLowPassFilter(cutoff_hz, filter_reset_gap_s)
        self._integrity = PacketIntegrityTracker(config.expected_sensors)
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
        """打开、配置并持续读取双侧全局力和逐 taxel 原始力。"""
        try:
            with self._client_factory(self._config) as client:
                client.configure_stream()
                bias_pending = self._clear_bias
                while not self._stop.is_set():
                    try:
                        packet = client.read_packet()
                    except PtsReadTimeout as error:
                        with self._condition:
                            self._last_timeout = error
                            self._condition.notify_all()
                        continue
                    diagnostics = self._integrity.inspect(packet)
                    if bias_pending:
                        client.clear_bias()
                        self._left_filter.reset()
                        self._right_filter.reset()
                        self._integrity.reset_stream_sequence()
                        bias_pending = False
                        continue
                    snapshot = self._snapshot_from_packet(packet, diagnostics)
                    if self._snapshot_transform is not None:
                        snapshot = self._snapshot_transform(snapshot)
                    if self._sample_sink is not None:
                        self._sample_sink(asdict(snapshot))
                    with self._condition:
                        self._snapshot = snapshot
                        self._last_timeout = None
                        self._condition.notify_all()
        except BaseException as error:  # noqa: BLE001
            if not self._stop.is_set():
                with self._condition:
                    self._error = error
                    self._condition.notify_all()

    def _snapshot_from_packet(
        self, packet: PtsPacket, diagnostics: PacketIntegrityDiagnostics
    ) -> TactileSnapshot:
        """把已验证包转换为不含 NumPy 值的双侧不可变快照。"""
        sample_time_s = int(packet.timestamp_us) * 1e-6
        raw_left_fz_n = float(packet.global_forces[0][2])
        raw_right_fz_n = float(packet.global_forces[1][2])
        return TactileSnapshot(
            received_at_s=self._clock(),
            packet_counter=int(packet.packet_counter),
            timestamp_us=int(packet.timestamp_us),
            left_force_n=max(0.0, self._left_filter.filter(raw_left_fz_n, sample_time_s)),
            right_force_n=max(0.0, self._right_filter.filter(raw_right_fz_n, sample_time_s)),
            raw_left_fz_n=raw_left_fz_n,
            raw_right_fz_n=raw_right_fz_n,
            raw_left_fx_n=float(packet.global_forces[0][0]),
            raw_left_fy_n=float(packet.global_forces[0][1]),
            raw_right_fx_n=float(packet.global_forces[1][0]),
            raw_right_fy_n=float(packet.global_forces[1][1]),
            left_taxel_forces_n=_taxel_forces(packet.pillar_forces[0]),
            right_taxel_forces_n=_taxel_forces(packet.pillar_forces[1]),
            counter_event=diagnostics.counter_event,
            counter_gap=diagnostics.counter_gap,
        )


def _taxel_forces(values: np.ndarray) -> TaxelForces:
    """将已验证的 NumPy taxel 数组转成 JSON 兼容的不可变普通浮点数。"""
    return tuple(tuple(float(component) for component in row) for row in values)


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

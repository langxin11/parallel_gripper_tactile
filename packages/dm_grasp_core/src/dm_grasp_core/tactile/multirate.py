"""独立采样的触觉预处理与不可变最新快照，不拥有控制时钟或设备。"""

from collections import deque
from dataclasses import dataclass
import math
from threading import Lock

import numpy as np

from .risk import TaxelRiskConfig, TaxelRiskObservation, TaxelRiskObserver


@dataclass(frozen=True, slots=True)
class TactileSamplingConfig:
    """采样与切向预处理配置；法向反馈滤波仍由原控制器独立拥有。"""

    period_s: float = 0.001
    median_window: int = 3
    stale_after_s: float = 0.01
    record_raw: bool = False

    def __post_init__(self) -> None:
        """拒绝不支持的窗口及退化时序。"""
        if any(
            isinstance(v, bool) or not math.isfinite(v) or v <= 0
            for v in (self.period_s, self.stale_after_s)
        ):
            raise ValueError("采样周期和新鲜度阈值必须为正有限数")
        if type(self.median_window) is not int or self.median_window not in (1, 3):
            raise ValueError("中值窗口只支持 1（旁路）或 3")
        if self.stale_after_s < self.period_s or not isinstance(self.record_raw, bool):
            raise ValueError("新鲜度阈值不得小于采样周期，日志开关必须为布尔值")


@dataclass(frozen=True, slots=True)
class FilteredTangentialLoad:
    """在采样侧完成唯一一次低通的分侧承载及总载荷变化率。"""

    left_n: float
    right_n: float
    rate_n_s: float

    def __post_init__(self) -> None:
        """保证控制侧不消费非有限承载。"""
        if not all(math.isfinite(v) for v in (self.left_n, self.right_n, self.rate_n_s)):
            raise ValueError("滤波承载必须有限")
        if min(self.left_n, self.right_n) < 0:
            raise ValueError("承载模长不得为负")


@dataclass(frozen=True, slots=True)
class TactileState:
    """一次完整采样快照；嵌套数据均不可变，时间必须来自同一单调时基。"""

    sequence_id: int
    sample_time_s: float
    left_taxels: tuple[tuple[float, ...], ...]
    right_taxels: tuple[tuple[float, ...], ...]
    load: FilteredTangentialLoad
    observation: TaxelRiskObservation
    valid: bool
    dropped_samples: int
    observed_events: int
    invalid_samples: int = 0
    latest_event: TaxelRiskObservation | None = None
    event_time_s: float | None = None

    def is_fresh(self, control_time_s: float, stale_after_s: float) -> bool:
        """负年龄与非有限时钟同样不可用，不能把未来数据视为新鲜。"""
        age = control_time_s - self.sample_time_s
        return math.isfinite(age) and 0 <= age <= stale_after_s


class LatestTactileBuffer:
    """单生产者／单消费者最新值缓冲；锁内仅交换不可变引用，不处理历史队列。"""

    def __init__(self) -> None:
        """创建空缓冲，不启动线程。"""
        self._lock = Lock()
        self._latest: TactileState | None = None

    def publish(self, state: TactileState) -> None:
        """拒绝重复或回退发布；坏帧状态也必须发布，不能被旧好帧掩盖。"""
        with self._lock:
            previous = self._latest
            if previous is not None and (
                state.sequence_id <= previous.sequence_id
                or state.sample_time_s <= previous.sample_time_s
            ):
                raise ValueError("触觉发布序号和时间必须递增")
            self._latest = state

    def latest(self) -> TactileState | None:
        """不等待新数据，只短暂锁定引用读取。"""
        with self._lock:
            return self._latest


class TactilePreprocessor:
    """逐采样预处理：原始安全检查 → 切向分量中值 → 聚合 → 承载低通。

    法向分量不经此中值链；采样侧只生成证据，事件是否增力由控制侧决定。
    单帧量程内毛刺可抑制，非有限或超量程输入不能靠滤波掩盖。
    """

    def __init__(
        self,
        config: TactileSamplingConfig,
        *,
        load_tau_s: float,
        observer_config: TaxelRiskConfig = TaxelRiskConfig(),
    ) -> None:
        """创建独占滤波状态；使用实际样本时间差积分。"""
        if not math.isfinite(load_tau_s) or load_tau_s <= 0:
            raise ValueError("载荷滤波时间常数必须为正有限数")
        self.config = config
        self.load_tau_s = load_tau_s
        self.observer = TaxelRiskObserver(observer_config)
        self._history: deque[np.ndarray] = deque(maxlen=config.median_window)
        self._last: TactileState | None = None
        self._filtered: np.ndarray | None = None
        self._dropped = 0
        self._events = 0
        self._invalid = 0
        self._latest_event: TaxelRiskObservation | None = None
        self._event_time_s: float | None = None

    def update(self, left, right, *, sample_time_s: float, sequence_id: int) -> TactileState:
        """处理双侧各九点三轴力；序号间断计数，长间断重建滤波历史。"""
        if type(sequence_id) is not int or sequence_id < 0 or not math.isfinite(sample_time_s):
            raise ValueError("采样序号必须为非负整数，时间必须有限")
        previous = self._last
        if previous is not None and (
            sequence_id <= previous.sequence_id or sample_time_s <= previous.sample_time_s
        ):
            raise ValueError("采样序号和时间必须递增")
        dt = self.config.period_s if previous is None else sample_time_s - previous.sample_time_s
        arrays = np.asarray((left, right), dtype=float)
        if arrays.shape != (2, 9, 3):
            raise ValueError("触觉采样必须为双侧各九点三轴力")
        if previous is not None:
            self._dropped += sequence_id - previous.sequence_id - 1
        limits = self.observer.config
        valid = bool(
            np.isfinite(arrays).all()
            and (np.abs(arrays[:, :, :2]) < limits.tangential_range_n).all()
            and (np.abs(arrays[:, :, 2]) < limits.normal_range_n).all()
        )
        if not valid or dt > self.config.stale_after_s or (previous and not previous.valid):
            self._history.clear()
            self._filtered = None
            self.observer.reset()
            self._latest_event = None
            self._event_time_s = None
        processed = arrays.copy()
        if valid:
            # 首帧播种三点窗口；避免启动时用两点均值代替中值导致毛刺泄漏。
            if not self._history:
                self._history.extend([arrays[:, :, :2].copy()] * self.config.median_window)
            else:
                self._history.append(arrays[:, :, :2].copy())
            processed[:, :, :2] = np.median(np.stack(self._history), axis=0)
            sample = np.linalg.norm(processed[:, :, :2].sum(axis=1), axis=1)
            old = self._filtered
            filtered = (
                sample
                if old is None
                else old + (-math.expm1(-dt / self.load_tau_s)) * (sample - old)
            )
            rate = 0.0 if old is None else float((filtered.sum() - old.sum()) / dt)
            self._filtered = filtered
            load = FilteredTangentialLoad(float(filtered[0]), float(filtered[1]), rate)
            observation = self.observer.update(*processed, time_s=sample_time_s)
        else:
            self._invalid += 1
            load = FilteredTangentialLoad(0.0, 0.0, 0.0)
            observation = TaxelRiskObservation(reason="invalid_sample")
        masks_changed = previous is not None and (
            previous.observation.left_valid_mask != observation.left_valid_mask
            or previous.observation.right_valid_mask != observation.right_valid_mask
        )
        if masks_changed or observation.contact_changed or not observation.valid:
            self._latest_event = None
            self._event_time_s = None
        if observation.event_id:
            self._latest_event = observation
            self._event_time_s = sample_time_s
        if (
            self._event_time_s is not None
            and sample_time_s - self._event_time_s > self.config.stale_after_s
        ):
            self._latest_event = None
            self._event_time_s = None
        self._events = max(self._events, observation.event_id)
        self._last = TactileState(
            sequence_id,
            sample_time_s,
            tuple(tuple(float(v) for v in row) for row in processed[0]),
            tuple(tuple(float(v) for v in row) for row in processed[1]),
            load,
            observation,
            valid,
            self._dropped,
            self._events,
            self._invalid,
            self._latest_event,
            self._event_time_s,
        )
        return self._last

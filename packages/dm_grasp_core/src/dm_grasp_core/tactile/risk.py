"""以设备时间窗生成逐触点纯力风险候选；不把候选解释为已证实滑移。"""

from collections import deque
from dataclasses import dataclass, fields, replace
import math
from numbers import Real

import numpy as np


@dataclass(frozen=True, slots=True)
class TaxelRiskConfig:
    """实验初值；4 N 切向和 15 N 法向量程须按实际传感器标定。

    质量值仅为有效触点覆盖率构成的工程分数，不是滑移概率。
    """

    window_s: float = 0.2
    confirmation_s: float = 0.04
    cooldown_s: float = 0.3
    max_gap_s: float = 0.1
    min_normal_n: float = 0.02
    contact_release_ratio: float = 0.7
    tangential_range_n: float = 4.0
    normal_range_n: float = 15.0
    min_load_rate_n_s: float = 0.05
    max_normal_rate_n_s: float = 0.3
    redistribution_threshold: float = 0.06
    release_ratio: float = 0.5

    def __post_init__(self) -> None:
        """严格拒绝非数值、非有限值及退化窗口和阈值。"""
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"{field.name} 必须为有限正数")
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{field.name} 必须为有限正数")
        if self.min_normal_n >= self.normal_range_n:
            raise ValueError("接触阈值必须小于法向量程")
        if (
            self.release_ratio >= 1
            or self.redistribution_threshold >= 1
            or self.contact_release_ratio >= 1
        ):
            raise ValueError("释放比例与剪切重分配阈值必须小于 1")


@dataclass(frozen=True, slots=True)
class TaxelRiskObservation:
    """一次观测的候选、事件及筛选诊断；候选只在新事件中给出。"""

    risk: float = 0.0
    event_id: int = 0
    left_candidate: float | None = None
    right_candidate: float | None = None
    left_quality: float = 0.0
    right_quality: float = 0.0
    contact_changed: bool = False
    valid: bool = False
    reason: str = "warming_up"
    left_valid_mask: tuple[bool, ...] = (False,) * 9
    right_valid_mask: tuple[bool, ...] = (False,) * 9


class TaxelRiskObserver:
    """用持续加载、局部重分配和法向变化排除门生成有界风险事件。"""

    def __init__(self, config: TaxelRiskConfig = TaxelRiskConfig()) -> None:
        """初始化纯计算状态，不访问时钟、设备或文件。"""
        self.config = config
        self._event_count = 0
        self.reset()

    def reset(self) -> None:
        """重建窗口，但保留递增事件编号，避免下游把新事件当成旧事件。"""
        self._history: deque[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = deque()
        self._last_time: float | None = None
        self._masks: tuple[tuple[bool, ...], tuple[bool, ...]] | None = None
        self._pending_s: float | None = None
        self._release_s: float | None = None
        self._last_event_s = -math.inf
        self._armed = True
        self._last = TaxelRiskObservation()

    def update(self, left, right, *, time_s: float) -> TaxelRiskObservation:
        """处理已规范为双侧各九点 Fx、Fy、Fn 的新数据。

        Args:
            left: 左侧形状为（9，3）的力数组，单位 N。
            right: 右侧同形状数组，压缩法向为正。
            time_s: 设备单调时间，单位 s；重复时间不推进状态。

        Returns:
            时间窗候选及诊断；坏帧使窗口失效，时间回退抛出异常。
        """
        if isinstance(time_s, bool) or not isinstance(time_s, Real) or not math.isfinite(time_s):
            raise ValueError("设备时间必须有限")
        if self._last_time is not None:
            if time_s < self._last_time:
                raise ValueError("设备时间不得回退")
            if time_s == self._last_time:
                return replace(
                    self._last,
                    event_id=0,
                    left_candidate=None,
                    right_candidate=None,
                    left_quality=0.0,
                    right_quality=0.0,
                    reason="duplicate_timestamp",
                )
        values = np.asarray((left, right), dtype=float)
        if values.shape != (2, 9, 3):
            raise ValueError("每侧必须为九个三轴触点")
        c = self.config
        within = np.isfinite(values).all(axis=2)
        within &= (np.abs(values[:, :, :2]) < c.tangential_range_n).all(axis=2)
        within &= np.abs(values[:, :, 2]) < c.normal_range_n
        previously_active = (
            np.zeros((2, 9), dtype=bool) if self._masks is None else np.asarray(self._masks)
        )
        threshold = np.where(
            previously_active, c.min_normal_n * c.contact_release_ratio, c.min_normal_n
        )
        active = within & (values[:, :, 2] >= threshold)
        masks = tuple(tuple(bool(v) for v in side) for side in active)
        changed = self._masks is not None and masks != self._masks
        gap = self._last_time is not None and time_s - self._last_time > c.max_gap_s
        valid = bool(within.all() and active.any(axis=1).all())
        self._last_time = time_s
        self._masks = masks
        result = TaxelRiskObservation(
            valid=valid,
            contact_changed=changed,
            left_valid_mask=masks[0],
            right_valid_mask=masks[1],
        )
        if changed or gap or not valid:
            self._history.clear()
            self._pending_s = self._release_s = None
        if not valid:
            self._last = replace(result, reason="invalid_taxels")
            return self._last
        normal = np.where(active, values[:, :, 2], 0).sum(axis=1)
        shear = np.where(active, np.linalg.norm(values[:, :, :2], axis=2), 0)
        load = np.linalg.norm(np.where(active[:, :, None], values[:, :, :2], 0).sum(axis=1), axis=1)
        share = shear / np.maximum(shear.sum(axis=1, keepdims=True), 1e-12)
        ratios = shear / np.maximum(values[:, :, 2], c.min_normal_n * c.contact_release_ratio)
        self._history.append((time_s, np.stack((load, normal)), share, ratios))
        while len(self._history) > 2 and self._history[1][0] <= time_s - c.window_s:
            self._history.popleft()
        first = self._history[0]
        elapsed = time_s - first[0]
        if elapsed < c.window_s or len(self._history) < 3:
            self._last = replace(result, reason="contact_changed" if changed else "warming_up")
            return self._last
        previous = self._history[-2]
        rate = (self._history[-1][1] - first[1]) / elapsed
        recent_rate = (self._history[-1][1] - previous[1]) / (time_s - previous[0])
        redistribution = np.abs(share - first[2]).sum(axis=1) / 2
        # 停止加载和主动增力即刻撤销证据，不能靠旧窗口延续确认。
        eligible = (recent_rate[0] >= c.min_load_rate_n_s) & (
            np.maximum(rate[1], recent_rate[1]) <= c.max_normal_rate_n_s
        )
        scores = np.where(
            eligible,
            np.minimum(rate[0] / c.min_load_rate_n_s, redistribution / c.redistribution_threshold),
            0,
        )
        score = max(0.0, float(scores.max()))
        if score < c.release_ratio:
            self._pending_s = None
            if self._release_s is None:
                self._release_s = time_s
            if (
                time_s - self._release_s >= c.confirmation_s
                and time_s - self._last_event_s >= c.cooldown_s
            ):
                self._armed = True
        else:
            self._release_s = None
        event = 0
        candidates: list[float | None] = [None, None]
        quality = np.zeros(2)
        if score >= 1 and self._armed:
            if self._pending_s is None:
                self._pending_s = time_s
            if (
                time_s - self._pending_s >= c.confirmation_s
                and time_s - self._last_event_s >= c.cooldown_s
            ):
                self._event_count += 1
                event = self._event_count
                self._last_event_s = time_s
                self._armed = False
                # 仅在剪切份额下降且局部力比不再增长的触点上形成摩擦候选。
                # 风险仍可更敏感；没有这条额外证据时只发风险，不更新摩擦。
                baseline = np.median(np.stack([frame[3] for frame in self._history][:-1]), axis=0)
                affected = active & (share < first[2]) & (ratios <= first[3])
                for side in range(2):
                    if scores[side] >= 1 and affected[side].any():
                        candidates[side] = float(np.median(baseline[side][affected[side]]))
                        quality[side] = active[side].sum() / 9
        elif score < 1:
            self._pending_s = None
        self._last = replace(
            result,
            risk=min(1.0, score),
            event_id=event,
            left_candidate=candidates[0],
            right_candidate=candidates[1],
            left_quality=float(quality[0]),
            right_quality=float(quality[1]),
            reason="risk_candidate" if event else "observing",
        )
        return self._last

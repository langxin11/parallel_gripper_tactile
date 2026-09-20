"""仅用逐 pillar 三轴力检测初始滑移并冻结摩擦系数候选。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .acquisition import TactileSnapshot


@dataclass(frozen=True, slots=True)
class PillarFrictionConfig:
    """逐 pillar 纯力摩擦估计的时间窗、判据与保守折减参数。"""

    window_duration_s: float = 0.2
    min_samples_per_half: int = 10
    min_ratio: float = 0.1
    arming_ratio_increase: float = 0.05
    saturation_ratio_increase: float = 0.005
    redistribution_share_drop: float = 0.05
    min_side_shear_increase_n: float = 0.03
    confirm_duration_s: float = 0.03
    estimate_quantile: float = 0.8
    safety_discount: float = 0.8
    min_friction_coefficient: float = 0.05
    max_friction_coefficient: float = 2.0

    def __post_init__(self) -> None:
        """拒绝非有限参数、无效窗口与矛盾摩擦范围。"""
        if (
            isinstance(self.min_samples_per_half, bool)
            or not isinstance(self.min_samples_per_half, int)
            or self.min_samples_per_half <= 0
        ):
            raise ValueError("min_samples_per_half 必须为正整数")
        values = (
            self.window_duration_s,
            self.min_ratio,
            self.arming_ratio_increase,
            self.saturation_ratio_increase,
            self.redistribution_share_drop,
            self.min_side_shear_increase_n,
            self.confirm_duration_s,
            self.estimate_quantile,
            self.safety_discount,
            self.min_friction_coefficient,
            self.max_friction_coefficient,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("逐 pillar 摩擦估计参数必须有限")
        if self.window_duration_s <= 0 or self.confirm_duration_s <= 0:
            raise ValueError("窗口与确认时间必须为正数")
        if self.min_ratio < 0 or self.arming_ratio_increase <= 0:
            raise ValueError("比值门槛必须非负，解锁增量必须为正数")
        if self.saturation_ratio_increase < 0 or self.redistribution_share_drop <= 0:
            raise ValueError("饱和增量必须非负，重分配门槛必须为正数")
        if self.min_side_shear_increase_n < 0:
            raise ValueError("分侧剪切增量门槛必须非负")
        if not 0 < self.estimate_quantile <= 1 or not 0 < self.safety_discount <= 1:
            raise ValueError("分位数与安全折减必须位于 (0, 1]")
        if self.min_friction_coefficient <= 0:
            raise ValueError("最小摩擦系数必须为正数")
        if self.max_friction_coefficient < self.min_friction_coefficient:
            raise ValueError("最大摩擦系数不得小于最小摩擦系数")


@dataclass(frozen=True, slots=True)
class PillarFrictionEstimate:
    """一次独立逐 pillar 初始滑移事件的冻结估计。"""

    side: int
    pillar_id: int
    raw_mu: float
    conservative_mu: float
    normal_force_n: float
    shear_force_n: float
    device_timestamp_us: int
    packet_counter: int


@dataclass(frozen=True, slots=True)
class _Frame:
    """一个分侧逐点力比窗口帧。"""

    time_s: float
    ratio: np.ndarray
    shear: np.ndarray
    active: np.ndarray


class _SideEstimator:
    """在一个传感器侧内独立维护各参考 pillar 的候选状态。"""

    def __init__(self, reference_mask: tuple[bool, ...], config: PillarFrictionConfig) -> None:
        """冻结参考接触集合并初始化时间窗。"""
        self.reference = np.asarray(reference_mask, dtype=bool)
        self.config = config
        self.history: deque[_Frame] = deque()
        self.armed = np.zeros(self.reference.shape, dtype=bool)
        self.candidate_duration_s = np.zeros(self.reference.shape, dtype=np.float64)
        self.candidate_raw_mu = np.full(self.reference.shape, np.nan)
        self.detected = np.zeros(self.reference.shape, dtype=bool)

    def update(
        self,
        *,
        time_s: float,
        forces: tuple[tuple[float, float, float], ...],
        contact_mask: tuple[bool, ...],
        dt_s: float,
    ) -> list[tuple[int, float, float, float, float]]:
        """更新一侧时间窗并返回新确认的逐点估计。"""
        values = np.asarray(forces, dtype=np.float64)
        mask = np.asarray(contact_mask, dtype=bool)
        if values.shape != (self.reference.size, 3) or mask.shape != self.reference.shape:
            raise ValueError("逐 pillar 力、接触集合与参考拓扑不一致")
        normal = np.maximum(values[:, 2], 0.0)
        shear = np.hypot(values[:, 0], values[:, 1])
        active = self.reference & mask & np.isfinite(normal) & np.isfinite(shear) & (normal > 0)
        lost = self.reference & ~active
        self.armed[lost] = False
        self.candidate_duration_s[lost] = 0.0
        self.candidate_raw_mu[lost] = np.nan
        ratio = np.full(normal.shape, np.nan)
        ratio[active] = shear[active] / normal[active]
        self.history.append(
            _Frame(
                time_s=time_s,
                ratio=ratio,
                shear=np.where(active, shear, 0.0),
                active=active,
            )
        )
        cutoff_s = time_s - self.config.window_duration_s
        while len(self.history) > 1 and self.history[1].time_s <= cutoff_s:
            self.history.popleft()
        if time_s - self.history[0].time_s + 1e-12 < self.config.window_duration_s:
            return []

        frames = tuple(self.history)
        midpoint_s = (frames[0].time_s + frames[-1].time_s) / 2
        early = tuple(frame for frame in frames if frame.time_s <= midpoint_s)
        recent = tuple(frame for frame in frames if frame.time_s > midpoint_s)
        if min(len(early), len(recent)) < self.config.min_samples_per_half:
            return []
        ratio_window = np.stack([frame.ratio for frame in frames])
        active_window = np.stack([frame.active for frame in frames])
        valid = active_window.all(axis=0) & np.isfinite(ratio_window).all(axis=0)
        safe_ratio = np.where(np.isfinite(ratio_window), ratio_window, 0.0)
        early_count = len(early)
        early_ratio = safe_ratio[:early_count].mean(axis=0)
        recent_ratio = safe_ratio[early_count:].mean(axis=0)
        ratio_increase = recent_ratio - early_ratio
        self.armed |= (
            valid
            & (recent_ratio >= self.config.min_ratio)
            & (ratio_increase >= self.config.arming_ratio_increase)
        )

        early_shear = np.stack([frame.shear for frame in early])
        recent_shear = np.stack([frame.shear for frame in recent])
        side_increase_n = recent_shear.sum(axis=1).mean() - early_shear.sum(axis=1).mean()
        all_shear = np.stack([frame.shear for frame in frames])
        shares = all_shear / np.maximum(all_shear.sum(axis=1, keepdims=True), 1e-12)
        share_drop = shares[:early_count].mean(axis=0) - shares[early_count:].mean(axis=0)
        candidate = (
            self.armed
            & ~self.detected
            & valid
            & (side_increase_n >= self.config.min_side_shear_increase_n)
            & (
                (ratio_increase <= self.config.saturation_ratio_increase)
                | (share_drop >= self.config.redistribution_share_drop)
            )
        )
        starting = candidate & (self.candidate_duration_s == 0.0)
        early_ratios = ratio_window[:early_count]
        for pillar_id in np.flatnonzero(starting):
            self.candidate_raw_mu[pillar_id] = float(
                np.quantile(early_ratios[:, pillar_id], self.config.estimate_quantile)
            )
        self.candidate_duration_s = np.where(
            candidate,
            self.candidate_duration_s + dt_s,
            0.0,
        )
        confirmed = (
            candidate
            & ~self.detected
            & (self.candidate_duration_s + 1e-12 >= self.config.confirm_duration_s)
        )
        self.detected |= confirmed
        events: list[tuple[int, float, float, float, float]] = []
        for pillar_id in np.flatnonzero(confirmed):
            raw_mu = float(self.candidate_raw_mu[pillar_id])
            conservative_mu = float(
                np.clip(
                    raw_mu * self.config.safety_discount,
                    self.config.min_friction_coefficient,
                    self.config.max_friction_coefficient,
                )
            )
            events.append(
                (
                    int(pillar_id),
                    raw_mu,
                    conservative_mu,
                    float(normal[pillar_id]),
                    float(shear[pillar_id]),
                )
            )
        return events


class PillarFrictionEstimator:
    """独立于原厂滑移状态的双侧逐 pillar 摩擦估计器。"""

    def __init__(self, config: PillarFrictionConfig | None = None) -> None:
        """保存配置，等待稳定接触时冻结参考集合。"""
        self.config = config or PillarFrictionConfig()
        self._sides: tuple[_SideEstimator, _SideEstimator] | None = None
        self._previous_timestamp_us: int | None = None

    def start(self, reference_mask: tuple[tuple[bool, ...], ...]) -> None:
        """以稳定接触时的双侧集合开始一次不可续接的估计周期。"""
        if len(reference_mask) != 2 or not all(any(side) for side in reference_mask):
            raise ValueError("逐 pillar 摩擦估计要求双侧均有参考触点")
        self._sides = (
            _SideEstimator(reference_mask[0], self.config),
            _SideEstimator(reference_mask[1], self.config),
        )
        self._previous_timestamp_us = None

    def stop(self) -> None:
        """停止消费新样本；已产生的事件由调用方保存。"""
        self._sides = None
        self._previous_timestamp_us = None

    def update(
        self,
        sample: TactileSnapshot,
        contact_mask: tuple[tuple[bool, ...], ...],
    ) -> list[PillarFrictionEstimate]:
        """只用当前逐点三轴力和设备时间更新自主估计。"""
        if self._sides is None:
            return []
        previous = self._previous_timestamp_us
        self._previous_timestamp_us = sample.timestamp_us
        if previous is None:
            return []
        dt_s = (sample.timestamp_us - previous) * 1e-6
        if not math.isfinite(dt_s) or dt_s <= 0:
            self.stop()
            raise ValueError("逐 pillar 摩擦估计要求严格递增的设备时间")
        forces = (sample.left_taxel_forces_n, sample.right_taxel_forces_n)
        estimates: list[PillarFrictionEstimate] = []
        for side, estimator in enumerate(self._sides):
            for pillar_id, raw_mu, conservative_mu, normal, shear in estimator.update(
                time_s=sample.timestamp_us * 1e-6,
                forces=forces[side],
                contact_mask=contact_mask[side],
                dt_s=dt_s,
            ):
                estimates.append(
                    PillarFrictionEstimate(
                        side=side,
                        pillar_id=pillar_id,
                        raw_mu=raw_mu,
                        conservative_mu=conservative_mu,
                        normal_force_n=normal,
                        shear_force_n=shear,
                        device_timestamp_us=sample.timestamp_us,
                        packet_counter=sample.packet_counter,
                    )
                )
        return estimates

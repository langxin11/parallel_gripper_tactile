"""控制周期的力—位置因果配对与刚度估计唯一所有者。

配对规则：一次控制周期使用命令发送前最新可用的触觉快照与电机反馈，
并记录两者时间差；估计只在新触觉观测上更新，重复快照不推进任何
确认窗口。设备时间回退按异常处理。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from dmgripper_hardware import MotorFeedback

from dm_grasp_core import (
    ContactStiffnessConfig,
    ContactStiffnessEstimator,
    CrankSliderKinematics,
    StiffnessSnapshot,
)

from .config import EstimationConfig
from .tactile import TactileSnapshot


def raw_axes(snapshot: TactileSnapshot) -> tuple[float, ...]:
    """取得完整双侧三轴力，缺失或非有限数据立即拒绝。"""
    values = tuple(
        getattr(snapshot, f"raw_{side}_f{axis}_n") for side in ("left", "right") for axis in "xyz"
    )
    if any(value is None or not math.isfinite(value) for value in values):
        raise RuntimeError("触觉三轴力缺失或包含非有限数值")
    return values  # type: ignore[return-value]


def tangential_forces(snapshot: TactileSnapshot) -> tuple[float, float]:
    """返回两侧各自切向合力模的和与左侧固定轴带符号切向力。"""
    axes = raw_axes(snapshot)
    magnitude = math.hypot(*axes[:2]) + math.hypot(*axes[3:5])
    signed = axes[1]
    return magnitude, signed


@dataclass(frozen=True, slots=True)
class PairedObservation:
    """一个控制周期消费的因果配对观测。

    力来自命令发送前的最新触觉快照，位置来自上一命令的反馈；两者
    时间差（``tactile_age_s``）写入 trace，不冒充硬件同步。

    Attributes:
        snapshot: 本周期使用的触觉快照。
        feedback: 本周期使用的电机反馈。
        is_new_tactile: 相对上一控制周期是否为新的触觉观测。
        tactile_age_s: 控制时刻与触觉接收时刻之差。
        tangential_force_n: 两侧切向合力模之和。
        signed_tangential_force_n: 左侧传感器固定局部轴的带符号切向力。
        closure_m: 由电机反馈换算的总闭合量。
    """

    snapshot: TactileSnapshot
    feedback: MotorFeedback
    is_new_tactile: bool
    tactile_age_s: float
    tangential_force_n: float
    signed_tangential_force_n: float
    closure_m: float

    @property
    def measured_force_n(self) -> float:
        """返回平均单侧滤波法向力。"""
        return (self.snapshot.left_force_n + self.snapshot.right_force_n) / 2


def pair_observation(
    *,
    snapshot: TactileSnapshot,
    feedback: MotorFeedback,
    previous: TactileSnapshot | None,
    now_s: float,
    kinematics: CrankSliderKinematics,
) -> PairedObservation:
    """把最新触觉快照与上一命令反馈配成一个控制周期观测。

    Args:
        snapshot: 最新触觉快照（已通过新鲜度检查）。
        feedback: 上一命令的电机反馈。
        previous: 上一控制周期使用的快照；``None`` 表示首包。
        now_s: 控制周期开始的单调时刻。
        kinematics: 曲柄滑块运动学。

    Returns:
        配对观测；``is_new_tactile`` 按接收时间判断。

    Raises:
        RuntimeError: 设备时间未递增（疑似重启）。
    """
    is_new = previous is None or snapshot.received_at_s != previous.received_at_s
    if is_new and previous is not None and snapshot.timestamp_us <= previous.timestamp_us:
        raise RuntimeError("触觉设备时间未递增或设备发生重启")
    magnitude, signed = tangential_forces(snapshot)
    return PairedObservation(
        snapshot=snapshot,
        feedback=feedback,
        is_new_tactile=is_new,
        tactile_age_s=now_s - snapshot.received_at_s,
        tangential_force_n=magnitude,
        signed_tangential_force_n=signed,
        closure_m=kinematics.closure(feedback.position_rad),
    )


class StiffnessDiagnostics:
    """刚度估计的唯一所有者：创建、按接触段重置与按新观测更新。

    默认只诊断：快照进入 trace 与终端，控制命令不因启用估计而变化，
    除非控制器配置显式启用前馈消费。估计使用外层滤波后的平均单侧
    法向力与配对的电机反馈位置。
    """

    def __init__(
        self,
        config: EstimationConfig,
        kinematics: CrankSliderKinematics,
    ) -> None:
        """创建估计器或保持禁用。"""
        self._enabled = config.enabled
        self._kinematics = kinematics
        self._estimator = (
            ContactStiffnessEstimator(
                ContactStiffnessConfig(
                    enabled=True,
                    initial_n_per_m=config.initial_n_per_m,
                    min_n_per_m=config.min_n_per_m,
                    max_n_per_m=config.max_n_per_m,
                    filter_alpha=config.filter_alpha,
                    min_delta_closure_m=config.min_delta_closure_m,
                    min_delta_force_n=config.min_delta_force_n,
                    method=config.method,
                    window_size=config.window_size,
                    min_samples=config.min_samples,
                ),
                kinematics,
            )
            if config.enabled
            else None
        )
        self._latest: StiffnessSnapshot | None = None

    @property
    def enabled(self) -> bool:
        """返回是否启用刚度估计。"""
        return self._enabled

    @property
    def latest(self) -> StiffnessSnapshot | None:
        """返回最近一次快照；禁用或尚未观测时为 ``None``。"""
        return self._latest

    def reset_contact(self, *, position_rad: float, normal_force_n: float) -> None:
        """接触边沿重置估计状态并登记新参考点。"""
        self._latest = None
        if self._estimator is None:
            return
        self._estimator.reset(position_rad=position_rad, normal_force_n=normal_force_n)

    def update(
        self,
        *,
        position_rad: float,
        normal_force_n: float,
        time_s: float,
        sample_id: int,
    ) -> StiffnessSnapshot | None:
        """以一对明确的位置与力更新估计；仅由新触觉观测驱动。

        Args:
            position_rad: 与力同一控制周期的电机反馈位置。
            normal_force_n: 同期的平均单侧滤波法向力。
            time_s: 控制线程单调时刻。
            sample_id: 观测编号（触觉包计数）。

        Returns:
            更新后的诊断快照；禁用时返回 ``None`` 且不更新任何状态。
        """
        if self._estimator is None:
            return None
        self._estimator.update(position_rad=position_rad, normal_force_n=normal_force_n)
        self._latest = self._estimator.snapshot(time_s=time_s, sample_id=sample_id)
        return self._latest

    def control_value(self) -> float | None:
        """返回可交给控制器消费的估计值；未有效估计时为 ``None``。"""
        if self._latest is None or not self._latest.valid:
            return None
        return self._latest.value_n_per_m

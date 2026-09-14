"""目标力来源：给定时间曲线与触觉动态增力。

目标来源拥有参考生成状态：曲线按任务时间采样并给出解析导数；动态
策略包装共享核 ``TactileDisturbancePolicy``，在 preload 学基线、进入
active 时冻结基线并启用增长，holding 阶段继续响应新的载荷增长。
任务与物体名称不进入任何公式。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from dm_grasp_core import (
    DisturbancePolicyParameters,
    ForceReferenceCurve,
    TactileDisturbancePolicy,
)
from dm_grasp_core.grasp.disturbance import DisturbanceCommand

from .config import AdaptiveReferenceConfig
from .observation import PairedObservation


@dataclass(frozen=True, slots=True)
class ForceTarget:
    """一个控制周期的目标力、导数与来源诊断。

    Attributes:
        force_n: 本周期受限后的目标力 (N)。
        rate_n_s: 目标力一阶导 (N/s)；``None`` 表示解析导数不可用。
        acceleration_n_s2: 目标力二阶导 (N/s²)；``None`` 表示不可用。
        source: 目标来源类型（curve 或 adaptive）。
        raw_force_n: 受限前的期望目标；仅动态策略提供。
        trigger_active: 动态策略本周期是否处于触发状态；曲线为 ``None``。
        measured_tangential_force_n: 策略滤波后的切向力；曲线为 ``None``。
        increase_count: 动态策略累计增力次数；曲线为 ``None``。
    """

    force_n: float
    rate_n_s: float | None
    acceleration_n_s2: float | None
    source: Literal["curve", "adaptive"]
    raw_force_n: float | None = None
    trigger_active: bool | None = None
    measured_tangential_force_n: float | None = None
    increase_count: int | None = None


class TargetSource:
    """目标力来源基类；子类拥有各自的参考生成状态。"""

    kind: Literal["curve", "adaptive"]

    @property
    def duration_s(self) -> float:
        """返回任务时长。"""
        raise NotImplementedError

    def preload_target(self, task_time_s: float) -> float:
        """返回 preload 阶段的稳定目标力。"""
        raise NotImplementedError

    def stabilize_preload(self) -> None:
        """进入 preload 时重置来源自身的稳定状态。"""
        raise NotImplementedError

    def observe(self, paired: PairedObservation, policy_dt_s: float) -> None:
        """消费一个新的触觉观测；无状态来源可以不做事。"""
        raise NotImplementedError

    def activate(self) -> None:
        """进入 active 阶段：曲线时间从此刻起算，动态策略冻结基线。"""
        raise NotImplementedError

    def active_reference(self, task_time_s: float) -> ForceTarget:
        """返回本控制周期的目标力与导数。"""
        raise NotImplementedError


class CurveTargetSource(TargetSource):
    """由共享核 ``ForceReferenceCurve`` 驱动的目标力来源。

    任务时间由运行时拥有（含重接近暂停），本类只做无状态采样，因此
    恢复接触后从暂停点继续剩余曲线，不重播已完成部分。
    """

    kind = "curve"

    def __init__(self, curve: ForceReferenceCurve) -> None:
        """保存冻结曲线。"""
        self._curve = curve

    @property
    def duration_s(self) -> float:
        """返回曲线持续时间。"""
        return self._curve.duration_s

    def preload_target(self, task_time_s: float) -> float:
        """返回暂停点（或起点）的曲线目标。"""
        return self._curve.target_at(task_time_s)

    def stabilize_preload(self) -> None:
        """曲线来源没有额外稳定状态。"""

    def observe(self, paired: PairedObservation, policy_dt_s: float) -> None:
        """曲线来源不消费触觉观测。"""

    def activate(self) -> None:
        """曲线来源无需在激活时改变状态。"""

    def active_reference(self, task_time_s: float) -> ForceTarget:
        """按任务时间采样曲线并携带解析导数。"""
        force_n, rate_n_s, acceleration_n_s2 = self._curve.sample_at(task_time_s)
        return ForceTarget(
            force_n=float(force_n),
            rate_n_s=float(rate_n_s),
            acceleration_n_s2=float(acceleration_n_s2),
            source="curve",
        )


class AdaptiveTargetSource(TargetSource):
    """由触觉切向反馈驱动、只增不减的动态增力来源。

    基线只在 preload 阶段学习；进入 active 后基线冻结，增长在 active
    与 holding 阶段持续响应。策略只在新触觉观测到来时更新，无新数据
    时保持上一参考，不补发已错过的动作。
    """

    kind = "adaptive"

    def __init__(
        self,
        config: AdaptiveReferenceConfig,
        *,
        max_target_force_n: float,
        control_rate_hz: float,
        contact_floor_n: float,
    ) -> None:
        """以真机专用较慢限幅创建共用增力策略。"""
        self._config = config
        self.policy = TactileDisturbancePolicy(
            DisturbancePolicyParameters(
                initial_force_n=config.initial_force_n,
                max_force_n=max_target_force_n,
                max_force_rate_n_s=config.max_force_rate_n_s,
                update_period_s=1.0 / control_rate_hz,
                filter_tau_s=config.filter_tau_s,
                shear_threshold_n=config.shear_threshold_n,
                shear_gain=config.shear_gain,
                contact_floor_n=contact_floor_n if contact_floor_n > 0.0 else 1e-6,
            )
        )
        self._activated = False
        self._command: DisturbanceCommand | None = None

    @property
    def duration_s(self) -> float:
        """返回显式任务时长。"""
        return self._config.duration_s

    @property
    def latest_command(self) -> DisturbanceCommand | None:
        """返回最近一次策略输出；尚无观测时为 ``None``。"""
        return self._command

    def preload_target(self, task_time_s: float) -> float:
        """返回初始抓力目标（基线阶段不随任务时间变化）。"""
        return self._config.initial_force_n

    def stabilize_preload(self) -> None:
        """进入 preload：基线重新开始学习，策略切回学习模式。"""
        self._activated = False

    def observe(self, paired: PairedObservation, policy_dt_s: float) -> None:
        """以新触觉观测推进策略；dt 为相邻两次观测的真实间隔。"""
        self._command = self.policy.update(
            left_normal_n=paired.snapshot.left_force_n,
            right_normal_n=paired.snapshot.right_force_n,
            tangential_force_n=paired.tangential_force_n,
            signed_tangential_force_n=paired.signed_tangential_force_n,
            closure_m=paired.closure_m,
            dt=policy_dt_s,
            enabled=self._activated,
        )

    def activate(self) -> None:
        """进入 active：冻结当前基线并启用增长。"""
        if self._command is None:
            raise RuntimeError("动态增力在激活前缺少基线观测")
        self._activated = True

    def active_reference(self, task_time_s: float) -> ForceTarget:
        """返回当前策略目标；无新观测时沿用最近一次输出。"""
        command = self._command
        if command is None:
            raise RuntimeError("动态增力尚未收到触觉观测")
        return ForceTarget(
            force_n=float(command.target_force_n),
            rate_n_s=float(command.target_force_rate_n_s),
            acceleration_n_s2=None,
            source="adaptive",
            raw_force_n=float(command.target_force_n),
            trigger_active=bool(command.trigger_active),
            measured_tangential_force_n=float(command.measured_tangential_force_n),
            increase_count=int(command.increase_count),
        )


def build_target_source(
    *,
    curve: ForceReferenceCurve | None,
    adaptive: AdaptiveReferenceConfig | None,
    max_target_force_n: float,
    control_rate_hz: float,
    contact_floor_n: float,
) -> TargetSource:
    """按配置构造唯一的目标力来源。

    Args:
        curve: 曲线模式共享核曲线。
        adaptive: 动态模式配置。
        max_target_force_n: 所有目标来源共用的目标力上限。
        control_rate_hz: 控制频率，用于策略动作周期。
        contact_floor_n: 有效接触的力下限。

    Returns:
        目标力来源实例。

    Raises:
        ValueError: 两种来源同时或都没有提供。
    """
    if (curve is None) == (adaptive is None):
        raise ValueError("目标来源必须且只能提供 curve 或 adaptive 之一")
    if curve is not None:
        return CurveTargetSource(curve)
    assert adaptive is not None
    return AdaptiveTargetSource(
        adaptive,
        max_target_force_n=max_target_force_n,
        control_rate_hz=control_rate_hz,
        contact_floor_n=contact_floor_n,
    )

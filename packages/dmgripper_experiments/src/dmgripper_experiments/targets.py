"""目标力来源：统一自适应触觉动态增力。

目标来源拥有参考生成状态：预载期仅观测，进入 active 后由切向承载
与在线摩擦之比产生目标；holding 阶段继续响应新的载荷增长。任务与
物体名称不进入任何公式。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal
import math


from dm_grasp_core.grasp.unified import (
    UnifiedAdaptiveCommand,
    UnifiedAdaptiveConfig,
    UnifiedAdaptivePolicy,
)

from .config import AdaptiveReferenceConfig
from .observation import PairedObservation


@dataclass(frozen=True, slots=True)
class ForceTarget:
    """一个控制周期的目标力、导数与来源诊断。

    Attributes:
        force_n: 本周期受限后的目标力 (N)。
        rate_n_s: 目标力一阶导 (N/s)；``None`` 表示解析导数不可用。
        acceleration_n_s2: 目标力二阶导 (N/s²)；``None`` 表示不可用。
        source: 目标来源类型，恒为 adaptive。
        raw_force_n: 受限前的期望目标。
        trigger_active: 本周期是否处于风险触发状态。
        measured_tangential_force_n: 策略滤波后的切向力。
        increase_count: 策略累计增力次数。
    """

    force_n: float
    rate_n_s: float | None
    acceleration_n_s2: float | None
    source: Literal["adaptive"]
    raw_force_n: float | None = None
    trigger_active: bool | None = None
    measured_tangential_force_n: float | None = None
    increase_count: int | None = None


class TargetSource:
    """目标力来源基类；子类拥有各自的参考生成状态。"""

    kind: Literal["adaptive"]

    def set_execution_limited(self, limited: bool) -> None:
        """接收上一个控制周期的执行约束；普通来源不消费。"""

    def trace_fields(self) -> dict[str, object]:
        """返回额外诊断；普通来源使用既有目标字段。"""
        return {}

    @property
    def failure_reason(self) -> str | None:
        """返回需要运行时处理的科学失败；普通来源无此状态。"""
        return None

    @property
    def duration_s(self) -> float | None:
        """返回任务时长；None 表示等待人工释放。"""
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
        """进入 active 阶段：冻结预载基线并启用增长。"""
        raise NotImplementedError

    def active_reference(self, task_time_s: float) -> ForceTarget:
        """返回本控制周期的目标力与导数。"""
        raise NotImplementedError


class UnifiedAdaptiveTargetSource(TargetSource):
    """以设备时间消费九点原始三轴力，不扣除预载时的真实载荷。"""

    kind = "adaptive"

    def __init__(
        self,
        config: AdaptiveReferenceConfig,
        unified: UnifiedAdaptiveConfig,
    ) -> None:
        """保存冻结配置与由 safety 派生的统一策略。"""
        self._config = config
        self.policy = UnifiedAdaptivePolicy(unified)
        self._activated = False
        self._execution_limited = False
        self._command: UnifiedAdaptiveCommand | None = None
        self._preload_force_n = config.initial_force_n
        self._contact_floor_set = False
        self._contact_closure_m: float | None = None

    def set_contact_closure(self, closure_m: float) -> None:
        """登记本次抓取首次双侧接触的总闭合量，不随触点集合变化重新归零。"""
        if isinstance(closure_m, bool) or not math.isfinite(closure_m):
            raise ValueError("首次接触闭合量必须有限")
        if self._contact_closure_m is None:
            self._contact_closure_m = closure_m

    def set_contact_floor(self, force_n: float) -> None:
        """预载目标只锁定一次，激活时再衔接调度器参考。"""
        if self._activated or self._contact_floor_set:
            raise RuntimeError("接触底力只能在预载中锁定一次")
        limits = self.policy.config.load
        if not limits.min_force_n <= force_n <= limits.max_force_n:
            raise ValueError("接触底力超出目标范围")
        self._preload_force_n = force_n
        self._contact_floor_set = True

    @property
    def duration_s(self) -> float | None:
        """返回显式任务时长。"""
        return self._config.duration_s

    def preload_target(self, task_time_s: float) -> float:
        """返回初始力或已锁定的刚度接触底力。"""
        return self._preload_force_n

    def stabilize_preload(self) -> None:
        """预载期间仅观测，不允许调度增力。"""
        self._activated = False

    def set_execution_limited(self, limited: bool) -> None:
        """登记上一步最终命令的执行约束。"""
        self._execution_limited = limited

    def observe(self, paired: PairedObservation, policy_dt_s: float) -> None:
        """设备时间戳决定窗口与积分；接收时间仅供运行时检查新鲜度。"""
        depth_inputs = (
            {
                "contact_depth_m": (
                    None
                    if self._contact_closure_m is None
                    else max(0.0, paired.closure_m - self._contact_closure_m)
                )
            }
            if self.policy.config.depth_friction_prior is not None
            else {}
        )
        self._command = self.policy.update(
            paired.snapshot.left_taxel_forces_n,
            paired.snapshot.right_taxel_forces_n,
            time_s=paired.snapshot.timestamp_us * 1e-6,
            measured_force_n=paired.measured_force_n,
            execution_limited=self._execution_limited,
            enabled=self._activated,
            **depth_inputs,
        )

    def activate(self) -> None:
        """完成预载后允许目标增长。"""
        if self._command is None:
            raise RuntimeError("统一策略在激活前缺少逐触点观测")
        if self._contact_floor_set:
            self.policy.establish_contact_floor(self._preload_force_n)
            self._command = self.policy.latest
        self._activated = True

    @property
    def failure_reason(self) -> str | None:
        """返回共享策略的明确失败原因。"""
        return self._command.failure_reason if self._command is not None else None

    def trace_fields(self) -> dict[str, object]:
        """提供与仿真及回放一致的逐触点诊断。"""
        return {
            **(self._command.trace_fields() if self._command is not None else {}),
            "depth_prior_contact_closure_m": self._contact_closure_m,
        }

    def active_reference(self, task_time_s: float) -> ForceTarget:
        """保持最近一次新样本产生的受限目标。"""
        if self._command is None:
            raise RuntimeError("统一策略尚未收到逐触点观测")
        load = self._command.load
        return ForceTarget(
            force_n=load.target_force_n,
            rate_n_s=load.target_force_rate_n_s,
            acceleration_n_s2=None,
            source="adaptive",
            raw_force_n=load.raw_target_force_n,
            trigger_active=self._command.risk > 0,
            measured_tangential_force_n=load.measured_tangential_force_n,
            increase_count=self._command.increase_count,
        )


def build_target_source(
    config: AdaptiveReferenceConfig,
    unified: UnifiedAdaptiveConfig,
) -> TargetSource:
    """构造统一自适应目标力来源。

    Args:
        config: 目标来源配置；必须声明统一策略。
        unified: 由 safety 单源边界派生的 core 统一策略配置。

    Returns:
        目标力来源实例。

    Raises:
        ValueError: 未声明统一策略。
    """
    if config.unified is None:
        raise ValueError("目标来源必须声明 reference.unified 统一策略")
    return UnifiedAdaptiveTargetSource(config, unified)

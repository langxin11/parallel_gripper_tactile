"""通用 DMgripper 真机抓取实验配置。

根配置按参数所有者拆分为十段：metadata／hardware／timing／lifecycle／
reference／controller／estimation／safety／output／terminal。任务与物体
名称只用于标识和输出目录，不进入目标生成或控制器选择逻辑。
"""

from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from types import UnionType
from typing import Any, Literal, get_args, get_origin, get_type_hints

import yaml
from dmgripper_hardware import DEFAULT_USB2CAN_PORT, make_dm4310p_gripper_config
from papillarray_hardware import DEFAULT_PAPILLARRAY_PORT

from dm_grasp_core import ForceInterpolation
from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig
from dm_grasp_core.tactile.multirate import TactileSamplingConfig

ControllerKind = Literal["admittance", "pid", "adrc"]
StiffnessConsumption = Literal["none", "feedforward"]
LostContactScope = Literal["any_side", "both_sides"]
LostContactAction = Literal["fault", "reapproach"]
FinishBehavior = Literal["hold", "return"]
TerminalMode = Literal["auto", "rich", "plain", "json"]
EstimationMethod = Literal["secant_ewma", "window_linear", "window_quadratic"]


def _finite_number(value: object, name: str, *, positive: bool = False) -> float:
    """验证并返回一个有限数值。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数值")
    number = float(value)
    if positive and number <= 0.0:
        raise ValueError(f"{name} 必须大于 0")
    return number


def sanitize_directory_component(name: str) -> str:
    """把显示名清理为可用的单个目录组件。

    拒绝空名与包含路径分隔符、``..`` 或控制字符的名称；其余非法字符
    替换为下划线。原始名称保留在 metadata 记录中，目录以清理后的名称
    加运行编号防碰撞。

    Args:
        name: 任务或物体的显示名。

    Returns:
        可安全用作单层目录组件的字符串。

    Raises:
        ValueError: 名称不是字符串、为空或包含路径穿越成分。
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("目录组件名称必须是非空字符串")
    if "/" in name or "\\" in name or ".." in name:
        raise ValueError(f"目录组件名称不得包含路径分隔符或 ..：{name!r}")
    if any(character < " " or character == "\x7f" for character in name):
        raise ValueError(f"目录组件名称不得包含控制字符：{name!r}")
    cleaned = re.sub(r"[^0-9A-Za-z._\-\u4e00-\u9fff]+", "_", name.strip())
    if not cleaned.strip("._"):
        raise ValueError(f"目录组件名称清理后为空：{name!r}")
    return cleaned


@dataclass(frozen=True, slots=True)
class MetadataConfig:
    """只用于标识的实验元数据。

    Attributes:
        task_name: 任务显示名；进入输出目录与记录。
        object_name: 物体显示名；进入输出目录与记录。
        description: 可选实验说明。
        tags: 可选标签集合，仅用于检索。
    """

    task_name: str = "grasp"
    object_name: str = "object"
    description: str = ""
    tags: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        """验证名称可用作目录组件。"""
        sanitize_directory_component(self.task_name)
        sanitize_directory_component(self.object_name)
        if not isinstance(self.description, str):
            raise ValueError("description 必须是字符串")
        for tag in self.tags:
            if not isinstance(tag, str) or not tag.strip():
                raise ValueError("tags 中的每一项必须是非空字符串")


@dataclass(frozen=True, slots=True)
class HardwareConfig:
    """设备端口、home 定义与反馈安全余量。

    命令工作范围仍由硬件部署固定为 ``[0, pi / 2]``；这里仅配置
    当前装配的 home 位置、判定容差和编码器反馈安全余量。
    """

    dm_port: str = DEFAULT_USB2CAN_PORT
    tactile_port: str = DEFAULT_PAPILLARRAY_PORT
    home_position_rad: float = 0.0
    home_tolerance_rad: float = 0.03
    feedback_position_margin_rad: float = 0.05

    def __post_init__(self) -> None:
        """验证端口为非空字符串。"""
        for name in ("dm_port", "tactile_port"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} 必须是非空字符串")
        _finite_number(self.home_position_rad, "hardware.home_position_rad")
        _finite_number(self.home_tolerance_rad, "hardware.home_tolerance_rad")
        _finite_number(
            self.feedback_position_margin_rad,
            "hardware.feedback_position_margin_rad",
        )
        if not 0.0 <= self.home_position_rad <= math.pi / 2.0:
            raise ValueError("hardware.home_position_rad 必须位于命令工作范围 [0, pi/2]")
        if self.home_tolerance_rad < 0.0:
            raise ValueError("hardware.home_tolerance_rad 不得为负")
        if self.feedback_position_margin_rad < 0.0:
            raise ValueError("hardware.feedback_position_margin_rad 不得为负")
        make_dm4310p_gripper_config(
            self.dm_port,
            feedback_position_margin_rad=self.feedback_position_margin_rad,
        )


@dataclass(frozen=True, slots=True)
class TimingConfig:
    """控制与采集时序参数。

    Attributes:
        control_rate_hz: 控制循环目标频率。
        max_control_gap_s: 相邻两个控制周期允许的最大真实间隔。
        tactile_startup_timeout_s: 首个触觉包的等待上限。
        tactile_timeout_s: 触觉快照的新鲜度上限，同时作为因果配对上限。
        tactile_bias_settle_s: 清零命令后等待传感器稳定的时长。
        tactile_cutoff_hz: 触觉外层一阶低通截止频率（导纳与记录使用）。
        tactile_filter_reset_gap_s: 外层滤波遇时间跳变时的重置阈值。
    """

    control_rate_hz: float = 100.0
    max_control_gap_s: float = 0.1
    tactile_startup_timeout_s: float = 5.0
    tactile_timeout_s: float = 0.2
    tactile_bias_settle_s: float = 2.0
    tactile_cutoff_hz: float = 10.0
    tactile_filter_reset_gap_s: float = 0.1

    def __post_init__(self) -> None:
        """验证时序参数与跨字段关系。"""
        for item in fields(self):
            _finite_number(getattr(self, item.name), f"timing.{item.name}")
        for name in (
            "control_rate_hz",
            "max_control_gap_s",
            "tactile_startup_timeout_s",
            "tactile_timeout_s",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"timing.{name} 必须大于 0")
        for name in ("tactile_bias_settle_s", "tactile_cutoff_hz", "tactile_filter_reset_gap_s"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"timing.{name} 不得为负")
        if self.max_control_gap_s <= 1.0 / self.control_rate_hz:
            raise ValueError("timing.max_control_gap_s 必须大于一个控制周期")
        if self.max_control_gap_s > self.tactile_timeout_s:
            raise ValueError("timing.max_control_gap_s 不得大于 timing.tactile_timeout_s")


@dataclass(frozen=True, slots=True)
class LifecycleConfig:
    """通用生命周期各阶段的判定与轨迹参数。

    Attributes:
        verify_zero_force: 使能前是否执行零力窗口验证。
        zero_force_threshold_n: 零力窗口滤波双侧 Fz 均值阈值。
        zero_force_stable_s: 零力窗口需要持续稳定的时长。
        zero_force_timeout_s: 零力验证总等待上限。
        contact_on_n: 双侧接触确认的每侧力阈值。
        contact_on_stable_s: 接触确认需要持续的时间。
        contact_off_n: 失接触判定的每侧力阈值。
        contact_off_stable_s: 失接触确认需要持续的时间。
        contact_transition_s: 接触后接近速度的过渡时长。
        lost_contact_scope: 失接触判据作用在哪一侧。
        lost_contact_action: 失接触后的处理动作。
        reapproach_max_attempts: 单次运行允许的重新接近次数上限。
        reapproach_timeout_s: 重接近路径的累计时间上限。
        preload_tolerance_n: 初始抓力稳定判定的低侧力误差容限。
        preload_stable_time_s: 初始抓力稳定需要持续的时长。
        preload_timeout_s: preload 等待上限。
        auto_start: ready 阶段是否跳过交互等待直接启动。
        on_finished: 任务计时完成后的行为（保持抓握或自动回位）。
        approach_closure_velocity_m_s: 接近段闭合速度上限。
        approach_closure_acceleration_m_s2: 接近段闭合加速度上限。
        approach_closure_jerk_m_s3: 接近段闭合加加速度上限。
        approach_endpoint_hold_s: 接近轨迹到达末端后的等待上限。
        return_closure_velocity_m_s: 回位段闭合速度上限。
        return_closure_acceleration_m_s2: 回位段闭合加速度上限。
        return_closure_jerk_m_s3: 回位段闭合加加速度上限。
        return_timeout_s: 回位总超时。
        return_settle_timeout_s: 回位轨迹结束后等待位置达标的余量。
        return_position_tolerance_rad: 回位达标的位置误差容限。
    """

    verify_zero_force: bool = True
    zero_force_threshold_n: float = 0.1
    zero_force_stable_s: float = 0.5
    zero_force_timeout_s: float = 5.0
    contact_on_n: float = 0.2
    contact_on_stable_s: float = 0.1
    contact_off_n: float = 0.1
    contact_off_stable_s: float = 0.15
    contact_transition_s: float = 0.15
    lost_contact_scope: LostContactScope = "any_side"
    lost_contact_action: LostContactAction = "fault"
    reapproach_max_attempts: int = 2
    reapproach_timeout_s: float = 20.0
    preload_tolerance_n: float = 0.15
    preload_stable_time_s: float = 2.0
    preload_timeout_s: float = 30.0
    auto_start: bool = False
    on_finished: FinishBehavior = "hold"
    approach_closure_velocity_m_s: float = 0.01
    approach_closure_acceleration_m_s2: float = 0.025
    approach_closure_jerk_m_s3: float = 0.1
    approach_endpoint_hold_s: float = 0.5
    return_closure_velocity_m_s: float = 0.012
    return_closure_acceleration_m_s2: float = 0.025
    return_closure_jerk_m_s3: float = 0.1
    return_timeout_s: float = 5.0
    return_settle_timeout_s: float = 2.0
    return_position_tolerance_rad: float = 0.02

    def __post_init__(self) -> None:
        """验证生命周期参数与跨字段关系。"""
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name in {"verify_zero_force", "auto_start"}:
                if not isinstance(value, bool):
                    raise ValueError(f"lifecycle.{item.name} 必须是布尔值")
                continue
            if item.name in {"lost_contact_scope", "lost_contact_action", "on_finished"}:
                continue
            if isinstance(value, int) and not isinstance(value, bool):
                if value <= 0:
                    raise ValueError(f"lifecycle.{item.name} 必须为正整数")
                continue
            _finite_number(value, f"lifecycle.{item.name}")
        if self.lost_contact_scope not in get_args(LostContactScope):
            raise ValueError("lifecycle.lost_contact_scope 必须是 any_side 或 both_sides")
        if self.lost_contact_action not in get_args(LostContactAction):
            raise ValueError("lifecycle.lost_contact_action 必须是 fault 或 reapproach")
        if self.on_finished not in get_args(FinishBehavior):
            raise ValueError("lifecycle.on_finished 必须是 hold 或 return")
        for name in (
            "zero_force_stable_s",
            "contact_on_stable_s",
            "contact_off_stable_s",
            "contact_transition_s",
            "preload_stable_time_s",
            "preload_timeout_s",
            "reapproach_timeout_s",
            "approach_endpoint_hold_s",
            "return_timeout_s",
            "return_settle_timeout_s",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"lifecycle.{name} 必须大于 0")
        for name in (
            "preload_tolerance_n",
            "return_position_tolerance_rad",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"lifecycle.{name} 不得为负")
        if not 0.0 <= self.contact_off_n < self.contact_on_n:
            raise ValueError("lifecycle.contact_off_n 必须非负且小于 contact_on_n")
        if not 0.0 < self.zero_force_threshold_n < self.contact_on_n:
            raise ValueError("lifecycle.zero_force_threshold_n 必须位于 0 与 contact_on_n 之间")
        if self.zero_force_stable_s >= self.zero_force_timeout_s:
            raise ValueError("lifecycle.zero_force_stable_s 必须小于 zero_force_timeout_s")


@dataclass(frozen=True, slots=True)
class WaypointConfig:
    """目标力曲线上的一个时间—力节点。

    Attributes:
        t_s: 相对曲线起点的时刻（s）。
        force_n: 目标力（N）。
    """

    t_s: float
    force_n: float

    def __post_init__(self) -> None:
        """验证节点为非负有限数值。"""
        _finite_number(self.t_s, "waypoint.t_s")
        _finite_number(self.force_n, "waypoint.force_n")
        if self.t_s < 0.0:
            raise ValueError("waypoint.t_s 不得为负")
        if self.force_n < 0.0:
            raise ValueError("waypoint.force_n 不得为负")


@dataclass(frozen=True, slots=True)
class CurveReferenceConfig:
    """给定时间曲线的目标力来源。

    Attributes:
        interpolation: waypoint 间插值方式。
        waypoints: 有序节点序列；首点必须是零时刻。
    """

    interpolation: ForceInterpolation = "smoothstep"
    waypoints: tuple[WaypointConfig, ...] = ()

    def __post_init__(self) -> None:
        """验证曲线结构。"""
        if self.interpolation not in get_args(ForceInterpolation):
            raise ValueError("reference.curve.interpolation 必须是 hold、linear 或 smoothstep")
        if len(self.waypoints) < 2:
            raise ValueError("reference.curve.waypoints 至少需要两个节点")
        previous = -math.inf
        for waypoint in self.waypoints:
            if waypoint.t_s <= previous:
                raise ValueError("reference.curve.waypoints 时间必须严格递增")
            previous = waypoint.t_s
        if self.waypoints[0].t_s != 0.0:
            raise ValueError("reference.curve.waypoints 首点必须是零时刻")
        if self.waypoints[-1].t_s <= 0.0:
            raise ValueError("reference.curve.waypoints 末点时间必须大于 0")

    @property
    def duration_s(self) -> float:
        """返回曲线持续时间（末节点时刻）。"""
        return self.waypoints[-1].t_s

    @property
    def initial_force_n(self) -> float:
        """返回曲线首值（preload 目标）。"""
        return self.waypoints[0].force_n

    @property
    def has_descending_segment(self) -> bool:
        """返回曲线是否包含力下降段。"""
        return any(
            end.force_n < start.force_n for start, end in zip(self.waypoints, self.waypoints[1:])
        )

    @property
    def min_force_n(self) -> float:
        """返回曲线上的最小目标力。"""
        return min(waypoint.force_n for waypoint in self.waypoints)


@dataclass(frozen=True, slots=True)
class AdaptiveReferenceConfig:
    """根据触觉反馈动态增力的目标来源。

    默认复用 ``TactileDisturbancePolicy`` 的 ``shear_increase`` 与
    ``dynamic_step``；显式提供 unified 时切换为九点统一策略。

    Attributes:
        initial_force_n: 初始抓力（preload 目标与策略基线初值）。
        duration_s: 任务计时长度。
        max_force_rate_n_s: 目标力变化率上限。
        filter_tau_s: 切向力滤波时间常数。
        shear_threshold_n: 触发阈值。
        shear_gain: 切向增量到目标包络的增益。
        unified: 可选统一策略；省略时保持既有切向增力行为。
        risk_validation_passed: 局部风险真机验收是否通过；仅开放风险闭环时要求。
        friction_validation_passed: 摩擦更新真机验收是否通过；仅开放摩擦闭环时要求。
        experimental_closed_loop: 显式授权未验收的风险／摩擦闭环实验，不等同于验收通过。
    """

    initial_force_n: float = 0.5
    duration_s: float = 10.0
    max_force_rate_n_s: float = 0.5
    filter_tau_s: float = 0.05
    shear_threshold_n: float = 0.06
    shear_gain: float = 1.0
    unified: UnifiedAdaptiveConfig | None = None
    tactile_sampling: TactileSamplingConfig | None = None
    risk_validation_passed: bool = False
    friction_validation_passed: bool = False
    experimental_closed_loop: bool = False

    def __post_init__(self) -> None:
        """验证动态增力参数。"""
        for item in fields(self):
            if item.name in {
                "unified",
                "tactile_sampling",
                "risk_validation_passed",
                "friction_validation_passed",
                "experimental_closed_loop",
            }:
                continue
            _finite_number(
                getattr(self, item.name), f"reference.adaptive.{item.name}", positive=True
            )
        for value in (
            self.risk_validation_passed,
            self.friction_validation_passed,
            self.experimental_closed_loop,
        ):
            if not isinstance(value, bool):
                raise ValueError("风险与摩擦验收门禁必须为布尔值")
        if self.tactile_sampling is not None:
            if not isinstance(self.tactile_sampling, TactileSamplingConfig) or self.unified is None:
                raise ValueError("多速率预处理要求统一策略和有效采样配置")
            if self.tactile_sampling.period_s != 0.002:
                raise ValueError("真机采集固定为 500 Hz")
        if self.unified is not None:
            if not isinstance(self.unified, UnifiedAdaptiveConfig):
                raise ValueError("unified 必须是 UnifiedAdaptiveConfig")
            if self.unified.load.min_force_n != self.initial_force_n:
                raise ValueError("unified 最低力必须等于 initial_force_n")
            if self.unified.load.max_force_rate_n_s != self.max_force_rate_n_s:
                raise ValueError("unified 与 adaptive 的最大增力速率必须一致")
            if self.unified.risk_enabled and not (
                self.risk_validation_passed or self.experimental_closed_loop
            ):
                raise ValueError("风险闭环需要 risk_validation_passed 验收门禁")
            if self.unified.friction_update_enabled and not (
                self.friction_validation_passed or self.experimental_closed_loop
            ):
                raise ValueError("摩擦闭环需要 friction_validation_passed 验收门禁")


@dataclass(frozen=True, slots=True)
class ReferenceConfig:
    """目标力来源判别；初始力与任务时长的唯一所有者。

    Attributes:
        curve: 给定时间曲线配置；与 ``adaptive`` 互斥。
        adaptive: 动态增力配置；与 ``curve`` 互斥。
    """

    curve: CurveReferenceConfig | None = None
    adaptive: AdaptiveReferenceConfig | None = None

    def __post_init__(self) -> None:
        """要求恰好选择一种目标来源。"""
        if (self.curve is None) == (self.adaptive is None):
            raise ValueError("reference 必须且只能选择 curve 或 adaptive 之一")

    @property
    def kind(self) -> Literal["curve", "adaptive"]:
        """返回目标来源类型。"""
        return "curve" if self.curve is not None else "adaptive"

    @property
    def duration_s(self) -> float:
        """返回任务时长（曲线取末节点，动态取显式 duration_s）。"""
        if self.curve is not None:
            return self.curve.duration_s
        assert self.adaptive is not None
        return self.adaptive.duration_s

    @property
    def initial_force_n(self) -> float:
        """返回初始目标力（曲线首值或动态初始抓力）。"""
        if self.curve is not None:
            return self.curve.initial_force_n
        assert self.adaptive is not None
        return self.adaptive.initial_force_n


@dataclass(frozen=True, slots=True)
class AdmittanceConfig:
    """二阶导纳外环参数；死区与单向闭合是导纳专属行为。"""

    mass_kg: float = 0.02
    damping_ns_m: float = 0.2
    stiffness_n_m: float = 1.0
    force_deadband_n: float = 0.1
    prevent_unloading: bool = True
    feedforward_ratio: float = 0.0

    def __post_init__(self) -> None:
        """验证导纳参数。"""
        for name in ("mass_kg", "damping_ns_m", "stiffness_n_m", "force_deadband_n"):
            _finite_number(getattr(self, name), f"controller.admittance.{name}")
        if self.mass_kg <= 0.0:
            raise ValueError("controller.admittance.mass_kg 必须大于 0")
        if self.damping_ns_m < 0.0 or self.stiffness_n_m < 0.0:
            raise ValueError("controller.admittance.damping_ns_m 与 stiffness_n_m 不得为负")
        if self.force_deadband_n < 0.0:
            raise ValueError("controller.admittance.force_deadband_n 不得为负")
        if not isinstance(self.prevent_unloading, bool):
            raise ValueError("controller.admittance.prevent_unloading 必须是布尔值")
        _finite_number(self.feedforward_ratio, "controller.admittance.feedforward_ratio")
        if not 0 <= self.feedforward_ratio <= 1:
            raise ValueError("导纳力矩前馈比例必须位于 0 与 1 之间")


@dataclass(frozen=True, slots=True)
class PIDConfig:
    """PID 力跟踪器参数。"""

    kp: float = 0.016
    ki: float = 0.2
    kd: float = 0.0
    max_position_adjustment_rad: float = 0.15

    def __post_init__(self) -> None:
        """验证 PID 参数。"""
        for name in ("kp", "ki", "kd"):
            if _finite_number(getattr(self, name), f"controller.pid.{name}") < 0.0:
                raise ValueError(f"controller.pid.{name} 不得为负")
        _finite_number(
            self.max_position_adjustment_rad,
            "controller.pid.max_position_adjustment_rad",
            positive=True,
        )


@dataclass(frozen=True, slots=True)
class AdrcConfig:
    """一阶位置型 LADRC 力跟踪器参数。"""

    b0_n_per_m: float = 1000.0
    controller_bandwidth_rad_s: float = 5.0
    observer_bandwidth_rad_s: float = 20.0
    max_closing_velocity_m_s: float = 0.002

    def __post_init__(self) -> None:
        """验证 LADRC 参数。"""
        for item in fields(self):
            _finite_number(getattr(self, item.name), f"controller.adrc.{item.name}", positive=True)


@dataclass(frozen=True, slots=True)
class ControllerConfig:
    """控制器判别、算法参数与 MIT 可调参数。

    ``pid``／``adrc`` 路径接收未滤波原始力，由共享核内部做一阶低通；
    ``admittance`` 路径消费外层已滤波的触觉力。导纳专属开关不进入
    PID／LADRC 的同名参数。

    Attributes:
        kind: 控制器类型。
        admittance: 导纳参数。
        pid: PID 参数。
        adrc: 一阶 LADRC 参数。
        mit_kp: 跟踪段 MIT 位置增益。
        mit_kd: 跟踪段 MIT 阻尼增益。
        velocity_limit_rad_s: 关节速度限幅。
        torque_limit_nm: 跟踪段力矩限幅。
        return_mit_kp: 回位段 MIT 位置增益。
        return_mit_kd: 回位段 MIT 阻尼增益。
        return_torque_limit_nm: 回位段力矩限幅。
        stiffness_consumption: 是否消费刚度估计前馈；默认只诊断。
    """

    kind: ControllerKind = "admittance"
    admittance: AdmittanceConfig = field(default_factory=AdmittanceConfig)
    pid: PIDConfig = field(default_factory=PIDConfig)
    adrc: AdrcConfig = field(default_factory=AdrcConfig)
    mit_kp: float = 2.0
    mit_kd: float = 0.5
    velocity_limit_rad_s: float = 0.3
    torque_limit_nm: float = 4.0
    return_mit_kp: float = 10.0
    return_mit_kd: float = 0.5
    return_torque_limit_nm: float = 2.0
    stiffness_consumption: StiffnessConsumption = "none"

    def __post_init__(self) -> None:
        """验证控制器选择与 MIT 参数范围。"""
        if self.kind not in get_args(ControllerKind):
            raise ValueError("controller.kind 必须是 admittance、pid 或 adrc")
        if self.stiffness_consumption not in get_args(StiffnessConsumption):
            raise ValueError("controller.stiffness_consumption 必须是 none 或 feedforward")
        for name in (
            "mit_kp",
            "mit_kd",
            "velocity_limit_rad_s",
            "torque_limit_nm",
            "return_mit_kp",
            "return_mit_kd",
            "return_torque_limit_nm",
        ):
            _finite_number(getattr(self, name), f"controller.{name}", positive=True)
        deployment = make_dm4310p_gripper_config("config://validation")
        limits = deployment.motor_limits
        upper_bounds = {
            "mit_kp": 500.0,
            "mit_kd": 5.0,
            "velocity_limit_rad_s": min(
                abs(limits.velocity_min_rad_s),
                limits.velocity_max_rad_s,
            ),
            "torque_limit_nm": min(abs(limits.torque_min_nm), limits.torque_max_nm),
            "return_mit_kp": 500.0,
            "return_mit_kd": 5.0,
            "return_torque_limit_nm": min(
                abs(limits.torque_min_nm),
                limits.torque_max_nm,
            ),
        }
        for name, upper_bound in upper_bounds.items():
            if getattr(self, name) > upper_bound:
                raise ValueError(f"controller.{name} 不得超过 DM 协议上限 {upper_bound}")
        if not isinstance(self.admittance, AdmittanceConfig):
            raise ValueError("controller.admittance 必须是 AdmittanceConfig")
        if not isinstance(self.pid, PIDConfig):
            raise ValueError("controller.pid 必须是 PIDConfig")
        if not isinstance(self.adrc, AdrcConfig):
            raise ValueError("controller.adrc 必须是 AdrcConfig")


@dataclass(frozen=True, slots=True)
class EstimationConfig:
    """等效接触刚度估计配置。

    估计量为平均单侧法向力相对于总闭合行程的局部等效刚度（N/m）。
    默认值沿用仿真验证过的起点，不是真机辨识值。默认只诊断，
    不因启用估计而改变控制律。

    Attributes:
        enabled: 是否更新刚度估计并记录诊断。
        method: 估计方法。
        initial_n_per_m: 初始等效刚度（N/m）。
        min_n_per_m: 单次样本刚度下限。
        max_n_per_m: 单次样本刚度上限。
        filter_alpha: 估计值的一阶平滑系数。
        min_delta_closure_m: 有效样本的最小闭合行程增量。
        min_delta_force_n: 有效样本的最小力增量。
        window_size: 滑动窗口容量。
        min_samples: 触发拟合的最少样本数。
    """

    enabled: bool = True
    method: EstimationMethod = "window_linear"
    initial_n_per_m: float = 3000.0
    min_n_per_m: float = 250.0
    max_n_per_m: float = 25000.0
    filter_alpha: float = 0.15
    min_delta_closure_m: float = 0.00005
    min_delta_force_n: float = 0.025
    window_size: int = 25
    min_samples: int = 8

    def __post_init__(self) -> None:
        """验证估计参数。"""
        if self.method not in get_args(EstimationMethod):
            raise ValueError(
                "estimation.method 必须是 secant_ewma、window_linear 或 window_quadratic"
            )
        if not isinstance(self.enabled, bool):
            raise ValueError("estimation.enabled 必须是布尔值")
        _finite_number(self.initial_n_per_m, "estimation.initial_n_per_m", positive=True)
        _finite_number(self.min_n_per_m, "estimation.min_n_per_m", positive=True)
        _finite_number(self.max_n_per_m, "estimation.max_n_per_m", positive=True)
        _finite_number(self.filter_alpha, "estimation.filter_alpha", positive=True)
        _finite_number(self.min_delta_closure_m, "estimation.min_delta_closure_m", positive=True)
        _finite_number(self.min_delta_force_n, "estimation.min_delta_force_n", positive=True)
        if isinstance(self.window_size, bool) or not isinstance(self.window_size, int):
            raise ValueError("estimation.window_size 必须是整数")
        if isinstance(self.min_samples, bool) or not isinstance(self.min_samples, int):
            raise ValueError("estimation.min_samples 必须是整数")
        if self.window_size <= 0 or self.min_samples <= 0:
            raise ValueError("estimation.window_size 与 min_samples 必须为正")
        if self.min_samples > self.window_size:
            raise ValueError("estimation.min_samples 不得大于 window_size")
        if self.min_n_per_m >= self.max_n_per_m:
            raise ValueError("estimation.min_n_per_m 必须小于 max_n_per_m")
        if not self.min_n_per_m <= self.initial_n_per_m <= self.max_n_per_m:
            raise ValueError("estimation.initial_n_per_m 必须位于刚度界限内")
        if self.filter_alpha > 1.0:
            raise ValueError("estimation.filter_alpha 不得大于 1")
        degree = {"window_linear": 1, "window_quadratic": 2}.get(self.method)
        if degree is not None and self.min_samples < degree + 1:
            raise ValueError("estimation.min_samples 不足以支撑所选拟合方法")


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    """原始力保护、目标上限与接近前馈边界。"""

    max_target_force_n: float = 1.5
    force_ceiling_n: float = 2.0
    approach_feedforward_force_n: float = 2.0
    approach_feedforward_ratio: float = 0.5

    def __post_init__(self) -> None:
        """验证保护参数。"""
        for name in (
            "max_target_force_n",
            "force_ceiling_n",
            "approach_feedforward_force_n",
        ):
            _finite_number(getattr(self, name), f"safety.{name}", positive=True)
        _finite_number(self.approach_feedforward_ratio, "safety.approach_feedforward_ratio")
        if not 0.0 <= self.approach_feedforward_ratio <= 1.0:
            raise ValueError("safety.approach_feedforward_ratio 必须位于 0 与 1 之间")


@dataclass(frozen=True, slots=True)
class OutputConfig:
    """输出目录与出图设置。"""

    root: str = "outputs/real"
    plots: bool = True

    def __post_init__(self) -> None:
        """验证输出根目录。"""
        if not isinstance(self.root, str) or not self.root.strip():
            raise ValueError("output.root 必须是非空字符串")
        if not isinstance(self.plots, bool):
            raise ValueError("output.plots 必须是布尔值")


@dataclass(frozen=True, slots=True)
class TerminalConfig:
    """终端展示模式与刷新频率；UI 不参与控制时钟。"""

    mode: TerminalMode = "auto"
    refresh_hz: float = 5.0

    def __post_init__(self) -> None:
        """验证终端配置。"""
        if self.mode not in get_args(TerminalMode):
            raise ValueError("terminal.mode 必须是 auto、rich、plain 或 json")
        _finite_number(self.refresh_hz, "terminal.refresh_hz", positive=True)


@dataclass(frozen=True, slots=True)
class ExperimentConfig:
    """一次通用抓取实验的完整冻结配置。"""

    metadata: MetadataConfig = field(default_factory=MetadataConfig)
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    lifecycle: LifecycleConfig = field(default_factory=LifecycleConfig)
    reference: ReferenceConfig = field(
        default_factory=lambda: ReferenceConfig(
            curve=CurveReferenceConfig(
                waypoints=(
                    WaypointConfig(t_s=0.0, force_n=0.5),
                    WaypointConfig(t_s=10.0, force_n=0.5),
                )
            )
        )
    )
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    estimation: EstimationConfig = field(default_factory=EstimationConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    terminal: TerminalConfig = field(default_factory=TerminalConfig)

    def __post_init__(self) -> None:
        """验证段类型与跨字段安全、控制相容性约束。"""
        expected = {
            "metadata": MetadataConfig,
            "hardware": HardwareConfig,
            "timing": TimingConfig,
            "lifecycle": LifecycleConfig,
            "reference": ReferenceConfig,
            "controller": ControllerConfig,
            "estimation": EstimationConfig,
            "safety": SafetyConfig,
            "output": OutputConfig,
            "terminal": TerminalConfig,
        }
        for name, cls in expected.items():
            if not isinstance(getattr(self, name), cls):
                raise ValueError(f"{name} 必须是 {cls.__name__}")
        self._validate_reference_bounds()
        self._validate_controller_compatibility()

    def _validate_reference_bounds(self) -> None:
        """校验目标力与阈值、上限的相对关系。"""
        lifecycle = self.lifecycle
        safety = self.safety
        initial = self.reference.initial_force_n
        if initial > safety.max_target_force_n:
            raise ValueError("初始目标力不得大于 safety.max_target_force_n")
        if initial < lifecycle.contact_on_n:
            raise ValueError("初始目标力不得小于 lifecycle.contact_on_n")
        if safety.max_target_force_n >= safety.force_ceiling_n:
            raise ValueError("safety.max_target_force_n 必须严格小于 force_ceiling_n")
        if self.unified_adaptive_enabled:
            unified = self.reference.adaptive.unified
            if unified.load.max_force_n != safety.max_target_force_n:
                raise ValueError("unified 最大目标力必须等于 safety.max_target_force_n")
            sampling = self.reference.adaptive.tactile_sampling
            if sampling is not None and (
                self.timing.control_rate_hz != 250
                or sampling.stale_after_s >= self.timing.tactile_timeout_s
            ):
                raise ValueError("多速率要求 250 Hz 控制，软 stale 阈值小于硬超时")
        if self.reference.curve is not None:
            curve = self.reference.curve
            peak = max(waypoint.force_n for waypoint in curve.waypoints)
            if peak > safety.max_target_force_n:
                raise ValueError("曲线峰值不得大于 safety.max_target_force_n")
            if curve.min_force_n <= lifecycle.contact_off_n:
                raise ValueError("曲线最小力不得低于失接触阈值，以免隐式穿越释放边界")
            if curve.has_descending_segment and self.controller.kind == "admittance":
                if self.controller.admittance.prevent_unloading:
                    raise ValueError(
                        "导纳 prevent_unloading 与曲线下降段不相容；"
                        "请改用 hold 语义外的其他插值、去掉下降段或关闭单向闭合"
                    )

    def _validate_controller_compatibility(self) -> None:
        """校验控制器、目标来源与失接触处理的组合约束。"""
        adaptive = self.reference.adaptive is not None
        if self.unified_adaptive_enabled and self.controller.kind != "admittance":
            raise ValueError("统一自适应模式只支持导纳控制器")
        if adaptive and self.lifecycle.lost_contact_action == "reapproach":
            raise ValueError("动态增力模式只接受失接触 fault；重接近的基线语义尚未定义")
        if self.controller.stiffness_consumption == "feedforward":
            if self.controller.kind == "admittance":
                raise ValueError("导纳路径不消费刚度前馈；请将 stiffness_consumption 设为 none")
            if not self.estimation.enabled:
                raise ValueError("刚度前馈消费要求 estimation.enabled 为真")

    @property
    def unified_adaptive_enabled(self) -> bool:
        """返回是否选择了逐触点统一自适应模式。"""
        return self.reference.adaptive is not None and self.reference.adaptive.unified is not None


_ROOT_SCHEMA: dict[str, type[Any]] = {
    "metadata": MetadataConfig,
    "hardware": HardwareConfig,
    "timing": TimingConfig,
    "lifecycle": LifecycleConfig,
    "reference": ReferenceConfig,
    "controller": ControllerConfig,
    "estimation": EstimationConfig,
    "safety": SafetyConfig,
    "output": OutputConfig,
    "terminal": TerminalConfig,
}

_STRING_ENUM_FIELDS: dict[tuple[type, str], tuple[str, ...]] = {
    (CurveReferenceConfig, "interpolation"): get_args(ForceInterpolation),
    (LifecycleConfig, "lost_contact_scope"): get_args(LostContactScope),
    (LifecycleConfig, "lost_contact_action"): get_args(LostContactAction),
    (LifecycleConfig, "on_finished"): get_args(FinishBehavior),
    (ControllerConfig, "kind"): get_args(ControllerKind),
    (ControllerConfig, "stiffness_consumption"): get_args(StiffnessConsumption),
    (EstimationConfig, "method"): get_args(EstimationMethod),
    (TerminalConfig, "mode"): get_args(TerminalMode),
}


def _strict_mapping(value: object, name: str) -> dict[str, Any]:
    """取得字符串键的 YAML 映射。"""
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{name} 必须是字符串键的映射")
    return value


def _decode_dataclass(data: object, cls: type[Any], name: str) -> Any:
    """按 dataclass 字段严格解码 YAML 映射。

    支持 bool、有限数值、枚举字符串、嵌套 dataclass 与 ``waypoints``
    节点序列；未知字段与非匹配类型一律报错，不做静默默认。
    """
    mapping = _strict_mapping(data, name)
    permitted = {item.name for item in fields(cls)}
    unknown = set(mapping) - permitted
    if unknown:
        raise ValueError(f"{name} 包含未知字段：{', '.join(sorted(unknown))}")
    type_hints = get_type_hints(cls)
    kwargs: dict[str, Any] = {}
    for item in fields(cls):
        if item.name not in mapping:
            continue
        value = mapping[item.name]
        field_name = f"{name}.{item.name}"
        enum_values = _STRING_ENUM_FIELDS.get((cls, item.name))
        if enum_values is not None:
            if not isinstance(value, str) or value not in enum_values:
                raise ValueError(f"{field_name} 必须是 {', '.join(enum_values)} 之一")
            kwargs[item.name] = value
        elif isinstance(item.default, bool):
            if not isinstance(value, bool):
                raise ValueError(f"{field_name} 必须是布尔值")
            kwargs[item.name] = value
        elif item.name == "waypoints":
            kwargs[item.name] = _decode_waypoints(value, field_name)
        elif item.name == "tags":
            if not isinstance(value, list) or any(not isinstance(tag, str) for tag in value):
                raise ValueError(f"{field_name} 必须是字符串列表")
            kwargs[item.name] = tuple(value)
        elif cls is MetadataConfig and item.name in {"task_name", "object_name", "description"}:
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field_name} 必须是非空字符串")
            kwargs[item.name] = value
        elif (cls is HardwareConfig and item.name in {"dm_port", "tactile_port"}) or (
            cls is OutputConfig and item.name == "root"
        ):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field_name} 必须是非空字符串")
            kwargs[item.name] = value
        elif _is_int_annotation(type_hints.get(item.name)):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{field_name} 必须是整数")
            kwargs[item.name] = value
        elif _is_optional_dataclass(type_hints.get(item.name)):
            if value is None:
                kwargs[item.name] = None
            else:
                kwargs[item.name] = _decode_dataclass(
                    value, get_args(type_hints[item.name])[0], field_name
                )
        elif _is_nested_dataclass(type_hints.get(item.name)):
            kwargs[item.name] = _decode_dataclass(value, type_hints[item.name], field_name)
        else:
            kwargs[item.name] = _finite_number(value, field_name)
    return cls(**kwargs)


def _is_int_annotation(annotation: Any) -> bool:
    """判断注解是否为纯 int。"""
    return annotation is int or annotation is bool


def _is_nested_dataclass(annotation: Any) -> bool:
    """判断注解是否为嵌套 dataclass 类型。"""
    origin = get_origin(annotation)
    if origin is not None:
        return False
    return isinstance(annotation, type) and hasattr(annotation, "__dataclass_fields__")


def _is_optional_dataclass(annotation: Any) -> bool:
    """判断注解是否为 ``X | None`` 形式的可选嵌套 dataclass。"""
    if get_origin(annotation) is not UnionType:
        return False
    arguments = get_args(annotation)
    return len(arguments) == 2 and arguments[1] is type(None) and _is_nested_dataclass(arguments[0])


def _decode_waypoints(data: object, name: str) -> tuple[WaypointConfig, ...]:
    """解码曲线节点序列。"""
    if not isinstance(data, list) or not data:
        raise ValueError(f"{name} 必须是非空节点列表")
    waypoints = []
    for index, item in enumerate(data):
        mapping = _strict_mapping(item, f"{name}[{index}]")
        unknown = set(mapping) - {"t_s", "force_n"}
        if unknown:
            raise ValueError(f"{name}[{index}] 包含未知字段：{', '.join(sorted(unknown))}")
        if "t_s" not in mapping or "force_n" not in mapping:
            raise ValueError(f"{name}[{index}] 必须同时提供 t_s 与 force_n")
        waypoints.append(
            WaypointConfig(
                t_s=_finite_number(mapping["t_s"], f"{name}[{index}].t_s"),
                force_n=_finite_number(mapping["force_n"], f"{name}[{index}].force_n"),
            )
        )
    return tuple(waypoints)


def load_experiment_config(path: Path) -> ExperimentConfig:
    """从严格 YAML 文件加载并冻结实验配置。

    Args:
        path: YAML 配置文件路径。

    Returns:
        通过全部校验的冻结 ``ExperimentConfig``。

    Raises:
        ValueError: 文件不可读、YAML 无效或配置违反校验。
    """
    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"无法读取配置文件：{path}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"YAML 格式无效：{path}") from error
    mapping = _strict_mapping(data, "配置根节点")
    unknown = set(mapping) - set(_ROOT_SCHEMA)
    if unknown:
        raise ValueError(f"配置根节点包含未知字段：{', '.join(sorted(unknown))}")
    kwargs: dict[str, Any] = {}
    for name, cls in _ROOT_SCHEMA.items():
        if name not in mapping:
            continue
        kwargs[name] = _decode_dataclass(mapping[name], cls, name)
    return ExperimentConfig(**kwargs)


def experiment_config_record(config: ExperimentConfig) -> dict[str, Any]:
    """返回可 JSON 序列化的普通配置记录。"""
    return asdict(config)

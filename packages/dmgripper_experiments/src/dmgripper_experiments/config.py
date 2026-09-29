"""通用 DMgripper 真机抓取实验配置。

设计约定（与 docs/dmgripper-experiments.md 一致）：

- 显式阶段依次为预载、人工起滑摩擦候选、冻结双侧先验自适应。
- ``safety`` 是唯一目标限幅定义点，目标上限与速率不在其他配置重复。
- 候选配置不内置臆测的摩擦先验；自适应必须注明人工接受的来源。
"""

from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass, field, fields
from fractions import Fraction
from pathlib import Path
from types import UnionType
from typing import Any, Literal, get_args, get_origin, get_type_hints

import yaml
from dmgripper_hardware import DEFAULT_USB2CAN_PORT, make_dm4310p_gripper_config
from papillarray_hardware import (
    DEFAULT_PAPILLARRAY_PORT,
)

from dm_grasp_core.grasp.adaptive import AdaptiveLoadConfig

FinishBehavior = Literal["hold", "return"]
ExperimentStage = Literal["preload", "friction", "adaptive"]

# 已接触触点的法向释放阈值比例（滞回），与单侧 3×3 布局无关的力比例。
_CONTACT_RELEASE_RATIO = float(Fraction(2, 3))


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

    说明与标签属一次性信息，由命令行 ``--metadata-*`` 覆盖入口提供，
    不占用实验配置字段。

    Attributes:
        task_name: 任务显示名；进入输出目录与记录。
        object_name: 物体显示名；进入输出目录与记录。
    """

    task_name: str = "grasp"
    object_name: str = "object"
    condition: str = "unspecified"

    def __post_init__(self) -> None:
        """验证名称可用作目录组件。"""
        sanitize_directory_component(self.task_name)
        sanitize_directory_component(self.object_name)
        if not isinstance(self.condition, str) or not self.condition.strip():
            raise ValueError("metadata.condition 必须是非空字符串")


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

    首包等待、清零稳定与滤波重置阈值属设备行为常数，由运行时模块常量
    提供；触觉包新鲜度只有 ``tactile_timeout_s`` 一条硬阈值。

    Attributes:
        control_rate_hz: 控制循环目标频率。
        max_control_gap_s: 相邻两个控制周期允许的最大真实间隔。
        tactile_timeout_s: 触觉快照的新鲜度上限，同时作为因果配对上限。
        tactile_cutoff_hz: 触觉外层一阶低通截止频率（导纳与记录使用）。
    """

    control_rate_hz: float = 250.0
    max_control_gap_s: float = 0.1
    tactile_timeout_s: float = 0.2
    tactile_cutoff_hz: float = 20.0

    def __post_init__(self) -> None:
        """验证时序参数与跨字段关系。"""
        for item in fields(self):
            _finite_number(getattr(self, item.name), f"timing.{item.name}")
        if self.tactile_cutoff_hz < 0.0:
            raise ValueError("timing.tactile_cutoff_hz 不得为负")
        for item in fields(self):
            if item.name != "tactile_cutoff_hz" and getattr(self, item.name) <= 0.0:
                raise ValueError(f"timing.{item.name} 必须大于 0")
        if self.max_control_gap_s <= 1.0 / self.control_rate_hz:
            raise ValueError("timing.max_control_gap_s 必须大于一个控制周期")
        if self.max_control_gap_s > self.tactile_timeout_s:
            raise ValueError("timing.max_control_gap_s 不得大于 timing.tactile_timeout_s")


@dataclass(frozen=True, slots=True)
class LifecycleConfig:
    """通用生命周期各阶段的判定与轨迹参数。

    零力窗口阈值、接近／回位加速度与 jerk、回位超时等运行保护默认值
    由运行时模块常量提供。失接触策略固定为任一侧失接触即故障。

    Attributes:
        verify_zero_force: 使能前是否执行零力窗口验证。
        contact_on_n: 双侧接触确认的每侧力阈值。
        contact_on_stable_s: 接触确认需要持续的时间。
        contact_off_n: 失接触判定的每侧力阈值。
        contact_off_stable_s: 失接触确认需要持续的时间。
        contact_transition_s: 接触后接近速度的过渡时长。
        approach_closure_velocity_m_s: 接近段闭合速度上限。
        return_closure_velocity_m_s: 回位段闭合速度上限。
        preload_min_force_ratio: 最终预载目标的最低比例。
        preload_stable_time_s: 初始抓力稳定需要持续的时长。
        preload_timeout_s: preload 等待上限。
        preload_force_rate_n_s: 预载平滑升力峰值速率；None 保留常值目标。
        on_finished: 任务计时完成后的行为（保持抓握或自动回位）。
    """

    verify_zero_force: bool = True
    contact_on_n: float = 0.2
    contact_on_stable_s: float = 0.1
    contact_off_n: float = 0.1
    contact_off_stable_s: float = 0.15
    contact_transition_s: float = 0.15
    approach_closure_velocity_m_s: float = 0.015
    return_closure_velocity_m_s: float = 0.035
    preload_min_force_ratio: float = 0.75
    preload_stable_time_s: float = 0.5
    preload_timeout_s: float = 30.0
    preload_force_rate_n_s: float | None = 1.0
    on_finished: FinishBehavior = "hold"

    def __post_init__(self) -> None:
        """验证生命周期参数与跨字段关系。"""
        for item in fields(self):
            name, value = f"lifecycle.{item.name}", getattr(self, item.name)
            if item.name == "preload_force_rate_n_s":
                if value is not None:
                    _finite_number(value, name, positive=True)
                continue
            if item.name == "verify_zero_force":
                if not isinstance(value, bool):
                    raise ValueError("lifecycle.verify_zero_force 必须是布尔值")
                continue
            if item.name == "on_finished":
                continue
            _finite_number(value, name)
        if self.on_finished not in get_args(FinishBehavior):
            raise ValueError("lifecycle.on_finished 必须是 hold 或 return")
        for name in (
            "contact_on_stable_s",
            "contact_off_stable_s",
            "contact_transition_s",
            "preload_stable_time_s",
            "preload_timeout_s",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"lifecycle.{name} 必须大于 0")
        if not 0.0 <= self.contact_off_n < self.contact_on_n:
            raise ValueError("lifecycle.contact_off_n 必须非负且小于 contact_on_n")
        if not 0.0 < self.preload_min_force_ratio <= 1.0:
            raise ValueError("lifecycle.preload_min_force_ratio 必须位于 (0, 1]")


@dataclass(frozen=True, slots=True)
class FrozenFrictionConfig:
    """人工接受的分侧有效摩擦／承载先验，仅用于相同抓取条件。"""

    left_mu: float = 0.0
    right_mu: float = 0.0
    source: str = ""
    safety_factor: float = 1.5
    filter_tau_s: float = 0.05
    allow_target_decrease: bool = False
    gap_gain_per_s: float = 8.0
    load_rate_gain: float = 1.0

    def __post_init__(self) -> None:
        """拒绝无效或缺失的已接受先验。"""
        for name in (
            "left_mu",
            "right_mu",
            "safety_factor",
            "filter_tau_s",
            "gap_gain_per_s",
            "load_rate_gain",
        ):
            _finite_number(getattr(self, name), f"reference.friction.{name}")
        if self.left_mu <= 0 or self.right_mu <= 0:
            raise ValueError("自适应阶段需要人工接受的双侧正数 μ")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("自适应阶段需要非空 reference.friction.source 标定来源")
        if (
            self.safety_factor < 1
            or min(self.filter_tau_s, self.gap_gain_per_s) <= 0
            or self.load_rate_gain < 0
        ):
            raise ValueError("摩擦调度参数超出有效范围")
        if not isinstance(self.allow_target_decrease, bool):
            raise ValueError("allow_target_decrease 必须是布尔值")


@dataclass(frozen=True, slots=True)
class AdaptiveReferenceConfig:
    """每侧固定预载候选与可选的人工冻结有效摩擦先验。"""

    initial_force_n: float = 0.5
    duration_s: float | None = None
    friction: FrozenFrictionConfig | None = None
    preload_source: str | None = None

    def __post_init__(self) -> None:
        """校验固定力与可选来源。"""
        _finite_number(self.initial_force_n, "reference.initial_force_n", positive=True)
        if self.duration_s is not None:
            _finite_number(self.duration_s, "reference.duration_s", positive=True)
        if self.friction is not None and not isinstance(self.friction, FrozenFrictionConfig):
            raise ValueError("reference.friction 必须为冻结摩擦配置")
        if self.preload_source is not None and (
            not isinstance(self.preload_source, str) or not self.preload_source.strip()
        ):
            raise ValueError("reference.preload_source 必须为非空字符串或 null")


@dataclass(frozen=True, slots=True)
class AdmittanceConfig:
    """二阶导纳外环参数；运动边界独立于 MIT 关节限幅。

    ``max_closing_velocity_m_s``、``max_opening_velocity_m_s`` 与
    ``max_acceleration_m_s2`` 为可选外环状态边界；省略时保持旧行为，只受
    MIT 关节速度、机械角和合成力矩约束。
    """

    mass_kg: float = 0.2
    damping_ns_m: float = 15.0
    stiffness_n_m: float = 1.0
    force_deadband_n: float = 0.1
    prevent_unloading: bool = False
    feedforward_ratio: float = 1.0
    max_closing_velocity_m_s: float | None = None
    max_opening_velocity_m_s: float | None = None
    max_acceleration_m_s2: float | None = None

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
        for name in (
            "max_closing_velocity_m_s",
            "max_opening_velocity_m_s",
            "max_acceleration_m_s2",
        ):
            value = getattr(self, name)
            if value is not None:
                _finite_number(value, f"controller.admittance.{name}", positive=True)


@dataclass(frozen=True, slots=True)
class ControllerConfig:
    """导纳控制器参数与 MIT 跟踪增益。

    导纳是唯一控制器；回位段增益独立于跟踪段，velocity／torque 限幅
    沿用 DM 协议边界。

    Attributes:
        closing_torque_only: 接近至正常保持期间禁止预测张开力矩。
        zero_tracking_velocity: 跟踪段 MIT 目标速度是否固定为零。
        mit_kp: 跟踪段 MIT 位置增益。
        mit_kd: 跟踪段 MIT 阻尼增益。
        velocity_limit_rad_s: 关节速度限幅。
        torque_limit_nm: 跟踪段力矩限幅。
        return_mit_kp: 回位段 MIT 位置增益。
        return_mit_kd: 回位段 MIT 阻尼增益。
        return_torque_limit_nm: 回位段力矩限幅。
        admittance: 二阶导纳参数。
    """

    closing_torque_only: bool = True
    zero_tracking_velocity: bool = True
    mit_kp: float = 10.0
    mit_kd: float = 2.0
    velocity_limit_rad_s: float = 0.3
    torque_limit_nm: float = 4.0
    return_mit_kp: float = 10.0
    return_mit_kd: float = 0.5
    return_torque_limit_nm: float = 2.0
    admittance: AdmittanceConfig = field(default_factory=AdmittanceConfig)

    def __post_init__(self) -> None:
        """验证 MIT 参数范围。"""
        for name in ("closing_torque_only", "zero_tracking_velocity"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"controller.{name} 必须是布尔值")
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


@dataclass(frozen=True, slots=True)
class EstimationConfig:
    """仅记录等效接触刚度的诊断开关。

    估计方法固定为滑动窗口线性拟合，内部界限与平滑系数沿用仿真验证
    起点；本节只决定是否更新估计并记录诊断。

    Attributes:
        enabled: 是否更新刚度估计并记录诊断。
    """

    enabled: bool = False

    def __post_init__(self) -> None:
        """验证开关类型。"""
        if not isinstance(self.enabled, bool):
            raise ValueError("estimation.enabled 必须是布尔值")


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    """唯一限幅定义点：原始力保护、目标上限与速率边界。

    目标上限同时是统一策略调度包络的上界；增／退力速率在此定义一次，
    调度器构造期派生，不再有第二处旋钮。

    Attributes:
        max_target_force_n: 平均单侧法向目标力上限（N）。
        force_ceiling_n: 任一侧原始法向力硬保护线（N）。
        max_contact_compression_m: 首次接触后两指总闭合增量上限（m）。
        max_force_rate_n_s: 目标力上升速率上限（N/s）。
        max_force_decrease_rate_n_s: 目标力下降速率上限（N/s）。
        approach_feedforward_force_n: 接近轨迹正向机构前馈等效力（N）。
    """

    max_target_force_n: float = 30.0
    force_ceiling_n: float = 40.0
    max_contact_compression_m: float = 0.040
    max_force_rate_n_s: float = 10.0
    max_force_decrease_rate_n_s: float = 1.0
    approach_feedforward_force_n: float = 1.0

    def __post_init__(self) -> None:
        """验证保护参数与有序性。"""
        for item in fields(self):
            _finite_number(getattr(self, item.name), f"safety.{item.name}", positive=True)
        if self.max_target_force_n >= self.force_ceiling_n:
            raise ValueError("safety.max_target_force_n 必须严格小于 force_ceiling_n")


@dataclass(frozen=True, slots=True)
class RecordingConfig:
    """触觉记录速率与体积上限策略。

    tactile_stride 按设备包计数抽样正常包，首个包与异常包始终保留；
    max_duration_s 或 max_recording_mib 触发后按 release 语义受限回位
    正常收尾，原因写入 manifest 的 stop_reason 与事件流。
    """

    tactile_stride: int = 4
    max_duration_s: float | None = 600.0
    max_recording_mib: float | None = 512.0

    def __post_init__(self) -> None:
        """验证抽样间隔与上限参数。"""
        if (
            isinstance(self.tactile_stride, bool)
            or not isinstance(self.tactile_stride, int)
            or self.tactile_stride < 1
        ):
            raise ValueError("recording.tactile_stride 必须是不小于 1 的整数")
        for name, value in (
            ("recording.max_duration_s", self.max_duration_s),
            ("recording.max_recording_mib", self.max_recording_mib),
        ):
            if value is not None:
                _finite_number(value, name, positive=True)


@dataclass(frozen=True, slots=True)
class ExperimentConfig:
    """一次通用抓取实验的完整冻结配置。"""

    stage: ExperimentStage = "preload"
    metadata: MetadataConfig = field(default_factory=MetadataConfig)
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    lifecycle: LifecycleConfig = field(default_factory=LifecycleConfig)
    reference: AdaptiveReferenceConfig = field(default_factory=AdaptiveReferenceConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    estimation: EstimationConfig = field(default_factory=EstimationConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    recording: RecordingConfig = field(default_factory=RecordingConfig)

    def __post_init__(self) -> None:
        """验证段类型与跨字段安全、控制相容性约束。"""
        expected = {
            "metadata": MetadataConfig,
            "hardware": HardwareConfig,
            "timing": TimingConfig,
            "lifecycle": LifecycleConfig,
            "reference": AdaptiveReferenceConfig,
            "controller": ControllerConfig,
            "estimation": EstimationConfig,
            "safety": SafetyConfig,
            "recording": RecordingConfig,
        }
        for name, cls in expected.items():
            if not isinstance(getattr(self, name), cls):
                raise ValueError(f"{name} 必须是 {cls.__name__}")
        if self.stage not in get_args(ExperimentStage):
            raise ValueError("stage 必须是 preload、friction 或 adaptive")
        if self.reference.initial_force_n > self.safety.max_target_force_n:
            raise ValueError("初始目标力不得大于 safety.max_target_force_n")
        if self.reference.initial_force_n < self.lifecycle.contact_on_n:
            raise ValueError("初始目标力不得小于 lifecycle.contact_on_n")
        if self.stage in {"friction", "adaptive"} and not self.reference.preload_source:
            raise ValueError("friction/adaptive 阶段必须记录 reference.preload_source")
        if self.stage == "adaptive" and self.reference.friction is None:
            raise ValueError("adaptive 阶段必须显式提供人工接受的 reference.friction")
        if self.stage != "adaptive" and self.reference.friction is not None:
            raise ValueError("仅 adaptive 阶段可提供 reference.friction")

    @property
    def adaptive_load_config(self) -> AdaptiveLoadConfig:
        """由冻结双侧先验与 safety 单源边界构造调度器。"""
        friction = self.reference.friction
        if self.stage != "adaptive" or friction is None:
            raise ValueError("仅 adaptive 阶段可构造调度器")
        return AdaptiveLoadConfig(
            left_friction=friction.left_mu,
            right_friction=friction.right_mu,
            safety_factor=friction.safety_factor,
            min_force_n=self.reference.initial_force_n,
            max_force_n=self.safety.max_target_force_n,
            max_force_rate_n_s=self.safety.max_force_rate_n_s,
            max_force_decrease_rate_n_s=self.safety.max_force_decrease_rate_n_s,
            gap_gain_per_s=friction.gap_gain_per_s,
            load_rate_gain=friction.load_rate_gain,
            filter_tau_s=friction.filter_tau_s,
            allow_target_decrease=friction.allow_target_decrease,
        )


_ROOT_SCHEMA: dict[str, type[Any]] = {
    "metadata": MetadataConfig,
    "hardware": HardwareConfig,
    "timing": TimingConfig,
    "lifecycle": LifecycleConfig,
    "reference": AdaptiveReferenceConfig,
    "controller": ControllerConfig,
    "estimation": EstimationConfig,
    "safety": SafetyConfig,
    "recording": RecordingConfig,
}

_STRING_ENUM_FIELDS: dict[tuple[type, str], tuple[str, ...]] = {
    (LifecycleConfig, "on_finished"): get_args(FinishBehavior),
    (ExperimentConfig, "stage"): get_args(ExperimentStage),
}


def _strict_mapping(value: object, name: str) -> dict[str, Any]:
    """取得字符串键的 YAML 映射。"""
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{name} 必须是字符串键的映射")
    return value


def _decode_dataclass(data: object, cls: type[Any], name: str) -> Any:
    """按 dataclass 字段严格解码 YAML 映射。

    支持 bool、有限数值、枚举字符串、嵌套 dataclass；未知字段与
    非匹配类型一律报错，不做静默默认。
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
        elif cls is MetadataConfig and item.name in {"task_name", "object_name", "condition"}:
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field_name} 必须是非空字符串")
            kwargs[item.name] = value
        elif (
            (cls is HardwareConfig and item.name in {"dm_port", "tactile_port"})
            or (cls is FrozenFrictionConfig and item.name == "source")
            or (
                cls is AdaptiveReferenceConfig
                and item.name == "preload_source"
                and value is not None
            )
        ):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field_name} 必须是非空字符串")
            kwargs[item.name] = value
        elif _is_int_annotation(type_hints.get(item.name)):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{field_name} 必须是整数")
            kwargs[item.name] = value
        elif type_hints.get(item.name) == str | None:
            kwargs[item.name] = None if value is None else str(value)
        elif type_hints.get(item.name) == float | None:
            kwargs[item.name] = None if value is None else _finite_number(value, field_name)
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
    unknown = set(mapping) - set(_ROOT_SCHEMA) - {"stage"}
    if unknown:
        raise ValueError(f"配置根节点包含未知字段：{', '.join(sorted(unknown))}")
    kwargs: dict[str, Any] = {}
    for name, cls in _ROOT_SCHEMA.items():
        if name not in mapping:
            continue
        kwargs[name] = _decode_dataclass(mapping[name], cls, name)
    if "stage" in mapping:
        if mapping["stage"] not in get_args(ExperimentStage):
            raise ValueError("stage 必须是 preload、friction 或 adaptive")
        kwargs["stage"] = mapping["stage"]
    return ExperimentConfig(**kwargs)


def experiment_config_record(config: ExperimentConfig) -> dict[str, Any]:
    """返回可 JSON 序列化的普通配置记录。"""
    return asdict(config)


def experiment_config_delta(config: ExperimentConfig) -> dict[str, Any]:
    """返回与默认配置的差异记录；只保留发生变化的叶子值。

    仿 transformers 的 diff-only 留档：默认值承载主力 profile，review
    一份短差异清单比通读全量 schema 更容易发现意外漂移。
    """
    return _diff_mapping(asdict(ExperimentConfig()), asdict(config))


def _diff_mapping(default: Any, actual: Any) -> Any:
    """递归比较默认值与实际值，返回仅含差异的结构。"""
    if isinstance(default, dict) and isinstance(actual, dict):
        delta = {key: _diff_mapping(default[key], actual[key]) for key in actual if key in default}
        return {key: value for key, value in delta.items() if value != {}}
    if isinstance(default, list) and isinstance(actual, list):
        return actual if default != actual else {}
    return {} if default == actual else actual

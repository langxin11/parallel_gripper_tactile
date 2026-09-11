"""用于夹爪仿真与工具的、经 Pydantic 校验的 YAML profile。"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal, TypeAlias
import xml.etree.ElementTree as ET

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, ValidationError, model_validator
import yaml


class ProfileLoadError(ValueError):
    """当 profile 在 Pydantic 校验之前无法解码时抛出。"""


class _FrozenModel(BaseModel):
    """拒绝拼写错误的字段以及运行时变更的基类模型。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class TaxelTactileLayout(_FrozenModel):
    """由力传感器或碰撞几何体组成的规则命名网格。"""

    mode: Literal["force_sensor", "contact_geom"]
    rows: Annotated[int, Field(gt=0)]
    cols: Annotated[int, Field(gt=0)]
    left_prefix: Annotated[str, Field(min_length=1)]
    right_prefix: Annotated[str, Field(min_length=1)]

    def names(self, side: Literal["left", "right"]) -> tuple[str, ...]:
        """返回单个指尖按行优先排列的传感器或几何体名称。"""
        prefix = self.left_prefix if side == "left" else self.right_prefix
        return tuple(
            f"{prefix}{row}{column}" for row in range(self.rows) for column in range(self.cols)
        )


class TouchGridTactileLayout(_FrozenModel):
    """一对 MuJoCo ``touch_grid`` 插件传感器。

    ``rows`` 与 ``cols`` 特意不出现在 YAML 中。加载器会从被引用 MJCF 中
    插件的 ``size`` 配置推导出它们，并将校验结果记录在此不可变模型上。
    """

    mode: Literal["touch_grid"]
    left_sensor: Annotated[str, Field(min_length=1)]
    right_sensor: Annotated[str, Field(min_length=1)]
    rows: Annotated[int, Field(gt=0, exclude=True)] = 1
    cols: Annotated[int, Field(gt=0, exclude=True)] = 1

    def names(self, side: Literal["left", "right"]) -> tuple[str, ...]:
        """返回与 ``side`` 关联的那一个插件传感器名称。"""
        return (self.left_sensor if side == "left" else self.right_sensor,)


TactileLayout: TypeAlias = TaxelTactileLayout | TouchGridTactileLayout
StiffnessEstimatorMethod: TypeAlias = Literal["secant_ewma", "window_linear", "window_quadratic"]
STIFFNESS_ESTIMATOR_METHODS: tuple[StiffnessEstimatorMethod, ...] = (
    "secant_ewma",
    "window_linear",
    "window_quadratic",
)


class MITControl(_FrozenModel):
    """MIT 风格输出轴的位置、速度和力矩限值与增益。"""

    p_min: FiniteFloat
    p_max: FiniteFloat
    v_max: Annotated[FiniteFloat, Field(gt=0)]
    t_max: Annotated[FiniteFloat, Field(gt=0)]
    kp: Annotated[FiniteFloat, Field(ge=0, le=500)]
    kd: Annotated[FiniteFloat, Field(ge=0, le=5)]
    t_ff: FiniteFloat = 0.0

    @model_validator(mode="after")
    def validate_limits(self) -> "MITControl":
        """确保命令位置与前馈范围相互一致。"""
        if self.p_min >= self.p_max:
            raise ValueError("p_min must be smaller than p_max")
        if abs(self.t_ff) > self.t_max:
            raise ValueError("t_ff must not exceed t_max")
        return self


class CrankSliderGeometry(_FrozenModel):
    """DM_Gripper 的曲柄滑块开度几何参数。"""

    theta0_rad: FiniteFloat
    crank_radius_m: Annotated[FiniteFloat, Field(gt=0)]
    link_length_m: Annotated[FiniteFloat, Field(gt=0)]
    offset_m: Annotated[FiniteFloat, Field(ge=0)]


class ContactStiffnessControl(_FrozenModel):
    """在线接触刚度估计、前馈与刚度感知位置限幅参数。"""

    enabled: bool = True
    method: StiffnessEstimatorMethod = "window_linear"
    initial_n_per_m: Annotated[FiniteFloat, Field(gt=0)]
    min_n_per_m: Annotated[FiniteFloat, Field(gt=0)]
    max_n_per_m: Annotated[FiniteFloat, Field(gt=0)]
    filter_alpha: Annotated[FiniteFloat, Field(gt=0, le=1)]
    min_delta_closure_m: Annotated[FiniteFloat, Field(gt=0)]
    min_delta_force_n: Annotated[FiniteFloat, Field(gt=0)]
    window_size: Annotated[int, Field(gt=0)] = 25
    min_samples: Annotated[int, Field(gt=0)] = 8
    position_feedforward_gain: Annotated[FiniteFloat, Field(ge=0, le=1)] = 0.25
    torque_feedforward_gain: Annotated[FiniteFloat, Field(ge=0, le=1)] = 1.0
    position_limit_enabled: bool = False
    position_limit_force_rate_n_s: Annotated[FiniteFloat, Field(gt=0)] = 10.0
    position_limit_stiffness_safety_factor: Annotated[FiniteFloat, Field(ge=1)] = 1.0

    @model_validator(mode="after")
    def validate_stiffness_limits(self) -> "ContactStiffnessControl":
        """要求刚度估计范围有效，初始值位于范围内。"""
        if self.min_n_per_m >= self.max_n_per_m:
            raise ValueError("min_n_per_m must be smaller than max_n_per_m")
        if not self.min_n_per_m <= self.initial_n_per_m <= self.max_n_per_m:
            raise ValueError("initial_n_per_m must lie within stiffness limits")
        if self.min_samples > self.window_size:
            raise ValueError("min_samples must not exceed window_size")
        degree = {"window_linear": 1, "window_quadratic": 2}.get(self.method)
        if degree is not None and self.min_samples < degree + 1:
            raise ValueError("min_samples must provide enough samples for the selected method")
        if self.position_limit_enabled and self.position_feedforward_gain > 0:
            raise ValueError(
                "stiffness position feedforward and position limit are mutually exclusive"
            )
        return self


class AdrcControl(_FrozenModel):
    """一阶线性自抗扰（LADRC）外环参数。

    被控假设为 ``df/dt = f + b0·u``：f 为滤波后的法向力 [N]，u 为闭合速度
    [m/s]，b0 为名义输入增益 [N/m]，量级约等于接触等效刚度。带宽默认值在
    step 任务、medium 材料上按“饱和比例不超 5% 中 rmse 最小”选定
    （ω_c ∈ {10, 20, 40}、ω_o = 3ω_c）。``NormalForceControl.adrc``
    默认 ``None`` 表示不启用；一旦配置，跟踪阶段由 LADRC 外环替换 PID 位置修正。
    """

    b0_n_per_m: Annotated[FiniteFloat, Field(gt=0)] = 2700.0
    controller_bandwidth_rad_s: Annotated[FiniteFloat, Field(gt=0)] = 40.0
    observer_bandwidth_rad_s: Annotated[FiniteFloat, Field(gt=0)] = 120.0
    max_closing_velocity_m_s: Annotated[FiniteFloat, Field(gt=0)] = 0.02


class TorqueAdrcControl(_FrozenModel):
    """二阶直接力矩 LADRC 的控制导向模型与工程约束。

    被控假设为 ``d²F/dt² = f + b0·τ``。名义输入增益按
    ``b0 = scale·K_hat·J(q)/I_eq`` 在线调度，其中 ``K_hat`` 为整体等效
    接触刚度，``J(q)`` 为总闭合行程雅可比，``I_eq`` 为折算到电机输出轴的
    等效惯量，``scale`` 用小信号辨识校准未建模的输入增益。跟踪阶段由
    LADRC 直接输出电机力矩，MIT ``kp/kd`` 仅在该阶段旁路；接近阶段仍使用
    原有阻抗参数。LESO 使用独立的一阶轻度预处理测量，不复用 PID 和指标的
    较低带宽滤波结果。可选线性 TD 仅安排参考过渡过程；默认 ``None`` 保持
    Yu 等文献所用的无显式 TD 结构。
    """

    equivalent_inertia_kg_m2: Annotated[FiniteFloat, Field(gt=0)] = 0.0021617741125
    input_gain_scale: Annotated[FiniteFloat, Field(gt=0, le=10)] = 2.0
    controller_bandwidth_rad_s: Annotated[FiniteFloat, Field(gt=0)] = 60.0
    observer_bandwidth_rad_s: Annotated[FiniteFloat, Field(gt=0)] = 240.0
    measurement_filter_cutoff_hz: Annotated[FiniteFloat, Field(gt=0)] = 40.0
    tracking_differentiator_bandwidth_rad_s: Annotated[FiniteFloat, Field(gt=0)] | None = None
    min_input_gain_n_per_n_m_s2: Annotated[FiniteFloat, Field(gt=0)] = 5_000.0
    max_input_gain_n_per_n_m_s2: Annotated[FiniteFloat, Field(gt=0)] = 200_000.0
    max_torque_rate_n_m_s: Annotated[FiniteFloat, Field(gt=0)] = 50.0

    @model_validator(mode="after")
    def validate_input_gain_limits(self) -> "TorqueAdrcControl":
        """要求名义输入增益裁剪范围严格递增。"""
        if self.min_input_gain_n_per_n_m_s2 >= self.max_input_gain_n_per_n_m_s2:
            raise ValueError(
                "min_input_gain_n_per_n_m_s2 must be smaller than max_input_gain_n_per_n_m_s2"
            )
        return self


class DMAdmittanceControl(_FrozenModel):
    """DM 共享导纳参数；质量 kg，阻尼 N·s/m，刚度 N/m，角度 rad。"""

    mass_kg: Annotated[FiniteFloat, Field(gt=0)] = 0.02
    damping_ns_m: Annotated[FiniteFloat, Field(ge=0)] = 0.20
    stiffness_n_m: Annotated[FiniteFloat, Field(ge=0)] = 1.0
    feedforward_ratio: Annotated[FiniteFloat, Field(ge=0, le=1)] = 0.2
    contact_stable_time_s: Annotated[FiniteFloat, Field(ge=0)] = 0.10
    contact_transition_time_s: Annotated[FiniteFloat, Field(gt=0)] = 0.15
    approach_velocity_rad_s: Annotated[FiniteFloat, Field(gt=0)] = 0.20
    approach_acceleration_rad_s2: Annotated[FiniteFloat, Field(gt=0)] = 0.50
    approach_jerk_rad_s3: Annotated[FiniteFloat, Field(gt=0)] = 2.0
    approach_feedforward_force_n: Annotated[FiniteFloat, Field(ge=0)] = 2.0
    approach_feedforward_ratio: Annotated[FiniteFloat, Field(ge=0, le=1)] = 1.0
    position_min_rad: FiniteFloat = 0.0
    position_max_rad: FiniteFloat = 1.5707963267948966
    velocity_limit_rad_s: Annotated[FiniteFloat, Field(gt=0)] = 0.30
    closing_direction: Literal[-1, 1] = 1
    feedforward_torque_limit_nm: Annotated[FiniteFloat, Field(gt=0)] = 4.0
    mit_torque_limit_nm: Annotated[FiniteFloat, Field(gt=0)] = 4.0

    @model_validator(mode="after")
    def validate_admittance_limits(self) -> "DMAdmittanceControl":
        """拒绝倒置机械范围和超出公共速度限制的接近轨迹。"""
        if self.position_min_rad >= self.position_max_rad:
            raise ValueError("admittance position limits must be increasing")
        if self.approach_velocity_rad_s > self.velocity_limit_rad_s:
            raise ValueError("approach velocity exceeds admittance velocity limit")
        if self.feedforward_torque_limit_nm > self.mit_torque_limit_nm:
            raise ValueError("feedforward torque limit exceeds MIT torque limit")
        return self


class DMForceSupervisorControl(_FrozenModel):
    """DM 公共接触阶段机参数。"""

    contact_stable_time_s: Annotated[FiniteFloat, Field(ge=0)] = 0.0
    contact_transition_time_s: Annotated[FiniteFloat, Field(ge=0)] = 0.05
    release_policy: Literal["any_side", "both_sides"] = "any_side"


class NormalForceControl(_FrozenModel):
    """外环法向力跟踪参数。"""

    target_n: Annotated[FiniteFloat, Field(gt=0)]
    contact_threshold_n: Annotated[FiniteFloat, Field(gt=0)]
    contact_confirm_steps: Annotated[int, Field(gt=0)]
    release_threshold_n: Annotated[FiniteFloat, Field(ge=0)]
    release_confirm_steps: Annotated[int, Field(gt=0)]
    kp: Annotated[FiniteFloat, Field(ge=0)]
    ki: Annotated[FiniteFloat, Field(ge=0)]
    kd: Annotated[FiniteFloat, Field(ge=0)] = 0.0
    max_position_adjustment: Annotated[FiniteFloat, Field(gt=0)]
    filter_cutoff_hz: Annotated[FiniteFloat, Field(gt=0)]
    sensor_taxel_normal_noise_std_n: tuple[
        Annotated[FiniteFloat, Field(ge=0)], Annotated[FiniteFloat, Field(ge=0)]
    ] = (0.0, 0.0)
    sensor_taxel_shear_noise_std_n: tuple[
        Annotated[FiniteFloat, Field(ge=0)], Annotated[FiniteFloat, Field(ge=0)]
    ] = (0.0, 0.0)
    sensor_noise_seed: Annotated[int, Field(ge=0)] = 0
    geometry: CrankSliderGeometry | None = None
    stiffness: ContactStiffnessControl | None = None
    # 直接力矩式力控增益：大于 0 时跟踪阶段把力误差直接注入 MIT 前馈力矩
    # （并将 MIT kp/kd 逐周期覆盖为 0）；默认 0 保持位置式行为。
    torque_feedback_gain: Annotated[FiniteFloat, Field(ge=0)] = 0.0
    # 一阶 LADRC 外环参数：非 None 时跟踪阶段以 LADRC 替换 PID 位置修正外环，
    # 输出的闭合速度逐周期积分进位置修正；默认 None 表示不启用。
    adrc: AdrcControl | None = None
    # 二阶直接力矩 LADRC 参数：非 None 时跟踪阶段旁路 MIT kp/kd，由 LESO
    # 依据在线刚度、机构雅可比和名义惯量调度输入增益并直接输出力矩。
    torque_adrc: TorqueAdrcControl | None = None
    admittance: DMAdmittanceControl | None = None
    supervisor: DMForceSupervisorControl | None = None

    @model_validator(mode="after")
    def validate_force_control(self) -> "NormalForceControl":
        """要求释放阈值和可选刚度估计配置相互一致。"""
        if self.admittance is not None:
            if self.geometry is None:
                raise ValueError("admittance requires geometry")
            if self.adrc is not None or self.torque_adrc is not None or self.torque_feedback_gain:
                raise ValueError("admittance cannot mix with ADRC or torque feedback")
        if self.release_threshold_n > self.contact_threshold_n:
            raise ValueError("release_threshold_n must not exceed contact_threshold_n")
        if self.stiffness is not None and self.stiffness.enabled and self.geometry is None:
            raise ValueError("geometry is required when contact stiffness estimation is enabled")
        return self


class PositionControl(_FrozenModel):
    """直接位置控制的执行器 profile。"""

    mode: Literal["position"]
    actuator: Annotated[str, Field(min_length=1)]
    open: FiniteFloat
    closed: FiniteFloat


class MITTorqueControl(_FrozenModel):
    """通过受限的 MIT 力矩命令控制的位置目标。"""

    mode: Literal["mit_torque"]
    actuator: Annotated[str, Field(min_length=1)]
    open: FiniteFloat
    closed: FiniteFloat
    mit: MITControl
    force: NormalForceControl | None = None

    @model_validator(mode="after")
    def validate_position_targets(self) -> "MITTorqueControl":
        """要求开合目标位于 MIT 范围之内。"""
        if not self.mit.p_min <= self.open <= self.mit.p_max:
            raise ValueError("open must lie within mit position limits")
        if not self.mit.p_min <= self.closed <= self.mit.p_max:
            raise ValueError("closed must lie within mit position limits")
        return self


ControlLayout: TypeAlias = Annotated[
    PositionControl | MITTorqueControl, Field(discriminator="mode")
]


class ModelSpec(_FrozenModel):
    """profile 所使用的 MJCF 源。"""

    path: Path


class Mount(_FrozenModel):
    """将夹爪附加到场景时所使用的位姿。"""

    pos: tuple[FiniteFloat, FiniteFloat, FiniteFloat]
    quat: tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]

    @model_validator(mode="after")
    def validate_quaternion(self) -> "Mount":
        """拒绝零四元数，同时将归一化留给 MuJoCo。"""
        if not any(self.quat):
            raise ValueError("quat must not be the zero quaternion")
        return self


class GripperProfile(_FrozenModel):
    """一个完整且带 schema 版本号的平行夹爪描述。"""

    schema_version: Literal[1]
    name: Annotated[str, Field(min_length=1)]
    model: ModelSpec
    control: ControlLayout
    mount: Mount
    tactile: Annotated[TactileLayout, Field(discriminator="mode")]

    @property
    def model_path(self) -> Path:
        """返回为兼容旧调用方而保留的、已解析的 MJCF 路径。"""
        return self.model.path

    @property
    def actuator(self) -> str:
        """返回为兼容旧调用方而保留的、已配置的执行器名称。"""
        return self.control.actuator

    @property
    def open_control(self) -> float:
        """返回为兼容旧调用方而保留的打开命令。"""
        return float(self.control.open)

    @property
    def closed_control(self) -> float:
        """返回为兼容旧调用方而保留的闭合命令。"""
        return float(self.control.closed)

    @property
    def control_mode(self) -> Literal["position", "mit_torque"]:
        """返回为兼容旧调用方而保留的控制判别符。"""
        return self.control.mode

    @property
    def mit(self) -> MITControl | None:
        """当此 profile 使用 MIT 力矩控制器时返回 MIT 限值。"""
        return self.control.mit if isinstance(self.control, MITTorqueControl) else None

    @property
    def normal_force(self) -> NormalForceControl | None:
        """返回为兼容旧调用方而保留的可选力跟踪参数。"""
        return self.control.force if isinstance(self.control, MITTorqueControl) else None

    @property
    def mount_pos(self) -> tuple[float, float, float]:
        """返回为兼容旧调用方而保留的安装位置。"""
        return tuple(float(value) for value in self.mount.pos)  # type: ignore[return-value]

    @property
    def mount_quat(self) -> tuple[float, float, float, float]:
        """返回为兼容旧调用方而保留的安装四元数。"""
        return tuple(float(value) for value in self.mount.quat)  # type: ignore[return-value]


def _load_yaml_mapping(profile_path: Path) -> dict[str, object]:
    """使用 PyYAML 的 safe 加载器加载恰好一个 YAML 映射。"""
    if profile_path.suffix.lower() not in {".yaml", ".yml"}:
        raise ProfileLoadError("profiles must use a .yaml or .yml extension; TOML is unsupported")
    try:
        with profile_path.open(encoding="utf-8") as stream:
            documents = list(yaml.safe_load_all(stream))
    except yaml.YAMLError as error:
        raise ProfileLoadError(f"invalid YAML in {profile_path}: {error}") from error
    if not documents or documents == [None]:
        raise ProfileLoadError(f"profile is empty: {profile_path}")
    if len(documents) != 1:
        raise ProfileLoadError(f"profile must contain exactly one YAML document: {profile_path}")
    raw = documents[0]
    if not isinstance(raw, dict):
        raise ProfileLoadError(f"profile root must be a mapping: {profile_path}")
    return raw


def _touch_grid_shape(model_path: Path, sensor_name: str) -> tuple[int, int]:
    """从其 MJCF 插件配置读取一个 touch-grid 传感器的 ``rows, cols``。"""
    try:
        root = ET.parse(model_path).getroot()
    except ET.ParseError as error:
        raise ProfileLoadError(f"invalid MJCF XML {model_path}: {error}") from error
    plugin = root.find(f"./sensor/plugin[@name='{sensor_name}']")
    if plugin is None:
        raise ProfileLoadError(f"touch_grid sensor {sensor_name!r} is missing from {model_path}")
    if plugin.get("plugin") != "mujoco.sensor.touch_grid":
        raise ProfileLoadError(f"sensor {sensor_name!r} is not a mujoco.sensor.touch_grid plugin")
    config = next((item for item in plugin.findall("config") if item.get("key") == "size"), None)
    if config is None or config.get("value") is None:
        raise ProfileLoadError(
            f"touch_grid sensor {sensor_name!r} is missing its size configuration"
        )
    try:
        cols, rows = (int(value) for value in config.get("value", "").split())
    except ValueError as error:
        raise ProfileLoadError(
            f"touch_grid sensor {sensor_name!r} has invalid size {config.get('value')!r}"
        ) from error
    if rows <= 0 or cols <= 0:
        raise ProfileLoadError(f"touch_grid sensor {sensor_name!r} has non-positive size")
    return rows, cols


def _resolve_touch_grid_shape(profile: GripperProfile) -> GripperProfile:
    """从 MJCF 填充并校验 touch-grid profile 的维度。"""
    tactile = profile.tactile
    if not isinstance(tactile, TouchGridTactileLayout):
        return profile
    left_shape = _touch_grid_shape(profile.model_path, tactile.left_sensor)
    right_shape = _touch_grid_shape(profile.model_path, tactile.right_sensor)
    if left_shape != right_shape:
        raise ProfileLoadError(
            "left and right touch_grid sensors must have matching dimensions, "
            f"got {left_shape} and {right_shape}"
        )
    resolved_tactile = tactile.model_copy(update={"rows": left_shape[0], "cols": left_shape[1]})
    return profile.model_copy(update={"tactile": resolved_tactile})


def validate_resolved_profile(profile: GripperProfile) -> GripperProfile:
    """重新执行完整 schema 与资源校验，返回不可变的最终 profile。

    该函数供组合配置和运行时控制器选择在应用覆盖后调用。它不会信任
    ``model_copy(update=...)`` 的中间结果，而是重新走 Pydantic 校验，并再次检查
    MJCF 文件与 ``touch_grid`` 布局。

    Args:
        profile: 已完成路径解析、可能包含运行时覆盖的 profile。

    Returns:
        经过完整领域与资源校验的最终 profile。

    Raises:
        FileNotFoundError: MJCF 文件不存在时抛出。
        ValidationError: profile 字段或跨字段约束无效时抛出。
        ProfileLoadError: ``touch_grid`` 资源不满足布局约束时抛出。
    """
    validated = GripperProfile.model_validate(profile.model_dump(mode="python"))
    model_path = validated.model_path.resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"gripper MJCF does not exist: {model_path}")
    if model_path != validated.model_path:
        validated = GripperProfile.model_validate(
            {
                **validated.model_dump(mode="python"),
                "model": {"path": model_path},
            }
        )
    return _resolve_touch_grid_shape(validated)


def load_profile(path: str | Path, *, repository_root: str | Path | None = None) -> GripperProfile:
    """加载、校验并解析路径的单个 YAML 夹爪 profile。

    相对模型路径始终相对于 profile 文件所在目录解析。``repository_root``
    仍作为已废弃的兼容参数保留；YAML profile 必须自包含，且不会推断仓库根目录。
    """
    if repository_root is not None:
        raise TypeError(
            "repository_root is no longer supported; use paths relative to the YAML profile"
        )
    profile_path = Path(path).resolve()
    raw = _load_yaml_mapping(profile_path)
    profile = GripperProfile.model_validate(raw)
    model_path = profile.model.path
    resolved_model_path = (
        model_path if model_path.is_absolute() else profile_path.parent / model_path
    )
    resolved_model_path = resolved_model_path.resolve()
    if not resolved_model_path.is_file():
        raise FileNotFoundError(f"gripper MJCF does not exist: {resolved_model_path}")
    resolved_profile = GripperProfile.model_validate(
        {
            **profile.model_dump(mode="python"),
            "model": {"path": resolved_model_path},
        }
    )
    return validate_resolved_profile(resolved_profile)


__all__ = [
    "AdrcControl",
    "ControlLayout",
    "DMAdmittanceControl",
    "DMForceSupervisorControl",
    "GripperProfile",
    "ContactStiffnessControl",
    "CrankSliderGeometry",
    "MITControl",
    "MITTorqueControl",
    "ModelSpec",
    "Mount",
    "NormalForceControl",
    "PositionControl",
    "ProfileLoadError",
    "STIFFNESS_ESTIMATOR_METHODS",
    "StiffnessEstimatorMethod",
    "TactileLayout",
    "TaxelTactileLayout",
    "TouchGridTactileLayout",
    "TorqueAdrcControl",
    "ValidationError",
    "load_profile",
    "validate_resolved_profile",
]

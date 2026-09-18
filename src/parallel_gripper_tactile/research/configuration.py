"""科研单次运行的组合选择、领域解析与兼容性校验。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError, model_validator
from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig

from ..config.profiles import (
    GripperProfile,
    MITTorqueControl,
    StiffnessEstimatorMethod,
    TorqueAdrcControl,
    load_profile,
    validate_resolved_profile,
)
from ..experiments.force_tracking import (
    ControllerVariant,
    ForceTrackingTask,
    configure_force_controller,
    validate_force_tracking_configuration,
)
from ..experiments.force_scheduling import ForceSchedulingTask, OracleForceSchedulerConfig
from ..experiments.friction_estimation import FrictionEstimationTask
from ..experiments.tangential_disturbance import (
    TangentialDisturbanceTask,
    validate_tangential_disturbance_configuration,
)
from ..tangential_disturbance import DisturbancePolicyConfig
from ..experiments.robotiq_discrete_force import (
    ControllerVariant as RobotiqControllerVariant,
    RobotiqDiscreteForceTask,
)
from ..scenes.custom import ObjectMaterial
from ..validation import validate_profile


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


class ResearchConfigurationError(ValueError):
    """组合配置未通过解析、资源或跨组件校验。"""


class _ResearchModel(BaseModel):
    """拒绝未知字段且禁止运行时修改的科研配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class DMPlatformSelection(_ResearchModel):
    """设备家族、运行后端及不可调机构参数。"""

    family: Literal["dm"]
    backend: Literal["simulation"]
    profile_name: str = Field(min_length=1)
    actuator: str = Field(min_length=1)
    open: float
    closed: float
    mit_limits: dict[str, object]
    mount: dict[str, object]
    geometry: dict[str, object]
    admittance_limits: dict[str, object]


class RobotiqPlatformSelection(_ResearchModel):
    """Robotiq 仿真机构与安装参数。"""

    family: Literal["robotiq"]
    backend: Literal["simulation"]
    profile_name: str = Field(min_length=1)
    actuator: str = Field(min_length=1)
    open: float
    mount: dict[str, object]


PlatformSelection: TypeAlias = Annotated[
    DMPlatformSelection | RobotiqPlatformSelection,
    Field(discriminator="family"),
]


class DMModelSelection(_ResearchModel):
    """MJCF 资源、触觉布局与测量噪声模型。"""

    family: Literal["dm"]
    name: str = Field(min_length=1)
    profile_name: str | None = Field(default=None, min_length=1)
    path: Path
    tactile: dict[str, object]
    sensor_taxel_normal_noise_std_n: tuple[float, float]
    sensor_taxel_shear_noise_std_n: tuple[float, float]


class RobotiqModelSelection(_ResearchModel):
    """Robotiq MJCF、模型相容命令上限与触觉布局。"""

    family: Literal["robotiq"]
    name: str = Field(min_length=1)
    profile_name: str | None = Field(default=None, min_length=1)
    path: Path
    closed: float
    tactile: dict[str, object]


ModelSelection: TypeAlias = Annotated[
    DMModelSelection | RobotiqModelSelection,
    Field(discriminator="family"),
]


class DMControllerSelection(_ResearchModel):
    """控制器变体及其可调控制参数。"""

    family: Literal["dm"]
    name: ControllerVariant
    mit_gains: dict[str, object]
    force: dict[str, object]
    stiffness_control: dict[str, object]
    torque_adrc: TorqueAdrcControl | None = None
    admittance: dict[str, object] | None = None

    @model_validator(mode="after")
    def validate_specific_parameters(self) -> "ControllerSelection":
        """拒绝算法专用参数出现在其他控制器配置中。"""
        uses_torque_adrc = self.name in {"adrc-torque", "adrc-torque-td"}
        if uses_torque_adrc != (self.torque_adrc is not None):
            raise ValueError(
                "torque_adrc parameters must be present exactly for torque ADRC controllers"
            )
        if (self.name == "admittance") != (self.admittance is not None):
            raise ValueError("admittance parameters must be present exactly for admittance")
        return self


class RobotiqControllerSelection(_ResearchModel):
    """Robotiq 离散命令控制器选择。"""

    family: Literal["robotiq"]
    name: RobotiqControllerVariant


ControllerSelection: TypeAlias = Annotated[
    DMControllerSelection | RobotiqControllerSelection,
    Field(discriminator="family"),
]


class EstimatorSelection(_ResearchModel):
    """在线接触刚度估计器选择及其估计参数。"""

    name: StiffnessEstimatorMethod | Literal["none"]
    stiffness: dict[str, object]


class SchedulerSelection(_ResearchModel):
    """目标力调度器选择；与外载任务、力跟踪控制器分离。"""

    family: Literal["none", "force", "disturbance"]
    name: Literal["none", "oracle", "adaptive", "dynamic_step"]
    path: Path | None = None
    definition: dict[str, object]

    @model_validator(mode="after")
    def validate_family_name(self) -> "SchedulerSelection":
        """要求调度器名称与家族一致，非空调度器必须给出来源路径。"""
        if (self.family == "none") != (self.name == "none"):
            raise ValueError("scheduler family and name are inconsistent")
        if self.family in {"force", "disturbance"} and self.path is None:
            raise ValueError("non-empty scheduler requires a source path")
        if self.family == "disturbance" and self.name != "dynamic_step":
            raise ValueError("disturbance scheduler must be named dynamic_step")
        return self


class TaskSelection(_ResearchModel):
    """任务家族、来源配置和可直接构造的任务片段。"""

    family: Literal[
        "force_tracking",
        "load",
        "friction_probe",
        "discrete_force",
        "tangential_disturbance",
    ]
    path: Path
    definition: dict[str, object]


class MaterialSelection(_ResearchModel):
    """接触材料 preset 选择。"""

    name: ObjectMaterial


class ExecutionConfig(_ResearchModel):
    """与科学条件无关的执行和存储选项。"""

    mode: Literal["plan", "run"] = "run"
    output_root: Path
    plot_mode: Literal["summary", "diagnostic"] = "summary"
    viewer: bool = False
    render_fps: float = Field(default=30.0, gt=0)
    realtime_factor: float = Field(default=1.0, gt=0)
    multiccd_enabled: bool = True
    trace_sample_period_s: float | None = Field(default=None, gt=0)
    trace_event_window_s: float = Field(default=0.2, ge=0)


class ExperimentSelection(_ResearchModel):
    """常用组合的名称；其组默认值只选择其他片段。"""

    name: str = Field(min_length=1)
    kind: Literal[
        "force_tracking",
        "force_scheduling",
        "friction_estimation",
        "discrete_force",
        "tangential_disturbance",
    ] = "force_tracking"
    profile_name: str | None = Field(default=None, min_length=1)


class ResearchRunConfig(_ResearchModel):
    """Hydra 完成组合和插值后进入领域层的单次运行 schema。"""

    schema_version: Literal[1]
    experiment: ExperimentSelection
    platform: PlatformSelection
    model: ModelSelection
    controller: ControllerSelection
    estimator: EstimatorSelection
    scheduler: SchedulerSelection
    task: TaskSelection
    material: MaterialSelection
    seed: int = Field(ge=0)
    execution: ExecutionConfig

    @model_validator(mode="after")
    def validate_component_compatibility(self) -> "ResearchRunConfig":
        """在读取模型前拒绝不支持的组件组合。"""
        if len({self.platform.family, self.model.family, self.controller.family}) != 1:
            raise ValueError("platform, model and controller families must match")
        expected_family = "robotiq" if self.experiment.kind == "discrete_force" else "dm"
        if self.platform.family != expected_family:
            raise ValueError("experiment kind is incompatible with the selected platform")
        expected_task_family = {
            "force_scheduling": "load",
            "friction_estimation": "friction_probe",
        }.get(self.experiment.kind, self.experiment.kind)
        if self.task.family != expected_task_family:
            raise ValueError("experiment kind and task family must match")
        if self.experiment.kind == "force_scheduling":
            if self.scheduler.family != "force":
                raise ValueError("force scheduling requires a force scheduler")
        elif self.experiment.kind == "tangential_disturbance":
            if self.scheduler.family != "disturbance":
                raise ValueError("tangential disturbance requires a disturbance scheduler")
        elif self.scheduler.family != "none":
            raise ValueError("non-empty scheduler is only valid for scheduling experiments")
        if self.experiment.kind == "tangential_disturbance":
            if self.execution.viewer:
                raise ValueError("tangential disturbance does not support viewer")
            if not self.execution.multiccd_enabled:
                raise ValueError("tangential disturbance currently requires multiccd_enabled=true")
            if self.execution.trace_sample_period_s is not None:
                raise ValueError(
                    "tangential disturbance records every physics step; leave trace_sample_period_s=null"
                )
            if not isinstance(
                self.controller, DMControllerSelection
            ) or self.controller.name not in {
                "pid-only",
                "pid-torque-ff",
            }:
                raise ValueError(
                    "tangential disturbance supports only pid-only or pid-torque-ff controllers"
                )
        if isinstance(self.controller, DMControllerSelection):
            if self.controller.name == "admittance":
                if self.estimator.name != "none":
                    raise ValueError("admittance requires estimator=none")
            elif self.estimator.name == "none" and self.controller.name != "pid-only":
                raise ValueError("estimator=none is supported only by pid-only or admittance")
        elif self.estimator.name != "none":
            raise ValueError("Robotiq discrete control requires estimator=none")
        return self


RunTask: TypeAlias = (
    ForceTrackingTask
    | ForceSchedulingTask
    | FrictionEstimationTask
    | RobotiqDiscreteForceTask
    | TangentialDisturbanceTask
)


@dataclass(frozen=True, slots=True)
class ResolvedResearchRun:
    """实际执行与保存共同使用的最终单次实验配置。"""

    selection: ResearchRunConfig
    profile_source: Path
    task_source: Path
    scheduler_source: Path | None
    profile: GripperProfile
    task: RunTask
    scheduler: OracleForceSchedulerConfig | UnifiedAdaptiveConfig | DisturbancePolicyConfig | None

    def effective_parameters(self) -> dict[str, object]:
        """返回可追溯且可 JSON 序列化的最终有效参数。"""
        return {
            "schema_version": 1,
            "selection": self.selection.model_dump(mode="json"),
            "profile": self.profile.model_dump(mode="json"),
            "task": self.task.model_dump(mode="json"),
            "scheduler": (
                None
                if self.scheduler is None
                else TypeAdapter(type(self.scheduler)).dump_python(self.scheduler, mode="json")
            ),
            "runtime": {
                "profile_path": "composed_profile",
                "profile_composition_source": str(self.profile_source),
                "task_path": str(self.task_source),
                "scheduler_path": (
                    None if self.scheduler_source is None else str(self.scheduler_source)
                ),
                "object_material": self.selection.material.name,
                "controller_variant": self.selection.controller.name,
                "stiffness_estimator_method": (
                    None
                    if self.selection.estimator.name == "none"
                    else self.selection.estimator.name
                ),
                "sensor_noise_seed": self.selection.seed,
                "multiccd_enabled": self.selection.execution.multiccd_enabled,
                "viewer": self.selection.execution.viewer,
                "plot_mode": self.selection.execution.plot_mode,
                "render_fps": self.selection.execution.render_fps,
                "realtime_factor": self.selection.execution.realtime_factor,
                "trace_sample_period_s": self.selection.execution.trace_sample_period_s,
                "trace_event_window_s": self.selection.execution.trace_event_window_s,
            },
        }


def _repository_path(path: Path, *, repository_root: Path) -> Path:
    """按明确的仓库根语义解析资源路径，而不读取当前工作目录。"""
    return path.resolve() if path.is_absolute() else (repository_root / path).resolve()


def _profile_from_fragments(
    selection: ResearchRunConfig, *, repository_root: Path
) -> GripperProfile:
    """从人工输入片段构造并校验完整 profile，绝不读取旧完整 profile。"""
    platform = selection.platform
    model = selection.model
    controller = selection.controller
    estimator = selection.estimator
    if isinstance(platform, RobotiqPlatformSelection):
        if not isinstance(model, RobotiqModelSelection) or not isinstance(
            controller, RobotiqControllerSelection
        ):
            raise ValueError("Robotiq platform requires Robotiq model and controller")
        profile = GripperProfile.model_validate(
            {
                "schema_version": 1,
                "name": (
                    selection.experiment.profile_name or model.profile_name or platform.profile_name
                ),
                "model": {"path": _repository_path(model.path, repository_root=repository_root)},
                "control": {
                    "mode": "position",
                    "actuator": platform.actuator,
                    "open": platform.open,
                    "closed": model.closed,
                },
                "mount": platform.mount,
                "tactile": model.tactile,
            }
        )
        return validate_resolved_profile(profile)
    if not isinstance(model, DMModelSelection) or not isinstance(controller, DMControllerSelection):
        raise ValueError("DM platform requires DM model and controller")
    stiffness = {**estimator.stiffness, **controller.stiffness_control}
    if estimator.name == "none":
        stiffness["enabled"] = False
    else:
        stiffness["enabled"] = True
        stiffness["method"] = estimator.name
    # 最终 profile 的共享数据模型和历史科学哈希仍保留位置式 PID 字段。非 PID
    # 控制器在 YAML 中不拥有这些参数；此处只补齐兼容值，实际模块启停仍由变体决定。
    legacy_position_pid: dict[str, object] = {
        "kp": 0.016,
        "ki": 0.2,
        "kd": 0.0,
        "max_position_adjustment": 0.15,
    }
    force: dict[str, object] = {
        **legacy_position_pid,
        **controller.force,
        "geometry": platform.geometry,
        "sensor_taxel_normal_noise_std_n": model.sensor_taxel_normal_noise_std_n,
        "sensor_taxel_shear_noise_std_n": model.sensor_taxel_shear_noise_std_n,
        "sensor_noise_seed": selection.seed,
        "stiffness": stiffness,
    }
    if controller.admittance is not None:
        admittance = {**controller.admittance, **platform.admittance_limits}
        if admittance.get("mit_torque_limit_nm") != platform.mit_limits.get("t_max"):
            raise ValueError("admittance MIT torque limit must equal platform mit.t_max")
        force["admittance"] = admittance
    profile = GripperProfile.model_validate(
        {
            "schema_version": 1,
            "name": (
                selection.experiment.profile_name or model.profile_name or platform.profile_name
            ),
            "model": {"path": _repository_path(model.path, repository_root=repository_root)},
            "control": {
                "mode": "mit_torque",
                "actuator": platform.actuator,
                "open": platform.open,
                "closed": platform.closed,
                "mit": {**platform.mit_limits, **controller.mit_gains},
                "force": force,
            },
            "mount": platform.mount,
            "tactile": model.tactile,
        }
    )
    return validate_resolved_profile(profile)


def _legacy_to_fragment_mapping(
    raw: dict[str, object], *, repository_root: Path
) -> dict[str, object]:
    """将迭代 1 旧入口临时适配为片段映射，供迁移期兼容入口使用。"""
    platform_raw = raw.get("platform")
    controller_raw = raw.get("controller")
    estimator_raw = raw.get("estimator")
    task_raw = raw.get("task")
    if not all(
        isinstance(value, dict) for value in (platform_raw, controller_raw, estimator_raw, task_raw)
    ):
        raise ValueError("legacy research configuration has incomplete components")
    profile_path = _repository_path(Path(platform_raw["profile"]), repository_root=repository_root)
    profile = load_profile(profile_path)
    if not isinstance(profile.control, MITTorqueControl) or profile.normal_force is None:
        raise ValueError("legacy force tracking requires MIT torque control with control.force")
    force = profile.normal_force
    if force.stiffness is None:
        raise ValueError("legacy force tracking requires control.force.stiffness")
    task_path = _repository_path(Path(task_raw["path"]), repository_root=repository_root)
    task = ForceTrackingTask.load(task_path)
    force_mapping = force.model_dump(mode="python")
    stiffness_mapping = force_mapping.pop("stiffness")
    geometry = force_mapping.pop("geometry")
    normal_noise = force_mapping.pop("sensor_taxel_normal_noise_std_n")
    shear_noise = force_mapping.pop("sensor_taxel_shear_noise_std_n")
    force_mapping.pop("sensor_noise_seed")
    admittance = force_mapping.pop("admittance")
    force_mapping.pop("adrc")
    force_mapping.pop("torque_adrc")
    force_mapping.pop("torque_feedback_gain")
    stiffness_control = {
        key: stiffness_mapping.pop(key)
        for key in (
            "torque_feedforward_gain",
            "position_limit_enabled",
            "position_limit_force_rate_n_s",
            "position_limit_stiffness_safety_factor",
        )
    }
    return {
        "schema_version": raw["schema_version"],
        "experiment": {"name": raw["experiment"], "profile_name": profile.name},
        "platform": {
            "family": platform_raw["family"],
            "backend": platform_raw["backend"],
            "profile_name": profile.name,
            "actuator": profile.control.actuator,
            "open": profile.control.open,
            "closed": profile.control.closed,
            "mit_limits": {
                key: value
                for key, value in profile.control.mit.model_dump(mode="python").items()
                if key not in {"kp", "kd"}
            },
            "mount": profile.mount.model_dump(mode="python"),
            "geometry": geometry,
            "admittance_limits": (
                {
                    key: admittance[key]
                    for key in (
                        "position_min_rad",
                        "position_max_rad",
                        "closing_direction",
                        "feedforward_torque_limit_nm",
                        "mit_torque_limit_nm",
                    )
                }
                if admittance is not None
                else {
                    "position_min_rad": 0.0,
                    "position_max_rad": 1.5707963267948966,
                    "closing_direction": 1,
                    "feedforward_torque_limit_nm": profile.control.mit.t_max,
                    "mit_torque_limit_nm": profile.control.mit.t_max,
                }
            ),
        },
        "model": {
            "family": platform_raw["family"],
            "name": "legacy",
            "profile_name": profile.name,
            "path": profile.model_path,
            "tactile": profile.tactile.model_dump(mode="python"),
            "sensor_taxel_normal_noise_std_n": normal_noise,
            "sensor_taxel_shear_noise_std_n": shear_noise,
        },
        "controller": {
            "family": controller_raw["family"],
            "name": controller_raw["name"],
            "mit_gains": {
                "kp": profile.control.mit.kp,
                "kd": profile.control.mit.kd,
            },
            "force": force_mapping,
            "stiffness_control": stiffness_control,
            "torque_adrc": controller_raw.get("torque_adrc"),
            "admittance": admittance,
        },
        "estimator": {"name": estimator_raw["name"], "stiffness": stiffness_mapping},
        "scheduler": {"family": "none", "name": "none", "definition": {}},
        "task": {
            "family": task_raw["family"],
            "path": task_path,
            "definition": task.model_dump(mode="python"),
        },
        "material": raw["material"],
        "seed": raw["seed"],
        "execution": raw["execution"],
    }


def resolve_research_run(
    raw: dict[str, object],
    *,
    repository_root: Path = REPOSITORY_ROOT,
) -> ResolvedResearchRun:
    """解析组合映射并完成 Pydantic、资源和跨组件校验。

    Args:
        raw: 已由 OmegaConf 完成插值解析的普通映射。
        repository_root: 相对资源路径的唯一基准。

    Returns:
        实际执行和有效参数快照共享的最终不可变配置。

    Raises:
        ResearchConfigurationError: 任一配置、路径或兼容性检查失败时抛出。
    """
    root = repository_root.resolve()
    try:
        if "model" not in raw:
            raw = _legacy_to_fragment_mapping(raw, repository_root=root)
        selection = ResearchRunConfig.model_validate(raw)
        task_source = _repository_path(selection.task.path, repository_root=root)
        task_model: dict[str, type[BaseModel]] = {
            "force_tracking": ForceTrackingTask,
            "load": ForceSchedulingTask,
            "friction_probe": FrictionEstimationTask,
            "discrete_force": RobotiqDiscreteForceTask,
            "tangential_disturbance": TangentialDisturbanceTask,
        }
        task = task_model[selection.task.family].model_validate(selection.task.definition)
        scheduler_source = (
            None
            if selection.scheduler.path is None
            else _repository_path(selection.scheduler.path, repository_root=root)
        )
        scheduler: (
            OracleForceSchedulerConfig | UnifiedAdaptiveConfig | DisturbancePolicyConfig | None
        )
        if selection.scheduler.name == "oracle":
            scheduler = OracleForceSchedulerConfig.model_validate(selection.scheduler.definition)
        elif selection.scheduler.name == "adaptive":
            scheduler = TypeAdapter(UnifiedAdaptiveConfig).validate_python(
                selection.scheduler.definition
            )
        elif selection.scheduler.name == "dynamic_step":
            scheduler = DisturbancePolicyConfig.model_validate(selection.scheduler.definition)
        else:
            scheduler = None
        if (
            isinstance(task, ForceSchedulingTask)
            and task.tactile_sampling is not None
            and not isinstance(scheduler, UnifiedAdaptiveConfig)
        ):
            raise ValueError("独立触觉采样仅支持 adaptive 目标力调度器")
        if isinstance(task, TangentialDisturbanceTask):
            task = task.model_copy(update={"object_material": selection.material.name})
        profile = _profile_from_fragments(selection, repository_root=root)
        if isinstance(selection.controller, DMControllerSelection):
            estimator = None if selection.estimator.name == "none" else selection.estimator.name
            profile = configure_force_controller(
                profile,
                variant=selection.controller.name,
                stiffness_estimator_method=estimator,
                sensor_noise_seed=selection.seed,
                torque_adrc_override=selection.controller.torque_adrc,
            )
        validate_profile(profile)
        trace_sample_period_s = selection.execution.trace_sample_period_s
        if trace_sample_period_s is None and not isinstance(task, TangentialDisturbanceTask):
            trace_sample_period_s = (
                task.control_period_s
                if selection.controller.name == "admittance"
                else 0.004
                if selection.controller.name in {"adrc-torque", "adrc-torque-td"}
                else 0.008
            )
        resolved_execution = ExecutionConfig.model_validate(
            {
                **selection.execution.model_dump(mode="python"),
                "trace_sample_period_s": trace_sample_period_s,
            }
        )
        if isinstance(task, ForceTrackingTask):
            validate_force_tracking_configuration(
                profile,
                task=task,
                object_material=selection.material.name,
                multiccd_enabled=selection.execution.multiccd_enabled,
                trace_sample_period_s=resolved_execution.trace_sample_period_s,
                trace_event_window_s=resolved_execution.trace_event_window_s,
                viewer=selection.execution.viewer,
                render_fps=resolved_execution.render_fps,
                realtime_factor=resolved_execution.realtime_factor,
            )
        elif isinstance(task, TangentialDisturbanceTask) and isinstance(
            scheduler, DisturbancePolicyConfig
        ):
            validate_tangential_disturbance_configuration(profile, task, scheduler)
    except (OSError, ValidationError, ValueError) as error:
        raise ResearchConfigurationError(str(error)) from error
    resolved_selection = ResearchRunConfig.model_validate(
        {
            **selection.model_dump(mode="python"),
            "model": {
                **selection.model.model_dump(mode="python"),
                "path": _repository_path(selection.model.path, repository_root=root),
            },
            "task": {
                **selection.task.model_dump(mode="python"),
                "path": task_source,
            },
            "scheduler": {
                **selection.scheduler.model_dump(mode="python"),
                "path": scheduler_source,
            },
            "execution": {
                **resolved_execution.model_dump(mode="python"),
                "output_root": _repository_path(
                    selection.execution.output_root, repository_root=root
                ),
            },
        }
    )
    return ResolvedResearchRun(
        selection=resolved_selection,
        profile_source=(
            root
            / "configs"
            / "platform"
            / ("dm_gripper" if selection.platform.family == "dm" else "robotiq_2f85")
            / "simulation.yaml"
        ),
        task_source=task_source,
        scheduler_source=scheduler_source,
        profile=profile,
        task=task,
        scheduler=scheduler,
    )


__all__ = [
    "ControllerSelection",
    "DMControllerSelection",
    "DMModelSelection",
    "DMPlatformSelection",
    "EstimatorSelection",
    "ExecutionConfig",
    "ExperimentSelection",
    "MaterialSelection",
    "ModelSelection",
    "PlatformSelection",
    "RobotiqControllerSelection",
    "RobotiqModelSelection",
    "RobotiqPlatformSelection",
    "REPOSITORY_ROOT",
    "ResearchConfigurationError",
    "ResearchRunConfig",
    "ResolvedResearchRun",
    "SchedulerSelection",
    "TaskSelection",
    "resolve_research_run",
]

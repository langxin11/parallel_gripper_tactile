"""科研单次运行的组合选择、领域解析与兼容性校验。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

from ..config.profiles import (
    GripperProfile,
    StiffnessEstimatorMethod,
    TorqueAdrcControl,
    load_profile,
)
from ..experiments.force_tracking import (
    ControllerVariant,
    ForceTrackingTask,
    configure_force_controller,
    validate_force_tracking_configuration,
)
from ..scenes.custom import ObjectMaterial
from ..validation import validate_profile


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


class ResearchConfigurationError(ValueError):
    """组合配置未通过解析、资源或跨组件校验。"""


class _ResearchModel(BaseModel):
    """拒绝未知字段且禁止运行时修改的科研配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class PlatformSelection(_ResearchModel):
    """设备家族与运行后端选择。"""

    family: Literal["dm"]
    backend: Literal["simulation"]
    profile: Path


class ControllerSelection(_ResearchModel):
    """控制器组件选择及其专属参数。"""

    family: Literal["dm"]
    name: ControllerVariant
    torque_adrc: TorqueAdrcControl | None = None

    @model_validator(mode="after")
    def validate_specific_parameters(self) -> "ControllerSelection":
        """拒绝算法专用参数出现在其他控制器配置中。"""
        uses_torque_adrc = self.name in {"adrc-torque", "adrc-torque-td"}
        if uses_torque_adrc != (self.torque_adrc is not None):
            raise ValueError(
                "torque_adrc parameters must be present exactly for torque ADRC controllers"
            )
        return self


class EstimatorSelection(_ResearchModel):
    """在线接触刚度估计器选择。"""

    name: StiffnessEstimatorMethod | Literal["none"]


class TaskSelection(_ResearchModel):
    """任务家族与领域配置文件选择。"""

    family: Literal["force_tracking"]
    path: Path


class MaterialSelection(_ResearchModel):
    """接触材料 preset 选择。"""

    name: ObjectMaterial


class ExecutionConfig(_ResearchModel):
    """与科学条件无关的执行和存储选项。"""

    mode: Literal["plan", "run"] = "run"
    output_root: Path
    viewer: bool = False
    render_fps: float = Field(default=30.0, gt=0)
    realtime_factor: float = Field(default=1.0, gt=0)
    multiccd_enabled: bool = True
    trace_sample_period_s: float | None = Field(default=None, gt=0)
    trace_event_window_s: float = Field(default=0.2, ge=0)


class ResearchRunConfig(_ResearchModel):
    """Hydra 完成组合和插值后进入领域层的单次运行 schema。"""

    schema_version: Literal[1]
    experiment: Literal["force_tracking"]
    platform: PlatformSelection
    controller: ControllerSelection
    estimator: EstimatorSelection
    task: TaskSelection
    material: MaterialSelection
    seed: int = Field(ge=0)
    execution: ExecutionConfig

    @model_validator(mode="after")
    def validate_component_compatibility(self) -> "ResearchRunConfig":
        """在读取模型前拒绝不支持的组件组合。"""
        if self.platform.family != self.controller.family:
            raise ValueError("platform and controller families must match")
        if self.controller.name == "admittance":
            if self.estimator.name != "none":
                raise ValueError("admittance requires estimator=none")
        elif self.estimator.name == "none" and self.controller.name != "pid-only":
            raise ValueError("estimator=none is supported only by pid-only or admittance")
        return self


@dataclass(frozen=True, slots=True)
class ResolvedResearchRun:
    """实际执行与保存共同使用的最终单次实验配置。"""

    selection: ResearchRunConfig
    profile_source: Path
    task_source: Path
    profile: GripperProfile
    task: ForceTrackingTask

    def effective_parameters(self) -> dict[str, object]:
        """返回可追溯且可 JSON 序列化的最终有效参数。"""
        return {
            "schema_version": 1,
            "selection": self.selection.model_dump(mode="json"),
            "profile": self.profile.model_dump(mode="json"),
            "task": self.task.model_dump(mode="json"),
            "runtime": {
                "profile_path": str(self.profile_source),
                "task_path": str(self.task_source),
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
                "render_fps": self.selection.execution.render_fps,
                "realtime_factor": self.selection.execution.realtime_factor,
                "trace_sample_period_s": self.selection.execution.trace_sample_period_s,
                "trace_event_window_s": self.selection.execution.trace_event_window_s,
            },
        }


def _repository_path(path: Path, *, repository_root: Path) -> Path:
    """按明确的仓库根语义解析资源路径，而不读取当前工作目录。"""
    return path.resolve() if path.is_absolute() else (repository_root / path).resolve()


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
        selection = ResearchRunConfig.model_validate(raw)
        profile_source = _repository_path(selection.platform.profile, repository_root=root)
        task_source = _repository_path(selection.task.path, repository_root=root)
        task = ForceTrackingTask.load(task_source)
        estimator = None if selection.estimator.name == "none" else selection.estimator.name
        profile = configure_force_controller(
            load_profile(profile_source),
            variant=selection.controller.name,
            stiffness_estimator_method=estimator,
            sensor_noise_seed=selection.seed,
            torque_adrc_override=selection.controller.torque_adrc,
        )
        validate_profile(profile)
        trace_sample_period_s = selection.execution.trace_sample_period_s
        if trace_sample_period_s is None:
            trace_sample_period_s = (
                task.control_period_s
                if selection.controller.name == "admittance"
                else 0.004
                if selection.controller.name in {"adrc-torque", "adrc-torque-td"}
                else 0.01
            )
        resolved_execution = ExecutionConfig.model_validate(
            {
                **selection.execution.model_dump(mode="python"),
                "trace_sample_period_s": trace_sample_period_s,
            }
        )
        validate_force_tracking_configuration(
            profile,
            task=task,
            object_material=selection.material.name,
            multiccd_enabled=selection.execution.multiccd_enabled,
            trace_sample_period_s=resolved_execution.trace_sample_period_s,
            trace_event_window_s=resolved_execution.trace_event_window_s,
            viewer=resolved_execution.viewer,
            render_fps=resolved_execution.render_fps,
            realtime_factor=resolved_execution.realtime_factor,
        )
    except (OSError, ValidationError, ValueError) as error:
        raise ResearchConfigurationError(str(error)) from error
    resolved_selection = ResearchRunConfig.model_validate(
        {
            **selection.model_dump(mode="python"),
            "platform": {
                **selection.platform.model_dump(mode="python"),
                "profile": profile_source,
            },
            "task": {
                **selection.task.model_dump(mode="python"),
                "path": task_source,
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
        profile_source=profile_source,
        task_source=task_source,
        profile=profile,
        task=task,
    )


__all__ = [
    "ControllerSelection",
    "EstimatorSelection",
    "ExecutionConfig",
    "MaterialSelection",
    "PlatformSelection",
    "REPOSITORY_ROOT",
    "ResearchConfigurationError",
    "ResearchRunConfig",
    "ResolvedResearchRun",
    "TaskSelection",
    "resolve_research_run",
]

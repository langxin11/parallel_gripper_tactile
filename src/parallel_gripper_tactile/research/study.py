"""正式研究方案的统一解析、计划与执行接口。"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Literal, Mapping

from pydantic import BaseModel, ConfigDict, ValidationError, model_validator

from ..config.profiles import TorqueAdrcControl, load_profile
from ..experiments.force_tracking import (
    ForceTrackingTask,
    configure_force_controller,
    validate_force_tracking_configuration,
)
from ..experiments.friction_estimation import FrictionEstimationTask
from ..studies.friction_estimation_local_slip import (
    FrictionEstimationLocalSlipStudyConfig,
    load_local_slip_study_config,
)
from ..studies.force_tracking_ablation import ForceTrackingAblationConfig, load_study_config
from ..studies.force_tracking_comparison import (
    ForceTrackingComparisonConfig,
    load_comparison_config,
)
from ..studies.force_tracking_torque_adrc_tuning import (
    ForceTrackingTorqueAdrcTuningConfig,
    TorqueAdrcTuningStageName,
    load_torque_adrc_tuning_config,
)
from ..studies.lifecycle import StudyPlan, assess_recovery, write_planned_study_manifest
from ..studies.protocols import force_tracking_ablation as ablation_protocol
from ..studies.protocols import friction_estimation_local_slip as friction_local_slip_protocol
from ..studies.protocols import (
    force_tracking_controller_comparison as comparison_protocol,
)
from ..studies.protocols import (
    force_tracking_torque_adrc_tuning as torque_tuning_protocol,
)
from .configuration import REPOSITORY_ROOT, ResearchConfigurationError


StudyKind = Literal[
    "force_tracking_controller_comparison",
    "force_tracking_ablation",
    "force_tracking_torque_adrc_tuning",
    "friction_estimation_local_slip",
]
SetupFailureStage = Literal["configuration", "preflight"]


class ResearchStudySetupError(ResearchConfigurationError):
    """携带配置或预检阶段归属的正式研究解析错误。"""

    def __init__(self, message: str, *, stage: SetupFailureStage) -> None:
        """保存失败阶段，供入口写入结构化 manifest。"""
        super().__init__(message)
        self.stage = stage


class _StudyModel(BaseModel):
    """拒绝未知字段且禁止修改的正式研究入口模型。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class StudySelection(_StudyModel):
    """正式研究方案及可选阶段谱系。"""

    kind: StudyKind
    config: Path
    stage: TorqueAdrcTuningStageName | None = None
    coarse_study_dir: Path | None = None

    @model_validator(mode="after")
    def validate_stage_fields(self) -> "StudySelection":
        """只允许 Torque ADRC tuning 使用 coarse/confirm 字段。"""
        if self.kind != "force_tracking_torque_adrc_tuning":
            if self.stage is not None or self.coarse_study_dir is not None:
                raise ValueError("stage fields are supported only by torque ADRC tuning")
            return self
        if self.stage is None:
            raise ValueError("torque ADRC tuning requires stage")
        if self.stage == "coarse" and self.coarse_study_dir is not None:
            raise ValueError("coarse stage forbids coarse_study_dir")
        if self.stage == "confirm" and self.coarse_study_dir is None:
            raise ValueError("confirm stage requires coarse_study_dir")
        return self


class StudyExecution(_StudyModel):
    """正式研究的计划、执行与只读恢复检测选项。"""

    mode: Literal["plan", "run"] = "plan"
    output_root: Path
    recovery_source: Path | None = None


class ResearchStudyConfig(_StudyModel):
    """Hydra 完成组合后进入领域层的正式研究 schema。"""

    schema_version: Literal[1]
    study: StudySelection
    execution: StudyExecution


StudyDomainConfig = (
    ForceTrackingComparisonConfig
    | ForceTrackingAblationConfig
    | ForceTrackingTorqueAdrcTuningConfig
    | FrictionEstimationLocalSlipStudyConfig
)


@dataclass(frozen=True, slots=True)
class ResolvedResearchStudy:
    """计划与执行共用的已校验领域配置和唯一有序计划。"""

    selection: ResearchStudyConfig
    config_source: Path
    domain_config: StudyDomainConfig
    plan: StudyPlan

    @property
    def conditions(self) -> tuple[dict[str, object], ...]:
        """返回兼容旧测试和人工审阅字段名的有序条件投影。"""
        rows: list[dict[str, object]] = []
        for condition in self.plan.conditions:
            parameters = dict(condition.parameters)
            row = {
                "condition_id": condition.condition_id,
                **parameters,
                "pair_key": condition.pair_key,
                "baseline_role": condition.baseline_role,
            }
            aliases = {
                "controller": parameters.get("controller_variant"),
                "task": parameters.get("task_path"),
                "material": parameters.get("object_material"),
                "seed": parameters.get("sensor_noise_seed"),
            }
            row.update({key: value for key, value in aliases.items() if value is not None})
            rows.append(row)
        return tuple(rows)

    def effective_configuration(self) -> dict[str, object]:
        """返回包括完整计划、预检与科学配置哈希的最终配置。"""
        return {
            "schema_version": 1,
            "selection": self.selection.model_dump(mode="json"),
            "source": str(self.config_source),
            "study": self.domain_config.model_dump(mode="json"),
            "study_plan": self.plan.model_dump(mode="json"),
        }


def _repository_path(path: Path, *, repository_root: Path) -> Path:
    """以仓库根为唯一基准解析 study 入口路径。"""
    return path.resolve() if path.is_absolute() else (repository_root / path).resolve()


def _validate_comparison(config: ForceTrackingComparisonConfig) -> None:
    """逐组件和科学维度预检控制器对比方案。"""
    base_profile = load_profile(config.profile)
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    seed = config.seeds.values()[0]
    for controller in config.controllers:
        profile = configure_force_controller(
            base_profile,
            variant=controller,
            stiffness_estimator_method=config.stiffness_estimator_method,
            sensor_noise_seed=seed,
        )
        for task in tasks.values():
            for material in config.materials:
                validate_force_tracking_configuration(profile, task=task, object_material=material)


def _validate_ablation(config: ForceTrackingAblationConfig) -> None:
    """逐组件和科学维度预检 PID 消融方案。"""
    base_profile = load_profile(config.profile)
    task = ForceTrackingTask.load(config.task)
    seed = config.seeds.values()[0]
    for controller in config.controllers:
        profile = configure_force_controller(
            base_profile,
            variant=controller,
            sensor_noise_seed=seed,
        )
        for material in config.materials:
            validate_force_tracking_configuration(profile, task=task, object_material=material)


def _validate_torque_tuning(
    config: ForceTrackingTorqueAdrcTuningConfig,
    plan: StudyPlan,
) -> None:
    """按实际计划中的候选、任务和材料预检 Torque ADRC 场景。"""
    base_profile = load_profile(config.profile)
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    validated: set[tuple[str, Path, str]] = set()
    for condition in plan.conditions:
        parameters = condition.parameters
        task_path = Path(str(parameters["task_path"]))
        material = str(parameters["object_material"])
        key = (str(parameters["candidate_id"]), task_path, material)
        if key in validated:
            continue
        control = TorqueAdrcControl.model_validate(parameters["torque_adrc"])
        profile = configure_force_controller(
            base_profile,
            variant="adrc-torque",
            stiffness_estimator_method=config.stiffness_estimator_method,
            sensor_noise_seed=int(parameters["sensor_noise_seed"]),
            torque_adrc_override=control,
        )
        validate_force_tracking_configuration(
            profile,
            task=tasks[task_path],
            object_material=material,
            trace_sample_period_s=0.004,
        )
        validated.add(key)


def _validate_local_slip(config: FrictionEstimationLocalSlipStudyConfig) -> None:
    """逐场景预检局部起滑方案的 profile 与任务文件。"""
    load_profile(config.profile)
    for scenario in config.scenarios:
        FrictionEstimationTask.load(scenario.task)


def _resolved_selection(
    selection: ResearchStudyConfig,
    *,
    repository_root: Path,
) -> ResearchStudyConfig:
    """把入口、输出和可选恢复路径统一解析为绝对路径。"""
    study = selection.study.model_dump(mode="python")
    study["config"] = _repository_path(selection.study.config, repository_root=repository_root)
    if selection.study.coarse_study_dir is not None:
        study["coarse_study_dir"] = _repository_path(
            selection.study.coarse_study_dir,
            repository_root=repository_root,
        )
    execution = selection.execution.model_dump(mode="python")
    execution["output_root"] = _repository_path(
        selection.execution.output_root,
        repository_root=repository_root,
    )
    if selection.execution.recovery_source is not None:
        execution["recovery_source"] = _repository_path(
            selection.execution.recovery_source,
            repository_root=repository_root,
        )
    return ResearchStudyConfig.model_validate(
        {
            "schema_version": selection.schema_version,
            "study": study,
            "execution": execution,
        }
    )


def resolve_research_study(
    raw: dict[str, object],
    *,
    repository_root: Path = REPOSITORY_ROOT,
) -> ResolvedResearchStudy:
    """加载正式方案，并在任何条件执行前生成和预检唯一计划。"""
    root = repository_root.resolve()
    try:
        selection = _resolved_selection(
            ResearchStudyConfig.model_validate(raw),
            repository_root=root,
        )
        source = selection.study.config
        if selection.study.kind == "force_tracking_controller_comparison":
            domain_config: StudyDomainConfig = load_comparison_config(source)
        elif selection.study.kind == "force_tracking_ablation":
            domain_config = load_study_config(source)
        elif selection.study.kind == "force_tracking_torque_adrc_tuning":
            domain_config = load_torque_adrc_tuning_config(source)
        elif selection.study.kind == "friction_estimation_local_slip":
            domain_config = load_local_slip_study_config(source)
        else:  # pragma: no cover - Literal 与 Pydantic 已阻止未知研究类型。
            raise ValueError(f"unsupported study kind: {selection.study.kind}")
    except (OSError, ValidationError, ValueError) as error:
        raise ResearchStudySetupError(str(error), stage="configuration") from error

    try:
        if isinstance(domain_config, ForceTrackingComparisonConfig):
            _validate_comparison(domain_config)
            plan = comparison_protocol.build_plan(domain_config)
        elif isinstance(domain_config, ForceTrackingAblationConfig):
            _validate_ablation(domain_config)
            plan = ablation_protocol.build_plan(domain_config)
        elif isinstance(domain_config, FrictionEstimationLocalSlipStudyConfig):
            _validate_local_slip(domain_config)
            plan = friction_local_slip_protocol.build_plan(domain_config)
        else:
            assert selection.study.stage is not None
            plan = torque_tuning_protocol.build_plan(
                domain_config,
                stage=selection.study.stage,
                coarse_study_dir=selection.study.coarse_study_dir,
            )
            _validate_torque_tuning(domain_config, plan)
        plan = plan.model_copy(
            update={
                "preflight": {
                    **plan.preflight,
                    "status": "passed",
                }
            }
        )
    except (OSError, ValidationError, ValueError) as error:
        raise ResearchStudySetupError(str(error), stage="preflight") from error
    return ResolvedResearchStudy(selection, source, domain_config, plan)


def _write_json(path: Path, value: object) -> Path:
    """以稳定格式写入 JSON 研究产物。"""
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def execute_research_study(
    resolved: ResolvedResearchStudy,
    *,
    hydra_output_directory: Path,
    provenance: Mapping[str, object],
) -> Path:
    """保存计划或把同一个已校验 ``StudyPlan`` 交给研究协议执行。"""
    output_directory = hydra_output_directory.resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    effective_path = _write_json(
        output_directory / "effective_study_configuration.json",
        resolved.effective_configuration(),
    )
    provenance_path = _write_json(
        output_directory / "composition_provenance.json", dict(provenance)
    )
    extra_artifacts = [effective_path, provenance_path]
    recovery_path = None
    if resolved.selection.execution.recovery_source is not None:
        recovery = assess_recovery(
            resolved.selection.execution.recovery_source,
            resolved.plan,
        )
        recovery_path = _write_json(
            output_directory / "recovery_assessment.json",
            recovery.model_dump(mode="json"),
        )
        extra_artifacts.append(recovery_path)

    lifecycle_fields: dict[str, object] = {
        "effective_configuration": effective_path.name,
        "composition_provenance": provenance_path.name,
        "provenance": dict(provenance),
        "recovery_assessment": None if recovery_path is None else recovery_path.name,
        "automatic_resume_enabled": False,
    }
    if resolved.selection.execution.mode == "plan":
        plan_path = _write_json(
            output_directory / "plan.json",
            {
                **resolved.plan.model_dump(mode="json"),
                "schema_version": 1,
                "artifact_kind": "study_plan",
                "validated": True,
                "compatibility": {"status": "passed"},
                "study": resolved.selection.study.kind,
                "condition_count": len(resolved.plan.conditions),
                "conditions": list(resolved.conditions),
                "output_directory": str(output_directory),
                "expected_run_directory_prefixes": [
                    {
                        "condition_id": condition.condition_id,
                        "directory_prefix": f"runs/{condition.condition_id}-",
                    }
                    for condition in resolved.plan.conditions
                ],
                "creates_simulation_data": False,
                "automatic_resume_enabled": False,
            },
        )
        extra_artifacts.append(plan_path)
        write_planned_study_manifest(
            resolved.plan,
            study_directory=output_directory,
            artifacts=extra_artifacts,
            manifest_fields={"schema_version": 1, **lifecycle_fields},
        )
        return output_directory

    common_arguments = {
        "config_source": resolved.config_source,
        "study_directory": output_directory,
        "study_plan": resolved.plan,
        "additional_artifacts": tuple(extra_artifacts),
        "lifecycle_manifest_fields": lifecycle_fields,
    }
    if isinstance(resolved.domain_config, ForceTrackingComparisonConfig):
        return comparison_protocol.run_study(resolved.domain_config, **common_arguments)
    if isinstance(resolved.domain_config, ForceTrackingAblationConfig):
        return ablation_protocol.run_study(resolved.domain_config, **common_arguments)
    if isinstance(resolved.domain_config, FrictionEstimationLocalSlipStudyConfig):
        return friction_local_slip_protocol.run_study(resolved.domain_config, **common_arguments)
    assert resolved.selection.study.stage is not None
    return torque_tuning_protocol.run_study(
        resolved.domain_config,
        stage=resolved.selection.study.stage,
        coarse_study_dir=resolved.selection.study.coarse_study_dir,
        **common_arguments,
    )


__all__ = [
    "ResearchStudyConfig",
    "ResearchStudySetupError",
    "ResolvedResearchStudy",
    "StudyExecution",
    "StudySelection",
    "execute_research_study",
    "resolve_research_study",
]

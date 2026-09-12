"""正式研究方案的统一解析、计划与执行接口。"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import json
from pathlib import Path
from types import ModuleType
from typing import Any, Literal, Mapping, get_args

from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

from ..config.profiles import GripperProfile, StiffnessRateControl, TorqueAdrcControl, load_profile
from ..experiments.force_tracking import (
    ForceTrackingTask,
    configure_force_controller,
    validate_force_tracking_configuration,
)
from ..experiments.friction_estimation import FrictionEstimationTask
from ..experiments.robotiq_discrete_force import RobotiqDiscreteForceTask
from ..scenes.robotiq import RobotiqObjectMaterial
from ..studies.dm_admittance_tuning import (
    DMAdmittanceTuningConfig,
)
from ..studies.friction_estimation_local_slip import (
    FrictionEstimationLocalSlipStudyConfig,
)
from ..studies.force_tracking_ablation import ForceTrackingAblationConfig
from ..studies.force_tracking_comparison import (
    ForceTrackingComparisonConfig,
)
from ..studies.force_tracking_diagnosis import (
    DiagnosisConfig,
    Phase as DiagnosisPhase,
)
from ..studies.force_tracking_stiffness_estimator_comparison import (
    ForceTrackingStiffnessEstimatorComparisonConfig,
)
from ..studies.force_tracking_stiffness_limit import ForceTrackingStiffnessLimitConfig
from ..studies.force_tracking_stiffness_rate_tuning import (
    ForceTrackingStiffnessRateTuningConfig,
)
from ..studies.force_tracking_torque_adrc_tuning import (
    ForceTrackingTorqueAdrcTuningConfig,
    TorqueAdrcTuningStageName,
)
from ..studies.stiffness_ground_truth_validation import (
    StiffnessGroundTruthValidationConfig,
)
from ..studies.lifecycle import (
    StudyPlan,
    StudyProgressCallback,
    assess_recovery,
    write_planned_study_manifest,
)
from ..studies.robotiq_discrete_force import (
    RobotiqDiscreteForceStudyConfig,
)
from ..studies.protocols import force_tracking_ablation as ablation_protocol
from ..studies.protocols import force_tracking_diagnosis as diagnosis_protocol
from ..studies.protocols import friction_estimation_local_slip as friction_local_slip_protocol
from ..studies.protocols import (
    force_tracking_controller_comparison as comparison_protocol,
)
from ..studies.protocols import (
    force_tracking_stiffness_estimator_comparison as stiffness_comparison_protocol,
)
from ..studies.protocols import (
    force_tracking_stiffness_limit as stiffness_limit_protocol,
)
from ..studies.protocols import (
    force_tracking_stiffness_rate_tuning as stiffness_rate_tuning_protocol,
)
from ..studies.protocols import (
    force_tracking_torque_adrc_tuning as torque_tuning_protocol,
)
from ..studies.protocols import dm_admittance_tuning as dm_admittance_tuning_protocol
from ..studies.protocols import (
    robotiq_discrete_force as robotiq_discrete_force_protocol,
)
from ..studies.protocols import (
    stiffness_ground_truth_validation as stiffness_ground_truth_protocol,
)
from .configuration import REPOSITORY_ROOT, ResearchConfigurationError
from .composition import compose_research_run


StudyKind = Literal[
    "force_tracking_controller_comparison",
    "force_tracking_ablation",
    "force_tracking_torque_adrc_tuning",
    "friction_estimation_local_slip",
    "force_tracking_stiffness_estimator_comparison",
    "force_tracking_stiffness_limit",
    "force_tracking_stiffness_rate_tuning",
    "force_tracking_stiffness_rate_confirmation",
    "stiffness_ground_truth_validation",
    "dm_admittance_tuning",
    "robotiq_discrete_force",
    "force_tracking_diagnosis",
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


class StudyRationale(_StudyModel):
    """研究问题、决策边界与已有排除证据。"""

    question: str
    decision: str
    inclusion_criteria: tuple[str, ...]
    task_rationale: str
    primary_metrics: tuple[str, ...]
    stop_conditions: tuple[str, ...]
    exclusions: dict[str, str]


class StudyProfileSelection(_StudyModel):
    """正式研究复用的命名 experiment 与 profile 组合覆盖。"""

    experiment: str
    overrides: tuple[str, ...] = ()


class StudySelection(_StudyModel):
    """正式研究方案及可选阶段谱系。"""

    kind: StudyKind
    source: Path
    profile: StudyProfileSelection
    rationale: StudyRationale
    definition: dict[str, object]
    stage: TorqueAdrcTuningStageName | None = None
    coarse_study_dir: Path | None = None
    phase: DiagnosisPhase | None = None

    @model_validator(mode="after")
    def validate_stage_fields(self) -> "StudySelection":
        """只允许 Torque ADRC tuning 使用 coarse/confirm 字段、诊断研究使用 phase 字段。"""
        if self.kind == "force_tracking_torque_adrc_tuning":
            if self.phase is not None:
                raise ValueError("phase field is supported only by force_tracking_diagnosis")
            if self.stage is None:
                raise ValueError("torque ADRC tuning requires stage")
            if self.stage == "coarse" and self.coarse_study_dir is not None:
                raise ValueError("coarse stage forbids coarse_study_dir")
            if self.stage == "confirm" and self.coarse_study_dir is None:
                raise ValueError("confirm stage requires coarse_study_dir")
            return self
        if self.stage is not None or self.coarse_study_dir is not None:
            raise ValueError("stage fields are supported only by torque ADRC tuning")
        if self.kind == "force_tracking_diagnosis":
            if self.phase is None:
                raise ValueError("force_tracking_diagnosis requires phase")
        elif self.phase is not None:
            raise ValueError("phase field is supported only by force_tracking_diagnosis")
        return self


class StudyExecution(_StudyModel):
    """正式研究的计划、执行与只读恢复检测选项。"""

    mode: Literal["plan", "run"] = "plan"
    output_root: Path
    recovery_source: Path | None = None
    workers: int = Field(default=1, ge=1)
    plot_mode: Literal["summary", "diagnostic"] = "summary"


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
    | ForceTrackingStiffnessEstimatorComparisonConfig
    | ForceTrackingStiffnessLimitConfig
    | ForceTrackingStiffnessRateTuningConfig
    | StiffnessGroundTruthValidationConfig
    | DMAdmittanceTuningConfig
    | RobotiqDiscreteForceStudyConfig
    | DiagnosisConfig
)


@dataclass(frozen=True, slots=True)
class ResolvedResearchStudy:
    """计划与执行共用的已校验领域配置和唯一有序计划。"""

    selection: ResearchStudyConfig
    config_source: Path
    profile: GripperProfile
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
            "profile": self.profile.model_dump(mode="json"),
            "study": self.domain_config.model_dump(mode="json"),
            "study_plan": self.plan.model_dump(mode="json"),
        }


def _repository_path(path: Path, *, repository_root: Path) -> Path:
    """以仓库根为唯一基准解析 study 入口路径。"""
    return path.resolve() if path.is_absolute() else (repository_root / path).resolve()


def _validate_profile_source_equivalence(
    profile: GripperProfile,
    source: Path,
) -> None:
    """确保组合对象除力跟踪公共状态机外与保留 profile 来源等价。"""
    legacy = load_profile(source)
    composed_values = profile.model_dump(mode="json")
    legacy_values = legacy.model_dump(mode="json")
    for values in (composed_values, legacy_values):
        control = values.get("control")
        if isinstance(control, dict):
            force = control.get("force")
            if isinstance(force, dict):
                force.pop("supervisor", None)
    if composed_values != legacy_values:
        raise ValueError(f"composed study profile differs from its legacy source: {source}")


def _validate_comparison(
    config: ForceTrackingComparisonConfig, base_profile: GripperProfile
) -> None:
    """逐组件和科学维度预检控制器对比方案。"""
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


def _validate_ablation(config: ForceTrackingAblationConfig, base_profile: GripperProfile) -> None:
    """逐组件和科学维度预检 PID 消融方案。"""
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


def _validate_stiffness_estimator_comparison(
    config: ForceTrackingStiffnessEstimatorComparisonConfig,
    base_profile: GripperProfile,
) -> None:
    """逐估计器、任务和材料预检刚度估计器对比方案。"""
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    seed = config.seeds.values()[0]
    for estimator in config.estimators:
        profile = configure_force_controller(
            base_profile,
            variant="pid-stiffness-ff",
            stiffness_estimator_method=estimator,
            sensor_noise_seed=seed,
        )
        for task in tasks.values():
            for material in config.materials:
                validate_force_tracking_configuration(profile, task=task, object_material=material)


def _validate_stiffness_ground_truth(
    config: StiffnessGroundTruthValidationConfig,
    profile: GripperProfile,
) -> None:
    """预检准静态任务和刚度估计器所需的 profile 组件。"""
    from ..experiments.stiffness_calibration import StiffnessCalibrationTask

    if profile.normal_force is None or profile.normal_force.stiffness is None:
        raise ValueError("等效接触刚度真值验证需要 control.force.stiffness。")
    if profile.normal_force.geometry is None:
        raise ValueError("等效接触刚度真值验证需要 control.force.geometry。")
    StiffnessCalibrationTask.load(config.task)


def _validate_stiffness_limit(
    config: ForceTrackingStiffnessLimitConfig,
    profile: GripperProfile,
) -> None:
    """逐三臂、任务和材料预检限幅实验。"""
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    seed = config.seeds.values()[0]
    for mode in config.modes:
        for material in config.materials:
            configured = stiffness_limit_protocol.configured_mode_profile(
                profile,
                mode=mode,
                material=material,
                seed=seed,
                oracle_values=config.oracle_stiffness_n_per_m,
                force_rate_limit_n_s=config.position_limit_force_rate_n_s,
            )
            for task in tasks.values():
                validate_force_tracking_configuration(
                    configured,
                    task=task,
                    object_material=material,
                    trace_sample_period_s=task.control_period_s,
                )


def _validate_stiffness_rate_tuning(
    config: ForceTrackingStiffnessRateTuningConfig,
    profile: GripperProfile,
) -> None:
    """逐候选、任务和材料预检速率控制调优。"""
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    seed = config.seeds.values()[0]
    controllers = [("pid-torque-ff", None)]
    controllers.extend(
        (
            "pid-stiffness-rate",
            StiffnessRateControl(
                kp_s_inv=candidate.kp_s_inv,
                ki_s_inv2=0.0,
                kd=0.0,
                max_force_rate_n_s=candidate.max_force_rate_n_s,
                max_joint_velocity_rad_s=config.max_joint_velocity_rad_s,
            ),
        )
        for candidate in config.candidates()
    )
    for controller, override in controllers:
        configured = configure_force_controller(
            profile,
            variant=controller,
            stiffness_estimator_method=config.stiffness_estimator_method,
            sensor_noise_seed=seed,
            stiffness_rate_override=override,
        )
        for task in tasks.values():
            for material in config.materials:
                validate_force_tracking_configuration(
                    configured,
                    task=task,
                    object_material=material,
                    trace_sample_period_s=task.control_period_s,
                )


def _validate_diagnosis(config: DiagnosisConfig, profile: GripperProfile) -> None:
    """预检诊断方案的 profile、任务与碰撞几何模型文件。"""
    if profile.normal_force is None:
        raise ValueError("力跟踪诊断需要 control.force。")
    ForceTrackingTask.load(config.task)
    for condition in config.collision_geometry_models:
        if not condition.model.is_file():
            raise ValueError(f"collision geometry model not found: {condition.model}")


def _validate_torque_tuning(
    config: ForceTrackingTorqueAdrcTuningConfig,
    plan: StudyPlan,
    base_profile: GripperProfile,
) -> None:
    """按实际计划中的候选、任务和材料预检 Torque ADRC 场景。"""
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


def _validate_local_slip(
    config: FrictionEstimationLocalSlipStudyConfig, profile: GripperProfile
) -> None:
    """逐场景预检局部起滑方案的 profile 与任务文件。"""
    if profile.normal_force is None:
        raise ValueError("局部起滑研究需要 control.force。")
    for scenario in config.scenarios:
        FrictionEstimationTask.load(scenario.task)


def _validate_dm_admittance_tuning(
    config: DMAdmittanceTuningConfig, profile: GripperProfile
) -> None:
    """预检导纳调参 profile 的导纳控制段和 Ramp 任务线性插值。"""
    if profile.normal_force is None or profile.normal_force.admittance is None:
        raise ValueError("导纳调参 profile 必须包含 control.force.admittance。")
    task = ForceTrackingTask.load(config.task)
    if task.reference.interpolation != "linear":
        raise ValueError("导纳调参仅接受线性 Ramp 力跟踪任务。")


def _validate_robotiq_discrete_force(
    config: RobotiqDiscreteForceStudyConfig, profile: GripperProfile
) -> None:
    """预检 Robotiq 离散力方案的 profile、任务文件与材料枚举。"""
    if profile.control_mode != "position":
        raise ValueError("Robotiq 离散力研究需要 Robotiq profile。")
    RobotiqDiscreteForceTask.load(config.task)
    valid_materials = get_args(RobotiqObjectMaterial)
    for material in config.materials:
        if material not in valid_materials:
            raise ValueError(f"unknown Robotiq object material: {material}")


def _resolved_selection(
    selection: ResearchStudyConfig,
    *,
    repository_root: Path,
) -> ResearchStudyConfig:
    """把入口、输出和可选恢复路径统一解析为绝对路径。"""
    study = selection.study.model_dump(mode="python")
    study["source"] = _repository_path(selection.study.source, repository_root=repository_root)
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


def _profile_options(selection: StudySelection, profile: GripperProfile) -> dict[str, object]:
    """标准研究向计划和执行传入同一冻结 profile。"""
    return {"resolved_profile": profile}


def _diagnosis_options(selection: StudySelection, profile: GripperProfile) -> dict[str, object]:
    """历史诊断保留专属 phase 与模型来源语义。"""
    return {"phase": selection.phase}


def _torque_options(selection: StudySelection, profile: GripperProfile) -> dict[str, object]:
    """Torque 调参在建计划和执行时都携带完整阶段谱系。"""
    return {
        "resolved_profile": profile,
        "stage": selection.stage,
        "coarse_study_dir": selection.coarse_study_dir,
    }


def _rate_options(selection: StudySelection, profile: GripperProfile) -> dict[str, object]:
    """共享速率协议显式区分调优与确认研究身份。"""
    return {"resolved_profile": profile, "study_kind": selection.kind}


@dataclass(frozen=True, slots=True)
class _StudyAdapter:
    """集中登记研究 schema、路径字段、预检及协议，矩阵仍由协议独占。"""

    model: type[BaseModel]
    protocol: ModuleType
    validate: Callable[[Any, GripperProfile], None] | None
    scalar_paths: tuple[str, ...] = ("profile", "output_root")
    sequence_paths: tuple[str, ...] = ("tasks",)
    nested_paths: tuple[tuple[str, str], ...] = ()
    options: Callable[[StudySelection, GripperProfile], dict[str, object]] = _profile_options
    validate_plan: Callable[[Any, StudyPlan, GripperProfile], None] | None = None

    def resolve(self, selection: StudySelection, repository_root: Path) -> StudyDomainConfig:
        """只规范此研究明确声明的路径字段，保持历史解析顺序和内容。"""
        config = self.model.model_validate(selection.definition)

        def path(value: Path) -> Path:
            """以本次解析指定的仓库根规范路径。"""
            return _repository_path(value, repository_root=repository_root)

        updates: dict[str, object] = {
            name: path(getattr(config, name)) for name in self.scalar_paths
        }
        updates.update(
            {
                name: tuple(path(value) for value in getattr(config, name))
                for name in self.sequence_paths
            }
        )
        for collection, field in self.nested_paths:
            updates[collection] = tuple(
                item.model_copy(update={field: path(getattr(item, field))})
                for item in getattr(config, collection)
            )
        return config.model_copy(update=updates)

    def build_plan(
        self, config: StudyDomainConfig, selection: StudySelection, profile: GripperProfile
    ) -> StudyPlan:
        """保留普通研究先预检、Torque 先校验谱系再按候选预检的顺序。"""
        if self.validate is not None:
            self.validate(config, profile)
        plan = self.protocol.build_plan(config, **self.options(selection, profile))
        if self.validate_plan is not None:
            self.validate_plan(config, plan, profile)
        return plan


_STUDY_ADAPTERS: dict[StudyKind, _StudyAdapter] = {
    "force_tracking_controller_comparison": _StudyAdapter(
        ForceTrackingComparisonConfig,
        comparison_protocol,
        _validate_comparison,
    ),
    "force_tracking_ablation": _StudyAdapter(
        ForceTrackingAblationConfig,
        ablation_protocol,
        _validate_ablation,
        scalar_paths=("profile", "task", "output_root"),
        sequence_paths=(),
    ),
    "force_tracking_torque_adrc_tuning": _StudyAdapter(
        ForceTrackingTorqueAdrcTuningConfig,
        torque_tuning_protocol,
        None,
        options=_torque_options,
        validate_plan=_validate_torque_tuning,
    ),
    "friction_estimation_local_slip": _StudyAdapter(
        FrictionEstimationLocalSlipStudyConfig,
        friction_local_slip_protocol,
        _validate_local_slip,
        sequence_paths=(),
        nested_paths=(("scenarios", "task"),),
    ),
    "force_tracking_stiffness_estimator_comparison": _StudyAdapter(
        ForceTrackingStiffnessEstimatorComparisonConfig,
        stiffness_comparison_protocol,
        _validate_stiffness_estimator_comparison,
    ),
    "stiffness_ground_truth_validation": _StudyAdapter(
        StiffnessGroundTruthValidationConfig,
        stiffness_ground_truth_protocol,
        _validate_stiffness_ground_truth,
        scalar_paths=("profile", "task", "output_root"),
        sequence_paths=(),
    ),
    "force_tracking_stiffness_limit": _StudyAdapter(
        ForceTrackingStiffnessLimitConfig,
        stiffness_limit_protocol,
        _validate_stiffness_limit,
    ),
    "force_tracking_stiffness_rate_tuning": _StudyAdapter(
        ForceTrackingStiffnessRateTuningConfig,
        stiffness_rate_tuning_protocol,
        _validate_stiffness_rate_tuning,
        options=_rate_options,
    ),
    "force_tracking_stiffness_rate_confirmation": _StudyAdapter(
        ForceTrackingStiffnessRateTuningConfig,
        stiffness_rate_tuning_protocol,
        _validate_stiffness_rate_tuning,
        options=_rate_options,
    ),
    "dm_admittance_tuning": _StudyAdapter(
        DMAdmittanceTuningConfig,
        dm_admittance_tuning_protocol,
        _validate_dm_admittance_tuning,
        scalar_paths=("task", "output_root"),
        sequence_paths=(),
    ),
    "robotiq_discrete_force": _StudyAdapter(
        RobotiqDiscreteForceStudyConfig,
        robotiq_discrete_force_protocol,
        _validate_robotiq_discrete_force,
        scalar_paths=("profile", "task", "output_root"),
        sequence_paths=(),
    ),
    "force_tracking_diagnosis": _StudyAdapter(
        DiagnosisConfig,
        diagnosis_protocol,
        _validate_diagnosis,
        scalar_paths=("profile", "task", "output_root"),
        sequence_paths=(),
        nested_paths=(("collision_geometry_models", "model"),),
        options=_diagnosis_options,
    ),
}


def _study_adapter(kind: StudyKind) -> _StudyAdapter:
    """拒绝未登记的研究，不把未知类型默认为 Torque 调参。"""
    try:
        return _STUDY_ADAPTERS[kind]
    except KeyError as error:
        raise ValueError(f"unsupported study kind: {kind}") from error


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
        source = selection.study.source
        adapter = _study_adapter(selection.study.kind)
        domain_config = adapter.resolve(selection.study, root)
        composed = compose_research_run(
            experiment=selection.study.profile.experiment,
            overrides=selection.study.profile.overrides,
        )
        profile = composed.profile
        # 活跃研究以 experiment 组合结果为唯一 profile；仅归档诊断仍校验历史来源。
        legacy_profile_source = getattr(domain_config, "profile", None)
        if selection.study.kind == "force_tracking_diagnosis" and isinstance(
            legacy_profile_source, Path
        ):
            _validate_profile_source_equivalence(profile, legacy_profile_source)
    except (OSError, ValidationError, ValueError) as error:
        raise ResearchStudySetupError(str(error), stage="configuration") from error

    try:
        plan = adapter.build_plan(domain_config, selection.study, profile)
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
    return ResolvedResearchStudy(selection, source, profile, domain_config, plan)


def _write_json(path: Path, value: object) -> Path:
    """以稳定格式写入 JSON 研究产物。"""
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def execute_research_study(
    resolved: ResolvedResearchStudy,
    *,
    hydra_output_directory: Path,
    provenance: Mapping[str, object],
    on_progress: StudyProgressCallback | None = None,
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
        "plot_mode": resolved.selection.execution.plot_mode,
        "effective_configuration": effective_path.name,
        "composition_provenance": provenance_path.name,
        "provenance": dict(provenance),
        "recovery_assessment": None if recovery_path is None else recovery_path.name,
        "automatic_resume_enabled": False,
        "condition_executor": {
            "backend": "process" if resolved.selection.execution.workers > 1 else "serial",
            "workers": resolved.selection.execution.workers,
        },
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
        "workers": resolved.selection.execution.workers,
        "plot_mode": resolved.selection.execution.plot_mode,
    }
    if on_progress is not None:
        common_arguments["on_progress"] = on_progress
    adapter = _study_adapter(resolved.selection.study.kind)
    return adapter.protocol.run_study(
        resolved.domain_config,
        **adapter.options(resolved.selection.study, resolved.profile),
        **common_arguments,
    )


__all__ = [
    "ResearchStudyConfig",
    "ResearchStudySetupError",
    "ResolvedResearchStudy",
    "StudyExecution",
    "StudyProfileSelection",
    "StudySelection",
    "execute_research_study",
    "resolve_research_study",
]

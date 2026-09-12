"""正式研究共享的条件执行、状态账本与科学配置哈希。"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from concurrent.futures import Future, ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field as dataclass_field
from hashlib import sha256
import json
import math
import multiprocessing
from pathlib import Path
from typing import Any, Literal
import warnings

from pydantic import BaseModel, ConfigDict, Field, model_validator


ConditionStatus = Literal["completed", "scientific_failure", "execution_error"]
StudyState = Literal["planned", "running", "partial", "completed", "failed"]
FailureStage = Literal[
    "entry",
    "configuration",
    "preflight",
    "condition_execution",
    "aggregation",
    "rendering",
    "manifest",
]


@dataclass(frozen=True, slots=True)
class StudyProgress:
    """仅供父进程观察的研究进度，不参与科学配置和持久化格式。"""

    study_directory: Path
    total: int
    finished: int
    scientific_failures: int
    execution_errors: int
    state: StudyState


StudyProgressCallback = Callable[[StudyProgress], None]


class _LifecycleModel(BaseModel):
    """拒绝未知字段且禁止修改的生命周期模型基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class StudyCondition(_LifecycleModel):
    """由研究专属层生成、由公共执行器消费的一个有序条件。"""

    condition_id: str = Field(min_length=1, pattern=r"^[A-Za-z0-9_.-]+$")
    parameters: dict[str, Any]
    pair_key: str | None = None
    baseline_role: str | None = None


class LifecycleFailure(_LifecycleModel):
    """一次明确归属到生命周期阶段的异常。"""

    stage: FailureStage
    error_type: str
    message: str


class ConditionOutcome(_LifecycleModel):
    """正常完成、科学验收失败或执行异常的统一条件结果。"""

    condition_id: str
    status: ConditionStatus
    parameters: dict[str, Any]
    pair_key: str | None = None
    baseline_role: str | None = None
    run_directory: str | None = None
    metrics: dict[str, Any] | None = None
    failure: LifecycleFailure | None = None

    @model_validator(mode="after")
    def validate_status_payload(self) -> "ConditionOutcome":
        """约束异常和正常结果各自允许携带的字段。"""
        if self.status == "execution_error":
            if self.failure is None or self.metrics is not None:
                raise ValueError("execution_error requires failure and forbids metrics")
        elif self.failure is not None or self.metrics is None or self.run_directory is None:
            raise ValueError("completed results require metrics and run_directory")
        return self


class StudyPlan(_LifecycleModel):
    """计划和执行共享的已校验有序研究方案。"""

    study_kind: str
    stage: str | None = None
    study_definition_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    scientific_configuration_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    conditions: tuple[StudyCondition, ...]
    seeds: tuple[int, ...]
    preflight: dict[str, Any]

    @model_validator(mode="after")
    def require_unique_conditions(self) -> "StudyPlan":
        """拒绝重复条件标识，保持恢复和配对语义唯一。"""
        identifiers = [condition.condition_id for condition in self.conditions]
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("study condition_id values must be unique")
        return self


class StudyManifest(_LifecycleModel):
    """不含历史兼容投影的公共生命周期 manifest。"""

    lifecycle_schema_version: Literal[1] = 1
    state: StudyState
    study_kind: str
    stage: str | None = None
    study_definition_sha256: str
    scientific_configuration_sha256: str
    planned_condition_count: int = Field(ge=0)
    seeds: tuple[int, ...]
    preflight: dict[str, Any]
    completed_condition_count: int = Field(ge=0)
    passed_condition_count: int = Field(ge=0)
    failed_condition_count: int = Field(ge=0)
    scientific_failure_count: int = Field(ge=0)
    execution_error_count: int = Field(ge=0)
    planned_conditions: tuple[dict[str, Any], ...]
    condition_results: tuple[dict[str, Any], ...]
    lifecycle_failures: tuple[LifecycleFailure, ...] = ()
    artifacts: tuple[str, ...] = ()
    artifact_sha256: dict[str, str] = Field(default_factory=dict)


class RecoveryAssessment(_LifecycleModel):
    """既有 study 与当前计划是否可安全恢复的只读判断。"""

    source_directory: Path
    manifest_found: bool
    compatible: bool
    automatic_resume_enabled: Literal[False] = False
    successful_condition_ids: tuple[str, ...] = ()
    retryable_condition_ids: tuple[str, ...] = ()
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class ConditionExecution:
    """研究专属执行钩子返回的正常结构化结果。"""

    row: dict[str, object]
    run_directory: str
    passed: bool
    metadata: Mapping[str, object] = dataclass_field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class StudyPostprocessResult:
    """聚合持久化钩子返回的产物和兼容 manifest 字段。"""

    artifacts: tuple[Path, ...]
    manifest_fields: Mapping[str, object]
    render_payload: object = None


class StudyLifecycleError(RuntimeError):
    """聚合、绘图或 manifest 阶段失败，但生命周期账本已保留。"""

    def __init__(self, failure: LifecycleFailure) -> None:
        """保存结构化失败并提供简洁异常消息。"""
        super().__init__(f"{failure.stage}: {failure.message}")
        self.failure = failure


def require_matching_study_plan(expected: StudyPlan, supplied: StudyPlan) -> StudyPlan:
    """拒绝把其他研究、阶段、哈希或条件集合的计划交给当前 protocol。"""
    comparable_fields = (
        "study_kind",
        "stage",
        "study_definition_sha256",
        "scientific_configuration_sha256",
        "conditions",
        "seeds",
    )
    mismatches = [
        field for field in comparable_fields if getattr(expected, field) != getattr(supplied, field)
    ]
    if mismatches:
        raise ValueError("supplied StudyPlan does not match protocol: " + ", ".join(mismatches))
    return supplied


def _canonical_value(value: object, *, repository_root: Path | None) -> object:
    """把科学配置转换成稳定 JSON 值，同时规范仓库内绝对路径。"""
    if isinstance(value, BaseModel):
        return _canonical_value(value.model_dump(mode="python"), repository_root=repository_root)
    if isinstance(value, Path):
        resolved = (
            (repository_root / value).resolve()
            if repository_root is not None and not value.is_absolute()
            else value.resolve()
        )
        if repository_root is not None:
            try:
                return resolved.relative_to(repository_root.resolve()).as_posix()
            except ValueError:
                pass
        return resolved.as_posix()
    if isinstance(value, Mapping):
        return {
            str(key): _canonical_value(item, repository_root=repository_root)
            for key, item in sorted(value.items(), key=lambda item: str(item[0]))
        }
    if isinstance(value, (list, tuple)):
        return [_canonical_value(item, repository_root=repository_root) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("scientific configuration cannot contain non-finite numbers")
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(f"unsupported scientific configuration value: {type(value).__name__}")


def scientific_configuration_hash(
    value: object,
    *,
    repository_root: Path | None = None,
) -> str:
    """计算不受字典顺序和仓库内绝对路径影响的科学配置哈希。"""
    canonical = _canonical_value(value, repository_root=repository_root)
    payload = json.dumps(
        canonical,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return sha256(payload).hexdigest()


def file_sha256(path: Path) -> str:
    """返回资源文件内容摘要，供科学配置和产物谱系共同使用。"""
    return sha256(path.read_bytes()).hexdigest()


def model_configuration_sha256(
    model: BaseModel,
    *,
    repository_root: Path | None = None,
) -> str:
    """摘要已校验模型，而不是摘要可能已被组合结果取代的来源文件。"""
    return scientific_configuration_hash(
        model.model_dump(mode="json"),
        repository_root=repository_root,
    )


def _write_json(path: Path, value: object) -> Path:
    """原子写入稳定 JSON，避免中断留下半个 manifest。"""
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def _artifact_records(
    study_directory: Path, paths: Sequence[Path]
) -> tuple[tuple[str, ...], dict[str, str]]:
    """登记实际存在且位于 study 目录内的去重文件及内容哈希。"""
    root = study_directory.resolve()
    relative_paths: list[str] = []
    hashes: dict[str, str] = {}
    for path in paths:
        resolved = path.resolve()
        try:
            relative = resolved.relative_to(root).as_posix()
        except ValueError as error:
            raise ValueError(f"study artifact escapes output directory: {path}") from error
        if not resolved.is_file():
            raise FileNotFoundError(f"study artifact does not exist: {resolved}")
        if relative in hashes:
            continue
        relative_paths.append(relative)
        hashes[relative] = sha256(resolved.read_bytes()).hexdigest()
    return tuple(relative_paths), hashes


def _legacy_failed_condition(outcome: ConditionOutcome) -> dict[str, object]:
    """把执行异常投影为旧版 failed_conditions 行。"""
    assert outcome.failure is not None
    return {
        "condition_id": outcome.condition_id,
        **outcome.parameters,
        "status": "failed",
        "error_type": outcome.failure.error_type,
        "error": outcome.failure.message,
    }


def execution_failure_rows(outcomes: Sequence[ConditionOutcome]) -> list[dict[str, object]]:
    """返回执行异常的旧版行投影，供兼容 summary 使用。"""
    return [
        _legacy_failed_condition(outcome)
        for outcome in outcomes
        if outcome.status == "execution_error"
    ]


def _manifest_mapping(
    plan: StudyPlan,
    *,
    state: StudyState,
    outcomes: Sequence[ConditionOutcome],
    failures: Sequence[LifecycleFailure],
    artifacts: Sequence[Path],
    study_directory: Path,
    legacy_fields: Mapping[str, object],
) -> dict[str, object]:
    """生成公共 manifest，并在顶层保留旧字段投影。"""
    artifact_names, artifact_hashes = _artifact_records(study_directory, artifacts)
    completed = [outcome for outcome in outcomes if outcome.status != "execution_error"]
    passed = [outcome for outcome in outcomes if outcome.status == "completed"]
    scientific_failures = [
        outcome for outcome in outcomes if outcome.status == "scientific_failure"
    ]
    execution_errors = [outcome for outcome in outcomes if outcome.status == "execution_error"]
    core = StudyManifest(
        state=state,
        study_kind=plan.study_kind,
        stage=plan.stage,
        study_definition_sha256=plan.study_definition_sha256,
        scientific_configuration_sha256=plan.scientific_configuration_sha256,
        planned_condition_count=len(plan.conditions),
        seeds=plan.seeds,
        preflight=plan.preflight,
        completed_condition_count=len(completed),
        passed_condition_count=len(passed),
        failed_condition_count=len(scientific_failures) + len(execution_errors),
        scientific_failure_count=len(scientific_failures),
        execution_error_count=len(execution_errors),
        planned_conditions=tuple(
            condition.model_dump(mode="json") for condition in plan.conditions
        ),
        condition_results=tuple(outcome.model_dump(mode="json") for outcome in outcomes),
        lifecycle_failures=tuple(failures),
        artifacts=artifact_names,
        artifact_sha256=artifact_hashes,
    ).model_dump(mode="json")
    conflicting = [
        key for key, value in legacy_fields.items() if key in core and core[key] != value
    ]
    if conflicting:
        raise ValueError(
            "legacy manifest fields conflict with lifecycle fields: "
            + ", ".join(sorted(conflicting))
        )
    core.update(legacy_fields)
    core.update(
        {
            "runs": [
                outcome.run_directory for outcome in completed if outcome.run_directory is not None
            ],
            "failed_runs": [
                outcome.run_directory
                for outcome in scientific_failures
                if outcome.run_directory is not None
            ],
            "failed_conditions": [
                _legacy_failed_condition(outcome) for outcome in execution_errors
            ],
            "artifacts": list(artifact_names),
        }
    )
    return core


def _failure(stage: FailureStage, error: Exception) -> LifecycleFailure:
    """把 Python 异常转换成可序列化生命周期失败。"""
    return LifecycleFailure(stage=stage, error_type=type(error).__name__, message=str(error))


def _condition_outcome(
    condition: StudyCondition,
    execution: ConditionExecution | None,
    error: Exception | None,
) -> ConditionOutcome:
    """把一个条件的返回值或异常转换成稳定的公共结果。"""
    if error is not None:
        return ConditionOutcome(
            condition_id=condition.condition_id,
            status="execution_error",
            parameters=condition.parameters,
            pair_key=condition.pair_key,
            baseline_role=condition.baseline_role,
            failure=_failure("condition_execution", error),
        )
    assert execution is not None
    return ConditionOutcome(
        condition_id=condition.condition_id,
        status="completed" if execution.passed else "scientific_failure",
        parameters=condition.parameters,
        pair_key=condition.pair_key,
        baseline_role=condition.baseline_role,
        run_directory=execution.run_directory,
        metrics=execution.row,
    )


def execute_study_lifecycle(
    plan: StudyPlan,
    *,
    study_directory: Path,
    execute_condition: Callable[[StudyCondition], ConditionExecution],
    aggregate_and_persist: Callable[
        [list[dict[str, object]], tuple[ConditionOutcome, ...], Path], StudyPostprocessResult
    ],
    render: Callable[[list[dict[str, object]], object, Path], Sequence[Path]],
    initial_artifacts: Sequence[Path] = (),
    legacy_manifest_fields: Mapping[str, object] | None = None,
    workers: int = 1,
    record_condition_execution: Callable[[ConditionExecution], None] | None = None,
    on_progress: StudyProgressCallback | None = None,
) -> Path:
    """执行公共 study 生命周期，并在所有可恢复边界保存 manifest。

    条件级并行仅覆盖仿真执行；manifest、聚合和绘图始终由父进程串行负责。
    """
    if workers < 1:
        raise ValueError("workers must be at least 1")
    directory = study_directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    manifest_path = directory / "study_manifest.json"
    legacy = dict(legacy_manifest_fields or {})
    artifacts = list(initial_artifacts)
    outcomes_by_id: dict[str, ConditionOutcome] = {}
    executions_by_id: dict[str, ConditionExecution] = {}
    lifecycle_failures: list[LifecycleFailure] = []

    def ordered_outcomes() -> list[ConditionOutcome]:
        """按 StudyPlan 顺序投影已完成结果，屏蔽并行完成顺序。"""
        return [
            outcomes_by_id[condition.condition_id]
            for condition in plan.conditions
            if condition.condition_id in outcomes_by_id
        ]

    def notify_progress(state: StudyState) -> None:
        """在 manifest 落盘后通知观察者，回调异常不参与研究失败分类。"""
        if on_progress is None:
            return
        outcomes = ordered_outcomes()
        progress = StudyProgress(
            study_directory=directory,
            total=len(plan.conditions),
            finished=len(outcomes),
            scientific_failures=sum(item.status == "scientific_failure" for item in outcomes),
            execution_errors=sum(item.status == "execution_error" for item in outcomes),
            state=state,
        )
        try:
            on_progress(progress)
        except Exception as error:
            # 即使调用者将警告配置成异常，观察失败也不能中断研究。
            with warnings.catch_warnings():
                warnings.simplefilter("always", RuntimeWarning)
                warnings.warn(
                    f"研究进度回调失败：{type(error).__name__}：{error}",
                    RuntimeWarning,
                    stacklevel=2,
                )

    def persist_progress() -> None:
        """只由父进程原子更新运行中 manifest。"""
        _write_json(
            manifest_path,
            _manifest_mapping(
                plan,
                state="running",
                outcomes=ordered_outcomes(),
                failures=lifecycle_failures,
                artifacts=artifacts,
                study_directory=directory,
                legacy_fields=legacy,
            ),
        )
        notify_progress("running")

    _write_json(
        manifest_path,
        _manifest_mapping(
            plan,
            state="running",
            outcomes=(),
            failures=lifecycle_failures,
            artifacts=artifacts,
            study_directory=directory,
            legacy_fields=legacy,
        ),
    )

    notify_progress("running")

    if workers == 1:
        for condition in plan.conditions:
            execution: ConditionExecution | None = None
            error: Exception | None = None
            try:
                execution = execute_condition(condition)
            except Exception as caught:
                error = caught
            outcomes_by_id[condition.condition_id] = _condition_outcome(condition, execution, error)
            if execution is not None:
                executions_by_id[condition.condition_id] = execution
            persist_progress()
    else:
        context = multiprocessing.get_context("spawn")
        with ProcessPoolExecutor(max_workers=workers, mp_context=context) as executor:
            futures: dict[Future[ConditionExecution], StudyCondition] = {
                executor.submit(execute_condition, condition): condition
                for condition in plan.conditions
            }
            for future in as_completed(futures):
                condition = futures[future]
                execution = None
                error = None
                try:
                    execution = future.result()
                except Exception as caught:
                    error = caught
                outcomes_by_id[condition.condition_id] = _condition_outcome(
                    condition, execution, error
                )
                if execution is not None:
                    executions_by_id[condition.condition_id] = execution
                persist_progress()

    outcomes = ordered_outcomes()
    if record_condition_execution is not None:
        for condition in plan.conditions:
            execution = executions_by_id.get(condition.condition_id)
            if execution is not None:
                record_condition_execution(execution)

    rows = [
        dict(outcome.metrics)
        for outcome in outcomes
        if outcome.status != "execution_error" and outcome.metrics is not None
    ]
    result_path = directory / "condition_results.json"
    _write_json(result_path, [outcome.model_dump(mode="json") for outcome in outcomes])
    artifacts.append(result_path)
    execution_errors = [outcome for outcome in outcomes if outcome.status == "execution_error"]
    if execution_errors:
        failure_path = directory / "failed_conditions.json"
        _write_json(
            failure_path,
            [_legacy_failed_condition(outcome) for outcome in execution_errors],
        )
        artifacts.append(failure_path)

    try:
        postprocess = aggregate_and_persist(rows, tuple(outcomes), directory)
        _artifact_records(directory, (*artifacts, *postprocess.artifacts))
        artifacts.extend(postprocess.artifacts)
        legacy.update(postprocess.manifest_fields)
    except Exception as error:
        failure = _failure("aggregation", error)
        lifecycle_failures.append(failure)
        failure_path = directory / "lifecycle_failure.json"
        _write_json(failure_path, failure.model_dump(mode="json"))
        artifacts.append(failure_path)
        _write_json(
            manifest_path,
            _manifest_mapping(
                plan,
                state="failed",
                outcomes=outcomes,
                failures=lifecycle_failures,
                artifacts=artifacts,
                study_directory=directory,
                legacy_fields=legacy,
            ),
        )
        notify_progress("failed")
        raise StudyLifecycleError(failure) from error

    try:
        rendered_artifacts = tuple(render(rows, postprocess.render_payload, directory))
        _artifact_records(directory, (*artifacts, *rendered_artifacts))
        artifacts.extend(rendered_artifacts)
    except Exception as error:
        failure = _failure("rendering", error)
        lifecycle_failures.append(failure)
        failure_path = directory / "lifecycle_failure.json"
        _write_json(failure_path, failure.model_dump(mode="json"))
        artifacts.append(failure_path)
        _write_json(
            manifest_path,
            _manifest_mapping(
                plan,
                state="failed",
                outcomes=outcomes,
                failures=lifecycle_failures,
                artifacts=artifacts,
                study_directory=directory,
                legacy_fields=legacy,
            ),
        )
        notify_progress("failed")
        raise StudyLifecycleError(failure) from error

    completed_count = sum(outcome.status != "execution_error" for outcome in outcomes)
    execution_error_count = sum(outcome.status == "execution_error" for outcome in outcomes)
    state: StudyState = (
        "failed" if completed_count == 0 else "partial" if execution_error_count else "completed"
    )
    _write_json(
        manifest_path,
        _manifest_mapping(
            plan,
            state=state,
            outcomes=outcomes,
            failures=lifecycle_failures,
            artifacts=artifacts,
            study_directory=directory,
            legacy_fields=legacy,
        ),
    )
    notify_progress(state)
    return directory


def write_planned_study_manifest(
    plan: StudyPlan,
    *,
    study_directory: Path,
    artifacts: Sequence[Path] = (),
    manifest_fields: Mapping[str, object] | None = None,
) -> Path:
    """为只读计划写入与执行器同构、但不含运行结果的 manifest。"""
    directory = study_directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    manifest_path = directory / "study_manifest.json"
    _write_json(
        manifest_path,
        _manifest_mapping(
            plan,
            state="planned",
            outcomes=(),
            failures=(),
            artifacts=artifacts,
            study_directory=directory,
            legacy_fields=dict(manifest_fields or {}),
        ),
    )
    return manifest_path


def record_study_setup_failure(
    *,
    study_directory: Path,
    stage: Literal["entry", "configuration", "preflight"],
    error: Exception,
    study_kind: str | None = None,
    artifacts: Sequence[Path] = (),
) -> Path:
    """在尚不能形成有效计划时保存入口、配置或预检失败。"""
    directory = study_directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    failure = _failure(stage, error)
    failure_path = directory / "setup_failure.json"
    _write_json(failure_path, failure.model_dump(mode="json"))
    artifact_names, artifact_hashes = _artifact_records(
        directory,
        (*artifacts, failure_path),
    )
    manifest = {
        "lifecycle_schema_version": 1,
        "state": "failed",
        "study_kind": study_kind,
        "stage": None,
        "study_definition_sha256": None,
        "scientific_configuration_sha256": None,
        "planned_condition_count": 0,
        "seeds": [],
        "preflight": {"status": "failed", "stage": stage},
        "completed_condition_count": 0,
        "passed_condition_count": 0,
        "failed_condition_count": 0,
        "scientific_failure_count": 0,
        "execution_error_count": 0,
        "planned_conditions": [],
        "condition_results": [],
        "lifecycle_failures": [failure.model_dump(mode="json")],
        "runs": [],
        "failed_runs": [],
        "failed_conditions": [],
        "artifacts": list(artifact_names),
        "artifact_sha256": artifact_hashes,
    }
    return _write_json(directory / "study_manifest.json", manifest)


def assess_recovery(source_directory: Path, plan: StudyPlan) -> RecoveryAssessment:
    """只读检查既有 manifest；本阶段明确不自动跳过任何条件。"""
    directory = source_directory.resolve()
    manifest_path = directory / "study_manifest.json"
    if not manifest_path.is_file():
        return RecoveryAssessment(
            source_directory=directory,
            manifest_found=False,
            compatible=False,
            retryable_condition_ids=tuple(condition.condition_id for condition in plan.conditions),
            reasons=("study_manifest.json 不存在",),
        )
    try:
        raw = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return RecoveryAssessment(
            source_directory=directory,
            manifest_found=True,
            compatible=False,
            retryable_condition_ids=tuple(condition.condition_id for condition in plan.conditions),
            reasons=(f"manifest 无法读取：{error}",),
        )
    reasons: list[str] = []
    for field, expected in (
        ("study_kind", plan.study_kind),
        ("stage", plan.stage),
        ("scientific_configuration_sha256", plan.scientific_configuration_sha256),
    ):
        if raw.get(field) != expected:
            reasons.append(f"{field} 不匹配")
    results = raw.get("condition_results")
    if not isinstance(results, list):
        reasons.append("condition_results 缺失或无效")
        results = []
    expected_ids = {condition.condition_id for condition in plan.conditions}
    successful = tuple(
        str(item["condition_id"])
        for item in results
        if isinstance(item, dict)
        and item.get("status") == "completed"
        and item.get("condition_id") in expected_ids
    )
    retryable = tuple(
        condition.condition_id
        for condition in plan.conditions
        if condition.condition_id not in set(successful)
    )
    return RecoveryAssessment(
        source_directory=directory,
        manifest_found=True,
        compatible=not reasons,
        successful_condition_ids=successful,
        retryable_condition_ids=retryable,
        reasons=tuple(reasons) or ("配置兼容；自动恢复尚未启用",),
    )


__all__ = [
    "ConditionExecution",
    "ConditionOutcome",
    "LifecycleFailure",
    "RecoveryAssessment",
    "StudyCondition",
    "StudyLifecycleError",
    "StudyManifest",
    "StudyPlan",
    "StudyPostprocessResult",
    "StudyProgress",
    "StudyProgressCallback",
    "assess_recovery",
    "execute_study_lifecycle",
    "execution_failure_rows",
    "file_sha256",
    "model_configuration_sha256",
    "record_study_setup_failure",
    "require_matching_study_plan",
    "scientific_configuration_hash",
    "write_planned_study_manifest",
]

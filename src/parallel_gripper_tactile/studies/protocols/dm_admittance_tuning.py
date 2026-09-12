"""运行 DMgripper 二阶导纳 Ramp 参数调优。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
from datetime import UTC, datetime
from functools import partial
import json
import math
from pathlib import Path
from statistics import fmean
from uuid import uuid4

import yaml

from parallel_gripper_tactile.config.profiles import GripperProfile, validate_resolved_profile
from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.aggregation import (
    aggregate_records,
    bool_sum,
    count,
    finite_mean,
    first,
    key,
    minimum,
    plain_mean,
    threshold_count,
)
from parallel_gripper_tactile.studies.dm_admittance_tuning import (
    DMAdmittanceCandidate,
    DMAdmittanceTuningConfig,
)
from parallel_gripper_tactile.studies.lifecycle import (
    ConditionExecution,
    ConditionOutcome,
    StudyCondition,
    StudyPlan,
    StudyProgressCallback,
    StudyPostprocessResult,
    execute_study_lifecycle,
    execution_failure_rows,
    file_sha256,
    require_matching_study_plan,
    scientific_configuration_hash,
)
from parallel_gripper_tactile.studies.tabular import (
    read_trace_rows,
    write_resolved_config,
    write_rows_csv_and_parquet,
)


METRICS = (
    "rmse_n",
    "mae_n",
    "peak_abs_error_n",
    "final_error_n",
    "raw_rmse_n",
    "raw_mae_n",
    "raw_peak_abs_error_n",
)
_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


def aggregate_rows(
    rows: list[dict[str, object]], *, minimum_ratio: float
) -> list[dict[str, object]]:
    """按候选聚合稳定性、跟踪完整度和误差指标。"""
    return aggregate_records(
        rows,
        keys=("candidate_id",),
        columns=(
            key("candidate_id"),
            first("mass_kg"),
            first("damping_ns_m"),
            first("stiffness_n_m"),
            first("filter_cutoff_hz"),
            first("velocity_limit_rad_s"),
            first("approach_velocity_rad_s"),
            first("contact_stable_time_s"),
            first("contact_transition_time_s"),
            first("approach_feedforward_force_n"),
            count("runs"),
            bool_sum("passed", "stable_runs"),
            threshold_count(
                "force_tracking_ratio", "complete_force_tracking_runs", at_least=minimum_ratio
            ),
            plain_mean("force_tracking_ratio"),
            minimum("force_tracking_ratio", "force_tracking_ratio_min"),
            *(finite_mean(metric) for metric in METRICS),
        ),
    )


def rank_candidates(aggregates: list[dict[str, object]]) -> list[dict[str, object]]:
    """优先稳定和完整跟踪，再按原始物理力峰值与 RMSE 排序。"""
    ranking: list[dict[str, object]] = []
    for aggregate in aggregates:
        stable = int(aggregate["stable_runs"]) == int(aggregate["runs"])
        complete = int(aggregate["complete_force_tracking_runs"]) == int(aggregate["runs"])
        ranking.append(
            {
                **aggregate,
                "stable": str(stable).lower(),
                "complete_force_tracking": str(complete).lower(),
            }
        )
    ranking.sort(
        key=lambda row: (
            row["stable"] != "true",
            row["complete_force_tracking"] != "true",
            math.inf
            if row["raw_peak_abs_error_n_mean"] is None
            else float(row["raw_peak_abs_error_n_mean"]),
            math.inf if row["raw_rmse_n_mean"] is None else float(row["raw_rmse_n_mean"]),
            math.inf if row["rmse_n_mean"] is None else float(row["rmse_n_mean"]),
            str(row["candidate_id"]),
        )
    )
    for index, row in enumerate(ranking, start=1):
        row["rank"] = index
    return ranking


def json_compatible(value: object) -> object:
    """把非有限浮点数转换为标准 JSON 的 ``null``。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_compatible(item) for item in value]
    return value


def _trace_diagnostics(run_directory: Path, *, ignore_initial_s: float) -> dict[str, float]:
    """从 trace 计算状态占比和未滤波物理力误差。"""
    rows = read_trace_rows(run_directory)
    tracking_rows = [row for row in rows if row.get("phase") == "track_reference"]
    if not tracking_rows:
        return {
            "force_tracking_ratio": 0.0,
            "raw_rmse_n": math.inf,
            "raw_mae_n": math.inf,
            "raw_peak_abs_error_n": math.inf,
        }
    active = sum(row.get("control_state") == "force_tracking" for row in tracking_rows)
    metric_rows = [
        row for row in tracking_rows if float(row["tracking_time_s"]) >= ignore_initial_s
    ]
    if not metric_rows:
        return {
            "force_tracking_ratio": active / len(tracking_rows),
            "raw_rmse_n": math.inf,
            "raw_mae_n": math.inf,
            "raw_peak_abs_error_n": math.inf,
        }

    def physical_force(row: dict[str, object]) -> float:
        """返回物理步进后左右触觉侧力的平均值。"""
        return 0.5 * (
            float(row["left_taxel_normal_force_n"]) + float(row["right_taxel_normal_force_n"])
        )

    all_errors = [
        float(row["target_normal_force_n"]) - physical_force(row) for row in tracking_rows
    ]
    errors = [float(row["target_normal_force_n"]) - physical_force(row) for row in metric_rows]
    return {
        "force_tracking_ratio": active / len(tracking_rows),
        "raw_rmse_n": math.sqrt(fmean(error * error for error in errors)),
        "raw_mae_n": fmean(abs(error) for error in errors),
        "raw_peak_abs_error_n": max(abs(error) for error in all_errors),
    }


def _candidate_profile(
    profile: GripperProfile,
    candidate: DMAdmittanceCandidate,
) -> GripperProfile:
    """从组合后的冻结 profile 派生当前候选，不生成第二份输入文件。"""
    candidate_profile = profile.model_dump(mode="json")
    force = candidate_profile["control"].get("force")
    if not isinstance(force, dict) or not isinstance(force.get("admittance"), dict):
        raise ValueError("导纳调参 profile 缺少 control.force.admittance。")
    admittance = candidate_profile["control"]["force"]["admittance"]
    admittance.update(
        {
            "mass_kg": candidate.mass_kg,
            "damping_ns_m": candidate.damping_ns_m,
            "stiffness_n_m": candidate.stiffness_n_m,
            "approach_velocity_rad_s": candidate.approach_velocity_rad_s,
            "contact_stable_time_s": candidate.contact_stable_time_s,
            "contact_transition_time_s": candidate.contact_transition_time_s,
            "approach_feedforward_force_n": candidate.approach_feedforward_force_n,
        }
    )
    candidate_profile["control"]["force"]["filter_cutoff_hz"] = candidate.filter_cutoff_hz
    admittance["velocity_limit_rad_s"] = candidate.velocity_limit_rad_s
    return validate_resolved_profile(GripperProfile.model_validate(candidate_profile))


def _create_study_directory(config: DMAdmittanceTuningConfig) -> Path:
    """为一次调参运行创建独占 study 输出目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def build_plan(
    config: DMAdmittanceTuningConfig,
    *,
    resolved_profile: GripperProfile,
) -> StudyPlan:
    """从权威 domain config 生成导纳调参的唯一有序计划。"""
    conditions = tuple(
        StudyCondition(
            condition_id=f"{candidate.identifier}-{material}-seed{seed:03d}",
            parameters={
                "candidate_id": candidate.identifier,
                "object_material": material,
                "sensor_noise_seed": seed,
                "candidate": candidate.model_dump(mode="python"),
            },
            pair_key=f"{material}:seed{seed:03d}",
            baseline_role=None,
        )
        for candidate, material, seed in config.conditions()
    )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "dm_admittance_tuning.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": scientific_configuration_hash(
                resolved_profile.model_dump(mode="python"),
                repository_root=_REPOSITORY_ROOT,
            ),
            "task_sha256": file_sha256(config.task),
        },
        # 旧脚本逐条件执行 kwargs 的关键隔离项：控制器固定为二阶导纳变体。
        "scientific_runtime": {"controller_variant": "admittance"},
        "ranking": (
            "stable==runs desc, complete_force_tracking==runs desc, "
            "raw_peak_abs_error_n_mean asc, raw_rmse_n_mean asc, "
            "rmse_n_mean asc, candidate_id asc"
        ),
    }
    definition_hash = scientific_configuration_hash(definition, repository_root=_REPOSITORY_ROOT)
    plan_hash = scientific_configuration_hash(
        {
            "study_definition_sha256": definition_hash,
            "stage": None,
            "conditions": [condition.model_dump(mode="python") for condition in conditions],
        },
        repository_root=_REPOSITORY_ROOT,
    )
    return StudyPlan(
        study_kind="dm_admittance_tuning",
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=conditions,
        seeds=tuple(config.seeds.values()),
        preflight={"status": "pending", "checks": ["profile", "task", "admittance"]},
    )


def _execute_condition(
    condition: StudyCondition,
    *,
    config: DMAdmittanceTuningConfig,
    profile: GripperProfile,
    task: ForceTrackingTask,
    study_dir: Path,
) -> ConditionExecution:
    """在独立进程中执行一个导纳候选条件。"""
    parameters = condition.parameters
    candidate = DMAdmittanceCandidate.model_validate(parameters["candidate"])
    material = str(parameters["object_material"])
    seed = int(parameters["sensor_noise_seed"])
    candidate_profile = _candidate_profile(profile, candidate)
    run, result = execute_force_tracking(
        profile="composed_profile",
        resolved_profile=candidate_profile,
        task_path=config.task,
        tracking_task=task,
        output_root=study_dir / "runs" / candidate.identifier,
        run_prefix=(f"{candidate.identifier}-{config.task.stem}-{material}-seed{seed:03d}"),
        object_material=material,  # type: ignore[arg-type]
        controller_variant="admittance",
        sensor_noise_seed=seed,
    )
    row = {
        "candidate_id": candidate.identifier,
        "mass_kg": candidate.mass_kg,
        "damping_ns_m": candidate.damping_ns_m,
        "stiffness_n_m": candidate.stiffness_n_m,
        "filter_cutoff_hz": candidate.filter_cutoff_hz,
        "velocity_limit_rad_s": candidate.velocity_limit_rad_s,
        "approach_velocity_rad_s": candidate.approach_velocity_rad_s,
        "contact_stable_time_s": candidate.contact_stable_time_s,
        "contact_transition_time_s": candidate.contact_transition_time_s,
        "approach_feedforward_force_n": candidate.approach_feedforward_force_n,
        "task_name": task.name,
        "task_path": str(config.task),
        "object_material": material,
        "sensor_noise_seed": seed,
        "passed": result.passed,
        "run_directory": str(run.path.relative_to(study_dir)),
        **asdict(result),
        **_trace_diagnostics(run.path, ignore_initial_s=task.metrics.ignore_initial_s),
    }
    return ConditionExecution(
        row=row,
        run_directory=str(row["run_directory"]),
        passed=result.passed,
    )


def run_study(
    config: DMAdmittanceTuningConfig,
    *,
    resolved_profile: GripperProfile,
    config_source: Path | None = None,
    study_directory: Path | None = None,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
    workers: int = 1,
    on_progress: StudyProgressCallback | None = None,
) -> Path:
    """通过公共生命周期执行导纳 Ramp 调参，并返回 study 父目录。"""
    profile = validate_resolved_profile(resolved_profile)
    task = ForceTrackingTask.load(config.task)
    study_dir = (
        _create_study_directory(config) if study_directory is None else study_directory.resolve()
    )
    study_dir.mkdir(parents=True, exist_ok=True)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        (study_dir / "study.yaml").write_text(
            yaml.safe_dump(config.model_dump(mode="json"), sort_keys=False), encoding="utf-8"
        )
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)
    expected_plan = build_plan(config, resolved_profile=profile)
    plan = (
        expected_plan
        if study_plan is None
        else require_matching_study_plan(expected_plan, study_plan)
    )

    execute = partial(
        _execute_condition,
        config=config,
        profile=profile,
        task=task,
        study_dir=study_dir,
    )

    def aggregate_and_persist(
        rows: list[dict[str, object]],
        outcomes: tuple[ConditionOutcome, ...],
        directory: Path,
    ) -> StudyPostprocessResult:
        aggregates = (
            aggregate_rows(rows, minimum_ratio=config.minimum_force_tracking_ratio) if rows else []
        )
        ranking = rank_candidates(aggregates) if aggregates else []
        artifacts: list[Path] = []
        if rows:
            artifacts.extend(write_rows_csv_and_parquet(directory / "summary.csv", rows))
        if aggregates:
            artifacts.extend(write_rows_csv_and_parquet(directory / "aggregate.csv", aggregates))
        if ranking:
            artifacts.extend(
                write_rows_csv_and_parquet(directory / "candidate_ranking.csv", ranking)
            )
        summary_json = directory / "summary.json"
        summary_json.write_text(
            json.dumps(
                json_compatible(
                    {
                        "runs": rows,
                        "aggregates": aggregates,
                        "ranking": ranking,
                        "failures": execution_failure_rows(outcomes),
                    }
                ),
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        artifacts.append(summary_json)
        return StudyPostprocessResult(tuple(artifacts), {}, aggregates)

    def render(rows: list[dict[str, object]], payload: object, directory: Path) -> tuple[Path, ...]:
        # 导纳调参研究历史上不产出图，迁移后同样只保留表格与 JSON 产物。
        return ()

    manifest_fields: dict[str, object] = {
        "schema_version": 1,
        "name": config.name,
        "config": "study.yaml",
        "resolved_config": str(resolved_config.relative_to(study_dir)),
    }
    manifest_fields.update(lifecycle_manifest_fields or {})
    return execute_study_lifecycle(
        plan,
        study_directory=study_dir,
        execute_condition=execute,
        aggregate_and_persist=aggregate_and_persist,
        render=render,
        initial_artifacts=(study_dir / "study.yaml", resolved_config, *additional_artifacts),
        legacy_manifest_fields=manifest_fields,
        workers=workers,
        on_progress=on_progress,
    )

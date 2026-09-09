"""并行运行 DMgripper 二阶导纳 Ramp 参数调优。"""

import argparse
from concurrent.futures import Future, ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Any, Callable, Iterable
from uuid import uuid4

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
from parallel_gripper_tactile.profiles import GripperProfile, load_profile
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.dm_admittance_tuning import (
    DMAdmittanceCandidate,
    DMAdmittanceTuningConfig,
    load_dm_admittance_tuning_config,
)
from parallel_gripper_tactile.studies.tabular import (
    read_trace_rows,
    write_resolved_config,
    write_rows_csv_and_parquet,
)
import yaml


METRICS = (
    "rmse_n",
    "mae_n",
    "peak_abs_error_n",
    "final_error_n",
    "raw_rmse_n",
    "raw_mae_n",
    "raw_peak_abs_error_n",
)


@dataclass(frozen=True, slots=True)
class RunCondition:
    """一个可由独立进程执行的候选、材料和 seed 组合。"""

    candidate: DMAdmittanceCandidate
    profile_path: Path
    task_path: Path
    output_root: Path
    study_directory: Path
    object_material: str
    sensor_noise_seed: int

    @property
    def key(self) -> tuple[str, str, int]:
        """返回用于重建配置顺序的稳定条件键。"""
        return (self.candidate.identifier, self.object_material, self.sensor_noise_seed)

    @property
    def run_prefix(self) -> str:
        """返回单次运行的稳定名称前缀。"""
        return (
            f"{self.candidate.identifier}-{self.task_path.stem}-"
            f"{self.object_material}-seed{self.sensor_noise_seed:03d}"
        )


def _create_study_directory(config: DMAdmittanceTuningConfig) -> Path:
    """为一次调参运行创建独占 study 输出目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def validate_inputs(config: DMAdmittanceTuningConfig) -> tuple[GripperProfile, ForceTrackingTask]:
    """加载并校验 profile 和 Ramp 任务输入。"""
    profile = load_profile(config.profile)
    if profile.normal_force is None or profile.normal_force.admittance is None:
        raise ValueError("导纳调参 profile 必须包含 control.force.admittance。")
    task = ForceTrackingTask.load(config.task)
    if task.reference.interpolation != "linear":
        raise ValueError("导纳调参仅接受线性 Ramp 力跟踪任务。")
    return profile, task


def _write_candidate_profiles(
    profile: GripperProfile,
    candidates: Iterable[DMAdmittanceCandidate],
    directory: Path,
) -> dict[DMAdmittanceCandidate, Path]:
    """为每个候选写入自包含且相互独立的临时 profile。"""
    raw_profile = profile.model_dump(mode="json")
    force = raw_profile["control"].get("force")
    if not isinstance(force, dict) or not isinstance(force.get("admittance"), dict):
        raise ValueError("导纳调参 profile 缺少 control.force.admittance。")
    directory.mkdir(parents=True, exist_ok=False)
    paths: dict[DMAdmittanceCandidate, Path] = {}
    try:
        for candidate in candidates:
            candidate_profile = json.loads(json.dumps(raw_profile))
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
            path = directory / f"{candidate.identifier}.yaml"
            path.write_text(
                yaml.safe_dump(candidate_profile, allow_unicode=True, sort_keys=False),
                encoding="utf-8",
            )
            paths[candidate] = path
    except Exception:
        _cleanup_candidate_profiles(paths.values(), directory)
        raise
    return paths


def _cleanup_candidate_profiles(paths: Iterable[Path], directory: Path) -> None:
    """清理仅供本次运行使用的候选 profile，不影响各子运行快照。"""
    for path in paths:
        path.unlink(missing_ok=True)
    try:
        directory.rmdir()
    except OSError:
        # 保留目录中的非预期文件，避免掩盖并行执行或文件系统问题。
        pass


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


def _execute_condition(condition: RunCondition) -> dict[str, object]:
    """在单独进程中执行一个候选条件并返回可序列化结果。"""
    task = ForceTrackingTask.load(condition.task_path)
    run, result = execute_force_tracking(
        profile=condition.profile_path,
        task_path=condition.task_path,
        output_root=condition.output_root,
        run_prefix=condition.run_prefix,
        object_material=condition.object_material,  # type: ignore[arg-type]
        controller_variant="admittance",
        sensor_noise_seed=condition.sensor_noise_seed,
    )
    return {
        "candidate_id": condition.candidate.identifier,
        "mass_kg": condition.candidate.mass_kg,
        "damping_ns_m": condition.candidate.damping_ns_m,
        "stiffness_n_m": condition.candidate.stiffness_n_m,
        "filter_cutoff_hz": condition.candidate.filter_cutoff_hz,
        "velocity_limit_rad_s": condition.candidate.velocity_limit_rad_s,
        "approach_velocity_rad_s": condition.candidate.approach_velocity_rad_s,
        "contact_stable_time_s": condition.candidate.contact_stable_time_s,
        "contact_transition_time_s": condition.candidate.contact_transition_time_s,
        "approach_feedforward_force_n": condition.candidate.approach_feedforward_force_n,
        "task_name": task.name,
        "task_path": str(condition.task_path),
        "object_material": condition.object_material,
        "sensor_noise_seed": condition.sensor_noise_seed,
        "passed": result.passed,
        "run_directory": str(run.path.relative_to(condition.study_directory)),
        **asdict(result),
        **_trace_diagnostics(run.path, ignore_initial_s=task.metrics.ignore_initial_s),
    }


ExecutorFactory = Callable[..., Any]
ConditionWorker = Callable[[RunCondition], dict[str, object]]


def execute_conditions(
    conditions: tuple[RunCondition, ...],
    *,
    max_workers: int,
    executor_factory: ExecutorFactory = ProcessPoolExecutor,
    worker: ConditionWorker = _execute_condition,
) -> list[dict[str, object]]:
    """并行执行条件并按配置顺序返回结果，而非按完成顺序返回。"""
    if max_workers <= 0:
        raise ValueError("max_workers 必须为正整数。")
    results: dict[tuple[str, str, int], dict[str, object]] = {}
    with executor_factory(max_workers=max_workers) as executor:
        futures: dict[Future[dict[str, object]], RunCondition] = {
            executor.submit(worker, condition): condition for condition in conditions
        }
        for future in as_completed(futures):
            condition = futures[future]
            try:
                results[condition.key] = future.result()
            except Exception as error:
                for pending in futures:
                    pending.cancel()
                raise RuntimeError(
                    f"导纳候选 {condition.candidate.identifier} 执行失败。"
                ) from error
    return [results[condition.key] for condition in conditions]


def aggregate_rows(
    rows: list[dict[str, object]], *, minimum_ratio: float
) -> list[dict[str, object]]:
    """按候选聚合稳定性、跟踪完整度和误差指标。"""
    groups: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        groups.setdefault(str(row["candidate_id"]), []).append(row)
    aggregates: list[dict[str, object]] = []
    for candidate_id, group in groups.items():
        first = group[0]
        aggregate: dict[str, object] = {
            "candidate_id": candidate_id,
            "mass_kg": first["mass_kg"],
            "damping_ns_m": first["damping_ns_m"],
            "stiffness_n_m": first["stiffness_n_m"],
            "filter_cutoff_hz": first["filter_cutoff_hz"],
            "velocity_limit_rad_s": first["velocity_limit_rad_s"],
            "approach_velocity_rad_s": first["approach_velocity_rad_s"],
            "contact_stable_time_s": first["contact_stable_time_s"],
            "contact_transition_time_s": first["contact_transition_time_s"],
            "approach_feedforward_force_n": first["approach_feedforward_force_n"],
            "runs": len(group),
            "stable_runs": sum(bool(row["passed"]) for row in group),
            "complete_force_tracking_runs": sum(
                float(row["force_tracking_ratio"]) >= minimum_ratio for row in group
            ),
            "force_tracking_ratio_mean": fmean(float(row["force_tracking_ratio"]) for row in group),
            "force_tracking_ratio_min": min(float(row["force_tracking_ratio"]) for row in group),
        }
        for metric in METRICS:
            values = [
                float(row[metric])
                for row in group
                if row[metric] is not None and math.isfinite(float(row[metric]))
            ]
            aggregate[f"{metric}_mean"] = fmean(values) if values else None
        aggregates.append(aggregate)
    return aggregates


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


def _json_compatible(value: object) -> object:
    """把非有限浮点转换为标准 JSON 可表达的 ``null``。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    return value


def describe_conditions(config: DMAdmittanceTuningConfig) -> str:
    """返回调参条件的稳定、可审阅文本。"""
    lines = [
        f"Study: {config.name}",
        f"Profile: {config.profile}",
        f"Task: {config.task}",
        "Controller: admittance",
        f"Max workers: {config.max_workers}",
        f"Conditions: {len(config.conditions())}",
    ]
    for index, (candidate, material, seed) in enumerate(config.conditions(), start=1):
        lines.append(
            f"{index:03d} candidate={candidate.identifier} material={material} seed={seed}"
        )
    return "\n".join(lines)


def run_study(
    config: DMAdmittanceTuningConfig,
    *,
    config_source: Path,
) -> Path:
    """执行并行导纳 Ramp 调参并写出 CSV、Parquet 和 JSON 汇总。"""
    profile, _ = validate_inputs(config)
    study_dir = _create_study_directory(config)
    (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)
    temporary_profiles = study_dir / ".candidate_profiles"
    candidate_profiles: dict[DMAdmittanceCandidate, Path] = {}
    try:
        candidate_profiles = _write_candidate_profiles(
            profile, config.candidates, temporary_profiles
        )
        conditions = tuple(
            RunCondition(
                candidate=candidate,
                profile_path=candidate_profiles[candidate],
                task_path=config.task,
                output_root=study_dir / "runs" / candidate.identifier,
                study_directory=study_dir,
                object_material=material,
                sensor_noise_seed=seed,
            )
            for candidate, material, seed in config.conditions()
        )
        rows = execute_conditions(conditions, max_workers=config.max_workers)
    finally:
        _cleanup_candidate_profiles(candidate_profiles.values(), temporary_profiles)

    aggregates = aggregate_rows(rows, minimum_ratio=config.minimum_force_tracking_ratio)
    ranking = rank_candidates(aggregates)
    summary_csv, summary_parquet = write_rows_csv_and_parquet(study_dir / "summary.csv", rows)
    aggregate_csv, aggregate_parquet = write_rows_csv_and_parquet(
        study_dir / "aggregate.csv", aggregates
    )
    ranking_csv, ranking_parquet = write_rows_csv_and_parquet(
        study_dir / "candidate_ranking.csv", ranking
    )
    summary_json = study_dir / "summary.json"
    summary_json.write_text(
        json.dumps(
            _json_compatible({"runs": rows, "aggregates": aggregates, "ranking": ranking}),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    artifacts = (
        summary_csv,
        summary_parquet,
        aggregate_csv,
        aggregate_parquet,
        ranking_csv,
        ranking_parquet,
        summary_json,
        resolved_config,
    )
    (study_dir / "study_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": config.name,
                "config": "study.yaml",
                "resolved_config": str(resolved_config.relative_to(study_dir)),
                "runs": [row["run_directory"] for row in rows],
                "artifacts": [str(path.relative_to(study_dir)) for path in artifacts],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return study_dir


def main() -> None:
    """解析命令行参数并运行或预览调参 study。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--dry-run", action="store_true")
    arguments = parser.parse_args()
    config_path = arguments.config.resolve()
    config = load_dm_admittance_tuning_config(config_path)
    if arguments.dry_run:
        validate_inputs(config)
        print(describe_conditions(config))
        return
    study_dir = run_study(config, config_source=config_path)
    print(study_dir)


if __name__ == "__main__":
    main()

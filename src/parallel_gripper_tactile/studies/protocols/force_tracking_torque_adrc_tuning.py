"""运行二阶直接力矩 ADRC 的两阶段带宽调参研究。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import csv
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from hashlib import sha256
import json
import math
from pathlib import Path
from statistics import fmean, stdev
from typing import Iterable
from uuid import uuid4

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
from parallel_gripper_tactile.visualization import (
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)
from parallel_gripper_tactile.config.profiles import GripperProfile, TorqueAdrcControl, load_profile
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.aggregation import (
    aggregate_records,
    bool_sum,
    count,
    finite_mean,
    finite_std,
    first,
    key,
)
from parallel_gripper_tactile.studies.force_tracking_torque_adrc_tuning import (
    ForceTrackingTorqueAdrcTuningConfig,
    TorqueAdrcCandidate,
    TorqueAdrcTuningStageName,
)
from parallel_gripper_tactile.studies.lifecycle import (
    ConditionExecution,
    ConditionOutcome,
    StudyCondition,
    StudyPlan,
    StudyPostprocessResult,
    execute_study_lifecycle,
    execution_failure_rows,
    file_sha256,
    model_configuration_sha256,
    require_matching_study_plan,
    scientific_configuration_hash,
)
from parallel_gripper_tactile.studies.tabular import (
    write_resolved_config,
    write_rows_csv,
    write_rows_csv_and_parquet,
)


METRICS = ("rmse_n", "overshoot_ratio", "settling_time_s", "torque_saturation_ratio")
_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
_STUDY_KIND = "force_tracking_torque_adrc_tuning"
_RANKING_COLUMNS = {
    "rank",
    "candidate_id",
    "measurement_filter_cutoff_hz",
    "controller_bandwidth_rad_s",
    "observer_bandwidth_ratio",
    "observer_bandwidth_rad_s",
    "feasible",
    "step_overshoot_ratio_mean",
    "step_rmse_n_mean",
    "ramp_rmse_n_mean",
    "mixed_rmse_n_mean",
    "ramp_rmse_ratio_to_baseline",
    "mixed_rmse_ratio_to_baseline",
    "max_torque_saturation_ratio",
}


@dataclass(frozen=True, slots=True)
class ValidatedCoarseReference:
    """通过 manifest、哈希和排名 schema 校验的 coarse 输入。"""

    directory: Path
    candidates: tuple[TorqueAdrcCandidate, ...]
    ranking_sha256: str
    scientific_configuration_sha256: str


def _create_study_directory(config: ForceTrackingTorqueAdrcTuningConfig, stage: str) -> Path:
    """为指定调参阶段创建独占输出目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / stage / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def _candidate_from_row(row: dict[str, str]) -> TorqueAdrcCandidate:
    """从候选排名 CSV 还原一组调参参数。"""
    return TorqueAdrcCandidate(
        measurement_filter_cutoff_hz=float(row["measurement_filter_cutoff_hz"]),
        controller_bandwidth_rad_s=float(row["controller_bandwidth_rad_s"]),
        observer_bandwidth_ratio=float(row["observer_bandwidth_ratio"]),
    )


def _study_definition(
    config: ForceTrackingTorqueAdrcTuningConfig,
    profile: GripperProfile,
) -> tuple[dict[str, object], str]:
    """返回包含隐式运行参数与资源内容的两阶段科学定义及哈希。"""
    baseline_control = TorqueAdrcControl(
        measurement_filter_cutoff_hz=config.baseline.measurement_filter_cutoff_hz,
        controller_bandwidth_rad_s=config.baseline.controller_bandwidth_rad_s,
        observer_bandwidth_rad_s=config.baseline.observer_bandwidth_rad_s,
    )
    definition: dict[str, object] = {
        "hash_schema_version": 1,
        "protocol_revision": "force_tracking_torque_adrc_tuning.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": model_configuration_sha256(profile, repository_root=_REPOSITORY_ROOT),
            "task_sha256": {str(task): file_sha256(task) for task in config.tasks},
        },
        "torque_adrc_defaults": baseline_control.model_dump(mode="python"),
        "scientific_runtime": {
            "controller_variant": "adrc-torque",
            "object_contact_model": "explicit",
            "multiccd_enabled": True,
            "force_semantics": "average_side",
            "trace_sample_period_s": 0.004,
            "trace_event_window_s": 0.2,
        },
        "ranking_policy": {
            "aggregation": "candidate,task,material; unweighted material mean v1",
            "feasibility": "all passed; continuous ratios and saturation bounded v1",
            "ordering": "feasible,step_overshoot,step_rmse,mixed_rmse; stable v1",
            "confirm_selection": "first feasible top_n; append baseline; stable unique v1",
        },
    }
    return definition, scientific_configuration_hash(definition, repository_root=_REPOSITORY_ROOT)


def _read_validated_coarse_reference(
    config: ForceTrackingTorqueAdrcTuningConfig,
    coarse_study_dir: Path,
) -> ValidatedCoarseReference:
    """严格校验 confirm 所依赖的 coarse manifest 和候选排名。"""
    directory = coarse_study_dir.resolve()
    manifest_path = directory / "study_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"cannot read coarse study manifest: {manifest_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid coarse study manifest JSON: {manifest_path}") from error
    if not isinstance(manifest, dict):
        raise ValueError(f"coarse study manifest must be an object: {manifest_path}")
    expected_coarse_plan = build_plan(config, stage="coarse")
    required = {
        "lifecycle_schema_version": 1,
        "study_kind": _STUDY_KIND,
        "stage": "coarse",
        "state": "completed",
        "study_definition_sha256": expected_coarse_plan.study_definition_sha256,
        "scientific_configuration_sha256": (expected_coarse_plan.scientific_configuration_sha256),
        "execution_error_count": 0,
    }
    mismatches = [key for key, value in required.items() if manifest.get(key) != value]
    if mismatches:
        raise ValueError("incompatible coarse study manifest fields: " + ", ".join(mismatches))
    if manifest.get("lifecycle_failures") not in ([], None):
        raise ValueError("coarse study contains lifecycle failures")
    coarse_scientific_hash = manifest.get("scientific_configuration_sha256")
    if not isinstance(coarse_scientific_hash, str) or len(coarse_scientific_hash) != 64:
        raise ValueError("coarse manifest lacks scientific configuration hash")
    artifacts = manifest.get("artifacts")
    hashes = manifest.get("artifact_sha256")
    if not isinstance(artifacts, list) or "candidate_ranking.csv" not in artifacts:
        raise ValueError("coarse manifest does not register candidate_ranking.csv")
    if not isinstance(hashes, dict) or not isinstance(hashes.get("candidate_ranking.csv"), str):
        raise ValueError("coarse manifest lacks candidate_ranking.csv digest")
    ranking_path = directory / "candidate_ranking.csv"
    ranking_digest = sha256(ranking_path.read_bytes()).hexdigest()
    if ranking_digest != hashes["candidate_ranking.csv"]:
        raise ValueError("coarse candidate_ranking.csv digest mismatch")
    with ranking_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not _RANKING_COLUMNS.issubset(reader.fieldnames):
            raise ValueError("coarse candidate ranking has an incompatible schema")
        ranking = list(reader)
    expected_candidates = set(config.stage_candidates("coarse"))
    if len(ranking) != len(expected_candidates):
        raise ValueError("coarse candidate ranking does not contain the complete candidate set")
    ranked_candidates: list[TorqueAdrcCandidate] = []
    for expected_rank, row in enumerate(ranking, start=1):
        try:
            rank = int(row["rank"])
            candidate = _candidate_from_row(row)
            observer_bandwidth = float(row["observer_bandwidth_rad_s"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("coarse candidate ranking contains invalid values") from error
        if rank != expected_rank or row["candidate_id"] != candidate.identifier:
            raise ValueError("coarse candidate ranking has invalid rank or candidate identity")
        if not math.isclose(observer_bandwidth, candidate.observer_bandwidth_rad_s):
            raise ValueError("coarse candidate ranking has inconsistent observer bandwidth")
        if row.get("feasible") not in {"true", "false"}:
            raise ValueError("coarse candidate ranking has invalid feasibility value")
        ranked_candidates.append(candidate)
    if (
        len(set(ranked_candidates)) != len(ranked_candidates)
        or set(ranked_candidates) != expected_candidates
    ):
        raise ValueError("coarse candidate ranking contains duplicate or unexpected candidates")
    feasible = [
        candidate
        for candidate, row in zip(ranked_candidates, ranking, strict=True)
        if row["feasible"] == "true"
    ]
    if not feasible:
        raise ValueError(f"coarse study has no feasible candidates: {ranking_path}")
    selected = feasible[: config.confirm_top_candidates]
    if config.baseline not in selected:
        selected.append(config.baseline)
    allowed = set(config.stage_candidates("confirm"))
    unexpected = [candidate.identifier for candidate in selected if candidate not in allowed]
    if unexpected:
        raise ValueError("confirm candidate is absent from confirm grid: " + ", ".join(unexpected))
    return ValidatedCoarseReference(
        directory=directory,
        candidates=tuple(dict.fromkeys(selected)),
        ranking_sha256=ranking_digest,
        scientific_configuration_sha256=coarse_scientific_hash,
    )


def stage_candidates(
    config: ForceTrackingTorqueAdrcTuningConfig,
    stage: TorqueAdrcTuningStageName,
    coarse_study_dir: Path | None,
) -> tuple[TorqueAdrcCandidate, ...]:
    """返回当前阶段要执行的候选，确认阶段从粗扫排名中选择。"""
    if stage == "coarse":
        return config.stage_candidates(stage)
    if coarse_study_dir is None:
        raise ValueError("--coarse-study-dir is required for confirm stage")
    return _read_validated_coarse_reference(config, coarse_study_dir).candidates


def _aggregate(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按候选、任务和材料聚合重复运行指标。"""
    return aggregate_records(
        rows,
        keys=("candidate_id", "task_name", "object_material"),
        columns=(
            first("candidate_id"),
            first("measurement_filter_cutoff_hz"),
            first("controller_bandwidth_rad_s"),
            first("observer_bandwidth_ratio"),
            first("observer_bandwidth_rad_s"),
            key("task_name"),
            key("object_material"),
            count("runs"),
            bool_sum("passed", "passed_runs"),
            *(stat for metric in METRICS for stat in (finite_mean(metric), finite_std(metric))),
        ),
    )


def _mean_metric(
    aggregates: Iterable[dict[str, object]], candidate_id: str, task_marker: str, metric: str
) -> float | None:
    """返回一个候选在某类任务上的跨材料均值。"""
    values = [
        float(row[f"{metric}_mean"])
        for row in aggregates
        if str(row["candidate_id"]) == candidate_id
        and task_marker in str(row["task_name"])
        and row[f"{metric}_mean"] is not None
        and math.isfinite(float(row[f"{metric}_mean"]))
    ]
    return fmean(values) if values else None


def _finite_or_nan(value: object) -> float:
    """将缺失或非有限指标映射为 NaN，便于图表保留缺口。"""
    if value is None:
        return math.nan
    number = float(value)
    return number if math.isfinite(number) else math.nan


def plot_candidate_ranking_and_feasibility(
    ranking: list[dict[str, object]],
    output: Path,
    *,
    max_torque_saturation_ratio: float,
    max_ramp_rmse_ratio_to_baseline: float,
    max_mixed_rmse_ratio_to_baseline: float,
) -> Path:
    """绘制候选排序以及连续任务和饱和约束的可行性。"""
    if not ranking:
        raise ValueError("cannot plot empty ranking")
    plt = science_pyplot()
    labels = [f"#{int(row['rank'])} {row['candidate_id']}" for row in ranking]
    feasible = [str(row.get("feasible")) == "true" for row in ranking]
    colors = ["#009E73" if item else "#D55E00" for item in feasible]
    hatches = ["" if item else "//" for item in feasible]
    ranking_height = max(2.0, 0.18 * len(ranking))
    figure, panels = plt.subplot_mosaic(
        [["ranking", "saturation"], ["constraints", "constraints"]],
        figsize=paper_figsize(ranking_height + 2.6),
        height_ratios=(ranking_height, 2.6),
        layout="constrained",
    )
    axes = (panels["ranking"], panels["constraints"], panels["saturation"])
    overshoot = [_finite_or_nan(row.get("step_overshoot_ratio_mean")) for row in ranking]
    axes[0].barh(labels, overshoot, color=colors, hatch=hatches, edgecolor="black", linewidth=0.4)
    axes[0].invert_yaxis()
    axes[0].set_xlabel("Step overshoot ratio")
    axes[0].set_title("Candidate ranking")
    axes[0].grid(True, axis="x", linewidth=0.3, alpha=0.5)

    ramp = [_finite_or_nan(row.get("ramp_rmse_ratio_to_baseline")) for row in ranking]
    mixed = [_finite_or_nan(row.get("mixed_rmse_ratio_to_baseline")) for row in ranking]
    for is_feasible, marker, color in ((True, "o", "#009E73"), (False, "x", "#D55E00")):
        indices = [index for index, value in enumerate(feasible) if value == is_feasible]
        axes[1].scatter(
            [ramp[index] for index in indices],
            [mixed[index] for index in indices],
            color=color,
            marker=marker,
            s=32,
            label="Feasible" if is_feasible else "Infeasible",
        )
    axes[1].legend(frameon=False, fontsize=8)
    axes[1].axvline(max_ramp_rmse_ratio_to_baseline, color="black", linestyle="--", linewidth=0.8)
    axes[1].axhline(max_mixed_rmse_ratio_to_baseline, color="black", linestyle="--", linewidth=0.8)
    for index, (x_value, y_value) in enumerate(zip(ramp, mixed, strict=True), start=1):
        if feasible[index - 1] and math.isfinite(x_value) and math.isfinite(y_value):
            axes[1].annotate(
                str(index), (x_value, y_value), xytext=(3, 3), textcoords="offset points"
            )
    axes[1].set_xlabel("Ramp RMSE / baseline")
    axes[1].set_ylabel("Mixed RMSE / baseline")
    axes[1].set_title("Continuous-task constraints")
    axes[1].grid(True, linewidth=0.3, alpha=0.5)

    saturation = [_finite_or_nan(row.get("max_torque_saturation_ratio")) for row in ranking]
    axes[2].barh(labels, saturation, color=colors, hatch=hatches, edgecolor="black", linewidth=0.4)
    axes[2].invert_yaxis()
    axes[2].tick_params(axis="y", labelleft=False)
    axes[2].axvline(max_torque_saturation_ratio, color="black", linestyle="--", linewidth=0.8)
    axes[2].set_xlabel("Maximum torque\nsaturation ratio")
    axes[2].set_title("Saturation constraint")
    axes[2].grid(True, axis="x", linewidth=0.3, alpha=0.5)
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def _candidate_performance(aggregates: list[dict[str, object]]) -> list[dict[str, float]]:
    """将各任务和材料的聚合指标压缩为候选级参数性能点。"""
    grouped: dict[str, list[dict[str, object]]] = {}
    for row in aggregates:
        grouped.setdefault(str(row["candidate_id"]), []).append(row)
    performances: list[dict[str, float]] = []
    for group in grouped.values():
        first = group[0]
        rmse = [
            _finite_or_nan(row.get("rmse_n_mean"))
            for row in group
            if math.isfinite(_finite_or_nan(row.get("rmse_n_mean")))
        ]
        performances.append(
            {
                "measurement_filter_cutoff_hz": float(first["measurement_filter_cutoff_hz"]),
                "controller_bandwidth_rad_s": float(first["controller_bandwidth_rad_s"]),
                "observer_bandwidth_ratio": float(first["observer_bandwidth_ratio"]),
                "rmse_n": fmean(rmse) if rmse else math.nan,
            }
        )
    return performances


def plot_parameter_performance(aggregates: list[dict[str, object]], output: Path) -> Path:
    """绘制滤波截止频率、控制带宽和观测器比例对总体 RMSE 的影响。"""
    if not aggregates:
        raise ValueError("cannot plot empty aggregates")
    plt = science_pyplot()
    performances = _candidate_performance(aggregates)
    parameters = (
        ("measurement_filter_cutoff_hz", "Filter cutoff (Hz)"),
        ("controller_bandwidth_rad_s", "Controller bandwidth\n(rad/s)"),
        ("observer_bandwidth_ratio", "Observer/controller\nbandwidth ratio"),
    )
    figure, axes = plt.subplots(1, 3, figsize=paper_figsize(3.6), layout="constrained")
    for axis, (parameter, label) in zip(axes, parameters, strict=True):
        grouped: dict[float, list[float]] = {}
        for item in performances:
            value = item["rmse_n"]
            if math.isfinite(value):
                grouped.setdefault(item[parameter], []).append(value)
        x_values = sorted(grouped)
        means = [fmean(grouped[value]) for value in x_values]
        errors = [stdev(grouped[value]) if len(grouped[value]) > 1 else 0.0 for value in x_values]
        axis.scatter(
            [item[parameter] for item in performances],
            [item["rmse_n"] for item in performances],
            color="#0072B2",
            alpha=0.25,
            s=24,
        )
        axis.errorbar(x_values, means, yerr=errors, color="#0072B2", marker="o", capsize=2)
        axis.set_xlabel(label)
        axis.set_ylabel("Mean RMSE (N)")
        axis.grid(True, linewidth=0.3, alpha=0.5)
    figure.suptitle("ADRC parameter-performance relationships")
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def render_study_figures(
    aggregates: list[dict[str, object]],
    ranking: list[dict[str, object]],
    study_dir: Path,
    *,
    max_torque_saturation_ratio: float,
    max_ramp_rmse_ratio_to_baseline: float,
    max_mixed_rmse_ratio_to_baseline: float,
) -> list[Path]:
    """生成适用于 coarse 和 confirm 阶段的论文级调参图表。"""
    figures_dir = study_dir / "figures"
    figures_dir.mkdir(exist_ok=True)
    ranking_plot = figures_dir / "candidate_ranking_and_feasibility.png"
    parameter_plot = figures_dir / "parameter_performance.png"
    ranking_path = plot_candidate_ranking_and_feasibility(
        ranking,
        ranking_plot,
        max_torque_saturation_ratio=max_torque_saturation_ratio,
        max_ramp_rmse_ratio_to_baseline=max_ramp_rmse_ratio_to_baseline,
        max_mixed_rmse_ratio_to_baseline=max_mixed_rmse_ratio_to_baseline,
    )
    parameter_path = plot_parameter_performance(aggregates, parameter_plot)
    return [ranking_path, parameter_path]


def rank_candidates(
    aggregates: list[dict[str, object]],
    candidates: Iterable[TorqueAdrcCandidate],
    *,
    baseline: TorqueAdrcCandidate,
    max_torque_saturation_ratio: float,
    max_ramp_rmse_ratio_to_baseline: float,
    max_mixed_rmse_ratio_to_baseline: float,
) -> list[dict[str, object]]:
    """按连续任务约束和阶跃超调排序调参候选。"""
    baseline_ramp = _mean_metric(aggregates, baseline.identifier, "ramp", "rmse_n")
    baseline_mixed = _mean_metric(aggregates, baseline.identifier, "mixed", "rmse_n")
    if baseline_ramp is None or baseline_mixed is None:
        raise ValueError("baseline lacks ramp or mixed RMSE")
    ranking: list[dict[str, object]] = []
    for candidate in candidates:
        candidate_rows = [row for row in aggregates if row["candidate_id"] == candidate.identifier]
        saturation_values = [
            float(row["torque_saturation_ratio_mean"])
            for row in candidate_rows
            if row["torque_saturation_ratio_mean"] is not None
        ]
        ramp_rmse = _mean_metric(aggregates, candidate.identifier, "ramp", "rmse_n")
        mixed_rmse = _mean_metric(aggregates, candidate.identifier, "mixed", "rmse_n")
        step_overshoot = _mean_metric(aggregates, candidate.identifier, "step", "overshoot_ratio")
        step_rmse = _mean_metric(aggregates, candidate.identifier, "step", "rmse_n")
        max_saturation = max(saturation_values, default=math.inf)
        ramp_ratio = None if ramp_rmse is None else ramp_rmse / baseline_ramp
        mixed_ratio = None if mixed_rmse is None else mixed_rmse / baseline_mixed
        feasible = (
            len(candidate_rows) > 0
            and all(int(row["passed_runs"]) == int(row["runs"]) for row in candidate_rows)
            and ramp_rmse is not None
            and mixed_rmse is not None
            and step_overshoot is not None
            and step_rmse is not None
            and max_saturation <= max_torque_saturation_ratio
            and ramp_ratio <= max_ramp_rmse_ratio_to_baseline
            and mixed_ratio <= max_mixed_rmse_ratio_to_baseline
        )
        ranking.append(
            {
                "candidate_id": candidate.identifier,
                "measurement_filter_cutoff_hz": candidate.measurement_filter_cutoff_hz,
                "controller_bandwidth_rad_s": candidate.controller_bandwidth_rad_s,
                "observer_bandwidth_ratio": candidate.observer_bandwidth_ratio,
                "observer_bandwidth_rad_s": candidate.observer_bandwidth_rad_s,
                "feasible": str(feasible).lower(),
                "step_overshoot_ratio_mean": step_overshoot,
                "step_rmse_n_mean": step_rmse,
                "ramp_rmse_n_mean": ramp_rmse,
                "mixed_rmse_n_mean": mixed_rmse,
                "ramp_rmse_ratio_to_baseline": ramp_ratio,
                "mixed_rmse_ratio_to_baseline": mixed_ratio,
                "max_torque_saturation_ratio": max_saturation,
            }
        )
    ranking.sort(
        key=lambda row: (
            row["feasible"] != "true",
            math.inf
            if row["step_overshoot_ratio_mean"] is None
            else row["step_overshoot_ratio_mean"],
            math.inf if row["step_rmse_n_mean"] is None else row["step_rmse_n_mean"],
            math.inf if row["mixed_rmse_n_mean"] is None else row["mixed_rmse_n_mean"],
        )
    )
    for index, row in enumerate(ranking, start=1):
        row["rank"] = index
    return ranking


def build_plan(
    config: ForceTrackingTorqueAdrcTuningConfig,
    *,
    stage: TorqueAdrcTuningStageName,
    coarse_study_dir: Path | None = None,
    resolved_profile: GripperProfile | None = None,
) -> StudyPlan:
    """生成 coarse 或经严格谱系校验的 confirm 唯一有序计划。"""
    base_profile = resolved_profile or load_profile(config.profile)
    reference = None
    if stage == "confirm":
        if coarse_study_dir is None:
            raise ValueError("--coarse-study-dir is required for confirm stage")
        reference = _read_validated_coarse_reference(config, coarse_study_dir)
    candidates = config.stage_candidates("coarse") if reference is None else reference.candidates
    conditions: list[StudyCondition] = []
    for candidate, task, material, seed in config.conditions(stage, candidates=candidates):
        control = TorqueAdrcControl(
            measurement_filter_cutoff_hz=candidate.measurement_filter_cutoff_hz,
            controller_bandwidth_rad_s=candidate.controller_bandwidth_rad_s,
            observer_bandwidth_rad_s=candidate.observer_bandwidth_rad_s,
        )
        conditions.append(
            StudyCondition(
                condition_id=(f"{candidate.identifier}-{task.stem}-{material}-seed{seed:03d}"),
                parameters={
                    "candidate_id": candidate.identifier,
                    "measurement_filter_cutoff_hz": candidate.measurement_filter_cutoff_hz,
                    "controller_bandwidth_rad_s": candidate.controller_bandwidth_rad_s,
                    "observer_bandwidth_ratio": candidate.observer_bandwidth_ratio,
                    "observer_bandwidth_rad_s": candidate.observer_bandwidth_rad_s,
                    "torque_adrc": control.model_dump(mode="json"),
                    "controller_variant": "adrc-torque",
                    "stiffness_estimator_method": config.stiffness_estimator_method,
                    "task_path": str(task),
                    "object_material": material,
                    "sensor_noise_seed": seed,
                },
                pair_key=f"{task.stem}:{material}:seed{seed:03d}",
                baseline_role="baseline" if candidate == config.baseline else None,
            )
        )
    definition, definition_hash = _study_definition(config, base_profile)
    lineage = (
        None
        if reference is None
        else {
            "coarse_scientific_configuration_sha256": (reference.scientific_configuration_sha256),
            "coarse_ranking_sha256": reference.ranking_sha256,
        }
    )
    plan_hash = scientific_configuration_hash(
        {
            "study_definition_sha256": definition_hash,
            "stage": stage,
            "conditions": [condition.model_dump(mode="python") for condition in conditions],
            "coarse_lineage": lineage,
        },
        repository_root=_REPOSITORY_ROOT,
    )
    stage_config = config.stage_config(stage)
    return StudyPlan(
        study_kind=_STUDY_KIND,
        stage=stage,
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=tuple(conditions),
        seeds=tuple(stage_config.seeds.values()),
        preflight={
            "status": "pending",
            "checks": ["profile", "tasks", "scenes", "coarse_lineage"]
            if stage == "confirm"
            else ["profile", "tasks", "scenes"],
            "definition": definition,
            "coarse_reference": None
            if reference is None
            else {
                "directory": str(reference.directory),
                "ranking_sha256": reference.ranking_sha256,
                "scientific_configuration_sha256": (reference.scientific_configuration_sha256),
            },
        },
    )


def describe_conditions(
    config: ForceTrackingTorqueAdrcTuningConfig,
    *,
    stage: TorqueAdrcTuningStageName,
    coarse_study_dir: Path | None = None,
) -> str:
    """返回可审阅的调参矩阵说明。"""
    plan = build_plan(config, stage=stage, coarse_study_dir=coarse_study_dir)
    identifiers = tuple(
        dict.fromkeys(str(condition.parameters["candidate_id"]) for condition in plan.conditions)
    )
    lines = [
        f"Study: {config.name}",
        f"Stage: {stage}",
        f"Conditions: {len(plan.conditions)}",
    ]
    lines.extend(identifiers)
    return "\n".join(lines)


def run_study(
    config: ForceTrackingTorqueAdrcTuningConfig,
    *,
    stage: TorqueAdrcTuningStageName,
    config_source: Path,
    resolved_profile: GripperProfile | None = None,
    coarse_study_dir: Path | None = None,
    study_directory: Path | None = None,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
) -> Path:
    """通过公共生命周期执行调参阶段并生成候选排名。"""
    if resolved_profile is None:
        load_profile(config.profile)
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    expected_plan = build_plan(
        config,
        stage=stage,
        coarse_study_dir=coarse_study_dir,
        resolved_profile=resolved_profile,
    )
    plan = (
        expected_plan
        if study_plan is None
        else require_matching_study_plan(expected_plan, study_plan)
    )
    study_dir = (
        _create_study_directory(config, stage)
        if study_directory is None
        else study_directory.resolve()
    )
    study_dir.mkdir(parents=True, exist_ok=True)
    (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)

    def execute(condition: StudyCondition) -> ConditionExecution:
        parameters = condition.parameters
        task_path = Path(str(parameters["task_path"]))
        candidate = TorqueAdrcCandidate(
            measurement_filter_cutoff_hz=float(parameters["measurement_filter_cutoff_hz"]),
            controller_bandwidth_rad_s=float(parameters["controller_bandwidth_rad_s"]),
            observer_bandwidth_ratio=float(parameters["observer_bandwidth_ratio"]),
        )
        override = TorqueAdrcControl(
            measurement_filter_cutoff_hz=candidate.measurement_filter_cutoff_hz,
            controller_bandwidth_rad_s=candidate.controller_bandwidth_rad_s,
            observer_bandwidth_rad_s=candidate.observer_bandwidth_rad_s,
        )
        task = tasks[task_path]
        material = str(parameters["object_material"])
        seed = int(parameters["sensor_noise_seed"])
        run, result = execute_force_tracking(
            profile=config.profile,
            resolved_profile=resolved_profile,
            task_path=task_path,
            tracking_task=task,
            output_root=study_dir / "runs",
            run_prefix=condition.condition_id,
            object_material=material,
            controller_variant="adrc-torque",
            stiffness_estimator_method=config.stiffness_estimator_method,
            sensor_noise_seed=seed,
            torque_adrc_override=override,
        )
        row = {
            "candidate_id": candidate.identifier,
            "measurement_filter_cutoff_hz": candidate.measurement_filter_cutoff_hz,
            "controller_bandwidth_rad_s": candidate.controller_bandwidth_rad_s,
            "observer_bandwidth_ratio": candidate.observer_bandwidth_ratio,
            "observer_bandwidth_rad_s": candidate.observer_bandwidth_rad_s,
            "task_name": task.name,
            "task_path": str(task_path),
            "object_material": material,
            "sensor_noise_seed": seed,
            "passed": result.passed,
            "run_directory": str(run.path.relative_to(study_dir)),
            **asdict(result),
        }
        return ConditionExecution(
            row=row,
            run_directory=str(row["run_directory"]),
            passed=result.passed,
        )

    candidates = tuple(
        dict.fromkeys(
            TorqueAdrcCandidate(
                measurement_filter_cutoff_hz=float(
                    condition.parameters["measurement_filter_cutoff_hz"]
                ),
                controller_bandwidth_rad_s=float(
                    condition.parameters["controller_bandwidth_rad_s"]
                ),
                observer_bandwidth_ratio=float(condition.parameters["observer_bandwidth_ratio"]),
            )
            for condition in plan.conditions
        )
    )

    def aggregate_and_persist(
        rows: list[dict[str, object]],
        outcomes: tuple[ConditionOutcome, ...],
        directory: Path,
    ) -> StudyPostprocessResult:
        aggregates = _aggregate(rows)
        ranking = rank_candidates(
            aggregates,
            candidates,
            baseline=config.baseline,
            max_torque_saturation_ratio=config.constraints.max_torque_saturation_ratio,
            max_ramp_rmse_ratio_to_baseline=(config.constraints.max_ramp_rmse_ratio_to_baseline),
            max_mixed_rmse_ratio_to_baseline=(config.constraints.max_mixed_rmse_ratio_to_baseline),
        )
        summary_csv, summary_parquet = write_rows_csv_and_parquet(directory / "summary.csv", rows)
        aggregate_csv, aggregate_parquet = write_rows_csv_and_parquet(
            directory / "aggregate.csv", aggregates
        )
        ranking_csv = write_rows_csv(directory / "candidate_ranking.csv", ranking)
        summary_json = directory / "summary.json"
        summary_json.write_text(
            json.dumps(
                {
                    "runs": rows,
                    "aggregates": aggregates,
                    "ranking": ranking,
                    "failures": execution_failure_rows(outcomes),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        return StudyPostprocessResult(
            artifacts=(
                summary_csv,
                summary_parquet,
                aggregate_csv,
                aggregate_parquet,
                ranking_csv,
                summary_json,
            ),
            manifest_fields={},
            render_payload={"aggregates": aggregates, "ranking": ranking},
        )

    def render(rows: list[dict[str, object]], payload: object, directory: Path) -> tuple[Path, ...]:
        del rows
        if not isinstance(payload, dict):
            raise TypeError("torque ADRC rendering payload must be a mapping")
        aggregates = payload.get("aggregates")
        ranking = payload.get("ranking")
        if not isinstance(aggregates, list) or not isinstance(ranking, list):
            raise TypeError("torque ADRC rendering payload lacks aggregates or ranking")
        return tuple(
            render_study_figures(
                aggregates,
                ranking,
                directory,
                max_torque_saturation_ratio=(config.constraints.max_torque_saturation_ratio),
                max_ramp_rmse_ratio_to_baseline=(
                    config.constraints.max_ramp_rmse_ratio_to_baseline
                ),
                max_mixed_rmse_ratio_to_baseline=(
                    config.constraints.max_mixed_rmse_ratio_to_baseline
                ),
            )
        )

    manifest_fields = {
        "schema_version": 1,
        "name": config.name,
        "stage": stage,
        "config": "study.yaml",
        "resolved_config": str(resolved_config.relative_to(study_dir)),
        "coarse_study_dir": None if coarse_study_dir is None else str(coarse_study_dir),
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
    )

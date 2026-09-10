"""运行二阶直接力矩 ADRC 的两阶段带宽调参研究。"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import UTC, datetime
from itertools import product
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
from parallel_gripper_tactile.config.profiles import TorqueAdrcControl, load_profile
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
    load_torque_adrc_tuning_config,
)
from parallel_gripper_tactile.studies.tabular import (
    write_resolved_config,
    write_rows_csv,
    write_rows_csv_and_parquet,
)


METRICS = ("rmse_n", "overshoot_ratio", "settling_time_s", "torque_saturation_ratio")


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


def _stage_candidates(
    config: ForceTrackingTorqueAdrcTuningConfig,
    stage: str,
    coarse_study_dir: Path | None,
) -> tuple[TorqueAdrcCandidate, ...]:
    """返回当前阶段要执行的候选，确认阶段从粗扫排名中选择。"""
    if stage == "coarse":
        return config.stage_candidates(stage)
    if coarse_study_dir is None:
        raise ValueError("--coarse-study-dir is required for confirm stage")
    ranking_path = coarse_study_dir / "candidate_ranking.csv"
    with ranking_path.open(newline="", encoding="utf-8") as stream:
        ranking = list(csv.DictReader(stream))
    feasible = [row for row in ranking if row.get("feasible") == "true"]
    if not feasible:
        raise ValueError(f"coarse study has no feasible candidates: {ranking_path}")
    selected = [_candidate_from_row(row) for row in feasible[: config.confirm_top_candidates]]
    if config.baseline not in selected:
        selected.append(config.baseline)
    allowed = set(config.stage_candidates("confirm"))
    unexpected = [candidate.identifier for candidate in selected if candidate not in allowed]
    if unexpected:
        raise ValueError("confirm candidate is absent from confirm grid: " + ", ".join(unexpected))
    return tuple(dict.fromkeys(selected))


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
    ranking_pdf = plot_candidate_ranking_and_feasibility(
        ranking,
        ranking_plot,
        max_torque_saturation_ratio=max_torque_saturation_ratio,
        max_ramp_rmse_ratio_to_baseline=max_ramp_rmse_ratio_to_baseline,
        max_mixed_rmse_ratio_to_baseline=max_mixed_rmse_ratio_to_baseline,
    )
    parameter_pdf = plot_parameter_performance(aggregates, parameter_plot)
    return [ranking_plot, ranking_pdf, parameter_plot, parameter_pdf]


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


def describe_conditions(
    config: ForceTrackingTorqueAdrcTuningConfig,
    *,
    stage: str,
    coarse_study_dir: Path | None = None,
) -> str:
    """返回可审阅的调参矩阵说明。"""
    candidates = _stage_candidates(config, stage, coarse_study_dir)
    stage_config = config.coarse if stage == "coarse" else config.confirm
    count = (
        len(candidates)
        * len(config.tasks)
        * len(stage_config.materials)
        * len(stage_config.seeds.values())
    )
    lines = [f"Study: {config.name}", f"Stage: {stage}", f"Conditions: {count}"]
    lines.extend(candidate.identifier for candidate in candidates)
    return "\n".join(lines)


def run_study(
    config: ForceTrackingTorqueAdrcTuningConfig,
    *,
    stage: str,
    config_source: Path,
    coarse_study_dir: Path | None = None,
) -> Path:
    """执行一个调参阶段并生成可供下一阶段读取的候选排名。"""
    load_profile(config.profile)
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    candidates = _stage_candidates(config, stage, coarse_study_dir)
    stage_config = config.coarse if stage == "coarse" else config.confirm
    study_dir = _create_study_directory(config, stage)
    (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)
    rows: list[dict[str, object]] = []
    for candidate in candidates:
        override = TorqueAdrcControl(
            measurement_filter_cutoff_hz=candidate.measurement_filter_cutoff_hz,
            controller_bandwidth_rad_s=candidate.controller_bandwidth_rad_s,
            observer_bandwidth_rad_s=candidate.observer_bandwidth_rad_s,
        )
        for task_path, material, seed in product(
            config.tasks, stage_config.materials, stage_config.seeds.values()
        ):
            task = tasks[task_path]
            condition = f"{candidate.identifier}-{task_path.stem}-{material}-seed{seed:03d}"
            run, result = execute_force_tracking(
                profile=config.profile,
                task_path=task_path,
                tracking_task=task,
                output_root=study_dir / "runs",
                run_prefix=condition,
                object_material=material,
                controller_variant="adrc-torque",
                stiffness_estimator_method=config.stiffness_estimator_method,
                sensor_noise_seed=seed,
                torque_adrc_override=override,
            )
            rows.append(
                {
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
            )
    aggregates = _aggregate(rows)
    ranking = rank_candidates(
        aggregates,
        candidates,
        baseline=config.baseline,
        max_torque_saturation_ratio=config.constraints.max_torque_saturation_ratio,
        max_ramp_rmse_ratio_to_baseline=config.constraints.max_ramp_rmse_ratio_to_baseline,
        max_mixed_rmse_ratio_to_baseline=config.constraints.max_mixed_rmse_ratio_to_baseline,
    )
    summary_csv, summary_parquet = write_rows_csv_and_parquet(study_dir / "summary.csv", rows)
    aggregate_csv, aggregate_parquet = write_rows_csv_and_parquet(
        study_dir / "aggregate.csv", aggregates
    )
    ranking_csv = write_rows_csv(study_dir / "candidate_ranking.csv", ranking)
    summary_json = study_dir / "summary.json"
    summary_json.write_text(
        json.dumps({"runs": rows, "aggregates": aggregates, "ranking": ranking}, indent=2) + "\n",
        encoding="utf-8",
    )
    figure_artifacts = render_study_figures(
        aggregates,
        ranking,
        study_dir,
        max_torque_saturation_ratio=config.constraints.max_torque_saturation_ratio,
        max_ramp_rmse_ratio_to_baseline=config.constraints.max_ramp_rmse_ratio_to_baseline,
        max_mixed_rmse_ratio_to_baseline=config.constraints.max_mixed_rmse_ratio_to_baseline,
    )
    (study_dir / "study_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": config.name,
                "stage": stage,
                "config": "study.yaml",
                "resolved_config": str(resolved_config.relative_to(study_dir)),
                "coarse_study_dir": None if coarse_study_dir is None else str(coarse_study_dir),
                "runs": [row["run_directory"] for row in rows],
                "artifacts": [
                    str(path.relative_to(study_dir))
                    for path in (
                        summary_csv,
                        summary_parquet,
                        aggregate_csv,
                        aggregate_parquet,
                        ranking_csv,
                        summary_json,
                        resolved_config,
                        *figure_artifacts,
                    )
                ],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return study_dir


def main() -> None:
    """解析参数并执行粗扫或确认阶段。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--stage", required=True, choices=("coarse", "confirm"))
    parser.add_argument("--coarse-study-dir", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    arguments = parser.parse_args()
    config_path = arguments.config.resolve()
    config = load_torque_adrc_tuning_config(config_path)
    if arguments.stage == "confirm" and arguments.coarse_study_dir is None:
        parser.error("--coarse-study-dir is required for confirm stage")
    coarse_study_dir = (
        None if arguments.coarse_study_dir is None else arguments.coarse_study_dir.resolve()
    )
    if arguments.dry_run:
        print(describe_conditions(config, stage=arguments.stage, coarse_study_dir=coarse_study_dir))
        return
    result = run_study(
        config,
        stage=arguments.stage,
        config_source=config_path,
        coarse_study_dir=coarse_study_dir,
    )
    print(f"Study: {result}")


if __name__ == "__main__":
    main()

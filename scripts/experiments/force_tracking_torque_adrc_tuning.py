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
from parallel_gripper_tactile.profiles import TorqueAdrcControl, load_profile
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.force_tracking_torque_adrc_tuning import (
    ForceTrackingTorqueAdrcTuningConfig,
    TorqueAdrcCandidate,
    load_torque_adrc_tuning_config,
)


METRICS = ("rmse_n", "overshoot_ratio", "settling_time_s", "torque_saturation_ratio")


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    """写入同构 CSV 行。"""
    if not rows:
        raise ValueError("cannot write empty CSV rows")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


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
    groups: dict[tuple[str, str, str], list[dict[str, object]]] = {}
    for row in rows:
        key = (str(row["candidate_id"]), str(row["task_name"]), str(row["object_material"]))
        groups.setdefault(key, []).append(row)
    aggregates: list[dict[str, object]] = []
    for (_, task_name, material), group in groups.items():
        first = group[0]
        aggregate: dict[str, object] = {
            "candidate_id": first["candidate_id"],
            "measurement_filter_cutoff_hz": first["measurement_filter_cutoff_hz"],
            "controller_bandwidth_rad_s": first["controller_bandwidth_rad_s"],
            "observer_bandwidth_ratio": first["observer_bandwidth_ratio"],
            "observer_bandwidth_rad_s": first["observer_bandwidth_rad_s"],
            "task_name": task_name,
            "object_material": material,
            "runs": len(group),
            "passed_runs": sum(bool(row["passed"]) for row in group),
        }
        for metric in METRICS:
            values = [
                float(row[metric])
                for row in group
                if row[metric] is not None and math.isfinite(float(row[metric]))
            ]
            aggregate[f"{metric}_mean"] = fmean(values) if values else None
            aggregate[f"{metric}_std"] = stdev(values) if len(values) >= 2 else None
        aggregates.append(aggregate)
    return aggregates


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
        feasible = (
            len(candidate_rows) > 0
            and all(int(row["passed_runs"]) == int(row["runs"]) for row in candidate_rows)
            and ramp_rmse is not None
            and mixed_rmse is not None
            and step_overshoot is not None
            and step_rmse is not None
            and max_saturation <= max_torque_saturation_ratio
            and ramp_rmse <= baseline_ramp * max_ramp_rmse_ratio_to_baseline
            and mixed_rmse <= baseline_mixed * max_mixed_rmse_ratio_to_baseline
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
    _write_csv(study_dir / "summary.csv", rows)
    _write_csv(study_dir / "aggregate.csv", aggregates)
    _write_csv(study_dir / "candidate_ranking.csv", ranking)
    (study_dir / "summary.json").write_text(
        json.dumps({"runs": rows, "aggregates": aggregates, "ranking": ranking}, indent=2) + "\n",
        encoding="utf-8",
    )
    (study_dir / "study_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": config.name,
                "stage": stage,
                "config": "study.yaml",
                "coarse_study_dir": None if coarse_study_dir is None else str(coarse_study_dir),
                "runs": [row["run_directory"] for row in rows],
                "artifacts": [
                    "summary.csv",
                    "aggregate.csv",
                    "candidate_ranking.csv",
                    "summary.json",
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

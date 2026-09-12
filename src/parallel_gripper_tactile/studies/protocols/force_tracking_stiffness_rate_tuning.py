"""运行刚度归一化速率控制器的小规模参数调优。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
from functools import partial
import json
import math
from pathlib import Path

import numpy as np

from parallel_gripper_tactile.config.profiles import GripperProfile, StiffnessRateControl
from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
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
from parallel_gripper_tactile.studies.force_tracking_stiffness_rate_tuning import (
    ForceTrackingStiffnessRateTuningConfig,
    StiffnessRateCandidate,
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
    read_trace_rows,
    write_resolved_config,
    write_rows_csv,
    write_rows_csv_and_parquet,
)
from parallel_gripper_tactile.visualization import (
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)


_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
_STUDY_KIND = "force_tracking_stiffness_rate_tuning"
_METRICS = (
    "rmse_n",
    "overshoot_ratio",
    "settling_time_s",
    "plateau_force_std_n",
    "plateau_force_peak_to_peak_n",
    "dominant_oscillation_amplitude_n",
)


def _platform_metrics(
    run_directory: Path,
    *,
    steady_window_s: tuple[float, float],
) -> dict[str, float]:
    """计算指定高力平台内的波动和 2～20 Hz 主谱线。"""
    start_s, end_s = steady_window_s
    samples = []
    for row in read_trace_rows(run_directory):
        if row.get("phase") != "track_reference":
            continue
        time_s = float(row["tracking_time_s"])
        force_n = float(row["filtered_normal_force_n"])
        if start_s <= time_s <= end_s and math.isfinite(force_n):
            samples.append((time_s, force_n))
    if len(samples) < 3:
        raise ValueError(f"稳态窗口内有效样本不足：{run_directory}")
    times = np.asarray([item[0] for item in samples], dtype=np.float64)
    forces = np.asarray([item[1] for item in samples], dtype=np.float64)
    dt_s = float(np.median(np.diff(times)))
    if not math.isfinite(dt_s) or dt_s <= 0:
        raise ValueError(f"稳态窗口采样周期无效：{run_directory}")
    centered = forces - np.mean(forces)
    spectrum = np.abs(np.fft.rfft(centered))
    frequencies = np.fft.rfftfreq(len(centered), dt_s)
    band = np.flatnonzero((frequencies >= 2.0) & (frequencies <= 20.0))
    if len(band) == 0:
        raise ValueError(f"稳态窗口不足以分析 2～20 Hz：{run_directory}")
    dominant = int(band[np.argmax(spectrum[band])])
    return {
        "plateau_force_std_n": float(np.std(forces)),
        "plateau_force_peak_to_peak_n": float(np.ptp(forces)),
        "dominant_oscillation_frequency_hz": float(frequencies[dominant]),
        "dominant_oscillation_amplitude_n": float(2.0 * spectrum[dominant] / len(forces)),
    }


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按控制器候选、任务和材料聚合配对 seed。"""
    return aggregate_records(
        rows,
        keys=("candidate_id", "task_name", "object_material"),
        columns=(
            first("candidate_id"),
            first("controller_variant"),
            first("kp_s_inv"),
            first("max_force_rate_n_s"),
            key("task_name"),
            key("object_material"),
            count("runs"),
            bool_sum("passed", "passed_runs"),
            *(stat for metric in _METRICS for stat in (finite_mean(metric), finite_std(metric))),
        ),
    )


def rank_candidates(
    aggregates: list[dict[str, object]],
    candidates: tuple[StiffnessRateCandidate, ...],
    *,
    max_plateau_force_std_n: float,
    max_overshoot_ratio: float,
) -> list[dict[str, object]]:
    """先应用稳定性与波动约束，再按 RMSE 排序候选。"""
    ranking: list[dict[str, object]] = []
    for candidate in candidates:
        rows = [row for row in aggregates if row["candidate_id"] == candidate.identifier]
        rmse = max((float(row["rmse_n_mean"]) for row in rows), default=math.inf)
        overshoot = max((float(row["overshoot_ratio_mean"]) for row in rows), default=math.inf)
        force_std = max((float(row["plateau_force_std_n_mean"]) for row in rows), default=math.inf)
        amplitude = max(
            (float(row["dominant_oscillation_amplitude_n_mean"]) for row in rows),
            default=math.inf,
        )
        feasible = (
            bool(rows)
            and all(int(row["passed_runs"]) == int(row["runs"]) for row in rows)
            and force_std <= max_plateau_force_std_n
            and overshoot <= max_overshoot_ratio
        )
        ranking.append(
            {
                "candidate_id": candidate.identifier,
                "kp_s_inv": candidate.kp_s_inv,
                "max_force_rate_n_s": candidate.max_force_rate_n_s,
                "feasible": str(feasible).lower(),
                "rmse_n": rmse,
                "overshoot_ratio": overshoot,
                "plateau_force_std_n": force_std,
                "dominant_oscillation_amplitude_n": amplitude,
            }
        )
    ranking.sort(
        key=lambda row: (
            row["feasible"] != "true",
            float(row["rmse_n"]),
            float(row["plateau_force_std_n"]),
        )
    )
    for index, row in enumerate(ranking, start=1):
        row["rank"] = index
    return ranking


def build_plan(
    config: ForceTrackingStiffnessRateTuningConfig,
    *,
    resolved_profile: GripperProfile | None = None,
) -> StudyPlan:
    """生成性能基线与速率参数网格的唯一有序计划。"""
    if resolved_profile is None:
        raise ValueError("resolved_profile is required")
    conditions = []
    for candidate, task, material, seed in config.conditions():
        identifier = "pid-torque-ff" if candidate is None else candidate.identifier
        conditions.append(
            StudyCondition(
                condition_id=f"{identifier}-{task.stem}-{material}-seed{seed:03d}",
                parameters={
                    "candidate_id": identifier,
                    "controller_variant": (
                        "pid-torque-ff" if candidate is None else "pid-stiffness-rate"
                    ),
                    "kp_s_inv": None if candidate is None else candidate.kp_s_inv,
                    "max_force_rate_n_s": (
                        None if candidate is None else candidate.max_force_rate_n_s
                    ),
                    "task_path": str(task),
                    "object_material": material,
                    "sensor_noise_seed": seed,
                },
                pair_key=f"{task.stem}:{material}:seed{seed:03d}",
                baseline_role="performance-baseline" if candidate is None else None,
            )
        )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "force_tracking_stiffness_rate_tuning.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": model_configuration_sha256(
                resolved_profile, repository_root=_REPOSITORY_ROOT
            ),
            "task_sha256": {str(task): file_sha256(task) for task in config.tasks},
        },
        "scientific_runtime": {
            "baseline": "pid-torque-ff",
            "candidate_controller": "pid-stiffness-rate",
            "ki_s_inv2": 0.0,
            "kd": 0.0,
            "trace_sample_period": "control_period_s",
            "spectral_band_hz": [2.0, 20.0],
        },
        "ranking_policy": {
            "constraints": {
                "max_plateau_force_std_n": config.max_plateau_force_std_n,
                "max_overshoot_ratio": config.max_overshoot_ratio,
            },
            "ordering": "feasible,rmse,plateau_force_std; stable v1",
        },
    }
    definition_hash = scientific_configuration_hash(definition, repository_root=_REPOSITORY_ROOT)
    plan_hash = scientific_configuration_hash(
        {
            "study_definition_sha256": definition_hash,
            "conditions": [condition.model_dump(mode="python") for condition in conditions],
        },
        repository_root=_REPOSITORY_ROOT,
    )
    return StudyPlan(
        study_kind=_STUDY_KIND,
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=tuple(conditions),
        seeds=tuple(config.seeds.values()),
        preflight={"status": "pending", "checks": ["profile", "tasks", "scenes"]},
    )


def _execute_condition(
    condition: StudyCondition,
    *,
    config: ForceTrackingStiffnessRateTuningConfig,
    resolved_profile: GripperProfile,
    tasks: Mapping[Path, ForceTrackingTask],
    study_dir: Path,
) -> ConditionExecution:
    """执行一个基线或速率参数候选条件。"""
    parameters = condition.parameters
    task_path = Path(str(parameters["task_path"]))
    task = tasks[task_path]
    controller = str(parameters["controller_variant"])
    override = None
    if controller == "pid-stiffness-rate":
        override = StiffnessRateControl(
            kp_s_inv=float(parameters["kp_s_inv"]),
            ki_s_inv2=0.0,
            kd=0.0,
            max_force_rate_n_s=float(parameters["max_force_rate_n_s"]),
            max_joint_velocity_rad_s=config.max_joint_velocity_rad_s,
        )
    run, result = execute_force_tracking(
        profile=config.profile,
        resolved_profile=resolved_profile,
        task_path=task_path,
        tracking_task=task,
        output_root=study_dir / "runs",
        run_prefix=condition.condition_id,
        object_material=str(parameters["object_material"]),
        controller_variant=controller,
        stiffness_estimator_method=config.stiffness_estimator_method,
        sensor_noise_seed=int(parameters["sensor_noise_seed"]),
        stiffness_rate_override=override,
        trace_sample_period_s=task.control_period_s,
    )
    row = {
        **dict(parameters),
        "task_name": task.name,
        "passed": result.passed,
        "run_directory": str(run.path.relative_to(study_dir)),
        **asdict(result),
        **_platform_metrics(run.path, steady_window_s=config.steady_window_s),
    }
    return ConditionExecution(
        row=row,
        run_directory=str(row["run_directory"]),
        passed=result.passed,
    )


def _render_heatmaps(
    aggregates: list[dict[str, object]],
    config: ForceTrackingStiffnessRateTuningConfig,
    output: Path,
) -> Path:
    """绘制三个参数—性能热图，坐标使用物理符号。"""
    candidate_rows = [
        row for row in aggregates if row["controller_variant"] == "pid-stiffness-rate"
    ]
    lookup = {
        (float(row["kp_s_inv"]), float(row["max_force_rate_n_s"])): row for row in candidate_rows
    }
    metrics = (
        ("rmse_n_mean", r"RMSE $\;\mathrm{(N)}$"),
        ("plateau_force_std_n_mean", r"$\sigma_F\;\mathrm{(N)}$"),
        ("overshoot_ratio_mean", r"$M_p$"),
    )
    plt = science_pyplot()
    figure, axes = plt.subplots(1, 3, figsize=paper_figsize(3.0, columns=2), layout="constrained")
    for axis, (metric, title) in zip(axes, metrics, strict=True):
        values = np.asarray(
            [
                [float(lookup[(kp, rate)][metric]) for rate in config.max_force_rate_n_s]
                for kp in config.kp_s_inv
            ]
        )
        image = axis.imshow(values, origin="lower", aspect="auto", cmap="viridis")
        axis.set_xticks(range(len(config.max_force_rate_n_s)), config.max_force_rate_n_s)
        axis.set_yticks(range(len(config.kp_s_inv)), config.kp_s_inv)
        axis.set_xlabel(r"$\dot F_{\max}\;\mathrm{(N\,s^{-1})}$")
        axis.set_ylabel(r"$K_P\;\mathrm{(s^{-1})}$")
        axis.set_title(title)
        for row_index in range(values.shape[0]):
            for column_index in range(values.shape[1]):
                axis.text(
                    column_index,
                    row_index,
                    f"{values[row_index, column_index]:.3f}",
                    ha="center",
                    va="center",
                    color="white"
                    if values[row_index, column_index] > np.nanmean(values)
                    else "black",
                    fontsize=7,
                )
        figure.colorbar(image, ax=axis, shrink=0.78)
    path = save_publication_figure(figure, output)
    plt.close(figure)
    return path


def run_study(
    config: ForceTrackingStiffnessRateTuningConfig,
    *,
    config_source: Path,
    resolved_profile: GripperProfile,
    study_directory: Path,
    study_plan: StudyPlan,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
    workers: int = 1,
) -> Path:
    """执行调优、生成聚合表、候选排名和参数热图。"""
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    directory = study_directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "study.yaml").write_bytes(config_source.read_bytes())
    resolved_config = write_resolved_config(directory / "study.resolved.json", config)
    expected_plan = build_plan(config, resolved_profile=resolved_profile)
    plan = require_matching_study_plan(expected_plan, study_plan)
    execute = partial(
        _execute_condition,
        config=config,
        resolved_profile=resolved_profile,
        tasks=tasks,
        study_dir=directory,
    )

    def aggregate_and_persist(
        rows: list[dict[str, object]],
        outcomes: tuple[ConditionOutcome, ...],
        output_directory: Path,
    ) -> StudyPostprocessResult:
        aggregates = aggregate_rows(rows)
        ranking = rank_candidates(
            aggregates,
            config.candidates(),
            max_plateau_force_std_n=config.max_plateau_force_std_n,
            max_overshoot_ratio=config.max_overshoot_ratio,
        )
        artifacts = [
            *write_rows_csv_and_parquet(output_directory / "summary.csv", rows),
            *write_rows_csv_and_parquet(output_directory / "aggregate.csv", aggregates),
            write_rows_csv(output_directory / "candidate_ranking.csv", ranking),
        ]
        summary_path = output_directory / "summary.json"
        summary_path.write_text(
            json.dumps(
                {
                    "runs": rows,
                    "aggregates": aggregates,
                    "candidate_ranking": ranking,
                    "failures": execution_failure_rows(outcomes),
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        artifacts.append(summary_path)
        return StudyPostprocessResult(tuple(artifacts), {}, aggregates)

    def render(
        rows: list[dict[str, object]], payload: object, output_directory: Path
    ) -> tuple[Path, ...]:
        del rows
        aggregates = list(payload) if isinstance(payload, list) else []
        return (
            _render_heatmaps(
                aggregates,
                config,
                output_directory / "figures/stiffness_rate_tuning.png",
            ),
        )

    manifest_fields = {
        "schema_version": 1,
        "name": config.name,
        "config": "study.yaml",
        "resolved_config": resolved_config.name,
        **(lifecycle_manifest_fields or {}),
    }
    return execute_study_lifecycle(
        plan,
        study_directory=directory,
        execute_condition=execute,
        aggregate_and_persist=aggregate_and_persist,
        render=render,
        initial_artifacts=(directory / "study.yaml", resolved_config, *additional_artifacts),
        legacy_manifest_fields=manifest_fields,
        workers=workers,
    )


__all__ = ["aggregate_rows", "build_plan", "rank_candidates", "run_study"]

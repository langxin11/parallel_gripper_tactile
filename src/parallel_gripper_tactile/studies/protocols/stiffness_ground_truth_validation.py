"""运行等效接触刚度参考验证，并聚合估计精度与安全风险指标。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
from functools import partial
import json
import math
from pathlib import Path

from parallel_gripper_tactile.config.profiles import GripperProfile, load_profile
from parallel_gripper_tactile.experiments.stiffness_calibration import (
    StiffnessCalibrationTask,
    run_stiffness_calibration,
)
from parallel_gripper_tactile.studies.aggregation import (
    aggregate_records,
    bool_sum,
    count,
    finite_mean,
    finite_std,
    key,
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
from parallel_gripper_tactile.studies.stiffness_ground_truth_validation import (
    StiffnessGroundTruthValidationConfig,
)
from parallel_gripper_tactile.studies.tabular import (
    write_resolved_config,
    write_rows_csv,
    write_rows_csv_and_parquet,
)
from parallel_gripper_tactile.visualization import (
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)


METRICS = (
    "reference_stiffness_mean_n_per_m",
    "estimate_mean_n_per_m",
    "log_rmse",
    "relative_rmse",
    "relative_bias",
    "underestimation_ratio",
    "estimate_log_jitter",
    "force_increment_rmse_n",
    "force_increment_underprediction_p95_n",
    "loading_unloading_hysteresis_ratio",
    "reference_coverage",
    "estimator_valid_fraction",
)
_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按估计器和材料聚合真值误差、安全风险与稳定性。"""
    return aggregate_records(
        rows,
        keys=("stiffness_estimator_method", "object_material"),
        columns=(
            key("stiffness_estimator_method"),
            key("object_material"),
            count("runs"),
            bool_sum("passed", "passed_runs"),
            *(
                statistic
                for metric in METRICS
                for statistic in (finite_mean(metric), finite_std(metric))
            ),
        ),
    )


def plot_accuracy_summary(
    aggregates: list[dict[str, object]],
    output: Path,
    *,
    estimator_order: Sequence[str],
) -> Path:
    """绘制对数误差、刚度低估率与增量力预测误差。"""
    if not aggregates:
        raise ValueError("cannot plot empty stiffness aggregates")
    plt = science_pyplot()
    materials = tuple(dict.fromkeys(str(row["object_material"]) for row in aggregates))
    lookup = {
        (str(row["stiffness_estimator_method"]), str(row["object_material"])): row
        for row in aggregates
    }
    metrics = (
        ("log_rmse_mean", r"$\mathrm{RMSE}_{\log k}$"),
        ("underestimation_ratio_mean", r"$P(\hat{k}<k_{\mathrm{ref}})$"),
        ("force_increment_underprediction_p95_n_mean", r"$Q_{0.95}(e_{\Delta f}^{+})$ (N)"),
    )
    colors = ("#0072B2", "#E69F00", "#009E73")
    import numpy as np

    x = np.arange(len(estimator_order), dtype=np.float64)
    width = 0.8 / len(materials)
    figure, axes = plt.subplots(1, 3, figsize=paper_figsize(3.3), layout="constrained")
    for axis, (metric, label) in zip(axes, metrics):
        for material_index, material in enumerate(materials):
            positions = x - 0.4 + width / 2.0 + material_index * width
            raw_values = [
                lookup.get((estimator, material), {}).get(metric) for estimator in estimator_order
            ]
            values = [float(value) if value is not None else float("nan") for value in raw_values]
            axis.bar(
                positions,
                values,
                width=width,
                label=material,
                color=colors[material_index % len(colors)],
                hatch=("", "//", "xx")[material_index % 3],
                edgecolor="black",
                linewidth=0.4,
            )
        axis.set_xticks(x, estimator_order, rotation=20, ha="right")
        axis.set_ylabel(label)
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    figure.legend(*axes[0].get_legend_handles_labels(), loc="outside upper center", ncol=3)
    path = save_publication_figure(figure, output)
    plt.close(figure)
    return path


def build_plan(
    config: StiffnessGroundTruthValidationConfig,
    *,
    resolved_profile: GripperProfile | None = None,
) -> StudyPlan:
    """从权威配置生成准静态真值验证的唯一有序计划。"""
    profile = resolved_profile or load_profile(config.profile)
    conditions = tuple(
        StudyCondition(
            condition_id=f"{estimator}-{material}-seed{seed:03d}",
            parameters={
                "stiffness_estimator_method": estimator,
                "object_material": material,
                "sensor_noise_seed": seed,
                "task_path": str(config.task),
            },
            pair_key=f"{material}:seed{seed:03d}",
            baseline_role="secant_ewma" if estimator == "secant_ewma" else None,
        )
        for estimator, material, seed in config.conditions()
    )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "stiffness_ground_truth_validation.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": model_configuration_sha256(profile, repository_root=_REPOSITORY_ROOT),
            "task_sha256": file_sha256(config.task),
        },
        "reference": "quasistatic branch-wise centered finite difference v1",
        "aggregation": "stiffness_estimator_method,object_material; finite v1",
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
        study_kind="stiffness_ground_truth_validation",
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=conditions,
        seeds=tuple(config.seeds.values()),
        preflight={"status": "pending", "checks": ["profile", "task", "estimators"]},
    )


def _execute_condition(
    condition: StudyCondition,
    *,
    config: StiffnessGroundTruthValidationConfig,
    resolved_profile: GripperProfile,
    task: StiffnessCalibrationTask,
    study_dir: Path,
) -> ConditionExecution:
    """在独立目录执行一个估计器—材料—seed 准静态扫描。"""
    parameters = condition.parameters
    estimator = str(parameters["stiffness_estimator_method"])
    material = str(parameters["object_material"])
    seed = int(parameters["sensor_noise_seed"])
    run_dir = study_dir / "runs" / condition.condition_id
    run_dir.mkdir(parents=True, exist_ok=False)
    points, result = run_stiffness_calibration(
        resolved_profile,
        task=task,
        estimator_method=estimator,
        object_material=material,
        sensor_noise_seed=seed,
    )
    trace_path = run_dir / "equilibrium_points.csv"
    if points:
        write_rows_csv(trace_path, points)
    else:
        trace_path.write_text("branch,closure_m,normal_force_n\n", encoding="utf-8")
    metrics = {
        name: None if isinstance(value, float) and not math.isfinite(value) else value
        for name, value in asdict(result).items()
    }
    (run_dir / "profile.resolved.json").write_text(
        resolved_profile.model_dump_json(indent=2) + "\n", encoding="utf-8"
    )
    metrics_path = run_dir / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    row = {
        "stiffness_estimator_method": estimator,
        "object_material": material,
        "sensor_noise_seed": seed,
        "passed": result.passed,
        "run_directory": str(run_dir.relative_to(study_dir)),
        **metrics,
    }
    return ConditionExecution(
        row=row,
        run_directory=str(run_dir.relative_to(study_dir)),
        passed=result.passed,
        metadata={
            "artifacts": [
                str(trace_path),
                str(metrics_path),
                str(run_dir / "profile.resolved.json"),
            ]
        },
    )


def run_study(
    config: StiffnessGroundTruthValidationConfig,
    *,
    resolved_profile: GripperProfile,
    config_source: Path | None = None,
    study_directory: Path,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
    workers: int = 1,
) -> Path:
    """通过公共生命周期执行准静态真值验证研究。"""
    study_dir = study_directory.resolve()
    study_dir.mkdir(parents=True, exist_ok=True)
    task = StiffnessCalibrationTask.load(config.task)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        (study_dir / "study.yaml").write_text(
            config.model_dump_json(indent=2) + "\n", encoding="utf-8"
        )
    task_snapshot = study_dir / "task.yaml"
    task_snapshot.write_bytes(config.task.read_bytes())
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)
    expected_plan = build_plan(config, resolved_profile=resolved_profile)
    plan = (
        expected_plan
        if study_plan is None
        else require_matching_study_plan(expected_plan, study_plan)
    )
    execute = partial(
        _execute_condition,
        config=config,
        resolved_profile=resolved_profile,
        task=task,
        study_dir=study_dir,
    )
    condition_artifacts: list[Path] = []

    def record_condition(execution: ConditionExecution) -> None:
        """把条件目录中的平衡点与指标登记到 study 产物账本。"""
        artifacts = execution.metadata.get("artifacts", ())
        condition_artifacts.extend(Path(str(path)) for path in artifacts)

    def aggregate_and_persist(
        rows: list[dict[str, object]],
        outcomes: tuple[ConditionOutcome, ...],
        directory: Path,
    ) -> StudyPostprocessResult:
        aggregates = aggregate_rows(rows) if rows else []
        artifacts: list[Path] = []
        if rows:
            artifacts.extend(write_rows_csv_and_parquet(directory / "summary.csv", rows))
        if aggregates:
            artifacts.extend(write_rows_csv_and_parquet(directory / "aggregate.csv", aggregates))
        summary = directory / "summary.json"
        summary.write_text(
            json.dumps(
                {
                    "runs": rows,
                    "aggregates": aggregates,
                    "failures": execution_failure_rows(outcomes),
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        artifacts.append(summary)
        artifacts.extend(condition_artifacts)
        return StudyPostprocessResult(tuple(artifacts), {}, aggregates)

    def render(rows: list[dict[str, object]], payload: object, directory: Path) -> tuple[Path, ...]:
        if not rows:
            return ()
        aggregates = list(payload) if isinstance(payload, list) else []
        figures = directory / "figures"
        figures.mkdir(exist_ok=True)
        return (
            plot_accuracy_summary(
                aggregates,
                figures / "estimator_accuracy.png",
                estimator_order=config.estimators,
            ),
        )

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
        initial_artifacts=(
            study_dir / "study.yaml",
            task_snapshot,
            resolved_config,
            *additional_artifacts,
        ),
        legacy_manifest_fields=manifest_fields,
        workers=workers,
        record_condition_execution=record_condition,
    )


__all__ = ["METRICS", "aggregate_rows", "build_plan", "plot_accuracy_summary", "run_study"]

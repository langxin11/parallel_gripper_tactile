"""运行刚度位置限幅三臂配对实验。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
from functools import partial
import json
import math
from pathlib import Path

import numpy as np

from parallel_gripper_tactile.config.profiles import GripperProfile
from parallel_gripper_tactile.experiments.force_tracking import (
    ForceTrackingTask,
    configure_force_controller,
)
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.aggregation import (
    aggregate_records,
    bool_sum,
    count,
    finite_mean,
    finite_std,
    key,
    nan_mean,
    nan_std,
)
from parallel_gripper_tactile.studies.force_tracking_stiffness_limit import (
    ForceTrackingStiffnessLimitConfig,
    StiffnessLimitMode,
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
    write_rows_csv_and_parquet,
)
from parallel_gripper_tactile.visualization import (
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)


_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
_BASE_METRICS = (
    "rmse_n",
    "mae_n",
    "peak_abs_error_n",
    "overshoot_ratio",
    "settling_time_s",
    "torque_saturation_ratio",
    "position_saturation_ratio",
    "stiffness_position_limit_ratio",
    "contact_window_peak_force_n",
    "max_positive_force_rate_n_s",
)
_NULLABLE_METRICS = ("overshoot_ratio", "settling_time_s")


def configured_mode_profile(
    base_profile: GripperProfile,
    *,
    mode: StiffnessLimitMode,
    material: str,
    seed: int,
    oracle_values: Mapping[str, float],
    force_rate_limit_n_s: float,
) -> GripperProfile:
    """构造三臂中一个条件的完整 profile。"""
    variant = "pid-torque-ff" if mode == "no-limit" else "pid-stiffness-limit"
    profile = configure_force_controller(
        base_profile,
        variant=variant,
        stiffness_estimator_method="window_linear",
        sensor_noise_seed=seed,
    )
    if profile.normal_force is None or profile.normal_force.stiffness is None:
        raise ValueError("stiffness-limit study requires contact stiffness control")
    stiffness = profile.normal_force.stiffness.model_copy(
        update={"position_limit_force_rate_n_s": force_rate_limit_n_s}
    )
    force = profile.normal_force.model_copy(update={"stiffness": stiffness})
    profile = profile.model_copy(
        update={"control": profile.control.model_copy(update={"force": force})}
    )
    if mode != "oracle-k":
        return profile
    oracle = float(oracle_values[material])
    # 以超出机构与任务量程的更新阈值冻结估计值；该值只注入限幅路径，
    # 不改变 PID、机构力矩前馈或接触动力学。
    stiffness = profile.normal_force.stiffness.model_copy(
        update={
            "initial_n_per_m": oracle,
            "min_n_per_m": oracle * 0.5,
            "max_n_per_m": oracle * 2.0,
            "min_delta_closure_m": 1.0,
            "min_delta_force_n": 1e6,
        }
    )
    force = profile.normal_force.model_copy(update={"stiffness": stiffness})
    return profile.model_copy(
        update={"control": profile.control.model_copy(update={"force": force})}
    )


def trace_safety_metrics(
    rows: list[dict[str, object]], *, contact_time_s: float, contact_window_s: float
) -> dict[str, float]:
    """从控制频率轨迹计算接触窗口峰值与最大正力增长率。"""
    contact = [
        row
        for row in rows
        if contact_time_s <= float(row["time_s"]) <= contact_time_s + contact_window_s
    ]
    tracking = [row for row in rows if row["phase"] == "track_reference"]
    if not contact or len(tracking) < 2:
        return {
            "contact_window_peak_force_n": math.nan,
            "max_positive_force_rate_n_s": math.nan,
        }
    times = np.asarray([float(row["time_s"]) for row in tracking], dtype=np.float64)
    forces = np.asarray(
        [float(row["filtered_normal_force_n"]) for row in tracking], dtype=np.float64
    )
    dt = np.diff(times)
    valid = dt > 0.0
    rates = np.diff(forces)[valid] / dt[valid]
    return {
        "contact_window_peak_force_n": max(
            float(row["filtered_normal_force_n"]) for row in contact
        ),
        "max_positive_force_rate_n_s": (
            float(np.max(np.maximum(rates, 0.0))) if rates.size else math.nan
        ),
    }


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按限幅模式、任务和材料聚合配对重复。"""
    return aggregate_records(
        rows,
        keys=("limit_mode", "task_name", "object_material"),
        columns=(
            key("limit_mode"),
            key("task_name"),
            key("object_material"),
            count("runs"),
            bool_sum("passed", "passed_runs"),
            *(
                statistic
                for metric in _BASE_METRICS
                for statistic in (
                    (nan_mean(metric), nan_std(metric))
                    if metric in _NULLABLE_METRICS
                    else (finite_mean(metric), finite_std(metric))
                )
            ),
        ),
    )


def build_plan(
    config: ForceTrackingStiffnessLimitConfig,
    *,
    resolved_profile: GripperProfile,
) -> StudyPlan:
    """生成唯一的三臂有序计划。"""
    conditions = tuple(
        StudyCondition(
            condition_id=f"{mode}-{task.stem}-{material}-seed{seed:03d}",
            parameters={
                "limit_mode": mode,
                "task_path": str(task),
                "object_material": material,
                "sensor_noise_seed": seed,
                "oracle_stiffness_n_per_m": config.oracle_stiffness_n_per_m[material],
                "position_limit_force_rate_n_s": config.position_limit_force_rate_n_s,
            },
            pair_key=f"{task.stem}:{material}:seed{seed:03d}",
            baseline_role="no-limit" if mode == "no-limit" else None,
        )
        for mode, task, material, seed in config.conditions()
    )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "force_tracking_stiffness_limit.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": model_configuration_sha256(
                resolved_profile, repository_root=_REPOSITORY_ROOT
            ),
            "task_sha256": {str(task): file_sha256(task) for task in config.tasks},
        },
        "oracle_semantics": "material-level quasistatic mean; frozen estimator input",
        "trace_metrics": "control-rate filtered force; finite difference v1",
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
        study_kind="force_tracking_stiffness_limit",
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=conditions,
        seeds=tuple(config.seeds.values()),
        preflight={"status": "pending", "checks": ["profile", "tasks", "oracle-values"]},
    )


def _execute_condition(
    condition: StudyCondition,
    *,
    config: ForceTrackingStiffnessLimitConfig,
    resolved_profile: GripperProfile,
    tasks: Mapping[Path, ForceTrackingTask],
    study_dir: Path,
) -> ConditionExecution:
    """运行一个模式—任务—材料—seed 条件。"""
    parameters = condition.parameters
    mode = str(parameters["limit_mode"])
    task_path = Path(str(parameters["task_path"]))
    material = str(parameters["object_material"])
    seed = int(parameters["sensor_noise_seed"])
    profile = configured_mode_profile(
        resolved_profile,
        mode=mode,
        material=material,
        seed=seed,
        oracle_values=config.oracle_stiffness_n_per_m,
        force_rate_limit_n_s=config.position_limit_force_rate_n_s,
    )
    controller_variant = "pid-torque-ff" if mode == "no-limit" else "pid-stiffness-limit"
    run, result = execute_force_tracking(
        profile=config.profile,
        resolved_profile=profile,
        task_path=task_path,
        tracking_task=tasks[task_path],
        output_root=study_dir / "runs",
        run_prefix=condition.condition_id,
        object_material=material,
        controller_variant=controller_variant,
        stiffness_estimator_method="window_linear",
        sensor_noise_seed=seed,
        trace_sample_period_s=tasks[task_path].control_period_s,
    )
    safety = trace_safety_metrics(
        read_trace_rows(run.path),
        contact_time_s=result.contact_time_s,
        contact_window_s=config.contact_peak_window_s,
    )
    row = {
        "limit_mode": mode,
        "controller_variant": controller_variant,
        "stiffness_estimator_method": "window_linear",
        "task_name": tasks[task_path].name,
        "task_path": str(task_path),
        "object_material": material,
        "sensor_noise_seed": seed,
        "oracle_stiffness_n_per_m": parameters["oracle_stiffness_n_per_m"],
        "position_limit_force_rate_n_s": parameters["position_limit_force_rate_n_s"],
        "passed": result.passed,
        "run_directory": str(run.path.relative_to(study_dir)),
        **asdict(result),
        **safety,
    }
    return ConditionExecution(
        row=row,
        run_directory=str(row["run_directory"]),
        passed=result.passed,
    )


def plot_summary(aggregates: list[dict[str, object]], output: Path) -> Path:
    """绘制三模式的接触峰值、最大力增长率与 RMSE。"""
    plt = science_pyplot()
    modes = ("no-limit", "window-linear", "oracle-k")
    materials = tuple(dict.fromkeys(str(row["object_material"]) for row in aggregates))
    lookup = {(str(row["limit_mode"]), str(row["object_material"])): row for row in aggregates}
    metrics = (
        ("contact_window_peak_force_n_mean", r"$F_{n,\max}$ (N)"),
        ("max_positive_force_rate_n_s_mean", r"$\max(\dot{F}_n^+)$ (N/s)"),
        ("rmse_n_mean", "RMSE (N)"),
    )
    figure, axes = plt.subplots(1, 3, figsize=paper_figsize(3.5), layout="constrained")
    x = np.arange(len(modes))
    width = 0.8 / len(materials)
    colors = ("#0072B2", "#E69F00", "#009E73")
    hatches = ("", "//", "xx")
    for axis, (metric, label) in zip(axes, metrics):
        for index, material in enumerate(materials):
            values = [float(lookup[(mode, material)][metric]) for mode in modes]
            error_metric = metric.removesuffix("_mean") + "_std"
            errors = [float(lookup[(mode, material)][error_metric]) for mode in modes]
            axis.bar(
                x - 0.4 + width / 2 + index * width,
                values,
                width,
                yerr=errors,
                capsize=2,
                label=material,
                color=colors[index],
                hatch=hatches[index],
                edgecolor="black",
                linewidth=0.4,
            )
        axis.set_xticks(x, modes, rotation=18, ha="right")
        axis.set_ylabel(label)
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    figure.legend(*axes[0].get_legend_handles_labels(), loc="outside upper center", ncol=3)
    path = save_publication_figure(figure, output)
    plt.close(figure)
    return path


def run_study(
    config: ForceTrackingStiffnessLimitConfig,
    *,
    resolved_profile: GripperProfile,
    config_source: Path,
    study_directory: Path,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
    workers: int = 1,
) -> Path:
    """通过公共生命周期执行限幅研究。"""
    study_dir = study_directory.resolve()
    study_dir.mkdir(parents=True, exist_ok=True)
    (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)
    tasks = {path: ForceTrackingTask.load(path) for path in config.tasks}
    expected = build_plan(config, resolved_profile=resolved_profile)
    plan = expected if study_plan is None else require_matching_study_plan(expected, study_plan)
    execute = partial(
        _execute_condition,
        config=config,
        resolved_profile=resolved_profile,
        tasks=tasks,
        study_dir=study_dir,
    )

    def aggregate_and_persist(
        rows: list[dict[str, object]], outcomes: tuple[ConditionOutcome, ...], directory: Path
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
        return StudyPostprocessResult(tuple(artifacts), {}, aggregates)

    def render(rows: list[dict[str, object]], payload: object, directory: Path) -> tuple[Path, ...]:
        aggregates = list(payload) if isinstance(payload, list) else []
        if not rows or not aggregates:
            return ()
        figures = directory / "figures"
        figures.mkdir(exist_ok=True)
        return (plot_summary(aggregates, figures / "stiffness_limit_summary.png"),)

    manifest = {"schema_version": 1, "name": config.name, "config": "study.yaml"}
    manifest.update(lifecycle_manifest_fields or {})
    return execute_study_lifecycle(
        plan,
        study_directory=study_dir,
        execute_condition=execute,
        aggregate_and_persist=aggregate_and_persist,
        render=render,
        initial_artifacts=(study_dir / "study.yaml", resolved_config, *additional_artifacts),
        legacy_manifest_fields=manifest,
        workers=workers,
    )


__all__ = [
    "aggregate_rows",
    "build_plan",
    "configured_mode_profile",
    "plot_summary",
    "run_study",
    "trace_safety_metrics",
]

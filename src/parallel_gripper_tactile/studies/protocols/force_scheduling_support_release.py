"""运行自适应撤支撑调度的对照臂 × seed 验证研究。"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict
from datetime import UTC, datetime
from functools import partial
import json
import math
from pathlib import Path
from typing import Literal
from uuid import uuid4

import numpy as np
import yaml

from parallel_gripper_tactile.config.profiles import GripperProfile, load_profile
from parallel_gripper_tactile.runners import execute_force_scheduling
from parallel_gripper_tactile.studies.aggregation import (
    aggregate_records,
    bool_rate,
    bool_sum,
    count,
    first,
    key,
    optional_mean,
    optional_std,
)
from parallel_gripper_tactile.studies.force_scheduling_support_release import (
    ForceSchedulingSupportReleaseConfig,
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
    model_configuration_sha256,
    require_matching_study_plan,
    scientific_configuration_hash,
)
from parallel_gripper_tactile.studies.tabular import (
    write_resolved_config,
    write_rows_csv_and_parquet,
)
from parallel_gripper_tactile.visualization import (
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)


_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


ArmComposer = Callable[..., object]


def build_plan(
    config: ForceSchedulingSupportReleaseConfig,
    *,
    resolved_profile: GripperProfile | None = None,
    arm_composer: ArmComposer | None = None,
) -> StudyPlan:
    """逐臂完成一次组合预检，并生成唯一有序计划。

    组合器由研究层注入，协议自身不依赖 research 组合服务。
    """
    if arm_composer is None:
        raise ValueError("support-release protocol requires an injected arm composer")
    base_profile = resolved_profile or load_profile(config.profile)
    composed_arms = {
        arm.name: arm_composer(arm, seed=config.seeds.values()[0]) for arm in config.arms
    }
    conditions = tuple(
        StudyCondition(
            condition_id=f"{arm.name}-seed{seed:03d}",
            parameters={
                "arm": arm.name,
                "experiment": arm.experiment,
                "controller_variant": composed_arms[arm.name].selection.controller.name,
                "scheduler": composed_arms[arm.name].selection.scheduler.name,
                "task_path": str(composed_arms[arm.name].task_source),
                "sensor_noise_seed": seed,
            },
            pair_key=f"{arm.name}:seed{seed:03d}",
            baseline_role=None,
        )
        for arm, seed in config.conditions()
    )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "force_scheduling_support_release.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": model_configuration_sha256(
                base_profile, repository_root=_REPOSITORY_ROOT
            ),
            "arms": {
                arm.name: {
                    "experiment": arm.experiment,
                    "controller_variant": composed_arms[arm.name].selection.controller.name,
                    "scheduler": composed_arms[arm.name].selection.scheduler.name,
                }
                for arm in config.arms
            },
        },
        "validation": {"aggregation": "arm; pass/slip rates and force margins v1"},
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
        study_kind="force_scheduling_support_release",
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=conditions,
        seeds=tuple(config.seeds.values()),
        preflight={"status": "pending", "checks": ["profile", "arm_compositions"]},
    )


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按对照臂汇总通过率、滑移率与力裕度。"""
    return aggregate_records(
        rows,
        keys=("arm",),
        columns=(
            key("arm"),
            first("controller_variant"),
            first("scheduler"),
            count("runs"),
            bool_sum("passed", "passed_runs"),
            bool_rate("passed", "pass_rate"),
            bool_rate("slip_passed", "slip_pass_rate"),
            optional_mean("final_target_force_n"),
            optional_mean("peak_target_force_n"),
            optional_mean("force_tracking_rmse_n"),
            optional_std("force_tracking_rmse_n"),
            optional_mean("minimum_friction_margin_n"),
            optional_std("minimum_friction_margin_n"),
        ),
    )


def json_compatible(value: object) -> object:
    """把非有限浮点数转换为标准 JSON 的 ``null``。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_compatible(item) for item in value]
    return value


def plot_summary(rows: list[dict[str, object]], output: Path) -> Path:
    """按臂绘制最终目标力与最小摩擦裕度的均值±标准差。"""
    plt = science_pyplot()
    aggregated = aggregate_rows(rows)
    labels = [str(row["arm"]) for row in aggregated]
    x = np.arange(len(aggregated))
    figure, axes = plt.subplots(1, 2, figsize=paper_figsize(3.2), layout="constrained")
    for axis, field, label in (
        (axes[0], "final_target_force_n", "Final target force (N)"),
        (axes[1], "minimum_friction_margin_n", "Min friction margin (N)"),
    ):
        means = np.asarray([float(row.get(f"{field}_mean") or 0.0) for row in aggregated])
        stds = np.asarray([float(row.get(f"{field}_std") or 0.0) for row in aggregated])
        axis.bar(x, means, yerr=stds, capsize=2.5, color="#0072B2", alpha=0.85, width=0.6)
        axis.axhline(0.0, color="0.35", linestyle="--", linewidth=0.6)
        axis.set_ylabel(label)
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    for axis in axes:
        axis.set_xticks(x, labels, rotation=25, ha="right")
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def render_study_figures(rows: list[dict[str, object]], study_dir: Path) -> list[Path]:
    """从 study 逐 run 行生成对照臂汇总图。"""
    return [plot_summary(rows, study_dir / "support_release_validation.png")]


def _create_study_directory(config: ForceSchedulingSupportReleaseConfig) -> Path:
    """创建一次独占的 study 目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def _execute_condition(
    condition: StudyCondition,
    *,
    config: ForceSchedulingSupportReleaseConfig,
    study_dir: Path,
    plot_mode: Literal["summary", "diagnostic"],
    diagnostic_seed: int,
    arm_composer: ArmComposer,
) -> ConditionExecution:
    """在独立进程中执行一个对照臂条件。"""
    parameters = condition.parameters
    arm = next(item for item in config.arms if item.name == parameters["arm"])
    seed = int(parameters["sensor_noise_seed"])
    resolved = arm_composer(arm, seed=seed)

    run, result = execute_force_scheduling(
        profile=resolved.profile_source,
        resolved_profile=resolved.profile,
        task_path=resolved.task_source,
        scheduling_task=resolved.task,
        scheduler_config=resolved.scheduler,
        scheduler_source=resolved.scheduler_source,
        output_root=study_dir / "runs",
        run_prefix=condition.condition_id,
    )
    row = {
        "arm": arm.name,
        "controller_variant": parameters["controller_variant"],
        "scheduler": parameters["scheduler"],
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


def run_study(
    config: ForceSchedulingSupportReleaseConfig,
    *,
    resolved_profile: GripperProfile | None = None,
    arm_composer: ArmComposer | None = None,
    config_source: Path | None = None,
    study_directory: Path | None = None,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
    workers: int = 1,
    on_progress: StudyProgressCallback | None = None,
    plot_mode: Literal["summary", "diagnostic"] = "summary",
) -> Path:
    """通过公共生命周期执行整个 protocol，并返回 study 父目录。"""
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
    expected_plan = build_plan(config, resolved_profile=resolved_profile, arm_composer=arm_composer)
    plan = (
        expected_plan
        if study_plan is None
        else require_matching_study_plan(expected_plan, study_plan)
    )

    execute = partial(
        _execute_condition,
        config=config,
        study_dir=study_dir,
        plot_mode=plot_mode,
        diagnostic_seed=min(plan.seeds),
        arm_composer=arm_composer,
    )

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
        summary_json = directory / "summary.json"
        summary_json.write_text(
            json.dumps(
                json_compatible(
                    {
                        "runs": rows,
                        "aggregates": aggregates,
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
        manifest_fields = {
            "all_conditions_passed": all(bool(row["passed"]) for row in rows),
        }
        return StudyPostprocessResult(tuple(artifacts), manifest_fields, aggregates)

    def render(rows: list[dict[str, object]], payload: object, directory: Path) -> tuple[Path, ...]:
        if not rows:
            return ()
        return tuple(render_study_figures(rows, directory))

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


__all__ = [
    "aggregate_rows",
    "build_plan",
    "plot_summary",
    "render_study_figures",
    "run_study",
]

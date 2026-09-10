"""运行纯力局部起滑的场景 × seed 多种子与无起滑负例验证研究。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
from datetime import UTC, datetime
import json
import math
from pathlib import Path
from uuid import uuid4

import numpy as np
import yaml

from parallel_gripper_tactile.experiments.friction_estimation import FrictionEstimationTask
from parallel_gripper_tactile.runners import execute_friction_estimation
from parallel_gripper_tactile.studies.aggregation import (
    aggregate_records,
    bool_rate,
    bool_sum,
    count,
    first_bool,
    key,
    optional_mean,
    optional_std,
)
from parallel_gripper_tactile.studies.friction_estimation_local_slip import (
    FrictionEstimationLocalSlipStudyConfig,
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
# 控制候选资格沿用历史研究口径：局部估计比落在该区间才允许进入后续力控候选。
_CONTROL_CANDIDATE_RATIO_BOUNDS = (0.55, 1.02)


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按场景汇总检测率、误报率、提前量和局部候选下界。"""
    return aggregate_records(
        rows,
        keys=("scenario",),
        columns=(
            key("scenario"),
            first_bool("expect_local_slip"),
            count("runs"),
            bool_sum("local_slip_detected", "local_detections"),
            bool_rate("local_slip_detected", "detection_rate"),
            bool_rate(
                "local_slip_detected",
                "false_positive_rate",
                zero_when=("expect_local_slip", True),
            ),
            optional_mean("detection_lead_s"),
            optional_std("detection_lead_s"),
            optional_mean("local_estimate_ratio"),
            optional_std("local_estimate_ratio"),
            bool_sum("validation_passed", "passed_runs"),
            bool_sum("control_candidate_qualified", "control_qualified_runs"),
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
    """绘制逐 seed 检测时刻与局部摩擦候选比。"""
    plt = science_pyplot()
    scenarios = tuple(dict.fromkeys(str(row["scenario"]) for row in rows))
    x_lookup = {scenario: index for index, scenario in enumerate(scenarios)}
    figure, axes = plt.subplots(1, 2, figsize=paper_figsize(3.2), layout="constrained")
    for row in rows:
        x = x_lookup[str(row["scenario"])]
        local_time = row["local_slip_detection_time_s"]
        global_time = row["probe_detection_time_s"]
        estimate_ratio = row["local_estimate_ratio"]
        if local_time is not None:
            axes[0].scatter(x, float(local_time), color="#D55E00", marker="o")
        if global_time is not None:
            axes[0].scatter(x, float(global_time), color="#0072B2", marker="x")
        if estimate_ratio is not None:
            axes[1].scatter(x, float(estimate_ratio), color="#009E73", marker="o")
    axes[0].scatter([], [], color="#D55E00", marker="o", label="Local force-only")
    axes[0].scatter([], [], color="#0072B2", marker="x", label="Global baseline")
    axes[0].set_ylabel("Detection time (s)")
    axes[0].legend()
    axes[1].axhline(1.0, color="0.35", linestyle="--", label="True friction")
    axes[1].set_ylabel("Local estimate / true friction")
    axes[1].legend()
    labels = [scenario.replace("_friction_probe", "").replace("_", " ") for scenario in scenarios]
    for axis in axes:
        axis.set_xticks(np.arange(len(scenarios)), labels, rotation=25, ha="right")
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def render_study_figures(rows: list[dict[str, object]], study_dir: Path) -> list[Path]:
    """从 study 逐 run 行生成局部起滑验证图。"""
    figure_pdf = plot_summary(rows, study_dir / "local_slip_validation.png")
    return [figure_pdf.with_suffix(".png"), figure_pdf]


def _create_study_directory(config: FrictionEstimationLocalSlipStudyConfig) -> Path:
    """创建一次独占的 study 目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def build_plan(config: FrictionEstimationLocalSlipStudyConfig) -> StudyPlan:
    """从权威 domain config 生成局部起滑验证的唯一有序计划。"""
    scenario_tasks = {
        scenario.task: FrictionEstimationTask.load(scenario.task) for scenario in config.scenarios
    }
    names = [task.name for task in scenario_tasks.values()]
    if len(set(names)) != len(names):
        raise ValueError("scenario task names must be unique for condition identifiers")
    conditions = tuple(
        StudyCondition(
            condition_id=f"{scenario_tasks[scenario.task].name}-seed{seed:03d}",
            parameters={
                "scenario": scenario_tasks[scenario.task].name,
                "task_path": str(scenario.task),
                "expect_local_slip": scenario.expect_local_slip,
                "sensor_noise_seed": seed,
            },
            pair_key=f"{scenario_tasks[scenario.task].name}:seed{seed:03d}",
            baseline_role=None,
        )
        for scenario, seed in config.conditions()
    )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "friction_estimation_local_slip.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": file_sha256(config.profile),
            "scenario_tasks": {
                task.name: file_sha256(path) for path, task in scenario_tasks.items()
            },
        },
        "validation": {
            "control_candidate_ratio_bounds": list(_CONTROL_CANDIDATE_RATIO_BOUNDS),
            "aggregation": "scenario; detection/false-positive rates v1",
        },
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
        study_kind="friction_estimation_local_slip",
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=conditions,
        seeds=tuple(config.seeds.values()),
        preflight={"status": "pending", "checks": ["profile", "scenario_tasks"]},
    )


def run_study(
    config: FrictionEstimationLocalSlipStudyConfig,
    *,
    config_source: Path | None = None,
    study_directory: Path | None = None,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
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
    expected_plan = build_plan(config)
    plan = (
        expected_plan
        if study_plan is None
        else require_matching_study_plan(expected_plan, study_plan)
    )

    def execute(condition: StudyCondition) -> ConditionExecution:
        parameters = condition.parameters
        task_path = Path(str(parameters["task_path"]))
        seed = int(parameters["sensor_noise_seed"])
        task = FrictionEstimationTask.load(task_path)
        run, result = execute_friction_estimation(
            profile=config.profile,
            task_path=task_path,
            estimation_task=task,
            output_root=study_dir / "runs",
            run_prefix=condition.condition_id,
            sensor_noise_seed=seed,
        )
        local_estimates = [
            value
            for value in (
                result.local_left_friction_estimate,
                result.local_right_friction_estimate,
            )
            if value is not None
        ]
        local_detected = result.local_slip_detection_time_s is not None
        local_estimate = min(local_estimates) if local_estimates else None
        local_estimate_ratio = (
            None if local_estimate is None else local_estimate / task.friction_coefficient
        )
        expect_local_slip = bool(parameters["expect_local_slip"])
        event_expectation_passed = local_detected == expect_local_slip
        control_candidate_qualified = bool(
            expect_local_slip
            and local_estimate_ratio is not None
            and _CONTROL_CANDIDATE_RATIO_BOUNDS[0]
            <= local_estimate_ratio
            <= _CONTROL_CANDIDATE_RATIO_BOUNDS[1]
        )
        lead = (
            None
            if result.probe_detection_time_s is None or result.local_slip_detection_time_s is None
            else result.probe_detection_time_s - result.local_slip_detection_time_s
        )
        validation_passed = event_expectation_passed and (
            control_candidate_qualified if expect_local_slip else True
        )
        row = {
            "scenario": task.name,
            "task": str(task_path),
            "sensor_noise_seed": seed,
            "expect_local_slip": expect_local_slip,
            "local_slip_detected": local_detected,
            "event_expectation_passed": event_expectation_passed,
            "control_candidate_qualified": control_candidate_qualified,
            "validation_passed": validation_passed,
            "local_estimate": local_estimate,
            "local_estimate_ratio": local_estimate_ratio,
            "detection_lead_s": lead,
            "run_directory": str(run.path.relative_to(study_dir)),
            **asdict(result),
        }
        return ConditionExecution(
            row=row,
            run_directory=str(row["run_directory"]),
            passed=validation_passed,
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
            "all_event_expectations_passed": all(
                bool(row["event_expectation_passed"]) for row in rows
            ),
            "all_expectations_passed": all(bool(row["validation_passed"]) for row in rows),
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
    )

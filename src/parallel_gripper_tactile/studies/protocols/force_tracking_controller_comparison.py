"""运行可复现的力跟踪控制器对比研究并生成 study 级图表。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
from datetime import UTC, datetime
import json
import math
from pathlib import Path
from statistics import fmean, stdev
from typing import Iterable
from uuid import uuid4

import numpy as np

from parallel_gripper_tactile.experiments.force_tracking import (
    ForceTrackingTask,
    configure_force_controller,
)
from parallel_gripper_tactile.visualization import (
    FULL_WIDTH_FONT_SCALE,
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)
from parallel_gripper_tactile.config.profiles import GripperProfile, load_profile
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
from parallel_gripper_tactile.studies.force_tracking_comparison import (
    ForceTrackingComparisonConfig,
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


METRICS = (
    "contact_time_s",
    "tracking_start_time_s",
    "rmse_n",
    "mae_n",
    "peak_abs_error_n",
    "mean_error_n",
    "final_error_n",
    "torque_saturation_ratio",
    "position_saturation_ratio",
    "mean_estimated_stiffness_n_per_m",
    "rise_time_s",
    "overshoot_ratio",
    "settling_time_s",
)
# 阶跃瞬态指标在非 hold 任务或无法判定时为 None，聚合需按 NaN 感知口径统计。
TRANSIENT_METRICS = ("rise_time_s", "overshoot_ratio", "settling_time_s")
PLOTTED_METRICS = (
    ("rmse_n", "RMSE (N)"),
    ("mae_n", "MAE (N)"),
    ("peak_abs_error_n", "Peak absolute error (N)"),
)
_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


def _json_compatible(value: object) -> object:
    """把非有限浮点数转换为标准 JSON 的 ``null``。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    return value


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按控制器、任务和材料计算有限指标的均值与样本标准差。"""
    return aggregate_records(
        rows,
        keys=("controller_variant", "task_name", "object_material"),
        columns=(
            key("controller_variant"),
            key("task_name"),
            key("object_material"),
            count("runs"),
            bool_sum("passed", "passed_runs"),
            *(
                stat
                for metric in METRICS
                for stat in (
                    (nan_mean(metric), nan_std(metric))
                    if metric in TRANSIENT_METRICS
                    else (finite_mean(metric), finite_std(metric))
                )
            ),
        ),
    )


def _finite_error(value: object) -> float:
    """将缺失或非有限标准差转换为零误差条。"""
    if value is None:
        return 0.0
    number = float(value)
    return number if math.isfinite(number) else 0.0


def plot_metric_summary(
    aggregates: list[dict[str, object]],
    output: Path,
    *,
    controller_order: Iterable[str],
) -> Path:
    """按任务绘制各控制器跟踪误差的均值与样本标准差。"""
    if not aggregates:
        raise ValueError("cannot plot empty aggregates")
    plt = science_pyplot()
    controllers = tuple(controller_order)
    tasks = tuple(dict.fromkeys(str(row["task_name"]) for row in aggregates))
    materials = tuple(dict.fromkeys(str(row["object_material"]) for row in aggregates))
    colors = ("#0072B2", "#E69F00", "#009E73", "#D55E00")
    figure, axes = plt.subplots(
        len(tasks),
        len(PLOTTED_METRICS),
        figsize=paper_figsize(3.1 * len(tasks)),
        squeeze=False,
        layout="constrained",
    )
    lookup = {
        (str(row["controller_variant"]), str(row["task_name"]), str(row["object_material"])): row
        for row in aggregates
    }
    x = np.arange(len(controllers), dtype=np.float64)
    width = 0.8 / max(1, len(materials))
    for task_index, task_name in enumerate(tasks):
        for metric_index, (metric, label) in enumerate(PLOTTED_METRICS):
            axis = axes[task_index][metric_index]
            for material_index, material in enumerate(materials):
                positions = x - 0.4 + width / 2.0 + material_index * width
                values: list[float] = []
                errors: list[float] = []
                for controller in controllers:
                    row = lookup.get((controller, task_name, material))
                    mean = None if row is None else row.get(f"{metric}_mean")
                    values.append(math.nan if mean is None else float(mean))
                    errors.append(0.0 if row is None else _finite_error(row.get(f"{metric}_std")))
                axis.bar(
                    positions,
                    values,
                    width=width,
                    yerr=errors,
                    capsize=2,
                    label=material,
                    color=colors[material_index % len(colors)],
                    hatch=("", "//", "xx", "..")[material_index % 4],
                    edgecolor="black",
                    linewidth=0.4,
                )
            axis.set_xticks(x, controllers, rotation=20, ha="right")
            axis.set_ylabel(label)
            axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
            if metric_index == 0:
                axis.set_title(task_name)
            if task_index == 0 and metric_index == len(PLOTTED_METRICS) - 1:
                figure.legend(
                    *axis.get_legend_handles_labels(),
                    loc="outside upper center",
                    ncol=len(materials),
                    frameon=False,
                    title="Material",
                )
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def plot_saturation_summary(
    aggregates: list[dict[str, object]],
    output: Path,
    *,
    controller_order: Iterable[str],
) -> Path:
    """绘制各控制器的力矩和位置饱和比例。"""
    if not aggregates:
        raise ValueError("cannot plot empty aggregates")
    plt = science_pyplot()
    controllers = tuple(controller_order)
    task_materials = tuple(
        dict.fromkeys((str(row["task_name"]), str(row["object_material"])) for row in aggregates)
    )
    lookup = {
        (str(row["controller_variant"]), str(row["task_name"]), str(row["object_material"])): row
        for row in aggregates
    }
    figure, axes = plt.subplots(1, 2, figsize=paper_figsize(3.7), layout="constrained")
    for axis, metric, title in (
        (axes[0], "torque_saturation_ratio", "Torque saturation"),
        (axes[1], "position_saturation_ratio", "Position saturation"),
    ):
        values = []
        errors = []
        for controller in controllers:
            condition_values = []
            for task_name, material in task_materials:
                row = lookup.get((controller, task_name, material))
                if row is not None and row.get(f"{metric}_mean") is not None:
                    condition_values.append(float(row[f"{metric}_mean"]))
            values.append(fmean(condition_values) if condition_values else math.nan)
            errors.append(stdev(condition_values) if len(condition_values) >= 2 else 0.0)
        axis.bar(controllers, values, yerr=errors, capsize=2, color="#0072B2")
        axis.set_title(title)
        axis.set_ylabel("Ratio")
        axis.tick_params(axis="x", rotation=20)
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def plot_ablation_delta(
    aggregates: list[dict[str, object]],
    output: Path,
    *,
    controller_order: Iterable[str],
) -> Path:
    """绘制各消融变体相对 ``full`` 的 RMSE 变化。"""
    if not aggregates:
        raise ValueError("cannot plot empty aggregates")
    plt = science_pyplot()
    controllers = tuple(controller for controller in controller_order if controller != "full")
    tasks = tuple(dict.fromkeys(str(row["task_name"]) for row in aggregates))
    materials = tuple(dict.fromkeys(str(row["object_material"]) for row in aggregates))
    lookup = {
        (str(row["controller_variant"]), str(row["task_name"]), str(row["object_material"])): row
        for row in aggregates
    }
    figure, axes = plt.subplots(
        1, len(tasks), figsize=paper_figsize(3.8), squeeze=False, layout="constrained"
    )
    x = np.arange(len(controllers), dtype=np.float64)
    width = 0.8 / max(1, len(materials))
    colors = ("#0072B2", "#E69F00", "#009E73")
    for task_index, task_name in enumerate(tasks):
        axis = axes[0][task_index]
        for material_index, material in enumerate(materials):
            baseline = lookup.get(("full", task_name, material))
            baseline_rmse = (
                math.nan
                if baseline is None or baseline.get("rmse_n_mean") is None
                else float(baseline["rmse_n_mean"])
            )
            deltas = []
            for controller in controllers:
                row = lookup.get((controller, task_name, material))
                value = (
                    math.nan
                    if row is None or row.get("rmse_n_mean") is None
                    else float(row["rmse_n_mean"])
                )
                deltas.append(value - baseline_rmse)
            positions = x - 0.4 + width / 2.0 + material_index * width
            axis.bar(
                positions,
                deltas,
                width=width,
                color=colors[material_index % len(colors)],
                hatch=("", "//", "xx", "..")[material_index % 4],
                edgecolor="black",
                linewidth=0.4,
                label=material,
            )
        axis.axhline(0.0, color="black", linewidth=0.8)
        axis.set_xticks(x, controllers, rotation=20, ha="right")
        axis.set_title(task_name)
        axis.set_ylabel("RMSE change relative to full (N)")
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
        if task_index == len(tasks) - 1:
            figure.legend(
                *axis.get_legend_handles_labels(),
                loc="outside upper center",
                ncol=len(materials),
                frameon=False,
                title="Material",
            )
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def _read_tracking_rows(run_directory: Path) -> list[dict[str, object]]:
    """读取单次 run 的跟踪阶段 trace。"""
    rows = [row for row in read_trace_rows(run_directory) if row["phase"] == "track_reference"]
    if not rows:
        raise ValueError(f"trace 没有跟踪阶段行：{run_directory}")
    return rows


def plot_tracking_overlays(
    rows: list[dict[str, object]],
    study_dir: Path,
    figures_dir: Path,
    *,
    controller_order: Iterable[str],
) -> list[Path]:
    """为有效条件绘制最小共同 seed 的控制器轨迹。

    未建立可跟踪接触的运行会保留在 summary 中作为失败条件，但不参与轨迹叠加，
    因为它们没有 ``track_reference`` 样本可供公平比较。
    """
    plt = science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)
    controllers = tuple(controller_order)
    groups: dict[tuple[str, str], list[dict[str, object]]] = {}
    for row in rows:
        key = (str(row["task_name"]), str(row["object_material"]))
        groups.setdefault(key, []).append(row)

    outputs: list[Path] = []
    for (task_name, material), group in groups.items():
        valid_group = [row for row in group if bool(row["passed"])]
        common_seeds = set.intersection(
            *(
                {
                    int(row["sensor_noise_seed"])
                    for row in valid_group
                    if row["controller_variant"] == controller
                }
                for controller in controllers
            )
        )
        if not common_seeds:
            continue
        seed = min(common_seeds)
        figure, axis = plt.subplots(figsize=paper_figsize(3.8), layout="constrained")
        target_drawn = False
        for controller in controllers:
            selected = next(
                row
                for row in valid_group
                if row["controller_variant"] == controller and int(row["sensor_noise_seed"]) == seed
            )
            trace = _read_tracking_rows(study_dir / str(selected["run_directory"]))
            time_s = [float(item["tracking_time_s"]) for item in trace]
            if not target_drawn:
                axis.plot(
                    time_s,
                    [float(item["target_normal_force_n"]) for item in trace],
                    color="black",
                    linewidth=1.2,
                    linestyle="--",
                    label="target",
                )
                target_drawn = True
            axis.plot(
                time_s,
                [float(item["filtered_normal_force_n"]) for item in trace],
                linewidth=1.0,
                label=controller,
            )
        axis.set_xlabel("Tracking time (s)")
        axis.set_ylabel("Mean side normal force (N)")
        axis.set_title(f"{task_name} / {material} / seed {seed}")
        axis.grid(True, linewidth=0.3, alpha=0.5)
        axis.legend(frameon=False, ncol=2)
        safe_task = task_name.replace(" ", "_").replace("/", "_")
        output = figures_dir / f"tracking_{safe_task}_{material}.png"
        pdf_path = save_publication_figure(figure, output)
        plt.close(figure)
        outputs.append(pdf_path)
    return outputs


def render_study_figures(
    rows: list[dict[str, object]],
    aggregates: list[dict[str, object]],
    study_dir: Path,
    *,
    controller_order: Iterable[str],
) -> list[Path]:
    """从已保存 summary 与子 run trace 生成或刷新论文级对比图。"""
    figures_dir = study_dir / "figures"
    figures_dir.mkdir(exist_ok=True)
    metric_plot = figures_dir / "metrics_by_controller.png"
    saturation_plot = figures_dir / "saturation_comparison.png"
    delta_plot = figures_dir / "ablation_delta.png"
    metric_path = plot_metric_summary(aggregates, metric_plot, controller_order=controller_order)
    saturation_path = plot_saturation_summary(
        aggregates, saturation_plot, controller_order=controller_order
    )
    delta_path = plot_ablation_delta(aggregates, delta_plot, controller_order=controller_order)
    trace_plots = plot_tracking_overlays(
        rows,
        study_dir,
        figures_dir,
        controller_order=controller_order,
    )
    return [
        metric_path,
        saturation_path,
        delta_path,
        *trace_plots,
    ]


def _create_study_directory(config: ForceTrackingComparisonConfig) -> Path:
    """为一次 comparison protocol 创建独占目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def validate_inputs(
    config: ForceTrackingComparisonConfig,
    *,
    resolved_profile: GripperProfile | None = None,
) -> dict[Path, ForceTrackingTask]:
    """加载 profile 和所有任务，确保 dry-run 也完成输入校验。"""
    if resolved_profile is None:
        load_profile(config.profile)
    return {task_path: ForceTrackingTask.load(task_path) for task_path in config.tasks}


def describe_conditions(config: ForceTrackingComparisonConfig) -> str:
    """返回稳定、可审阅的条件矩阵文本。"""
    lines = [
        f"Study: {config.name}",
        f"Profile: {config.profile}",
        f"Stiffness estimator: {config.stiffness_estimator_method}",
        f"Conditions: {len(config.conditions())}",
    ]
    for index, (controller, task_path, material, seed) in enumerate(config.conditions(), start=1):
        lines.append(
            f"{index:03d} controller={controller} task={task_path.name} "
            f"material={material} seed={seed}"
        )
    return "\n".join(lines)


def build_plan(
    config: ForceTrackingComparisonConfig,
    *,
    resolved_profile: GripperProfile | None = None,
) -> StudyPlan:
    """从权威 comparison config 生成唯一有序计划。"""
    base_profile = resolved_profile or load_profile(config.profile)
    representative_seed = config.seeds.values()[0]
    controller_profiles = {
        controller: configure_force_controller(
            base_profile,
            variant=controller,
            stiffness_estimator_method=config.stiffness_estimator_method,
            sensor_noise_seed=representative_seed,
        ).model_dump(mode="python")
        for controller in config.controllers
    }
    conditions = tuple(
        StudyCondition(
            condition_id=f"{controller}-{task.stem}-{material}-seed{seed:03d}",
            parameters={
                "controller_variant": controller,
                "stiffness_estimator_method": config.stiffness_estimator_method,
                "task_path": str(task),
                "object_material": material,
                "sensor_noise_seed": seed,
            },
            pair_key=f"{task.stem}:{material}:seed{seed:03d}",
            baseline_role="full" if controller == "full" else None,
        )
        for controller, task, material, seed in config.conditions()
    )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "force_tracking_controller_comparison.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": model_configuration_sha256(
                base_profile, repository_root=_REPOSITORY_ROOT
            ),
            "task_sha256": {str(task): file_sha256(task) for task in config.tasks},
        },
        "resolved_controller_profiles": controller_profiles,
        "scientific_runtime": {
            "object_contact_model": "explicit",
            "multiccd_enabled": True,
            "force_semantics": "average_side",
            "trace_sample_period_s": {
                "default": 0.01,
                "adrc-torque": 0.004,
            },
            "trace_event_window_s": 0.2,
        },
        "aggregation": "controller_variant,task_name,object_material; finite/nan-aware v1",
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
        study_kind="force_tracking_controller_comparison",
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=conditions,
        seeds=tuple(config.seeds.values()),
        preflight={"status": "pending", "checks": ["profile", "tasks", "scenes"]},
    )


def run_study(
    config: ForceTrackingComparisonConfig,
    *,
    resolved_profile: GripperProfile | None = None,
    config_source: Path | None = None,
    study_directory: Path | None = None,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
) -> Path:
    """通过公共生命周期执行 comparison protocol 并返回 study 目录。"""
    tasks = validate_inputs(config, resolved_profile=resolved_profile)
    study_dir = (
        _create_study_directory(config) if study_directory is None else study_directory.resolve()
    )
    study_dir.mkdir(parents=True, exist_ok=True)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        raise ValueError("config_source is required for a reproducible comparison study")
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)
    expected_plan = build_plan(config, resolved_profile=resolved_profile)
    plan = (
        expected_plan
        if study_plan is None
        else require_matching_study_plan(expected_plan, study_plan)
    )

    def execute(condition: StudyCondition) -> ConditionExecution:
        parameters = condition.parameters
        task_path = Path(str(parameters["task_path"]))
        controller = str(parameters["controller_variant"])
        material = str(parameters["object_material"])
        seed = int(parameters["sensor_noise_seed"])
        task = tasks[task_path]
        run, result = execute_force_tracking(
            profile=config.profile,
            resolved_profile=resolved_profile,
            task_path=task_path,
            tracking_task=task,
            output_root=study_dir / "runs",
            run_prefix=condition.condition_id,
            object_material=material,
            controller_variant=controller,
            stiffness_estimator_method=config.stiffness_estimator_method,
            sensor_noise_seed=seed,
        )
        row = {
            "controller_variant": controller,
            "stiffness_estimator_method": config.stiffness_estimator_method,
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
                _json_compatible(
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
        return StudyPostprocessResult(tuple(artifacts), {}, aggregates)

    def render(rows: list[dict[str, object]], payload: object, directory: Path) -> tuple[Path, ...]:
        if not rows:
            return ()
        aggregates = list(payload) if isinstance(payload, list) else []
        return tuple(
            render_study_figures(
                rows,
                aggregates,
                directory,
                controller_order=config.controllers,
            )
        )

    manifest_fields = {
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


def _write_failure_records(path: Path, rows: list[dict[str, object]]) -> Path:
    """保存无法产生正常 run 结果的条件及异常信息。"""
    path.write_text(json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def render_existing_study(study_dir: Path) -> list[Path]:
    """从已完成 study 的 summary 重建论文级图表并更新 manifest。"""
    summary_path = study_dir / "summary.json"
    manifest_path = study_dir / "study_manifest.json"
    try:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"cannot read study summary: {summary_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid study summary JSON: {summary_path}") from error
    rows = summary.get("runs") if isinstance(summary, dict) else None
    aggregates = summary.get("aggregates") if isinstance(summary, dict) else None
    if not isinstance(rows, list) or not isinstance(aggregates, list):
        raise ValueError(f"study summary lacks runs or aggregates: {summary_path}")
    if not all(isinstance(row, dict) for row in rows + aggregates):
        raise ValueError(f"study summary rows must be objects: {summary_path}")
    typed_rows = [dict(row) for row in rows]
    typed_aggregates = [dict(row) for row in aggregates]
    controller_order = tuple(dict.fromkeys(str(row["controller_variant"]) for row in typed_rows))
    figure_artifacts = render_study_figures(
        typed_rows,
        typed_aggregates,
        study_dir,
        controller_order=controller_order,
    )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"cannot read study manifest: {manifest_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid study manifest JSON: {manifest_path}") from error
    if not isinstance(manifest, dict):
        raise ValueError(f"study manifest must be an object: {manifest_path}")
    existing = manifest.get("artifacts", [])
    if not isinstance(existing, list) or not all(isinstance(item, str) for item in existing):
        raise ValueError(f"study manifest has invalid artifacts: {manifest_path}")
    manifest["artifacts"] = [
        *[item for item in existing if not item.startswith("figures/")],
        *(str(path.relative_to(study_dir)) for path in figure_artifacts),
    ]
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return figure_artifacts

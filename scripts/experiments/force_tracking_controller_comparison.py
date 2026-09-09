"""运行可复现的力跟踪控制器对比研究并生成 study 级图表。"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import UTC, datetime
import json
import math
from pathlib import Path
from statistics import fmean, stdev
from typing import Iterable
from uuid import uuid4
import warnings

import numpy as np

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
from parallel_gripper_tactile.visualization import (
    FULL_WIDTH_FONT_SCALE,
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)
from parallel_gripper_tactile.profiles import load_profile
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.force_tracking_comparison import (
    ForceTrackingComparisonConfig,
    load_comparison_config,
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


def _json_compatible(value: object) -> object:
    """把非有限浮点数转换为标准 JSON 的 ``null``。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    return value


def _transient_stats(group: list[dict[str, object]], metric: str) -> tuple[float, float]:
    """对可能缺失的阶跃瞬态指标做 NaN 感知的均值与样本标准差。"""
    values = np.asarray(
        [math.nan if row.get(metric) is None else float(row[metric]) for row in group],
        dtype=np.float64,
    )
    with warnings.catch_warnings():
        # 全 NaN 切片或单样本 ddof=1 时 numpy 会发 RuntimeWarning，结果按 NaN 输出即可。
        warnings.simplefilter("ignore", RuntimeWarning)
        mean = float(np.nanmean(values))
        std = float(np.nanstd(values, ddof=1))
    return mean, std


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按控制器、任务和材料计算有限指标的均值与样本标准差。"""
    groups: dict[tuple[str, str, str], list[dict[str, object]]] = {}
    for row in rows:
        key = (
            str(row["controller_variant"]),
            str(row["task_name"]),
            str(row["object_material"]),
        )
        groups.setdefault(key, []).append(row)

    aggregates: list[dict[str, object]] = []
    for (controller, task_name, material), group in groups.items():
        aggregate: dict[str, object] = {
            "controller_variant": controller,
            "task_name": task_name,
            "object_material": material,
            "runs": len(group),
            "passed_runs": sum(bool(row["passed"]) for row in group),
        }
        for metric in METRICS:
            if metric in TRANSIENT_METRICS:
                mean, std = _transient_stats(group, metric)
                aggregate[f"{metric}_mean"] = mean
                aggregate[f"{metric}_std"] = std
                continue
            finite = [float(row[metric]) for row in group if math.isfinite(float(row[metric]))]
            aggregate[f"{metric}_mean"] = fmean(finite) if finite else None
            aggregate[f"{metric}_std"] = stdev(finite) if len(finite) >= 2 else None
        aggregates.append(aggregate)
    return aggregates


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
        outputs.extend((output, pdf_path))
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
    metric_pdf = plot_metric_summary(aggregates, metric_plot, controller_order=controller_order)
    saturation_pdf = plot_saturation_summary(
        aggregates, saturation_plot, controller_order=controller_order
    )
    delta_pdf = plot_ablation_delta(aggregates, delta_plot, controller_order=controller_order)
    trace_plots = plot_tracking_overlays(
        rows,
        study_dir,
        figures_dir,
        controller_order=controller_order,
    )
    return [
        metric_plot,
        metric_pdf,
        saturation_plot,
        saturation_pdf,
        delta_plot,
        delta_pdf,
        *trace_plots,
    ]


def _create_study_directory(config: ForceTrackingComparisonConfig) -> Path:
    """为一次 comparison protocol 创建独占目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def validate_inputs(config: ForceTrackingComparisonConfig) -> dict[Path, ForceTrackingTask]:
    """加载 profile 和所有任务，确保 dry-run 也完成输入校验。"""
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


def run_study(
    config: ForceTrackingComparisonConfig,
    *,
    config_source: Path | None = None,
) -> Path:
    """执行完整 comparison protocol 并返回 study 目录。"""
    tasks = validate_inputs(config)
    study_dir = _create_study_directory(config)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        raise ValueError("config_source is required for a reproducible comparison study")
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)

    rows: list[dict[str, object]] = []
    for controller, task_path, material, seed in config.conditions():
        task = tasks[task_path]
        condition = f"{controller}-{task_path.stem}-{material}-seed{seed:03d}"
        run, result = execute_force_tracking(
            profile=config.profile,
            task_path=task_path,
            tracking_task=task,
            output_root=study_dir / "runs",
            run_prefix=condition,
            object_material=material,
            controller_variant=controller,
            stiffness_estimator_method=config.stiffness_estimator_method,
            sensor_noise_seed=seed,
        )
        rows.append(
            {
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
        )

    aggregates = aggregate_rows(rows)
    summary_csv = study_dir / "summary.csv"
    aggregate_csv = study_dir / "aggregate.csv"
    summary_json = study_dir / "summary.json"
    _, summary_parquet = write_rows_csv_and_parquet(summary_csv, rows)
    _, aggregate_parquet = write_rows_csv_and_parquet(aggregate_csv, aggregates)
    summary_json.write_text(
        json.dumps(
            _json_compatible({"runs": rows, "aggregates": aggregates}),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    figure_artifacts = render_study_figures(
        rows, aggregates, study_dir, controller_order=config.controllers
    )
    artifacts = [
        summary_csv,
        summary_parquet,
        aggregate_csv,
        aggregate_parquet,
        summary_json,
        resolved_config,
        *figure_artifacts,
    ]
    (study_dir / "study_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": config.name,
                "config": "study.yaml",
                "resolved_config": str(resolved_config.relative_to(study_dir)),
                "runs": [row["run_directory"] for row in rows],
                "failed_runs": [row["run_directory"] for row in rows if not bool(row["passed"])],
                "artifacts": [str(path.relative_to(study_dir)) for path in artifacts],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return study_dir


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


def main() -> None:
    """解析 study 配置，打印矩阵或执行完整研究。"""
    parser = argparse.ArgumentParser(description=__doc__)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--config", type=Path, help="Study YAML path")
    input_group.add_argument(
        "--render-study-dir",
        type=Path,
        help="Regenerate publication figures from an existing study directory",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate inputs and print the condition matrix without running MuJoCo",
    )
    arguments = parser.parse_args()
    if arguments.render_study_dir is not None:
        if arguments.dry_run:
            parser.error("--dry-run requires --config")
        artifacts = render_existing_study(arguments.render_study_dir.resolve())
        print(f"Publication figures: {len(artifacts)} files")
        return
    assert arguments.config is not None
    config_path = arguments.config.resolve()
    config = load_comparison_config(config_path)
    validate_inputs(config)
    if arguments.dry_run:
        print(describe_conditions(config))
        return
    result = run_study(config, config_source=config_path)
    print(f"Study: {result}")


if __name__ == "__main__":
    main()

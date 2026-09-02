"""运行可复现的刚度估计器对比研究并生成 study 级图表。"""

from __future__ import annotations

import argparse
import csv
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
from parallel_gripper_tactile.profiles import load_profile
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.force_tracking_stiffness_estimator_comparison import (
    ForceTrackingStiffnessEstimatorComparisonConfig,
    load_stiffness_estimator_comparison_config,
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
    ("final_error_n", "Final error (N)"),
)
PUBLICATION_DPI = 600
BASELINE_ESTIMATOR = "secant_ewma"


def _json_compatible(value: object) -> object:
    """把非有限浮点数转换为标准 JSON 的 ``null``。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    return value


def _write_rows_csv(path: Path, rows: list[dict[str, object]]) -> None:
    """把同构结果行写入 CSV。"""
    if not rows:
        raise ValueError("cannot write an empty study summary")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


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
    """按估计器、任务和材料计算有限指标的均值与样本标准差。"""
    groups: dict[tuple[str, str, str], list[dict[str, object]]] = {}
    for row in rows:
        key = (
            str(row["stiffness_estimator_method"]),
            str(row["task_name"]),
            str(row["object_material"]),
        )
        groups.setdefault(key, []).append(row)

    aggregates: list[dict[str, object]] = []
    for (estimator, task_name, material), group in groups.items():
        aggregate: dict[str, object] = {
            "stiffness_estimator_method": estimator,
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


def _science_pyplot():
    """加载项目统一的论文级无 LaTeX SciencePlots 样式。"""
    try:
        import matplotlib.pyplot as plt
        import scienceplots  # noqa: F401 -- 导入后注册样式。
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error
    plt.style.use(["science", "ieee", "no-latex"])
    plt.rcParams.update({"pdf.fonttype": 42, "ps.fonttype": 42, "savefig.dpi": PUBLICATION_DPI})
    return plt


def _save_publication_figure(figure, png_path: Path, **savefig_kwargs: object) -> Path:
    """同时保存 600 DPI PNG 与嵌入 TrueType 字体的矢量 PDF。"""
    pdf_path = png_path.with_suffix(".pdf")
    figure.savefig(png_path, dpi=PUBLICATION_DPI, **savefig_kwargs)
    figure.savefig(pdf_path, **savefig_kwargs)
    return pdf_path


def _finite_error(value: object) -> float:
    """将缺失或非有限标准差转换为零误差条。"""
    if value is None:
        return 0.0
    number = float(value)
    return number if math.isfinite(number) else 0.0


def plot_metric_summary(
    aggregates: list[dict[str, object]], output: Path, *, estimator_order: Iterable[str]
) -> Path:
    """按任务绘制各刚度估计器的跟踪指标均值与样本标准差。"""
    if not aggregates:
        raise ValueError("cannot plot empty aggregates")
    plt = _science_pyplot()
    estimators = tuple(estimator_order)
    tasks = tuple(dict.fromkeys(str(row["task_name"]) for row in aggregates))
    materials = tuple(dict.fromkeys(str(row["object_material"]) for row in aggregates))
    colors = ("#0072B2", "#E69F00", "#009E73")
    figure, axes = plt.subplots(
        len(tasks), len(PLOTTED_METRICS), figsize=(10.5, 3.1 * len(tasks)), squeeze=False
    )
    lookup = {
        (
            str(row["stiffness_estimator_method"]),
            str(row["task_name"]),
            str(row["object_material"]),
        ): row
        for row in aggregates
    }
    x = np.arange(len(estimators), dtype=np.float64)
    width = 0.8 / max(1, len(materials))
    for task_index, task_name in enumerate(tasks):
        for metric_index, (metric, label) in enumerate(PLOTTED_METRICS):
            axis = axes[task_index][metric_index]
            for material_index, material in enumerate(materials):
                positions = x - 0.4 + width / 2.0 + material_index * width
                values: list[float] = []
                errors: list[float] = []
                for estimator in estimators:
                    row = lookup.get((estimator, task_name, material))
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
                )
            axis.set_xticks(x, estimators, rotation=20, ha="right")
            axis.set_ylabel(label)
            axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
            if metric_index == 0:
                axis.set_title(task_name)
            if task_index == 0 and metric_index == len(PLOTTED_METRICS) - 1:
                axis.legend(frameon=False, title="Material")
    pdf_path = _save_publication_figure(figure, output, bbox_inches="tight")
    plt.close(figure)
    return pdf_path


def plot_delta_vs_secant(
    aggregates: list[dict[str, object]], output: Path, *, estimator_order: Iterable[str]
) -> Path:
    """绘制窗口方法相对 ``secant_ewma`` 的 RMSE 变化。"""
    if not aggregates:
        raise ValueError("cannot plot empty aggregates")
    plt = _science_pyplot()
    estimators = tuple(
        estimator for estimator in estimator_order if estimator != BASELINE_ESTIMATOR
    )
    tasks = tuple(dict.fromkeys(str(row["task_name"]) for row in aggregates))
    materials = tuple(dict.fromkeys(str(row["object_material"]) for row in aggregates))
    lookup = {
        (
            str(row["stiffness_estimator_method"]),
            str(row["task_name"]),
            str(row["object_material"]),
        ): row
        for row in aggregates
    }
    figure, axes = plt.subplots(1, len(tasks), figsize=(4.2 * len(tasks), 3.8), squeeze=False)
    x = np.arange(len(estimators), dtype=np.float64)
    width = 0.8 / max(1, len(materials))
    colors = ("#0072B2", "#E69F00", "#009E73")
    for task_index, task_name in enumerate(tasks):
        axis = axes[0][task_index]
        for material_index, material in enumerate(materials):
            baseline = lookup.get((BASELINE_ESTIMATOR, task_name, material))
            baseline_rmse = (
                math.nan
                if baseline is None or baseline.get("rmse_n_mean") is None
                else float(baseline["rmse_n_mean"])
            )
            deltas = []
            for estimator in estimators:
                row = lookup.get((estimator, task_name, material))
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
                label=material,
            )
        axis.axhline(0.0, color="black", linewidth=0.8)
        axis.set_xticks(x, estimators, rotation=20, ha="right")
        axis.set_title(task_name)
        axis.set_ylabel("RMSE change relative to secant (N)")
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
        if task_index == len(tasks) - 1:
            axis.legend(frameon=False, title="Material")
    pdf_path = _save_publication_figure(figure, output, bbox_inches="tight")
    plt.close(figure)
    return pdf_path


def _read_tracking_rows(path: Path) -> list[dict[str, str]]:
    """读取单次 run 的跟踪阶段 trace。"""
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["phase"] == "track_reference"]
    if not rows:
        raise ValueError(f"trace has no tracking rows: {path}")
    return rows


def plot_tracking_and_stiffness_overlays(
    rows: list[dict[str, object]],
    study_dir: Path,
    figures_dir: Path,
    *,
    estimator_order: Iterable[str],
) -> list[Path]:
    """为共同有效 seed 叠加目标力、实际力和刚度估计轨迹。"""
    plt = _science_pyplot()
    estimators = tuple(estimator_order)
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
                    if row["stiffness_estimator_method"] == estimator
                }
                for estimator in estimators
            )
        )
        if not common_seeds:
            continue
        seed = min(common_seeds)
        figure, axes = plt.subplots(2, 1, figsize=(7.16, 5.6), sharex=True, layout="constrained")
        target_drawn = False
        for estimator in estimators:
            selected = next(
                row
                for row in valid_group
                if row["stiffness_estimator_method"] == estimator
                and int(row["sensor_noise_seed"]) == seed
            )
            trace = _read_tracking_rows(study_dir / str(selected["run_directory"]) / "trace.csv")
            time_s = [float(item["tracking_time_s"]) for item in trace]
            if not target_drawn:
                axes[0].plot(
                    time_s,
                    [float(item["target_normal_force_n"]) for item in trace],
                    color="black",
                    linewidth=1.2,
                    linestyle="--",
                    label="target",
                )
                target_drawn = True
            axes[0].plot(
                time_s,
                [float(item["filtered_normal_force_n"]) for item in trace],
                linewidth=1.0,
                label=estimator,
            )
            axes[1].plot(
                time_s,
                [float(item["estimated_contact_stiffness_n_per_m"]) for item in trace],
                linewidth=1.0,
                label=estimator,
            )
        axes[0].set_ylabel("Mean side normal force (N)")
        axes[0].set_title(f"{task_name} / {material} / seed {seed}")
        axes[1].set_xlabel("Tracking time (s)")
        axes[1].set_ylabel("K estimate (N/m)")
        for axis in axes:
            axis.grid(True, linewidth=0.3, alpha=0.5)
        axes[0].legend(frameon=False, ncol=2)
        axes[1].legend(frameon=False, ncol=2)
        safe_task = task_name.replace(" ", "_").replace("/", "_")
        output = figures_dir / f"tracking_and_stiffness_{safe_task}_{material}.png"
        pdf_path = _save_publication_figure(figure, output)
        plt.close(figure)
        outputs.extend((output, pdf_path))
    return outputs


def render_study_figures(
    rows: list[dict[str, object]],
    aggregates: list[dict[str, object]],
    study_dir: Path,
    *,
    estimator_order: Iterable[str],
) -> list[Path]:
    """从已保存 summary 与子 run trace 生成或刷新论文级对比图。"""
    figures_dir = study_dir / "figures"
    figures_dir.mkdir(exist_ok=True)
    metric_plot = figures_dir / "metrics_by_estimator.png"
    delta_plot = figures_dir / "delta_vs_secant.png"
    metric_pdf = plot_metric_summary(aggregates, metric_plot, estimator_order=estimator_order)
    delta_pdf = plot_delta_vs_secant(aggregates, delta_plot, estimator_order=estimator_order)
    trace_plots = plot_tracking_and_stiffness_overlays(
        rows, study_dir, figures_dir, estimator_order=estimator_order
    )
    return [metric_plot, metric_pdf, delta_plot, delta_pdf, *trace_plots]


def _create_study_directory(config: ForceTrackingStiffnessEstimatorComparisonConfig) -> Path:
    """为一次估计器 comparison protocol 创建独占目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def validate_inputs(
    config: ForceTrackingStiffnessEstimatorComparisonConfig,
) -> dict[Path, ForceTrackingTask]:
    """加载 profile 和所有任务，确保 dry-run 也完成输入校验。"""
    load_profile(config.profile)
    return {task_path: ForceTrackingTask.load(task_path) for task_path in config.tasks}


def describe_conditions(config: ForceTrackingStiffnessEstimatorComparisonConfig) -> str:
    """返回稳定、可审阅的条件矩阵文本。"""
    lines = [
        f"Study: {config.name}",
        f"Profile: {config.profile}",
        "Controller: pid-stiffness-ff",
        f"Conditions: {len(config.conditions())}",
    ]
    for index, (estimator, task_path, material, seed) in enumerate(config.conditions(), start=1):
        lines.append(
            f"{index:03d} estimator={estimator} task={task_path.name} "
            f"material={material} seed={seed}"
        )
    return "\n".join(lines)


def run_study(
    config: ForceTrackingStiffnessEstimatorComparisonConfig,
    *,
    config_source: Path | None = None,
) -> Path:
    """执行完整估计器 comparison protocol 并返回 study 目录。"""
    tasks = validate_inputs(config)
    study_dir = _create_study_directory(config)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        raise ValueError("config_source is required for a reproducible stiffness estimator study")

    rows: list[dict[str, object]] = []
    for estimator, task_path, material, seed in config.conditions():
        task = tasks[task_path]
        condition = f"pid-stiffness-ff-{estimator}-{task_path.stem}-{material}-seed{seed:03d}"
        run, result = execute_force_tracking(
            profile=config.profile,
            task_path=task_path,
            tracking_task=task,
            output_root=study_dir / "runs",
            run_prefix=condition,
            object_material=material,
            controller_variant="pid-stiffness-ff",
            stiffness_estimator_method=estimator,
            sensor_noise_seed=seed,
        )
        rows.append(
            {
                "controller_variant": "pid-stiffness-ff",
                "stiffness_estimator_method": estimator,
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
    _write_rows_csv(summary_csv, rows)
    _write_rows_csv(aggregate_csv, aggregates)
    summary_json.write_text(
        json.dumps(
            _json_compatible({"runs": rows, "aggregates": aggregates}), indent=2, sort_keys=True
        )
        + "\n",
        encoding="utf-8",
    )
    figure_artifacts = render_study_figures(
        rows, aggregates, study_dir, estimator_order=config.estimators
    )
    artifacts = [summary_csv, aggregate_csv, summary_json, *figure_artifacts]
    (study_dir / "study_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": config.name,
                "config": "study.yaml",
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
    estimator_order = tuple(
        dict.fromkeys(str(row["stiffness_estimator_method"]) for row in typed_rows)
    )
    figure_artifacts = render_study_figures(
        typed_rows, typed_aggregates, study_dir, estimator_order=estimator_order
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
    config = load_stiffness_estimator_comparison_config(config_path)
    validate_inputs(config)
    if arguments.dry_run:
        print(describe_conditions(config))
        return
    result = run_study(config, config_source=config_path)
    print(f"Study: {result}")


if __name__ == "__main__":
    main()

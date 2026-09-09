"""运行可复现的力跟踪 controller × material × seed 消融研究。."""

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
import yaml

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
from parallel_gripper_tactile.visualization import (
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.force_tracking_ablation import (
    ForceTrackingAblationConfig,
    load_study_config,
)
from parallel_gripper_tactile.studies.tabular import (
    write_resolved_config,
    write_rows_csv_and_parquet,
)


_METRICS = (
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
_TRANSIENT_METRICS = ("rise_time_s", "overshoot_ratio", "settling_time_s")
_PLOTTED_METRICS = (
    ("rmse_n", "RMSE (N)"),
    ("mae_n", "MAE (N)"),
    ("torque_saturation_ratio", "Torque saturation ratio"),
)


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
    """按控制器和材料计算有限指标的均值与样本标准差。."""
    groups: dict[tuple[str, str], list[dict[str, object]]] = {}
    for row in rows:
        key = (str(row["controller_variant"]), str(row["object_material"]))
        groups.setdefault(key, []).append(row)
    aggregates: list[dict[str, object]] = []
    for (controller, material), group in groups.items():
        aggregate: dict[str, object] = {
            "controller_variant": controller,
            "object_material": material,
            "runs": len(group),
            "passed_runs": sum(bool(row["passed"]) for row in group),
        }
        for metric in _METRICS:
            if metric in _TRANSIENT_METRICS:
                mean, std = _transient_stats(group, metric)
                aggregate[f"{metric}_mean"] = mean
                aggregate[f"{metric}_std"] = std
                continue
            finite = [float(row[metric]) for row in group if math.isfinite(float(row[metric]))]
            aggregate[f"{metric}_mean"] = fmean(finite) if finite else None
            aggregate[f"{metric}_std"] = stdev(finite) if len(finite) >= 2 else None
        aggregates.append(aggregate)
    return aggregates


def json_compatible(value: object) -> object:
    """将 NaN 和无穷数转换为标准 JSON 的 ``null``。."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_compatible(item) for item in value]
    return value


def _finite_error(value: object) -> float:
    """将缺失或非有限标准差转换为零误差条。"""
    if value is None:
        return 0.0
    number = float(value)
    return number if math.isfinite(number) else 0.0


def plot_material_summary(
    aggregates: list[dict[str, object]], output: Path, *, controller_order: Iterable[str]
) -> Path:
    """按材料展示 RMSE、MAE 与力矩饱和比例的总体对比。"""
    if not aggregates:
        raise ValueError("cannot plot empty aggregates")
    plt = science_pyplot()
    controllers = tuple(controller_order)
    materials = tuple(dict.fromkeys(str(row["object_material"]) for row in aggregates))
    lookup = {
        (str(row["controller_variant"]), str(row["object_material"])): row for row in aggregates
    }
    figure, axes = plt.subplots(
        len(materials),
        len(_PLOTTED_METRICS),
        figsize=paper_figsize(max(3.0, 2.7 * len(materials))),
        squeeze=False,
        layout="constrained",
    )
    x = np.arange(len(controllers), dtype=np.float64)
    for material_index, material in enumerate(materials):
        for metric_index, (metric, label) in enumerate(_PLOTTED_METRICS):
            axis = axes[material_index][metric_index]
            values: list[float] = []
            errors: list[float] = []
            for controller in controllers:
                row = lookup.get((controller, material))
                mean = None if row is None else row.get(f"{metric}_mean")
                values.append(math.nan if mean is None else float(mean))
                errors.append(0.0 if row is None else _finite_error(row.get(f"{metric}_std")))
            axis.bar(x, values, yerr=errors, capsize=2, color="#0072B2")
            axis.set_xticks(x, controllers, rotation=20, ha="right")
            axis.set_ylabel(label)
            axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
            if material_index == 0:
                axis.set_title(label)
            if metric_index == 0:
                axis.text(
                    0.01,
                    0.94,
                    material,
                    transform=axis.transAxes,
                    va="top",
                    fontweight="bold",
                )
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def paired_pid_factorial_effects(rows: list[dict[str, object]]) -> dict[str, list[float]]:
    """从相同材料和 seed 的 PID 2×2 数据提取 RMSE 主效应与交互数据。"""
    variants = {
        (False, False): "pid-only",
        (True, False): "pid-torque-ff",
        (False, True): "pid-stiffness-ff",
        (True, True): "full",
    }
    by_condition = {
        (
            str(row["object_material"]),
            int(row["sensor_noise_seed"]),
            str(row["controller_variant"]),
        ): row
        for row in rows
        if bool(row.get("passed"))
        and row.get("rmse_n") is not None
        and math.isfinite(float(row["rmse_n"]))
    }
    cells: dict[tuple[bool, bool], list[float]] = {key: [] for key in variants}
    torque_effects: list[float] = []
    stiffness_effects: list[float] = []
    conditions = {(material, seed) for material, seed, _ in by_condition}
    for material, seed in conditions:
        values = {
            factors: float(by_condition[(material, seed, variant)]["rmse_n"])
            for factors, variant in variants.items()
            if (material, seed, variant) in by_condition
        }
        if len(values) != len(variants):
            continue
        for factors, value in values.items():
            cells[factors].append(value)
        for stiffness in (False, True):
            off = values.get((False, stiffness))
            on = values.get((True, stiffness))
            if off is not None and on is not None:
                torque_effects.append(on - off)
        for torque in (False, True):
            off = values.get((torque, False))
            on = values.get((torque, True))
            if off is not None and on is not None:
                stiffness_effects.append(on - off)
    return {
        "none": cells[(False, False)],
        "torque": cells[(True, False)],
        "stiffness": cells[(False, True)],
        "both": cells[(True, True)],
        "torque_effect": torque_effects,
        "stiffness_effect": stiffness_effects,
    }


def _mean_and_standard_error(values: list[float]) -> tuple[float, float]:
    """返回有限样本的均值与标准误；没有样本时返回 NaN。"""
    if not values:
        return math.nan, 0.0
    return fmean(values), stdev(values) / math.sqrt(len(values)) if len(values) > 1 else 0.0


def plot_pid_factorial_effects(rows: list[dict[str, object]], output: Path) -> Path:
    """绘制 torque FF 与 stiffness FF 的配对主效应和 2×2 交互作用。"""
    effects = paired_pid_factorial_effects(rows)
    plt = science_pyplot()
    figure, axes = plt.subplots(1, 2, figsize=paper_figsize(3.6), layout="constrained")
    interaction = axes[0]
    x = np.asarray((0.0, 1.0))
    for label, off_key, on_key, color in (
        ("stiffness FF off", "none", "torque", "#0072B2"),
        ("stiffness FF on", "stiffness", "both", "#D55E00"),
    ):
        off_mean, off_error = _mean_and_standard_error(effects[off_key])
        on_mean, on_error = _mean_and_standard_error(effects[on_key])
        interaction.errorbar(
            x,
            (off_mean, on_mean),
            yerr=(off_error, on_error),
            marker="o",
            capsize=2,
            label=label,
            color=color,
        )
    interaction.set_xticks(x, ("torque FF off", "torque FF on"))
    interaction.set_ylabel("Paired RMSE (N)")
    interaction.set_title("PID 2×2 interaction")
    interaction.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    interaction.legend(frameon=False)

    main_effect = axes[1]
    labels = ("torque FF", "stiffness FF")
    means, errors = zip(
        *(_mean_and_standard_error(effects[key]) for key in ("torque_effect", "stiffness_effect")),
        strict=True,
    )
    main_effect.bar(
        labels,
        means,
        yerr=errors,
        capsize=2,
        color=("#009E73", "#CC79A7"),
        hatch=("//", "xx"),
        edgecolor="black",
        linewidth=0.4,
    )
    main_effect.axhline(0.0, color="black", linewidth=0.8)
    main_effect.set_ylabel("Paired ΔRMSE, FF on − off (N)")
    main_effect.set_title("Main effects")
    main_effect.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def render_study_figures(
    rows: list[dict[str, object]],
    aggregates: list[dict[str, object]],
    study_dir: Path,
    *,
    controller_order: Iterable[str],
) -> list[Path]:
    """从 study 聚合结果生成服务消融目的的论文级图表。"""
    figures_dir = study_dir / "figures"
    figures_dir.mkdir(exist_ok=True)
    summary_plot = figures_dir / "metrics_by_material.png"
    factorial_plot = figures_dir / "pid_factorial_effects.png"
    summary_pdf = plot_material_summary(aggregates, summary_plot, controller_order=controller_order)
    factorial_pdf = plot_pid_factorial_effects(rows, factorial_plot)
    return [summary_plot, summary_pdf, factorial_plot, factorial_pdf]


def _create_study_directory(config: ForceTrackingAblationConfig) -> Path:
    """为一次 protocol 调用创建独占的父目录。."""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def run_study(config: ForceTrackingAblationConfig, *, config_source: Path | None = None) -> Path:
    """直接调用 runner 执行整个 protocol，并返回 study 父目录。."""
    study_dir = _create_study_directory(config)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        (study_dir / "study.yaml").write_text(
            yaml.safe_dump(config.model_dump(mode="json"), sort_keys=False), encoding="utf-8"
        )
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)
    task = ForceTrackingTask.load(config.task)
    rows: list[dict[str, object]] = []
    for controller, material, seed in config.conditions():
        condition = f"{controller}-{material}-seed{seed:03d}"
        run, result = execute_force_tracking(
            profile=config.profile,
            task_path=config.task,
            tracking_task=task,
            output_root=study_dir / "runs",
            run_prefix=condition,
            object_material=material,
            controller_variant=controller,
            sensor_noise_seed=seed,
        )
        rows.append(
            {
                "controller_variant": controller,
                "object_material": material,
                "sensor_noise_seed": seed,
                "passed": result.passed,
                "run_directory": str(run.path.relative_to(study_dir)),
                **asdict(result),
            }
        )
    aggregates = aggregate_rows(rows)
    summary_csv, summary_parquet = write_rows_csv_and_parquet(study_dir / "summary.csv", rows)
    aggregate_csv, aggregate_parquet = write_rows_csv_and_parquet(
        study_dir / "aggregate.csv", aggregates
    )
    summary_json = study_dir / "summary.json"
    summary_json.write_text(
        json.dumps(
            json_compatible({"runs": rows, "aggregates": aggregates}), indent=2, sort_keys=True
        )
        + "\n",
        encoding="utf-8",
    )
    figure_artifacts = render_study_figures(
        rows,
        aggregates,
        study_dir,
        controller_order=config.controllers,
    )
    (study_dir / "study_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": config.name,
                "config": "study.yaml",
                "resolved_config": str(resolved_config.relative_to(study_dir)),
                "runs": [row["run_directory"] for row in rows],
                "artifacts": [
                    str(path.relative_to(study_dir))
                    for path in (
                        summary_csv,
                        summary_parquet,
                        aggregate_csv,
                        aggregate_parquet,
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
    """解析配置文件并运行该科研 protocol。."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="Study YAML path")
    arguments = parser.parse_args()
    config_path = arguments.config.resolve()
    result = run_study(load_study_config(config_path), config_source=config_path)
    print(f"Study: {result}")


if __name__ == "__main__":
    main()

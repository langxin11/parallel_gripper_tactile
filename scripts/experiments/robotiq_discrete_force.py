"""运行 Robotiq 单 tick 力增量离散控制的完整消融矩阵。"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from datetime import UTC, datetime
import json
import os
from pathlib import Path
from statistics import fmean
from uuid import uuid4

import numpy as np
import yaml

from parallel_gripper_tactile.experiments.robotiq_discrete_force import (
    RobotiqDiscreteForceTask,
)
from parallel_gripper_tactile.visualization import (
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)
from parallel_gripper_tactile.runners import execute_robotiq_discrete_force
from parallel_gripper_tactile.studies.robotiq_discrete_force import (
    RobotiqDiscreteForceStudyConfig,
    load_robotiq_discrete_force_study_config,
)
from parallel_gripper_tactile.studies.tabular import (
    write_resolved_config,
    write_rows_csv_and_parquet,
)


_MAX_AUTO_JOBS = 12


def _available_cpu_count() -> int:
    """返回当前进程实际可用的 CPU 数，优先遵守亲和性限制。"""
    try:
        return max(1, len(os.sched_getaffinity(0)))
    except (AttributeError, OSError):
        return max(1, os.cpu_count() or 1)


def _resolve_worker_count(
    jobs: int | None,
    condition_count: int,
    *,
    available_cpus: int | None = None,
) -> int:
    """解析并行进程数；自动模式兼顾 CPU、条件数与内存压力。"""
    if condition_count < 1:
        raise ValueError("condition_count must be positive")
    if jobs is not None and jobs < 1:
        raise ValueError("jobs must be a positive integer")
    if jobs is not None:
        return min(jobs, condition_count)
    cpu_count = _available_cpu_count() if available_cpus is None else max(1, available_cpus)
    return min(cpu_count, condition_count, _MAX_AUTO_JOBS)


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按控制器汇总安全性、动作、振荡、时间和误差指标。"""
    groups: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        groups.setdefault(str(row["controller_variant"]), []).append(row)
    output: list[dict[str, object]] = []
    for controller, group in groups.items():
        settling = [
            float(row["settling_time_s"]) for row in group if row["settling_time_s"] is not None
        ]
        prediction = [
            float(row["prediction_mae_n"]) for row in group if row["prediction_mae_n"] is not None
        ]
        output.append(
            {
                "controller_variant": controller,
                "runs": len(group),
                "passed_runs": sum(bool(row["passed"]) for row in group),
                "safety_violations": sum(bool(row["safety_violated"]) for row in group),
                "safety_violation_duration_s_mean": fmean(
                    float(row["safety_violation_duration_s"]) for row in group
                ),
                "release_count_mean": fmean(float(row["release_count"]) for row in group),
                "action_count_mean": fmean(float(row["action_count"]) for row in group),
                "average_nonzero_action_step_mean": fmean(
                    float(row["average_nonzero_action_step"]) for row in group
                ),
                "command_movement_mean": fmean(
                    float(row["total_command_movement"]) for row in group
                ),
                "reverse_count_mean": fmean(float(row["reverse_count"]) for row in group),
                "oscillation_count_mean": fmean(float(row["oscillation_count"]) for row in group),
                "settling_time_s_mean": fmean(settling) if settling else None,
                "hold_ratio_mean": fmean(float(row["hold_ratio"]) for row in group),
                "steady_force_error_n_mean": fmean(
                    float(row["steady_force_error_n"]) for row in group
                ),
                "rmse_n_mean": fmean(float(row["rmse_n"]) for row in group),
                "peak_overshoot_n_mean": fmean(float(row["peak_overshoot_n"]) for row in group),
                "prediction_mae_n_mean": fmean(prediction) if prediction else None,
            }
        )
    return output


# 总览图中的控制器展示顺序：量化 PI 为连续对照，其余按消融链排序。
OVERVIEW_CONTROLLERS = (
    "quantized-pi",
    "fixed-step",
    "adaptive-deadband",
    "predictive",
    "dynamic-step",
)
# 离散控制器消融链：每个变体相对前一个只多一个机制。
# 链上各步新增的机制依次为自适应死区、一步预测、动态步长，见 docs/discrete-force-control.md。
ABLATION_CHAIN = ("fixed-step", "adaptive-deadband", "predictive", "dynamic-step")
CONTROLLER_COLOR = {
    "quantized-pi": "#7F7F7F",
    "fixed-step": "#56B4E9",
    "adaptive-deadband": "#0072B2",
    "predictive": "#E69F00",
    "dynamic-step": "#D55E00",
}
CONTROLLER_MARKER = {
    "quantized-pi": "s",
    "fixed-step": "o",
    "adaptive-deadband": "^",
    "predictive": "D",
    "dynamic-step": "v",
}
MATERIAL_ORDER = ("soft", "medium", "hard", "stiff")
MATERIAL_COLOR = {
    "soft": "#56B4E9",
    "medium": "#009E73",
    "hard": "#E69F00",
    "stiff": "#CC79A7",
}
MATERIAL_MARKER = {"soft": "o", "medium": "s", "hard": "^", "stiff": "D"}


def _condition_mean(
    rows: list[dict[str, object]],
    field: str,
    controller: str,
    *,
    material: str | None = None,
) -> float | None:
    """对指定控制器（可选限定材料）的全部条件求指标均值；无有效样本时返回 None。"""
    values = [
        float(row[field])
        for row in rows
        if row["controller_variant"] == controller
        and row[field] is not None
        and (material is None or row["object_material"] == material)
    ]
    if not values:
        return None
    return fmean(values)


def _mark_reference(containers: list) -> None:
    """给连续对照 quantized-pi 的柱形加斜纹，避免仅靠颜色区分参照系。"""
    for container in containers:
        container[0].set_hatch("//")
        container[0].set_edgecolor("white")
        container[0].set_linewidth(0.5)


def _metric_legend(axis, entries: list[tuple[str, str]], **kwargs: object) -> None:
    """用无斜纹的纯色块构建指标图例，避免图例继承对照柱的斜纹样式。"""
    from matplotlib import patches

    handles = [patches.Patch(facecolor=color, label=label) for label, color in entries]
    axis.legend(handles=handles, **kwargs)


def plot_controller_summary(rows: list[dict[str, object]], output: Path) -> Path:
    """按方案优先级生成控制器总览图：资格、精度、经济性与抖动四个面板。"""
    plt = science_pyplot()
    figure, axes = plt.subplots(2, 2, figsize=paper_figsize(5.6), layout="constrained")
    present = {str(row["controller_variant"]) for row in rows}
    controllers = [item for item in OVERVIEW_CONTROLLERS if item in present]
    x = np.arange(len(controllers))
    width = 0.38
    width_three = 0.27

    passed = [_condition_mean(rows, "passed", controller) for controller in controllers]
    settled = [
        fmean(
            float(row["settled_platform_count"]) / max(float(row["platform_count"]), 1.0)
            for row in rows
            if row["controller_variant"] == controller
        )
        for controller in controllers
    ]
    hold = [_condition_mean(rows, "hold_ratio", controller) for controller in controllers]
    group_passed = axes[0][0].bar(x - width_three, passed, width_three, color="#0072B2")
    group_settled = axes[0][0].bar(x, settled, width_three, color="#CC79A7")
    group_hold = axes[0][0].bar(x + width_three, hold, width_three, color="#009E73")
    _mark_reference([group_passed, group_settled, group_hold])
    axes[0][0].set_ylim(0.0, 1.45)
    axes[0][0].set_yticks([0.0, 0.25, 0.5, 0.75, 1.0])
    axes[0][0].set_ylabel("占比")
    _metric_legend(
        axes[0][0],
        [
            ("passed 条件占比", "#0072B2"),
            ("平台持续 HOLD 占比", "#CC79A7"),
            ("全程 HOLD 占比", "#009E73"),
        ],
        loc="upper left",
        fontsize=6,
    )
    axes[0][0].set_title("（a）资格与稳态质量", fontsize=9)

    steady = [
        _condition_mean(rows, "steady_force_error_n", controller) for controller in controllers
    ]
    rmse = [_condition_mean(rows, "rmse_n", controller) for controller in controllers]
    group_steady = axes[0][1].bar(x - width / 2, steady, width, color="#009E73")
    group_rmse = axes[0][1].bar(x + width / 2, rmse, width, color="#D55E00")
    _mark_reference([group_steady, group_rmse])
    axes[0][1].set_ylabel("力误差 / N")
    _metric_legend(
        axes[0][1],
        [("稳态误差", "#009E73"), ("RMSE", "#D55E00")],
        loc="upper left",
        fontsize=6.5,
    )
    axes[0][1].set_title("（b）跟踪精度", fontsize=9)

    actions = [_condition_mean(rows, "action_count", controller) for controller in controllers]
    movement = [
        _condition_mean(rows, "total_command_movement", controller) for controller in controllers
    ]
    group_actions = axes[1][0].bar(x - width / 2, actions, width, color="#0072B2")
    group_movement = axes[1][0].bar(x + width / 2, movement, width, color="#CC79A7")
    _mark_reference([group_actions, group_movement])
    axes[1][0].set_ylabel("tick")
    _metric_legend(
        axes[1][0],
        [("动作次数", "#0072B2"), ("总移动量", "#CC79A7")],
        loc="upper right",
        fontsize=6.5,
    )
    axes[1][0].set_title("（c）动作经济性", fontsize=9)

    reverse = [_condition_mean(rows, "reverse_count", controller) for controller in controllers]
    oscillation = [
        _condition_mean(rows, "oscillation_count", controller) for controller in controllers
    ]
    group_reverse = axes[1][1].bar(x, reverse, 0.55, color="#D55E00")
    axes[1][1].bar(x, oscillation, 0.55, bottom=reverse, color="#CC79A7")
    _mark_reference([group_reverse])
    axes[1][1].set_ylabel("平均抖动事件数")
    _metric_legend(
        axes[1][1],
        [("方向反转", "#D55E00"), ("相邻位置振荡", "#CC79A7")],
        loc="upper right",
        fontsize=6.5,
    )
    axes[1][1].set_title("（d）命令抖动", fontsize=9)

    for axis in axes.flat:
        axis.set_xticks(x, controllers, rotation=25, ha="right", fontsize=7)
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    pdf = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf


def plot_ablation_summary(rows: list[dict[str, object]], output: Path) -> Path:
    """生成离散控制器消融链图：动作、精度与稳定时间随机制累加的变化。"""
    plt = science_pyplot()
    figure, axes = plt.subplots(1, 3, figsize=paper_figsize(2.6), layout="constrained")
    present = {str(row["controller_variant"]) for row in rows}
    chain = [item for item in ABLATION_CHAIN if item in present]
    x = np.arange(len(chain))
    panels = (
        ("action_count", "平均动作次数 / tick"),
        ("rmse_n", "RMSE / N"),
        ("settling_time_s", "平均稳定时间 / s"),
    )
    present_materials = {str(row["object_material"]) for row in rows}
    materials = [item for item in MATERIAL_ORDER if item in present_materials]
    plotted = False
    for axis, (field, ylabel) in zip(axes, panels):
        for material in materials:
            points = [
                (x[index], value)
                for index, controller in enumerate(chain)
                if (value := _condition_mean(rows, field, controller, material=material))
                is not None
            ]
            if not points:
                continue
            plotted = True
            axis.plot(
                [point[0] for point in points],
                [point[1] for point in points],
                marker=MATERIAL_MARKER[material],
                color=MATERIAL_COLOR[material],
                label=material,
            )
        axis.set_xticks(x, chain, rotation=20, ha="right", fontsize=7)
        axis.set_ylabel(ylabel)
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    if plotted:
        axes[0].legend(fontsize=6.5, loc="upper right")
    pdf = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf


def plot_platform_error(platform_rows: list[dict[str, object]], output: Path) -> Path:
    """生成逐平台稳态误差图：按材料分面，观察局部增益随压缩程度的变化。"""
    plt = science_pyplot()
    present_materials = {str(row["object_material"]) for row in platform_rows}
    materials = [item for item in MATERIAL_ORDER if item in present_materials]
    figure, axes = plt.subplots(
        1, len(materials), figsize=paper_figsize(2.4), sharey=True, layout="constrained"
    )
    axes = np.atleast_1d(axes)
    for axis, material in zip(axes, materials):
        for controller in OVERVIEW_CONTROLLERS:
            samples: dict[float, list[float]] = {}
            for row in platform_rows:
                if row["controller_variant"] != controller or row["object_material"] != material:
                    continue
                target = float(row["target_force_n"])
                samples.setdefault(target, []).append(float(row["steady_force_error_n"]))
            if not samples:
                continue
            targets = sorted(samples)
            axis.plot(
                targets,
                [fmean(samples[target]) for target in targets],
                color=CONTROLLER_COLOR[controller],
                marker=CONTROLLER_MARKER[controller],
                markersize=3.5,
                linewidth=1.1,
                label=controller if material == materials[0] else None,
            )
        axis.set_xticks([2.0, 4.0, 6.0, 8.0])
        axis.set_title(material, fontsize=9)
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    axes[0].set_ylabel("平台稳态误差 / N")
    axes[0].legend(fontsize=6, loc="upper left")
    figure.supxlabel("目标力 / N")
    pdf = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf


def _create_study_directory(config: RobotiqDiscreteForceStudyConfig) -> Path:
    """创建一次独占的 study 目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def _evaluate_condition(
    payload: dict[str, object],
) -> tuple[int, dict[str, object], list[dict[str, object]]]:
    """运行单个 study 条件，返回条件序号、汇总行与平台指标行。

    作为进程池工作函数使用：各条件相互独立且种子固定，结果与串行执行一致。
    """

    def _base_row(run_path: Path) -> dict[str, object]:
        return {
            "controller_variant": payload["controller_variant"],
            "object_material": payload["object_material"],
            "force_noise_std_n": payload["force_noise_std_n"],
            "noise_seed": payload["noise_seed"],
            "run_directory": str(run_path.relative_to(payload["study_dir"])),
        }

    task = RobotiqDiscreteForceTask.load(Path(str(payload["task_path"])))
    run, result = execute_robotiq_discrete_force(
        profile=Path(str(payload["profile"])),
        task_path=Path(str(payload["task_path"])),
        discrete_task=task,
        output_root=Path(str(payload["output_root"])),
        run_prefix=str(payload["run_prefix"]),
        controller_variant=payload["controller_variant"],
        object_material=payload["object_material"],
        force_noise_std_n=float(payload["force_noise_std_n"]),
        noise_seed=int(payload["noise_seed"]),
    )
    result_values = asdict(result)
    platforms = result_values.pop("platform_metrics")
    row = {**_base_row(run.path), **result_values}
    platform_rows = [{**_base_row(run.path), **platform} for platform in platforms]
    return int(payload["condition_index"]), row, platform_rows


def run_study(
    config: RobotiqDiscreteForceStudyConfig,
    *,
    config_source: Path | None = None,
    jobs: int | None = None,
) -> Path:
    """执行矩阵全部条件并写入逐次、聚合、图像和 manifest 产物。

    Args:
        config: 已解析的 study 配置。
        config_source: 配置文件原始路径，会原样存档到 study 目录。
        jobs: 并行工作进程数；``None`` 自动选择，1 强制串行。

    Returns:
        本次 study 的输出目录。
    """
    conditions = config.conditions()
    worker_count = _resolve_worker_count(jobs, len(conditions))
    study_dir = _create_study_directory(config)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        (study_dir / "study.yaml").write_text(
            yaml.safe_dump(config.model_dump(mode="json"), sort_keys=False),
            encoding="utf-8",
        )
    write_resolved_config(study_dir / "study.resolved.json", config)
    payloads: list[dict[str, object]] = []
    for index, (controller, material, noise, seed) in enumerate(conditions):
        condition = f"{controller}-{material}-noise{noise:g}-seed{seed:03d}".replace(".", "p")
        payloads.append(
            {
                "condition_index": index,
                "profile": config.profile,
                "task_path": config.task,
                "study_dir": study_dir,
                "output_root": study_dir / "runs",
                "run_prefix": condition,
                "controller_variant": controller,
                "object_material": material,
                "force_noise_std_n": noise,
                "noise_seed": seed,
            }
        )

    results: list[tuple[int, dict[str, object], list[dict[str, object]]]] = []
    if worker_count == 1:
        for payload in payloads:
            results.append(_evaluate_condition(payload))
    else:
        print(f"使用 {worker_count} 个进程并行执行 {len(payloads)} 个条件", flush=True)
        with ProcessPoolExecutor(max_workers=worker_count) as executor:
            futures = {executor.submit(_evaluate_condition, item): item for item in payloads}
            for done, future in enumerate(as_completed(futures), start=1):
                results.append(future.result())
                print(f"[{done}/{len(futures)}] {futures[future]['run_prefix']} 完成", flush=True)
    results.sort(key=lambda item: item[0])
    rows = [row for _, row, _ in results]
    platform_rows = [
        platform_row
        for _, _, condition_platforms in results
        for platform_row in condition_platforms
    ]
    aggregates = aggregate_rows(rows)
    summary_csv, summary_parquet = write_rows_csv_and_parquet(study_dir / "summary.csv", rows)
    aggregate_csv, aggregate_parquet = write_rows_csv_and_parquet(
        study_dir / "aggregate.csv", aggregates
    )
    platforms_csv, platforms_parquet = write_rows_csv_and_parquet(
        study_dir / "platforms.csv", platform_rows
    )
    figure_pdf = plot_controller_summary(rows, study_dir / "controller_comparison.png")
    ablation_pdf = plot_ablation_summary(rows, study_dir / "controller_ablation.png")
    platform_pdf = plot_platform_error(platform_rows, study_dir / "platform_error.png")
    summary_json = study_dir / "summary.json"
    summary_json.write_text(
        json.dumps(
            {"runs": rows, "platforms": platform_rows, "aggregates": aggregates},
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    manifest = {
        "schema_version": 1,
        "runs": len(rows),
        "worker_count": worker_count,
        "passed_runs": sum(bool(row["passed"]) for row in rows),
        "safety_violations": sum(bool(row["safety_violated"]) for row in rows),
        "artifacts": sorted(
            path.name
            for path in (
                summary_csv,
                summary_parquet,
                aggregate_csv,
                aggregate_parquet,
                platforms_csv,
                platforms_parquet,
                summary_json,
                figure_pdf,
                figure_pdf.with_suffix(".png"),
                ablation_pdf,
                ablation_pdf.with_suffix(".png"),
                platform_pdf,
                platform_pdf.with_suffix(".png"),
            )
        ),
    }
    (study_dir / "study_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return study_dir


def main() -> None:
    """解析命令行，预览或执行离散力控制 study。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/studies/robotiq_discrete_force.yaml"),
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--jobs",
        type=int,
        default=None,
        help="并行工作进程数；默认自动选择且最多 12，设为 1 可强制串行",
    )
    arguments = parser.parse_args()
    config_path = arguments.config.resolve()
    config = load_robotiq_discrete_force_study_config(config_path)
    if arguments.dry_run:
        print(f"conditions={len(config.conditions())}")
        print(f"workers={_resolve_worker_count(arguments.jobs, len(config.conditions()))}")
        for condition in config.conditions():
            print(" ".join(str(value) for value in condition))
        return
    print(run_study(config, config_source=config_path, jobs=arguments.jobs))


if __name__ == "__main__":
    main()

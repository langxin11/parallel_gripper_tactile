"""串行运行 Robotiq 单 tick 力增量离散控制的完整消融矩阵。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
from datetime import UTC, datetime
import json
import math
from pathlib import Path
from statistics import fmean
from uuid import uuid4

import numpy as np
import yaml

from parallel_gripper_tactile.config.profiles import GripperProfile
from parallel_gripper_tactile.experiments.robotiq_discrete_force import (
    RobotiqDiscreteForceTask,
)
from parallel_gripper_tactile.runners import execute_robotiq_discrete_force
from parallel_gripper_tactile.studies.aggregation import (
    aggregate_records,
    bool_sum,
    count,
    key,
    optional_mean,
    plain_mean,
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
from parallel_gripper_tactile.studies.robotiq_discrete_force import (
    RobotiqDiscreteForceStudyConfig,
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


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    """按控制器汇总安全性、动作、振荡、时间和误差指标。"""
    return aggregate_records(
        rows,
        keys=("controller_variant",),
        columns=(
            key("controller_variant"),
            count("runs"),
            bool_sum("passed", "passed_runs"),
            bool_sum("safety_violated", "safety_violations"),
            plain_mean("safety_violation_duration_s"),
            plain_mean("release_count"),
            plain_mean("action_count"),
            plain_mean("average_nonzero_action_step"),
            plain_mean("total_command_movement", "command_movement_mean"),
            plain_mean("reverse_count"),
            plain_mean("oscillation_count"),
            optional_mean("settling_time_s"),
            plain_mean("hold_ratio"),
            plain_mean("steady_force_error_n"),
            plain_mean("rmse_n"),
            plain_mean("peak_overshoot_n"),
            optional_mean("prediction_mae_n"),
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


def build_plan(config: RobotiqDiscreteForceStudyConfig) -> StudyPlan:
    """从权威 domain config 生成离散力消融矩阵的唯一有序计划。"""
    conditions = tuple(
        StudyCondition(
            condition_id=f"{controller}-{material}-noise{noise}-seed{seed:03d}",
            parameters={
                "controller_variant": controller,
                "object_material": material,
                "force_noise_std_n": noise,
                "sensor_noise_seed": seed,
            },
            pair_key=f"{material}:noise{noise}:seed{seed:03d}",
            baseline_role="quantized_pi" if controller == "quantized-pi" else None,
        )
        for controller, material, noise, seed in config.conditions()
    )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "robotiq_discrete_force.v1",
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": file_sha256(config.profile),
            "task_sha256": file_sha256(config.task),
        },
        # 延续旧脚本的 17 列控制器聚合口径；quantized-pi 为连续对照基线。
        "aggregation": "controller_variant; safety/action/oscillation/error columns v1",
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
        study_kind="robotiq_discrete_force",
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=conditions,
        seeds=tuple(config.seeds.values()),
        preflight={"status": "pending", "checks": ["profile", "task", "materials"]},
    )


def run_study(
    config: RobotiqDiscreteForceStudyConfig,
    *,
    resolved_profile: GripperProfile | None = None,
    config_source: Path | None = None,
    study_directory: Path | None = None,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
) -> Path:
    """通过公共生命周期串行执行离散力消融矩阵，并返回 study 目录。"""
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
    # 逐平台指标行不进入 summary 行，按计划执行顺序单独收集，保持旧 platforms.csv 口径。
    platform_rows: list[dict[str, object]] = []

    def execute(condition: StudyCondition) -> ConditionExecution:
        parameters = condition.parameters
        controller_variant = str(parameters["controller_variant"])
        material = str(parameters["object_material"])
        noise_std_n = float(parameters["force_noise_std_n"])
        noise_seed = int(parameters["sensor_noise_seed"])
        # 沿用旧脚本的 run 目录命名：小数点替换为 p，避免噪声档位被误读为扩展名。
        run_prefix = (
            f"{controller_variant}-{material}-noise{noise_std_n:g}-seed{noise_seed:03d}".replace(
                ".", "p"
            )
        )

        def _base_row(run_path: Path) -> dict[str, object]:
            return {
                "controller_variant": controller_variant,
                "object_material": material,
                "force_noise_std_n": noise_std_n,
                "noise_seed": noise_seed,
                "run_directory": str(run_path.relative_to(study_dir)),
            }

        task = RobotiqDiscreteForceTask.load(config.task)
        run, result = execute_robotiq_discrete_force(
            profile=config.profile,
            resolved_profile=resolved_profile,
            task_path=config.task,
            discrete_task=task,
            output_root=study_dir / "runs",
            run_prefix=run_prefix,
            controller_variant=controller_variant,
            object_material=material,
            force_noise_std_n=noise_std_n,
            noise_seed=noise_seed,
        )
        result_values = asdict(result)
        platforms = result_values.pop("platform_metrics")
        row = {**_base_row(run.path), **result_values}
        platform_rows.extend({**_base_row(run.path), **platform} for platform in platforms)
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
        if platform_rows:
            artifacts.extend(write_rows_csv_and_parquet(directory / "platforms.csv", platform_rows))
        summary_json = directory / "summary.json"
        summary_json.write_text(
            json.dumps(
                json_compatible(
                    {
                        "runs": rows,
                        "platforms": platform_rows,
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
        controller_pdf = plot_controller_summary(rows, directory / "controller_comparison.png")
        ablation_pdf = plot_ablation_summary(rows, directory / "controller_ablation.png")
        platform_pdf = plot_platform_error(platform_rows, directory / "platform_error.png")
        return (
            controller_pdf.with_suffix(".png"),
            controller_pdf,
            ablation_pdf.with_suffix(".png"),
            ablation_pdf,
            platform_pdf.with_suffix(".png"),
            platform_pdf,
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
        initial_artifacts=(study_dir / "study.yaml", resolved_config, *additional_artifacts),
        legacy_manifest_fields=manifest_fields,
    )

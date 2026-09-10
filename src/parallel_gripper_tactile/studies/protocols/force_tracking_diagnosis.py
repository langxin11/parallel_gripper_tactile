"""运行单 phase 的力跟踪因果诊断研究。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict
from datetime import UTC, datetime
import json
import math
from pathlib import Path
from uuid import uuid4

import yaml

from parallel_gripper_tactile.control import ForceSemantics
from parallel_gripper_tactile.experiments.force_tracking import (
    ControllerVariant,
)
from parallel_gripper_tactile.visualization import (
    FULL_WIDTH_FONT_SCALE,
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.scenes.custom import ObjectContactModel, ObjectMaterial
from parallel_gripper_tactile.studies.force_tracking_diagnosis import (
    DiagnosisConfig,
    DiagnosisConfigError,
    Phase,
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
    read_trace_rows,
    write_resolved_config,
    write_rows_csv_and_parquet,
)


_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
_NUMERIC_SCAN_FIELDS: dict[Phase, tuple[str, str]] = {
    "force-scale": ("force_scale", "Target force scale"),
    "position-limit": ("max_position_adjustment_rad", "Max. position adjustment (rad)"),
    "integral-gain": ("pid_ki", "Integral gain Ki"),
    "filter-cutoff": ("filter_cutoff_hz", "Filter cutoff (Hz)"),
}
_DIAGNOSTIC_METRICS = (
    ("rmse_n", "RMSE (N)"),
    ("peak_abs_error_n", "Peak abs. error (N)"),
    ("torque_saturation_ratio", "Torque saturation ratio"),
    ("position_saturation_ratio", "Position saturation ratio"),
    ("contact_collapse_events", "Contact-collapse events"),
)
_EMPTY_CONTACT_DIAGNOSTICS: dict[str, object] = {
    "min_active_taxel_contacts": None,
    "max_active_taxel_contacts": None,
    "contact_collapse_events": None,
    "mean_left_force_n": None,
    "mean_right_force_n": None,
    "mean_average_side_force_n": None,
    "mean_total_force_n": None,
    "pid_position_adjustment_peak_to_peak_rad": None,
    "stiffness_position_adjustment_peak_to_peak_rad": None,
    "force_feedforward_torque_peak_to_peak_nm": None,
    "motor_torque_peak_to_peak_nm": None,
}
# 单因素设计中的对照条件：全控制器、标称力比例、显式接触模型与当前力语义。
_DIAGNOSIS_BASELINE_LABELS: dict[Phase, str] = {
    "controllers": "full",
    "force-scale": "scale-1.000",
    "contact-model": "explicit",
    "force-semantics": "average-8N-current",
}


def _study_directory(config: DiagnosisConfig, phase: Phase) -> Path:
    """为一个诊断 phase 创建独占目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / phase / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def _scaled_task(source: Path, scale: float, task_dir: Path) -> Path:
    """生成缩放后的 task，使每个 run 快照其实际力参考。"""
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise DiagnosisConfigError(f"力跟踪 task 必须是映射：{source}")
    approach = raw.get("approach")
    reference = raw.get("reference")
    if not isinstance(approach, dict) or not isinstance(reference, dict):
        raise DiagnosisConfigError(f"力跟踪 task 缺少 approach/reference 映射：{source}")
    approach["feedforward_force_n"] = float(approach["feedforward_force_n"]) * scale
    waypoints = reference.get("waypoints")
    if not isinstance(waypoints, list):
        raise DiagnosisConfigError(f"力跟踪 task 没有 waypoint 列表：{source}")
    for waypoint in waypoints:
        if not isinstance(waypoint, dict) or "force_n" not in waypoint:
            raise DiagnosisConfigError(f"力跟踪 task 含无效 waypoint：{source}")
        waypoint["force_n"] = float(waypoint["force_n"]) * scale
    task_dir.mkdir(parents=True, exist_ok=True)
    output = task_dir / f"force-scale-{scale:.3f}.yaml"
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return output


def _semantics_scaled_profile(source: Path, profile_dir: Path) -> Path:
    """生成等效平均单侧 task 所需的参数换算 profile。"""
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        force = raw["control"]["force"]
        stiffness = force["stiffness"]
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(f"profile 缺少 force/stiffness 设置：{source}") from error
    for key in ("kp", "ki", "kd"):
        force[key] = float(force[key]) * 2.0
    for key in ("initial_n_per_m", "min_n_per_m", "max_n_per_m", "min_delta_force_n"):
        stiffness[key] = float(stiffness[key]) * 0.5
    try:
        model_path = Path(raw["model"]["path"])
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(f"profile 缺少 model 路径：{source}") from error
    raw["model"]["path"] = str(
        model_path if model_path.is_absolute() else (source.parent / model_path).resolve()
    )
    profile_dir.mkdir(parents=True, exist_ok=True)
    output = profile_dir / "average-side-scaled-parameters.yaml"
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return output


def _tuning_profile(
    source: Path, profile_dir: Path, label: str, override: tuple[str, float]
) -> Path:
    """生成只覆盖一个力环参数的 profile。"""
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        force = raw["control"]["force"]
        model_path = Path(raw["model"]["path"])
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(f"profile 缺少 force/model 设置：{source}") from error
    key, value = override
    force[key] = value
    raw["model"]["path"] = str(
        model_path if model_path.is_absolute() else (source.parent / model_path).resolve()
    )
    profile_dir.mkdir(parents=True, exist_ok=True)
    output = profile_dir / f"{label}.yaml"
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return output


def _model_profile(source: Path, profile_dir: Path, label: str, model_path: Path) -> Path:
    """生成只修改引用碰撞模型的 profile。"""
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        raw["model"]["path"] = str(model_path.resolve())
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(f"profile 缺少 model 设置：{source}") from error
    profile_dir.mkdir(parents=True, exist_ok=True)
    output = profile_dir / f"{label}.yaml"
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return output


def _force_parameters(profile_path: Path) -> dict[str, float]:
    """读取诊断 run 记录所需的力控参数。"""
    raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    try:
        force = raw["control"]["force"]
        stiffness = force["stiffness"]
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(f"profile 缺少 force/stiffness 设置：{profile_path}") from error
    return {
        "pid_kp": float(force["kp"]),
        "pid_ki": float(force["ki"]),
        "pid_kd": float(force["kd"]),
        "stiffness_initial_n_per_m": float(stiffness["initial_n_per_m"]),
        "stiffness_min_n_per_m": float(stiffness["min_n_per_m"]),
        "stiffness_max_n_per_m": float(stiffness["max_n_per_m"]),
        "min_delta_force_n": float(stiffness["min_delta_force_n"]),
        "max_position_adjustment_rad": float(force["max_position_adjustment"]),
        "filter_cutoff_hz": float(force["filter_cutoff_hz"]),
    }


def _task_targets(task_path: Path, force_semantics: ForceSemantics) -> dict[str, float]:
    """返回所选力语义和物理力语义下的最终目标力。"""
    raw = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    try:
        target = float(raw["reference"]["waypoints"][-1]["force_n"])
    except (KeyError, IndexError, TypeError) as error:
        raise DiagnosisConfigError(f"task 缺少参考 waypoint：{task_path}") from error
    side = target if force_semantics == "average_side" else 0.5 * target
    return {
        "target_force_n": target,
        "equivalent_side_force_n": side,
        "equivalent_total_force_n": 2.0 * side,
    }


def _contact_diagnostics(run_directory: Path) -> dict[str, object]:
    """汇总 taxel 接触骤降和单侧原始力波动。"""
    try:
        rows = [
            row for row in read_trace_rows(run_directory) if row.get("phase") == "track_reference"
        ]
    except (FileNotFoundError, OSError):
        return dict(_EMPTY_CONTACT_DIAGNOSTICS)
    if not rows:
        return dict(_EMPTY_CONTACT_DIAGNOSTICS)
    try:
        contacts = [int(row["active_taxel_contacts"]) for row in rows]
        forces = [
            0.5
            * (float(row["left_taxel_normal_force_n"]) + float(row["right_taxel_normal_force_n"]))
            for row in rows
        ]
        left_forces = [float(row["left_taxel_normal_force_n"]) for row in rows]
        right_forces = [float(row["right_taxel_normal_force_n"]) for row in rows]
    except (KeyError, TypeError, ValueError):
        return dict(_EMPTY_CONTACT_DIAGNOSTICS)
    maximum = max(contacts)
    collapse_threshold = 0.5 * maximum
    events = sum(
        previous >= collapse_threshold and current < collapse_threshold
        for previous, current in zip(contacts, contacts[1:])
    )

    def peak_to_peak(column: str) -> float:
        values = [float(row[column]) for row in rows]
        return max(values) - min(values)

    try:
        return {
            "min_active_taxel_contacts": min(contacts),
            "max_active_taxel_contacts": maximum,
            "contact_collapse_events": events,
            "mean_left_force_n": sum(left_forces) / len(left_forces),
            "mean_right_force_n": sum(right_forces) / len(right_forces),
            "mean_average_side_force_n": sum(forces) / len(forces),
            "mean_total_force_n": 2.0 * sum(forces) / len(forces),
            "pid_position_adjustment_peak_to_peak_rad": peak_to_peak("pid_position_adjustment_rad"),
            "stiffness_position_adjustment_peak_to_peak_rad": peak_to_peak(
                "stiffness_position_adjustment_rad"
            ),
            "force_feedforward_torque_peak_to_peak_nm": peak_to_peak(
                "force_feedforward_torque_n_m"
            ),
            "motor_torque_peak_to_peak_nm": peak_to_peak("motor_torque_n_m"),
        }
    except (KeyError, TypeError, ValueError):
        return dict(_EMPTY_CONTACT_DIAGNOSTICS)


def _finite_metric(row: dict[str, object], field: str) -> float | None:
    """返回一项有限数值；缺失、无穷和非数值均视为不可绘制。"""
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def plot_diagnostic_metrics(rows: list[dict[str, object]], output: Path, *, phase: Phase) -> Path:
    """绘制单个诊断 phase 的误差、饱和和接触崩塌指标。

    数值扫描严格使用配置中的真实数值作为横轴；其余 phase 使用条件标签，避免将
    不可排序的分类变量伪装成连续量。
    """
    if not rows:
        raise ValueError("不能为没有运行结果的 phase 绘图。")
    plt = science_pyplot()
    numeric_scan = _NUMERIC_SCAN_FIELDS.get(phase)
    figure, axes = plt.subplots(2, 3, figsize=paper_figsize(5.8), layout="constrained")
    for axis, (field, metric_label) in zip(axes.flat, _DIAGNOSTIC_METRICS):
        values = [_finite_metric(row, field) for row in rows]
        if numeric_scan is not None:
            x_field, x_label = numeric_scan
            pairs = [
                (x_value, value)
                for row, value in zip(rows, values)
                if (x_value := _finite_metric(row, x_field)) is not None and value is not None
            ]
            pairs.sort(key=lambda pair: pair[0])
            if pairs:
                x_values, y_values = zip(*pairs)
                axis.plot(x_values, y_values, marker="o", linewidth=1.1)
            axis.set_xlabel(x_label)
        else:
            categories = [str(row.get("label", "unknown condition")) for row in rows]
            x_values = list(range(len(categories)))
            plotted = [math.nan if value is None else value for value in values]
            axis.bar(x_values, plotted, color="#0072B2")
            axis.set_xticks(x_values, categories, rotation=25, ha="right")
        axis.set_ylabel(metric_label)
        axis.grid(True, axis="y", linewidth=0.3, alpha=0.5)
    axes.flat[-1].set_visible(False)
    figure.suptitle(f"{phase}: causal force-tracking diagnostics")
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return pdf_path


def _read_tracking_rows(run_directory: Path) -> list[dict[str, object]]:
    """读取并校验一个 run 可用于叠加的跟踪阶段轨迹。"""
    try:
        rows = read_trace_rows(run_directory)
    except (FileNotFoundError, OSError):
        return []
    usable: list[dict[str, object]] = []
    for row in rows:
        if row.get("phase") != "track_reference":
            continue
        try:
            time_s = float(row["tracking_time_s"])
            target = float(row["target_normal_force_n"])
            filtered = float(row["filtered_normal_force_n"])
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in (time_s, target, filtered)):
            usable.append(
                {
                    "tracking_time_s": time_s,
                    "target_normal_force_n": target,
                    "filtered_normal_force_n": filtered,
                }
            )
    return usable


def _row_passed(row: dict[str, object]) -> bool:
    """判断运行是否通过，并兼容尚未写入 ``passed`` 的历史汇总。"""
    passed = row.get("passed")
    if isinstance(passed, str):
        return passed.lower() == "true"
    if passed is not None:
        return bool(passed)
    stable = row.get("simulation_stable")
    if isinstance(stable, str):
        stable = stable.lower() == "true"
    rmse = _finite_metric(row, "rmse_n")
    return bool(stable) and rmse is not None


def plot_tracking_overlay(
    rows: list[dict[str, object]], study_dir: Path, output: Path, *, phase: Phase
) -> list[Path]:
    """叠加所有有效运行的目标力和滤波力，并跳过无跟踪段的运行。"""
    traces: list[tuple[str, list[dict[str, object]]]] = []
    for row in rows:
        if not _row_passed(row):
            continue
        trace = _read_tracking_rows(study_dir / str(row.get("run_directory", "")))
        if trace:
            traces.append((str(row.get("label", "unknown condition")), trace))
    if not traces:
        return []

    plt = science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)
    figure, axis = plt.subplots(figsize=paper_figsize(4.0), layout="constrained")
    for index, (label, trace) in enumerate(traces):
        time_s = [float(item["tracking_time_s"]) for item in trace]
        if index == 0:
            axis.plot(
                time_s,
                [float(item["target_normal_force_n"]) for item in trace],
                color="black",
                linestyle="--",
                linewidth=1.0,
                label="Target",
            )
        axis.plot(
            time_s,
            [float(item["filtered_normal_force_n"]) for item in trace],
            linewidth=1.1,
            label=label,
        )
    axis.set_xlabel("Tracking time (s)")
    axis.set_ylabel("Normal force (N)")
    axis.set_title(f"{phase}: target and valid filtered-force trajectories")
    axis.grid(True, linewidth=0.3, alpha=0.5)
    axis.legend(frameon=False, ncol=2, fontsize=8)
    pdf_path = save_publication_figure(figure, output)
    plt.close(figure)
    return [output, pdf_path]


def render_phase_figures(
    rows: list[dict[str, object]], study_dir: Path, *, phase: Phase
) -> list[Path]:
    """生成一个 phase 的诊断指标图和有效运行轨迹叠加图。"""
    figures_dir = study_dir / "figures"
    figures_dir.mkdir(exist_ok=True)
    metrics_path = figures_dir / "diagnostic_metrics.png"
    metrics_pdf = plot_diagnostic_metrics(rows, metrics_path, phase=phase)
    trace_paths = plot_tracking_overlay(
        rows, study_dir, figures_dir / "tracking_overlay.png", phase=phase
    )
    return [metrics_path, metrics_pdf, *trace_paths]


def _conditions(
    config: DiagnosisConfig, phase: Phase
) -> tuple[
    tuple[
        str,
        ControllerVariant,
        ObjectMaterial,
        float,
        ObjectContactModel,
        ForceSemantics,
        bool,
        tuple[str, float] | None,
        Path | None,
        bool,
    ],
    ...,
]:
    """返回一个 phase 中刻意构造的单因素条件。"""
    if phase == "reproducibility":
        return tuple(
            (
                f"repeat-{index:02d}",
                "full",
                "hard",
                1.0,
                "explicit",
                "average_side",
                False,
                None,
                None,
                True,
            )
            for index in range(config.reproducibility_repeats)
        )
    if phase == "controllers":
        return tuple(
            (variant, variant, "hard", 1.0, "explicit", "average_side", False, None, None, True)
            for variant in config.controllers
        )
    if phase == "materials":
        return tuple(
            (material, "full", material, 1.0, "explicit", "average_side", False, None, None, True)
            for material in config.materials
        )
    if phase == "force-scale":
        return tuple(
            (
                f"scale-{scale:.3f}",
                "full",
                "hard",
                scale,
                "explicit",
                "average_side",
                False,
                None,
                None,
                True,
            )
            for scale in config.force_scales
        )
    if phase == "contact-model":
        return tuple(
            (model, "full", "hard", 0.5, model, "average_side", False, None, None, True)
            for model in config.contact_models
        )
    if phase == "collision-geometry":
        return tuple(
            (
                condition.label,
                "full",
                "hard",
                1.0,
                "explicit",
                "average_side",
                False,
                None,
                condition.model,
                condition.multiccd_enabled,
            )
            for condition in config.collision_geometry_models
        )
    if phase == "force-semantics":
        return (
            (
                "legacy-sum-8N",
                "full",
                "hard",
                1.0,
                "legacy",
                "total",
                False,
                None,
                None,
                True,
            ),
            (
                "average-4N-unscaled",
                "full",
                "hard",
                0.5,
                "legacy",
                "average_side",
                False,
                None,
                None,
                True,
            ),
            (
                "average-4N-scaled",
                "full",
                "hard",
                0.5,
                "legacy",
                "average_side",
                True,
                None,
                None,
                True,
            ),
            (
                "average-8N-current",
                "full",
                "hard",
                1.0,
                "explicit",
                "average_side",
                False,
                None,
                None,
                True,
            ),
        )
    if phase == "position-limit":
        return tuple(
            (
                f"limit-{value:.3f}",
                config.stability_controller,
                "hard",
                1.0,
                "explicit",
                "average_side",
                False,
                ("max_position_adjustment", value),
                None,
                True,
            )
            for value in config.position_adjustment_limits_rad
        )
    if phase == "integral-gain":
        return tuple(
            (
                f"ki-{value:.3f}",
                config.stability_controller,
                "hard",
                1.0,
                "explicit",
                "average_side",
                False,
                ("ki", value),
                None,
                True,
            )
            for value in config.integral_gains
        )
    return tuple(
        (
            f"cutoff-{value:.0f}",
            config.stability_controller,
            "hard",
            1.0,
            "explicit",
            "average_side",
            False,
            ("filter_cutoff_hz", value),
            None,
            True,
        )
        for value in config.filter_cutoffs_hz
    )


def _baseline_role(phase: Phase, label: str) -> str | None:
    """返回单因素设计中的对照条件角色；扫描与重复 phase 无对照。"""
    baseline = _DIAGNOSIS_BASELINE_LABELS.get(phase)
    return baseline if label == baseline else None


def build_plan(config: DiagnosisConfig, *, phase: Phase) -> StudyPlan:
    """从权威 domain config 与 phase 生成因果诊断的唯一有序计划。"""
    conditions = tuple(
        StudyCondition(
            condition_id=label,
            parameters={
                "label": label,
                "controller_variant": controller,
                "object_material": material,
                "object_contact_model": contact_model,
                "force_semantics": force_semantics,
                "force_scale": scale,
                "parameter_scale_conversion": scale_parameters,
                "swept_parameter": None if tuning_override is None else tuning_override[0],
                "swept_value": None if tuning_override is None else tuning_override[1],
                "collision_model": (
                    None if collision_model_path is None else str(collision_model_path)
                ),
                "multiccd_enabled": multiccd_enabled,
                "sensor_noise_seed": config.sensor_noise_seed,
            },
            pair_key=phase,
            baseline_role=_baseline_role(phase, label),
        )
        for (
            label,
            controller,
            material,
            scale,
            contact_model,
            force_semantics,
            scale_parameters,
            tuning_override,
            collision_model_path,
            multiccd_enabled,
        ) in _conditions(config, phase)
    )
    definition = {
        "hash_schema_version": 1,
        "protocol_revision": "force_tracking_diagnosis.v1",
        "phase": phase,
        "study": config.model_dump(mode="python", exclude={"output_root"}),
        "resources": {
            "profile_sha256": file_sha256(config.profile),
            "task_sha256": file_sha256(config.task),
            "collision_models": {
                condition.label: file_sha256(condition.model)
                for condition in config.collision_geometry_models
            },
        },
        "aggregation": "none; per-run causal single-factor rows v1",
    }
    definition_hash = scientific_configuration_hash(definition, repository_root=_REPOSITORY_ROOT)
    plan_hash = scientific_configuration_hash(
        {
            "study_definition_sha256": definition_hash,
            "stage": phase,
            "conditions": [condition.model_dump(mode="python") for condition in conditions],
        },
        repository_root=_REPOSITORY_ROOT,
    )
    return StudyPlan(
        study_kind="force_tracking_diagnosis",
        stage=phase,
        study_definition_sha256=definition_hash,
        scientific_configuration_sha256=plan_hash,
        conditions=conditions,
        seeds=(config.sensor_noise_seed,),
        preflight={"status": "pending", "checks": ["profile", "task", "collision_models"]},
    )


def run_study(
    config: DiagnosisConfig,
    *,
    phase: Phase,
    config_source: Path | None = None,
    study_directory: Path | None = None,
    study_plan: StudyPlan | None = None,
    additional_artifacts: Sequence[Path] = (),
    lifecycle_manifest_fields: Mapping[str, object] | None = None,
) -> Path:
    """通过公共生命周期执行一个诊断 phase，并返回 study 父目录。"""
    study_dir = (
        _study_directory(config, phase) if study_directory is None else study_directory.resolve()
    )
    study_dir.mkdir(parents=True, exist_ok=True)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        (study_dir / "study.yaml").write_text(
            yaml.safe_dump(config.model_dump(mode="json"), sort_keys=False), encoding="utf-8"
        )
    resolved_config = write_resolved_config(study_dir / "study.resolved.json", config)
    expected_plan = build_plan(config, phase=phase)
    plan = (
        expected_plan
        if study_plan is None
        else require_matching_study_plan(expected_plan, study_plan)
    )

    def execute(condition: StudyCondition) -> ConditionExecution:
        parameters = condition.parameters
        label = str(parameters["label"])
        task_path = _scaled_task(config.task, float(parameters["force_scale"]), study_dir / "tasks")
        profile_path = (
            _semantics_scaled_profile(config.profile, study_dir / "profiles")
            if bool(parameters["parameter_scale_conversion"])
            else config.profile
        )
        swept_parameter = parameters["swept_parameter"]
        if swept_parameter is not None:
            profile_path = _tuning_profile(
                config.profile,
                study_dir / "profiles",
                label,
                (str(swept_parameter), float(parameters["swept_value"])),
            )
        collision_model = parameters["collision_model"]
        if collision_model is not None:
            profile_path = _model_profile(
                config.profile,
                study_dir / "profiles",
                label,
                Path(str(collision_model)),
            )
        targets = _task_targets(task_path, str(parameters["force_semantics"]))
        run, result = execute_force_tracking(
            profile=profile_path,
            task_path=task_path,
            output_root=study_dir / "runs",
            run_prefix=condition.condition_id,
            object_material=str(parameters["object_material"]),
            object_contact_model=str(parameters["object_contact_model"]),
            multiccd_enabled=bool(parameters["multiccd_enabled"]),
            force_semantics=str(parameters["force_semantics"]),
            controller_variant=str(parameters["controller_variant"]),
            sensor_noise_seed=int(parameters["sensor_noise_seed"]),
        )
        row = {
            "label": label,
            "controller_variant": parameters["controller_variant"],
            "object_material": parameters["object_material"],
            "object_contact_model": parameters["object_contact_model"],
            "collision_model": (
                None if collision_model is None else Path(str(collision_model)).name
            ),
            "multiccd_enabled": parameters["multiccd_enabled"],
            "force_semantics": parameters["force_semantics"],
            "force_scale": parameters["force_scale"],
            "parameter_scale_conversion": parameters["parameter_scale_conversion"],
            "swept_parameter": swept_parameter,
            "swept_value": parameters["swept_value"],
            "sensor_noise_seed": int(parameters["sensor_noise_seed"]),
            "run_directory": str(run.path.relative_to(study_dir)),
            "passed": result.passed,
            **targets,
            **_force_parameters(profile_path),
            **asdict(result),
            **_contact_diagnostics(run.path),
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
        artifacts: list[Path] = []
        if rows:
            artifacts.extend(write_rows_csv_and_parquet(directory / "summary.csv", rows))
        summary_json = directory / "summary.json"
        summary_json.write_text(
            json.dumps(
                {
                    "phase": phase,
                    "runs": rows,
                    "failures": execution_failure_rows(outcomes),
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        artifacts.append(summary_json)
        return StudyPostprocessResult(tuple(artifacts), {}, rows)

    def render(rows: list[dict[str, object]], payload: object, directory: Path) -> tuple[Path, ...]:
        if not rows:
            return ()
        return tuple(render_phase_figures(rows, directory, phase=phase))

    manifest_fields: dict[str, object] = {
        "schema_version": 1,
        "name": config.name,
        "phase": phase,
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

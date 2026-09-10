"""运行纯力局部起滑的多种子与无起滑负例验证。"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import UTC, datetime
import json
import math
from pathlib import Path
from uuid import uuid4

import numpy as np
import yaml

from parallel_gripper_tactile.experiments.friction_estimation import FrictionEstimationTask
from parallel_gripper_tactile.visualization import (
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)
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
    load_local_slip_study_config,
)
from parallel_gripper_tactile.studies.tabular import (
    write_resolved_config,
    write_rows_csv_and_parquet,
)


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


def _json_compatible(value: object) -> object:
    """把非有限浮点数转换为标准 JSON 的 ``null``。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
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


def _create_study_directory(config: FrictionEstimationLocalSlipStudyConfig) -> Path:
    """创建一次独占的 study 目录。"""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def run_study(
    config: FrictionEstimationLocalSlipStudyConfig,
    *,
    config_source: Path | None = None,
) -> Path:
    """执行全部场景与 seed，并保存汇总表、图和 manifest。"""
    study_dir = _create_study_directory(config)
    if config_source is not None:
        (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    else:
        (study_dir / "study.yaml").write_text(
            yaml.safe_dump(config.model_dump(mode="json"), sort_keys=False),
            encoding="utf-8",
        )
    write_resolved_config(study_dir / "study.resolved.json", config)
    rows: list[dict[str, object]] = []
    for scenario, seed in config.conditions():
        task = FrictionEstimationTask.load(scenario.task)
        condition = f"{task.name}-seed{seed:03d}"
        run, result = execute_friction_estimation(
            profile=config.profile,
            task_path=scenario.task,
            estimation_task=task,
            output_root=study_dir / "runs",
            run_prefix=condition,
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
        event_expectation_passed = local_detected == scenario.expect_local_slip
        control_candidate_qualified = bool(
            scenario.expect_local_slip
            and local_estimate_ratio is not None
            and 0.55 <= local_estimate_ratio <= 1.02
        )
        lead = (
            None
            if result.probe_detection_time_s is None or result.local_slip_detection_time_s is None
            else result.probe_detection_time_s - result.local_slip_detection_time_s
        )
        rows.append(
            {
                "scenario": task.name,
                "task": str(scenario.task),
                "sensor_noise_seed": seed,
                "expect_local_slip": scenario.expect_local_slip,
                "local_slip_detected": local_detected,
                "event_expectation_passed": event_expectation_passed,
                "control_candidate_qualified": control_candidate_qualified,
                "validation_passed": (
                    event_expectation_passed
                    and (control_candidate_qualified if scenario.expect_local_slip else True)
                ),
                "local_estimate": local_estimate,
                "local_estimate_ratio": local_estimate_ratio,
                "detection_lead_s": lead,
                "run_directory": str(run.path.relative_to(study_dir)),
                **asdict(result),
            }
        )
    aggregates = aggregate_rows(rows)
    summary_csv, summary_parquet = write_rows_csv_and_parquet(study_dir / "summary.csv", rows)
    aggregate_csv, aggregate_parquet = write_rows_csv_and_parquet(
        study_dir / "aggregate.csv", aggregates
    )
    figure_pdf = plot_summary(rows, study_dir / "local_slip_validation.png")
    summary_json = study_dir / "summary.json"
    summary_json.write_text(
        json.dumps(
            _json_compatible({"runs": rows, "aggregates": aggregates}),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    manifest = {
        "schema_version": 1,
        "runs": len(rows),
        "all_event_expectations_passed": all(bool(row["event_expectation_passed"]) for row in rows),
        "all_expectations_passed": all(bool(row["validation_passed"]) for row in rows),
        "artifacts": sorted(
            path.name
            for path in (
                summary_csv,
                summary_parquet,
                aggregate_csv,
                aggregate_parquet,
                summary_json,
                figure_pdf,
                figure_pdf.with_suffix(".png"),
            )
        ),
    }
    (study_dir / "study_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return study_dir


def main() -> None:
    """解析命令行并执行 study。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/studies/friction_estimation_local_slip.yaml"),
    )
    arguments = parser.parse_args()
    config_path = arguments.config.resolve()
    study_dir = run_study(load_local_slip_study_config(config_path), config_source=config_path)
    print(study_dir)


if __name__ == "__main__":
    main()

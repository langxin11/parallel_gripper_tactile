"""运行可复现的力跟踪 controller × material × seed 消融研究。."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import UTC, datetime
import json
import math
from pathlib import Path
from statistics import fmean, stdev
from uuid import uuid4
import warnings

import numpy as np
import yaml

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.studies.force_tracking_ablation import (
    ForceTrackingAblationConfig,
    load_study_config,
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


def _write_rows_csv(path: Path, rows: list[dict[str, object]]) -> None:
    """写入同构的 study 结果行。."""
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
    _write_rows_csv(study_dir / "summary.csv", rows)
    _write_rows_csv(study_dir / "aggregate.csv", aggregates)
    (study_dir / "summary.json").write_text(
        json.dumps(
            json_compatible({"runs": rows, "aggregates": aggregates}), indent=2, sort_keys=True
        )
        + "\n",
        encoding="utf-8",
    )
    (study_dir / "study_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": config.name,
                "config": "study.yaml",
                "runs": [row["run_directory"] for row in rows],
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

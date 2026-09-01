"""验证刚度估计器对比 study 的配置、聚合和图表输出。"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.force_tracking_ablation import SeedSweep
from parallel_gripper_tactile.studies.force_tracking_stiffness_estimator_comparison import (
    ForceTrackingStiffnessEstimatorComparisonConfig,
)


ROOT = Path(__file__).resolve().parents[1]


def _protocol_module() -> object:
    """加载仓库内的估计器对比入口脚本。"""
    path = ROOT / "scripts/experiments/force_tracking_stiffness_estimator_comparison.py"
    spec = importlib.util.spec_from_file_location(
        "force_tracking_stiffness_estimator_comparison", path
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _result_row(estimator: str, *, seed: int, rmse: float) -> dict[str, object]:
    """构造一行完整的合成估计器 study 结果。"""
    return {
        "controller_variant": "pid-stiffness-ff",
        "stiffness_estimator_method": estimator,
        "task_name": "step_force_tracking",
        "task_path": "step.yaml",
        "object_material": "hard",
        "sensor_noise_seed": seed,
        "passed": True,
        "run_directory": "runs/example",
        "contact_time_s": 1.0,
        "tracking_start_time_s": 1.2,
        "tracking_duration_s": 4.0,
        "rmse_n": rmse,
        "mae_n": 0.1,
        "peak_abs_error_n": 0.3,
        "mean_error_n": 0.02,
        "final_error_n": 0.01,
        "torque_saturation_ratio": 0.0,
        "position_saturation_ratio": 0.0,
        "mean_estimated_stiffness_n_per_m": math.nan,
        "simulation_stable": True,
    }


def test_estimator_conditions_are_stable_and_fixed_to_position_feedforward(tmp_path: Path) -> None:
    """矩阵只展开估计器、任务、材料和 seed，不引入控制器混杂变量。"""
    protocol = _protocol_module()
    config = ForceTrackingStiffnessEstimatorComparisonConfig(
        profile=tmp_path / "profile.yaml",
        tasks=(tmp_path / "step.yaml",),
        estimators=("secant_ewma", "window_linear", "window_quadratic"),
        materials=("medium", "hard"),
        seeds=SeedSweep(start=3, count=2),
        output_root=tmp_path / "outputs",
    )

    description = protocol.describe_conditions(config)  # type: ignore[attr-defined]

    assert len(config.conditions()) == 12
    assert "Controller: pid-stiffness-ff" in description
    assert "001 estimator=secant_ewma task=step.yaml material=medium seed=3" in description
    assert "012 estimator=window_quadratic task=step.yaml material=hard seed=4" in description


def test_aggregate_rows_groups_estimator_task_and_material() -> None:
    """聚合键包含估计器、任务和材料，并忽略非有限指标。"""
    protocol = _protocol_module()
    rows = [
        _result_row("window_linear", seed=0, rmse=0.2),
        _result_row("window_linear", seed=1, rmse=0.4),
    ]

    aggregate = protocol.aggregate_rows(rows)[0]  # type: ignore[attr-defined]

    assert aggregate["stiffness_estimator_method"] == "window_linear"
    assert aggregate["task_name"] == "step_force_tracking"
    assert aggregate["object_material"] == "hard"
    assert aggregate["runs"] == 2
    assert aggregate["rmse_n_mean"] == pytest.approx(0.3)
    assert aggregate["rmse_n_std"] == pytest.approx(2**0.5 * 0.1)
    assert aggregate["mean_estimated_stiffness_n_per_m_mean"] is None


def test_estimator_summary_plots_are_generated(tmp_path: Path) -> None:
    """指标和相对割线的增量图均生成非空 PNG 与 PDF。"""
    protocol = _protocol_module()
    estimators = ("secant_ewma", "window_linear", "window_quadratic")
    aggregates = protocol.aggregate_rows(  # type: ignore[attr-defined]
        [
            _result_row(estimator, seed=0, rmse=0.2 + 0.05 * index)
            for index, estimator in enumerate(estimators)
        ]
    )
    outputs = (tmp_path / "metrics.png", tmp_path / "delta.png")

    protocol.plot_metric_summary(  # type: ignore[attr-defined]
        aggregates, outputs[0], estimator_order=estimators
    )
    protocol.plot_delta_vs_secant(  # type: ignore[attr-defined]
        aggregates, outputs[1], estimator_order=estimators
    )

    artifacts = (*outputs, *(path.with_suffix(".pdf") for path in outputs))
    assert all(path.is_file() and path.stat().st_size > 0 for path in artifacts)

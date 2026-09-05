"""验证微滑移探测、保守摩擦估计与估计值调度实验。"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from parallel_gripper_tactile.experiments.friction_estimation import (
    FrictionEstimationConfigError,
    FrictionEstimationTask,
    run_friction_estimation,
)
from parallel_gripper_tactile.runners import execute_friction_estimation


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/custom_parallel_gripper.yaml"
TASK_ROOT = ROOT / "configs/friction_estimation"
STANDARD_TASKS = tuple(
    TASK_ROOT / name
    for name in (
        "low_friction.yaml",
        "nominal_friction.yaml",
        "high_friction.yaml",
        "noisy_friction.yaml",
    )
)


@pytest.mark.parametrize("task_path", STANDARD_TASKS)
def test_standard_friction_estimation_tasks_load(task_path: Path) -> None:
    """低、中、高摩擦和噪声场景均通过严格配置校验。"""
    task = FrictionEstimationTask.load(task_path)

    assert task.schema_version == 1
    assert task.solver.noslip_iterations == 5
    assert task.estimator.ratio_trend_enabled
    assert task.taxel_observer.contact_exit_force_n < task.taxel_observer.contact_enter_force_n
    assert task.probe.maximum_duration_s > 0


def test_friction_estimation_task_rejects_unknown_fields(tmp_path: Path) -> None:
    """摩擦估计 task 拒绝未声明字段。"""
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(
        """schema_version: 1
name: invalid
unexpected: true
downward_load:
  waypoints:
    - {t_s: 0.0, force_n: 0.0}
    - {t_s: 1.0, force_n: 0.0}
""",
        encoding="utf-8",
    )

    with pytest.raises(FrictionEstimationConfigError, match="unexpected"):
        FrictionEstimationTask.load(invalid)


@pytest.mark.parametrize("task_path", STANDARD_TASKS)
def test_standard_scenarios_estimate_conservatively_and_hold(task_path: Path) -> None:
    """标准场景都以保守估计完成探测，并用估计值稳定抵抗后续载荷。"""
    task = FrictionEstimationTask.load(task_path)

    result = run_friction_estimation(PROFILE, task=task)

    assert result.passed
    assert result.slip_detected
    assert not result.using_fallback
    assert result.estimated_friction_coefficient <= (
        task.friction_coefficient + task.metrics.conservative_tolerance
    )
    assert result.estimate_ratio >= task.metrics.minimum_estimate_ratio
    assert result.max_probe_displacement_m <= task.metrics.probe_slip_threshold_m
    assert result.max_hold_displacement_m <= task.metrics.hold_slip_threshold_m
    assert result.minimum_hold_friction_margin_n > 0
    assert result.peak_active_taxel_count > 0
    assert result.local_weighted_ratio_at_detection is not None
    assert result.local_ratio_p90_at_detection is not None
    assert result.local_slip_detection_time_s is not None
    assert result.local_slip_detection_time_s <= result.probe_detection_time_s
    assert result.local_slip_detected_taxel_count > 0
    local_estimates = [
        estimate
        for estimate in (
            result.local_left_friction_estimate,
            result.local_right_friction_estimate,
        )
        if estimate is not None
    ]
    assert local_estimates
    assert min(local_estimates) <= task.friction_coefficient + task.metrics.conservative_tolerance
    assert min(local_estimates) / task.friction_coefficient >= 0.55


def test_uninformative_probe_uses_explicit_fallback() -> None:
    """探测量程不足时不把静摩擦利用率冒充 μ，而是明确使用保守回退值。"""
    task = FrictionEstimationTask.load(TASK_ROOT / "nominal_friction.yaml")
    weak_probe = task.probe.model_copy(update={"max_force_n": 0.2})
    relaxed_metrics = task.metrics.model_copy(
        update={"require_detection": False, "minimum_estimate_ratio": 0.0}
    )
    fallback_task = task.model_copy(update={"probe": weak_probe, "metrics": relaxed_metrics})

    result = run_friction_estimation(PROFILE, task=fallback_task)

    assert result.passed
    assert not result.slip_detected
    assert result.using_fallback
    assert result.raw_friction_coefficient is None
    assert result.estimated_friction_coefficient == pytest.approx(
        task.estimator.fallback_friction_coefficient
    )


def test_disabling_noslip_invalidates_the_identification_task() -> None:
    """关闭 NoSlip 后锥内数值爬移会破坏探测和后续保持验收。"""
    task = FrictionEstimationTask.load(TASK_ROOT / "nominal_friction.yaml")
    no_noslip_task = task.model_copy(
        update={"solver": task.solver.model_copy(update={"noslip_iterations": 0})}
    )

    result = run_friction_estimation(PROFILE, task=no_noslip_task)

    assert not result.passed
    assert not result.probe_slip_passed
    assert not result.hold_slip_passed


def test_execute_friction_estimation_writes_blind_estimator_artifacts(tmp_path: Path) -> None:
    """执行器保存输入、轨迹、图表、指标及不含 oracle 输入的运行声明。"""
    task_path = TASK_ROOT / "low_friction.yaml"
    task = FrictionEstimationTask.load(task_path)

    run, result = execute_friction_estimation(
        profile=PROFILE,
        task_path=task_path,
        estimation_task=task,
        output_root=tmp_path,
        run_name="friction-test",
    )

    assert result.passed
    expected = {
        "profile.yaml",
        "task.yaml",
        "effective_parameters.json",
        "trace.csv",
        "plot.png",
        "plot.pdf",
        "taxel_plot.png",
        "taxel_plot.pdf",
        "metrics.json",
        "manifest.json",
    }
    assert expected == {path.name for path in run.path.iterdir()}
    effective = json.loads((run.path / "effective_parameters.json").read_text(encoding="utf-8"))
    assert effective["runtime"]["scheduler_kind"] == "estimated_friction"
    assert effective["runtime"]["oracle_signals_used_by_estimator"] == []
    assert effective["runtime"]["taxel_observer_kind"] == (
        "contact_hysteresis_local_friction_ratio"
    )
    assert effective["runtime"]["taxel_slip_detector_kind"] == (
        "force_ratio_saturation_and_redistribution"
    )
    with (run.path / "trace.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert {row["phase"] for row in rows} >= {"probe", "recovery", "schedule_load"}
    assert "left_taxel_ratio_0_0" in rows[0]
    assert "right_taxel_contact_2_2" in rows[0]
    assert max(int(row["active_taxel_count"]) for row in rows) > 0
    assert "left_taxel_slip_detected_0_0" in rows[0]
    assert max(int(row["local_slip_detected_count"]) for row in rows) > 0

"""验证目标力调度任务、仿真场景和运行工件。"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from parallel_gripper_tactile.experiments.force_scheduling import (
    DownwardLoadReference,
    DownwardLoadWaypoint,
    ForceSchedulingConfigError,
    ForceSchedulingTask,
    run_force_scheduling,
)
from parallel_gripper_tactile.runners import execute_force_scheduling


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/dm_gripper.yaml"
GRAVITY_TASK = ROOT / "configs/force_scheduling/gravity_hold.yaml"
FILLING_TASK = ROOT / "configs/force_scheduling/dynamic_filling.yaml"


def test_downward_load_reference_samples_linear_ramp() -> None:
    """重力方向附加载荷支持线性插值及变化率。"""
    reference = DownwardLoadReference(
        interpolation="linear",
        waypoints=(
            DownwardLoadWaypoint(t_s=0.0, force_n=0.0),
            DownwardLoadWaypoint(t_s=2.0, force_n=1.0),
        ),
    )

    assert reference.sample_at(1.0) == pytest.approx((0.5, 0.5))
    assert reference.sample_at(3.0) == pytest.approx((1.0, 0.0))


def test_force_scheduling_task_rejects_nonzero_start_time(tmp_path: Path) -> None:
    """载荷曲线必须从场景零时刻开始。"""
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(
        """schema_version: 1
name: invalid
downward_load:
  waypoints:
    - {t_s: 1.0, force_n: 0.0}
    - {t_s: 2.0, force_n: 1.0}
""",
        encoding="utf-8",
    )

    with pytest.raises(ForceSchedulingConfigError, match="must start at t_s=0"):
        ForceSchedulingTask.load(invalid)


@pytest.mark.parametrize("task_path", [GRAVITY_TASK, FILLING_TASK])
def test_standard_force_scheduling_tasks_load(task_path: Path) -> None:
    """两个标准目标力调度场景都通过严格配置校验。"""
    task = ForceSchedulingTask.load(task_path)

    assert task.schema_version == 1
    assert task.solver.noslip_iterations == 5
    assert task.downward_load.duration_s > 0


def test_gravity_hold_uses_small_target_without_slip() -> None:
    """仅克服 50 g 方块重力时，0.5 N 平均单侧目标即可稳定保持。"""
    task = ForceSchedulingTask.load(GRAVITY_TASK)

    result = run_force_scheduling(PROFILE, task=task)

    assert result.passed
    assert result.mean_target_force_n == pytest.approx(0.5)
    assert result.max_tangential_displacement_m < task.metrics.slip_threshold_m
    assert result.minimum_friction_margin_n > 0


def test_dynamic_filling_increases_target_and_remains_stable(tmp_path: Path) -> None:
    """模拟注水的持续增载会提高目标力，并在完整斜坡和保持段内不滑移。"""
    task = ForceSchedulingTask.load(FILLING_TASK)
    trace_path = tmp_path / "trace.csv"

    result = run_force_scheduling(PROFILE, task=task, output_csv=trace_path)

    assert result.passed
    assert result.final_target_force_n > 2.0
    assert result.final_target_force_n > result.mean_target_force_n
    assert result.max_tangential_displacement_m < task.metrics.slip_threshold_m
    with trace_path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["phase"] == "schedule_load"]
    assert rows
    assert float(rows[-1]["additional_downward_force_n"]) == pytest.approx(2.0)
    assert float(rows[-1]["scheduled_target_force_n"]) > float(rows[0]["scheduled_target_force_n"])


def test_noslip_solver_does_not_hide_insufficient_grip() -> None:
    """目标力被限制在 0.5 N 时仍会真实滑落，noslip 只消除锥内数值爬移。"""
    task = ForceSchedulingTask.load(FILLING_TASK)
    weak_scheduler = task.scheduler.model_copy(update={"max_force_n": 0.5})
    weak_task = task.model_copy(update={"scheduler": weak_scheduler})

    result = run_force_scheduling(PROFILE, task=weak_task)

    assert not result.passed
    assert not result.slip_passed
    assert result.target_force_maximum_ratio > 0.5
    assert result.minimum_friction_margin_n < 0


def test_execute_force_scheduling_writes_reproducible_artifacts(
    tmp_path: Path, fast_png_render: None
) -> None:
    """执行器保存任务快照、有效参数、轨迹、图表和指标。"""
    task = ForceSchedulingTask.load(GRAVITY_TASK)

    run, result = execute_force_scheduling(
        profile=PROFILE,
        task_path=GRAVITY_TASK,
        scheduling_task=task,
        output_root=tmp_path,
        run_name="gravity-test",
    )

    assert result.passed
    expected = {
        "profile.yaml",
        "task.yaml",
        "effective_parameters.json",
        "trace.csv",
        "plot.png",
        "plot.pdf",
        "metrics.json",
        "manifest.json",
    }
    assert expected == {path.name for path in run.path.iterdir()}
    metrics = json.loads((run.path / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["slip_passed"] is True
    effective = json.loads((run.path / "effective_parameters.json").read_text(encoding="utf-8"))
    assert effective["runtime"]["scheduler_kind"] == "oracle"

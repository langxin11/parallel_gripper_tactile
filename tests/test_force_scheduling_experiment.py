"""验证目标力调度任务、仿真场景和运行工件。"""

from __future__ import annotations

import csv
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from parallel_gripper_tactile.experiments.force_scheduling import (
    AdaptivePriorTaskConfig,
    DownwardLoadReference,
    DownwardLoadWaypoint,
    ForceSchedulingConfigError,
    ForceSchedulingTask,
    _plot_force_scheduling,
    run_force_scheduling,
)
from parallel_gripper_tactile.runners import execute_force_scheduling
from parallel_gripper_tactile.research import compose_research_run


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/dm_gripper.yaml"
GRAVITY_TASK = ROOT / "configs/task/force_scheduling/gravity_hold.yaml"
FILLING_TASK = ROOT / "configs/task/force_scheduling/dynamic_filling.yaml"


@pytest.mark.parametrize("transient", [False, True])
def test_unified_fast_candidate_only_changes_admittance_limits(tmp_path, transient) -> None:
    """候选只覆盖指定参数；瞬态回归额外检查未排除初始窗口的位移。"""
    baseline = compose_research_run(experiment="dm_gripper/unified_step_load")
    suffix = "transient" if transient else "fast"
    candidate = compose_research_run(experiment=f"dm_gripper/unified_step_load_{suffix}")
    expected_task = baseline.task
    if transient:
        config = expected_task.unified_adaptive
        expected_task = expected_task.model_copy(
            update={
                "unified_adaptive": replace(config, load=replace(config.load, filter_tau_s=0.01))
            }
        )
    assert candidate.task == expected_task
    expected = baseline.profile.model_dump(mode="json")
    admittance = expected["control"]["force"]["admittance"]
    assert admittance["velocity_limit_rad_s"] == 0.05
    assert admittance["feedforward_ratio"] == 0.2
    admittance.update(velocity_limit_rad_s=0.2, feedforward_ratio=1.0)
    assert candidate.profile.model_dump(mode="json") == expected
    trace = tmp_path / "trace.csv"
    result = run_force_scheduling(candidate.profile, task=candidate.task, output_csv=trace)
    assert result.simulation_stable and result.force_tracking_passed
    assert result.failure_reason is None
    if transient:
        assert result.passed
        with trace.open() as handle:
            rows = [r for r in csv.DictReader(handle) if r["phase"] == "schedule_load"]
        assert max(abs(float(r["tangential_displacement_m"])) for r in rows) < 0.002
        assert any(
            float(r["scenario_time_s"]) <= 0.1 and float(r["filtered_normal_force_n"]) >= 2.0
            for r in rows
        )
        assert all(r["adaptive_increase_count"] == "0" for r in rows)
    else:
        assert 0.003 < result.max_tangential_displacement_m < 0.005
        assert not result.slip_passed and not result.passed


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_unified_pid_comparison_preserves_policy_and_feedforward(tmp_path, seed) -> None:
    """同一上层配置接入真实 PID 力矩前馈；临界滑移结果不能统一标成成功。"""
    baseline = compose_research_run(
        experiment="dm_gripper/unified_step_load", overrides=[f"seed={seed}"]
    )
    resolved = compose_research_run(
        experiment="dm_gripper/unified_step_load_pid", overrides=[f"seed={seed}"]
    )
    assert resolved.task == baseline.task
    assert resolved.profile.mit == baseline.profile.mit
    force = resolved.profile.normal_force
    assert force.filter_cutoff_hz == baseline.profile.normal_force.filter_cutoff_hz
    assert force.stiffness.enabled and force.stiffness.position_feedforward_gain == 0
    assert force.stiffness.torque_feedforward_gain == 1
    trace = tmp_path / "trace.csv"
    result = run_force_scheduling(resolved.profile, task=resolved.task, output_csv=trace)
    assert result.simulation_stable and result.force_tracking_passed
    assert result.failure_reason is None
    assert result.max_tangential_displacement_m < 0.003
    assert result.passed == (
        result.max_tangential_displacement_m <= resolved.task.metrics.slip_threshold_m
    )
    with trace.open() as handle:
        rows = [r for r in csv.DictReader(handle) if r["phase"] == "schedule_load"]
    assert max(float(r["force_feedforward_torque_n_m"]) for r in rows) > 0
    assert max(abs(float(r["motor_torque_n_m"])) for r in rows) <= resolved.profile.mit.t_max
    assert all(r["adaptive_increase_count"] == "0" for r in rows)


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


@pytest.mark.parametrize("has_tactile", [True, False])
def test_plot_separates_tactile_and_ground_truth(tmp_path, monkeypatch, has_tactile) -> None:
    """失接触时实测为零而真值载荷仍为 2.5 N；旧记录不伪造缺失观测。"""
    from parallel_gripper_tactile.experiments import force_scheduling as module

    figures = []
    monkeypatch.setattr(
        module, "save_publication_figure", lambda fig, path: figures.append((fig, path))
    )
    rows = [
        dict(
            phase="schedule_load",
            scenario_time_s=t,
            scheduled_target_force_n=0.7,
            filtered_normal_force_n=0,
            tangential_demand_n=2.5,
            available_friction_n=0,
            tangential_displacement_m=0.1,
        )
        for t in (0, 1)
    ]
    if has_tactile:
        for row in rows:
            row.update(measured_left_tangential_n=0, measured_right_tangential_n=0)
    _plot_force_scheduling(tmp_path / "plot.png", rows, task=ForceSchedulingTask.load(GRAVITY_TASK))
    assert [path.suffix for _, path in figures] == [".png", ".pdf"]
    axes = figures[0][0].axes
    assert len(axes) == 3
    curves = {line.get_label(): line for line in axes[1].lines}
    assert (r"$T$ (tactile)" in curves) is has_tactile
    np.testing.assert_array_equal(curves[r"$D^{\mathrm{gt}}$ (true load)"].get_ydata(), [2.5, 2.5])
    np.testing.assert_array_equal(curves[r"$C^{\mathrm{gt}}$ (true capacity)"].get_ydata(), [0, 0])
    if has_tactile:
        np.testing.assert_array_equal(curves[r"$T$ (tactile)"].get_ydata(), [0, 0])


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


@pytest.mark.parametrize("mode", ["oracle", "adaptive_prior", "unified_adaptive"])
def test_noslip_solver_does_not_hide_insufficient_grip(mode: str) -> None:
    """目标力被限制在 0.5 N 时仍会真实滑落，noslip 只消除锥内数值爬移。"""
    task = ForceSchedulingTask.load(FILLING_TASK)
    weak_scheduler = task.scheduler.model_copy(update={"max_force_n": 0.5})
    weak_task = task.model_copy(update={"scheduler": weak_scheduler})
    profile = PROFILE
    if mode == "adaptive_prior":
        weak_task = weak_task.model_copy(update={"adaptive_prior": AdaptivePriorTaskConfig()})
    elif mode == "unified_adaptive":
        resolved = compose_research_run(experiment="dm_gripper/unified_adaptive")
        profile = resolved.profile
        unified = resolved.task.unified_adaptive
        weak_task = resolved.task.model_copy(
            update={
                "unified_adaptive": replace(unified, load=replace(unified.load, max_force_n=0.5))
            }
        )

    result = run_force_scheduling(profile, task=weak_task)

    assert not result.passed
    assert not result.slip_passed
    if mode == "oracle":
        assert result.target_force_maximum_ratio > 0.5
    assert result.minimum_friction_margin_n < 0
    if mode != "oracle":
        # 滑落失接触后测得载荷会下降，不能沿用真值需求的超限时长。
        assert result.capacity_limited
    if mode == "unified_adaptive":
        assert result.failure_reason is not None


@pytest.mark.parametrize("mode", ["oracle", "adaptive_prior", "unified_adaptive"])
def test_execute_force_scheduling_writes_reproducible_artifacts(
    tmp_path: Path, fast_png_render: None, mode: str
) -> None:
    """执行器保存任务快照、有效参数、轨迹、图表和指标。"""
    task = ForceSchedulingTask.load(GRAVITY_TASK)
    profile = PROFILE
    if mode == "adaptive_prior":
        task = task.model_copy(update={"adaptive_prior": AdaptivePriorTaskConfig()})
    elif mode == "unified_adaptive":
        resolved = compose_research_run(experiment="dm_gripper/unified_adaptive")
        profile = resolved.profile
        task = resolved.task.model_copy(update={"downward_load": task.downward_load})

    run, result = execute_force_scheduling(
        profile=PROFILE,
        resolved_profile=profile if mode == "unified_adaptive" else None,
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
    assert effective["runtime"]["scheduler_kind"] == mode
    if mode == "unified_adaptive":
        assert metrics["failure_reason"] is None
        assert effective["task"]["unified_adaptive"]["risk_enabled"] is False


@pytest.mark.parametrize("scenario", ["steady", "ramp", "step"])
@pytest.mark.parametrize("mode", ["adaptive_prior", "unified_adaptive"])
def test_adaptive_prior_simulation_tracks_measured_load(
    tmp_path: Path, scenario: str, mode: str
) -> None:
    """比较恒载、缓增与自重 2.5 N 的撤支撑；不以外力代替新增质量。"""
    resolved = compose_research_run(experiment=f"dm_gripper/{mode}")
    task = resolved.task
    if scenario == "steady":
        load = {"waypoints": [{"t_s": 0, "force_n": 0}, {"t_s": 3, "force_n": 0}]}
    elif scenario == "step":
        step_task = compose_research_run(experiment="dm_gripper/unified_step_load").task
        task = task.model_copy(update={"cube_mass_kg": step_task.cube_mass_kg})
        if mode == "unified_adaptive":
            task = task.model_copy(update={"unified_adaptive": step_task.unified_adaptive})
        load = step_task.downward_load.model_dump()
    else:
        load = task.downward_load.model_dump()
    task = task.model_copy(update={"downward_load": DownwardLoadReference.model_validate(load)})
    trace = tmp_path / "trace.csv"
    result = run_force_scheduling(resolved.profile, task=task, output_csv=trace)
    assert result.simulation_stable
    if scenario == "step":
        assert not result.passed
        assert not result.slip_passed
        assert result.max_tangential_displacement_m > task.metrics.slip_threshold_m
        assert task.cube_mass_kg * 9.81 == pytest.approx(2.5)
        assert all(point.force_n == 0 for point in task.downward_load.waypoints)
    else:
        assert result.passed
    with trace.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    targets = np.array([float(row["scheduled_target_force_n"]) for row in rows])
    if scenario == "step":
        assert all(float(row["cube_mass_kg"]) == pytest.approx(2.5 / 9.81) for row in rows)
        assert all(float(row["gravity_tangential_force_n"]) == pytest.approx(2.5) for row in rows)
        assert all(float(row["additional_downward_force_n"]) == 0 for row in rows)
        assert result.scenario_start_time_s > result.contact_time_s
    assert np.all(np.diff(targets) >= -1e-12)
    max_rate = task.unified_adaptive.load.max_force_rate_n_s if mode == "unified_adaptive" else 1
    assert max(float(row["scheduled_target_force_rate_n_s"]) for row in rows) <= max_rate + 1e-12
    assert all(row["capacity_limited"] == "False" for row in rows)
    assert result.final_target_force_n > 0.5
    if mode == "unified_adaptive":
        assert result.failure_reason is None
        if scenario == "step":
            assert targets[0] == 1
            assert task.unified_adaptive.load.max_force_rate_n_s == 50
            assert result.force_tracking_passed
            assert result.max_tangential_displacement_m < 0.01
        assert all(row["adaptive_left_mu"] == "0.6" for row in rows)
        assert all(row["adaptive_increase_count"] == "0" for row in rows)

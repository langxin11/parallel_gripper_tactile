"""验证目标力调度任务、仿真场景和运行工件。"""

from __future__ import annotations

import csv
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from dm_grasp_core.tactile.multirate import TactileSamplingConfig
from parallel_gripper_tactile.experiments.force_scheduling import (
    AdaptivePriorTaskConfig,
    DownwardLoadReference,
    DownwardLoadWaypoint,
    ForceSchedulingConfigError,
    ForceSchedulingTask,
    _plot_force_scheduling,
    run_force_scheduling,
    TactileFaultConfig,
)
from parallel_gripper_tactile.runners import execute_force_scheduling
from parallel_gripper_tactile.research import compose_research_run


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/dm_gripper.yaml"
GRAVITY_TASK = ROOT / "configs/task/force_scheduling/gravity_hold.yaml"
FILLING_TASK = ROOT / "configs/task/force_scheduling/dynamic_filling.yaml"


@pytest.mark.parametrize(
    "fault",
    [
        TactileFaultConfig(drop_frames=16),
        TactileFaultConfig(jitter=True),
        TactileFaultConfig(spike_n=2.0, spike_frames=1),
        TactileFaultConfig(spike_n=2.0, spike_frames=2),
    ],
)
def test_multirate_fault_injection_obeys_sampling_and_freeze(tmp_path, fault) -> None:
    """检查真实丢帧／抖动与冻结，不把抓取成功当作故障逻辑的充分证据。"""
    resolved = compose_research_run(experiment="dm_gripper/unified_step_load_multirate")
    task = resolved.task.model_copy(update={"tactile_fault": fault})
    trace, raw = tmp_path / "trace.csv", tmp_path / "tactile.jsonl"
    result = run_force_scheduling(resolved.profile, task=task, output_csv=trace, output_tactile=raw)
    assert result.simulation_stable
    samples = [json.loads(line) for line in raw.read_text().splitlines()]
    assert sum(s.get("injected_drop", False) for s in samples) == fault.drop_frames
    assert sum(s.get("injected_spike", False) for s in samples) == fault.spike_frames
    with trace.open() as handle:
        rows = list(csv.DictReader(handle))
    assert np.diff([float(r["control_time_s"]) for r in rows]) == pytest.approx(0.004)
    if fault.jitter:
        intervals = np.diff([s["sensor_time_s"] for s in samples])
        assert min(intervals) == pytest.approx(0.001)
        assert max(intervals) == pytest.approx(0.003)
    if fault.drop_frames:
        stale = [r for r in rows if r["sensor_stale"] == "True"]
        assert stale
        assert all(float(r["scheduled_target_force_rate_n_s"]) == 0 for r in stale)
        assert int(rows[-1]["sensor_dropped_samples"]) == fault.drop_frames
    for before, after in zip(rows, rows[1:]):
        increment = float(after["scheduled_target_force_n"]) - float(
            before["scheduled_target_force_n"]
        )
        assert -1e-12 <= increment <= 0.2 + 1e-9
        if before["sensor_sequence_id"] == after["sensor_sequence_id"]:
            assert increment == pytest.approx(0)


def test_unified_multirate_preserves_transient_admittance_and_filter_overrides(tmp_path) -> None:
    """多速率组合保留瞬态导纳、前馈和载荷滤波覆盖。"""
    baseline = compose_research_run(experiment="dm_gripper/unified_step_load")
    candidate = compose_research_run(experiment="dm_gripper/unified_step_load_multirate")
    config = baseline.task.unified_adaptive
    expected_task = baseline.task.model_copy(
        update={
            "tactile_sampling": TactileSamplingConfig(
                period_s=0.001,
                median_window=3,
                stale_after_s=0.01,
                record_raw=True,
            ),
            "unified_adaptive": replace(config, load=replace(config.load, filter_tau_s=0.01)),
        }
    )
    assert candidate.task == expected_task
    expected = baseline.profile.model_dump(mode="json")
    admittance = expected["control"]["force"]["admittance"]
    assert admittance["velocity_limit_rad_s"] == 0.05
    assert admittance["feedforward_ratio"] == 0.2
    admittance.update(velocity_limit_rad_s=0.2, feedforward_ratio=1.0)
    assert candidate.profile.model_dump(mode="json") == expected
    assert baseline.task.unified_adaptive.load.filter_tau_s == 0.05
    assert candidate.task.unified_adaptive.load.filter_tau_s == 0.01
    trace = tmp_path / "trace.csv"
    result = run_force_scheduling(candidate.profile, task=candidate.task, output_csv=trace)
    assert result.simulation_stable and result.force_tracking_passed
    assert result.failure_reason is None
    assert result.passed
    with trace.open() as handle:
        rows = [r for r in csv.DictReader(handle) if r["phase"] == "schedule_load"]
    assert max(abs(float(r["tangential_displacement_m"])) for r in rows) < 0.002
    assert any(
        float(r["scenario_time_s"]) <= 0.1 and float(r["filtered_normal_force_n"]) >= 2.0
        for r in rows
    )
    assert all(r["adaptive_increase_count"] == "0" for r in rows)


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


def test_tactile_sampling_task_rejects_unsupported_combinations() -> None:
    """独立采样仅接入统一自适应模式，且采样周期不得慢于控制。"""
    base = ForceSchedulingTask.load(GRAVITY_TASK).model_dump()
    with pytest.raises(ValidationError, match="独立触觉采样当前仅接入统一自适应仿真"):
        ForceSchedulingTask.model_validate({**base, "tactile_sampling": {}})
    ForceSchedulingTask.model_validate(
        {**base, "unified_adaptive": {"risk_enabled": True}, "tactile_sampling": {}}
    )
    with pytest.raises(ValidationError, match="触觉采样周期不得大于控制周期"):
        ForceSchedulingTask.model_validate(
            {
                **base,
                "unified_adaptive": {},
                "control_period_s": 0.002,
                "tactile_sampling": {"period_s": 0.004},
            }
        )


def test_multirate_rejects_sampling_period_not_aligned_to_physics() -> None:
    """与物理步长错相的采样周期在进入仿真循环前直接拒绝。"""
    resolved = compose_research_run(experiment="dm_gripper/unified_step_load_multirate")
    misaligned = resolved.task.model_copy(
        update={"tactile_sampling": TactileSamplingConfig(period_s=0.0015)}
    )

    with pytest.raises(ValueError, match="整数倍"):
        run_force_scheduling(resolved.profile, task=misaligned)


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


@pytest.mark.parametrize("mode", ["oracle", "adaptive_prior", "unified_adaptive", "multirate"])
def test_execute_force_scheduling_writes_reproducible_artifacts(
    tmp_path: Path, fast_png_render: None, mode: str
) -> None:
    """执行器保存任务快照、有效参数、轨迹、图表和指标。"""
    task = ForceSchedulingTask.load(GRAVITY_TASK)
    profile = PROFILE
    if mode == "adaptive_prior":
        task = task.model_copy(update={"adaptive_prior": AdaptivePriorTaskConfig()})
    elif mode in {"unified_adaptive", "multirate"}:
        experiment = "unified_step_load_multirate" if mode == "multirate" else "unified_adaptive"
        resolved = compose_research_run(experiment=f"dm_gripper/{experiment}")
        profile = resolved.profile
        task = (
            resolved.task
            if mode == "multirate"
            else resolved.task.model_copy(update={"downward_load": task.downward_load})
        )

    run, result = execute_force_scheduling(
        profile=PROFILE,
        resolved_profile=profile if mode in {"unified_adaptive", "multirate"} else None,
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
    if mode == "multirate":
        expected.add("tactile.jsonl")
    assert expected == {path.name for path in run.path.iterdir()}
    metrics = json.loads((run.path / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["slip_passed"] is True
    effective = json.loads((run.path / "effective_parameters.json").read_text(encoding="utf-8"))
    assert effective["runtime"]["scheduler_kind"] == (
        "unified_adaptive" if mode == "multirate" else mode
    )
    if mode in {"unified_adaptive", "multirate"}:
        assert metrics["failure_reason"] is None
        assert effective["task"]["unified_adaptive"]["risk_enabled"] is False
    if mode == "multirate":
        samples = [
            json.loads(line) for line in (run.path / "tactile.jsonl").read_text().splitlines()
        ]
        with (run.path / "trace.csv").open() as handle:
            controls = list(csv.DictReader(handle))
        assert "control_updated" not in controls[0]
        assert np.diff([s["sensor_sequence_id"] for s in samples]) == pytest.approx(1)
        assert np.diff([s["sensor_time_s"] for s in samples]) == pytest.approx(0.001)
        assert np.diff([float(r["control_time_s"]) for r in controls]) == pytest.approx(0.004)
        assert np.diff([int(r["sensor_sequence_id"]) for r in controls]) == pytest.approx(4)
        assert all(float(r["sensor_age_s"]) == 0 for r in controls)
        assert all(r["adaptive_increase_count"] == "0" for r in controls)
        assert max(abs(float(r["tangential_displacement_m"])) for r in controls) < 0.002


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

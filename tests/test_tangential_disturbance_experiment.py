"""用真实物理场景验收 MIT 触觉增力、失败对照和运行产物。"""

import csv
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from parallel_gripper_tactile.experiments.tangential_disturbance import run_tangential_disturbance
from parallel_gripper_tactile.research.composition import compose_research_run
from parallel_gripper_tactile.runners.tangential_disturbance import execute_tangential_disturbance


def _resolve(*overrides):
    """统一使用已冻结的组合与真实场景。"""
    return compose_research_run(experiment="dm_gripper/tangential_disturbance", overrides=overrides)


@pytest.mark.parametrize("kind", ["ramp", "step", "pulse"])
def test_dynamic_policy_recovers_from_disturbance(tmp_path: Path, kind: str):
    """相同初始抓力下的三种扰动均需满足滑移上限和最终稳定。"""
    resolved = _resolve(f"task=tangential_disturbance/{kind}", "seed=1")
    trace = tmp_path / "trace.csv"
    result = run_tangential_disturbance(
        resolved.profile, task=resolved.task, policy_config=resolved.scheduler, output_csv=trace
    )
    assert result.passed
    assert result.initial_hold_passed and result.completed
    assert result.final_target_force_n > resolved.scheduler.initial_force_n + 0.5
    assert result.max_tangential_displacement_m < resolved.task.metrics.slip_threshold_m
    rows = list(csv.DictReader(trace.open()))
    for row in rows:
        assert float(row["measurement_time_s"]) <= float(row["time_s"])
        if row["phase"] == "disturbance":
            assert float(row["disturbance_time_s"]) == pytest.approx(
                float(row["time_s"]) - result.disturbance_start_time_s
            )
    targets = np.asarray([float(r["target_force_n"]) for r in rows])
    assert np.min(np.diff(targets)) >= -1e-12
    assert (
        max(float(r["target_force_rate_n_s"]) for r in rows)
        <= resolved.scheduler.max_force_rate_n_s + 1e-9
    )
    assert (
        max(abs(float(r["motor_torque_n_m"])) for r in rows) <= resolved.profile.control.mit.t_max
    )
    if kind == "pulse":
        after = [
            r
            for r in rows
            if float(r["disturbance_time_s"]) > resolved.task.disturbance.final_change_time_s + 0.2
        ]
        assert after and all(float(r["applied_tangential_force_n"]) == 0 for r in after)
        assert float(after[-1]["target_force_n"]) >= float(after[0]["target_force_n"])


def test_constant_force_fails_same_step_without_hiding_slip():
    """同样求解器下，固定抓力不足时必须保留科学失败。"""
    resolved = _resolve(
        "task=tangential_disturbance/step", "scheduler.definition.strategy=constant"
    )
    result = run_tangential_disturbance(
        resolved.profile, task=resolved.task, policy_config=resolved.scheduler
    )
    assert result.simulation_stable and result.initial_hold_passed and result.completed
    assert not result.passed and not result.slip_passed
    assert result.max_tangential_displacement_m > resolved.task.metrics.slip_threshold_m
    assert result.final_target_force_n == resolved.scheduler.initial_force_n
    assert result.increase_count == 0


def test_zero_disturbance_runner_preserves_artifacts_without_force_ratcheting(tmp_path: Path):
    """真实 runner 的负例应保持初始力，并生成可读输入、轨迹与双格式图。"""
    resolved = _resolve("task.definition.disturbance.amplitude_n=0")
    run, result = execute_tangential_disturbance(
        profile=resolved.profile_source,
        resolved_profile=resolved.profile,
        task_path=resolved.task_source,
        disturbance_task=resolved.task,
        policy_config=resolved.scheduler,
        policy_source=resolved.scheduler_source,
        output_root=tmp_path,
    )
    assert result.passed and result.increase_count == 0
    assert result.final_target_force_n == resolved.scheduler.initial_force_n
    metrics = json.loads((run.path / "metrics.json").read_text())
    assert metrics["passed"] is True
    assert (run.path / "plot.pdf").read_bytes().startswith(b"%PDF")
    assert (run.path / "plot.png").read_bytes().startswith(b"\x89PNG")
    snapshot = json.loads((run.path / "effective_parameters.json").read_text())
    assert snapshot["task"]["disturbance"]["amplitude_n"] == 0


def test_insufficient_initial_grip_cannot_be_scored_as_recovered():
    """初始保持已失败时，不启动扰动，也不伪造恢复时间。"""
    resolved = _resolve("task.definition.cube_mass_kg=1.0")
    result = run_tangential_disturbance(
        resolved.profile, task=resolved.task, policy_config=resolved.scheduler
    )
    assert not result.passed and not result.initial_hold_passed
    assert result.disturbance_start_time_s is None
    assert result.recovery_time_s is None


def test_transient_tracking_loss_during_initial_hold_is_not_forgotten(monkeypatch):
    """保持中短暂丢失跟踪，即使末刻恢复也不能通过稳定预载门。"""
    from parallel_gripper_tactile.experiments import tangential_disturbance as experiment

    original = experiment.NormalForceController.step
    injected = []

    def transient_state(self, data, *, observation, reference):
        """只注入一次状态失效，随后恢复真实控制器输出。"""
        command = original(self, data, observation=observation, reference=reference)
        if not injected and 2.0 <= observation.time_s < 2.01:
            injected.append(observation.time_s)
            return replace(command, state="approach")
        return command

    monkeypatch.setattr(experiment.NormalForceController, "step", transient_state)
    resolved = _resolve()
    result = run_tangential_disturbance(
        resolved.profile, task=resolved.task, policy_config=resolved.scheduler
    )
    assert injected
    assert result.simulation_stable and not result.initial_hold_passed
    assert result.disturbance_start_time_s is None and not result.passed

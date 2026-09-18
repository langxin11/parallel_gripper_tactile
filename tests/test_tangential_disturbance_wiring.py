"""验证切向扰动单次实验的组合、预检与 runner 工件接线。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from parallel_gripper_tactile.experiments.tangential_disturbance import (
    TangentialDisturbanceResult,
    TangentialDisturbanceTask,
)
from parallel_gripper_tactile.research import (
    ResearchConfigurationError,
    compose_research_run,
)
from parallel_gripper_tactile.runners import tangential_disturbance as disturbance_runner


def _resolved(*overrides: str):
    """组合切向扰动默认实验，并附加调用方给出的覆盖项。"""
    return compose_research_run(
        experiment="dm_gripper/tangential_disturbance",
        overrides=("execution=plan", *overrides),
    )


def _result() -> TangentialDisturbanceResult:
    """构造完整且通过的轻量结果，避免在 runner 接线测试中推进仿真。"""
    return TangentialDisturbanceResult(
        simulation_stable=True,
        completed=True,
        initial_hold_passed=True,
        slip_passed=True,
        contact_time_s=0.4,
        disturbance_start_time_s=0.8,
        max_tangential_displacement_m=0.001,
        peak_actual_force_n=1.2,
        final_actual_force_n=1.0,
        final_target_force_n=1.0,
        recovery_time_s=0.1,
        increase_count=2,
    )


def test_composed_disturbance_uses_selected_material_and_supported_controller() -> None:
    """实验组合把 material 选择写入实际任务，并保留 PID-only 兼容控制器。"""
    resolved = _resolved("controller=dm_gripper/pid_only", "estimator=none", "material=soft")

    assert isinstance(resolved.task, TangentialDisturbanceTask)
    assert resolved.selection.experiment.kind == "tangential_disturbance"
    assert resolved.selection.controller.name == "pid-only"
    assert resolved.task.object_material == "soft"


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        (("controller=dm_gripper/admittance_unified", "estimator=none"), "tangential disturbance"),
        (("execution.viewer=true",), "does not support viewer"),
        (("execution.multiccd_enabled=false",), "multiccd_enabled=true"),
        (("execution.trace_sample_period_s=0.01",), "every physics step"),
    ],
)
def test_composed_disturbance_rejects_unsupported_options(
    overrides: tuple[str, ...], message: str
) -> None:
    """导纳和 viewer 均在计划阶段被明确拒绝。"""
    with pytest.raises(ResearchConfigurationError, match=message):
        _resolved(*overrides)


def test_runner_snapshots_effective_task_and_registers_existing_artifacts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """runner 保存组合后的任务快照，且只登记实际由实验写出的 PNG／PDF／CSV。"""
    resolved = _resolved("material=medium")
    assert isinstance(resolved.task, TangentialDisturbanceTask)
    seen: dict[str, object] = {}

    def fake_run(profile: object, *, task: object, output_csv: Path, output_plot: Path):
        seen["profile"] = profile
        seen["task"] = task
        output_csv.write_text("time_s\n0.0\n", encoding="utf-8")
        output_plot.write_bytes(b"png")
        output_plot.with_suffix(".pdf").write_bytes(b"pdf")
        return _result()

    monkeypatch.setattr(disturbance_runner, "run_tangential_disturbance", fake_run)
    run, result = disturbance_runner.execute_tangential_disturbance(
        profile=resolved.profile_source,
        resolved_profile=resolved.profile,
        task_path=resolved.task_source,
        disturbance_task=resolved.task,
        output_root=tmp_path,
        run_name="wired",
    )

    assert result.passed
    assert seen == {"profile": resolved.profile, "task": resolved.task}
    assert json.loads((run.path / "metrics.json").read_text(encoding="utf-8"))["passed"] is True
    effective = json.loads((run.path / "effective_parameters.json").read_text(encoding="utf-8"))
    assert effective["task"] == resolved.task.model_dump(mode="json")
    manifest = json.loads((run.path / "manifest.json").read_text(encoding="utf-8"))
    assert {"trace.csv", "plot.png", "plot.pdf", "metrics.json"} <= set(manifest["artifacts"])

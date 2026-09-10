"""验证科研计划与共享 runner 使用同一最终配置。"""

from __future__ import annotations

import json
from pathlib import Path

from hydra import compose, initialize_config_dir
import pytest

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingResult
from parallel_gripper_tactile.research import (
    REPOSITORY_ROOT,
    execute_research_run,
    resolve_research_run,
)
from parallel_gripper_tactile.research.hydra_support import register_resolvers, resolved_mapping
from parallel_gripper_tactile.research import execution as research_execution
from parallel_gripper_tactile.runners import force_tracking as force_tracking_runner


CONFIG_ROOT = REPOSITORY_ROOT / "configs" / "research"


def _resolved(*overrides: str):
    """组合并解析真实科研配置。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        config = compose(config_name="dm_force_track", overrides=list(overrides))
    return resolve_research_run(resolved_mapping(config))


def _result() -> ForceTrackingResult:
    """返回无需仿真的最小成功结果。"""
    return ForceTrackingResult(
        contact_time_s=1.0,
        tracking_start_time_s=1.1,
        tracking_duration_s=2.0,
        rmse_n=0.1,
        mae_n=0.1,
        peak_abs_error_n=0.2,
        mean_error_n=0.0,
        final_error_n=0.0,
        torque_saturation_ratio=0.0,
        position_saturation_ratio=0.0,
        mean_estimated_stiffness_n_per_m=1000.0,
        rise_time_s=None,
        overshoot_ratio=None,
        settling_time_s=None,
        simulation_stable=True,
    )


def test_plan_writes_validated_artifacts_without_calling_runner(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """计划模式保存最终配置与条件，但不登记已执行实验。"""
    resolved = _resolved("execution=plan")

    def forbidden(**kwargs: object):
        raise AssertionError("plan mode must not call the experiment runner")

    monkeypatch.setattr(research_execution, "execute_force_tracking", forbidden)
    outcome = execute_research_run(
        resolved,
        hydra_output_directory=tmp_path,
        provenance={"choices": {}, "overrides": []},
    )

    plan = json.loads((tmp_path / "plan.json").read_text(encoding="utf-8"))
    assert outcome.mode == "plan"
    assert outcome.run_directory is None
    assert plan["artifact_kind"] == "plan"
    assert plan["validated"] is True
    assert plan["condition_count"] == 1
    assert not (tmp_path / "execution.json").exists()
    assert not (tmp_path / "artifacts").exists()


def test_execution_passes_the_same_final_profile_to_shared_runner(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """执行适配器传递已保存的同一个最终 profile 与 task 对象。"""
    resolved = _resolved()
    captured: dict[str, object] = {}

    class FakeRun:
        path = tmp_path / "artifacts/run"

    def fake_execute(**kwargs: object):
        captured.update(kwargs)
        return FakeRun(), _result()

    monkeypatch.setattr(research_execution, "execute_force_tracking", fake_execute)
    outcome = execute_research_run(
        resolved,
        hydra_output_directory=tmp_path,
        provenance={"choices": {}, "overrides": []},
    )

    effective = json.loads((tmp_path / "effective_configuration.json").read_text(encoding="utf-8"))
    assert captured["resolved_profile"] is resolved.profile
    assert captured["tracking_task"] is resolved.task
    assert effective["profile"] == resolved.profile.model_dump(mode="json")
    assert outcome.passed is True


def test_runner_does_not_reload_profile_when_final_profile_is_supplied(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """兼容 runner 收到最终 profile 后不再读取或覆盖磁盘源。"""
    resolved = _resolved()
    seen: dict[str, object] = {}

    def forbidden_load(path: Path):
        raise AssertionError(f"unexpected reload: {path}")

    def fake_run(profile: object, *, output_parquet: Path, output_plot: Path, **kwargs: object):
        seen["profile"] = profile
        output_parquet.write_bytes(b"parquet")
        output_plot.write_bytes(b"plot")
        output_plot.with_suffix(".pdf").write_bytes(b"pdf")
        return _result()

    monkeypatch.setattr(force_tracking_runner, "load_profile", forbidden_load)
    monkeypatch.setattr(force_tracking_runner, "run_force_tracking", fake_run)
    _, result = force_tracking_runner.execute_force_tracking(
        profile=resolved.profile_source,
        resolved_profile=resolved.profile,
        task_path=resolved.task_source,
        tracking_task=resolved.task,
        output_root=tmp_path,
        run_name="resolved",
        object_material=resolved.selection.material.name,
        controller_variant=resolved.selection.controller.name,
        stiffness_estimator_method=resolved.selection.estimator.name,
        sensor_noise_seed=resolved.selection.seed,
        torque_adrc_override=resolved.selection.controller.torque_adrc,
        trace_sample_period_s=resolved.selection.execution.trace_sample_period_s,
    )

    assert result.passed is True
    assert seen["profile"] == resolved.profile

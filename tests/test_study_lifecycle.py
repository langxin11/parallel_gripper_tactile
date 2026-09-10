"""验证正式研究公共生命周期、失败分类、哈希和恢复边界。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.lifecycle import (
    ConditionExecution,
    StudyCondition,
    StudyLifecycleError,
    StudyPlan,
    StudyPostprocessResult,
    assess_recovery,
    execute_study_lifecycle,
    record_study_setup_failure,
    require_matching_study_plan,
    scientific_configuration_hash,
    write_planned_study_manifest,
)


def _plan(count: int = 3, *, digest: str = "a" * 64) -> StudyPlan:
    """构造小型、稳定且覆盖配对字段的测试计划。"""
    return StudyPlan(
        study_kind="synthetic",
        stage="coarse",
        study_definition_sha256="b" * 64,
        scientific_configuration_sha256=digest,
        conditions=tuple(
            StudyCondition(
                condition_id=f"condition-{index}",
                parameters={"index": index, "seed": index},
                pair_key=f"pair-{index}",
                baseline_role="baseline" if index == 0 else None,
            )
            for index in range(count)
        ),
        seeds=tuple(range(count)),
        preflight={"status": "passed", "checks": ["synthetic"]},
    )


def _postprocess(
    rows: list[dict[str, object]], outcomes: tuple[object, ...], directory: Path
) -> StudyPostprocessResult:
    """写出可登记的测试聚合产物。"""
    del outcomes
    summary = directory / "summary.json"
    summary.write_text(json.dumps(rows), encoding="utf-8")
    return StudyPostprocessResult((summary,), {"protocol_field": "kept"}, rows)


def _render(rows: list[dict[str, object]], payload: object, directory: Path) -> tuple[Path, ...]:
    """写出可登记的测试绘图产物。"""
    del rows, payload
    figure = directory / "figure.txt"
    figure.write_text("figure", encoding="utf-8")
    return (figure,)


def test_plan_manifest_contains_no_executed_runs(tmp_path: Path) -> None:
    """计划状态登记完整条件和产物，但不伪造任何执行结果。"""
    artifact = tmp_path / "plan.json"
    artifact.write_text("{}\n", encoding="utf-8")
    path = write_planned_study_manifest(
        _plan(),
        study_directory=tmp_path,
        artifacts=(artifact,),
    )

    manifest = json.loads(path.read_text(encoding="utf-8"))
    assert manifest["state"] == "planned"
    assert manifest["planned_condition_count"] == 3
    assert manifest["completed_condition_count"] == 0
    assert manifest["condition_results"] == []
    assert manifest["runs"] == []
    assert manifest["artifact_sha256"]["plan.json"]
    assert not (tmp_path / "runs").exists()


def test_mixed_condition_results_are_classified_without_stopping(tmp_path: Path) -> None:
    """科学失败与 Python 异常分列，异常后继续执行后续条件。"""
    executed: list[str] = []
    observed_progress: list[int] = []

    def execute(condition: StudyCondition) -> ConditionExecution:
        running = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
        assert running["state"] == "running"
        observed_progress.append(len(running["condition_results"]))
        executed.append(condition.condition_id)
        index = int(condition.parameters["index"])
        if index == 2:
            raise RuntimeError("synthetic execution error")
        run_directory = f"runs/{condition.condition_id}"
        return ConditionExecution(
            row={"index": index, "passed": index == 0, "run_directory": run_directory},
            run_directory=run_directory,
            passed=index == 0,
        )

    execute_study_lifecycle(
        _plan(),
        study_directory=tmp_path,
        execute_condition=execute,
        aggregate_and_persist=_postprocess,
        render=_render,
    )

    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    assert executed == ["condition-0", "condition-1", "condition-2"]
    assert observed_progress == [0, 1, 2]
    assert manifest["state"] == "partial"
    assert manifest["completed_condition_count"] == 2
    assert manifest["passed_condition_count"] == 1
    assert manifest["scientific_failure_count"] == 1
    assert manifest["execution_error_count"] == 1
    assert manifest["runs"] == ["runs/condition-0", "runs/condition-1"]
    assert manifest["failed_runs"] == ["runs/condition-1"]
    assert manifest["failed_conditions"][0]["condition_id"] == "condition-2"
    assert [item["status"] for item in manifest["condition_results"]] == [
        "completed",
        "scientific_failure",
        "execution_error",
    ]
    assert set(manifest["artifact_sha256"]) == {
        "condition_results.json",
        "failed_conditions.json",
        "summary.json",
        "figure.txt",
    }


@pytest.mark.parametrize(
    ("mode", "expected_state", "completed", "scientific_failures", "execution_errors"),
    [
        ("all_passed", "completed", 2, 0, 0),
        ("all_scientific_failure", "completed", 2, 2, 0),
        ("all_execution_error", "failed", 0, 0, 2),
    ],
)
def test_zero_partial_and_complete_success_states(
    tmp_path: Path,
    mode: str,
    expected_state: str,
    completed: int,
    scientific_failures: int,
    execution_errors: int,
) -> None:
    """零正常 run、零科学通过和全部通过具有不同且稳定的语义。"""

    def execute(condition: StudyCondition) -> ConditionExecution:
        if mode == "all_execution_error":
            raise RuntimeError("no run")
        passed = mode == "all_passed"
        return ConditionExecution(
            row={"passed": passed},
            run_directory=f"runs/{condition.condition_id}",
            passed=passed,
        )

    execute_study_lifecycle(
        _plan(2),
        study_directory=tmp_path,
        execute_condition=execute,
        aggregate_and_persist=_postprocess,
        render=_render,
    )
    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == expected_state
    assert manifest["completed_condition_count"] == completed
    assert manifest["scientific_failure_count"] == scientific_failures
    assert manifest["execution_error_count"] == execution_errors


@pytest.mark.parametrize("failure_stage", ["aggregation", "rendering"])
def test_postprocess_failure_is_persisted_with_its_stage(
    tmp_path: Path, failure_stage: str
) -> None:
    """聚合和绘图异常保留已完成条件，并明确归属失败阶段。"""

    def execute(condition: StudyCondition) -> ConditionExecution:
        return ConditionExecution(
            row={"passed": True},
            run_directory=f"runs/{condition.condition_id}",
            passed=True,
        )

    def fail_aggregation(*args: object) -> StudyPostprocessResult:
        del args
        raise ArithmeticError("aggregate failed")

    def fail_render(*args: object) -> tuple[Path, ...]:
        del args
        raise OSError("render failed")

    with pytest.raises(StudyLifecycleError, match=failure_stage):
        execute_study_lifecycle(
            _plan(1),
            study_directory=tmp_path,
            execute_condition=execute,
            aggregate_and_persist=(
                fail_aggregation if failure_stage == "aggregation" else _postprocess
            ),
            render=fail_render if failure_stage == "rendering" else _render,
        )
    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "failed"
    assert manifest["completed_condition_count"] == 1
    assert manifest["lifecycle_failures"][0]["stage"] == failure_stage
    assert "lifecycle_failure.json" in manifest["artifacts"]


@pytest.mark.parametrize("hook", ["aggregation", "rendering"])
def test_artifact_outside_study_is_rejected_by_owning_hook(tmp_path: Path, hook: str) -> None:
    """聚合或绘图钩子不能把 study 目录外的文件登记为自身产物。"""
    outside = tmp_path.parent / f"outside-{hook}.txt"
    outside.write_text("outside", encoding="utf-8")

    def execute(condition: StudyCondition) -> ConditionExecution:
        return ConditionExecution(
            row={"passed": True},
            run_directory=f"runs/{condition.condition_id}",
            passed=True,
        )

    def aggregate(*args: object) -> StudyPostprocessResult:
        del args
        artifacts = (outside,) if hook == "aggregation" else ()
        return StudyPostprocessResult(artifacts, {}, None)

    def render(*args: object) -> tuple[Path, ...]:
        del args
        return (outside,) if hook == "rendering" else ()

    with pytest.raises(StudyLifecycleError, match=hook):
        execute_study_lifecycle(
            _plan(1),
            study_directory=tmp_path,
            execute_condition=execute,
            aggregate_and_persist=aggregate,
            render=render,
        )


def test_setup_failure_has_explicit_stage_and_no_fake_conditions(tmp_path: Path) -> None:
    """形成计划前的配置失败仍有结构化失败 manifest。"""
    provenance = tmp_path / "composition_provenance.json"
    provenance.write_text("{}\n", encoding="utf-8")
    record_study_setup_failure(
        study_directory=tmp_path,
        stage="configuration",
        error=ValueError("invalid study"),
        study_kind="synthetic",
        artifacts=(provenance,),
    )

    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "failed"
    assert manifest["planned_condition_count"] == 0
    assert manifest["condition_results"] == []
    assert manifest["lifecycle_failures"] == [
        {
            "stage": "configuration",
            "error_type": "ValueError",
            "message": "invalid study",
        }
    ]
    assert set(manifest["artifact_sha256"]) == {
        "composition_provenance.json",
        "setup_failure.json",
    }


def test_scientific_hash_is_canonical_and_repository_relative(tmp_path: Path) -> None:
    """字典顺序和仓库绝对路径不影响哈希，科学值变化会影响哈希。"""
    resource = tmp_path / "configs" / "task.yaml"
    resource.parent.mkdir()
    resource.write_text("task", encoding="utf-8")
    first = scientific_configuration_hash(
        {"gain": 1.0, "resource": resource}, repository_root=tmp_path
    )
    reordered = scientific_configuration_hash(
        {"resource": Path("configs/task.yaml"), "gain": 1.0},
        repository_root=tmp_path,
    )
    changed = scientific_configuration_hash(
        {"resource": resource, "gain": 1.1}, repository_root=tmp_path
    )
    assert first == reordered
    assert first != changed


def test_recovery_report_accepts_only_completed_matching_conditions(tmp_path: Path) -> None:
    """恢复检测不把科学失败或执行异常当成成功，也不自动续跑。"""
    plan = _plan()
    manifest = {
        "study_kind": plan.study_kind,
        "stage": plan.stage,
        "scientific_configuration_sha256": plan.scientific_configuration_sha256,
        "condition_results": [
            {"condition_id": "condition-0", "status": "completed"},
            {"condition_id": "condition-1", "status": "scientific_failure"},
            {"condition_id": "condition-2", "status": "execution_error"},
        ],
    }
    (tmp_path / "study_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    assessment = assess_recovery(tmp_path, plan)
    assert assessment.compatible is True
    assert assessment.automatic_resume_enabled is False
    assert assessment.successful_condition_ids == ("condition-0",)
    assert assessment.retryable_condition_ids == ("condition-1", "condition-2")

    incompatible = assess_recovery(tmp_path, _plan(digest="c" * 64))
    assert incompatible.compatible is False
    assert "scientific_configuration_sha256 不匹配" in incompatible.reasons


def test_protocol_plan_guard_rejects_cross_configuration_execution() -> None:
    """执行协议不能消费其他哈希或条件集合的计划。"""
    expected = _plan()
    supplied = _plan(digest="c" * 64)
    with pytest.raises(ValueError, match="scientific_configuration_sha256"):
        require_matching_study_plan(expected, supplied)

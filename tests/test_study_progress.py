"""验证进度只观察父进程已落盘状态，不改变研究分类和执行。"""

from __future__ import annotations

from dataclasses import FrozenInstanceError
import json
import os
from pathlib import Path
import warnings

import pytest

from parallel_gripper_tactile.research.progress import report_study_progress
from parallel_gripper_tactile.studies.lifecycle import (
    ConditionExecution,
    StudyCondition,
    StudyLifecycleError,
    StudyPlan,
    StudyPostprocessResult,
    StudyProgress,
    execute_study_lifecycle,
)


def _plan(count: int = 3) -> StudyPlan:
    """构造只含结果类别的小型计划。"""
    return StudyPlan(
        study_kind="progress_test",
        study_definition_sha256="a" * 64,
        scientific_configuration_sha256="b" * 64,
        conditions=tuple(
            StudyCondition(condition_id=f"condition-{index}", parameters={"index": index})
            for index in range(count)
        ),
        seeds=(),
        preflight={},
    )


def _execute(condition: StudyCondition) -> ConditionExecution:
    """生成正常、科学失败与执行异常，供 spawn 直接序列化。"""
    index = condition.parameters["index"]
    if index == 2:
        raise RuntimeError("条件执行失败")
    return ConditionExecution(
        row={"index": index, "pid": os.getpid()},
        run_directory=f"runs/{condition.condition_id}",
        passed=index == 0,
    )


def _aggregate(*args: object) -> StudyPostprocessResult:
    """仅返回空产物，避免引入真实仿真与绘图成本。"""
    return StudyPostprocessResult(artifacts=(), manifest_fields={})


def _render(*args: object) -> tuple[Path, ...]:
    """测试观察边界，不绘制真实图表。"""
    return ()


@pytest.mark.parametrize("workers", [1, 2])
def test_progress_observes_persisted_parent_state_and_failure_categories(
    tmp_path: Path, workers: int
) -> None:
    """串行与 spawn 均在父进程通知，且通知计数与已落盘账本逐次一致。"""
    parent_pid = os.getpid()
    updates: list[StudyProgress] = []

    def observe(progress: StudyProgress) -> None:
        """局部闭包不能发送至 worker，直接核对父进程和磁盘结果。"""
        assert os.getpid() == parent_pid
        manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
        assert progress.finished == len(manifest["condition_results"])
        assert progress.scientific_failures == manifest["scientific_failure_count"]
        assert progress.execution_errors == manifest["execution_error_count"]
        assert progress.state == manifest["state"]
        updates.append(progress)

    execute_study_lifecycle(
        _plan(),
        study_directory=tmp_path,
        execute_condition=_execute,
        aggregate_and_persist=_aggregate,
        render=_render,
        workers=workers,
        on_progress=observe,
    )

    assert [item.finished for item in updates] == [0, 1, 2, 3, 3]
    assert [item.state for item in updates] == ["running"] * 4 + ["partial"]
    assert all(item.total == 3 and item.study_directory == tmp_path.resolve() for item in updates)
    assert updates[-1].scientific_failures == updates[-1].execution_errors == 1
    with pytest.raises(FrozenInstanceError):
        updates[-1].finished = 0
    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    rows = manifest["condition_results"]
    assert [row["condition_id"] for row in rows] == [f"condition-{index}" for index in range(3)]
    worker_pids = {row["metrics"]["pid"] for row in rows if row["metrics"] is not None}
    assert (parent_pid in worker_pids) == (workers == 1)


def test_observer_exception_warns_without_altering_results_even_with_warning_errors(
    tmp_path: Path,
) -> None:
    """观察者抛错明确告警，即便警告被配置成异常也继续完成研究。"""

    def broken_observer(progress: StudyProgress) -> None:
        """模拟进度展示故障。"""
        raise ValueError("展示故障")

    with warnings.catch_warnings(record=True) as recorded:
        warnings.simplefilter("error")
        result = execute_study_lifecycle(
            _plan(1),
            study_directory=tmp_path,
            execute_condition=_execute,
            aggregate_and_persist=_aggregate,
            render=_render,
            on_progress=broken_observer,
        )
    assert len(recorded) == 3
    assert all("研究进度回调失败：ValueError：展示故障" in str(item.message) for item in recorded)
    manifest = json.loads((result / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "completed"
    assert manifest["failed_condition_count"] == 0
    assert manifest["lifecycle_failures"] == []


@pytest.mark.parametrize("stage", ["aggregation", "rendering"])
def test_postprocessing_failure_notifies_final_persisted_state(tmp_path: Path, stage: str) -> None:
    """聚合和绘图异常也在失败账本落盘后给出最终通知。"""
    updates: list[StudyProgress] = []

    def fail(*args: object) -> object:
        """模拟后处理异常。"""
        raise RuntimeError("后处理失败")

    with pytest.raises(StudyLifecycleError) as caught:
        execute_study_lifecycle(
            _plan(1),
            study_directory=tmp_path,
            execute_condition=_execute,
            aggregate_and_persist=fail if stage == "aggregation" else _aggregate,
            render=fail if stage == "rendering" else _render,
            on_progress=updates.append,
        )
    assert caught.value.failure.stage == stage
    assert [(item.finished, item.state) for item in updates] == [
        (0, "running"),
        (1, "running"),
        (1, "failed"),
    ]
    assert updates[-1].scientific_failures == updates[-1].execution_errors == 0
    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == updates[-1].state
    assert manifest["lifecycle_failures"][0]["stage"] == stage


def test_text_progress_includes_counts_state_and_absolute_directory(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """终端进度不混淆两种失败，目录可直接定位产物。"""
    report_study_progress(StudyProgress(tmp_path, 10, 4, 1, 2, "running"))
    assert capsys.readouterr().out == (
        f"研究进度：已处理 4/10，科学失败 1，执行异常 2，状态 running，目录：{tmp_path.resolve()}\n"
    )


@pytest.mark.parametrize("mode", ["plan", "run"])
def test_study_entry_displays_plan_or_execution_progress(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, mode: str
) -> None:
    """入口区分计划摘要和执行进度，并保留兼容的末尾目录输出。"""
    from types import SimpleNamespace

    from hydra.types import RunMode
    from omegaconf import OmegaConf

    from scripts.research import study as entry

    resolved = SimpleNamespace(
        selection=SimpleNamespace(execution=SimpleNamespace(mode=mode)), conditions=("one", "two")
    )
    monkeypatch.setattr(
        entry.HydraConfig,
        "get",
        lambda: SimpleNamespace(
            runtime=SimpleNamespace(output_dir=str(tmp_path)), mode=RunMode.RUN
        ),
    )
    monkeypatch.setattr(entry, "composition_provenance", lambda: {})
    monkeypatch.setattr(entry, "resolve_research_study", lambda config: resolved)

    def execute(selected: object, *, on_progress: object, **kwargs: object) -> Path:
        """模拟公共入口的计划静默与运行通知契约。"""
        assert selected is resolved
        assert kwargs["hydra_output_directory"] == tmp_path
        assert on_progress is report_study_progress
        if mode == "run":
            on_progress(StudyProgress(tmp_path, 2, 0, 0, 0, "running"))
        return tmp_path

    monkeypatch.setattr(entry, "execute_research_study", execute)
    entry.main.__wrapped__(OmegaConf.create({}))
    output = capsys.readouterr().out
    assert output.endswith(f"Study {mode}: {tmp_path}\n")
    if mode == "plan":
        assert f"研究计划：条件数 2，目录：{tmp_path.resolve()}" in output
        assert "研究进度" not in output
    else:
        assert "研究进度：已处理 0/2" in output
        assert "研究计划" not in output

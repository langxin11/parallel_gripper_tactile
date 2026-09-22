"""验证各单次 runner 的异常留档与原始故障保留。"""

from __future__ import annotations

import json
from pathlib import Path
from types import ModuleType

import pytest

from parallel_gripper_tactile.artifacts import RunDirectory
from parallel_gripper_tactile.research import compose_research_run
from parallel_gripper_tactile.runners import (
    force_scheduling,
    force_tracking,
    friction_estimation,
    robotiq_discrete_force,
    tangential_disturbance,
)
from parallel_gripper_tactile.runners.common import run_artifact_lifecycle


@pytest.mark.parametrize("failure_stage", ["snapshot", "experiment"])
@pytest.mark.parametrize(
    ("experiment", "module", "task_argument", "scheduler_argument"),
    [
        ("dm_gripper/force_tracking", force_tracking, "tracking_task", None),
        (
            "dm_gripper/force_scheduling_oracle",
            force_scheduling,
            "scheduling_task",
            "scheduler_config",
        ),
        ("dm_gripper/friction_estimation", friction_estimation, "estimation_task", None),
        ("robotiq_2f85/discrete_force", robotiq_discrete_force, "discrete_task", None),
        (
            "dm_gripper/tangential_disturbance",
            tangential_disturbance,
            "disturbance_task",
            "policy_config",
        ),
    ],
)
def test_runner_failure_preserves_inputs_partial_artifacts_and_original_error(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    experiment: str,
    module: ModuleType,
    task_argument: str,
    scheduler_argument: str | None,
    failure_stage: str,
) -> None:
    """快照或实验失败均留档；未返回前产生的部分轨迹也登记，异常对象保持不变。"""
    resolved = compose_research_run(experiment=experiment, overrides=("execution=plan",))
    error = RuntimeError("注入的执行故障")
    partial_path: Path | None = None

    def fail(*args: object, **kwargs: object) -> None:
        """模拟输入写入失败，或实验写出部分轨迹后失败。"""
        nonlocal partial_path
        if failure_stage == "experiment":
            partial_path = kwargs.get("output_csv") or kwargs["output_parquet"]
            partial_path.write_bytes(b"partial trace\n")
        raise error

    name = module.__name__.rsplit(".", 1)[-1]
    monkeypatch.setattr(
        module, "write_task_snapshot" if failure_stage == "snapshot" else f"run_{name}", fail
    )
    kwargs = {
        "profile": resolved.profile_source,
        "resolved_profile": resolved.profile,
        "task_path": resolved.task_source,
        task_argument: resolved.task,
        "output_root": tmp_path,
        "run_name": "failure-test",
    }
    if scheduler_argument is not None:
        kwargs[scheduler_argument] = resolved.scheduler
    if module is force_tracking:
        kwargs["controller_variant"] = resolved.selection.controller.name
    with pytest.raises(RuntimeError) as caught:
        getattr(module, f"execute_{name}")(**kwargs)
    assert caught.value is error

    manifests = list(tmp_path.rglob("manifest.json"))
    assert len(manifests) == 1
    directory = manifests[0].parent
    manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
    assert json.loads((directory / "error.json").read_text(encoding="utf-8")) == {
        "error_type": "RuntimeError",
        "message": str(error),
    }
    artifacts = set(manifest["artifacts"])
    assert {"profile.yaml", "error.json"} <= artifacts
    assert "metrics.json" not in artifacts
    assert all((directory / name).is_file() for name in artifacts)
    if failure_stage == "experiment":
        assert {"task.yaml", "effective_parameters.json", partial_path.name} <= artifacts
        assert partial_path.read_bytes() == b"partial trace\n"


@pytest.mark.parametrize("recording_failure", ["error_file", "manifest"])
def test_failure_recording_error_does_not_replace_original_exception(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, recording_failure: str
) -> None:
    """磁盘或 manifest 留档失败只追加诊断，不掩盖控制或仿真的原始异常。"""
    run = RunDirectory.create(
        tmp_path,
        profile_name="test",
        experiment="test",
        profile_source="schema_version: 1\nname: test\n",
    )
    original_write = Path.write_text

    def fail_write(path: Path, *args: object, **kwargs: object) -> int:
        """仅令指定留档文件写入失败，其余 I/O 保持真实行为。"""
        if (recording_failure == "error_file" and path.name == "error.json") or (
            recording_failure == "manifest" and path.name.startswith(".manifest.json.")
        ):
            raise OSError("磁盘不可写")
        return original_write(path, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", fail_write)
    error = RuntimeError("原始执行故障")
    with pytest.raises(RuntimeError) as caught:
        with run_artifact_lifecycle(run):
            raise error
    assert caught.value is error
    assert any("OSError" in note and "磁盘不可写" in note for note in error.__notes__)

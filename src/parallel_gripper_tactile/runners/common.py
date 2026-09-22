"""单次实验的输入快照、产物写入与异常留档。"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
import json
from pathlib import Path

from pydantic import BaseModel
import yaml

from ..artifacts import RunDirectory
from ..config.profiles import GripperProfile


def profile_snapshot_source(
    profile: Path | str, configured: GripperProfile, *, composed: bool
) -> Path | str:
    """组合入口保存有效配置，直接入口保留原始 profile 来源。"""
    return (
        yaml.safe_dump(configured.model_dump(mode="json"), allow_unicode=True, sort_keys=True)
        if composed
        else profile
    )


def profile_source_label(profile: Path | str, *, composed: bool) -> str:
    """返回有效参数快照中沿用的 profile 来源标识。"""
    if composed:
        return "composed_profile"
    return str(profile.resolve()) if isinstance(profile, Path) else "serialized_profile"


def write_json_artifact(
    run: RunDirectory,
    name: str,
    payload: Mapping[str, object],
    *,
    ensure_ascii: bool = True,
) -> Path:
    """按既有缩进和排序规则写入 JSON，并登记成功写入的文件。"""
    path = run.artifact_path(name)
    path.write_text(
        json.dumps(payload, ensure_ascii=ensure_ascii, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    run.register_artifact(path)
    return path


def write_yaml_artifact(run: RunDirectory, name: str, payload: object) -> Path:
    """按既有 Unicode 和排序规则写入 YAML，并登记成功写入的文件。"""
    path = run.artifact_path(name)
    path.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=True), encoding="utf-8")
    run.register_artifact(path)
    return path


def write_task_snapshot(
    run: RunDirectory, task_path: Path, resolved_task: BaseModel | None
) -> Path:
    """冻结任务对象优先；直接入口逐字节保留原始任务文件。"""
    if resolved_task is not None:
        return write_yaml_artifact(run, "task.yaml", resolved_task.model_dump(mode="json"))
    path = run.artifact_path("task.yaml")
    path.write_bytes(task_path.read_bytes())
    run.register_artifact(path)
    return path


@contextmanager
def run_artifact_lifecycle(run: RunDirectory) -> Iterator[None]:
    """正常结束时定稿；执行异常时尽力保存已有产物并重新抛出原异常。

    科学失败由实验结果表达，不进入此异常分支。留档自身失败时给原异常
    添加诊断说明，避免磁盘或登记错误覆盖实验的首个故障。
    """
    try:
        yield
        run.finalize()
    except Exception as error:
        try:
            write_json_artifact(
                run,
                "error.json",
                {"error_type": type(error).__name__, "message": str(error)},
                ensure_ascii=False,
            )
            # 实验可能在返回前已经写出部分轨迹或图像；这些文件仍需可追溯。
            for path in sorted(run.path.rglob("*")):
                if path.is_file() and path.name != "manifest.json":
                    run.register_artifact(path)
            run.finalize()
        except Exception as recording_error:
            error.add_note(f"运行异常留档失败：{type(recording_error).__name__}：{recording_error}")
        raise

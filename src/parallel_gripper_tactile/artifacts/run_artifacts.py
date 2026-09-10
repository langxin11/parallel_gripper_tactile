"""用于可复现实验运行的结构化、磁盘安全工件。

该模块刻意不依赖 CLI 或 profile 加载器。实验可以用一个已校验的 profile
路径（或其已序列化的 YAML）来使用它，并在工件生成时进行注册。
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from datetime import UTC, datetime, timedelta
from hashlib import sha256
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
import json
import os
import re
import shutil
import subprocess
from typing import Any
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field


_COMPONENT_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_MANIFEST_NAME = "manifest.json"
_PROFILE_SNAPSHOT_NAME = "profile.yaml"


class ArtifactError(RuntimeError):
    """当工件会逃逸、覆盖或使一次运行失效时抛出。"""


class GitState(BaseModel):
    """在运行创建时捕获的、尽力而为的源代码控制信息。"""

    model_config = ConfigDict(extra="forbid", frozen=True)

    commit: str | None = None
    dirty: bool | None = None


class RunManifest(BaseModel):
    """对实验输入与输出的自描述记录。"""

    model_config = ConfigDict(extra="forbid", frozen=True)

    schema_version: int = Field(default=1, ge=1)
    created_at: datetime
    profile_name: str
    experiment: str
    command: tuple[str, ...] = ()
    parameters: dict[str, Any] = Field(default_factory=dict)
    source_profile_sha256: str
    git: GitState
    dependency_versions: dict[str, str] = Field(default_factory=dict)
    artifacts: tuple[str, ...] = ()


class RunInfo(BaseModel):
    """一个被发现的运行目录，在其有效时包含解析后的 manifest。"""

    model_config = ConfigDict(extra="forbid", frozen=True)

    path: Path
    manifest: RunManifest | None = None


class CleanPreview(BaseModel):
    """清理操作可能移除的、经过校验的精确运行目录。"""

    model_config = ConfigDict(extra="forbid", frozen=True)

    output_root: Path
    targets: tuple[Path, ...]

    @property
    def count(self) -> int:
        """返回被选中进行清理的目录数量。"""
        return len(self.targets)


def _validate_component(value: str, field_name: str) -> str:
    """返回一个安全的路径分量，拒绝类路径的用户输入。"""
    if not _COMPONENT_RE.fullmatch(value) or value in {".", ".."}:
        raise ValueError(f"{field_name} must be a simple path component")
    return value


def _resolved_root(output_root: str | Path, *, create: bool = False) -> Path:
    """解析显式提供的输出根目录，可选地创建它。"""
    root = Path(output_root).expanduser().resolve(strict=False)
    if create:
        root.mkdir(parents=True, exist_ok=True)
    if not root.is_dir():
        raise FileNotFoundError(f"output root is not a directory: {root}")
    return root.resolve()


def _require_descendant(root: Path, candidate: Path) -> Path:
    """解析 ``candidate`` 并确保它无法通过链接逃逸出 ``root``。"""
    resolved = candidate.resolve(strict=False)
    if resolved == root or root not in resolved.parents:
        raise ArtifactError(f"refusing to operate outside output root: {candidate}")
    return resolved


def _run_git(directory: Path, *args: str) -> str | None:
    """运行一个小的非交互式 git 查询，在仓库之外返回 ``None``。"""
    result = subprocess.run(
        ["git", "-C", str(directory), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode:
        return None
    return result.stdout.strip()


def collect_git_state(directory: str | Path) -> GitState:
    """在不要求目录是仓库的情况下捕获提交与脏状态。"""
    path = Path(directory).resolve()
    root = _run_git(path, "rev-parse", "--show-toplevel")
    if root is None:
        return GitState()
    commit = _run_git(Path(root), "rev-parse", "HEAD")
    porcelain = _run_git(Path(root), "status", "--porcelain")
    return GitState(commit=commit, dirty=bool(porcelain) if porcelain is not None else None)


def collect_dependency_versions(
    distributions: Iterable[str] = (
        "parallel-gripper-tactile",
        "mujoco",
        "numpy",
        "matplotlib",
        "pyarrow",
        "pydantic",
        "PyYAML",
        "typer",
        "rich",
        "hydra-core",
        "omegaconf",
    ),
) -> dict[str, str]:
    """返回项目直接运行时依赖的已安装版本。"""
    versions: dict[str, str] = {}
    for distribution in distributions:
        try:
            versions[distribution] = version(distribution)
        except PackageNotFoundError:
            continue
    return versions


def _profile_bytes(profile_source: str | Path) -> bytes:
    """从源文件读取 YAML，或接受已序列化的 YAML 文本。"""
    if isinstance(profile_source, Path):
        path = profile_source
    else:
        possible_path = Path(profile_source)
        if "\n" not in profile_source and possible_path.is_file():
            path = possible_path
        else:
            return profile_source.encode("utf-8")
    if path.suffix.lower() not in {".yaml", ".yml"}:
        raise ValueError("profile snapshots must originate from a YAML profile")
    return path.read_bytes()


class RunDirectory:
    """一个新建的运行目录，其 manifest 会被显式地最终确定。"""

    def __init__(self, path: Path, manifest: RunManifest) -> None:
        """初始化一个已由 :meth:`create` 创建的目录。"""
        self.path = path
        self._manifest = manifest
        self._artifacts: set[str] = set(manifest.artifacts)
        self._finalized = False

    @property
    def manifest(self) -> RunManifest:
        """返回当前不可变的 manifest 快照。"""
        return self._manifest.model_copy(update={"artifacts": tuple(sorted(self._artifacts))})

    @classmethod
    def create(
        cls,
        output_root: str | Path,
        *,
        profile_name: str,
        experiment: str,
        profile_source: str | Path,
        command: Sequence[str] = (),
        parameters: Mapping[str, Any] | None = None,
        run_name: str | None = None,
        run_prefix: str | None = None,
        run_suffix: str | None = None,
        now: datetime | None = None,
    ) -> "RunDirectory":
        """创建一个独占的、以 UTC 命名的运行目录与 profile 快照。

        ``profile_source`` 可以是 YAML profile 路径或已序列化的 YAML 文本。
        调用方提供的 ``run_name`` 有意从不会被修改；发生冲突时抛出
        :class:`FileExistsError`。``run_prefix`` 与 ``run_suffix`` 仅修饰自动生成的
        ``UTC时间戳-短ID`` 名称，并且不能与 ``run_name`` 同时使用。
        """
        root = _resolved_root(output_root, create=True)
        profile_component = _validate_component(profile_name, "profile_name")
        experiment_component = _validate_component(experiment, "experiment")
        if run_name is not None and (run_prefix is not None or run_suffix is not None):
            raise ValueError("run_name cannot be combined with run_prefix or run_suffix")
        if run_name is not None:
            name = _validate_component(run_name, "run_name")
        else:
            timestamp = (now or datetime.now(UTC)).astimezone(UTC).strftime("%Y%m%dT%H%M%SZ")
            generated = f"{timestamp}-{uuid4().hex[:8]}"
            components = []
            if run_prefix is not None:
                components.append(_validate_component(run_prefix, "run_prefix"))
            components.append(generated)
            if run_suffix is not None:
                components.append(_validate_component(run_suffix, "run_suffix"))
            name = "-".join(components)

        run_path = root / profile_component / experiment_component / name
        _require_descendant(root, run_path)
        run_path.mkdir(parents=True, exist_ok=False)

        try:
            profile_bytes = _profile_bytes(profile_source)
            snapshot = run_path / _PROFILE_SNAPSHOT_NAME
            snapshot.write_bytes(profile_bytes)
            profile_hash = sha256(profile_bytes).hexdigest()
            git_directory = (
                Path(profile_source).resolve().parent if isinstance(profile_source, Path) else root
            )
            manifest = RunManifest(
                created_at=(now or datetime.now(UTC)).astimezone(UTC),
                profile_name=profile_component,
                experiment=experiment_component,
                command=tuple(str(part) for part in command),
                parameters=dict(parameters or {}),
                source_profile_sha256=profile_hash,
                git=collect_git_state(git_directory),
                dependency_versions=collect_dependency_versions(),
                artifacts=(_PROFILE_SNAPSHOT_NAME,),
            )
        except Exception:
            # 该目录是此调用独占创建的，除快照外为空，
            # 因此回滚既安全也不意外。
            shutil.rmtree(run_path)
            raise
        return cls(run_path, manifest)

    def artifact_path(self, relative_path: str | Path) -> Path:
        """返回此运行内部一个安全路径，用于写入工件。"""
        if self._finalized:
            raise ArtifactError("cannot create artifacts after manifest finalization")
        candidate = self.path / relative_path
        return _require_descendant(self.path.resolve(), candidate)

    def register_artifact(self, path: str | Path) -> Path:
        """注册一个相对于此运行目录的现有常规文件。"""
        if self._finalized:
            raise ArtifactError("cannot register artifacts after manifest finalization")
        candidate = Path(path)
        full_path = candidate if candidate.is_absolute() else self.path / candidate
        resolved = _require_descendant(self.path.resolve(), full_path)
        if not resolved.is_file():
            raise ArtifactError(f"artifact does not exist or is not a regular file: {full_path}")
        self._artifacts.add(resolved.relative_to(self.path.resolve()).as_posix())
        return resolved

    def finalize(self) -> RunManifest:
        """在所有已注册工件都存在后写入 manifest。"""
        if self._finalized:
            return self.manifest
        for relative_path in self._artifacts:
            path = _require_descendant(self.path.resolve(), self.path / relative_path)
            if not path.is_file():
                raise ArtifactError(f"registered artifact is missing: {relative_path}")
        self._manifest = self.manifest
        manifest_path = self.path / _MANIFEST_NAME
        # os.replace 在单个文件系统上提供原子的 manifest 更新。
        temporary_path = self.path / f".{_MANIFEST_NAME}.{uuid4().hex}.tmp"
        temporary_path.write_text(self._manifest.model_dump_json(indent=2) + "\n", encoding="utf-8")
        os.replace(temporary_path, manifest_path)
        self._finalized = True
        return self._manifest


def _iter_run_directories(root: Path) -> Iterable[Path]:
    """只产出文档所述的 ``profile/experiment/run`` 目录结构。"""
    for profile_dir in root.iterdir():
        if not profile_dir.is_dir() or profile_dir.is_symlink() or profile_dir.name.startswith("."):
            continue
        for experiment_dir in profile_dir.iterdir():
            if not experiment_dir.is_dir() or experiment_dir.is_symlink():
                continue
            for run_dir in experiment_dir.iterdir():
                if run_dir.is_dir() and not run_dir.is_symlink():
                    yield _require_descendant(root, run_dir)


def list_runs(output_root: str | Path) -> list[RunInfo]:
    """发现现有的结构化运行，最新的优先，且不修改它们。"""
    if not Path(output_root).expanduser().exists():
        return []
    root = _resolved_root(output_root)
    runs: list[RunInfo] = []
    for run_dir in _iter_run_directories(root):
        manifest_path = run_dir / _MANIFEST_NAME
        manifest: RunManifest | None = None
        if manifest_path.is_file():
            try:
                manifest = RunManifest.model_validate_json(
                    manifest_path.read_text(encoding="utf-8")
                )
            except (OSError, ValueError, json.JSONDecodeError):
                pass
        runs.append(RunInfo(path=run_dir, manifest=manifest))
    return sorted(
        runs,
        key=lambda run: (
            run.manifest.created_at
            if run.manifest
            else datetime.fromtimestamp(run.path.stat().st_mtime, UTC)
        ),
        reverse=True,
    )


def plan_clean(
    output_root: str | Path,
    *,
    older_than_days: int | None = None,
    all_runs: bool = False,
    cache: bool = False,
    now: datetime | None = None,
) -> CleanPreview:
    """校验并预览清理目标；此函数从不删除数据。"""
    selected = sum((older_than_days is not None, all_runs, cache))
    if selected != 1:
        raise ValueError("select exactly one of older_than_days, all_runs, or cache")
    if older_than_days is not None and older_than_days < 0:
        raise ValueError("older_than_days must be nonnegative")
    output_path = Path(output_root).expanduser()
    if not output_path.exists():
        return CleanPreview(output_root=output_path.resolve(strict=False), targets=())
    root = _resolved_root(output_path)
    if cache:
        cache_dir = root / ".cache"
        targets = (_require_descendant(root, cache_dir),) if cache_dir.is_dir() else ()
        return CleanPreview(output_root=root, targets=targets)

    runs = _iter_run_directories(root)
    if older_than_days is not None:
        threshold = (now or datetime.now(UTC)).astimezone(UTC) - timedelta(days=older_than_days)
        targets = tuple(
            run_dir
            for run_dir in runs
            if datetime.fromtimestamp(run_dir.stat().st_mtime, UTC) < threshold
        )
    else:
        targets = tuple(runs)
    return CleanPreview(output_root=root, targets=tuple(sorted(targets)))


def clean_runs(preview: CleanPreview, *, apply: bool = False) -> CleanPreview:
    """仅当 ``apply`` 为真时删除此前已校验的预览。"""
    if not apply:
        return preview
    root = _resolved_root(preview.output_root)
    if root != preview.output_root.resolve():
        raise ArtifactError("output root changed since cleanup was planned")
    for target in preview.targets:
        resolved = _require_descendant(root, target)
        if not resolved.is_dir() or resolved.is_symlink():
            raise ArtifactError(f"cleanup target is not a safe directory: {target}")
        shutil.rmtree(resolved)
    return preview


__all__ = [
    "ArtifactError",
    "CleanPreview",
    "GitState",
    "RunDirectory",
    "RunInfo",
    "RunManifest",
    "clean_runs",
    "collect_dependency_versions",
    "collect_git_state",
    "list_runs",
    "plan_clean",
]

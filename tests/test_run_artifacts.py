"""针对结构化、可复现的实验运行目录的测试。"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta
import os
from pathlib import Path

import pytest

from parallel_gripper_tactile.artifacts import (
    ArtifactError,
    CleanPreview,
    RunDirectory,
    clean_runs,
    list_runs,
    plan_clean,
)


def _profile(tmp_path: Path) -> Path:
    """创建一个最小的 YAML 源配置。"""
    profile = tmp_path / "profile.yaml"
    profile.write_text("schema_version: 1\nname: test\n", encoding="utf-8")
    return profile


def test_run_snapshot_manifest_and_artifact_registration(tmp_path: Path) -> None:
    """一次已定稿的运行包含不可变的输入快照与已注册的文件。"""
    output_root = tmp_path / "outputs"
    created = datetime(2026, 8, 26, 1, 2, 3, tzinfo=UTC)
    run = RunDirectory.create(
        output_root,
        profile_name="test-profile",
        experiment="grasp",
        profile_source=_profile(tmp_path),
        command=("pgt", "run", "grasp"),
        parameters={"viewer": False},
        now=created,
    )
    trace = run.artifact_path("trace.csv")
    trace.write_text("time\n0.0\n", encoding="utf-8")
    run.register_artifact(trace)

    manifest = run.finalize()

    assert run.path.parent.parent.parent == output_root
    assert (run.path / "profile.yaml").read_text(
        encoding="utf-8"
    ) == "schema_version: 1\nname: test\n"
    assert manifest.created_at == created
    assert manifest.artifacts == ("profile.yaml", "trace.csv")
    assert '"trace.csv"' in (run.path / "manifest.json").read_text(encoding="utf-8")
    assert list_runs(output_root)[0].manifest == manifest


def test_run_name_is_exclusive_and_profile_must_be_yaml(tmp_path: Path) -> None:
    """调用方指定的运行名称绝不会静默覆盖先前的运行。"""
    source = _profile(tmp_path)
    kwargs = dict(
        profile_name="profile",
        experiment="demo",
        profile_source=source,
        run_name="named-run",
    )
    RunDirectory.create(tmp_path / "outputs", **kwargs)
    with pytest.raises(FileExistsError):
        RunDirectory.create(tmp_path / "outputs", **kwargs)

    toml = tmp_path / "legacy.toml"
    toml.write_text("name = 'legacy'\n", encoding="utf-8")
    with pytest.raises(ValueError, match="YAML"):
        RunDirectory.create(
            tmp_path / "other-outputs",
            profile_name="profile",
            experiment="demo",
            profile_source=toml,
        )


def test_generated_run_name_supports_safe_prefix_and_suffix(tmp_path: Path) -> None:
    """自动名称可以在保留时间戳与短 ID 的同时增加可读标签。"""
    created = datetime(2026, 8, 30, 10, 47, 26, tzinfo=UTC)
    run = RunDirectory.create(
        tmp_path / "outputs",
        profile_name="profile",
        experiment="force-track",
        profile_source=_profile(tmp_path),
        run_prefix="pid-only-soft",
        run_suffix="seed01",
        now=created,
    )

    assert run.path.name.startswith("pid-only-soft-20260830T104726Z-")
    assert run.path.name.endswith("-seed01")
    with pytest.raises(ValueError, match="cannot be combined"):
        RunDirectory.create(
            tmp_path / "outputs",
            profile_name="profile",
            experiment="force-track",
            profile_source=_profile(tmp_path),
            run_name="exact",
            run_prefix="study",
        )
    with pytest.raises(ValueError, match="simple path component"):
        RunDirectory.create(
            tmp_path / "outputs",
            profile_name="profile",
            experiment="force-track",
            profile_source=_profile(tmp_path),
            run_prefix="../escape",
        )


def test_artifacts_cannot_escape_run_or_be_registered_after_finalization(tmp_path: Path) -> None:
    """工件 API 强制限定在边界内，且定稿后的清单已关闭。"""
    run = RunDirectory.create(
        tmp_path / "outputs",
        profile_name="profile",
        experiment="demo",
        profile_source=_profile(tmp_path),
        run_name="run",
    )
    with pytest.raises(ArtifactError):
        run.artifact_path("../../outside.csv")
    with pytest.raises(ArtifactError):
        run.register_artifact("missing.csv")

    run.finalize()
    with pytest.raises(ArtifactError):
        run.artifact_path("late.csv")


def test_cleanup_is_a_preview_and_never_accepts_an_outside_target(tmp_path: Path) -> None:
    """清理仅在显式 apply 后移除已校验的后代。"""
    root = tmp_path / "outputs"
    run = RunDirectory.create(
        root,
        profile_name="profile",
        experiment="demo",
        profile_source=_profile(tmp_path),
        run_name="run",
    )
    run.finalize()

    preview = plan_clean(root, all_runs=True)
    assert preview.targets == (run.path,)
    clean_runs(preview)
    assert run.path.is_dir()
    clean_runs(preview, apply=True)
    assert not run.path.exists()

    outside = tmp_path / "outside"
    outside.mkdir()
    malicious = CleanPreview(output_root=root, targets=(outside,))
    with pytest.raises(ArtifactError, match="outside output root"):
        clean_runs(malicious, apply=True)


def test_age_and_cache_cleanup_selection(tmp_path: Path) -> None:
    """按龄与缓存模式是显式且互斥的。"""
    root = tmp_path / "outputs"
    run = RunDirectory.create(
        root,
        profile_name="profile",
        experiment="demo",
        profile_source=_profile(tmp_path),
        run_name="old-run",
    )
    run.finalize()
    old = (datetime.now(UTC) - timedelta(days=4)).timestamp()
    os.utime(run.path, (old, old))

    assert plan_clean(root, older_than_days=3).targets == (run.path,)
    assert plan_clean(root, cache=True).targets == ()
    cache = root / ".cache"
    cache.mkdir()
    assert plan_clean(root, cache=True).targets == (cache,)
    with pytest.raises(ValueError, match="exactly one"):
        plan_clean(root, all_runs=True, cache=True)

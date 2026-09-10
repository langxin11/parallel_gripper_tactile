"""Hydra 入口共享的解析器注册与复现元数据。"""

from __future__ import annotations

from importlib import metadata
from pathlib import Path
import subprocess
from typing import Any

from hydra.core.hydra_config import HydraConfig
from omegaconf import DictConfig, OmegaConf

from .configuration import REPOSITORY_ROOT


def register_resolvers() -> None:
    """注册与调用目录无关的仓库根路径解析器。"""
    if not OmegaConf.has_resolver("pgt_repo"):
        OmegaConf.register_new_resolver("pgt_repo", lambda: str(REPOSITORY_ROOT), use_cache=True)


def resolved_mapping(config: DictConfig) -> dict[str, object]:
    """解析所有插值和缺失值，并返回普通严格映射。"""
    value = OmegaConf.to_container(config, resolve=True, throw_on_missing=True)
    if not isinstance(value, dict):
        raise TypeError("research configuration root must be a mapping")
    return value


def _git(*arguments: str) -> str | None:
    """读取仓库 Git 信息；不可用时返回 ``None``。"""
    result = subprocess.run(
        ("git", *arguments),
        cwd=REPOSITORY_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def composition_provenance() -> dict[str, Any]:
    """返回 Hydra 选择、覆盖和仓库版本信息。"""
    hydra_config = HydraConfig.get()
    status = _git("status", "--short")
    return {
        "schema_version": 1,
        "config_sources": [str(source.path) for source in hydra_config.runtime.config_sources],
        "choices": dict(hydra_config.runtime.choices),
        "overrides": list(hydra_config.overrides.task),
        "hydra_output_directory": str(Path(hydra_config.runtime.output_dir).resolve()),
        "repository_root": str(REPOSITORY_ROOT),
        "dependency_versions": {
            distribution: metadata.version(distribution)
            for distribution in ("parallel-gripper-tactile", "hydra-core", "omegaconf")
        },
        "git": {
            "commit": _git("rev-parse", "HEAD"),
            "branch": _git("branch", "--show-current"),
            "dirty": bool(status),
            "status": status.splitlines() if status else [],
        },
    }


__all__ = [
    "composition_provenance",
    "register_resolvers",
    "resolved_mapping",
]

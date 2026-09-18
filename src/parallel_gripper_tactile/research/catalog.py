"""科研配置目录的只读发现能力。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .configuration import REPOSITORY_ROOT


CONFIG_GROUPS: tuple[str, ...] = (
    "experiment",
    "platform",
    "model",
    "controller",
    "estimator",
    "task",
    "material",
    "execution",
    "research",
)


class ConfigCatalogError(ValueError):
    """配置目录请求无效或其中的 YAML 无法读取。"""


@dataclass(frozen=True, slots=True)
class ConfigCatalogEntry:
    """一个可由配置组选择的 YAML 配置。"""

    group: str
    name: str
    purpose: str


def list_configurations(
    group: str | None = None,
    *,
    search: str | None = None,
    repository_root: Path = REPOSITORY_ROOT,
) -> tuple[ConfigCatalogEntry, ...]:
    """列出受支持配置组中的 YAML，并按组和选择名排序。

    Args:
        group: 可选的配置组名。
        search: 可选的名称或用途子串过滤条件。
        repository_root: 仓库根目录，供测试注入隔离目录。

    Returns:
        只含可复制 Hydra 选择名的配置条目。

    Raises:
        ConfigCatalogError: 配置组无效或 YAML 顶层不是映射时抛出。
    """
    groups = _selected_groups(group)
    normalized_search = search.casefold().strip() if search else ""
    entries: list[ConfigCatalogEntry] = []
    configs_root = repository_root.resolve() / "configs"
    for selected_group in groups:
        directory = configs_root / selected_group
        if not directory.is_dir():
            continue
        for source in sorted(directory.rglob("*.yaml")):
            relative_source = source.relative_to(directory)
            if any(part.startswith("_") for part in relative_source.parts):
                continue
            entry = ConfigCatalogEntry(
                group=selected_group,
                name=relative_source.with_suffix("").as_posix(),
                purpose=_purpose(_load_mapping(source), selected_group),
            )
            if normalized_search and normalized_search not in (
                f"{entry.name}\n{entry.purpose}".casefold()
            ):
                continue
            entries.append(entry)
    return tuple(sorted(entries, key=lambda entry: (entry.group, entry.name)))


def _selected_groups(group: str | None) -> tuple[str, ...]:
    """校验可选配置组，避免目录路径参与发现。"""
    if group is None:
        return CONFIG_GROUPS
    if group not in CONFIG_GROUPS:
        available = "、".join(CONFIG_GROUPS)
        raise ConfigCatalogError(f"未知配置组“{group}”。可用配置组：{available}。")
    return (group,)


def _load_mapping(source: Path) -> Mapping[str, Any]:
    """读取一个配置文件的顶层映射，不解析 Hydra 插值。"""
    try:
        with source.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ConfigCatalogError(f"无法读取配置“{source}”：{error}") from error
    if not isinstance(document, Mapping):
        raise ConfigCatalogError(f"配置“{source}”的顶层必须是映射。")
    return document


def _purpose(document: Mapping[str, Any], group: str) -> str:
    """从既有领域元数据中提取面向发现的用途说明。"""
    if group == "research":
        question = _nested_text(document, "study", "rationale", "question")
        if question:
            return question
    if group == "experiment":
        kind = _nested_text(document, "experiment", "kind")
        name = _nested_text(document, "experiment", "name")
        return _join_description(kind, name)
    if group == "task":
        name = _nested_text(document, "definition", "name") or _text(document.get("name"))
        family = _text(document.get("family"))
        return _join_description(family, name)
    if group == "controller":
        name = _text(document.get("name"))
        family = _text(document.get("family"))
        return _join_description(family, name)
    if group == "platform":
        return _join_description(_text(document.get("family")), _text(document.get("backend")))
    if group == "model":
        return _join_description(_text(document.get("family")), _text(document.get("name")))
    if group == "estimator":
        return _text(document.get("name"))
    if group == "material":
        return _text(document.get("name"))
    if group == "execution":
        return _text(document.get("mode"))
    return ""


def _nested_text(document: Mapping[str, Any], *keys: str) -> str:
    """安全取得嵌套映射中的字符串值。"""
    current: Any = document
    for key in keys:
        if not isinstance(current, Mapping):
            return ""
        current = current.get(key)
    return _text(current)


def _text(value: object) -> str:
    """将元数据中的标量转换为可显示文本。"""
    return value.strip() if isinstance(value, str) else ""


def _join_description(*parts: str) -> str:
    """连接存在的元数据字段。"""
    return "／".join(part for part in parts if part)


__all__ = ["CONFIG_GROUPS", "ConfigCatalogEntry", "ConfigCatalogError", "list_configurations"]

"""科研配置发现目录与 CLI 的行为测试。"""

from __future__ import annotations

from pathlib import Path

import pytest
from typer.testing import CliRunner

from parallel_gripper_tactile.cli import app
from parallel_gripper_tactile.research.catalog import ConfigCatalogError, list_configurations


ROOT = Path(__file__).resolve().parents[1]
RUNNER = CliRunner()


def test_catalog_discovers_real_nested_research_and_experiment_choices() -> None:
    """现有嵌套研究与实验配置均返回相对组路径选择名。"""
    research = list_configurations("research")
    experiments = list_configurations("experiment")

    selection = next(
        entry for entry in research if entry.name == "force_controller_selection/study"
    )
    assert "哪种仍有竞争力" in selection.purpose
    assert "dm_gripper/force_tracking_default" in {entry.name for entry in experiments}


def test_catalog_search_filters_metadata_and_results_are_stably_ordered() -> None:
    """检索覆盖用途字段，结果按完整选择名排序。"""
    entries = list_configurations("research", search="哪种仍有竞争力")

    assert [entry.name for entry in entries] == ["force_controller_selection/study"]
    assert [entry.name for entry in list_configurations("controller")] == sorted(
        entry.name for entry in list_configurations("controller")
    )


def test_catalog_hides_internal_controller_fragments() -> None:
    """控制器目录只展示可直接选择的入口，不暴露继承片段。"""
    controllers = {entry.name for entry in list_configurations("controller")}

    assert "dm_gripper/admittance" in controllers
    assert "dm_gripper/admittance_unified" not in controllers
    assert all(not part.startswith("_") for name in controllers for part in name.split("/"))


def test_catalog_rejects_unknown_group_without_reading_paths() -> None:
    """组名白名单拒绝路径遍历和未支持组。"""
    with pytest.raises(ConfigCatalogError, match="未知配置组"):
        list_configurations("../research")


def test_configs_cli_is_cwd_independent_and_does_not_create_outputs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """发现命令只读取仓库配置，在其他工作目录不运行实验。"""
    output_root = ROOT / "outputs"
    existed_before = output_root.exists()
    monkeypatch.chdir(tmp_path)
    result = RUNNER.invoke(
        app,
        ["configs", "list", "experiment", "--search", "dm_force_tracking"],
        catch_exceptions=False,
        terminal_width=160,
    )

    assert result.exit_code == 0
    assert "可用科研配置" in result.output
    assert output_root.exists() is existed_before


def test_configs_cli_reports_unknown_group_with_exit_code_two() -> None:
    """CLI 将目录层错误转为统一的用户退出码。"""
    result = RUNNER.invoke(app, ["configs", "list", "../research"])

    assert result.exit_code == 2
    assert "未知配置组" in result.output

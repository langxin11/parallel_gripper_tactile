"""增量测试模块映射入口的验证。"""

from __future__ import annotations

import importlib.util
from pathlib import Path


def _module() -> object:
    """加载增量测试脚本而不执行命令行入口。"""
    path = Path(__file__).parents[1] / "scripts" / "test_changed.py"
    spec = importlib.util.spec_from_file_location("test_changed_script", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_changed_test_file_maps_to_itself() -> None:
    """直接修改测试文件时只选择该文件。"""
    module = _module()

    assert module.select_tests(["tests/test_control.py"]) == ("tests/test_control.py",)


def test_changed_force_tracking_module_maps_to_related_tests() -> None:
    """力跟踪实现应映射到同名前缀测试。"""
    module = _module()

    selected = module.select_tests(["src/parallel_gripper_tactile/experiments/force_tracking.py"])

    assert selected is not None
    assert "tests/test_force_tracking.py" in selected
    assert "tests/test_force_tracking_runner.py" in selected


def test_changed_package_maps_to_its_workspace_tests() -> None:
    """workspace 包改动应落到该包自己的测试目录。"""
    module = _module()

    assert module.select_tests(["packages/robotiq_grasp_core/src/robotiq_grasp_core/core.py"]) == (
        "packages/robotiq_grasp_core/tests",
    )


def test_changed_dm_core_maps_to_real_and_simulation_consumers() -> None:
    """共享核改动应同时覆盖核心、真机实验包与仿真力跟踪回归。"""
    module = _module()

    selected = module.select_tests(
        ["packages/dm_grasp_core/src/dm_grasp_core/control/normal_force.py"]
    )

    assert selected == (
        "packages/dm_grasp_core/tests",
        "packages/dmgripper_experiments/tests",
        "tests/test_force_tracking.py",
    )


def test_changed_experiment_package_maps_to_core_contract_tests() -> None:
    """真机实验包改动应同时运行本包测试与共享核契约回归。"""
    module = _module()

    selected = module.select_tests(
        ["packages/dmgripper_experiments/src/dmgripper_experiments/runtime.py"]
    )

    assert selected == (
        "packages/dm_grasp_core/tests",
        "packages/dmgripper_experiments/tests",
    )


def test_changed_hardware_package_maps_to_experiment_consumers() -> None:
    """DM 与 PapillArray 硬件包改动应覆盖真机实验包测试。"""
    module = _module()

    selected = module.select_tests(
        ["packages/papillarray_hardware/src/papillarray_hardware/sensor.py"]
    )

    assert selected == (
        "packages/dmgripper_experiments/tests",
        "packages/papillarray_hardware/tests",
    )


def test_test_infrastructure_change_falls_back_to_full_suite() -> None:
    """测试基础设施改动不得由增量映射缩小门禁范围。"""
    module = _module()

    assert module.select_tests(["conftest.py"]) is None


def test_research_entry_and_config_map_to_complete_research_tests() -> None:
    """科研入口与组合配置改动应覆盖配置、执行和 study 三层测试。"""
    module = _module()
    expected = (
        "tests/test_research_configuration.py",
        "tests/test_research_execution.py",
        "tests/test_research_study.py",
        "tests/test_study_lifecycle.py",
        "tests/test_study_progress.py",
    )

    assert module.select_tests(["scripts/research/run.py"]) == expected
    assert module.select_tests(["configs/controller/dm_gripper/full.yaml"]) == expected


def test_study_lifecycle_maps_to_all_migrated_protocol_tests() -> None:
    """公共生命周期改动必须覆盖活跃协议及生命周期专属测试。"""
    module = _module()
    selected = module.select_tests(["src/parallel_gripper_tactile/studies/lifecycle.py"])

    assert selected is not None
    assert "tests/test_study_lifecycle.py" in selected
    assert "tests/test_force_tracking_controller_comparison_script.py" in selected
    assert "tests/test_friction_estimator_validation_study.py" in selected


def test_documentation_only_change_needs_no_pytest() -> None:
    """纯文档改动不选择 pytest。"""
    module = _module()

    assert module.select_tests(["docs/testing.md"]) == ()


def test_config_discovery_maps_to_catalog_and_cli_tests() -> None:
    """配置目录和展示入口改动不会漏掉发现命令的行为覆盖。"""
    module = _module()
    selected = module.select_tests(["src/parallel_gripper_tactile/research/catalog.py"])
    assert "tests/test_config_catalog.py" in selected
    selected = module.select_tests(["src/parallel_gripper_tactile/cli/configs.py"])
    assert "tests/test_config_catalog.py" in selected
    assert "tests/test_cli.py" in selected

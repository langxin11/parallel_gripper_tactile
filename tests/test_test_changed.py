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

    assert module.select_tests(["packages/dm_grasp_core/src/dm_grasp_core/control.py"]) == (
        "packages/dm_grasp_core/tests",
    )


def test_test_infrastructure_change_falls_back_to_full_suite() -> None:
    """测试基础设施改动不得由增量映射缩小门禁范围。"""
    module = _module()

    assert module.select_tests(["conftest.py"]) is None


def test_documentation_only_change_needs_no_pytest() -> None:
    """纯文档改动不选择 pytest。"""
    module = _module()

    assert module.select_tests(["docs/testing.md"]) == ()

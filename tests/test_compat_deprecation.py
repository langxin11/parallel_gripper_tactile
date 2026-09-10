"""根路径兼容导出层的弃用警示与转发等价性。

兼容层计划于 0.4.0 移除（见 CHANGELOG 与 docs/architecture.md），
在删除前必须持续满足两件事：导入时发出指向规范路径的 ``DeprecationWarning``，
以及转发名称与规范实现保持同一对象。
"""

from __future__ import annotations

import importlib

import pytest

_COMPAT_SHIMS = {
    "parallel_gripper_tactile.profiles": "parallel_gripper_tactile.config.profiles",
    "parallel_gripper_tactile.run_artifacts": "parallel_gripper_tactile.artifacts.run_artifacts",
    "parallel_gripper_tactile.plotstyle": "parallel_gripper_tactile.visualization.plotstyle",
    "parallel_gripper_tactile.friction_plots": "parallel_gripper_tactile.visualization.friction",
    "parallel_gripper_tactile.taxel_friction": "parallel_gripper_tactile.perception.taxels",
    "parallel_gripper_tactile.tactile_slip": "parallel_gripper_tactile.perception.slip",
    "parallel_gripper_tactile.friction_estimation": "parallel_gripper_tactile.perception.friction",
    "parallel_gripper_tactile.discrete_force_control": "robotiq_grasp_core.discrete_force_control",
}


@pytest.mark.parametrize(("shim", "canonical"), sorted(_COMPAT_SHIMS.items()))
def test_shim_warns_on_import(shim: str, canonical: str) -> None:
    """重新导入兼容层必须发出指向规范路径的 ``DeprecationWarning``。"""
    module = importlib.import_module(shim)
    with pytest.warns(DeprecationWarning, match=canonical.replace(".", r"\.")):
        importlib.reload(module)


@pytest.mark.parametrize(("shim", "canonical"), sorted(_COMPAT_SHIMS.items()))
def test_shim_forwards_canonical_names(shim: str, canonical: str) -> None:
    """兼容层公开的每个名称都必须与规范实现中的对象保持同一。"""
    shim_module = importlib.import_module(shim)
    canonical_module = importlib.import_module(canonical)
    public_names = [name for name in dir(shim_module) if not name.startswith("_")]
    assert public_names, "兼容层不应为空模块。"
    for name in public_names:
        assert getattr(shim_module, name) is getattr(canonical_module, name), name

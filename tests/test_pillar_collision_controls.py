"""验证默认 Pillar 球体模型生成与求解器控制。"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from parallel_gripper_tactile.config.profiles import load_profile
from parallel_gripper_tactile.scenes.custom import build_custom_grasp_model


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "grippers" / "dm_gripper" / "parallel_gripper_prepared.xml"


def _generator_module():
    """加载仓库内的高度球体模型生成器。"""
    path = ROOT / "scripts" / "prepare_height_sphere_collision_model.py"
    spec = importlib.util.spec_from_file_location("pillar_collision_model_generator", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _named_elements(root: ET.Element, expression: str) -> dict[str, ET.Element]:
    """按名称索引指定 XML 元素。"""
    return {
        name: element
        for element in root.findall(expression)
        if (name := element.get("name")) is not None
    }


def test_height_sphere_generator_preserves_taxel_site_heights(tmp_path: Path) -> None:
    """默认球体代理逐点保留原始 taxel site 高度。"""
    generator = _generator_module()
    height_spheres = tmp_path / "height-spheres.xml"
    generator.make_height_sphere_model(SOURCE, height_spheres)

    source_root = ET.parse(SOURCE).getroot()
    source_sites = _named_elements(source_root, ".//site")
    sphere_geoms = _named_elements(
        ET.parse(height_spheres).getroot(), ".//geom[@class='pillar_collision']"
    )

    assert len(sphere_geoms) == 18
    assert {geom.get("type") for geom in sphere_geoms.values()} == {"sphere"}
    for name, geom in sphere_geoms.items():
        site_name = name.replace("_geom_", "_", 1)
        assert geom.get("pos") == source_sites[site_name].get("pos")
        assert geom.get("size") == "0.0028"


def test_custom_scene_can_disable_multiccd_without_changing_the_model() -> None:
    """关闭 multiccd 只设置求解选项，不替换 Pillar mesh。"""
    default_profile = load_profile(ROOT / "configs" / "dm_gripper.yaml")
    profile = default_profile.model_copy(
        update={"model": default_profile.model.model_copy(update={"path": SOURCE})}
    )
    model = build_custom_grasp_model(profile, multiccd_enabled=False)
    flag = int(mujoco.mjtDisableBit.mjDSBL_MULTICCD)

    assert model.opt.disableflags & flag
    assert model.geom("gripper/left_taxel_geom_11").type == mujoco.mjtGeom.mjGEOM_MESH

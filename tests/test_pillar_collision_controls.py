"""验证 Pillar 碰撞几何的三个因果对照。"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco

from parallel_gripper_tactile.profiles import load_profile
from parallel_gripper_tactile.scenes.custom import build_custom_grasp_model


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets" / "grippers" / "custom_parallel_gripper" / "parallel_gripper_prepared.xml"


def _generator_module():
    """Load the repository-native model generator for direct unit testing."""
    path = ROOT / "scripts" / "prepare_flat_sphere_collision_model.py"
    spec = importlib.util.spec_from_file_location("pillar_collision_model_generator", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _diagnosis_module():
    """Load the explicit force-tracking diagnosis protocol."""
    path = ROOT / "scripts" / "experiments" / "force_tracking_diagnosis.py"
    module_name = "force_tracking_diagnosis_protocol"
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _named_elements(root: ET.Element, expression: str) -> dict[str, ET.Element]:
    """Index named XML elements selected by ``expression``."""
    return {
        name: element
        for element in root.findall(expression)
        if (name := element.get("name")) is not None
    }


def test_collision_variants_change_one_geometry_factor_at_a_time(tmp_path: Path) -> None:
    """球体保留原高度，而共面 mesh 仅平移碰撞几何。"""
    generator = _generator_module()
    height_spheres = tmp_path / "height-spheres.xml"
    coplanar_mesh = tmp_path / "coplanar-mesh.xml"
    generator.make_collision_variant_model(SOURCE, height_spheres, variant="height-sphere")
    generator.make_collision_variant_model(SOURCE, coplanar_mesh, variant="coplanar-mesh")

    source_root = ET.parse(SOURCE).getroot()
    source_geoms = _named_elements(source_root, ".//geom[@class='pillar_collision']")
    source_sites = _named_elements(source_root, ".//site")
    sphere_geoms = _named_elements(
        ET.parse(height_spheres).getroot(), ".//geom[@class='pillar_collision']"
    )
    mesh_geoms = _named_elements(
        ET.parse(coplanar_mesh).getroot(), ".//geom[@class='pillar_collision']"
    )

    assert len(sphere_geoms) == len(mesh_geoms) == 18
    sphere_heights = {float(geom.get("pos", "").split()[2]) for geom in sphere_geoms.values()}
    assert sphere_heights == {0.03295, 0.03325, 0.03345}
    assert {geom.get("type") for geom in sphere_geoms.values()} == {"sphere"}
    assert {geom.get("type") for geom in mesh_geoms.values()} == {"mesh"}

    effective_tip_heights = set()
    for name, geom in mesh_geoms.items():
        source_geom_z = float(source_geoms[name].get("pos", "").split()[2])
        shifted_geom_z = float(geom.get("pos", "").split()[2])
        site_name = name.replace("_geom_", "_", 1)
        source_tip_z = float(source_sites[site_name].get("pos", "").split()[2])
        effective_tip_heights.add(round(source_tip_z + shifted_geom_z - source_geom_z, 8))
    assert effective_tip_heights == {0.03345}


def test_custom_scene_can_disable_multiccd_without_changing_the_model() -> None:
    """关闭 multiccd 只设置求解选项，不替换 Pillar mesh。"""
    default_profile = load_profile(ROOT / "configs" / "custom_parallel_gripper.yaml")
    profile = default_profile.model_copy(
        update={"model": default_profile.model.model_copy(update={"path": SOURCE})}
    )
    model = build_custom_grasp_model(profile, multiccd_enabled=False)
    flag = int(mujoco.mjtDisableBit.mjDSBL_MULTICCD)

    assert model.opt.disableflags & flag
    assert model.geom("gripper/left_taxel_geom_11").type == mujoco.mjtGeom.mjGEOM_MESH


def test_diagnosis_protocol_contains_baselines_and_three_causal_controls() -> None:
    """碰撞几何阶段按固定顺序运行两个端点和三个新增对照。"""
    diagnosis = _diagnosis_module()
    config = diagnosis.load_config(ROOT / "configs" / "studies" / "force_tracking_diagnosis.yaml")
    conditions = diagnosis._conditions(config, "collision-geometry")

    assert [condition[0] for condition in conditions] == [
        "original-mesh-multiccd",
        "height-spheres-multiccd",
        "coplanar-mesh-multiccd",
        "original-mesh-single-contact",
        "coplanar-spheres-multiccd",
    ]
    assert [condition[-1] for condition in conditions] == [True, True, True, False, True]

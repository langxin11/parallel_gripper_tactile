"""从 taxel 夹爪资产生成水平夹爪闭合抓取正方体的 MuJoCo 场景。"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GRIPPER_XML = REPOSITORY_ROOT / "assets" / "robotiq_2f85" / "2f85_taxels.xml"
DEFAULT_OUTPUT_XML = REPOSITORY_ROOT / "assets" / "scenes" / "cube_grasp.xml"

# 基础 2F-85 模型的指长轴是局部 +Z。绕 X 轴旋转 90° 后，指长轴转为世界 -Y，
# 而两侧指尖法向仍保持世界 ±X，满足水平侧向夹取的约定。
_HORIZONTAL_GRIPPER_POS = "0 0 0.08"
_HORIZONTAL_GRIPPER_QUAT = "0.70710678 0.70710678 0 0"
_CUBE_POS = "0 -0.155 0.08"


def build_cube_grasp_tree(gripper_xml: Path) -> ET.ElementTree:
    """构建夹爪水平、方块居中的带地面闭合抓取场景。

    无重力用于隔离触觉接触参数，避免物块在闭合前从两指之间掉落。
    后续若研究抬升或滑移，应基于此场景添加机械臂、支撑面与重力。

    Args:
        gripper_xml: 已含 taxel 的 Robotiq 2F-85 MJCF 文件。

    Returns:
        包含自由正方体物块、相机与 keyframe 的 MJCF 树。
    """
    tree = ET.parse(gripper_xml)
    root = tree.getroot()
    root.set("model", "robotiq_2f85_cube_grasp")
    compiler = root.find("compiler")
    if compiler is None:
        raise ValueError("夹爪资产缺少 compiler 节点。")
    # 输出文件位于 ``assets/scenes``，因此 meshdir 需相对该目录重新定位。
    compiler.set("meshdir", "../robotiq_2f85/assets")
    option = root.find("option")
    if option is None:
        raise ValueError("夹爪资产缺少 option 节点。")
    option.set("gravity", "0 0 0")
    option.set("timestep", "0.002")
    option.set("integrator", "implicitfast")

    asset = root.find("asset")
    if asset is None:
        raise ValueError("夹爪资产缺少 asset 节点。")
    ET.SubElement(
        asset,
        "texture",
        name="ground_checker",
        type="2d",
        builtin="checker",
        rgb1="0.12 0.15 0.18",
        rgb2="0.48 0.52 0.56",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset,
        "material",
        name="ground_checker",
        texture="ground_checker",
        texuniform="true",
        texrepeat="16 16",
        reflectance="0.15",
    )

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("夹爪资产缺少 worldbody 节点。")
    base = worldbody.find("body[@name='base']")
    if base is None:
        raise ValueError("夹爪资产缺少 base body。")
    base.set("pos", _HORIZONTAL_GRIPPER_POS)
    base.set("quat", _HORIZONTAL_GRIPPER_QUAT)

    ET.SubElement(
        worldbody,
        "geom",
        name="ground",
        type="plane",
        size="0 0 0.05",
        material="ground_checker",
        friction="0.9 0.02 0.001",
    )
    ET.SubElement(worldbody, "light", pos="0 -0.3 0.5", directional="true", dir="0 0.5 -1")
    ET.SubElement(
        worldbody,
        "camera",
        name="grasp_overview",
        pos="0.28 -0.35 0.22",
        xyaxes="0.78 0.62 0  -0.30 0.38 0.87",
    )
    # 此位置由指尖局部中心 (0, 0, 0.155) 经根节点水平变换得到。
    cube = ET.SubElement(worldbody, "body", name="target_cube", pos=_CUBE_POS)
    ET.SubElement(cube, "freejoint", name="target_cube_free_joint")
    ET.SubElement(
        cube,
        "geom",
        name="target_cube_geom",
        type="box",
        size="0.015 0.015 0.015",
        mass="0.03",
        friction="0.8 0.02 0.001",
        solimp="0.90 0.95 0.002",
        solref="0.015 1",
        rgba="0.95 0.55 0.1 1",
    )
    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframe, "key", name="open", ctrl="0")
    ET.SubElement(keyframe, "key", name="closed", ctrl="220")
    return tree


def main() -> None:
    """写出水平夹爪闭合抓取方块的派生 MJCF 场景。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gripper-xml", type=Path, default=DEFAULT_GRIPPER_XML)
    parser.add_argument("--output-xml", type=Path, default=DEFAULT_OUTPUT_XML)
    args = parser.parse_args()
    tree = build_cube_grasp_tree(args.gripper_xml)
    ET.indent(tree, space="  ")
    args.output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output_xml, encoding="utf-8", xml_declaration=True)
    print(f"已生成 {args.output_xml}")


if __name__ == "__main__":
    main()

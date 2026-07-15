"""从 taxel 夹爪资产生成水平夹爪闭合抓取正方体的 MuJoCo 场景。"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GRIPPER_XML = REPOSITORY_ROOT / "assets" / "robotiq_2f85" / "2f85_taxels.xml"
DEFAULT_OUTPUT_XML = REPOSITORY_ROOT / "assets" / "scenes" / "cube_grasp.xml"


def build_cube_grasp_tree(gripper_xml: Path) -> ET.ElementTree:
    """构建夹爪水平、方块居中的无重力闭合抓取场景。

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

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("夹爪资产缺少 worldbody 节点。")
    ET.SubElement(worldbody, "light", pos="0 -0.3 0.5", directional="true", dir="0 0.5 -1")
    ET.SubElement(
        worldbody,
        "camera",
        name="grasp_overview",
        pos="0.24 -0.28 0.23",
        xyaxes="0.76 0.65 0  -0.36 0.42 0.83",
    )
    # 2F-85 在闭合过程中指尖会略向 +z 方向抬升；该高度对齐闭合后的 3×3 taxel 网格。
    cube = ET.SubElement(worldbody, "body", name="target_cube", pos="0 0 0.155")
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

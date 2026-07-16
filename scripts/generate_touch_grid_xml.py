"""从基础 2F-85 MJCF 生成使用 MuJoCo ``touch_grid`` 插件的触觉资产。

常见用法::

    uv run scripts/generate_touch_grid_xml.py
    uv run scripts/generate_touch_grid_xml.py --output-xml /tmp/2f85_touch_grid.xml
    uv run scripts/generate_touch_grid_xml.py --rows 3 --cols 3 \\
        --output-xml assets/robotiq_2f85/2f85_touch_grid_3x3.xml
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASE_XML = REPOSITORY_ROOT / "assets/robotiq_2f85/2f85.xml"
DEFAULT_OUTPUT_XML = REPOSITORY_ROOT / "assets/robotiq_2f85/2f85_touch_grid.xml"
DEFAULT_GRID_ROWS = 32
DEFAULT_GRID_COLS = 32
PAD_X = 0.043258
PAD_HALF_X = 0.002
PAD_HALF_Y = 0.011
PAD_MIN_Z = 0.110625
PAD_MAX_Z = 0.148125


def _append_touch_grid(pad: ET.Element, side: str, rows: int, cols: int) -> None:
    """将一侧 pad 替换为指定分辨率的接触单元及一个 touch_grid site。"""
    for geom in list(pad.findall("geom")):
        pad.remove(geom)
    cell_half_y = PAD_HALF_Y / cols
    cell_half_z = (PAD_MAX_Z - PAD_MIN_Z) / (2 * rows)
    for row in range(rows):
        z = PAD_MIN_Z + (2 * row + 1) * cell_half_z
        for col in range(cols):
            y = -PAD_HALF_Y + (2 * col + 1) * cell_half_y
            ET.SubElement(
                pad,
                "geom",
                name=f"{side}_touch_cell_{row:02d}_{col:02d}",
                type="box",
                pos=f"{PAD_X:.6f} {y:.6f} {z:.6f}",
                size=f"{PAD_HALF_X:.6f} {cell_half_y:.6f} {cell_half_z:.6f}",
                mass="1e-6",
                friction="0.7 0.03 0.01",
                solimp="0.90 0.95 0.002",
                solref="0.015 1",
                rgba="0.2 0.2 0.2 1",
            )
    ET.SubElement(
        pad,
        "site",
        name=f"touch_{side}",
        pos=f"{PAD_X:.6f} 0 {(PAD_MIN_Z + PAD_MAX_Z) / 2:.6f}",
        quat="1 0 -1 0",
        size="0.001",
        rgba="0 0 0 0",
    )


def build_touch_grid_tree(
    base_xml: Path, rows: int = DEFAULT_GRID_ROWS, cols: int = DEFAULT_GRID_COLS
) -> ET.ElementTree:
    """构建每侧输出 ``3×rows×cols`` 的 touch_grid 夹爪模型。

    Args:
        base_xml: 未添加触觉结构的 Robotiq 2F-85 MJCF 文件。

    Returns:
        含碰撞网格、site 与 ``touch_grid`` 插件传感器的 MJCF 树。
    """
    if rows <= 0 or cols <= 0:
        raise ValueError("touch_grid 的行数和列数必须为正整数。")
    tree = ET.parse(base_xml)
    root = tree.getroot()
    root.set("model", "robotiq_2f85_touch_grid")
    extension = ET.Element("extension")
    ET.SubElement(extension, "plugin", plugin="mujoco.sensor.touch_grid")
    root.insert(2, extension)
    size = root.find("size")
    if size is None:
        size = ET.Element("size")
        root.insert(3, size)
    # 2×1024 个 pad 接触单元在闭合时需要比基础夹爪更大的碰撞工作区。
    size.attrib.pop("njmax", None)
    size.attrib.pop("nconmax", None)
    size.set("memory", "128M")
    sensor = ET.Element("sensor")
    for side in ("left", "right"):
        pad = root.find(f".//body[@name='{side}_pad']")
        if pad is None:
            raise ValueError(f"基础模型缺少 {side}_pad body。")
        _append_touch_grid(pad, side, rows, cols)
        plugin = ET.SubElement(
            sensor,
            "plugin",
            name=f"touch_{side}",
            plugin="mujoco.sensor.touch_grid",
            objtype="site",
            objname=f"touch_{side}",
        )
        ET.SubElement(plugin, "config", key="size", value=f"{cols} {rows}")
        ET.SubElement(plugin, "config", key="fov", value="23 38")
        ET.SubElement(plugin, "config", key="gamma", value="0")
        ET.SubElement(plugin, "config", key="nchannel", value="3")
    root.append(sensor)
    return tree


def main() -> None:
    """生成 touch_grid MJCF 资产。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-xml", type=Path, default=DEFAULT_BASE_XML)
    parser.add_argument("--output-xml", type=Path, default=DEFAULT_OUTPUT_XML)
    parser.add_argument("--rows", type=int, default=DEFAULT_GRID_ROWS, help="触觉网格行数。")
    parser.add_argument("--cols", type=int, default=DEFAULT_GRID_COLS, help="触觉网格列数。")
    args = parser.parse_args()
    try:
        tree = build_touch_grid_tree(args.base_xml, args.rows, args.cols)
    except ValueError as error:
        parser.error(str(error))
    ET.indent(tree, space="  ")
    args.output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output_xml, encoding="utf-8", xml_declaration=True)
    print(f"已生成 {args.output_xml}")


if __name__ == "__main__":
    main()

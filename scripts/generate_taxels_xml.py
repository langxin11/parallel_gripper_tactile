"""从基础 Robotiq 2F-85 MJCF 生成带离散触觉 taxel 的派生资产。"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASE_XML = REPOSITORY_ROOT / "assets" / "robotiq_2f85" / "2f85.xml"
DEFAULT_OUTPUT_XML = REPOSITORY_ROOT / "assets" / "robotiq_2f85" / "2f85_taxels.xml"


@dataclass(frozen=True, slots=True)
class TaxelGridSpec:
    """描述 pad 局部坐标系中的规则 taxel 网格。

    Attributes:
        x: taxel 中心在 pad 法向上的坐标 (m)。
        col_pitch: 相邻列的 y 方向间距 (m)。
        row_pitch: 相邻行的 z 方向间距 (m)。
        rows: 网格行数，必须为奇数。
        cols: 网格列数。
    """

    x: float
    col_pitch: float
    row_pitch: float
    rows: int
    cols: int

    @property
    def first_col_y(self) -> float:
        """返回关于 y=0 居中时第一列的中心坐标。"""
        return -0.5 * (self.cols - 1) * self.col_pitch


TAXEL_GRID = TaxelGridSpec(x=0.0453, col_pitch=0.007, row_pitch=0.007, rows=3, cols=3)
PAD_TOP_EDGE_Z = 0.148125
MIDDLE_ROW_TO_TOP_EDGE = 0.012
TAXEL_RADIUS = 0.0028
TAXEL_SITE_QUAT = "1 0 -1 0"


def _format_xyz(x: float, y: float, z: float) -> str:
    """将三维坐标格式化为 MJCF 属性值。"""
    return f"{x:.4f} {y:.4f} {z:.4f}"


def _iter_taxel_layout(grid: TaxelGridSpec) -> Iterable[tuple[str, str]]:
    """按行优先顺序生成 taxel 名称索引与局部坐标。"""
    if grid.rows % 2 != 1:
        raise ValueError("taxel 网格行数必须为奇数。")
    first_row_z = PAD_TOP_EDGE_Z - MIDDLE_ROW_TO_TOP_EDGE - (grid.rows // 2) * grid.row_pitch
    for row in range(grid.rows):
        for col in range(grid.cols):
            yield (
                f"{row}{col}",
                _format_xyz(
                    grid.x,
                    grid.first_col_y + col * grid.col_pitch,
                    first_row_z + row * grid.row_pitch,
                ),
            )


def _find_pad_body(root: ET.Element, side: str) -> ET.Element:
    """查找指定侧的 pad body，不存在时抛出异常。"""
    pad = root.find(f".//body[@name='{side}_pad']")
    if pad is None:
        raise ValueError(f"基础模型缺少 {side}_pad body。")
    return pad


def _append_taxels(pad: ET.Element, side: str) -> None:
    """在单侧 pad 下添加球形接触体、力传感 site 与 pad 力矩 site。"""
    color = "0 1 0 0.45" if side == "left" else "0 0 1 0.45"
    for index, pos in _iter_taxel_layout(TAXEL_GRID):
        body = ET.SubElement(pad, "body", name=f"{side}_taxel_body_{index}", pos=pos)
        ET.SubElement(
            body,
            "geom",
            name=f"{side}_taxel_geom_{index}",
            type="sphere",
            size=f"{TAXEL_RADIUS:.4f}",
            mass="1e-6",
            friction="0.7 0.03 0.01",
            solimp="0.95 0.99 0.001",
            solref="0.004 1",
            priority="2",
            rgba=color,
        )
        ET.SubElement(
            body,
            "site",
            name=f"{side}_taxel_site_{index}",
            size="0.0008",
            quat=TAXEL_SITE_QUAT,
            rgba=color,
        )
    ET.SubElement(
        pad,
        "site",
        name=f"{side}_pad_ft_site",
        pos=_format_xyz(0.043258, 0, PAD_TOP_EDGE_Z - MIDDLE_ROW_TO_TOP_EDGE),
        quat=TAXEL_SITE_QUAT,
        size="0.003",
        rgba="1 0 0 0.5",
    )


def build_taxel_tree(base_xml: Path) -> ET.ElementTree:
    """从基础 MJCF 构建含 18 个法向力 taxel 的 XML 树。

    Args:
        base_xml: 未添加触觉结构的 Robotiq 2F-85 MJCF 文件。

    Returns:
        添加了左右 pad taxel 和传感器定义的 MJCF 树。
    """
    tree = ET.parse(base_xml)
    root = tree.getroot()
    root.set("model", "robotiq_2f85_taxels")
    size = root.find("size")
    if size is None:
        option = root.find("option")
        if option is None:
            raise ValueError("基础模型缺少 option 节点。")
        root.insert(list(root).index(option) + 1, ET.Element("size"))
        size = root.find("size")
    assert size is not None
    size.set("njmax", "128")
    size.set("nconmax", "256")

    sensor = ET.Element("sensor")
    for side in ("left", "right"):
        _append_taxels(_find_pad_body(root, side), side)
        for index, _ in _iter_taxel_layout(TAXEL_GRID):
            ET.SubElement(
                sensor,
                "force",
                name=f"{side}_taxel_force_{index}",
                site=f"{side}_taxel_site_{index}",
            )
        ET.SubElement(sensor, "force", name=f"{side}_pad_force", site=f"{side}_pad_ft_site")
        ET.SubElement(sensor, "torque", name=f"{side}_pad_torque", site=f"{side}_pad_ft_site")
    root.append(sensor)
    return tree


def main() -> None:
    """生成 XML，并输出生成文件位置。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-xml", type=Path, default=DEFAULT_BASE_XML)
    parser.add_argument("--output-xml", type=Path, default=DEFAULT_OUTPUT_XML)
    args = parser.parse_args()
    tree = build_taxel_tree(args.base_xml)
    ET.indent(tree, space="  ")
    args.output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output_xml, encoding="utf-8", xml_declaration=True)
    print(f"已生成 {args.output_xml}")


if __name__ == "__main__":
    main()

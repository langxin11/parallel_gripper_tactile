"""从基础 Robotiq 2F-85 MJCF 生成带离散触觉 taxel 的派生资产。

常见用法::

    uv run scripts/generate_taxels_xml.py
    uv run scripts/generate_taxels_xml.py --shape box
    uv run scripts/generate_taxels_xml.py --output-xml /tmp/2f85_taxels.xml
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ROBOTIQ_ASSET_DIR = REPOSITORY_ROOT / "assets" / "grippers" / "robotiq_2f85"
DEFAULT_BASE_XML = ROBOTIQ_ASSET_DIR / "2f85.xml"
DEFAULT_OUTPUT_XML = ROBOTIQ_ASSET_DIR / "2f85_taxels.xml"
DEFAULT_BOX_OUTPUT_XML = ROBOTIQ_ASSET_DIR / "2f85_taxels_box.xml"


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
PAD_X = 0.043258
PAD_HALF_X = 0.002
PAD_HALF_Y = 0.011
PAD_MIN_Z = 0.110625
PAD_MAX_Z = 0.148125
BOX_ROWS = 3
BOX_COLS = 3
# 该参数组实现的是“软化的刚体接触”：允许约毫米级的接触过渡，
# 但不表示硅胶几何发生真实形变。
SOFT_CONTACT_KWARGS = {
    "solimp": "0.90 0.95 0.002",
    "solref": "0.015 1",
}
# 公平比较用 box taxel 与参考 touch_grid 使用相同的平面、摩擦和接触参数。
BOX_CONTACT_KWARGS = {
    "mass": "0",
    "friction": "0.7",
    "solimp": "0.95 0.99 0.001",
    "solref": "0.004 1",
    "priority": "1",
}


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


def _append_sphere_taxels(pad: ET.Element, side: str) -> None:
    """在单侧 pad 下添加球形接触体、力传感 site 与 pad 力矩 site。"""
    color = "0 1 0 0.45" if side == "left" else "0 0 1 0.45"
    for index, pos in _iter_taxel_layout(TAXEL_GRID):
        # 独立 taxel 子 body 让 force sensor 测得它传递给 pad 父 body 的
        # 相互作用力，而非 pad 整体外力；向量在下方 site 局部系中表达。
        body = ET.SubElement(pad, "body", name=f"{side}_taxel_body_{index}", pos=pos)
        ET.SubElement(
            body,
            "geom",
            name=f"{side}_taxel_geom_{index}",
            type="sphere",
            size=f"{TAXEL_RADIUS:.4f}",
            mass="1e-6",
            friction="0.7 0.03 0.01",
            priority="2",
            rgba=color,
            **SOFT_CONTACT_KWARGS,
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


def _append_box_taxels(pad: ET.Element, side: str) -> None:
    """用与 3×3 touch_grid 完全相同的平面网格替换单侧 pad。"""
    for geom in list(pad.findall("geom")):
        pad.remove(geom)
    color = "0 1 0 0.45" if side == "left" else "0 0 1 0.45"
    cell_half_y = PAD_HALF_Y / BOX_COLS
    cell_half_z = (PAD_MAX_Z - PAD_MIN_Z) / (2 * BOX_ROWS)
    for row in range(BOX_ROWS):
        z = PAD_MIN_Z + (2 * row + 1) * cell_half_z
        for col in range(BOX_COLS):
            y = -PAD_HALF_Y + (2 * col + 1) * cell_half_y
            index = f"{row}{col}"
            body = ET.SubElement(
                pad,
                "body",
                name=f"{side}_taxel_body_{index}",
                pos=f"{PAD_X:.6f} {y:.6f} {z:.6f}",
            )
            ET.SubElement(
                body,
                "geom",
                name=f"{side}_taxel_geom_{index}",
                type="box",
                size=f"{PAD_HALF_X:.6f} {cell_half_y:.6f} {cell_half_z:.6f}",
                rgba=color,
                **BOX_CONTACT_KWARGS,
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
        pos=_format_xyz(PAD_X, 0, (PAD_MIN_Z + PAD_MAX_Z) / 2),
        quat=TAXEL_SITE_QUAT,
        size="0.003",
        rgba="1 0 0 0.5",
    )


def build_taxel_tree(base_xml: Path, shape: str = "sphere") -> ET.ElementTree:
    """从基础 MJCF 构建含 18 个法向力 taxel 的 XML 树。

    Args:
        base_xml: 未添加触觉结构的 Robotiq 2F-85 MJCF 文件。
        shape: taxel 几何形状，``sphere`` 或 ``box``。

    Returns:
        添加了左右 pad taxel 和传感器定义的 MJCF 树。
    """
    if shape not in {"sphere", "box"}:
        raise ValueError("taxel shape 必须是 sphere 或 box。")
    tree = ET.parse(base_xml)
    root = tree.getroot()
    root.set("model", "robotiq_2f85_taxels" if shape == "sphere" else "robotiq_2f85_box_taxels")
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
        pad = _find_pad_body(root, side)
        if shape == "sphere":
            _append_sphere_taxels(pad, side)
            indices = [index for index, _ in _iter_taxel_layout(TAXEL_GRID)]
        else:
            _append_box_taxels(pad, side)
            indices = [f"{row}{col}" for row in range(BOX_ROWS) for col in range(BOX_COLS)]
        for index in indices:
            # 输出方向是 taxel 子 body -> pad 父 body；本资产的 site +Z 指向
            # 表面外侧，因此压缩载荷的原始 z 分量为负，正压力应取 -Fz。
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
    parser.add_argument("--shape", choices=("sphere", "box"), default="sphere")
    parser.add_argument("--output-xml", type=Path)
    args = parser.parse_args()
    output_xml = args.output_xml or (
        DEFAULT_BOX_OUTPUT_XML if args.shape == "box" else DEFAULT_OUTPUT_XML
    )
    tree = build_taxel_tree(args.base_xml, args.shape)
    ET.indent(tree, space="  ")
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    print(f"已生成 {output_xml}")


if __name__ == "__main__":
    main()

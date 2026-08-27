"""用于夹爪 MJCF 资源的可复用生成器与准备辅助函数。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Literal
import xml.etree.ElementTree as ET

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ROBOTIQ_ASSET_DIR = REPOSITORY_ROOT / "assets" / "grippers" / "robotiq_2f85"
DEFAULT_BASE_XML = ROBOTIQ_ASSET_DIR / "2f85.xml"
DEFAULT_TAXEL_OUTPUT_XML = ROBOTIQ_ASSET_DIR / "2f85_taxels.xml"
DEFAULT_BOX_TAXEL_OUTPUT_XML = ROBOTIQ_ASSET_DIR / "2f85_taxels_box.xml"
DEFAULT_TOUCH_GRID_OUTPUT_XML = ROBOTIQ_ASSET_DIR / "2f85_touch_grid.xml"


@dataclass(frozen=True, slots=True)
class TaxelGridSpec:
    """在垫局部坐标系中描述规则的 taxel 网格。"""

    x: float
    col_pitch: float
    row_pitch: float
    rows: int
    cols: int

    @property
    def first_col_y(self) -> float:
        """返回以 ``y=0`` 为中心的网格的第一列中心。"""
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
SOFT_CONTACT_KWARGS = {"solimp": "0.90 0.95 0.002", "solref": "0.015 1"}
REFERENCE_CONTACT_KWARGS = {
    "mass": "0",
    "friction": "0.7",
    "solimp": "0.95 0.99 0.001",
    "solref": "0.004 1",
    "priority": "1",
}
TOUCH_SITE_TO_SURFACE_DISTANCE = 0.0426
TOUCH_SITE_X = PAD_X + PAD_HALF_X - TOUCH_SITE_TO_SURFACE_DISTANCE
TOUCH_GRID_FOV_DEGREES = "24 15"
DEFAULT_GRID_ROWS = 32
DEFAULT_GRID_COLS = 32


def _format_xyz(x: float, y: float, z: float) -> str:
    """为 MJCF 属性格式化一个三维位置。"""
    return f"{x:.4f} {y:.4f} {z:.4f}"


def _iter_taxel_layout(grid: TaxelGridSpec) -> Iterable[tuple[str, str]]:
    """产出按行优先排列的 taxel 名称与局部位置。"""
    if grid.rows % 2 != 1:
        raise ValueError("taxel grid row count must be odd")
    first_row_z = PAD_TOP_EDGE_Z - MIDDLE_ROW_TO_TOP_EDGE - (grid.rows // 2) * grid.row_pitch
    for row in range(grid.rows):
        for column in range(grid.cols):
            yield (
                f"{row}{column}",
                _format_xyz(
                    grid.x,
                    grid.first_col_y + column * grid.col_pitch,
                    first_row_z + row * grid.row_pitch,
                ),
            )


def _find_pad_body(root: ET.Element, side: str) -> ET.Element:
    """查找一个具名的指尖垫 body。"""
    pad = root.find(f".//body[@name='{side}_pad']")
    if pad is None:
        raise ValueError(f"base model is missing {side}_pad body")
    return pad


def _append_sphere_taxels(pad: ET.Element, side: str) -> None:
    """向一个垫添加独立的球形力感测 taxel。"""
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
        pos=_format_xyz(PAD_X, 0, PAD_TOP_EDGE_Z - MIDDLE_ROW_TO_TOP_EDGE),
        quat=TAXEL_SITE_QUAT,
        size="0.003",
        rgba="1 0 0 0.5",
    )


def _append_box_taxels(pad: ET.Element, side: str) -> None:
    """将一个垫几何体替换为用于比较的 3x3 taxel 盒。"""
    for geom in list(pad.findall("geom")):
        pad.remove(geom)
    color = "0 1 0 0.45" if side == "left" else "0 0 1 0.45"
    cell_half_y = PAD_HALF_Y / BOX_COLS
    cell_half_z = (PAD_MAX_Z - PAD_MIN_Z) / (2 * BOX_ROWS)
    for row in range(BOX_ROWS):
        z = PAD_MIN_Z + (2 * row + 1) * cell_half_z
        for column in range(BOX_COLS):
            y = -PAD_HALF_Y + (2 * column + 1) * cell_half_y
            index = f"{row}{column}"
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
                **REFERENCE_CONTACT_KWARGS,
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


def build_taxel_tree(base_xml: Path, shape: Literal["sphere", "box"] = "sphere") -> ET.ElementTree:
    """构建一个包含两个 3x3 力 taxel 网格的 Robotiq 模型。"""
    if shape not in {"sphere", "box"}:
        raise ValueError("taxel shape must be sphere or box")
    tree = ET.parse(base_xml)
    root = tree.getroot()
    root.set("model", "robotiq_2f85_taxels" if shape == "sphere" else "robotiq_2f85_box_taxels")
    size = root.find("size")
    if size is None:
        option = root.find("option")
        if option is None:
            raise ValueError("base model is missing option element")
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
            indices = [f"{row}{column}" for row in range(BOX_ROWS) for column in range(BOX_COLS)]
        for index in indices:
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


def build_touch_grid_tree(
    base_xml: Path, rows: int = DEFAULT_GRID_ROWS, cols: int = DEFAULT_GRID_COLS
) -> ET.ElementTree:
    """构建一个带 MuJoCo 三通道 ``touch_grid`` 传感器的 Robotiq 模型。"""
    if rows <= 0 or cols <= 0:
        raise ValueError("touch_grid rows and columns must be positive")
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
    size.attrib.pop("njmax", None)
    size.attrib.pop("nconmax", None)
    size.set("memory", "128M")

    sensor = ET.Element("sensor")
    for side in ("left", "right"):
        pad = _find_pad_body(root, side)
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
        ET.SubElement(plugin, "config", key="fov", value=TOUCH_GRID_FOV_DEGREES)
        ET.SubElement(plugin, "config", key="gamma", value="0")
        ET.SubElement(plugin, "config", key="nchannel", value="3")
    root.append(sensor)
    return tree


def _append_touch_grid(pad: ET.Element, side: str, rows: int, cols: int) -> None:
    """向一个垫添加碰撞单元以及后置的 touch-grid site。"""
    for geom in list(pad.findall("geom")):
        pad.remove(geom)
    cell_half_y = PAD_HALF_Y / cols
    cell_half_z = (PAD_MAX_Z - PAD_MIN_Z) / (2 * rows)
    for row in range(rows):
        z = PAD_MIN_Z + (2 * row + 1) * cell_half_z
        for column in range(cols):
            y = -PAD_HALF_Y + (2 * column + 1) * cell_half_y
            ET.SubElement(
                pad,
                "geom",
                name=f"{side}_touch_cell_{row:02d}_{column:02d}",
                type="box",
                pos=f"{PAD_X:.6f} {y:.6f} {z:.6f}",
                size=f"{PAD_HALF_X:.6f} {cell_half_y:.6f} {cell_half_z:.6f}",
                rgba="0.2 0.2 0.2 1",
                **REFERENCE_CONTACT_KWARGS,
            )
    ET.SubElement(
        pad,
        "site",
        name=f"touch_{side}",
        pos=f"{TOUCH_SITE_X:.6f} 0 {(PAD_MIN_Z + PAD_MAX_Z) / 2:.6f}",
        quat=TAXEL_SITE_QUAT,
        size="0.001",
        rgba="0 0 0 0",
    )


def write_tree(tree: ET.ElementTree, output_xml: Path) -> Path:
    """写入一个带缩进的 XML 树，仅创建所请求的父目录。"""
    ET.indent(tree, space="  ")
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def _body_with_joint(root: ET.Element, joint_name: str) -> ET.Element:
    """返回直接包含 ``joint_name`` 的 body。"""
    for body in root.findall(".//body"):
        if body.find(f"./joint[@name='{joint_name}']") is not None:
            return body
    raise ValueError(f"cannot find a body containing joint {joint_name!r}")


def _nearest_site_assignment(
    geom_positions: np.ndarray, site_positions: np.ndarray
) -> tuple[int, ...]:
    """返回精确的最小距离一对一 geom 到 site 的分配。"""
    if geom_positions.shape != site_positions.shape:
        raise ValueError("geom and site position arrays must have the same shape")
    count = len(geom_positions)
    costs = np.sum((geom_positions[:, None, :] - site_positions[None, :, :]) ** 2, axis=2)
    states: dict[int, tuple[float, tuple[int, ...]]] = {0: (0.0, ())}
    for geom_index in range(count):
        next_states: dict[int, tuple[float, tuple[int, ...]]] = {}
        for mask, (cost, assignment) in states.items():
            for site_index in range(count):
                if mask & (1 << site_index):
                    continue
                next_mask = mask | (1 << site_index)
                candidate = (
                    cost + float(costs[geom_index, site_index]),
                    assignment + (site_index,),
                )
                if next_mask not in next_states or candidate[0] < next_states[next_mask][0]:
                    next_states[next_mask] = candidate
        states = next_states
    return states[(1 << count) - 1][1]


def label_taxel_geoms(
    input_xml: Path,
    output_xml: Path,
    *,
    left_joint: str = "left_finger_slide",
    right_joint: str = "right_finger_slide",
    rows: int = 3,
    cols: int = 3,
    row_axis: int = 1,
    col_axis: int = 0,
) -> None:
    """写出一个带稳定按行优先 taxel 碰撞名称的 Onshape 导出。"""
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("install project dependencies with `uv sync`") from error

    tree = ET.parse(input_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF is missing worldbody")
    xml_geoms = list(worldbody.iter("geom"))
    model = mujoco.MjModel.from_xml_path(str(input_xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if len(xml_geoms) != model.ngeom:
        raise ValueError("XML geom order does not match the compiled model")

    expected = rows * cols
    for side, joint_name in (("left", left_joint), ("right", right_joint)):
        body = _body_with_joint(root, joint_name)
        body_name = body.get("name")
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        geom_ids = [
            geom_id
            for geom_id in range(model.ngeom)
            if int(model.geom_bodyid[geom_id]) == body_id
            and (int(model.geom_contype[geom_id]) or int(model.geom_conaffinity[geom_id]))
        ]
        if len(geom_ids) != expected:
            raise ValueError(
                f"{side} fingertip has {len(geom_ids)} active collision geoms; expected {expected}"
            )
        site_names = [
            f"{side}_taxel_{row}{column}" for row in range(rows) for column in range(cols)
        ]
        site_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in site_names]
        if all(site_id >= 0 for site_id in site_ids):
            assignments = _nearest_site_assignment(
                data.geom_xpos[geom_ids], data.site_xpos[site_ids]
            )
            for geom_id, site_index in zip(geom_ids, assignments, strict=True):
                row, column = divmod(site_index, cols)
                xml_geoms[geom_id].set("name", f"{side}_taxel_geom_{row}{column}")
            continue
        geom_ids.sort(
            key=lambda geom_id: (
                float(model.geom_pos[geom_id, row_axis]),
                float(model.geom_pos[geom_id, col_axis]),
            )
        )
        for index, geom_id in enumerate(geom_ids):
            row, column = divmod(index, cols)
            xml_geoms[geom_id].set("name", f"{side}_taxel_geom_{row}{column}")

    if output_xml.resolve().parent != input_xml.resolve().parent:
        compiler = root.find("compiler")
        if compiler is not None and (meshdir := compiler.get("meshdir")):
            compiler.set("meshdir", str((input_xml.parent / meshdir).resolve()))
    write_tree(tree, output_xml)


__all__ = [
    "BOX_COLS",
    "BOX_ROWS",
    "DEFAULT_BASE_XML",
    "DEFAULT_BOX_TAXEL_OUTPUT_XML",
    "DEFAULT_GRID_COLS",
    "DEFAULT_GRID_ROWS",
    "DEFAULT_TAXEL_OUTPUT_XML",
    "DEFAULT_TOUCH_GRID_OUTPUT_XML",
    "REFERENCE_CONTACT_KWARGS",
    "TAXEL_GRID",
    "TOUCH_GRID_FOV_DEGREES",
    "TOUCH_SITE_X",
    "TaxelGridSpec",
    "build_taxel_tree",
    "build_touch_grid_tree",
    "label_taxel_geoms",
    "write_tree",
]

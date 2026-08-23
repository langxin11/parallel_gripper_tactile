"""为新的 Onshape MJCF 导出分配稳定的行优先 taxel geom 名称。

导出器保留了 Pillars 网格，但没有给它们的碰撞 geom 稳定的实例名。当存在
命名为 ``frame_*_taxel_*`` 的 site 时，本工具把每个碰撞 geom 匹配到最近的
site，并写入 ``left/right_taxel_geom_RC``。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _body_with_joint(root: ET.Element, joint_name: str) -> ET.Element:
    """返回包含指定关节的 body。"""
    for body in root.findall(".//body"):
        if body.find(f"./joint[@name='{joint_name}']") is not None:
            return body
    raise ValueError(f"cannot find a body containing joint {joint_name!r}")


def _nearest_site_assignment(
    geom_positions: np.ndarray, site_positions: np.ndarray
) -> tuple[int, ...]:
    """返回 taxel geom 到 site 的最小距离一一对应分配。

    用位掩码动态规划求精确解；规模为单侧 9 个 taxel，2^9 个状态可接受。
    """
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
    """写出两侧 Pillars 阵列都带稳定名称的整理版 MJCF。"""
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

        site_names = [f"{side}_taxel_{row}{col}" for row in range(rows) for col in range(cols)]
        site_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in site_names]
        if all(site_id >= 0 for site_id in site_ids):
            assignments = _nearest_site_assignment(
                data.geom_xpos[geom_ids], data.site_xpos[site_ids]
            )
            for geom_id, site_index in zip(geom_ids, assignments, strict=True):
                row, col = divmod(site_index, cols)
                xml_geoms[geom_id].set("name", f"{side}_taxel_geom_{row}{col}")
            continue

        geom_ids.sort(
            key=lambda geom_id: (
                float(model.geom_pos[geom_id, row_axis]),
                float(model.geom_pos[geom_id, col_axis]),
            )
        )
        for index, geom_id in enumerate(geom_ids):
            row, col = divmod(index, cols)
            xml_geoms[geom_id].set("name", f"{side}_taxel_geom_{row}{col}")

    if output_xml.resolve().parent != input_xml.resolve().parent:
        compiler = root.find("compiler")
        if compiler is not None and (meshdir := compiler.get("meshdir")):
            compiler.set("meshdir", str((input_xml.parent / meshdir).resolve()))

    ET.indent(tree, space="  ")
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)


def main() -> None:
    """解析输入/输出路径并执行 taxel geom 命名整理。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_xml", type=Path)
    parser.add_argument("output_xml", type=Path)
    parser.add_argument("--left-joint", default="left_finger_slide")
    parser.add_argument("--right-joint", default="right_finger_slide")
    args = parser.parse_args()
    label_taxel_geoms(
        args.input_xml,
        args.output_xml,
        left_joint=args.left_joint,
        right_joint=args.right_joint,
    )
    print(f"prepared {args.output_xml}")


if __name__ == "__main__":
    main()

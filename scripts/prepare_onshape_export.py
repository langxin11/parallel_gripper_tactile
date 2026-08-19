"""Assign stable row-major taxel geom names to a fresh Onshape MJCF export.

The exporter preserves the Pillars meshes but does not give their collision geoms
stable instance names. This tool compiles the model, sorts each fingertip's active
collision geoms by body-local position, and writes ``left/right_taxel_geom_RC``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def _body_with_joint(root: ET.Element, joint_name: str) -> ET.Element:
    for body in root.findall(".//body"):
        if body.find(f"./joint[@name='{joint_name}']") is not None:
            return body
    raise ValueError(f"cannot find a body containing joint {joint_name!r}")


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
    """Write a curated MJCF with stable names for both Pillars arrays."""
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

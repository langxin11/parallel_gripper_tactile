"""从具名 Pillar mesh 生成保留触点高度差的球体碰撞模型。"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


SPHERE_RADIUS_M = 0.0028


def _site_name(geom_name: str) -> str:
    """返回碰撞 geom 对应的触觉 site 名称。"""
    return geom_name.replace("_geom_", "_", 1)


def _coordinates(element: ET.Element, *, description: str) -> list[str]:
    """读取并校验元素的三维位置。"""
    position = element.get("pos")
    if position is None:
        raise ValueError(f"{description} has no position")
    coordinates = position.split()
    if len(coordinates) != 3:
        raise ValueError(f"invalid {description} position {position!r}")
    return coordinates


def make_height_sphere_model(source: Path, output: Path) -> None:
    """生成保留各 taxel site 高度的球体碰撞模型。"""
    tree = ET.parse(source)
    root = tree.getroot()
    sites = {
        site.get("name"): site for site in root.findall(".//site") if site.get("name") is not None
    }
    geoms = [
        geom
        for geom in root.findall(".//geom[@class='pillar_collision']")
        if geom.get("name") is not None
    ]
    if len(geoms) != 18:
        raise ValueError(f"expected 18 named Pillar collision geoms, found {len(geoms)}")

    for geom in geoms:
        name = geom.get("name")
        assert name is not None
        site = sites.get(_site_name(name))
        if site is None:
            raise ValueError(f"missing tactile site for collision geom {name!r}")
        coordinates = _coordinates(site, description=f"tactile site {site.get('name')!r}")
        for attribute in ("mesh", "material", "quat"):
            geom.attrib.pop(attribute, None)
        geom.set("type", "sphere")
        geom.set("pos", " ".join(coordinates))
        geom.set("size", f"{SPHERE_RADIUS_M:.8g}")

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main() -> None:
    """解析路径并生成高度球体碰撞模型。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    make_height_sphere_model(arguments.source, arguments.output)
    print(arguments.output)


if __name__ == "__main__":
    main()

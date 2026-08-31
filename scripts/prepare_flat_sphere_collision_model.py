"""Generate controlled MJCF variants of the Pillar collision geometry.

The source model keeps its visual Pillar meshes.  Collision geoms can be
changed to height-preserving spheres, coplanar spheres, or vertically shifted
coplanar meshes.  These variants independently probe height variation and
mesh contact-manifold effects without changing the gripper kinematics.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Literal
import xml.etree.ElementTree as ET


SPHERE_RADIUS_M = 0.0028
CollisionVariant = Literal["flat-sphere", "height-sphere", "coplanar-mesh"]


def _site_name(geom_name: str) -> str:
    return geom_name.replace("_geom_", "_", 1)


def _coordinates(element: ET.Element, *, description: str) -> list[str]:
    """Return one element's validated three-dimensional position."""
    position = element.get("pos")
    if position is None:
        raise ValueError(f"{description} has no position")
    coordinates = position.split()
    if len(coordinates) != 3:
        raise ValueError(f"invalid {description} position {position!r}")
    return coordinates


def make_collision_variant_model(source: Path, output: Path, *, variant: CollisionVariant) -> None:
    """Write one controlled Pillar collision variant to ``output``."""
    if variant not in {"flat-sphere", "height-sphere", "coplanar-mesh"}:
        raise ValueError(f"unknown collision variant: {variant}")
    tree = ET.parse(source)
    root = tree.getroot()
    sites = {
        site.get("name"): site for site in root.findall(".//site") if site.get("name") is not None
    }
    geoms = root.findall(".//geom[@class='pillar_collision']")
    named_geoms = [geom for geom in geoms if geom.get("name") is not None]
    if len(named_geoms) != 18:
        raise ValueError(f"expected 18 named Pillar collision geoms, found {len(named_geoms)}")
    site_z_by_geom: dict[str, float] = {}
    for geom in named_geoms:
        name = geom.get("name")
        assert name is not None
        site = sites.get(_site_name(name))
        if site is None:
            raise ValueError(f"missing tactile site for collision geom {name!r}")
        site_z_by_geom[name] = float(
            _coordinates(site, description=f"tactile site {site.get('name')!r}")[2]
        )
    common_center_z = max(site_z_by_geom.values())

    for geom in named_geoms:
        name = geom.get("name")
        assert name is not None
        site = sites[_site_name(name)]
        site_coordinates = _coordinates(site, description=f"tactile site {_site_name(name)!r}")
        if variant in {"flat-sphere", "height-sphere"}:
            center_z = common_center_z if variant == "flat-sphere" else site_z_by_geom[name]
            site_coordinates[2] = f"{center_z:.8g}"
            for attribute in ("mesh", "material", "quat"):
                geom.attrib.pop(attribute, None)
            geom.set("type", "sphere")
            geom.set("pos", " ".join(site_coordinates))
            geom.set("size", f"{SPHERE_RADIUS_M:.8g}")
        else:
            geom_coordinates = _coordinates(geom, description=f"collision geom {name!r}")
            shift_z = common_center_z - site_z_by_geom[name]
            geom_coordinates[2] = f"{float(geom_coordinates[2]) + shift_z:.8g}"
            geom.set("pos", " ".join(geom_coordinates))

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)


def make_flat_sphere_model(source: Path, output: Path) -> None:
    """Write the original coplanar-sphere A/B model."""
    make_collision_variant_model(source, output, variant="flat-sphere")


def main() -> None:
    """Parse paths and generate the flat-sphere collision model."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--variant",
        choices=("flat-sphere", "height-sphere", "coplanar-mesh"),
        default="flat-sphere",
    )
    arguments = parser.parse_args()
    make_collision_variant_model(arguments.source, arguments.output, variant=arguments.variant)
    print(arguments.output)


if __name__ == "__main__":
    main()

"""Generate an A/B MJCF model with coplanar spherical Pillar colliders.

The source model keeps its visual Pillar meshes.  Only ``pillar_collision``
geometries are replaced so that all 3x3 contact tips on each finger have a
common surface plane.  This isolates the effect of the original 0.5 mm
convex-array height variation without changing the gripper kinematics.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


SPHERE_RADIUS_M = 0.0028
# In each finger's local frame, +Z points from the backing plate toward the
# gripped object.  The original central mesh tip is at z=0.03625 m, so a
# sphere centered at 0.03345 m with 2.8 mm radius preserves that reference
# contact plane for every taxel.
CONTACT_SURFACE_Z_M = 0.03625
SPHERE_CENTER_Z_M = CONTACT_SURFACE_Z_M - SPHERE_RADIUS_M


def _site_name(geom_name: str) -> str:
    return geom_name.replace("_geom_", "_", 1)


def make_flat_sphere_model(source: Path, output: Path) -> None:
    """Write ``output`` with named Pillar collision meshes replaced by spheres."""
    tree = ET.parse(source)
    root = tree.getroot()
    sites = {
        site.get("name"): site for site in root.findall(".//site") if site.get("name") is not None
    }
    replaced = 0
    for geom in root.findall(".//geom[@class='pillar_collision']"):
        name = geom.get("name")
        if name is None:
            continue
        site = sites.get(_site_name(name))
        if site is None:
            raise ValueError(f"missing tactile site for collision geom {name!r}")
        position = site.get("pos")
        if position is None:
            raise ValueError(f"tactile site {site.get('name')!r} has no position")
        coordinates = position.split()
        if len(coordinates) != 3:
            raise ValueError(f"invalid tactile-site position {position!r}")
        coordinates[2] = f"{SPHERE_CENTER_Z_M:.8g}"
        for attribute in ("mesh", "material", "quat"):
            geom.attrib.pop(attribute, None)
        geom.set("type", "sphere")
        geom.set("pos", " ".join(coordinates))
        geom.set("size", f"{SPHERE_RADIUS_M:.8g}")
        replaced += 1
    if replaced != 18:
        raise ValueError(f"expected 18 Pillar collision geoms, replaced {replaced}")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main() -> None:
    """Parse paths and generate the flat-sphere collision model."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    make_flat_sphere_model(arguments.source, arguments.output)
    print(arguments.output)


if __name__ == "__main__":
    main()

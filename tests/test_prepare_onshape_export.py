"""验证 Onshape 导出的 taxel geom 命名整理工具。"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]


def _load_script():
    path = ROOT / "scripts" / "prepare_onshape_export.py"
    spec = spec_from_file_location("prepare_onshape_export", path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_prepare_onshape_export_assigns_stable_taxel_names(tmp_path: Path) -> None:
    """后处理脚本为 Pillars geom 分配稳定的 taxel 命名。"""
    module = _load_script()
    source = ROOT / "assets/grippers/custom_parallel_gripper/parallel_gripper.xml"
    output = tmp_path / "prepared.xml"
    module.label_taxel_geoms(source, output)

    model = mujoco.MjModel.from_xml_path(str(output))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for side in ("left", "right"):
        for row in range(3):
            for col in range(3):
                name = f"{side}_taxel_geom_{row}{col}"
                geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                site_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_taxel_{row}{col}"
                )
                assert geom_id >= 0
                assert site_id >= 0
                distances = [
                    sum((data.geom_xpos[geom_id] - data.site_xpos[other_site_id]) ** 2)
                    for other_site_id in range(model.nsite)
                    if (
                        other_name := mujoco.mj_id2name(
                            model, mujoco.mjtObj.mjOBJ_SITE, other_site_id
                        )
                    )
                    and other_name.startswith(f"{side}_taxel_")
                ]
                own_distance = sum((data.geom_xpos[geom_id] - data.site_xpos[site_id]) ** 2)
                assert own_distance == min(distances)

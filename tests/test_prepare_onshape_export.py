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
    module = _load_script()
    source = ROOT / "assets/grippers/custom_parallel_gripper/parallel_gripper.xml"
    output = tmp_path / "prepared.xml"
    module.label_taxel_geoms(source, output)

    model = mujoco.MjModel.from_xml_path(str(output))
    for side in ("left", "right"):
        for row in range(3):
            for col in range(3):
                name = f"{side}_taxel_geom_{row}{col}"
                assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0

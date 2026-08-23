"""验证接触力读取器的符号约定、命名解析与切向剪切映射。"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from parallel_gripper_tactile import ContactTaxelReader, load_profile


ROOT = Path(__file__).resolve().parents[1]


def _load_custom_demo():
    path = ROOT / "scripts" / "run_custom_gripper_tactile_demo.py"
    spec = spec_from_file_location("run_custom_gripper_tactile_demo", path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_contact_force_sign_depends_on_contact_geom_order() -> None:
    """geom1/geom2 两种接触顺序下作用力方向一致。"""
    contact = SimpleNamespace(geom1=3, geom2=7, frame=np.eye(3).ravel())
    contact_force = np.array([2.0, -3.0, 4.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(
        ContactTaxelReader._force_on_geom_world(contact, contact_force, 7), contact_force[:3]
    )
    np.testing.assert_allclose(
        ContactTaxelReader._force_on_geom_world(contact, contact_force, 3), -contact_force[:3]
    )


def test_custom_gripper_reader_resolves_all_taxels_and_reports_zero_without_contact() -> None:
    """全部 taxel 命名可解析且无接触时输出全零网格。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.toml")
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    frame = ContactTaxelReader.from_profile(model, profile).read(data)

    assert frame.left.shape == (3, 3, 3)
    assert frame.right.shape == (3, 3, 3)
    np.testing.assert_array_equal(frame.left, np.zeros((3, 3, 3)))
    np.testing.assert_array_equal(frame.right, np.zeros((3, 3, 3)))


def test_prescribed_world_shear_maps_to_expected_local_components() -> None:
    """预设世界系剪切正确映射为左右指尖局部 Fx/Fy。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.toml")
    demo = _load_custom_demo()

    def read_after_shear(axis: str):
        model = demo.build_demo_model(profile, ("left", 1, 1), 0.003, shear_axis=axis)
        data = mujoco.MjData(model)
        reader = ContactTaxelReader.from_profile(model, profile)
        drive_id = model.actuator(profile.actuator).id
        mocap_id = int(model.body_mocapid[model.body(demo.CUBE_MOCAP).id])
        initial_position = data.mocap_pos[mocap_id].copy()
        for step in range(1000):
            data.ctrl[drive_id] = profile.closed_control * min(1.0, step / 333)
            progress = min(1.0, max(0.0, (step - 500) / 250))
            data.mocap_pos[mocap_id] = initial_position
            data.mocap_pos[mocap_id, 1 if axis == "y" else 2] += 0.002 * progress
            mujoco.mj_step(model, data)
        return reader.read(data)

    y_shear = read_after_shear("y")
    assert y_shear.left[1].sum() < -1e-3
    assert y_shear.right[1].sum() > 1e-3

    z_shear = read_after_shear("z")
    assert z_shear.left[0].sum() < -1e-3
    assert z_shear.right[0].sum() < -1e-3

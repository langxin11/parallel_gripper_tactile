"""验证自研夹爪固定基座抓取场景与验收脚本。"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import mujoco
import numpy as np

from parallel_gripper_tactile import load_profile


ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, filename: str):
    path = ROOT / "scripts" / filename
    spec = spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_custom_scene_fixes_reserved_base_and_keeps_free_cube() -> None:
    """场景固定预留 base 自由关节且方块保持自由。"""
    scene = _load_module("custom_grasp_scene", "custom_grasp_scene.py")
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.toml")
    model = scene.build_custom_grasp_model(profile)

    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper/base_freejoint") == -1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube/target_cube_free_joint") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, scene.SUPPORT_GEOM_NAME) >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper/gripper_drive") >= 0
    assert np.allclose(
        scene._quaternion_rotate(profile.mount_quat, np.array((1.0, 0.0, 0.0))),
        (1.0, 0.0, 0.0),
    )
    assert np.allclose(
        scene._quaternion_rotate(profile.mount_quat, np.array((0.0, 1.0, 0.0))),
        (0.0, 0.0, 1.0),
    )
    cube_size = model.geom_size[model.geom("cube/target_cube_geom").id, :3]
    assert np.allclose(cube_size, (0.003, 0.0125, 0.0125))
    assert 2.0 * cube_size[1] > 0.024
    assert 2.0 * cube_size[2] > 0.024
    pillar_id = model.geom("gripper/left_taxel_geom_11").id
    assert np.allclose(model.geom_solref[pillar_id], (-6000.0, -10.0))
    assert np.allclose(model.geom_solimp[pillar_id], (0.75, 0.95, 0.0025, 0.5, 2.0))


def test_custom_horizontal_hold_and_zero_disturbance_baseline_passes() -> None:
    """水平无支撑保持与零扰动基线验收通过。"""
    _load_module("custom_grasp_scene", "custom_grasp_scene.py")
    validation = _load_module("run_custom_grasp_validation", "run_custom_grasp_validation.py")
    protocol = validation.DisturbanceProtocol(force_n=0.0)
    result = validation.run_acceptance(
        ROOT / "configs/custom_parallel_gripper.toml", protocol=protocol
    )

    assert result.hold_passed
    assert result.disturbance_passed
    assert result.simulation_stable


def test_custom_grasp_trace_plot_is_written(tmp_path: Path) -> None:
    """验收脚本实际写出 CSV 与绘图文件。"""
    _load_module("custom_grasp_scene", "custom_grasp_scene.py")
    validation = _load_module("run_custom_grasp_validation_plot", "run_custom_grasp_validation.py")
    output_csv = tmp_path / "custom_grasp.csv"
    output_plot = tmp_path / "custom_grasp.png"
    validation.run_acceptance(
        ROOT / "configs/custom_parallel_gripper.toml",
        protocol=validation.DisturbanceProtocol(force_n=0.0),
        output_csv=output_csv,
        output_plot=output_plot,
    )

    assert output_csv.is_file()
    assert output_plot.is_file()
    assert output_plot.stat().st_size > 0

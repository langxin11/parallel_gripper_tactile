"""验证自研夹爪固定基座抓取场景与验收脚本。"""

import csv
from pathlib import Path

import mujoco
import numpy as np
import pytest

from parallel_gripper_tactile import load_profile
from parallel_gripper_tactile.experiments import grasp as validation
from parallel_gripper_tactile.scenes import custom as scene


ROOT = Path(__file__).resolve().parents[1]


def test_custom_scene_fixes_reserved_base_and_keeps_free_cube() -> None:
    """场景固定预留 base 自由关节且方块保持自由。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
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
    cube_mass = model.body_mass[model.body("cube/target_cube").id]
    assert np.allclose(cube_size, (0.003, 0.0125, 0.0125))
    assert np.isclose(cube_mass, 0.050)
    assert 2.0 * cube_size[1] > 0.024
    assert 2.0 * cube_size[2] > 0.024
    pillar_id = model.geom("gripper/left_taxel_geom_11").id
    assert np.allclose(model.geom_solref[pillar_id], (-1200.0, -10.0))
    assert np.allclose(model.geom_solimp[pillar_id], (0.75, 0.95, 0.0025, 0.5, 2.0))


def test_custom_horizontal_hold_and_zero_disturbance_baseline_passes() -> None:
    """水平无支撑保持与零扰动基线验收通过。"""
    protocol = validation.DisturbanceProtocol(force_n=0.0)
    result = validation.run_acceptance(
        ROOT / "configs/custom_parallel_gripper.yaml", protocol=protocol
    )

    assert result.hold_passed
    assert result.disturbance_passed
    assert result.simulation_stable


def test_custom_grasp_rejects_cube_mass_below_fifty_grams() -> None:
    """自研夹爪验收场景拒绝低于 50 g 的测试块。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    with pytest.raises(ValueError, match="at least 0.05 kg"):
        scene.build_custom_grasp_model(profile, cube_mass=0.049)


def test_custom_grasp_trace_plot_is_written(tmp_path: Path) -> None:
    """验收脚本实际写出 CSV 与绘图文件。"""
    output_csv = tmp_path / "custom_grasp.csv"
    output_plot = tmp_path / "custom_grasp.png"
    validation.run_acceptance(
        ROOT / "configs/custom_parallel_gripper.yaml",
        protocol=validation.DisturbanceProtocol(force_n=0.0),
        output_csv=output_csv,
        output_plot=output_plot,
    )

    assert output_csv.is_file()
    assert output_plot.is_file()
    assert output_plot.stat().st_size > 0
    with output_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows
    required_columns = {
        "control_state",
        "target_normal_force_n",
        "measured_normal_force_n",
        "filtered_normal_force_n",
        "normal_force_error_n",
        "force_position_adjustment_rad",
        "measured_left_fx",
        "measured_left_fy",
        "measured_left_fz",
        "measured_right_fx",
        "measured_right_fy",
        "measured_right_fz",
        "taxel_normal_force_n",
        "left_taxel_normal_force_n",
        "right_taxel_normal_force_n",
        "active_taxel_contacts",
        "available_friction_n",
        "static_hold_demand_tangential_n",
        "friction_margin_n",
        "friction_utilization",
    }
    assert required_columns.issubset(rows[0])
    assert any(float(row["taxel_normal_force_n"]) > 0 for row in rows)
    assert any(row["control_state"] == "force_tracking" for row in rows)
    assert all(float(row["available_friction_n"]) >= 0 for row in rows)

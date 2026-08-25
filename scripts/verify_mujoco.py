"""对导出的平行夹爪做无界面结构与运动学检查。

运行方式::

    uv run scripts/verify_mujoco.py
    uv run scripts/verify_mujoco.py --mjcf assets/grippers/custom_parallel_gripper/parallel_gripper.xml
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from parallel_gripper_tactile import MITTorqueController, load_profile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPOSITORY_ROOT / "configs/custom_parallel_gripper.toml"
SAFE_CLOSED_TARGET = 1.30  # rad；避免无物体时 Pillar 之间相互接触。


def joint_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    """返回命名关节的当前 qpos 值。"""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"Missing joint: {name}")
    return float(data.qpos[model.jnt_qposadr[joint_id]])


def main() -> None:
    """编译模型，检查执行器、闭链、碰撞过滤与运动扫描。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--mjcf", type=Path, help="覆盖 profile 中的 MJCF 路径。")
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument(
        "--force-limit",
        type=float,
        help="为本次检查临时收紧 MIT 力矩命令上限 (N m)。",
    )
    args = parser.parse_args()

    profile = load_profile(args.profile)
    model_path = args.mjcf or profile.model_path
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    model.opt.gravity[:] = 0.0  # 这是运动学检查，不是下落测试。

    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_drive")
    if model.nu != 1 or actuator_id != 0:
        raise RuntimeError(
            f"Expected one actuator named gripper_drive; got nu={model.nu}, id={actuator_id}"
        )
    if args.force_limit is not None:
        if args.force_limit <= 0:
            parser.error("--force-limit 必须为正数。")
        model.actuator_forcerange[actuator_id] = [-args.force_limit, args.force_limit]
    if model.neq != 4:
        raise RuntimeError(
            f"Expected four equality rows (two per closed-loop pin); got {model.neq}"
        )

    active_collision_meshes = [
        mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_MESH,
            int(model.geom_dataid[geom_id]),
        )
        for geom_id in range(model.ngeom)
        if model.geom_contype[geom_id] and model.geom_conaffinity[geom_id]
    ]
    pillar_collision_meshes = [
        mesh_name for mesh_name in active_collision_meshes if mesh_name.startswith("pillars_")
    ]
    expected_pillar_collision_geoms = 18  # 每个指尖 9 个 Pillars 网格。
    if len(pillar_collision_meshes) != expected_pillar_collision_geoms:
        raise RuntimeError(
            f"Expected {expected_pillar_collision_geoms} active Pillars collision geoms; "
            f"got {len(pillar_collision_meshes)}: {pillar_collision_meshes}"
        )
    exterior_collision_meshes = set(active_collision_meshes) - set(pillar_collision_meshes)
    expected_exterior_collision_meshes = {"base", "stator", "bracket"}
    if not any(mesh_name.startswith("ls__mgn9_rail") for mesh_name in exterior_collision_meshes):
        raise RuntimeError("Missing active collision geom for the MGN9 rail")
    if (
        exterior_collision_meshes
        - expected_exterior_collision_meshes
        - {
            mesh_name
            for mesh_name in exterior_collision_meshes
            if mesh_name.startswith("ls__mgn9_rail")
        }
    ):
        raise RuntimeError(
            f"Unexpected active exterior collision meshes: {exterior_collision_meshes}"
        )

    drive_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_drive")
    drive_range = model.jnt_range[drive_id]
    targets = np.linspace(drive_range[0] + 0.05, min(SAFE_CLOSED_TARGET, drive_range[1] - 0.05), 3)

    controller = MITTorqueController.from_profile(model, profile)

    print(f"Loaded {model_path}")
    print(f"actuator: gripper_drive, range: [{drive_range[0]:.4f}, {drive_range[1]:.4f}] rad")
    if args.force_limit is not None:
        print(f"temporary actuator force limit: +/-{args.force_limit:.3f} N m")
    print(
        f"equalities: {model.neq}; active Pillars collision geoms: {len(pillar_collision_meshes)}; "
        f"active exterior collision meshes: {sorted(exterior_collision_meshes)}"
    )
    max_tracking_error = 0.0
    for target in targets:
        for _ in range(args.steps):
            command = controller.apply(data, target_position=float(target))
            if args.force_limit is not None:
                data.ctrl[actuator_id] = np.clip(
                    command.torque, -args.force_limit, args.force_limit
                )
            mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("Simulation produced non-finite state values")
        drive_position = joint_position(model, data, "gripper_drive")
        max_tracking_error = max(max_tracking_error, abs(target - drive_position))
        print(
            f"target={target:+.4f} rad | drive={drive_position:+.4f} | "
            f"left_slide={joint_position(model, data, 'left_finger_slide'):.5f} m | "
            f"right_slide={joint_position(model, data, 'right_finger_slide'):.5f} m"
        )

    print(
        "PASS: actuator, closed loops, Pillars/exterior collision filter, and motion sweep are stable."
    )
    if max_tracking_error > 0.05:
        print(
            f"WARNING: largest position tracking error is {max_tracking_error:.4f} rad. "
            "Do not use the full exported drive range until the mechanical limit is reconciled."
        )


if __name__ == "__main__":
    main()

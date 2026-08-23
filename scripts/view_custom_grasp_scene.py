"""交互式检查自研夹爪配置的安装朝向。

运行方式::

    uv run scripts/view_custom_grasp_scene.py

场景与自研抓取验收使用相同的运行时 attach、物块几何和临时支撑。本脚本
刻意保持静态：在运行验收实验前，用它检查世界坐标系、直线滑轨、指尖与
25×25 mm 接触面之间的关系。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco

from custom_grasp_scene import DEFAULT_PROFILE, GRIPPER_PREFIX, build_custom_grasp_model
from parallel_gripper_tactile import load_profile


def main() -> None:
    """为安装好的自研夹爪场景打开被动 MuJoCo viewer。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--closed", action="store_true", help="检查闭合夹爪姿态。")
    parser.add_argument(
        "--frame",
        choices=("world", "body", "none"),
        default="world",
        help="坐标系标架叠加：world（默认）、body 或 none。",
    )
    args = parser.parse_args()

    profile = load_profile(args.profile)
    model = build_custom_grasp_model(profile)
    data = mujoco.MjData(model)
    actuator_id = model.actuator(f"{GRIPPER_PREFIX}{profile.actuator}").id
    data.ctrl[actuator_id] = profile.closed_control if args.closed else profile.open_control
    mujoco.mj_forward(model, data)

    from mujoco import viewer

    with viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as scene_viewer:
        scene_viewer.cam.lookat[:] = data.xpos[model.body("cube/target_cube").id]
        scene_viewer.cam.distance = 0.42
        scene_viewer.cam.azimuth = 135
        scene_viewer.cam.elevation = -18
        frame_modes = {
            "world": mujoco.mjtFrame.mjFRAME_WORLD,
            "body": mujoco.mjtFrame.mjFRAME_BODY,
            "none": mujoco.mjtFrame.mjFRAME_NONE,
        }
        scene_viewer.opt.frame = frame_modes[args.frame]
        while scene_viewer.is_running():
            mujoco.mj_forward(model, data)
            scene_viewer.sync()


if __name__ == "__main__":
    main()

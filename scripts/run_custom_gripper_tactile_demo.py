"""在自研夹爪的指定 taxel 前放置静态方块，并显示三维接触力。

示例::

    .venv/bin/python scripts/run_custom_gripper_tactile_demo.py --auto-close
    .venv/bin/python scripts/run_custom_gripper_tactile_demo.py --auto-close --no-viewer
    .venv/bin/python scripts/run_custom_gripper_tactile_demo.py --taxel right:12 --auto-close
    .venv/bin/python scripts/run_custom_gripper_tactile_demo.py \
      --auto-close --shear-axis y --shear-distance 0.002
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from parallel_gripper_tactile import ContactTaxelReader, load_profile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPOSITORY_ROOT / "configs" / "custom_parallel_gripper.toml"
CUBE_FREEJOINT = "cube_shear_freejoint"
CUBE_MOCAP = "cube_shear_mocap"


def _parse_taxel(value: str) -> tuple[str, int, int]:
    """解析紧凑的 ``left:11`` 或 ``right:02`` taxel 选择器。"""
    try:
        side, index = value.split(":", maxsplit=1)
        row, column = int(index[0]), int(index[1])
    except (IndexError, ValueError) as error:
        raise argparse.ArgumentTypeError("taxel 必须形如 left:11 或 right:02") from error
    if side not in {"left", "right"} or row not in range(3) or column not in range(3):
        raise argparse.ArgumentTypeError("taxel 必须是 left/right:00 到 left/right:22")
    return side, row, column


def _taxel_position(profile, side: str, row: int, column: int) -> np.ndarray:
    """从准备好的模型中读取命名触觉 site 的默认位置。"""
    source_model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    source_data = mujoco.MjData(source_model)
    mujoco.mj_forward(source_model, source_data)
    geom_name = profile.tactile.names(side)[row * profile.tactile.cols + column]
    site_name = geom_name.replace("_geom_", "_", 1)
    return source_data.site_xpos[source_model.site(site_name).id].copy()


def build_demo_model(
    profile,
    taxel: tuple[str, int, int],
    cube_half_size: float,
    *,
    shear_axis: str | None = None,
) -> mujoco.MjModel:
    """构建固定基座模型，并在单个触觉 site 前放置静态方块。"""
    if cube_half_size <= 0:
        raise ValueError("cube_half_size must be positive")
    tree = ET.parse(profile.model_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is not None and (meshdir := compiler.get("meshdir")):
        compiler.set("meshdir", str((profile.model_path.parent / meshdir).resolve()))

    # 导出的根是自由的，以便外部机械臂安装；这个独立的触觉测试改为固定
    # base，让静态方块测试可行。
    base = root.find(".//body[@name='base']")
    if base is None:
        raise ValueError("model is missing base body")
    for freejoint in base.findall("freejoint"):
        base.remove(freejoint)

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("gravity", "0 0 0")

    side, row, column = taxel
    taxel_position = _taxel_position(profile, side, row, column)
    cube_position = (0.0, taxel_position[1], taxel_position[2])
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("model is missing worldbody")
    cube = ET.SubElement(
        worldbody,
        "body",
        name="target_cube",
        pos=" ".join(f"{value:.9g}" for value in cube_position),
    )
    if shear_axis is not None:
        if shear_axis not in {"y", "z"}:
            raise ValueError("shear_axis must be 'y', 'z', or None")
        # 仿真循环通过刚性 weld 把该自由体焊到 mocap 测试夹具上，以指定其
        # 位置与切向速度。
        ET.SubElement(cube, "freejoint", name=CUBE_FREEJOINT)
    ET.SubElement(
        cube,
        "geom",
        name="target_cube_geom",
        type="box",
        size=f"0.012 {cube_half_size:.9g} {cube_half_size:.9g}",
        friction="0.8 0.02 0.001",
        rgba="0.95 0.55 0.1 1",
    )
    if shear_axis is not None:
        ET.SubElement(
            worldbody,
            "body",
            name=CUBE_MOCAP,
            mocap="true",
            pos=" ".join(f"{value:.9g}" for value in cube_position),
        )
        equality = root.find("equality")
        if equality is None:
            equality = ET.SubElement(root, "equality")
        ET.SubElement(
            equality,
            "weld",
            name="cube_shear_weld",
            body1="target_cube",
            body2=CUBE_MOCAP,
            solref="0.002 1",
            solimp="0.99 0.999 0.0005 0.5 2",
        )
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def main() -> None:
    """运行单 taxel 静态方块压缩检查。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--taxel", type=_parse_taxel, default=("left", 1, 1))
    parser.add_argument("--steps", type=int, default=1800)
    parser.add_argument("--auto-close", action="store_true")
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--render-fps", type=float, default=60.0)
    parser.add_argument("--cube-half-size", type=float, default=0.003)
    parser.add_argument("--shear-axis", choices=("y", "z"), help="夹紧后方块的世界系切向移动轴")
    parser.add_argument(
        "--shear-distance", type=float, default=0.0, help="夹紧后的切向位移（m，正负均可）"
    )
    args = parser.parse_args()
    if args.no_viewer and not args.auto_close:
        parser.error("无界面运行需要 --auto-close")
    if args.steps <= 0 or args.render_fps <= 0:
        parser.error("--steps 与 --render-fps 必须为正数")
    if args.shear_distance and not args.auto_close:
        parser.error("切向加载需要 --auto-close")
    if args.shear_distance and args.shear_axis is None:
        parser.error("设置 --shear-distance 时还需要 --shear-axis y 或 z")
    if abs(args.shear_distance) > 0.01:
        parser.error("--shear-distance 不能超过 0.01 m")

    profile = load_profile(args.profile)
    model = build_demo_model(
        profile,
        args.taxel,
        args.cube_half_size,
        shear_axis=args.shear_axis if args.shear_distance else None,
    )
    data = mujoco.MjData(model)
    actuator_id = model.actuator(profile.actuator).id
    shear_mocap_id = (
        int(model.body_mocapid[model.body(CUBE_MOCAP).id]) if args.shear_distance else -1
    )
    initial_mocap_position = data.mocap_pos[shear_mocap_id].copy() if args.shear_distance else None
    reader = ContactTaxelReader.from_profile(model, profile)
    selected_side, selected_row, selected_column = args.taxel

    viewer = None
    if not args.no_viewer:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True)
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        viewer.cam.lookat[:] = (0.0, 0.0, 0.14)
        viewer.cam.distance = 0.24
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

    try:
        step = 0
        wall_start = time.monotonic()
        next_render_time = wall_start
        while viewer is None or viewer.is_running():
            if args.auto_close:
                data.ctrl[actuator_id] = profile.closed_control * min(1.0, step / (args.steps / 3))
            if initial_mocap_position is not None:
                shear_start = args.steps * 0.5
                shear_duration = args.steps * 0.25
                shear_progress = min(1.0, max(0.0, (step - shear_start) / shear_duration))
                axis_index = 1 if args.shear_axis == "y" else 2
                data.mocap_pos[shear_mocap_id] = initial_mocap_position
                data.mocap_pos[shear_mocap_id, axis_index] += args.shear_distance * shear_progress
            mujoco.mj_step(model, data)
            frame = reader.read(data)
            selected_force = (
                frame.left[:, selected_row, selected_column]
                if selected_side == "left"
                else frame.right[:, selected_row, selected_column]
            )
            if step % 100 == 0 or step == args.steps - 1:
                shear_control = (
                    f" shear_{args.shear_axis}={args.shear_distance * shear_progress:+.4f}"
                    if initial_mocap_position is not None
                    else ""
                )
                shear_totals = ""
                if initial_mocap_position is not None:
                    left_total = frame.left.sum(axis=(1, 2))
                    right_total = frame.right.sum(axis=(1, 2))
                    shear_totals = (
                        f" Lsum=[{left_total[0]:+.3f}, {left_total[1]:+.3f}, {left_total[2]:+.3f}]"
                        f" Rsum=[{right_total[0]:+.3f}, {right_total[1]:+.3f}, {right_total[2]:+.3f}]"
                    )
                print(
                    f"step={step:4d} ctrl={data.ctrl[actuator_id]:.3f} "
                    f"{shear_control} "
                    f"{selected_side}_taxel_{selected_row}{selected_column}="
                    f"[{selected_force[0]:+.3f}, {selected_force[1]:+.3f}, {selected_force[2]:+.3f}] N"
                    f"{shear_totals}"
                )
            step += 1
            if args.auto_close and step >= args.steps:
                break
            if viewer is not None:
                viewer.sync()
                next_render_time += 1.0 / args.render_fps
                time.sleep(max(0.0, next_render_time - time.monotonic()))
    finally:
        if viewer is not None and viewer.is_running():
            viewer.close()


if __name__ == "__main__":
    main()

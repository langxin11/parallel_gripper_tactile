"""闭合 Robotiq 2F-85 抓取居中正方体，并输出两侧 taxel 力。"""

from __future__ import annotations

import argparse
import time
from pathlib import Path


def _taxel_force_sum(data, side: str) -> float:
    """汇总一侧 3×3 taxel 的法向力读数。"""
    return sum(
        -float(data.sensor(f"{side}_taxel_force_{row}{column}").data[2])
        for row in range(3)
        for column in range(3)
    )


def main() -> None:
    """运行闭合抓取演示，并默认打开实时 MuJoCo viewer。

    物块初始位于两个指尖之间，夹爪控制量从零匀速增长至指定值。
    场景关闭重力，以隔离接触模型和 taxel 映射验证。
    """
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请以 `uv run --extra sim` 运行此脚本。") from error

    default_scene = Path(__file__).resolve().parents[1] / "assets/scenes/cube_grasp.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=default_scene)
    parser.add_argument("--steps", type=int, default=1500, help="仿真步数。")
    parser.add_argument(
        "--close-control", type=float, default=220, help="最终夹爪控制量，范围 0~255。"
    )
    parser.add_argument("--no-viewer", action="store_true", help="不打开 MuJoCo 交互式 viewer。")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    viewer = None
    if not args.no_viewer:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, "grasp_overview"
        )

    try:
        for step in range(args.steps):
            data.ctrl[0] = args.close_control * min(1.0, step / max(1, args.steps // 3))
            mujoco.mj_step(model, data)
            if step % 100 == 0 or step == args.steps - 1:
                print(
                    f"step={step:4d} ctrl={data.ctrl[0]:6.1f} "
                    f"left={_taxel_force_sum(data, 'left'):8.3f} N "
                    f"right={_taxel_force_sum(data, 'right'):8.3f} N"
                )
            if viewer is not None:
                viewer.sync()
                time.sleep(model.opt.timestep)
    finally:
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()

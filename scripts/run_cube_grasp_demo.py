"""闭合 Robotiq 2F-85 抓取正方体，并输出两侧 taxel 力。

常见用法::

    uv run scripts/run_cube_grasp_demo.py
    uv run scripts/run_cube_grasp_demo.py --auto-close
    uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer
"""

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
    """运行手动抓取演示，并默认打开实时 MuJoCo viewer。

    物块初始位于两个指尖之间。默认由 viewer 控制滑块直接控制夹爪；
    ``--auto-close`` 才会让夹爪按预设轨迹闭合。默认视角是可用鼠标旋转、
    平移和缩放的自由相机；物块由下方薄板承托并受重力作用。
    """
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    default_scene = Path(__file__).resolve().parents[1] / "assets/scenes/cube_grasp.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=default_scene)
    parser.add_argument("--steps", type=int, default=1500, help="仿真步数。")
    parser.add_argument(
        "--close-control", type=float, default=220, help="最终夹爪控制量，范围 0~255。"
    )
    parser.add_argument("--no-viewer", action="store_true", help="不打开 MuJoCo 交互式 viewer。")
    parser.add_argument("--auto-close", action="store_true", help="按预设轨迹自动闭合夹爪。")
    args = parser.parse_args()
    if args.no_viewer and not args.auto_close:
        parser.error("手动模式需要 MuJoCo viewer；无界面运行请同时传入 --auto-close。")

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    viewer = None
    if not args.no_viewer:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = (0.0, -0.12, 0.07)
        viewer.cam.distance = 0.38
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25

    try:
        step = 0
        while viewer is None or viewer.is_running():
            if args.auto_close:
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
            step += 1
            if args.auto_close and step >= args.steps:
                break
    finally:
        # 用户手动关闭窗口时 viewer 已请求退出；避免对已经销毁的 GLFW
        # 上下文再次调用 close，从而触发退出阶段的 GLFW 警告。
        if viewer is not None and viewer.is_running():
            viewer.close()


if __name__ == "__main__":
    main()

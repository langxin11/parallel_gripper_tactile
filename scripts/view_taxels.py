"""在 MuJoCo 原生 viewer 中检查 Robotiq 2F-85 指尖 taxel 布局。

常见用法::

    uv run scripts/view_taxels.py
    uv run scripts/view_taxels.py assets/scenes/cube_grasp.xml
    uv run scripts/view_taxels.py --no-site-frames
    uv run scripts/view_taxels.py --no-physics
    uv run scripts/view_taxels.py --render-fps 30
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time


def main() -> None:
    """加载派生 MJCF，并实时渲染带 taxel site 标架的交互式 viewer。

    Args:
        无。

    Raises:
        ModuleNotFoundError: 未安装项目依赖时抛出。
    """
    try:
        import mujoco
        import mujoco.viewer
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    default_xml = Path(__file__).resolve().parents[1] / "assets/scenes/cube_grasp.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "xml", nargs="?", type=Path, default=default_xml, help="待显示的 MJCF 文件。"
    )
    parser.add_argument(
        "--no-site-frames",
        action="store_true",
        help="不显示 site 局部坐标系标架。",
    )
    parser.add_argument(
        "--no-physics",
        action="store_true",
        help="保持初始状态，仅渲染而不推进物理仿真。",
    )
    parser.add_argument(
        "--render-fps",
        type=float,
        default=60.0,
        help="目标渲染帧率，默认 60 FPS。",
    )
    args = parser.parse_args()
    if args.render_fps <= 0:
        parser.error("--render-fps 必须为正数。")

    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=True, show_right_ui=True
    ) as viewer:
        if not args.no_site_frames:
            # mjFRAME_SITE 在每个 site 处绘制局部 XYZ 标架；force/torque sensor
            # 的读数正是在其绑定 site 的局部坐标系中表达。
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

        render_period = 1.0 / args.render_fps
        wall_start = time.monotonic()
        simulation_start = data.time
        next_render_time = wall_start
        while viewer.is_running():
            if not args.no_physics:
                # 物理始终按 MJCF 的细粒度 timestep 推进，渲染只按目标 FPS 刷新，
                # 以避免降低显示帧率时同时放慢仿真。
                target_simulation_time = simulation_start + (time.monotonic() - wall_start)
                while data.time < target_simulation_time:
                    mujoco.mj_step(model, data)
            viewer.sync()
            next_render_time += render_period
            time.sleep(max(0.0, next_render_time - time.monotonic()))


if __name__ == "__main__":
    main()

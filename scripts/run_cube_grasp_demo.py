"""闭合 Robotiq 2F-85 抓取正方体，并输出两侧 taxel 力。

常见用法::

    uv run scripts/run_cube_grasp_demo.py
    uv run scripts/run_cube_grasp_demo.py --auto-close
    uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer --no-rerun
    uv run scripts/run_cube_grasp_demo.py --render-fps 30
    uv run scripts/run_cube_grasp_demo.py --auto-close --record-rrd outputs/taxel.rrd
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from grasp_scene import DEFAULT_GRIPPER_XML, gripper_name_in_model, load_grasp_model
from recording import ForceCsvRecorder, RerunTactileLogger, TactileFrame


def _taxel_force_sum(data, side: str, sensor_name) -> float:
    """汇总一侧 3×3 taxel 的法向力读数。"""
    return sum(
        -float(data.sensor(sensor_name(f"{side}_taxel_force_{row}{column}")).data[2])
        for row in range(3)
        for column in range(3)
    )


def _taxel_force_vector(data, side: str, sensor_name) -> tuple[float, float, float]:
    """汇总一侧 taxel 子 body 传给 pad 父 body 的三维力。

    向量在各 taxel site 局部系中表达。site +Z 指向表面外侧，因此压缩载荷
    的原始 z 分量为负；若只需要正的压力标量，应使用 ``-fz``。
    """
    return tuple(
        sum(
            float(data.sensor(sensor_name(f"{side}_taxel_force_{row}{column}")).data[axis])
            for row in range(3)
            for column in range(3)
        )
        for axis in range(3)
    )


def _taxel_surface_force_grid(data, side: str, sensor_name) -> np.ndarray:
    """返回一侧 ``(3, 3, 3)`` 的 xyz×row×column 表面受力网格。

    MuJoCo force sensor 测量 taxel 子 body 传给 pad 父 body 的力；这里依据
    牛顿第三定律对完整向量取反，使返回值表示物体施加给 taxel 表面的力。
    """
    tactile = np.empty((3, 3, 3), dtype=np.float64)
    for row in range(3):
        for column in range(3):
            tactile[:, row, column] = -data.sensor(
                sensor_name(f"{side}_taxel_force_{row}{column}")
            ).data
    return tactile


def _taxel_surface_force_vector(data, side: str, sensor_name) -> tuple[float, float, float]:
    """返回物体施加给 taxel 表面的力，压缩时局部 Fz 为正。

    MuJoCo force sensor 的原始方向是 taxel 子 body -> pad 父 body；依据
    牛顿第三定律整体取反，得到 taxel 子 body 实际受到的表面载荷。
    """
    return tuple(_taxel_surface_force_grid(data, side, sensor_name).sum(axis=(1, 2)))


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

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, help="加载外部完整 MJCF，而非运行时 attach 场景。")
    parser.add_argument("--gripper-xml", type=Path, default=DEFAULT_GRIPPER_XML)
    parser.add_argument("--steps", type=int, default=1500, help="仿真步数。")
    parser.add_argument(
        "--close-control", type=float, default=220, help="最终夹爪控制量，范围 0~255。"
    )
    parser.add_argument("--no-viewer", action="store_true", help="不打开 MuJoCo 交互式 viewer。")
    parser.add_argument("--auto-close", action="store_true", help="按预设轨迹自动闭合夹爪。")
    parser.add_argument("--render-fps", type=float, default=60.0, help="目标 viewer 帧率，默认 60 FPS。")
    parser.add_argument("--record-csv", type=Path, help="将控制量和左右三维力写入 CSV。")
    parser.add_argument("--record-every", type=int, default=1, help="每隔多少物理步记录一次。")
    parser.add_argument("--no-rerun", action="store_true", help="不启动实时 Rerun 触觉仪表盘。")
    parser.add_argument("--record-rrd", type=Path, help="将完整触觉网格写入 Rerun RRD。")
    parser.add_argument("--rerun-hz", type=float, default=100.0, help="Rerun 采样频率，默认 100 Hz。")
    args = parser.parse_args()
    if args.no_viewer and not args.auto_close:
        parser.error("手动模式需要 MuJoCo viewer；无界面运行请同时传入 --auto-close。")
    if args.render_fps <= 0:
        parser.error("--render-fps 必须为正数。")
    if args.record_every <= 0:
        parser.error("--record-every 必须为正整数。")
    if args.rerun_hz <= 0:
        parser.error("--rerun-hz 必须为正数。")

    model = load_grasp_model(args.scene, args.gripper_xml)
    physics_hz = 1.0 / model.opt.timestep
    if args.rerun_hz > physics_hz:
        parser.error(f"--rerun-hz 不能超过 MuJoCo 物理频率 {physics_hz:g} Hz。")

    def sensor_name(name: str) -> str:
        return gripper_name_in_model(mujoco, model, name)

    data = mujoco.MjData(model)
    recorder = ForceCsvRecorder(args.record_csv, args.record_every)
    rerun_logger = RerunTactileLogger(
        "robotiq2f85_taxel_grasp",
        live=not args.no_rerun,
        path=args.record_rrd,
        hz=args.rerun_hz,
        pressure_max=15.0,
    )

    def sample_frame(step: int) -> TactileFrame:
        return TactileFrame(
            step=step,
            time_s=data.time,
            control=float(data.ctrl[0]),
            left=_taxel_surface_force_grid(data, "left", sensor_name),
            right=_taxel_surface_force_grid(data, "right", sensor_name),
        )

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
        wall_start = time.monotonic()
        simulation_start = data.time
        render_period = 1.0 / args.render_fps
        next_render_time = wall_start
        while viewer is None or viewer.is_running():
            if viewer is not None:
                # 渲染与物理步解耦：降低 FPS 不会降低仿真时间或物理精度。
                target_simulation_time = simulation_start + (time.monotonic() - wall_start)
                while data.time < target_simulation_time:
                    if args.auto_close:
                        data.ctrl[0] = args.close_control * min(
                            1.0, step / max(1, args.steps // 3)
                        )
                    mujoco.mj_step(model, data)
                    frame = sample_frame(step)
                    recorder.record(frame)
                    rerun_logger.record(frame)
                    if step % 100 == 0 or step == args.steps - 1:
                        print(
                            f"step={step:4d} ctrl={data.ctrl[0]:6.1f} "
                            f"left={frame.left_force[2]:8.3f} N "
                            f"right={frame.right_force[2]:8.3f} N"
                        )
                    step += 1
                    if args.auto_close and step >= args.steps:
                        break
                viewer.sync()
                if args.auto_close and step >= args.steps:
                    break
                next_render_time += render_period
                time.sleep(max(0.0, next_render_time - time.monotonic()))
            else:
                data.ctrl[0] = args.close_control * min(1.0, step / max(1, args.steps // 3))
                mujoco.mj_step(model, data)
                frame = sample_frame(step)
                recorder.record(frame)
                rerun_logger.record(frame)
                if step % 100 == 0 or step == args.steps - 1:
                    print(
                        f"step={step:4d} ctrl={data.ctrl[0]:6.1f} "
                        f"left={frame.left_force[2]:8.3f} N "
                        f"right={frame.right_force[2]:8.3f} N"
                    )
                step += 1
                if step >= args.steps:
                    break
    finally:
        recorder.close()
        rerun_logger.close()
        # 用户手动关闭窗口时 viewer 已请求退出；避免对已经销毁的 GLFW
        # 上下文再次调用 close，从而触发退出阶段的 GLFW 警告。
        if viewer is not None and viewer.is_running():
            viewer.close()


if __name__ == "__main__":
    main()

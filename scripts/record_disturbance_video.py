"""录制切向扰动仿真视频，在方块上以渲染级力箭头显示扰动力。

使用 MuJoCo 推荐的 mjvScene 渲染级几何体范式：
- ``renderer.update_scene()`` 构建物理场景
- ``mjv_connector`` 将箭头写入已申领的场景槽位
- ``renderer.render()`` 一并渲染

箭头不参与物理计算，仅在扰动阶段显示。

常见用法::

    uv run scripts/record_disturbance_video.py
    uv run scripts/record_disturbance_video.py --disturbance-duration 8 --rotation-rate 1.5
    uv run scripts/record_disturbance_video.py --force 8 --frequency 1.5 --gripper touch_grid
"""

from __future__ import annotations

import argparse
import math
import shutil
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from parallel_gripper_tactile import DisturbanceProtocol

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOX_TAXEL_XML = REPOSITORY_ROOT / "assets/grippers/robotiq_2f85/2f85_taxels_box.xml"
DEFAULT_TOUCH_GRID_XML = REPOSITORY_ROOT / "assets/grippers/robotiq_2f85/2f85_touch_grid_3x3.xml"
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "outputs/robotiq/disturbance_video/tactile_disturbance_simulation.mp4"
)
SUPPORT_GEOM_NAME = "target_cube_support_plate"
CUBE_BODY_NAME = "cube/target_cube"


def _object_id(mujoco, model, object_type, name: str) -> int:
    """按名称解析 MuJoCo 对象 id，缺失时抛出异常。"""
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"模型中缺少 {name!r}。")
    return object_id


def record_disturbance_video(
    gripper_xml: Path = DEFAULT_BOX_TAXEL_XML,
    output: Path = DEFAULT_OUTPUT,
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    camera_name: str = "grasp_overview",
    force_scale: float = 0.008,
    protocol: DisturbanceProtocol | None = None,
    close_control: float = 220.0,
    keep_frames: bool = False,
) -> None:
    """运行扰动仿真并以指定参数录制 MP4 视频。

    力箭头通过直接操作 mjvScene 实现——属于 MuJoCo 推荐的渲染级
    可视化范式，不修改物理模型。

    Args:
        gripper_xml: 夹爪 MJCF。
        output: 输出 MP4 路径。
        width: 视频宽度（像素）。
        height: 视频高度（像素）。
        fps: 视频帧率。
        camera_name: MuJoCo 摄像机名称。
        force_scale: 力矢量缩放系数 (m/N)。
        protocol: 扰动实验协议；缺省时使用本脚本的录制默认值。
        close_control: 最终夹爪控制量。
        keep_frames: 是否保留 PNG 中间帧。
    """
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    from grasp_scene import load_grasp_model
    from parallel_gripper_tactile import DisturbanceProtocol
    from parallel_gripper_tactile.video import add_arrow_to_scene, encode_video, save_pixels

    if protocol is None:
        protocol = DisturbanceProtocol(
            disturbance_duration=5.0, recovery_duration=1.0, rotation_rate_rad_s=0.8
        )

    # ---- 加载场景 (不含箭头的干净场景) ----
    model = load_grasp_model(None, gripper_xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    steps = math.ceil(protocol.total_duration / float(model.opt.timestep))

    # ---- 获取句柄 ----
    support_geom_id = _object_id(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, SUPPORT_GEOM_NAME)
    cube_body_id = _object_id(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, CUBE_BODY_NAME)

    # ---- 摄像机 ----
    camera_id = -1
    try:
        camera_id = _object_id(mujoco, model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    except ValueError:
        print(f"注意: 摄像机 {camera_name!r} 不存在，使用默认视角。")

    # ---- 离屏渲染器 ----
    renderer = mujoco.Renderer(model, height, width)

    # ---- 帧目录 ----
    frames_dir = (
        Path(tempfile.mkdtemp(prefix="disturbance_frames_")) / "frames"
        if not keep_frames
        else output.with_name(output.stem + "_frames")
    )
    frames_dir.mkdir(parents=True, exist_ok=True)
    tmp_root = frames_dir.parent if not keep_frames else None

    print(f"仿真步数: {steps}  物理步长: {model.opt.timestep:.4f} s")
    print(
        f"总时长: {protocol.total_duration:.2f} s  "
        f"扰动: {protocol.disturbance_duration:.1f}s / "
        f"{protocol.force_n}N@{protocol.frequency_hz}Hz / "
        f"旋转 {protocol.rotation_rate_rad_s:.1f} rad/s"
    )
    print(f"视频: {fps} FPS  {width}×{height}")

    # ---- 逐帧渲染 ----
    video_frames = int(protocol.total_duration * fps)
    render_times = [i / fps for i in range(video_frames)]

    frame_idx = 0
    sim_step = 0

    for target_time in render_times:
        # 推进仿真到目标时间
        while data.time < target_time and sim_step < steps:
            protocol.step(
                model,
                data,
                actuator_id=0,
                close_control=close_control,
                support_geom_id=support_geom_id,
                cube_body_id=cube_body_id,
            )
            mujoco.mj_step(model, data)
            sim_step += 1

        if sim_step >= steps:
            break

        # 1) 构建物理场景
        renderer.update_scene(data, camera=camera_id)
        scene = renderer.scene
        base_ngeom = scene.ngeom  # 物理场景占用的槽位数

        # 2) 添加力箭头 (渲染级装饰物)
        applied = protocol.force_vector_at(data.time)
        if float(np.linalg.norm(applied)) > 1e-6:
            cube_pos = data.xpos[cube_body_id]
            # 略微上移避免箭头被方块遮挡
            arrow_origin = cube_pos + np.array([0.0, 0.0, 0.015], dtype=np.float64)
            add_arrow_to_scene(scene, base_ngeom, arrow_origin, applied, force_scale)

        # 3) 渲染
        pixels = renderer.render()

        frame_path = frames_dir / f"frame_{frame_idx:06d}.png"
        save_pixels(pixels, frame_path)
        frame_idx += 1

        if frame_idx % 30 == 0:
            magnitude = float(np.linalg.norm(applied))
            info = f"|F|={magnitude:.1f}N" if magnitude > 1e-6 else ""
            print(f"  已渲染 {frame_idx}/{video_frames} 帧 (t={data.time:.2f}s {info})")

    print(f"共渲染 {frame_idx} 帧。")

    # ---- 编码视频 ----
    output.parent.mkdir(parents=True, exist_ok=True)
    encode_video(frames_dir, output, fps, width, height)

    # ---- 清理 ----
    if not keep_frames and tmp_root is not None:
        shutil.rmtree(tmp_root, ignore_errors=True)
        print(f"已清理临时帧: {tmp_root}")
    else:
        print(f"帧文件保留在: {frames_dir}")

    renderer.close()
    print(f"视频已保存: {output}")


# ---- CLI ----


def main() -> None:
    """解析命令行参数并录制扰动视频。"""
    from parallel_gripper_tactile import DisturbanceProtocol

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gripper", choices=["box_taxel", "touch_grid"], default="box_taxel")
    parser.add_argument("--gripper-xml", type=Path, help="自定义夹爪 MJCF。")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 720), metavar=("W", "H"))
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera", type=str, default="grasp_overview")
    parser.add_argument("--force-scale", type=float, default=0.008)
    parser.add_argument("--force", type=float, default=5.0, help="扰动正弦力幅值 N。")
    parser.add_argument("--frequency", type=float, default=2.0, help="扰动正弦力频率 Hz。")
    parser.add_argument("--disturbance-duration", type=float, default=5.0, help="扰动时长 s。")
    parser.add_argument(
        "--rotation-rate", type=float, default=0.8, help="力方向角速度 rad/s (0=不旋转)。"
    )
    parser.add_argument("--close-duration", type=float, default=1.0)
    parser.add_argument("--recovery-duration", type=float, default=1.0)
    parser.add_argument("--close-control", type=float, default=220.0, help="最终夹爪控制量。")
    parser.add_argument("--keep-frames", action="store_true")

    args = parser.parse_args()

    if args.fps <= 0:
        parser.error("--fps 必须为正数。")
    if args.force_scale <= 0:
        parser.error("--force-scale 必须为正数。")

    gripper_xml = (
        args.gripper_xml
        or {
            "box_taxel": DEFAULT_BOX_TAXEL_XML,
            "touch_grid": DEFAULT_TOUCH_GRID_XML,
        }[args.gripper]
    )

    width, height = args.resolution
    protocol = DisturbanceProtocol(
        close_duration=args.close_duration,
        disturbance_duration=args.disturbance_duration,
        recovery_duration=args.recovery_duration,
        force_n=args.force,
        frequency_hz=args.frequency,
        rotation_rate_rad_s=args.rotation_rate,
    )

    record_disturbance_video(
        gripper_xml=gripper_xml,
        output=args.output,
        width=width,
        height=height,
        fps=args.fps,
        camera_name=args.camera,
        force_scale=args.force_scale,
        protocol=protocol,
        close_control=args.close_control,
        keep_frames=args.keep_frames,
    )


if __name__ == "__main__":
    raise SystemExit(main())

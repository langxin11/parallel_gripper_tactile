"""录制切向扰动仿真视频，在方块上以渲染级力箭头显示扰动力。

使用 MuJoCo 推荐的 mjvScene 渲染级几何体范式：
- ``renderer.update_scene()`` 构建物理场景
- ``mjv_initGeom`` 将箭头（杆体+头部圆柱）写入已申领的场景槽位
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
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOX_TAXEL_XML = REPOSITORY_ROOT / "assets/grippers/robotiq_2f85/2f85_taxels_box.xml"
DEFAULT_TOUCH_GRID_XML = REPOSITORY_ROOT / "assets/grippers/robotiq_2f85/2f85_touch_grid_3x3.xml"
DEFAULT_OUTPUT = REPOSITORY_ROOT / "outputs/tactile_disturbance_simulation.mp4"
SUPPORT_GEOM_NAME = "target_cube_support_plate"
CUBE_BODY_NAME = "cube/target_cube"


@dataclass(frozen=True, slots=True)
class DisturbanceProtocol:
    """抓稳、撤去支撑、施加切向正弦扰动力并观察恢复的实验协议。

    扰动力的幅值按正弦变化，方向在 YZ 切向平面内以恒定角速度旋转。
    """

    close_duration: float = 1.0
    support_settle_duration: float = 0.5
    release_settle_duration: float = 0.5
    disturbance_duration: float = 5.0
    recovery_duration: float = 1.0
    force_n: float = 5.0
    frequency_hz: float = 2.0
    rotation_rate_rad_s: float = 0.8

    @property
    def release_time(self) -> float:
        return self.close_duration + self.support_settle_duration

    @property
    def disturbance_start(self) -> float:
        return self.release_time + self.release_settle_duration

    @property
    def disturbance_end(self) -> float:
        return self.disturbance_start + self.disturbance_duration

    @property
    def total_duration(self) -> float:
        return self.disturbance_end + self.recovery_duration

    def force_vector_at(self, time_s: float) -> np.ndarray:
        """返回施加在方块质心的世界系 3D 力向量。"""
        if not self.disturbance_start <= time_s < self.disturbance_end:
            return np.zeros(3, dtype=np.float64)

        elapsed = time_s - self.disturbance_start
        magnitude = self.force_n * math.sin(2.0 * math.pi * self.frequency_hz * elapsed)
        angle = self.rotation_rate_rad_s * elapsed

        return np.array(
            [0.0, magnitude * math.cos(angle), magnitude * math.sin(angle)],
            dtype=np.float64,
        )


# ---- 渲染级力箭头 ----

def _add_arrow_to_scene(
    mujoco, scene, base_idx: int, origin, force, scale: float
) -> int:
    """在 mjvScene 中写入一个 MuJoCo 原生 mjGEOM_ARROW 几何体作为力箭头。

    ``mjv_connector`` 在 from_→to 两点之间创建带锥头的完整箭头，
    消耗 1 个场景槽位。

    返回消耗的槽位数（0 或 1）。"""
    magnitude = float(np.linalg.norm(force))
    if magnitude < 1e-9:
        return 0

    direction = force / magnitude
    arrow_len = max(0.01, magnitude * scale)
    arrow_tip = origin + direction * arrow_len

    scene.ngeom += 1
    slot = scene.geoms[base_idx]
    mujoco.mjv_connector(
        slot,
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        width=0.006,                      # 杆体半径
        from_=origin.astype(np.float64),
        to=arrow_tip.astype(np.float64),
    )
    # matid=-1 让 MuJoCo 使用 geom 自身的 rgba，而非模型材质
    slot.matid = -1
    slot.rgba = np.array([1.0, 0.08, 0.08, 0.92], dtype=np.float32)
    slot.emission = 0.15
    slot.specular = 0.1
    slot.shininess = 0.5
    slot.segid = -1
    slot.objtype = mujoco.mjtObj.mjOBJ_UNKNOWN
    slot.dataid = -1
    slot.category = int(mujoco.mjtCatBit.mjCAT_DECOR)

    return 1


# ---- 仿真与录制 ----

def _object_id(mujoco, model, object_type, name: str) -> int:
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
    keep_frames: bool = False,
) -> None:
    """运行扰动仿真并以指定参数录制 MP4 视频。

    力箭头通过直接操作 mjvScene 实现——属于 MuJoCo 推荐的渲染级
    可视化范式，不修改物理模型。

    Parameters
    ----------
    gripper_xml: 夹爪 MJCF。
    output: 输出 MP4 路径。
    width, height: 视频分辨率。
    fps: 视频帧率。
    camera_name: MuJoCo 摄像机名称。
    force_scale: 力矢量缩放系数 (m/N)。
    protocol: 扰动实验协议。
    keep_frames: 是否保留 PNG 中间帧。
    """
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    from grasp_scene import load_grasp_model

    if protocol is None:
        protocol = DisturbanceProtocol()

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
    print(f"总时长: {protocol.total_duration:.2f} s  "
          f"扰动: {protocol.disturbance_duration:.1f}s / "
          f"{protocol.force_n}N@{protocol.frequency_hz}Hz / "
          f"旋转 {protocol.rotation_rate_rad_s:.1f} rad/s")
    print(f"视频: {fps} FPS  {width}×{height}")

    # ---- 逐帧渲染 ----
    video_frames = int(protocol.total_duration * fps)
    render_times = [i / fps for i in range(video_frames)]

    frame_idx = 0
    sim_step = 0

    for target_time in render_times:
        # 推进仿真到目标时间
        while data.time < target_time and sim_step < steps:
            _disturbance_step(model, data, protocol, support_geom_id, cube_body_id)
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
            _add_arrow_to_scene(mujoco, scene, base_ngeom, arrow_origin, applied, force_scale)

        # 3) 渲染
        pixels = renderer.render()

        frame_path = frames_dir / f"frame_{frame_idx:06d}.png"
        _save_pixels(pixels, frame_path)
        frame_idx += 1

        if frame_idx % 30 == 0:
            magnitude = float(np.linalg.norm(applied))
            info = f"|F|={magnitude:.1f}N" if magnitude > 1e-6 else ""
            print(f"  已渲染 {frame_idx}/{video_frames} 帧 "
                  f"(t={data.time:.2f}s {info})")

    print(f"共渲染 {frame_idx} 帧。")

    # ---- 编码视频 ----
    output.parent.mkdir(parents=True, exist_ok=True)
    _encode_video(frames_dir, output, fps, width, height)

    # ---- 清理 ----
    if not keep_frames and tmp_root is not None:
        shutil.rmtree(tmp_root, ignore_errors=True)
        print(f"已清理临时帧: {tmp_root}")
    else:
        print(f"帧文件保留在: {frames_dir}")

    renderer.close()
    print(f"视频已保存: {output}")


def _disturbance_step(
    model, data, protocol: DisturbanceProtocol,
    support_geom_id: int, cube_body_id: int,
) -> None:
    """设置控制、支撑碰撞状态以及方块外力。"""
    time_s = float(data.time)
    data.ctrl[0] = 220.0 * min(1.0, time_s / protocol.close_duration)

    if time_s >= protocol.release_time:
        model.geom_contype[support_geom_id] = 0
        model.geom_conaffinity[support_geom_id] = 0

    data.xfrc_applied[cube_body_id] = 0.0
    data.xfrc_applied[cube_body_id, :3] = protocol.force_vector_at(time_s)


# ---- 像素保存与视频编码 ----

def _save_pixels(pixels: np.ndarray, path: Path) -> None:
    try:
        from PIL import Image
        Image.fromarray(pixels, mode="RGB").save(path)
    except ImportError:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(
            figsize=(pixels.shape[1] / 100, pixels.shape[0] / 100),
            dpi=100, frameon=False,
        )
        ax.imshow(pixels)
        ax.axis("off")
        fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
        fig.savefig(path, dpi=100)
        plt.close(fig)


def _encode_video(
    frames_dir: Path, output: Path, fps: int, width: int, height: int
) -> None:
    if shutil.which("ffmpeg"):
        subprocess.run(
            [
                "ffmpeg", "-y",
                "-r", str(fps),
                "-i", str(frames_dir / "frame_%06d.png"),
                "-vf", f"scale={width}:{height}:flags=lanczos,format=yuv420p",
                "-c:v", "libx264",
                "-preset", "medium",
                "-crf", "23",
                "-pix_fmt", "yuv420p",
                str(output),
            ],
            check=True,
        )
        return

    try:
        import imageio
        writer = imageio.get_writer(output, fps=fps, format="FFMPEG", codec="libx264")
        for png_file in sorted(frames_dir.glob("frame_*.png")):
            from PIL import Image
            writer.append_data(np.array(Image.open(png_file)))
        writer.close()
        return
    except ImportError:
        pass

    raise RuntimeError(
        "需要 ffmpeg 或 imageio 将帧编码为 MP4。"
        "请安装 ffmpeg 或 `uv run pip install imageio imageio-ffmpeg`。"
    )


# ---- CLI ----

def main() -> None:
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
    parser.add_argument("--rotation-rate", type=float, default=0.8, help="力方向角速度 rad/s (0=不旋转)。")
    parser.add_argument("--close-duration", type=float, default=1.0)
    parser.add_argument("--recovery-duration", type=float, default=1.0)
    parser.add_argument("--keep-frames", action="store_true")

    args = parser.parse_args()

    if args.fps <= 0:
        parser.error("--fps 必须为正数。")
    if args.force_scale <= 0:
        parser.error("--force-scale 必须为正数。")

    gripper_xml = args.gripper_xml or {
        "box_taxel": DEFAULT_BOX_TAXEL_XML,
        "touch_grid": DEFAULT_TOUCH_GRID_XML,
    }[args.gripper]

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
        keep_frames=args.keep_frames,
    )


if __name__ == "__main__":
    raise SystemExit(main())

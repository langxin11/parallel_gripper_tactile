r"""录制自研夹爪的无支撑保持与扰动实验视频。

示例::

    uv run scripts/record_custom_grasp_video.py
    uv run scripts/record_custom_grasp_video.py --force 0 \\
      --output outputs/custom_gripper/disturbance_video/custom_gripper_zero_disturbance.mp4

源模型 ``base`` 仍是未来转接法兰的安装根；录制时只移除其临时仿真自由关节，
并将它刚性固定到场景中。
"""

from __future__ import annotations

import argparse
import math
import shutil
import tempfile
from pathlib import Path

import mujoco
import numpy as np

from custom_grasp_scene import (
    CUBE_PREFIX,
    DEFAULT_CUBE_HALF_CONTACT_SIDE,
    DEFAULT_CUBE_HALF_THICKNESS,
    DEFAULT_CUBE_MASS,
    DEFAULT_PROFILE,
    GRIPPER_PREFIX,
    SUPPORT_GEOM_NAME,
    build_custom_grasp_model,
)
from parallel_gripper_tactile import DisturbanceProtocol, load_profile
from parallel_gripper_tactile.video import add_arrow_to_scene, encode_video, save_pixels


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "outputs"
    / "custom_gripper"
    / "disturbance_video"
    / "custom_gripper_disturbance.mp4"
)
CUBE_BODY_NAME = f"{CUBE_PREFIX}target_cube"


def _phase_label(phase: str) -> str:
    """把内部相位名转换为紧凑的视频标签。"""
    return {
        "close": "CLOSE",
        "support_settle": "SUPPORT SETTLE",
        "unsupported_hold": "UNSUPPORTED HOLD",
        "disturbance": "DISTURBANCE",
        "recovery": "RECOVERY",
    }[phase]


def _annotate_pixels(pixels: np.ndarray, phase: str, force_y: float, time_s: float) -> np.ndarray:
    """叠加简洁的实验元信息，不修改 MuJoCo 物理。"""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return pixels
    image = Image.fromarray(pixels, mode="RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    scale = image.width / 960.0
    title_font = ImageFont.truetype("DejaVuSans.ttf", round(20 * scale))
    detail_font = ImageFont.truetype("DejaVuSans.ttf", round(16 * scale))
    left, top = round(18 * scale), round(18 * scale)
    draw.rounded_rectangle(
        (left, top, left + round(380 * scale), top + round(92 * scale)),
        radius=round(7 * scale),
        fill=(0, 0, 0, 165),
    )
    draw.text(
        (left + round(15 * scale), top + round(12 * scale)),
        _phase_label(phase),
        font=title_font,
        fill=(255, 255, 255, 255),
    )
    draw.text(
        (left + round(15 * scale), top + round(50 * scale)),
        f"t = {time_s:4.2f} s    Fy = {force_y:+4.2f} N",
        font=detail_font,
        fill=(230, 230, 230, 255),
    )
    return np.asarray(image)


def record_custom_grasp_video(
    *,
    profile_path: Path = DEFAULT_PROFILE,
    output: Path = DEFAULT_OUTPUT,
    width: int = 1920,
    height: int = 1080,
    fps: int = 30,
    protocol: DisturbanceProtocol = DisturbanceProtocol(),
    cube_half_thickness: float = DEFAULT_CUBE_HALF_THICKNESS,
    cube_half_contact_side: float = DEFAULT_CUBE_HALF_CONTACT_SIDE,
    cube_mass: float = DEFAULT_CUBE_MASS,
    keep_frames: bool = False,
) -> bool:
    """录制水平自研夹爪抓取视频，并返回仿真稳定性。"""
    if width <= 0 or height <= 0 or fps <= 0:
        raise ValueError("width, height, and fps must be positive")
    profile = load_profile(profile_path)
    model = build_custom_grasp_model(
        profile,
        cube_half_thickness=cube_half_thickness,
        cube_half_contact_side=cube_half_contact_side,
        cube_mass=cube_mass,
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    actuator_id = model.actuator(f"{GRIPPER_PREFIX}{profile.actuator}").id
    support_id = model.geom(SUPPORT_GEOM_NAME).id
    cube_body_id = model.body(CUBE_BODY_NAME).id
    renderer = mujoco.Renderer(model, height, width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = data.xpos[cube_body_id]
    camera.distance = 0.34
    camera.azimuth = 135
    camera.elevation = -20

    frames_dir = (
        output.with_name(output.stem + "_frames")
        if keep_frames
        else Path(tempfile.mkdtemp(prefix="custom_grasp_frames_")) / "frames"
    )
    frames_dir.mkdir(parents=True, exist_ok=True)
    temporary_root = None if keep_frames else frames_dir.parent
    total_frames = math.ceil(protocol.total_duration * fps)
    stable = True
    frame_count = 0
    try:
        for target_time in (frame / fps for frame in range(total_frames)):
            while data.time < target_time:
                time_s = float(data.time)
                phase = protocol.phase_at(time_s)
                protocol.step(
                    model,
                    data,
                    actuator_id=actuator_id,
                    close_control=profile.closed_control,
                    support_geom_id=support_id,
                    cube_body_id=cube_body_id,
                )
                mujoco.mj_step(model, data)
                if data.time <= time_s or not np.isfinite(data.qpos).all():
                    stable = False
                    break
            if not stable:
                break

            time_s = float(data.time)
            phase = protocol.phase_at(time_s)
            force_y = protocol.force_y_at(time_s)
            renderer.update_scene(data, camera=camera)
            scene = renderer.scene
            if abs(force_y) > 1e-6:
                origin = data.xpos[cube_body_id] + np.array([0.0, 0.0, 0.018])
                add_arrow_to_scene(
                    scene,
                    scene.ngeom,
                    origin,
                    np.array([0.0, force_y, 0.0]),
                    scale=0.012,
                )
            pixels = _annotate_pixels(renderer.render(), phase, force_y, time_s)
            save_pixels(pixels, frames_dir / f"frame_{frame_count:06d}.png")
            frame_count += 1
    finally:
        renderer.close()

    if frame_count == 0:
        raise RuntimeError("no video frames were rendered")
    output.parent.mkdir(parents=True, exist_ok=True)
    encode_video(frames_dir, output, fps, width, height)
    if temporary_root is not None:
        shutil.rmtree(temporary_root, ignore_errors=True)
    print(f"Video: {output}")
    print(f"Frames: {frame_count}/{total_frames}")
    print(f"Simulation stability: {'PASS' if stable else 'FAIL'}")
    return stable


def main() -> int:
    """运行视频录制，并把数值失稳作为失败状态传播。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--resolution", type=int, nargs=2, default=(1920, 1080), metavar=("W", "H"))
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--cube-half-thickness", type=float, default=DEFAULT_CUBE_HALF_THICKNESS, metavar="M"
    )
    parser.add_argument(
        "--cube-half-contact-side", type=float, default=DEFAULT_CUBE_HALF_CONTACT_SIDE, metavar="M"
    )
    parser.add_argument("--cube-mass", type=float, default=DEFAULT_CUBE_MASS, metavar="KG")
    parser.add_argument("--force", type=float, default=5.0, help="世界 Y 方向扰动幅值 (N)。")
    parser.add_argument("--frequency", type=float, default=2.0, help="扰动频率 (Hz)。")
    parser.add_argument("--keep-frames", action="store_true")
    args = parser.parse_args()
    if args.force < 0 or args.frequency <= 0:
        parser.error("--force 必须非负，--frequency 必须为正数。")
    if args.cube_half_thickness <= 0 or args.cube_half_contact_side <= 0 or args.cube_mass <= 0:
        parser.error("方块尺寸和 --cube-mass 必须为正数。")
    width, height = args.resolution
    stable = record_custom_grasp_video(
        profile_path=args.profile,
        output=args.output,
        width=width,
        height=height,
        fps=args.fps,
        protocol=DisturbanceProtocol(force_n=args.force, frequency_hz=args.frequency),
        cube_half_thickness=args.cube_half_thickness,
        cube_half_contact_side=args.cube_half_contact_side,
        cube_mass=args.cube_mass,
        keep_frames=args.keep_frames,
    )
    return 0 if stable else 1


if __name__ == "__main__":
    raise SystemExit(main())

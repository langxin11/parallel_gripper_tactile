"""录制 DM_Gripper 法向力抓取验收实验。"""

from __future__ import annotations

import math
import shutil
import tempfile
from pathlib import Path

import mujoco
import numpy as np

from ..scenes.custom import (
    CUBE_PREFIX,
    DEFAULT_CUBE_HALF_CONTACT_SIDE,
    DEFAULT_CUBE_HALF_THICKNESS,
    DEFAULT_CUBE_MASS,
    DEFAULT_PROFILE,
    GRIPPER_PREFIX,
    ObjectMaterial,
    SUPPORT_GEOM_NAME,
    build_custom_grasp_model,
)
from ..control import NormalForceController
from ..config.profiles import GripperProfile, load_profile, validate_resolved_profile
from ..protocols import DisturbanceProtocol
from ..timing import SimulationTimer
from ..video import add_arrow_to_scene, encode_video, save_pixels
from .grasp import _friction_capacity, _taxel_geom_sides

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "outputs"
    / "custom_gripper"
    / "disturbance_video"
    / "custom_gripper_disturbance.mp4"
)
CUBE_BODY_NAME = f"{CUBE_PREFIX}target_cube"
CUBE_GEOM_NAME = f"{CUBE_PREFIX}target_cube_geom"


def _phase_label(phase: str) -> str:
    """把内部相位名转换为紧凑的视频标签。"""
    return {
        "close": "CLOSE",
        "support_settle": "SUPPORT SETTLE",
        "unsupported_hold": "UNSUPPORTED HOLD",
        "disturbance": "DISTURBANCE",
        "recovery": "RECOVERY",
    }[phase]


def _annotate_pixels(
    pixels: np.ndarray,
    phase: str,
    force_y: float,
    time_s: float,
    control_state: str,
    normal_force_n: float,
    target_force_n: float,
) -> np.ndarray:
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
        (left, top, left + round(470 * scale), top + round(118 * scale)),
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
    state_label = "FORCE PID" if control_state == "force_tracking" else "MIT APPROACH"
    draw.text(
        (left + round(15 * scale), top + round(78 * scale)),
        f"{state_label}    Fn = {normal_force_n:4.2f}/{target_force_n:4.2f} N",
        font=detail_font,
        fill=(120, 220, 255, 255),
    )
    return np.asarray(image)


def record_custom_grasp_video(
    *,
    profile_path: Path | str = DEFAULT_PROFILE,
    resolved_profile: GripperProfile | None = None,
    output: Path = DEFAULT_OUTPUT,
    width: int = 1920,
    height: int = 1080,
    fps: int = 30,
    protocol: DisturbanceProtocol = DisturbanceProtocol(),
    cube_half_thickness: float = DEFAULT_CUBE_HALF_THICKNESS,
    cube_half_contact_side: float = DEFAULT_CUBE_HALF_CONTACT_SIDE,
    cube_mass: float = DEFAULT_CUBE_MASS,
    object_material: ObjectMaterial = "hard",
    target_force_n: float | None = None,
    control_period_s: float = 0.002,
    keep_frames: bool = False,
) -> bool:
    """录制水平 DM_Gripper 抓取视频，并返回仿真稳定性。"""
    if width <= 0 or height <= 0 or fps <= 0 or control_period_s <= 0:
        raise ValueError("width, height, fps, and control_period_s must be positive")
    profile = (
        load_profile(profile_path)
        if resolved_profile is None
        else validate_resolved_profile(resolved_profile)
    )
    if target_force_n is not None:
        if target_force_n <= 0:
            raise ValueError("target_force_n must be positive")
        if profile.normal_force is None:
            raise ValueError("profile does not define control.force")
        profile = profile.model_copy(
            update={
                "control": profile.control.model_copy(
                    update={
                        "force": profile.normal_force.model_copy(
                            update={"target_n": target_force_n}
                        )
                    }
                )
            }
        )
    if profile.normal_force is None:
        raise ValueError("profile does not define control.force")
    model = build_custom_grasp_model(
        profile,
        cube_half_thickness=cube_half_thickness,
        cube_half_contact_side=cube_half_contact_side,
        cube_mass=cube_mass,
        object_material=object_material,
    )
    if control_period_s + 1e-12 < float(model.opt.timestep):
        raise ValueError("control_period_s must not be smaller than the physics timestep")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    control_timer = SimulationTimer(control_period_s, float(data.time))
    controller = NormalForceController.from_profile(model, profile, name_prefix=GRIPPER_PREFIX)
    actuator_id = controller.actuator_id
    support_id = model.geom(SUPPORT_GEOM_NAME).id
    cube_body_id = model.body(CUBE_BODY_NAME).id
    cube_geom_id = model.geom(CUBE_GEOM_NAME).id
    taxel_geom_sides = _taxel_geom_sides(model, profile)
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
    control_state = "approach"
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
                    apply_actuator_control=False,
                )
                control_dt = control_timer.pop_due(time_s)
                if control_dt is not None:
                    feedback = _friction_capacity(
                        model,
                        data,
                        taxel_geom_sides=taxel_geom_sides,
                        cube_geom_id=cube_geom_id,
                    )
                    force_command = controller.apply(
                        data,
                        approach_position=protocol.close_target_at(
                            time_s, profile.open_control, profile.closed_control
                        ),
                        total_normal_force_n=feedback.normal_force_n,
                        left_normal_force_n=feedback.left_normal_force_n,
                        right_normal_force_n=feedback.right_normal_force_n,
                        dt=control_dt,
                    )
                    control_state = force_command.state
                mujoco.mj_step(model, data)
                if data.time <= time_s or not np.isfinite(data.qpos).all():
                    stable = False
                    break
            if not stable:
                break

            time_s = float(data.time)
            phase = protocol.phase_at(time_s)
            force_y = protocol.force_y_at(time_s)
            force_feedback = _friction_capacity(
                model,
                data,
                taxel_geom_sides=taxel_geom_sides,
                cube_geom_id=cube_geom_id,
            )
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
            pixels = _annotate_pixels(
                renderer.render(),
                phase,
                force_y,
                time_s,
                control_state,
                force_feedback.normal_force_n,
                profile.normal_force.target_n,
            )
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

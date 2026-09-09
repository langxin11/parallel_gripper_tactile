"""在共享姿态下比较原生网格接触与 MuJoCo mesh-SDF。"""

from __future__ import annotations

import csv
import shutil
import tempfile
import time
from pathlib import Path

import mujoco
import numpy as np

from ..contact_taxels import ContactTaxelReader
from ..control import MITTorqueController
from ..visualization import (
    FULL_WIDTH_FONT_SCALE,
    paper_figsize,
    save_publication_figure,
    science_pyplot,
)

from ..profiles import load_profile
from ..video import encode_video, save_pixels
from .custom_demo import build_demo_model


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
CONTACT_PROFILE = REPOSITORY_ROOT / "configs" / "custom_parallel_gripper.yaml"
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "outputs/custom_gripper/experiments/contact_model_ab"


def _force_row(prefix: str, force: np.ndarray) -> dict[str, float]:
    """返回一个局部三轴 taxel 力的稳定 CSV 键。"""
    return {f"{prefix}_{axis}_n": float(value) for axis, value in zip(("fx", "fy", "fz"), force)}


def _selected_force(frame, side: str, row: int, column: int) -> np.ndarray:
    """返回选定 taxel 的局部力向量。"""
    return frame.left[:, row, column] if side == "left" else frame.right[:, row, column]


def _selected_contact_count(data: mujoco.MjData, geom_id: int) -> int:
    """统计涉及选定触觉几何体的活动接触数量。"""
    return sum(
        contact.geom1 == geom_id or contact.geom2 == geom_id
        for contact in data.contact[: data.ncon]
    )


def _write_csv(rows: list[dict[str, float | int]], output_path: Path) -> None:
    """写出普通接触与 mesh-SDF 的逐姿态数据。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _plot(rows: list[dict[str, float | int]], output_path: Path) -> None:
    """按项目 SciencePlots 规范绘制 A/B 法向力、接触数与耗时。"""
    plt = science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)
    time_s = np.asarray([row["time_s"] for row in rows], dtype=np.float64)
    native_fz = np.asarray([row["native_fz_n"] for row in rows], dtype=np.float64)
    sdf_fz = np.asarray([row["sdf_fz_n"] for row in rows], dtype=np.float64)
    native_contacts = np.asarray([row["native_selected_contacts"] for row in rows])
    sdf_contacts = np.asarray([row["sdf_selected_contacts"] for row in rows])
    native_ms = np.asarray([row["native_step_ms"] for row in rows], dtype=np.float64)
    sdf_ms = np.asarray([row["sdf_evaluation_ms"] for row in rows], dtype=np.float64)

    colors = {"black": "#000000", "blue": "#0072B2", "orange": "#D55E00"}
    figure, axes = plt.subplots(3, 1, sharex=True, figsize=paper_figsize(7.2), layout="constrained")
    axes[0].plot(time_s, native_fz, color=colors["blue"], label="Mesh contact", linewidth=1.3)
    axes[0].plot(
        time_s,
        sdf_fz,
        color=colors["orange"],
        linestyle="--",
        label="Mesh-SDF",
        linewidth=1.3,
    )
    axes[0].set_ylabel("Taxel local Fz (N)")
    axes[0].legend()
    axes[1].step(
        time_s, native_contacts, where="post", color=colors["blue"], label="Mesh", linewidth=1.2
    )
    axes[1].step(
        time_s,
        sdf_contacts,
        where="post",
        color=colors["orange"],
        linestyle="--",
        label="Mesh-SDF",
        linewidth=1.2,
    )
    axes[1].set_ylabel("Selected contacts")
    axes[1].legend()
    axes[2].plot(time_s, native_ms, color=colors["blue"], label="Mesh physics step", linewidth=1.2)
    axes[2].plot(
        time_s,
        sdf_ms,
        color=colors["orange"],
        linestyle="--",
        label="Mesh-SDF evaluation",
        linewidth=1.2,
    )
    axes[2].set_xlabel("Reference simulation time (s)")
    axes[2].set_ylabel("Wall time (ms)")
    for axis in axes:
        axis.legend(frameon=False, loc="upper left")
        axis.tick_params(direction="in", which="both", top=True, right=True)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    save_publication_figure(figure, output_path)
    plt.close(figure)


def _annotate_pair(
    native_pixels: np.ndarray,
    sdf_pixels: np.ndarray,
    *,
    time_s: float,
    native_force: np.ndarray,
    sdf_force: np.ndarray,
) -> np.ndarray:
    """拼接两张同视角画面，并标注各自的局部法向力。"""
    pixels = np.concatenate((native_pixels, sdf_pixels), axis=1)
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return pixels
    image = Image.fromarray(pixels, mode="RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    scale = image.height / 540.0
    title_font = ImageFont.truetype("DejaVuSans.ttf", round(20 * scale))
    detail_font = ImageFont.truetype("DejaVuSans.ttf", round(16 * scale))
    panel_width = image.width // 2
    labels = (
        ("A  MuJoCo mesh contact", native_force),
        ("B  MuJoCo mesh-SDF", sdf_force),
    )
    for index, (title, force) in enumerate(labels):
        left = index * panel_width + round(14 * scale)
        top = round(14 * scale)
        draw.rounded_rectangle(
            (left, top, left + round(300 * scale), top + round(72 * scale)),
            radius=round(6 * scale),
            fill=(0, 0, 0, 170),
        )
        draw.text(
            (left + round(10 * scale), top + round(8 * scale)),
            title,
            font=title_font,
            fill=(255, 255, 255, 255),
        )
        draw.text(
            (left + round(10 * scale), top + round(39 * scale)),
            f"t={time_s:.3f} s   local Fz={force[2]:+.3f} N",
            font=detail_font,
            fill=(220, 230, 255, 255),
        )
    return np.asarray(image)


def _camera() -> mujoco.MjvCamera:
    """返回适合单 taxel 方块接触测试的固定观察相机。"""
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (-0.04, 0.0, 0.14)
    camera.distance = 0.16
    camera.azimuth = 135
    camera.elevation = -20
    return camera


def record_contact_ab(
    *,
    output_dir: Path,
    profile_path: Path = CONTACT_PROFILE,
    taxel: tuple[str, int, int],
    cube_half_size: float,
    steps: int,
    stop_above_native_force: float,
    fps: int,
    width: int,
    height: int,
    keep_frames: bool,
) -> tuple[Path, Path, Path]:
    """生成 A/B CSV、曲线和并排 MP4，并返回三个文件路径。"""
    profile = load_profile(profile_path)
    native_model = build_demo_model(profile, taxel, cube_half_size)
    sdf_model = build_demo_model(profile, taxel, cube_half_size, pillar_type="sdf")
    native_data = mujoco.MjData(native_model)
    sdf_data = mujoco.MjData(sdf_model)
    controller = MITTorqueController.from_profile(native_model, profile)
    native_reader = ContactTaxelReader.from_profile(native_model, profile)
    sdf_reader = ContactTaxelReader.from_profile(sdf_model, profile)
    side, row, column = taxel
    taxel_name = profile.tactile.names(side)[row * profile.tactile.cols + column]
    native_geom_id = native_model.geom(taxel_name).id
    sdf_geom_id = sdf_model.geom(taxel_name).id
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "native_vs_sdf.csv"
    plot_path = output_dir / "native_vs_sdf.png"
    video_path = output_dir / "native_vs_sdf.mp4"
    frames_dir = (
        output_dir / "native_vs_sdf_frames"
        if keep_frames
        else Path(tempfile.mkdtemp(prefix="pgt_native_sdf_frames_")) / "frames"
    )
    frames_dir.mkdir(parents=True, exist_ok=True)
    temporary_root = None if keep_frames else frames_dir.parent
    scene_option = mujoco.MjvOption()
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    native_renderer = mujoco.Renderer(native_model, height, width)
    sdf_renderer = mujoco.Renderer(sdf_model, height, width)
    camera = _camera()
    rows: list[dict[str, float | int]] = []
    next_frame_time = 0.0
    frame_count = 0
    try:
        for step in range(steps):
            progress = min(1.0, step / (steps / 3.0))
            target_position = profile.open_control + progress * (
                profile.closed_control - profile.open_control
            )
            controller.apply(native_data, target_position=target_position)
            native_start = time.perf_counter()
            mujoco.mj_step(native_model, native_data)
            native_step_ms = (time.perf_counter() - native_start) * 1_000.0
            native_force = _selected_force(native_reader.read(native_data), side, row, column)
            sdf_data.qpos[:] = native_data.qpos
            sdf_data.qvel[:] = native_data.qvel
            sdf_data.time = native_data.time
            sdf_start = time.perf_counter()
            mujoco.mj_forward(sdf_model, sdf_data)
            sdf_force = _selected_force(sdf_reader.read(sdf_data), side, row, column)
            sdf_ms = (time.perf_counter() - sdf_start) * 1_000.0
            rows.append(
                {
                    "step": step,
                    "time_s": float(native_data.time),
                    "control": float(native_data.ctrl[controller.actuator_id]),
                    **_force_row("native", native_force),
                    **_force_row("sdf", sdf_force),
                    "native_selected_contacts": _selected_contact_count(
                        native_data, native_geom_id
                    ),
                    "sdf_selected_contacts": _selected_contact_count(sdf_data, sdf_geom_id),
                    "native_step_ms": native_step_ms,
                    "sdf_evaluation_ms": sdf_ms,
                }
            )
            if native_data.time + 1e-12 >= next_frame_time:
                native_renderer.update_scene(native_data, camera=camera, scene_option=scene_option)
                sdf_renderer.update_scene(sdf_data, camera=camera, scene_option=scene_option)
                pixels = _annotate_pair(
                    native_renderer.render(),
                    sdf_renderer.render(),
                    time_s=float(native_data.time),
                    native_force=native_force,
                    sdf_force=sdf_force,
                )
                save_pixels(pixels, frames_dir / f"frame_{frame_count:06d}.png")
                frame_count += 1
                next_frame_time += 1.0 / fps
            if float(np.linalg.norm(native_force)) >= stop_above_native_force:
                break
    finally:
        native_renderer.close()
        sdf_renderer.close()

    if not rows or frame_count == 0:
        raise RuntimeError("接触 A/B 实验未产生数据或视频帧")
    _write_csv(rows, csv_path)
    _plot(rows, plot_path)
    encode_video(frames_dir, video_path, fps, width * 2, height)
    if temporary_root is not None:
        shutil.rmtree(temporary_root)
    return csv_path, plot_path, video_path

"""Shared offscreen-rendering helpers for the disturbance video scripts."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import mujoco
import numpy as np


def add_arrow_to_scene(scene, base_idx: int, origin, force, scale: float) -> int:
    """Write a native MuJoCo ``mjGEOM_ARROW`` force arrow into a free scene slot.

    ``mjv_connector`` builds a complete arrow with a conical head between
    ``from_`` and ``to`` and consumes one scene slot.

    Returns:
        The number of slots consumed (0 when the force is negligible).
    """
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
        width=0.006,  # shaft radius
        from_=origin.astype(np.float64),
        to=arrow_tip.astype(np.float64),
    )
    # matid=-1 makes MuJoCo use the geom's own rgba instead of a model material.
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


def save_pixels(pixels: np.ndarray, path: Path) -> None:
    """Save an RGB render to a PNG, falling back to Matplotlib without Pillow."""
    try:
        from PIL import Image

        Image.fromarray(pixels, mode="RGB").save(path)
    except ImportError:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(
            figsize=(pixels.shape[1] / 100, pixels.shape[0] / 100),
            dpi=100,
            frameon=False,
        )
        ax.imshow(pixels)
        ax.axis("off")
        fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
        fig.savefig(path, dpi=100)
        plt.close(fig)


def encode_video(frames_dir: Path, output: Path, fps: int, width: int, height: int) -> None:
    """Encode a numbered PNG frame sequence into an MP4 with ffmpeg or imageio."""
    if shutil.which("ffmpeg"):
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-r",
                str(fps),
                "-i",
                str(frames_dir / "frame_%06d.png"),
                "-vf",
                f"scale={width}:{height}:flags=lanczos,format=yuv420p",
                "-c:v",
                "libx264",
                "-preset",
                "medium",
                "-crf",
                "23",
                "-pix_fmt",
                "yuv420p",
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
        "encoding MP4 requires ffmpeg or imageio; "
        "install ffmpeg or run `uv run pip install imageio imageio-ffmpeg`"
    )

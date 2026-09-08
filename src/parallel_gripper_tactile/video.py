"""供扰动视频脚本共享的离屏渲染辅助函数。"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import mujoco
import numpy as np


def add_arrow_to_scene(
    scene,
    base_idx: int,
    origin,
    force,
    scale: float,
    *,
    width: float = 0.006,
    min_length: float = 0.01,
    max_length: float | None = None,
    emission: float = 0.15,
    tip_at_origin: bool = False,
) -> int:
    """将一个原生 MuJoCo ``mjGEOM_ARROW`` 力箭头写入空闲的 scene 槽。

    ``mjv_connector`` 会在 ``from_`` 与 ``to`` 之间构建一个带锥形箭头的
    完整箭头，并占用一个 scene 槽。

    Returns:
        被占用的槽数量（当力可忽略时返回 0）。
    """
    magnitude = float(np.linalg.norm(force))
    if magnitude < 1e-9:
        return 0

    direction = force / magnitude
    arrow_len = max(min_length, magnitude * scale)
    if max_length is not None:
        arrow_len = min(arrow_len, max_length)
    if tip_at_origin:
        arrow_base = origin - direction * arrow_len
        arrow_tip = origin
    else:
        arrow_base = origin
        arrow_tip = origin + direction * arrow_len

    slot = scene.geoms[base_idx]
    rgba = np.array([1.0, 0.08, 0.08, 0.92], dtype=np.float32)
    mujoco.mjv_initGeom(
        slot,
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        size=np.zeros(3, dtype=np.float64),
        pos=np.zeros(3, dtype=np.float64),
        mat=np.eye(3, dtype=np.float64).reshape(-1),
        rgba=rgba,
    )
    mujoco.mjv_connector(
        slot,
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        width=width,
        from_=arrow_base.astype(np.float64),
        to=arrow_tip.astype(np.float64),
    )
    # matid=-1 让 MuJoCo 使用几何体自身的 rgba，而不是模型材质。
    slot.matid = -1
    slot.emission = emission
    slot.specular = 0.1
    slot.shininess = 0.5
    slot.segid = -1
    # scene 槽会跨帧复用；mjv_connector 不会清空旧标签和对象编号。
    # 显式重置可避免内部绑定说明被误绘制成悬浮文字。
    slot.label = ""
    slot.objid = -1
    # 用 mjOBJ_GEOM 而非 mjOBJ_UNKNOWN：后者会让 MuJoCo 在开启标签时对 dataid
    # 解引用而产生乱码标签（如 void* / bitgen_t* / PyObject*）；几何体自身
    # 没有名字，标签渲染自然会跳过。
    slot.objtype = mujoco.mjtObj.mjOBJ_GEOM
    slot.dataid = -1
    slot.category = int(mujoco.mjtCatBit.mjCAT_DECOR)
    scene.ngeom += 1

    return 1


def save_pixels(pixels: np.ndarray, path: Path) -> None:
    """将 RGB 渲染保存为 PNG，在没有 Pillow 时回退到 Matplotlib。"""
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
    """使用 ffmpeg 或 imageio 将编号的 PNG 帧序列编码为 MP4。"""
    if shutil.which("ffmpeg"):
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-framerate",
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
                "18",
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

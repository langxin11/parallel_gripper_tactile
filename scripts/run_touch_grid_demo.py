"""闭合抓取方块，并以 OpenCV 显示两侧 touch_grid 的切向力与压力。"""

from __future__ import annotations

import argparse
from pathlib import Path


def tactile_to_bgr(tactile, scale: int = 12, max_shear: float = 8, max_pressure: float = 20):
    """将 ``(3, H, W)`` 触觉图转换为箭头与压力颜色组成的 BGR 图像。"""
    import cv2
    import numpy as np

    _channels, rows, cols = tactile.shape
    image = np.zeros((rows * scale, cols * scale, 3), dtype=np.uint8)
    for row in range(rows):
        for col in range(cols):
            shear_x = float(np.clip(tactile[0, row, col] / max_shear, -1, 1))
            shear_y = float(np.clip(tactile[1, row, col] / max_shear, -1, 1))
            pressure = float(np.clip(tactile[2, row, col] / max_pressure, 0, 1))
            center = (int((col + 0.5) * scale), int((row + 0.5) * scale))
            end = (int(center[0] + shear_x * scale), int(center[1] - shear_y * scale))
            color = (0, int(255 * (1 - pressure)), int(255 * pressure))
            cv2.arrowedLine(image, center, end, color, 1, tipLength=0.35)
    return image


def _read_tactile(data, name: str):
    """读取插件输出，并从参考项目的 zxy 顺序转换为 xyz 顺序。"""
    return data.sensor(name).data.reshape((3, 32, 32))[[1, 2, 0]]


def main() -> None:
    """执行闭合抓取并实时显示左右触觉图；按 Esc 可提前退出。"""
    try:
        import cv2
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请以 `uv run --extra sim --extra viz` 运行此脚本。") from error

    default_scene = Path(__file__).resolve().parents[1] / "assets/scenes/cube_grasp_touch_grid.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=default_scene)
    parser.add_argument("--steps", type=int, default=1500)
    parser.add_argument(
        "--no-window", action="store_true", help="不打开 OpenCV 窗口，适用于自动验证。"
    )
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    for step in range(args.steps):
        data.ctrl[0] = 220 * min(1.0, step / max(1, args.steps // 3))
        mujoco.mj_step(model, data)
        if not args.no_window and step % 5 == 0:
            cv2.imshow("touch_left", tactile_to_bgr(_read_tactile(data, "touch_left")))
            cv2.imshow("touch_right", tactile_to_bgr(_read_tactile(data, "touch_right")))
            if cv2.waitKey(1) == 27:
                break
    if not args.no_window:
        cv2.destroyAllWindows()
    left = _read_tactile(data, "touch_left")
    right = _read_tactile(data, "touch_right")
    print(f"left pressure sum={left[2].sum():.3f}, right pressure sum={right[2].sum():.3f}")


if __name__ == "__main__":
    main()

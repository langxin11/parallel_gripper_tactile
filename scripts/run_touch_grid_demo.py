"""闭合抓取方块，并以 OpenCV 显示两侧 touch_grid 的切向力与压力。"""

from __future__ import annotations

import argparse
import time
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
    """实时运行手动抓取，并显示左右触觉图。

    默认由 MuJoCo viewer 的控制滑块直接写入 ``fingers_actuator``；本脚本不覆盖
    ``data.ctrl``，因此可在闭合过程中观察 OpenCV 触觉图。传入 ``--auto-close``
    时才执行预设的自动闭合轨迹。
    """
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
    parser.add_argument("--no-viewer", action="store_true", help="不打开 MuJoCo 交互式 viewer。")
    parser.add_argument("--auto-close", action="store_true", help="按预设轨迹自动闭合夹爪。")
    args = parser.parse_args()
    if args.no_viewer and not args.auto_close:
        parser.error("手动模式需要 MuJoCo viewer；无界面运行请同时传入 --auto-close。")
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    viewer = None
    if not args.no_viewer:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, "grasp_overview"
        )
    try:
        step = 0
        while viewer is None or viewer.is_running():
            if args.auto_close:
                data.ctrl[0] = 220 * min(1.0, step / max(1, args.steps // 3))
            mujoco.mj_step(model, data)
            if not args.no_window and step % 5 == 0:
                cv2.imshow("touch_left", tactile_to_bgr(_read_tactile(data, "touch_left")))
                cv2.imshow("touch_right", tactile_to_bgr(_read_tactile(data, "touch_right")))
                if cv2.waitKey(1) == 27:
                    break
            if viewer is not None:
                viewer.sync()
                time.sleep(model.opt.timestep)
            step += 1
            if args.auto_close and step >= args.steps:
                break
    finally:
        if viewer is not None:
            viewer.close()
    if not args.no_window:
        cv2.destroyAllWindows()
    left = _read_tactile(data, "touch_left")
    right = _read_tactile(data, "touch_right")
    print(f"left pressure sum={left[2].sum():.3f}, right pressure sum={right[2].sum():.3f}")


if __name__ == "__main__":
    main()

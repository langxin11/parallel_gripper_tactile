"""在 MuJoCo 原生 viewer 中检查 Robotiq 2F-85 指尖 taxel 布局。

常见用法::

    uv run scripts/view_taxels.py
    uv run scripts/view_taxels.py assets/scenes/cube_grasp.xml
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    """加载派生 MJCF，并打开显示 site 与接触点的交互式 viewer。

    Args:
        无。

    Raises:
        ModuleNotFoundError: 未安装项目依赖时抛出。
    """
    try:
        import mujoco
        import mujoco.viewer
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    default_xml = Path(__file__).resolve().parents[1] / "assets/scenes/cube_grasp.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "xml", nargs="?", type=Path, default=default_xml, help="待显示的 MJCF 文件。"
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mujoco.viewer.launch(model, data, show_left_ui=True, show_right_ui=True)


if __name__ == "__main__":
    main()

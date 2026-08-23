"""编译 MJCF 资产，验证其可被 MuJoCo 加载。

常见用法::

    uv run scripts/check_mjcf.py
    uv run scripts/check_mjcf.py assets/grippers/robotiq_2f85/2f85_touch_grid.xml
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    """编译指定 MJCF，输出模型维度。

    Raises:
        ModuleNotFoundError: 未安装项目依赖时抛出。
    """
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    default_xml = (
        Path(__file__).resolve().parents[1] / "assets/grippers/robotiq_2f85/2f85_taxels.xml"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xml", nargs="?", type=Path, default=default_xml)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.xml))
    print(f"{args.xml}: nsensor={model.nsensor}, nsite={model.nsite}, nu={model.nu}")


if __name__ == "__main__":
    main()

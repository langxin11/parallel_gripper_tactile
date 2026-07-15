"""以左右 3×3 网格格式输出 Robotiq 2F-85 taxel 力传感器读数。"""

from __future__ import annotations

import argparse
from collections.abc import Iterable
from pathlib import Path

TAXEL_SENSOR_NAMES = tuple(
    f"{side}_taxel_force_{row}{column}"
    for side in ("left", "right")
    for row in range(3)
    for column in range(3)
)


def _format_grid(values: Iterable[float]) -> str:
    """将一侧 9 个 taxel 力按行优先顺序格式化为 3×3 网格。"""
    values_list = list(values)
    if len(values_list) != 9:
        raise ValueError("每侧必须提供 9 个 taxel 力值。")
    return "\n".join(
        " ".join(f"{value:8.3f}" for value in values_list[row : row + 3]) for row in range(0, 9, 3)
    )


def read_taxel_forces(xml_path: Path) -> dict[str, list[float]]:
    """加载 MJCF 并读取当前仿真状态的 18 个 taxel 力。

    Args:
        xml_path: 含 taxel 力传感器的 MJCF 文件。

    Returns:
        以 ``left`` 和 ``right`` 为键的两个 3×3 力值列表。

    Raises:
        ModuleNotFoundError: 未安装 ``sim`` 可选依赖时抛出。
    """
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请以 `uv run --extra sim` 运行此脚本。") from error

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    forces: dict[str, list[float]] = {"left": [], "right": []}
    for name in TAXEL_SENSOR_NAMES:
        side = name.split("_", maxsplit=1)[0]
        forces[side].append(float(data.sensor(name).data[0]))
    return forces


def main() -> None:
    """读取模型初始状态，并以两块 3×3 表格输出 taxel 力。"""
    default_xml = Path(__file__).resolve().parents[1] / "assets/robotiq_2f85/2f85_taxels.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "xml", nargs="?", type=Path, default=default_xml, help="待读取的 MJCF 文件。"
    )
    args = parser.parse_args()
    forces = read_taxel_forces(args.xml)
    print("left taxels (N)")
    print(_format_grid(forces["left"]))
    print("\nright taxels (N)")
    print(_format_grid(forces["right"]))


if __name__ == "__main__":
    main()

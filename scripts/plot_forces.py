"""绘制抓取演示导出的控制量与三维接触力曲线。

常见用法::

    uv run scripts/plot_forces.py outputs/taxel_forces.csv
    uv run scripts/plot_forces.py outputs/touch_grid_forces.csv --output outputs/touch_grid_forces.pdf
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


REQUIRED_COLUMNS = {
    "time_s",
    "control",
    "left_fx",
    "left_fy",
    "left_fz",
    "right_fx",
    "right_fy",
    "right_fz",
}


def read_force_csv(path: Path) -> dict[str, list[float]]:
    """读取记录 CSV，并将每列转换为浮点数序列。"""
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        fieldnames = set(reader.fieldnames or ())
        missing = REQUIRED_COLUMNS - fieldnames
        if missing:
            raise ValueError(f"CSV 缺少字段: {', '.join(sorted(missing))}")
        values = {column: [] for column in REQUIRED_COLUMNS}
        for row in reader:
            for column in values:
                values[column].append(float(row[column]))
    if not values["time_s"]:
        raise ValueError("CSV 不含记录数据。")
    return values


def plot_forces(values: dict[str, list[float]], output: Path, show: bool = False) -> None:
    """按 SciencePlots 风格绘制控制量、左右三维力曲线。"""
    try:
        import matplotlib.pyplot as plt
        import scienceplots  # noqa: F401 -- 导入后才会注册 science 样式。
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    # ieee: IEEE 风格 Times 衬线+网格；no-latex: 不依赖 TeX 发行版。
    plt.style.use(["science", "ieee", "no-latex"])
    time_s = values["time_s"]
    figure, (control_axis, left_axis, right_axis) = plt.subplots(
        3, 1, figsize=(7.16, 6.5), sharex=True, layout="constrained"
    )
    control_axis.plot(time_s, values["control"], color="black", label="control")
    control_axis.set_ylabel("Control")
    control_axis.legend(frameon=False)

    for axis, side, title in (
        (left_axis, "left", "Left tactile force"),
        (right_axis, "right", "Right tactile force"),
    ):
        for component, color in zip(("x", "y", "z"), ("#1f77b4", "#ff7f0e", "#2ca02c")):
            axis.plot(time_s, values[f"{side}_f{component}"], color=color, label=f"F{component}")
        axis.set_ylabel("Force (N)")
        axis.set_title(title)
        axis.legend(ncol=3, frameon=False)

    right_axis.set_xlabel("Simulation time (s)")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=600)
    print(f"已保存 {output}")
    if show:
        plt.show()
    plt.close(figure)


def main() -> None:
    """读取 CSV 并保存曲线图。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="由抓取演示生成的记录 CSV。")
    parser.add_argument("--output", type=Path, help="输出图片路径，默认与 CSV 同名的 PNG。")
    parser.add_argument("--show", action="store_true", help="保存后同时打开 Matplotlib 窗口。")
    args = parser.parse_args()
    output = args.output or args.csv.with_suffix(".png")
    plot_forces(read_force_csv(args.csv), output, args.show)


if __name__ == "__main__":
    main()

"""读取并绘制常见的触觉力 CSV 轨迹。"""

from __future__ import annotations

import csv
from pathlib import Path

from .plotstyle import paper_figsize, save_publication_figure, science_pyplot


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
    plt = science_pyplot()
    time_s = values["time_s"]
    figure, (control_axis, left_axis, right_axis) = plt.subplots(
        3, 1, figsize=paper_figsize(6.5), sharex=True, layout="constrained"
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
    save_publication_figure(figure, output)
    print(f"已保存 {output}")
    if show:
        plt.show()
    plt.close(figure)

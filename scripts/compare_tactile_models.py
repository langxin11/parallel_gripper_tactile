"""在相同抓取条件下比较 3×3 box taxel 与 3×3 touch_grid。

常见用法::

    uv run scripts/compare_tactile_models.py
    uv run scripts/compare_tactile_models.py --output-csv outputs/comparison.csv
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean

from grasp_scene import gripper_name_in_model, load_grasp_model
from run_cube_grasp_demo import _taxel_surface_force_vector
from run_touch_grid_demo import _read_tactile, _touch_grid_shape


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOX_TAXEL_XML = REPOSITORY_ROOT / "assets/robotiq_2f85/2f85_taxels_box.xml"
DEFAULT_TOUCH_GRID_XML = REPOSITORY_ROOT / "assets/robotiq_2f85/2f85_touch_grid_3x3.xml"
DEFAULT_OUTPUT_CSV = REPOSITORY_ROOT / "outputs/tactile_model_comparison.csv"


@dataclass(slots=True)
class ForceTrace:
    """一次确定性抓取的逐步三维力。"""

    time_s: list[float]
    control: list[float]
    left: list[tuple[float, float, float]]
    right: list[tuple[float, float, float]]


@dataclass(frozen=True, slots=True)
class SideSummary:
    """一侧指尖的稳态与峰值比较。"""

    box_steady_n: float
    grid_steady_n: float
    relative_error: float
    box_peak_n: float
    grid_peak_n: float


def _close_control(step: int, steps: int, close_control: float) -> float:
    return close_control * min(1.0, step / max(1, steps // 3))


def _run_box_taxels(mujoco, xml: Path, steps: int, close_control: float) -> ForceTrace:
    model = load_grasp_model(None, xml)
    data = mujoco.MjData(model)

    def sensor_name(name: str) -> str:
        return gripper_name_in_model(mujoco, model, name)

    trace = ForceTrace([], [], [], [])
    for step in range(steps):
        data.ctrl[0] = _close_control(step, steps, close_control)
        mujoco.mj_step(model, data)
        trace.time_s.append(float(data.time))
        trace.control.append(float(data.ctrl[0]))
        trace.left.append(_taxel_surface_force_vector(data, "left", sensor_name))
        trace.right.append(_taxel_surface_force_vector(data, "right", sensor_name))
    return trace


def _run_touch_grid(mujoco, xml: Path, steps: int, close_control: float) -> ForceTrace:
    model = load_grasp_model(None, xml)
    data = mujoco.MjData(model)
    left_sensor = gripper_name_in_model(mujoco, model, "touch_left")
    right_sensor = gripper_name_in_model(mujoco, model, "touch_right")
    left_shape = _touch_grid_shape(mujoco, model, left_sensor)
    right_shape = _touch_grid_shape(mujoco, model, right_sensor)

    trace = ForceTrace([], [], [], [])
    for step in range(steps):
        data.ctrl[0] = _close_control(step, steps, close_control)
        mujoco.mj_step(model, data)
        left = _read_tactile(data, left_sensor, left_shape).sum(axis=(1, 2))
        right = _read_tactile(data, right_sensor, right_shape).sum(axis=(1, 2))
        trace.time_s.append(float(data.time))
        trace.control.append(float(data.ctrl[0]))
        trace.left.append(tuple(float(value) for value in left))
        trace.right.append(tuple(float(value) for value in right))
    return trace


def run_comparison(
    box_xml: Path = DEFAULT_BOX_TAXEL_XML,
    grid_xml: Path = DEFAULT_TOUCH_GRID_XML,
    steps: int = 1500,
    close_control: float = 220.0,
) -> tuple[ForceTrace, ForceTrace]:
    """以相同初态和控制轨迹分别运行两个触觉模型。"""
    if steps <= 0:
        raise ValueError("steps 必须为正整数。")
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error
    return (
        _run_box_taxels(mujoco, box_xml, steps, close_control),
        _run_touch_grid(mujoco, grid_xml, steps, close_control),
    )


def _pressure(force: tuple[float, float, float]) -> float:
    return max(0.0, force[2])


def _relative_error(first: float, second: float) -> float:
    """返回对称相对误差，避免任一模型被隐式指定为真值。"""
    return abs(first - second) / max(0.5 * (abs(first) + abs(second)), 1e-12)


def summarize_side(
    box_forces: list[tuple[float, float, float]],
    grid_forces: list[tuple[float, float, float]],
) -> SideSummary:
    """比较末段 20% 的稳态法向力及全过程峰值。"""
    if not box_forces or len(box_forces) != len(grid_forces):
        raise ValueError("两个模型必须包含相同数量的非空样本。")
    steady_count = max(1, len(box_forces) // 5)
    box_pressure = [_pressure(force) for force in box_forces]
    grid_pressure = [_pressure(force) for force in grid_forces]
    box_steady = fmean(box_pressure[-steady_count:])
    grid_steady = fmean(grid_pressure[-steady_count:])
    return SideSummary(
        box_steady_n=box_steady,
        grid_steady_n=grid_steady,
        relative_error=_relative_error(box_steady, grid_steady),
        box_peak_n=max(box_pressure),
        grid_peak_n=max(grid_pressure),
    )


def write_comparison_csv(
    path: Path, box: ForceTrace, grid: ForceTrace, record_every: int = 1
) -> None:
    """写出采用公共“压缩 Fz 为正”约定的联合逐步数据。"""
    if record_every <= 0:
        raise ValueError("record_every 必须为正整数。")
    if len(box.time_s) != len(grid.time_s):
        raise ValueError("两个模型的轨迹长度不一致。")
    fieldnames = ["step", "time_s", "control"]
    for model_name in ("box_taxel", "touch_grid"):
        for side in ("left", "right"):
            fieldnames.extend(
                (
                    f"{model_name}_{side}_fx",
                    f"{model_name}_{side}_fy",
                    f"{model_name}_{side}_fz",
                    f"{model_name}_{side}_pressure_n",
                )
            )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        last_step = len(box.time_s) - 1
        for step in range(len(box.time_s)):
            if step % record_every and step != last_step:
                continue
            row: dict[str, float | int] = {
                "step": step,
                "time_s": box.time_s[step],
                "control": box.control[step],
            }
            for model_name, trace in (("box_taxel", box), ("touch_grid", grid)):
                for side in ("left", "right"):
                    force = getattr(trace, side)[step]
                    for axis, value in zip(("x", "y", "z"), force):
                        row[f"{model_name}_{side}_f{axis}"] = value
                    row[f"{model_name}_{side}_pressure_n"] = _pressure(force)
            writer.writerow(row)


def plot_comparison(path: Path, box: ForceTrace, grid: ForceTrace) -> None:
    """使用 SciencePlots 绘制控制量与左右法向力叠加曲线。"""
    try:
        import matplotlib.pyplot as plt
        import scienceplots  # noqa: F401 -- 导入后注册 science 样式。
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    plt.style.use(["science", "no-latex"])
    figure, (control_axis, left_axis, right_axis) = plt.subplots(
        3, 1, figsize=(7.0, 6.5), sharex=True, layout="constrained"
    )
    control_axis.plot(box.time_s, box.control, color="black", label="control")
    control_axis.set_ylabel("Control")
    control_axis.legend(frameon=False)
    for axis, side, title in (
        (left_axis, "left", "Left normal pressure"),
        (right_axis, "right", "Right normal pressure"),
    ):
        axis.plot(
            box.time_s,
            [_pressure(force) for force in getattr(box, side)],
            label="box taxel",
        )
        axis.plot(
            grid.time_s,
            [_pressure(force) for force in getattr(grid, side)],
            linestyle="--",
            label="touch grid",
        )
        axis.set_ylabel("Force (N)")
        axis.set_title(title)
        axis.legend(frameon=False)
    right_axis.set_xlabel("Simulation time (s)")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=300)
    plt.close(figure)


def _print_summary(left: SideSummary, right: SideSummary, tolerance: float) -> bool:
    print("side   box steady   grid steady   rel. error   box peak   grid peak")
    for side, summary in (("left", left), ("right", right)):
        print(
            f"{side:5s} {summary.box_steady_n:10.3f} N {summary.grid_steady_n:10.3f} N "
            f"{summary.relative_error:9.2%} {summary.box_peak_n:10.3f} N "
            f"{summary.grid_peak_n:10.3f} N"
        )
    passed = left.relative_error <= tolerance and right.relative_error <= tolerance
    print(f"result: {'PASS' if passed else 'FAIL'} (tolerance={tolerance:.1%})")
    return passed


def main() -> int:
    """运行比较，保存联合 CSV 和叠加曲线，并按容差返回状态。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--box-xml", type=Path, default=DEFAULT_BOX_TAXEL_XML)
    parser.add_argument("--grid-xml", type=Path, default=DEFAULT_TOUCH_GRID_XML)
    parser.add_argument("--steps", type=int, default=1500)
    parser.add_argument("--close-control", type=float, default=220.0)
    parser.add_argument("--record-every", type=int, default=1)
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV)
    parser.add_argument("--output-plot", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.10)
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps 必须为正整数。")
    if args.record_every <= 0:
        parser.error("--record-every 必须为正整数。")
    if args.tolerance < 0:
        parser.error("--tolerance 不得为负数。")

    box, grid = run_comparison(args.box_xml, args.grid_xml, args.steps, args.close_control)
    output_plot = args.output_plot or args.output_csv.with_suffix(".png")
    write_comparison_csv(args.output_csv, box, grid, args.record_every)
    plot_comparison(output_plot, box, grid)
    print(f"CSV:  {args.output_csv}")
    print(f"Plot: {output_plot}")
    left = summarize_side(box.left, grid.left)
    right = summarize_side(box.right, grid.right)
    return 0 if _print_summary(left, right, args.tolerance) else 1


if __name__ == "__main__":
    raise SystemExit(main())

"""``dmgripper-plot`` 命令行：绘制或离线重绘既有运行目录。

默认在源目录生成 ``plot.pdf``／``plot.png``；``--repaint`` 写入独占
重绘目录，保留源 trace 与原 manifest。历史 cup 记录（v1／v2）按字段
名兼容读取。``--phase``／``--task-time`` 只能配合 ``--repaint`` 做
窗口化评价图，避免覆盖源目录的整段图。
"""

from __future__ import annotations

import sys
from collections.abc import Sequence
from pathlib import Path

_USAGE = "用法：dmgripper-plot [--repaint] [--phase 阶段1,阶段2] [--task-time 起:止] <运行目录>"


def run(argv: Sequence[str] | None = None) -> int:
    """解析目录、重绘开关与可选窗口参数并执行绘图。"""
    arguments = list(sys.argv[1:] if argv is None else argv)
    repaint = "--repaint" in arguments
    phases: tuple[str, ...] | None = None
    task_time_range: tuple[float, float] | None = None
    directories: list[str] = []
    index = 0
    while index < len(arguments):
        item = arguments[index]
        index += 1
        if item == "--repaint":
            continue
        if item in {"--phase", "--task-time"}:
            if index >= len(arguments):
                print(f"{item} 缺少取值；{_USAGE}", file=sys.stderr)
                return 2
            value = arguments[index]
            index += 1
        elif item.startswith("--"):
            print(f"未知参数 {item}；{_USAGE}", file=sys.stderr)
            return 2
        else:
            directories.append(item)
            continue
        if item == "--phase":
            names = tuple(name.strip() for name in value.split(",") if name.strip())
            if not names:
                print("--phase 至少需要一个非空阶段名", file=sys.stderr)
                return 2
            phases = names
        else:
            parts = value.split(":", maxsplit=1)
            try:
                bounds = tuple(float(part) for part in parts)  # type: ignore[arg-type]
            except ValueError:
                bounds = ()
            if len(bounds) != 2:
                print(f"--task-time 需要 起:止 形式的数值区间；{_USAGE}", file=sys.stderr)
                return 2
            task_time_range = (bounds[0], bounds[1])
    if len(directories) != 1:
        print(_USAGE, file=sys.stderr)
        return 2
    if (phases is not None or task_time_range is not None) and not repaint:
        print(
            "指定 --phase／--task-time 时必须配合 --repaint，避免覆盖源目录整段图。",
            file=sys.stderr,
        )
        return 2
    directory = Path(directories[0])
    from .plotting import plot_experiment_run, read_trace_rows, repaint_run

    if not read_trace_rows(directory):
        print(f"目录没有可绘制数据：{directory}", file=sys.stderr)
        return 1
    if repaint:
        try:
            output = repaint_run(directory, phases=phases, task_time_range=task_time_range)
        except ValueError as error:
            print(str(error), file=sys.stderr)
            return 1
        print(output / "plot.pdf")
        print(output / "plot.png")
    else:
        for path in plot_experiment_run(directory):
            print(path)
    return 0


def main(argv: Sequence[str] | None = None) -> None:
    """命令行入口。"""
    raise SystemExit(run(argv))

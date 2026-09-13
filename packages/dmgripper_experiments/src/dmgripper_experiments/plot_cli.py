"""``dmgripper-plot`` 命令行：绘制或离线重绘既有运行目录。

默认在源目录生成 ``plot.pdf``／``plot.png``；``--repaint`` 写入独占
重绘目录，保留源 trace 与原 manifest。历史 cup 记录（v1／v2）按字段
名兼容读取。
"""

from __future__ import annotations

import sys
from collections.abc import Sequence
from pathlib import Path


def run(argv: Sequence[str] | None = None) -> int:
    """解析目录与 ``--repaint`` 开关并执行绘图。"""
    arguments = list(sys.argv[1:] if argv is None else argv)
    repaint = "--repaint" in arguments
    directories = [item for item in arguments if item != "--repaint"]
    if len(directories) != 1:
        print("用法：dmgripper-plot [--repaint] <运行目录>", file=sys.stderr)
        return 2
    directory = Path(directories[0])
    from .plotting import plot_experiment_run, read_trace_rows, repaint_run

    if not read_trace_rows(directory):
        print(f"目录没有可绘制数据：{directory}", file=sys.stderr)
        return 1
    if repaint:
        output = repaint_run(directory)
        print(output / "plot.pdf")
        print(output / "plot.png")
    else:
        for path in plot_experiment_run(directory):
            print(path)
    return 0


def main(argv: Sequence[str] | None = None) -> None:
    """运行绘图命令行。"""
    raise SystemExit(run(argv))

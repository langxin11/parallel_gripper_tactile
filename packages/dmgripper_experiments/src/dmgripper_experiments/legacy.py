"""旧实验入口的过渡兼容层。

只做迁移提示、配置转换诊断或委托，不重建任何 cup 专用流程；旧
``--execute`` 一律明确拒绝并给出新命令，避免旧场景交互被无声映射
成自动阶段推进。
"""

from __future__ import annotations

import sys
from collections.abc import Sequence

_CUP_COMMAND = "uv run --package dmgripper-experiments dmgripper-run --config configs/hardware/dmgripper/adaptive_grip.yaml"
_DEMO_COMMAND = "uv run --package dmgripper-experiments dmgripper-run --config configs/hardware/dmgripper/force_curve.yaml"
_PLOT_COMMAND = "uv run --package dmgripper-experiments dmgripper-plot <运行目录>"


def _reject(old_name: str, new_command: str) -> int:
    """打印迁移提示并拒绝执行。"""
    print(
        f"{old_name} 已被通用抓取实验入口取代。\n"
        f"请审阅新的生命周期配置后改用：\n  {new_command}\n"
        "旧场景交互不会映射成自动阶段推进，因此旧 --execute 不再可用。",
        file=sys.stderr,
    )
    return 2


def cup_main(argv: Sequence[str] | None = None) -> None:
    """旧 ``dmgripper-cup`` 入口：拒绝执行并提示新命令。"""
    raise SystemExit(_reject("dmgripper-cup", _CUP_COMMAND))


def force_demo_main(argv: Sequence[str] | None = None) -> None:
    """旧 ``dmgripper-force-demo`` 入口：拒绝执行并提示新命令。"""
    raise SystemExit(_reject("dmgripper-force-demo", _DEMO_COMMAND))


def cup_plot_main(argv: Sequence[str] | None = None) -> None:
    """旧 ``dmgripper-cup-plot`` 的薄别名：委托通用历史读取器。

    兼容旧 cup 的 v1／v2 trace 字段（``state`` 列自动回填 ``phase``），
    在源目录内生成 ``plot.pdf``／``plot.png``，不保留第二套绘图。
    """
    from .plotting import plot_experiment_run, read_trace_rows

    directories = list(argv if argv is not None else sys.argv[1:])
    if len(directories) != 1:
        print(f"用法：dmgripper-cup-plot <运行目录>\n新入口：{_PLOT_COMMAND}", file=sys.stderr)
        raise SystemExit(2)
    directory = directories[0]
    if not read_trace_rows(directory):
        print(f"目录没有可绘制数据：{directory}", file=sys.stderr)
        raise SystemExit(1)
    for path in plot_experiment_run(directory):
        print(path)

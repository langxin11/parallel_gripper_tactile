"""重绘既有运行目录的薄工具：调用包内绘图，不另实现绘图逻辑。"""

from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> None:
    """委托 ``dmgripper_experiments.plot_cli.run``。"""
    from dmgripper_experiments.plot_cli import run

    raise SystemExit(run(argv if argv is not None else sys.argv[1:]))


if __name__ == "__main__":
    main()

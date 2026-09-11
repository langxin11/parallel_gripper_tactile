"""从已有单次力跟踪目录重绘学术风格诊断图。"""

from __future__ import annotations

import argparse
from pathlib import Path

from parallel_gripper_tactile.runners.rendering import render_force_tracking_run


def main() -> None:
    """解析重绘选项，将实际生成的独占目录输出到终端。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path, help="已有单次力跟踪目录。")
    parser.add_argument("--output-dir", type=Path, help="新建输出目录，不覆盖已有目录。")
    parser.add_argument("--tactile-detail", action="store_true", help="绘制可用的逐 taxel 细节。")
    parser.add_argument("--format", choices=("png", "pdf", "both"), default="png")
    args = parser.parse_args()
    formats = ("png", "pdf") if args.format == "both" else (args.format,)
    output = render_force_tracking_run(
        args.run_directory,
        output_directory=args.output_dir,
        tactile_detail=args.tactile_detail,
        formats=formats,
    )
    print(f"重绘产物：{output}")


if __name__ == "__main__":
    main()

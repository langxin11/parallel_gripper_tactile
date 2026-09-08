"""录制带实时曲线叠加的摩擦估计与 Ramp 力跟踪演示 MP4。

在组会等场合展示时，可把右侧面板的三行实时曲线与左侧 MuJoCo 场景
合成一段 16:9 演示视频。录制只读快照、不产生实验 trace，产物默认写入
``outputs/demos/``（不入库）。需要 ffmpeg 可用。

用法示例：

.. code-block:: bash

    uv run python scripts/demos/record_experiment_demos.py --demo both
    uv run python scripts/demos/record_experiment_demos.py --demo ramp --fps 60
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path


def _parser() -> argparse.ArgumentParser:
    """构造演示录制命令的参数解析器。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo",
        choices=("ramp", "friction", "both"),
        default="both",
        help="要录制的演示：ramp 力跟踪、friction 摩擦估计或两者（默认 both）。",
    )
    parser.add_argument(
        "--profile",
        type=Path,
        default="configs/custom_parallel_gripper.yaml",
        help="自研夹爪 profile 路径（默认 configs/custom_parallel_gripper.yaml）。",
    )
    parser.add_argument(
        "--ramp-task",
        type=Path,
        default="configs/force_tracking/ramp.yaml",
        help="Ramp 力跟踪任务 YAML（默认 configs/force_tracking/ramp.yaml）。",
    )
    parser.add_argument(
        "--friction-task",
        type=Path,
        default="configs/friction_estimation/nominal_friction.yaml",
        help="摩擦估计任务 YAML（默认 nominal_friction.yaml）。",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default="outputs/demos",
        help="MP4 输出目录（默认 outputs/demos）。",
    )
    parser.add_argument("--width", type=int, default=1920, help="画面总宽（默认 1920）。")
    parser.add_argument("--height", type=int, default=1080, help="画面高（默认 1080）。")
    parser.add_argument("--fps", type=int, default=30, help="视频帧率（默认 30）。")
    parser.add_argument(
        "--panel-width",
        type=int,
        default=864,
        help="右侧实时曲线面板宽度（默认 864，配合 1920 总宽实现 MuJoCo:Data≈55:45）。",
    )
    parser.add_argument(
        "--no-arrow",
        action="store_true",
        help="摩擦演示不绘制场景中的切向外加载荷箭头。",
    )
    parser.add_argument(
        "--noise-seed",
        type=int,
        default=None,
        help="触觉噪声种子；缺省使用 profile 默认（可复现演示）。",
    )
    return parser


def _print_result(label: str, result: object) -> None:
    """按演示类型打印关键验收指标。"""
    if label == "ramp":
        print(
            f"[ramp] passed={bool(result.passed)} rmse={float(result.rmse_n):.3f} N "
            f"mae={float(result.mae_n):.3f} N "
            f"peak_error={float(result.peak_abs_error_n):.3f} N"
        )
    else:
        print(
            f"[friction] passed={bool(result.passed)} slip_detected={bool(result.slip_detected)} "
            f"mu_hat={float(result.estimated_friction_coefficient):.3f} "
            f"(true={float(result.true_friction_coefficient):.3f})"
        )


def main(argv: list[str] | None = None) -> None:
    """按命令行参数录制一个或两个演示视频。"""
    # 无界面环境优先使用 EGL 离屏渲染；必须在导入 mujoco 前设置。
    os.environ.setdefault("MUJOCO_GL", "egl")
    from parallel_gripper_tactile.experiments.demo_videos import (
        record_force_tracking_ramp_video,
        record_friction_demo_video,
    )

    args = _parser().parse_args(argv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    demos = ("ramp", "friction") if args.demo == "both" else (args.demo,)
    for demo in demos:
        if demo == "ramp":
            result, frames = record_force_tracking_ramp_video(
                profile_path=Path(args.profile),
                task_path=Path(args.ramp_task),
                output=output_dir / "force_tracking_ramp.mp4",
                width=args.width,
                height=args.height,
                fps=args.fps,
                panel_width=args.panel_width,
                sensor_noise_seed=args.noise_seed,
            )
        else:
            result, frames = record_friction_demo_video(
                profile_path=Path(args.profile),
                task_path=Path(args.friction_task),
                output=output_dir / "friction_estimation_with_curves.mp4",
                width=args.width,
                height=args.height,
                fps=args.fps,
                panel_width=args.panel_width,
                sensor_noise_seed=args.noise_seed,
                draw_arrow=not args.no_arrow,
            )
        _print_result(demo, result)
        print(f"[{demo}] frames={frames}")


if __name__ == "__main__":
    main()

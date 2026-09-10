"""验证控制器对比 protocol 的聚合、矩阵说明和图表生成。"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.force_tracking_ablation import SeedSweep
from parallel_gripper_tactile.studies.force_tracking_comparison import (
    ForceTrackingComparisonConfig,
)
from parallel_gripper_tactile.studies.tabular import write_rows_csv_and_parquet
from parallel_gripper_tactile.studies.protocols import (
    force_tracking_controller_comparison as protocol,
)


ROOT = Path(__file__).resolve().parents[1]


def _result_row(
    controller: str,
    *,
    seed: int,
    rmse: float,
    run_directory: str = "runs/example",
    passed: bool = True,
) -> dict[str, object]:
    """构造一行完整的合成 study 结果。"""
    return {
        "controller_variant": controller,
        "task_name": "step_force_tracking",
        "task_path": "step.yaml",
        "object_material": "hard",
        "sensor_noise_seed": seed,
        "passed": passed,
        "run_directory": run_directory,
        "contact_time_s": 1.0,
        "tracking_start_time_s": 1.2,
        "tracking_duration_s": 4.0,
        "rmse_n": rmse,
        "mae_n": 0.1,
        "peak_abs_error_n": 0.3,
        "mean_error_n": 0.02,
        "final_error_n": 0.01,
        "torque_saturation_ratio": 0.0,
        "position_saturation_ratio": 0.0,
        "mean_estimated_stiffness_n_per_m": math.nan,
        "simulation_stable": True,
    }


def test_aggregate_rows_groups_controller_task_and_material() -> None:
    """聚合维度包含控制器、任务和材料，并忽略非有限值。"""
    rows = [
        _result_row("full", seed=0, rmse=0.2),
        _result_row("full", seed=1, rmse=0.4),
    ]

    aggregate = protocol.aggregate_rows(rows)[0]  # type: ignore[attr-defined]

    assert aggregate["controller_variant"] == "full"
    assert aggregate["task_name"] == "step_force_tracking"
    assert aggregate["object_material"] == "hard"
    assert aggregate["runs"] == 2
    assert aggregate["rmse_n_mean"] == pytest.approx(0.3)
    assert aggregate["rmse_n_std"] == pytest.approx(2**0.5 * 0.1)
    assert aggregate["mean_estimated_stiffness_n_per_m_mean"] is None


def test_describe_conditions_is_stable_and_complete(tmp_path: Path) -> None:
    """dry-run 文本包含条件总数和每个矩阵维度。"""
    config = ForceTrackingComparisonConfig(
        profile=tmp_path / "profile.yaml",
        tasks=(tmp_path / "step.yaml",),
        controllers=("pid-only", "full"),
        materials=("soft", "hard"),
        seeds=SeedSweep(start=3, count=2),
        output_root=tmp_path / "outputs",
    )

    description = protocol.describe_conditions(config)  # type: ignore[attr-defined]

    assert "Conditions: 8" in description
    assert "001 controller=pid-only task=step.yaml material=soft seed=3" in description
    assert "008 controller=full task=step.yaml material=hard seed=4" in description


def test_summary_plots_are_generated(tmp_path: Path, fast_plot_render: None) -> None:
    """聚合指标、饱和比例和消融增量均生成非空图片。"""
    controllers = ("pid-only", "pid-torque-ff", "pid-stiffness-ff", "full")
    aggregates = protocol.aggregate_rows(  # type: ignore[attr-defined]
        [
            _result_row(controller, seed=0, rmse=0.2 + 0.05 * index)
            for index, controller in enumerate(controllers)
        ]
    )
    outputs = (
        tmp_path / "metrics.png",
        tmp_path / "saturation.png",
        tmp_path / "delta.png",
    )

    protocol.plot_metric_summary(  # type: ignore[attr-defined]
        aggregates, outputs[0], controller_order=controllers
    )
    protocol.plot_saturation_summary(  # type: ignore[attr-defined]
        aggregates, outputs[1], controller_order=controllers
    )
    protocol.plot_ablation_delta(  # type: ignore[attr-defined]
        aggregates, outputs[2], controller_order=controllers
    )

    artifacts = (*outputs, *(path.with_suffix(".pdf") for path in outputs))
    assert all(path.is_file() and path.stat().st_size > 0 for path in artifacts)


def _write_trace(path: Path, *, offset: float) -> None:
    """写入供轨迹叠加测试使用的最小 trace。"""
    path.parent.mkdir(parents=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "phase",
                "tracking_time_s",
                "target_normal_force_n",
                "filtered_normal_force_n",
            ),
        )
        writer.writeheader()
        writer.writerows(
            (
                {
                    "phase": "track_reference",
                    "tracking_time_s": 0.0,
                    "target_normal_force_n": 1.0,
                    "filtered_normal_force_n": 1.0 + offset,
                },
                {
                    "phase": "track_reference",
                    "tracking_time_s": 1.0,
                    "target_normal_force_n": 6.0,
                    "filtered_normal_force_n": 5.8 + offset,
                },
            )
        )


def test_tracking_overlay_uses_common_seed(tmp_path: Path, fast_plot_render: None) -> None:
    """轨迹对比选择所有控制器共有的最小 seed。"""
    figures = tmp_path / "figures"
    figures.mkdir()
    controllers = ("pid-only", "full")
    rows = []
    for index, controller in enumerate(controllers):
        run_directory = f"runs/{controller}"
        _write_trace(tmp_path / run_directory / "trace.csv", offset=0.1 * index)
        rows.append(
            _result_row(
                controller,
                seed=2,
                rmse=0.2,
                run_directory=run_directory,
            )
        )

    outputs = protocol.plot_tracking_overlays(  # type: ignore[attr-defined]
        rows,
        tmp_path,
        figures,
        controller_order=controllers,
    )

    assert len(outputs) == 2
    assert outputs[0].suffix == ".png"
    assert outputs[1].suffix == ".pdf"
    assert all(path.is_file() and path.stat().st_size > 0 for path in outputs)


def test_tracking_rows_prefer_parquet_over_legacy_csv(tmp_path: Path) -> None:
    """轨迹读取在新旧文件同时存在时优先使用 Parquet。"""
    run_directory = tmp_path / "runs" / "example"
    run_directory.mkdir(parents=True)
    write_rows_csv_and_parquet(
        run_directory / "trace.csv",
        [
            {
                "phase": "track_reference",
                "tracking_time_s": 3.0,
                "target_normal_force_n": 7.0,
                "filtered_normal_force_n": 6.5,
            }
        ],
    )
    with (run_directory / "trace.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "phase",
                "tracking_time_s",
                "target_normal_force_n",
                "filtered_normal_force_n",
            ),
        )
        writer.writeheader()
        writer.writerow(
            {
                "phase": "track_reference",
                "tracking_time_s": 0.0,
                "target_normal_force_n": 1.0,
                "filtered_normal_force_n": 1.0,
            }
        )

    rows = protocol._read_tracking_rows(run_directory)  # type: ignore[attr-defined]

    assert rows == [
        {
            "phase": "track_reference",
            "tracking_time_s": 3.0,
            "target_normal_force_n": 7.0,
            "filtered_normal_force_n": 6.5,
        }
    ]


def test_tracking_overlay_skips_failed_conditions_without_trace(tmp_path: Path) -> None:
    """未形成跟踪段的失败条件不阻断其他 study 产物生成。"""
    figures = tmp_path / "figures"
    figures.mkdir()
    rows = [
        _result_row("pid-only", seed=0, rmse=math.inf, passed=False),
        _result_row("full", seed=0, rmse=0.2, run_directory="runs/full"),
    ]
    _write_trace(tmp_path / "runs/full/trace.csv", offset=0.0)

    outputs = protocol.plot_tracking_overlays(  # type: ignore[attr-defined]
        rows,
        tmp_path,
        figures,
        controller_order=("pid-only", "full"),
    )

    assert outputs == []

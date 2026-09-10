"""验证力跟踪因果诊断的图表产物与失败 run 隔离。"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingResult
from parallel_gripper_tactile.studies.protocols import (
    force_tracking_diagnosis as protocol,
)
from parallel_gripper_tactile.studies.force_tracking_diagnosis import (
    CollisionGeometryCondition,
    DiagnosisConfig,
)
from parallel_gripper_tactile.studies.tabular import write_rows_csv_and_parquet


ROOT = Path(__file__).resolve().parents[1]


def _result(*, passed: bool) -> ForceTrackingResult:
    """构造不需要 MuJoCo 的最小力跟踪结果。"""
    return ForceTrackingResult(
        contact_time_s=1.0,
        tracking_start_time_s=1.2,
        tracking_duration_s=2.0,
        rmse_n=0.2 if passed else math.inf,
        mae_n=0.1,
        peak_abs_error_n=0.4,
        mean_error_n=0.02,
        final_error_n=0.01,
        torque_saturation_ratio=0.1,
        position_saturation_ratio=0.2,
        mean_estimated_stiffness_n_per_m=300.0,
        rise_time_s=None,
        overshoot_ratio=None,
        settling_time_s=None,
        simulation_stable=passed,
    )


def _diagnosis_row(
    label: str,
    *,
    run_directory: str,
    force_scale: float,
    passed: bool = True,
) -> dict[str, object]:
    """构造用于诊断图的合成汇总行。"""
    return {
        "label": label,
        "run_directory": run_directory,
        "passed": passed,
        "force_scale": force_scale,
        "max_position_adjustment_rad": 0.1,
        "pid_ki": 0.1,
        "filter_cutoff_hz": 40.0,
        "rmse_n": 0.2 * force_scale,
        "peak_abs_error_n": 0.4 * force_scale,
        "torque_saturation_ratio": 0.1,
        "position_saturation_ratio": 0.2,
        "contact_collapse_events": 1,
    }


def _write_trace(path: Path, *, parquet: bool) -> None:
    """写入包含目标力和滤波力的最小跟踪段。"""
    rows = [
        {
            "phase": "track_reference",
            "tracking_time_s": 0.0,
            "target_normal_force_n": 1.0,
            "filtered_normal_force_n": 0.9,
        },
        {
            "phase": "track_reference",
            "tracking_time_s": 1.0,
            "target_normal_force_n": 4.0,
            "filtered_normal_force_n": 3.8,
        },
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    if parquet:
        write_rows_csv_and_parquet(path, rows)
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def test_render_phase_figures_uses_numeric_x_and_skips_missing_traces(
    tmp_path: Path, fast_plot_render: None
) -> None:
    """数值扫描输出双格式图表，缺失 trace 的失败运行不会中止绘图。"""
    _write_trace(tmp_path / "runs" / "parquet" / "trace.csv", parquet=True)
    rows = [
        _diagnosis_row("scale-0.500", run_directory="runs/parquet", force_scale=0.5),
        _diagnosis_row("scale-1.000", run_directory="runs/missing", force_scale=1.0, passed=False),
    ]

    artifacts = protocol.render_phase_figures(rows, tmp_path, phase="force-scale")

    expected = {
        "figures/diagnostic_metrics.png",
        "figures/diagnostic_metrics.pdf",
        "figures/tracking_overlay.png",
        "figures/tracking_overlay.pdf",
    }
    assert {str(path.relative_to(tmp_path)) for path in artifacts} == expected
    assert all(path.is_file() and path.stat().st_size > 0 for path in artifacts)


def test_historical_summary_infers_passed_from_stability_and_finite_rmse() -> None:
    """旧汇总缺少 passed 字段时，仍可识别稳定且指标有限的有效运行。"""
    assert protocol._row_passed({"simulation_stable": True, "rmse_n": 0.2})
    assert not protocol._row_passed({"simulation_stable": False, "rmse_n": 0.2})
    assert not protocol._row_passed({"simulation_stable": True, "rmse_n": math.inf})


def test_run_phase_registers_figures_and_keeps_failed_run_without_trace(
    tmp_path, monkeypatch, fast_plot_render: None
) -> None:
    """phase 收尾登记 PNG/PDF，并让没有跟踪段的失败 run 留在 manifest。"""
    profile = tmp_path / "profile.yaml"
    task = tmp_path / "task.yaml"
    config_source = tmp_path / "diagnosis.yaml"
    collision_model = tmp_path / "unused.xml"
    collision_model.write_text("<mujoco/>", encoding="utf-8")
    profile.write_text(
        """
model:
  path: model.xml
control:
  force:
    kp: 1.0
    ki: 0.1
    kd: 0.01
    max_position_adjustment: 0.1
    filter_cutoff_hz: 40.0
    stiffness:
      initial_n_per_m: 10.0
      min_n_per_m: 1.0
      max_n_per_m: 100.0
      min_delta_force_n: 0.1
""".lstrip(),
        encoding="utf-8",
    )
    task.write_text(
        """
approach:
  feedforward_force_n: 1.0
reference:
  waypoints:
    - t_s: 0.0
      force_n: 2.0
""".lstrip(),
        encoding="utf-8",
    )
    config_source.write_text("name: 合成诊断\n", encoding="utf-8")
    config = DiagnosisConfig(
        name="synthetic",
        profile=profile,
        task=task,
        output_root=tmp_path / "outputs",
        controllers=("pid-only", "full"),
        collision_geometry_models=(
            CollisionGeometryCondition(label="unused", model=collision_model),
        ),
    )

    def fake_execute_force_tracking(*, output_root: Path, run_prefix: str, **_kwargs: object):
        run_path = output_root / run_prefix
        run_path.mkdir(parents=True)
        passed = run_prefix == "full"
        if passed:
            _write_trace(run_path / "trace.csv", parquet=False)
        return SimpleNamespace(path=run_path), _result(passed=passed)

    monkeypatch.setattr(protocol, "execute_force_tracking", fake_execute_force_tracking)
    study_dir = protocol.run_study(
        config,
        phase="controllers",
        config_source=config_source,
        study_directory=tmp_path / "study",
    )

    manifest = json.loads((study_dir / "study_manifest.json").read_text(encoding="utf-8"))
    assert set(manifest["failed_runs"]) == {"runs/pid-only"}
    assert {
        "figures/diagnostic_metrics.png",
        "figures/diagnostic_metrics.pdf",
        "figures/tracking_overlay.png",
        "figures/tracking_overlay.pdf",
    } <= set(manifest["artifacts"])
    assert all((study_dir / artifact).is_file() for artifact in manifest["artifacts"])
    summary = json.loads((study_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["runs"][0]["contact_collapse_events"] is None

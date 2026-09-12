"""验证摩擦估计 runner 的单次绘图工件策略。"""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
from typing import Literal

import pytest

from parallel_gripper_tactile.experiments.friction_estimation import (
    FrictionEstimationResult,
    FrictionEstimationTask,
    run_friction_estimation,
)
from parallel_gripper_tactile.runners import friction_estimation as runner
from parallel_gripper_tactile.visualization import friction as friction_plot


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/dm_gripper.yaml"
TASK_PATH = ROOT / "configs/task/friction_estimation/nominal_friction.yaml"


def _result() -> FrictionEstimationResult:
    """构造通过全部 runner 级验收的最小结果。"""
    return FrictionEstimationResult(
        true_friction_coefficient=0.4,
        estimated_friction_coefficient=0.35,
        raw_friction_coefficient=0.36,
        estimate_ratio=0.875,
        absolute_estimation_error=0.05,
        slip_detected=True,
        using_fallback=False,
        probe_detection_time_s=2.5,
        probe_force_at_detection_n=1.0,
        max_probe_displacement_m=0.001,
        max_hold_displacement_m=0.002,
        hold_force_tracking_rmse_n=0.05,
        minimum_hold_friction_margin_n=0.5,
        peak_active_taxel_count=4,
        peak_local_friction_ratio=0.9,
        local_weighted_ratio_at_detection=0.85,
        local_ratio_p90_at_detection=0.9,
        local_slip_detection_time_s=2.0,
        local_slip_detected_taxel_count=2,
        local_left_friction_estimate=0.35,
        local_right_friction_estimate=None,
        simulation_stable=True,
        detection_passed=True,
        conservatism_passed=True,
        informativeness_passed=True,
        probe_slip_passed=True,
        hold_slip_passed=True,
        force_tracking_passed=True,
        detection_reason="合成结果。",
        detection_features={},
        relative_estimation_error=0.125,
        probe_displacement_limit_exceeded=False,
        hold_succeeded=True,
    )


@pytest.mark.parametrize(
    ("plot_mode", "result_passed", "simulation_stable", "diagnostic_failure", "expected"),
    [
        ("summary", True, True, False, {"plot.png"}),
        ("diagnostic", True, True, False, {"plot.png", "taxel_plot.png"}),
        ("none", True, True, False, set()),
        ("none", True, True, True, {"plot.png", "taxel_plot.png"}),
        ("none", False, True, False, set()),
        ("none", True, False, False, {"plot.png", "taxel_plot.png"}),
    ],
)
def test_execute_friction_estimation_keeps_requested_or_failure_diagnostics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    plot_mode: Literal["summary", "diagnostic", "none"],
    result_passed: bool,
    simulation_stable: bool,
    diagnostic_failure: bool,
    expected: set[str],
) -> None:
    """按 protocol 验收选择诊断，运行不稳定条件始终保留证据。"""
    task = FrictionEstimationTask.load(TASK_PATH)

    def fake_run(*_args: object, **kwargs: object) -> FrictionEstimationResult:
        Path(str(kwargs["output_csv"])).write_text("time_s\n0\n", encoding="utf-8")
        result = replace(
            _result(),
            detection_passed=result_passed,
            simulation_stable=simulation_stable,
        )
        kwargs["on_result"]([], result)
        return result

    monkeypatch.setattr(runner, "run_friction_estimation", fake_run)
    monkeypatch.setattr(
        friction_plot, "plot_summary", lambda path, _rows, **_kwargs: path.write_bytes(b"plot")
    )
    monkeypatch.setattr(
        friction_plot,
        "plot_taxel_diagnostics",
        lambda path, _rows, **_kwargs: path.write_bytes(b"taxel"),
    )
    run, _ = runner.execute_friction_estimation(
        profile=PROFILE,
        task_path=TASK_PATH,
        estimation_task=task,
        output_root=tmp_path,
        run_name="policy",
        plot_mode=plot_mode,
        diagnostic_on_result=(lambda _result: diagnostic_failure),
    )

    manifest = json.loads((run.path / "manifest.json").read_text(encoding="utf-8"))
    artifacts = {Path(artifact).name for artifact in manifest["artifacts"]}
    assert artifacts & {"plot.png", "taxel_plot.png"} == expected
    assert manifest["parameters"]["plot_mode"] == plot_mode


def test_run_friction_estimation_preserves_taxel_only_legacy_output(
    tmp_path: Path, fast_png_render: None
) -> None:
    """未传结果回调时，taxel 图仍可独立于主图生成。"""
    task = FrictionEstimationTask.load(TASK_PATH)
    taxel_plot = tmp_path / "taxel_only.png"

    run_friction_estimation(PROFILE, task=task, output_taxel_plot=taxel_plot)

    assert taxel_plot.is_file()
    assert taxel_plot.stat().st_size > 0

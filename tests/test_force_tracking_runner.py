"""验证单次力跟踪 runner 的可复现工件边界。"""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import pytest
import pyarrow as pa
import pyarrow.parquet as pq
from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingResult
from parallel_gripper_tactile.config.profiles import TorqueAdrcControl
from parallel_gripper_tactile.runners import force_tracking


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    ("plot_mode", "stable", "study_failure", "expected_plots"),
    [
        ("summary", True, False, {"tracking"}),
        ("diagnostic", True, False, {"tracking", "tactile", "controller"}),
        ("none", True, False, set()),
        ("none", False, False, {"tracking", "tactile", "controller"}),
        ("none", True, True, {"tracking", "tactile", "controller"}),
    ],
)
def test_execute_force_tracking_writes_complete_run_artifacts(
    tmp_path: Path,
    monkeypatch,
    fast_plot_render: None,
    plot_mode: str,
    stable: bool,
    study_failure: bool,
    expected_plots: set[str],
) -> None:
    """runner 保存输入快照、结果、trace 及完整 manifest，而无需真实仿真。"""
    result = ForceTrackingResult(
        contact_time_s=1.0,
        tracking_start_time_s=1.2,
        tracking_duration_s=2.0,
        rmse_n=0.1,
        mae_n=0.08,
        peak_abs_error_n=0.2,
        mean_error_n=0.01,
        final_error_n=0.02,
        torque_saturation_ratio=0.0,
        position_saturation_ratio=0.0,
        mean_estimated_stiffness_n_per_m=100.0,
        rise_time_s=0.12,
        overshoot_ratio=0.04,
        settling_time_s=0.35,
        simulation_stable=True,
    )

    result = replace(result, simulation_stable=stable)

    def fake_run(
        *args: object, output_parquet: Path, on_result, **kwargs: object
    ) -> ForceTrackingResult:
        pq.write_table(pa.table({"time_s": [0.0]}), output_parquet, compression="zstd")
        rows = [
            {
                "time_s": t,
                "target_normal_force_n": 1.0,
                "filtered_normal_force_n": 0.9,
                "measured_left_fz": 0.9,
                "measured_right_fz": 0.9,
                "measured_left_fx": 0.1,
                "measured_left_fy": 0.0,
                "measured_right_fx": 0.1,
                "measured_right_fy": 0.0,
                "control": 0.5,
                "drive_position_rad": 0.4,
                "drive_velocity_rad_s": 0.0,
                "desired_velocity_rad_s": 0.0,
                "motor_torque_n_m": 0.1,
            }
            for t in (0.0, 1.0)
        ]
        on_result(rows, result)
        return result

    monkeypatch.setattr(force_tracking, "run_force_tracking", fake_run)
    profile = ROOT / "configs" / "dm_gripper.yaml"
    task = ROOT / "configs" / "task" / "force_tracking" / "default_waypoints.yaml"
    run, returned = force_tracking.execute_force_tracking(
        profile=profile,
        task_path=task,
        output_root=tmp_path,
        run_name="runner-test",
        plot_mode=plot_mode,
        diagnostic_on_result=lambda result: study_failure,
        object_material="soft",
        controller_variant="adrc-torque",
        stiffness_estimator_method="window_quadratic",
        sensor_noise_seed=7,
        torque_adrc_override=TorqueAdrcControl(measurement_filter_cutoff_hz=60.0),
    )

    assert returned == result
    manifest = json.loads((run.path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["parameters"]["controller_variant"] == "adrc-torque"
    assert manifest["parameters"]["stiffness_estimator_method"] == "window_quadratic"
    assert manifest["parameters"]["object_material"] == "soft"
    assert manifest["parameters"]["multiccd_enabled"] is True
    assert manifest["parameters"]["force_semantics"] == "average_side"
    assert manifest["parameters"]["sensor_noise_seed"] == 7
    assert manifest["parameters"]["trace_schema_version"] == 2
    assert manifest["parameters"]["trace_sample_period_s"] == 0.004
    assert manifest["parameters"]["trace_event_window_s"] == 0.2
    assert manifest["parameters"]["torque_adrc_override"]["measurement_filter_cutoff_hz"] == 60.0
    assert {
        "profile.yaml",
        "task.yaml",
        "effective_parameters.json",
        "trace.parquet",
        "metrics.json",
    } <= set(manifest["artifacts"])
    assert {path.stem for path in (run.path / "plots").glob("*.png")} == expected_plots
    assert {name for name in manifest["artifacts"] if name.startswith("plots/")} == {
        f"plots/{name}.png" for name in expected_plots
    }
    assert manifest["parameters"]["plot_mode"] == plot_mode
    assert (run.path / "task.yaml").read_bytes() == task.read_bytes()
    assert (
        pq.ParquetFile(run.path / "trace.parquet").metadata.row_group(0).column(0).compression
        == "ZSTD"
    )
    effective = json.loads((run.path / "effective_parameters.json").read_text(encoding="utf-8"))
    assert effective["schema_version"] == 1
    assert Path(effective["profile"]["model"]["path"]).is_absolute()
    assert Path(effective["profile"]["model"]["path"]).is_file()
    assert effective["task"]["name"] == "default_waypoint_force_tracking"
    assert effective["runtime"]["controller_variant"] == "adrc-torque"
    assert effective["runtime"]["object_material"] == "soft"
    assert effective["runtime"]["sensor_noise_seed"] == 7
    assert effective["runtime"]["trace_format"] == "parquet"
    assert effective["runtime"]["trace_compression"] == "zstd"
    assert effective["runtime"]["trace_sample_period_s"] == 0.004
    assert effective["runtime"]["trace_event_window_s"] == 0.2
    assert effective["runtime"]["torque_adrc_override"]["measurement_filter_cutoff_hz"] == 60.0

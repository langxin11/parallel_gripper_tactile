"""验证单次力跟踪 runner 的可复现工件边界。"""

from __future__ import annotations

import json
from pathlib import Path

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingResult
from parallel_gripper_tactile.profiles import TorqueAdrcControl
from parallel_gripper_tactile.runners import force_tracking


ROOT = Path(__file__).resolve().parents[1]


def test_execute_force_tracking_writes_complete_run_artifacts(tmp_path: Path, monkeypatch) -> None:
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

    def fake_run(
        *args: object, output_csv: Path, output_plot: Path, **kwargs: object
    ) -> ForceTrackingResult:
        output_csv.write_text("time_s\n0\n", encoding="utf-8")
        output_plot.write_bytes(b"plot")
        return result

    monkeypatch.setattr(force_tracking, "run_force_tracking", fake_run)
    profile = ROOT / "configs" / "custom_parallel_gripper.yaml"
    task = ROOT / "configs" / "force_tracking" / "default_waypoints.yaml"
    run, returned = force_tracking.execute_force_tracking(
        profile=profile,
        task_path=task,
        output_root=tmp_path,
        run_name="runner-test",
        object_material="soft",
        controller_variant="pid-only",
        stiffness_estimator_method="window_quadratic",
        sensor_noise_seed=7,
        torque_adrc_override=TorqueAdrcControl(measurement_filter_cutoff_hz=60.0),
    )

    assert returned == result
    manifest = json.loads((run.path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["parameters"]["controller_variant"] == "pid-only"
    assert manifest["parameters"]["stiffness_estimator_method"] == "window_quadratic"
    assert manifest["parameters"]["object_material"] == "soft"
    assert manifest["parameters"]["multiccd_enabled"] is True
    assert manifest["parameters"]["force_semantics"] == "average_side"
    assert manifest["parameters"]["sensor_noise_seed"] == 7
    assert manifest["parameters"]["torque_adrc_override"]["measurement_filter_cutoff_hz"] == 60.0
    assert {"profile.yaml", "task.yaml", "trace.csv", "plot.png", "metrics.json"} <= set(
        manifest["artifacts"]
    )
    assert (run.path / "task.yaml").read_bytes() == task.read_bytes()

"""带结构化运行工件的一次保守摩擦估计仿真。"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

from ..experiments.friction_estimation import (
    FrictionEstimationResult,
    FrictionEstimationTask,
    run_friction_estimation,
)
from ..config.profiles import load_profile
from ..artifacts import RunDirectory


def execute_friction_estimation(
    *,
    profile: Path,
    task_path: Path,
    estimation_task: FrictionEstimationTask | None = None,
    output_root: Path = Path("outputs"),
    run_name: str | None = None,
    run_prefix: str | None = None,
    run_suffix: str | None = None,
    sensor_noise_seed: int | None = None,
) -> tuple[RunDirectory, FrictionEstimationResult]:
    """运行一次探测—估计—调度仿真并保存全部复现产物。"""
    task = estimation_task or FrictionEstimationTask.load(task_path)
    configured = load_profile(profile)
    if configured.normal_force is None:
        raise ValueError("friction estimation requires profile control.force")
    if sensor_noise_seed is not None:
        if sensor_noise_seed < 0:
            raise ValueError("sensor_noise_seed must be non-negative")
        configured_force = configured.normal_force.model_copy(
            update={"sensor_noise_seed": sensor_noise_seed}
        )
        configured = configured.model_copy(
            update={"control": configured.control.model_copy(update={"force": configured_force})}
        )
    run = RunDirectory.create(
        output_root,
        profile_name=configured.name,
        experiment="friction-estimate",
        profile_source=profile,
        command=tuple(sys.argv),
        parameters={
            "task": str(task_path),
            "task_name": task.name,
            "true_friction_coefficient": float(task.friction_coefficient),
            "cube_mass_kg": float(task.cube_mass_kg),
            "sensor_noise_scale": float(task.sensor_noise_scale),
            "sensor_noise_seed": int(configured.normal_force.sensor_noise_seed),
            "probe": task.probe.model_dump(mode="json"),
            "estimator": task.estimator.model_dump(mode="json"),
            "taxel_observer": task.taxel_observer.model_dump(mode="json"),
            "taxel_slip_detector": task.taxel_slip_detector.model_dump(mode="json"),
            "scheduler": task.scheduler.model_dump(mode="json"),
            "solver": task.solver.model_dump(mode="json"),
        },
        run_name=run_name,
        run_prefix=run_prefix,
        run_suffix=run_suffix,
    )
    task_snapshot = run.artifact_path("task.yaml")
    task_snapshot.write_bytes(task_path.read_bytes())
    run.register_artifact(task_snapshot)
    effective_parameters_path = run.artifact_path("effective_parameters.json")
    effective_parameters_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile": configured.model_dump(mode="json"),
                "task": task.model_dump(mode="json"),
                "runtime": {
                    "profile_path": str(profile.resolve()),
                    "task_path": str(task_path.resolve()),
                    "estimator_kind": "tactile_only_contact_change_score",
                    "taxel_observer_kind": "contact_hysteresis_local_friction_ratio",
                    "taxel_slip_detector_kind": "force_ratio_saturation_and_redistribution",
                    "scheduler_kind": "estimated_friction",
                    "scheduler_input": "measured_tactile_shear",
                    "detector_inputs": ["taxel_normal", "taxel_shear", "tactile_history"],
                    "legacy_estimator_detection_parameters": "ignored; use tactile_slip",
                    "oracle_signals_used_by_estimator": [],
                    "sensor_noise_seed": int(configured.normal_force.sensor_noise_seed),
                },
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    run.register_artifact(effective_parameters_path)
    trace_path = run.artifact_path("trace.csv")
    plot_path = run.artifact_path("plot.png")
    taxel_plot_path = run.artifact_path("taxel_plot.png")
    result = run_friction_estimation(
        profile,
        task=task,
        output_csv=trace_path,
        output_plot=plot_path,
        output_taxel_plot=taxel_plot_path,
        sensor_noise_seed=sensor_noise_seed,
    )
    for artifact in (
        trace_path,
        plot_path,
        plot_path.with_suffix(".pdf"),
        taxel_plot_path,
        taxel_plot_path.with_suffix(".pdf"),
    ):
        run.register_artifact(artifact)
    metrics_path = run.artifact_path("metrics.json")
    metrics_path.write_text(
        json.dumps(asdict(result), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    run.register_artifact(metrics_path)
    run.finalize()
    return run, result


__all__ = ["execute_friction_estimation"]

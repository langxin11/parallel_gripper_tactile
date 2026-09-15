"""带结构化运行工件的一次目标力调度仿真。"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

import yaml

from ..experiments.force_scheduling import (
    ForceSchedulingResult,
    ForceSchedulingTask,
    run_force_scheduling,
)
from ..config.profiles import GripperProfile, load_profile, validate_resolved_profile
from ..artifacts import RunDirectory


def execute_force_scheduling(
    *,
    profile: Path | str,
    resolved_profile: GripperProfile | None = None,
    task_path: Path,
    scheduling_task: ForceSchedulingTask | None = None,
    output_root: Path = Path("outputs"),
    run_name: str | None = None,
    run_prefix: str | None = None,
    run_suffix: str | None = None,
) -> tuple[RunDirectory, ForceSchedulingResult]:
    """运行一次目标力调度仿真并保存任务、轨迹、图像和指标。"""
    task = scheduling_task or ForceSchedulingTask.load(task_path)
    configured = (
        validate_resolved_profile(resolved_profile)
        if resolved_profile is not None
        else load_profile(profile)
    )
    run = RunDirectory.create(
        output_root,
        profile_name=configured.name,
        experiment="force-schedule",
        profile_source=(
            yaml.safe_dump(configured.model_dump(mode="json"), allow_unicode=True, sort_keys=True)
            if resolved_profile is not None
            else profile
        ),
        command=tuple(sys.argv),
        parameters={
            "task": str(task_path),
            "task_name": task.name,
            "friction_coefficient": float(task.friction_coefficient),
            "cube_mass_kg": float(task.cube_mass_kg),
            "object_material": task.object_material,
            "scenario_duration_s": task.downward_load.duration_s,
            "scheduler": task.scheduler.model_dump(mode="json"),
            "solver": task.solver.model_dump(mode="json"),
        },
        run_name=run_name,
        run_prefix=run_prefix,
        run_suffix=run_suffix,
    )
    task_snapshot = run.artifact_path("task.yaml")
    if scheduling_task is None:
        task_snapshot.write_bytes(task_path.read_bytes())
    else:
        task_snapshot.write_text(
            yaml.safe_dump(task.model_dump(mode="json"), allow_unicode=True, sort_keys=True),
            encoding="utf-8",
        )
    run.register_artifact(task_snapshot)
    effective_parameters_path = run.artifact_path("effective_parameters.json")
    effective_parameters_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile": configured.model_dump(mode="json"),
                "task": task.model_dump(mode="json"),
                "runtime": {
                    "profile_path": (
                        "composed_profile"
                        if resolved_profile is not None
                        else str(profile.resolve())
                        if isinstance(profile, Path)
                        else "serialized_profile"
                    ),
                    "task_path": str(task_path.resolve()),
                    "scheduler_kind": (
                        "unified_adaptive"
                        if task.unified_adaptive is not None
                        else "oracle"
                        if task.adaptive_prior is None
                        else "adaptive_prior"
                    ),
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
    result = run_force_scheduling(
        configured,
        task=task,
        output_csv=trace_path,
        output_plot=plot_path,
    )
    for artifact in (trace_path, plot_path, plot_path.with_suffix(".pdf")):
        run.register_artifact(artifact)
    metrics_path = run.artifact_path("metrics.json")
    metrics_path.write_text(
        json.dumps(asdict(result), indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    run.register_artifact(metrics_path)
    run.finalize()
    return run, result


__all__ = ["execute_force_scheduling"]

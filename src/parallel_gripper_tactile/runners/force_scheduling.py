"""带结构化运行工件的一次目标力调度仿真。"""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import sys

from pydantic import TypeAdapter
from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig

from ..experiments.force_scheduling import (
    ForceSchedulingResult,
    ForceSchedulingTask,
    OracleForceSchedulerConfig,
    run_force_scheduling,
)
from ..config.profiles import GripperProfile, load_profile, validate_resolved_profile
from ..artifacts import RunDirectory

from .common import (
    profile_snapshot_source,
    profile_source_label,
    run_artifact_lifecycle,
    write_json_artifact,
    write_task_snapshot,
    write_yaml_artifact,
)


def execute_force_scheduling(
    *,
    profile: Path | str,
    resolved_profile: GripperProfile | None = None,
    task_path: Path,
    scheduling_task: ForceSchedulingTask | None = None,
    scheduler_config: OracleForceSchedulerConfig | UnifiedAdaptiveConfig,
    scheduler_source: Path | None = None,
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
        profile_source=profile_snapshot_source(
            profile, configured, composed=resolved_profile is not None
        ),
        command=tuple(sys.argv),
        parameters={
            "task": str(task_path),
            "task_name": task.name,
            "friction_coefficient": float(task.friction_coefficient),
            "cube_mass_kg": float(task.cube_mass_kg),
            "object_material": task.object_material,
            "scenario_duration_s": task.downward_load.duration_s,
            "scheduler": TypeAdapter(type(scheduler_config)).dump_python(
                scheduler_config, mode="json"
            ),
            "solver": task.solver.model_dump(mode="json"),
        },
        run_name=run_name,
        run_prefix=run_prefix,
        run_suffix=run_suffix,
    )
    with run_artifact_lifecycle(run):
        write_task_snapshot(run, task_path, scheduling_task)
        write_yaml_artifact(
            run,
            "scheduler.yaml",
            TypeAdapter(type(scheduler_config)).dump_python(scheduler_config, mode="json"),
        )
        write_json_artifact(
            run,
            "effective_parameters.json",
            {
                "schema_version": 1,
                "profile": configured.model_dump(mode="json"),
                "task": task.model_dump(mode="json"),
                "scheduler": TypeAdapter(type(scheduler_config)).dump_python(
                    scheduler_config, mode="json"
                ),
                "runtime": {
                    "profile_path": profile_source_label(
                        profile, composed=resolved_profile is not None
                    ),
                    "task_path": str(task_path.resolve()),
                    "scheduler_kind": (
                        "adaptive"
                        if isinstance(scheduler_config, UnifiedAdaptiveConfig)
                        else "oracle"
                    ),
                    "scheduler_path": (
                        None if scheduler_source is None else str(scheduler_source.resolve())
                    ),
                },
            },
        )
        trace_path = run.artifact_path("trace.csv")
        plot_path = run.artifact_path("plot.png")
        tactile_path = (
            run.artifact_path("tactile.jsonl")
            if task.tactile_sampling is not None and task.tactile_sampling.record_raw
            else None
        )
        result = run_force_scheduling(
            configured,
            task=task,
            scheduler_config=scheduler_config,
            output_csv=trace_path,
            output_plot=plot_path,
            output_tactile=tactile_path,
        )
        for artifact in (trace_path, plot_path, plot_path.with_suffix(".pdf")):
            run.register_artifact(artifact)
        if tactile_path is not None:
            run.register_artifact(tactile_path)
        write_json_artifact(run, "metrics.json", asdict(result))
        return run, result


__all__ = ["execute_force_scheduling"]

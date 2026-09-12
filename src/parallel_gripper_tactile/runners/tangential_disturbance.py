"""带可复现运行工件的一次切向扰动仿真。"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

import yaml

from ..artifacts import RunDirectory
from ..config.profiles import GripperProfile, load_profile, validate_resolved_profile
from ..experiments.tangential_disturbance import (
    TangentialDisturbanceResult,
    TangentialDisturbanceTask,
    run_tangential_disturbance,
)


def execute_tangential_disturbance(
    *,
    profile: Path | str,
    resolved_profile: GripperProfile | None = None,
    task_path: Path,
    disturbance_task: TangentialDisturbanceTask | None = None,
    output_root: Path = Path("outputs"),
    run_name: str | None = None,
    run_prefix: str | None = None,
    run_suffix: str | None = None,
) -> tuple[RunDirectory, TangentialDisturbanceResult]:
    """运行一次切向扰动实验，并保存可复现输入与实际生成的产物。"""
    task = disturbance_task or TangentialDisturbanceTask.load(task_path)
    configured = (
        validate_resolved_profile(resolved_profile)
        if resolved_profile is not None
        else load_profile(profile)
    )
    run = RunDirectory.create(
        output_root,
        profile_name=configured.name,
        experiment="tangential-disturbance",
        profile_source=(
            yaml.safe_dump(configured.model_dump(mode="json"), allow_unicode=True, sort_keys=True)
            if resolved_profile is not None
            else profile
        ),
        command=tuple(sys.argv),
        parameters={
            "task": str(task_path),
            "task_name": task.name,
            "control_period_s": float(task.control_period_s),
            "object_material": task.object_material,
            "cube_mass_kg": float(task.cube_mass_kg),
            "friction_coefficient": float(task.friction_coefficient),
        },
        run_name=run_name,
        run_prefix=run_prefix,
        run_suffix=run_suffix,
    )
    try:
        task_snapshot = run.artifact_path("task.yaml")
        if disturbance_task is None:
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
                        "object_material": task.object_material,
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
        pdf_path = plot_path.with_suffix(".pdf")
        result = run_tangential_disturbance(
            configured,
            task=task,
            output_csv=trace_path,
            output_plot=plot_path,
        )
        for artifact in (trace_path, plot_path, pdf_path):
            if artifact.is_file():
                run.register_artifact(artifact)
        metrics_path = run.artifact_path("metrics.json")
        metrics_path.write_text(
            json.dumps({**asdict(result), "passed": result.passed}, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
        run.register_artifact(metrics_path)
        run.finalize()
        return run, result
    except Exception as error:
        error_path = run.artifact_path("error.json")
        error_path.write_text(
            json.dumps(
                {"error_type": type(error).__name__, "message": str(error)},
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        run.register_artifact(error_path)
        run.finalize()
        raise


__all__ = ["execute_tangential_disturbance"]

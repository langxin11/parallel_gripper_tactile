"""带完整运行工件的 Robotiq 离散力控制仿真。"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

import yaml

from ..experiments.robotiq_discrete_force import (
    ControllerVariant,
    RobotiqDiscreteForceResult,
    RobotiqDiscreteForceTask,
    run_robotiq_discrete_force,
)
from ..config.profiles import GripperProfile, load_profile, validate_resolved_profile
from ..artifacts import RunDirectory
from ..scenes.robotiq import RobotiqObjectMaterial


def execute_robotiq_discrete_force(
    *,
    profile: Path | str,
    resolved_profile: GripperProfile | None = None,
    task_path: Path,
    discrete_task: RobotiqDiscreteForceTask | None = None,
    output_root: Path = Path("outputs"),
    run_name: str | None = None,
    run_prefix: str | None = None,
    run_suffix: str | None = None,
    controller_variant: ControllerVariant | None = None,
    object_material: RobotiqObjectMaterial | None = None,
    force_noise_std_n: float | None = None,
    noise_seed: int | None = None,
) -> tuple[RunDirectory, RobotiqDiscreteForceResult]:
    """运行一次离散力控制并保存输入快照、轨迹、图像和指标。"""
    task = discrete_task or RobotiqDiscreteForceTask.load(task_path)
    configured = (
        validate_resolved_profile(resolved_profile)
        if resolved_profile is not None
        else load_profile(profile)
    )
    effective_variant = controller_variant or task.controller_variant
    effective_material = object_material or task.object_material
    effective_noise = task.force_noise_std_n if force_noise_std_n is None else force_noise_std_n
    effective_seed = task.noise_seed if noise_seed is None else noise_seed
    run = RunDirectory.create(
        output_root,
        profile_name=configured.name,
        experiment="discrete-force",
        profile_source=(
            yaml.safe_dump(configured.model_dump(mode="json"), allow_unicode=True, sort_keys=True)
            if resolved_profile is not None
            else profile
        ),
        command=tuple(sys.argv),
        parameters={
            "task": str(task_path),
            "task_name": task.name,
            "controller_variant": effective_variant,
            "object_material": effective_material,
            "reference": task.reference.model_dump(mode="json"),
            "force_noise_std_n": effective_noise,
            "noise_seed": effective_seed,
        },
        run_name=run_name,
        run_prefix=run_prefix,
        run_suffix=run_suffix,
    )
    task_snapshot = run.artifact_path("task.yaml")
    if discrete_task is None:
        task_snapshot.write_bytes(task_path.read_bytes())
    else:
        task_snapshot.write_text(
            yaml.safe_dump(task.model_dump(mode="json"), allow_unicode=True, sort_keys=True),
            encoding="utf-8",
        )
    run.register_artifact(task_snapshot)
    effective_parameters = run.artifact_path("effective_parameters.json")
    effective_parameters.write_text(
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
                    "controller_variant": effective_variant,
                    "object_material": effective_material,
                    "reference": task.reference.model_dump(mode="json"),
                    "force_noise_std_n": effective_noise,
                    "noise_seed": effective_seed,
                },
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    run.register_artifact(effective_parameters)
    trace_path = run.artifact_path("trace.csv.gz")
    plot_path = run.artifact_path("plot.png")
    result = run_robotiq_discrete_force(
        configured,
        task=task,
        controller_variant=controller_variant,
        object_material=object_material,
        force_noise_std_n=force_noise_std_n,
        noise_seed=noise_seed,
        output_csv=trace_path,
        output_plot=plot_path,
    )
    for artifact in (trace_path, plot_path, plot_path.with_suffix(".pdf")):
        run.register_artifact(artifact)
    metrics_path = run.artifact_path("metrics.json")
    metrics_path.write_text(
        json.dumps(asdict(result), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    run.register_artifact(metrics_path)
    run.finalize()
    return run, result


__all__ = ["execute_robotiq_discrete_force"]

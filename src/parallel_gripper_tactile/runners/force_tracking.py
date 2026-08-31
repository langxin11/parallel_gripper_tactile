"""带可复现运行工件的一次力跟踪执行器。"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

from ..experiments.force_tracking import (
    ControllerVariant,
    ForceTrackingResult,
    ForceTrackingTask,
    run_force_tracking,
)
from ..control import ForceSemantics
from ..profiles import load_profile
from ..run_artifacts import RunDirectory
from ..scenes.custom import ObjectContactModel, ObjectMaterial


def execute_force_tracking(
    *,
    profile: Path,
    task_path: Path,
    tracking_task: ForceTrackingTask | None = None,
    output_root: Path = Path("outputs"),
    run_name: str | None = None,
    run_prefix: str | None = None,
    run_suffix: str | None = None,
    object_material: ObjectMaterial = "hard",
    object_contact_model: ObjectContactModel = "explicit",
    multiccd_enabled: bool = True,
    force_semantics: ForceSemantics = "average_side",
    controller_variant: ControllerVariant = "full",
    sensor_noise_seed: int | None = None,
    viewer: bool = False,
    render_fps: float = 30.0,
    realtime_factor: float = 1.0,
) -> tuple[RunDirectory, ForceTrackingResult]:
    """运行一次完整力跟踪，并返回其目录与结构化结果。"""
    task = tracking_task or ForceTrackingTask.load(task_path)
    configured = load_profile(profile)
    run = RunDirectory.create(
        output_root,
        profile_name=configured.name,
        experiment="force-track",
        profile_source=profile,
        command=tuple(sys.argv),
        parameters={
            "task": str(task_path),
            "task_name": task.name,
            "tracking_duration_s": task.reference.duration_s,
            "viewer": viewer,
            "render_fps": render_fps,
            "realtime_factor": realtime_factor,
            "object_material": object_material,
            "object_contact_model": object_contact_model,
            "multiccd_enabled": multiccd_enabled,
            "force_semantics": force_semantics,
            "controller_variant": controller_variant,
            "sensor_noise_seed": sensor_noise_seed,
        },
        run_name=run_name,
        run_prefix=run_prefix,
        run_suffix=run_suffix,
    )
    try:
        task_snapshot = run.artifact_path("task.yaml")
        task_snapshot.write_bytes(task_path.read_bytes())
        run.register_artifact(task_snapshot)
        csv_path = run.artifact_path("trace.csv")
        plot_path = run.artifact_path("plot.png")
        result = run_force_tracking(
            profile,
            task=task,
            output_csv=csv_path,
            output_plot=plot_path,
            viewer=viewer,
            render_fps=render_fps,
            realtime_factor=realtime_factor,
            object_material=object_material,
            object_contact_model=object_contact_model,
            multiccd_enabled=multiccd_enabled,
            force_semantics=force_semantics,
            controller_variant=controller_variant,
            sensor_noise_seed=sensor_noise_seed,
        )
        run.register_artifact(csv_path)
        run.register_artifact(plot_path)
        metrics_path = run.artifact_path("metrics.json")
        metrics_path.write_text(
            json.dumps(asdict(result), indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run.register_artifact(metrics_path)
        run.finalize()
    except Exception:
        # 保留未完成目录及其快照，便于诊断失败的可复现实验输入。
        raise
    return run, result

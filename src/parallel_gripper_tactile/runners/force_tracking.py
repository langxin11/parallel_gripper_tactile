"""带可复现运行工件的一次力跟踪执行器。"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys

import yaml

from ..experiments.force_tracking import (
    ControllerVariant,
    ForceTrackingResult,
    ForceTrackingTask,
    configure_force_controller,
    run_force_tracking,
)
from ..control import ForceSemantics
from ..config.profiles import (
    GripperProfile,
    StiffnessEstimatorMethod,
    TorqueAdrcControl,
    StiffnessRateControl,
    load_profile,
    validate_resolved_profile,
)
from ..artifacts import RunDirectory
from ..scenes.custom import ObjectContactModel, ObjectMaterial
from ..visualization.force_tracking import render_run_artifacts


def execute_force_tracking(
    *,
    profile: Path | str,
    resolved_profile: GripperProfile | None = None,
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
    stiffness_estimator_method: StiffnessEstimatorMethod | None = None,
    sensor_noise_seed: int | None = None,
    torque_adrc_override: TorqueAdrcControl | None = None,
    stiffness_rate_override: StiffnessRateControl | None = None,
    trace_sample_period_s: float | None = None,
    trace_event_window_s: float = 0.2,
    viewer: bool = False,
    render_fps: float = 30.0,
    realtime_factor: float = 1.0,
    tactile_detail: bool = False,
) -> tuple[RunDirectory, ForceTrackingResult]:
    """运行一次完整力跟踪，并返回其目录与结构化结果。"""
    task = tracking_task or ForceTrackingTask.load(task_path)
    configured = (
        configure_force_controller(
            validate_resolved_profile(resolved_profile),
            variant=controller_variant,
            stiffness_estimator_method=stiffness_estimator_method,
            sensor_noise_seed=sensor_noise_seed,
            torque_adrc_override=torque_adrc_override,
            stiffness_rate_override=stiffness_rate_override,
        )
        if resolved_profile is not None
        else configure_force_controller(
            load_profile(profile),
            variant=controller_variant,
            stiffness_estimator_method=stiffness_estimator_method,
            sensor_noise_seed=sensor_noise_seed,
            torque_adrc_override=torque_adrc_override,
            stiffness_rate_override=stiffness_rate_override,
        )
    )
    core_metadata = {}
    if controller_variant == "admittance":
        from dm_grasp_core import __version__

        core_metadata = {
            "dm_grasp_core_version": __version__,
            "command_application": "shared_request_then_existing_sim_mit_quantization_each_physics_step",
        }
    resolved_trace_sample_period_s = trace_sample_period_s
    if resolved_trace_sample_period_s is None:
        resolved_trace_sample_period_s = (
            task.control_period_s
            if controller_variant == "admittance"
            else 0.004
            if controller_variant in {"adrc-torque", "adrc-torque-td"}
            else 0.01
        )
    run = RunDirectory.create(
        output_root,
        profile_name=configured.name,
        experiment="force-track",
        profile_source=(
            yaml.safe_dump(configured.model_dump(mode="json"), allow_unicode=True, sort_keys=True)
            if resolved_profile is not None
            else profile
        ),
        command=tuple(sys.argv),
        parameters={
            **core_metadata,
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
            "stiffness_estimator_method": stiffness_estimator_method,
            "sensor_noise_seed": sensor_noise_seed,
            "trace_format": "parquet",
            "trace_compression": "zstd",
            "trace_schema_version": 2,
            "tactile_detail": tactile_detail,
            "trace_sample_period_s": resolved_trace_sample_period_s,
            "trace_event_window_s": trace_event_window_s,
            "torque_adrc_override": (
                None
                if torque_adrc_override is None
                else torque_adrc_override.model_dump(mode="json")
            ),
            "stiffness_rate_override": (
                None
                if stiffness_rate_override is None
                else stiffness_rate_override.model_dump(mode="json")
            ),
        },
        run_name=run_name,
        run_prefix=run_prefix,
        run_suffix=run_suffix,
    )
    try:
        task_snapshot = run.artifact_path("task.yaml")
        if tracking_task is None:
            task_snapshot.write_bytes(task_path.read_bytes())
        else:
            task_snapshot.write_text(
                yaml.safe_dump(
                    task.model_dump(mode="json"),
                    allow_unicode=True,
                    sort_keys=True,
                ),
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
                        **core_metadata,
                        "profile_path": (
                            "composed_profile"
                            if resolved_profile is not None
                            else str(profile.resolve())
                            if isinstance(profile, Path)
                            else "serialized_profile"
                        ),
                        "task_path": str(task_path.resolve()),
                        "object_material": object_material,
                        "object_contact_model": object_contact_model,
                        "multiccd_enabled": multiccd_enabled,
                        "force_semantics": force_semantics,
                        "controller_variant": controller_variant,
                        "stiffness_estimator_method": stiffness_estimator_method,
                        "sensor_noise_seed": sensor_noise_seed,
                        "viewer": viewer,
                        "render_fps": render_fps,
                        "realtime_factor": realtime_factor,
                        "trace_format": "parquet",
                        "trace_compression": "zstd",
                        "trace_schema_version": 2,
                        "tactile_detail": tactile_detail,
                        "trace_sample_period_s": resolved_trace_sample_period_s,
                        "trace_event_window_s": trace_event_window_s,
                        "torque_adrc_override": (
                            None
                            if torque_adrc_override is None
                            else torque_adrc_override.model_dump(mode="json")
                        ),
                        "stiffness_rate_override": (
                            None
                            if stiffness_rate_override is None
                            else stiffness_rate_override.model_dump(mode="json")
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
        parquet_path = run.artifact_path("trace.parquet")
        metrics_path = run.artifact_path("metrics.json")
        plot_config = json.loads(effective_parameters_path.read_text(encoding="utf-8"))

        def render_result(rows: list[dict[str, float | str]], result: ForceTrackingResult) -> None:
            """使用统一指标和未降采样轨迹绘图，并登记实际产物。"""
            metrics = asdict(result)
            metrics_path.write_text(
                json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            run.register_artifact(metrics_path)
            for path in render_run_artifacts(
                trace=rows,
                metrics=metrics,
                config=plot_config,
                output_dir=run.path / "plots",
                tactile_detail=tactile_detail,
            ):
                run.register_artifact(path)

        result = run_force_tracking(
            configured,
            task=task,
            output_parquet=parquet_path,
            on_result=render_result,
            trace_sample_period_s=resolved_trace_sample_period_s,
            trace_event_window_s=trace_event_window_s,
            viewer=viewer,
            render_fps=render_fps,
            realtime_factor=realtime_factor,
            object_material=object_material,
            object_contact_model=object_contact_model,
            multiccd_enabled=multiccd_enabled,
            force_semantics=force_semantics,
            controller_variant=controller_variant,
            stiffness_estimator_method=stiffness_estimator_method,
            sensor_noise_seed=sensor_noise_seed,
            torque_adrc_override=torque_adrc_override,
            stiffness_rate_override=stiffness_rate_override,
        )
        run.register_artifact(parquet_path)
        metrics_path.write_text(
            json.dumps(asdict(result), indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run.register_artifact(metrics_path)
        run.finalize()
    except Exception:
        # 保留未完成目录及其快照，便于诊断失败的可复现实验输入。
        raise
    return run, result

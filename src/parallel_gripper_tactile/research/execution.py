"""科研单次运行的计划与共享 runner 适配。"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping

import yaml

from ..experiments.force_scheduling import ForceSchedulingTask
from ..experiments.force_tracking import ForceTrackingTask
from ..experiments.friction_estimation import FrictionEstimationTask
from ..experiments.robotiq_discrete_force import RobotiqDiscreteForceTask
from ..experiments.tangential_disturbance import TangentialDisturbanceTask
from ..runners import (
    execute_force_scheduling,
    execute_force_tracking,
    execute_friction_estimation,
    execute_robotiq_discrete_force,
    execute_tangential_disturbance,
)
from .configuration import DMControllerSelection, ResolvedResearchRun


@dataclass(frozen=True, slots=True)
class ResearchRunOutcome:
    """科研入口一次调用的结构化结果。"""

    mode: str
    output_directory: Path
    run_directory: Path | None
    passed: bool | None


def _write_json(path: Path, value: object) -> Path:
    """以稳定格式写入一个 JSON 产物。"""
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def execute_research_run(
    resolved: ResolvedResearchRun,
    *,
    hydra_output_directory: Path,
    provenance: Mapping[str, object],
) -> ResearchRunOutcome:
    """保存最终配置，并按同一配置执行计划预览或单次实验。"""
    output_directory = hydra_output_directory.resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    effective = resolved.effective_parameters()
    _write_json(output_directory / "effective_configuration.json", effective)
    _write_json(output_directory / "composition_provenance.json", dict(provenance))

    selection = resolved.selection
    condition = {
        "controller": selection.controller.name,
        "estimator": selection.estimator.name,
        "task": resolved.task.name,
        "task_path": str(resolved.task_source),
        "scheduler": selection.scheduler.name,
        "material": selection.material.name,
        "seed": selection.seed,
    }
    if selection.execution.mode == "plan":
        _write_json(
            output_directory / "plan.json",
            {
                "schema_version": 1,
                "artifact_kind": "plan",
                "validated": True,
                "compatibility": {"status": "passed"},
                "condition_count": 1,
                "conditions": [condition],
                "output_directory": str(output_directory),
                "effective_configuration": effective,
            },
        )
        return ResearchRunOutcome("plan", output_directory, None, None)

    profile_snapshot = yaml.safe_dump(
        resolved.profile.model_dump(mode="json"),
        allow_unicode=True,
        sort_keys=True,
    )
    common = {
        "profile": profile_snapshot,
        "resolved_profile": resolved.profile,
        "task_path": resolved.task_source,
        "output_root": output_directory / "artifacts",
    }
    if isinstance(resolved.task, ForceTrackingTask):
        if not isinstance(selection.controller, DMControllerSelection):
            raise TypeError("force tracking requires a DM controller")
        run, result = execute_force_tracking(
            **common,
            tracking_task=resolved.task,
            object_material=selection.material.name,
            multiccd_enabled=selection.execution.multiccd_enabled,
            controller_variant=selection.controller.name,
            stiffness_estimator_method=(
                None if selection.estimator.name == "none" else selection.estimator.name
            ),
            sensor_noise_seed=selection.seed,
            torque_adrc_override=selection.controller.torque_adrc,
            trace_sample_period_s=selection.execution.trace_sample_period_s,
            trace_event_window_s=selection.execution.trace_event_window_s,
            plot_mode=selection.execution.plot_mode,
            viewer=selection.execution.viewer,
            render_fps=selection.execution.render_fps,
            realtime_factor=selection.execution.realtime_factor,
        )
    elif isinstance(resolved.task, ForceSchedulingTask):
        if resolved.scheduler is None:
            raise TypeError("force scheduling requires a resolved scheduler")
        run, result = execute_force_scheduling(
            **common,
            scheduling_task=resolved.task,
            scheduler_config=resolved.scheduler,
            scheduler_source=resolved.scheduler_source,
        )
    elif isinstance(resolved.task, FrictionEstimationTask):
        run, result = execute_friction_estimation(
            **common,
            estimation_task=resolved.task,
            plot_mode=selection.execution.plot_mode,
            sensor_noise_seed=selection.seed,
        )
    elif isinstance(resolved.task, RobotiqDiscreteForceTask):
        run, result = execute_robotiq_discrete_force(
            **common,
            discrete_task=resolved.task,
            controller_variant=selection.controller.name,
            object_material=selection.material.name,
            noise_seed=selection.seed,
        )
    elif isinstance(resolved.task, TangentialDisturbanceTask):
        run, result = execute_tangential_disturbance(
            **common,
            disturbance_task=resolved.task,
            policy_config=resolved.scheduler,
            policy_source=resolved.scheduler_source,
        )
    else:
        raise TypeError(f"unsupported run task: {type(resolved.task).__name__}")
    _write_json(
        output_directory / "execution.json",
        {
            "schema_version": 1,
            "artifact_kind": "execution",
            "condition": condition,
            "run_directory": str(run.path),
            "passed": result.passed,
        },
    )
    return ResearchRunOutcome("run", output_directory, run.path, result.passed)


__all__ = ["ResearchRunOutcome", "execute_research_run"]

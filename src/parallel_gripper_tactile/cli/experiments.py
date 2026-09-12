"""用于仿真、对比与交互检查的 Typer 命令。"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys
from typing import Annotated

from rich.table import Table
import mujoco
import typer
import yaml

from ..experiments.contact_compare import record_contact_ab
from ..experiments.force_tracking import (
    ForceTrackingTask,
)
from ..experiments.force_scheduling import ForceSchedulingTask
from ..experiments.friction_estimation import FrictionEstimationTask
from ..experiments.grasp import run_acceptance
from ..experiments.grasp_video import record_custom_grasp_video
from ..experiments.robotiq_discrete_force import RobotiqDiscreteForceTask
from ..config.profiles import GripperProfile, validate_resolved_profile
from ..protocols import DisturbanceProtocol
from ..artifacts import RunDirectory
from ..runners import (
    execute_force_scheduling,
    execute_force_tracking,
    execute_friction_estimation,
    execute_robotiq_discrete_force,
)
from ..research import compose_research_run
from ..research.configuration import DMControllerSelection
from ..scenes.custom import build_custom_grasp_model
from ..scenes.robotiq import load_grasp_model
from ..tactile import create_tactile_reader
from .common import fail, state


run_app = typer.Typer(help="Run headless tactile simulations.", no_args_is_help=True)
compare_app = typer.Typer(help="Compare tactile/contact representations.", no_args_is_help=True)
view_app = typer.Typer(help="Open interactive MuJoCo views.", no_args_is_help=True)


def _format_optional_seconds(value: float | None) -> str:
    """把可能缺失的瞬态时间格式化为表格文本。"""
    return "n/a" if value is None else f"{value:.3f} s"


def _format_optional_ratio(value: float | None) -> str:
    """把可能缺失的瞬态比例格式化为表格文本。"""
    return "n/a" if value is None else f"{100.0 * value:.1f}%"


def _run_directory(
    profile: Path | str,
    experiment: str,
    output_root: Path,
    run_name: str | None,
    parameters: dict[str, object],
    *,
    resolved_profile: GripperProfile,
    run_prefix: str | None = None,
    run_suffix: str | None = None,
) -> RunDirectory:
    """创建一个输出目录，包含 profile 快照与 manifest。"""
    validated = validate_resolved_profile(resolved_profile)
    return RunDirectory.create(
        output_root,
        profile_name=validated.name,
        experiment=experiment,
        profile_source=profile,
        command=tuple(sys.argv),
        parameters=parameters,
        run_name=run_name,
        run_prefix=run_prefix,
        run_suffix=run_suffix,
    )


def _profile_snapshot(profile: GripperProfile) -> str:
    """稳定序列化组合后的 profile，供 runner 与运行产物共同保存。"""
    return yaml.safe_dump(profile.model_dump(mode="json"), allow_unicode=True, sort_keys=True)


def _composed_run(experiment: str, set_values: list[str] | None = None):
    """组合 CLI 选择并执行完整领域校验。"""
    resolved = compose_research_run(experiment=experiment, overrides=tuple(set_values or ()))
    if resolved.selection.execution.mode != "run":
        raise ValueError(
            "pgt runtime commands require execution=run; use scripts/research/run.py for plan"
        )
    return resolved


def _run_profile_trace(profile: GripperProfile, steps: int) -> tuple[float, float, float]:
    """运行一个位置控制 profile，并返回最终时间与法向载荷。"""
    configured = validate_resolved_profile(profile)
    if configured.control_mode != "position":
        raise ValueError("tactile comparison currently requires position-control profiles")
    model = load_grasp_model(None, configured.model_path)
    data = mujoco.MjData(model)
    reader = create_tactile_reader(
        model,
        configured.tactile,
        name_resolver=lambda name: f"gripper/{name}",
    )
    actuator = model.actuator(f"gripper/{configured.actuator}").id
    for step in range(steps):
        progress = min(1.0, step / max(1.0, steps / 3.0))
        data.ctrl[actuator] = configured.open_control + progress * (
            configured.closed_control - configured.open_control
        )
        mujoco.mj_step(model, data)
    frame = reader.read(data)
    return float(data.time), float(frame.left[2].sum()), float(frame.right[2].sum())


@run_app.command("grasp")
def run_grasp(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "dm_gripper/force_tracking_default",
    set_values: Annotated[
        list[str] | None,
        typer.Option("--set", help="Repeatable Hydra override applied after the experiment."),
    ] = None,
    run_name: Annotated[str | None, typer.Option()] = None,
    run_prefix: Annotated[str | None, typer.Option("--run-prefix")] = None,
    run_suffix: Annotated[str | None, typer.Option("--run-suffix")] = None,
    disturbance_force: Annotated[float, typer.Option(min=0.0)] = 5.0,
    disturbance_frequency: Annotated[float, typer.Option(min=0.001)] = 2.0,
    target_force: Annotated[float | None, typer.Option(min=0.001)] = None,
    video: Annotated[bool, typer.Option()] = False,
) -> None:
    """运行 DM_Gripper 抓取验收实验。"""
    try:
        resolved = _composed_run(experiment, set_values)
        object_material = resolved.selection.material.name
        profile_snapshot = _profile_snapshot(resolved.profile)
        run = _run_directory(
            profile_snapshot,
            "grasp",
            resolved.selection.execution.output_root,
            run_name,
            {
                "disturbance_force_n": disturbance_force,
                "disturbance_frequency_hz": disturbance_frequency,
                "target_force_n": target_force,
                "video": video,
                "object_material": object_material,
            },
            resolved_profile=resolved.profile,
            run_prefix=run_prefix,
            run_suffix=run_suffix,
        )
        csv_path = run.artifact_path("trace.csv")
        plot_path = run.artifact_path("plot.png")
        protocol = DisturbanceProtocol(
            force_n=disturbance_force, frequency_hz=disturbance_frequency
        )
        result = run_acceptance(
            profile_snapshot,
            resolved_profile=resolved.profile,
            protocol=protocol,
            target_force_n=target_force,
            output_csv=csv_path,
            output_plot=plot_path,
            object_material=object_material,
        )
        run.register_artifact(csv_path)
        run.register_artifact(plot_path)
        run.register_artifact(plot_path.with_suffix(".pdf"))
        metrics_path = run.artifact_path("metrics.json")
        metrics_path.write_text(
            json.dumps(asdict(result), indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run.register_artifact(metrics_path)
        if video:
            video_path = run.artifact_path("video.mp4")
            record_custom_grasp_video(
                profile_path=profile_snapshot,
                resolved_profile=resolved.profile,
                output=video_path,
                protocol=protocol,
                target_force_n=target_force,
                object_material=object_material,
            )
            run.register_artifact(video_path)
        run.finalize()
    except Exception as error:
        fail(context, error, title="Grasp experiment failed")
    table = Table(title="Grasp acceptance")
    table.add_column("Result")
    table.add_column("Value")
    table.add_row("Passed", "PASS" if result.passed else "FAIL")
    table.add_row("Hold displacement", f"{result.hold_displacement_m * 1000:.3f} mm")
    table.add_row("Disturbance displacement", f"{result.disturbance_displacement_m * 1000:.3f} mm")
    table.add_row("Force RMSE", f"{result.force_tracking_rmse_n:.3f} N")
    state(context).console.print(table)
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")
    if not result.passed:
        raise typer.Exit(1)


@run_app.command("force-schedule")
def run_force_schedule(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "dm_gripper/force_scheduling_gravity_hold",
    set_values: Annotated[
        list[str] | None,
        typer.Option("--set", help="Repeatable Hydra override applied after the experiment."),
    ] = None,
    run_name: Annotated[str | None, typer.Option()] = None,
    run_prefix: Annotated[str | None, typer.Option("--run-prefix")] = None,
    run_suffix: Annotated[str | None, typer.Option("--run-suffix")] = None,
) -> None:
    """运行基于已知摩擦系数的抓取目标力调度实验。"""
    try:
        resolved = _composed_run(experiment, set_values)
        if not isinstance(resolved.task, ForceSchedulingTask):
            raise ValueError("selected experiment does not define a force-scheduling task")
        run, result = execute_force_scheduling(
            profile=_profile_snapshot(resolved.profile),
            resolved_profile=resolved.profile,
            task_path=resolved.task_source,
            scheduling_task=resolved.task,
            output_root=resolved.selection.execution.output_root,
            run_name=run_name,
            run_prefix=run_prefix,
            run_suffix=run_suffix,
        )
    except Exception as error:
        fail(context, error, title="Force scheduling experiment failed")
    table = Table(title="Force scheduling")
    table.add_column("Result")
    table.add_column("Value")
    table.add_row("Passed", "PASS" if result.passed else "FAIL")
    table.add_row("Force RMSE", f"{result.force_tracking_rmse_n:.3f} N")
    table.add_row(
        "Max tangential displacement",
        f"{result.max_tangential_displacement_m * 1000:.3f} mm",
    )
    table.add_row("Mean target force", f"{result.mean_target_force_n:.3f} N")
    table.add_row("Final target force", f"{result.final_target_force_n:.3f} N")
    state(context).console.print(table)
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")
    if not result.passed:
        raise typer.Exit(1)


@run_app.command("friction-estimate")
def run_friction_estimate(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "dm_gripper/friction_estimation_nominal",
    set_values: Annotated[
        list[str] | None,
        typer.Option("--set", help="Repeatable Hydra override applied after the experiment."),
    ] = None,
    run_name: Annotated[str | None, typer.Option()] = None,
    run_prefix: Annotated[str | None, typer.Option("--run-prefix")] = None,
    run_suffix: Annotated[str | None, typer.Option("--run-suffix")] = None,
) -> None:
    """运行微滑移探测、保守摩擦估计与估计值力调度实验。"""
    try:
        resolved = _composed_run(experiment, set_values)
        if not isinstance(resolved.task, FrictionEstimationTask):
            raise ValueError("selected experiment does not define a friction-estimation task")
        run, result = execute_friction_estimation(
            profile=_profile_snapshot(resolved.profile),
            resolved_profile=resolved.profile,
            task_path=resolved.task_source,
            estimation_task=resolved.task,
            plot_mode=resolved.selection.execution.plot_mode,
            output_root=resolved.selection.execution.output_root,
            run_name=run_name,
            run_prefix=run_prefix,
            run_suffix=run_suffix,
            sensor_noise_seed=resolved.selection.seed,
        )
    except Exception as error:
        fail(context, error, title="Friction estimation experiment failed")
    table = Table(title="Friction estimation")
    table.add_column("Result")
    table.add_column("Value")
    table.add_row("Passed", "PASS" if result.passed else "FAIL")
    table.add_row("Slip detected", "yes" if result.slip_detected else "fallback")
    table.add_row("Estimated friction", f"{result.estimated_friction_coefficient:.3f}")
    table.add_row("Detection reason", result.detection_reason)
    table.add_row(
        "Detection time (probe)",
        "none"
        if result.probe_detection_time_s is None
        else f"{result.probe_detection_time_s:.3f} s",
    )
    table.add_row("Estimate / true", f"{result.estimate_ratio:.1%}")
    table.add_row(
        "Max probe displacement",
        f"{result.max_probe_displacement_m * 1000:.3f} mm",
    )
    table.add_row(
        "Max hold displacement",
        f"{result.max_hold_displacement_m * 1000:.3f} mm",
    )
    state(context).console.print(table)
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")
    if not result.passed:
        raise typer.Exit(1)


@run_app.command("discrete-force")
def run_discrete_force(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "robotiq_2f85/discrete_force",
    set_values: Annotated[
        list[str] | None,
        typer.Option("--set", help="Repeatable Hydra override applied after the experiment."),
    ] = None,
    run_name: Annotated[str | None, typer.Option()] = None,
    run_prefix: Annotated[str | None, typer.Option("--run-prefix")] = None,
    run_suffix: Annotated[str | None, typer.Option("--run-suffix")] = None,
) -> None:
    """运行基于单 tick 力增量的 Robotiq 离散力控制实验。"""
    try:
        resolved = _composed_run(experiment, set_values)
        if not isinstance(resolved.task, RobotiqDiscreteForceTask):
            raise ValueError("selected experiment does not define a discrete-force task")
        run, result = execute_robotiq_discrete_force(
            profile=_profile_snapshot(resolved.profile),
            resolved_profile=resolved.profile,
            task_path=resolved.task_source,
            discrete_task=resolved.task,
            output_root=resolved.selection.execution.output_root,
            run_name=run_name,
            run_prefix=run_prefix,
            run_suffix=run_suffix,
            controller_variant=resolved.selection.controller.name,
            object_material=resolved.selection.material.name,
            noise_seed=resolved.selection.seed,
        )
    except Exception as error:
        fail(context, error, title="Discrete force experiment failed")
    table = Table(title="Robotiq discrete force control")
    table.add_column("Result")
    table.add_column("Value")
    table.add_row("Passed", "PASS" if result.passed else "FAIL")
    table.add_row("Variant", result.controller_variant)
    table.add_row("Steady MAE", f"{result.steady_force_error_n:.3f} N")
    table.add_row("Actions", str(result.action_count))
    table.add_row("Reversals", str(result.reverse_count))
    table.add_row("Oscillations", str(result.oscillation_count))
    table.add_row(
        "Settled platforms",
        f"{result.settled_platform_count}/{result.platform_count}",
    )
    table.add_row("HOLD ratio", f"{100.0 * result.hold_ratio:.1f}%")
    table.add_row(
        "Delta F / tick",
        "n/a"
        if result.delta_f_tick_estimate_n is None
        else f"{result.delta_f_tick_estimate_n:.3f} N/tick",
    )
    state(context).console.print(table)
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")
    if not result.passed:
        raise typer.Exit(1)


@run_app.command("force-track")
def run_force_track(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "dm_gripper/force_tracking_default",
    set_values: Annotated[
        list[str] | None,
        typer.Option("--set", help="Repeatable Hydra override applied after the experiment."),
    ] = None,
    run_name: Annotated[str | None, typer.Option()] = None,
    run_prefix: Annotated[str | None, typer.Option("--run-prefix")] = None,
    run_suffix: Annotated[str | None, typer.Option("--run-suffix")] = None,
) -> None:
    """运行 waypoint 目标法向力跟踪实验。"""
    try:
        resolved = _composed_run(experiment, set_values)
        if not isinstance(resolved.task, ForceTrackingTask):
            raise ValueError("selected experiment does not define a force-tracking task")
        if not isinstance(resolved.selection.controller, DMControllerSelection):
            raise ValueError("force tracking requires a DM controller")
        selection = resolved.selection
        run, result = execute_force_tracking(
            profile=_profile_snapshot(resolved.profile),
            resolved_profile=resolved.profile,
            task_path=resolved.task_source,
            tracking_task=resolved.task,
            output_root=selection.execution.output_root,
            run_name=run_name,
            run_prefix=run_prefix,
            run_suffix=run_suffix,
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
    except Exception as error:
        fail(context, error, title="Force tracking experiment failed")
    table = Table(title="Force tracking")
    table.add_column("Result")
    table.add_column("Value")
    table.add_row("Passed", "PASS" if result.passed else "FAIL")
    table.add_row("Contact time", f"{result.contact_time_s:.3f} s")
    table.add_row("Tracking start", f"{result.tracking_start_time_s:.3f} s")
    table.add_row("RMSE", f"{result.rmse_n:.3f} N")
    table.add_row("MAE", f"{result.mae_n:.3f} N")
    table.add_row("Peak error", f"{result.peak_abs_error_n:.3f} N")
    table.add_row("Rise time", _format_optional_seconds(result.rise_time_s))
    table.add_row("Overshoot", _format_optional_ratio(result.overshoot_ratio))
    table.add_row("Settling time", _format_optional_seconds(result.settling_time_s))
    table.add_row("Torque saturation", f"{100.0 * result.torque_saturation_ratio:.1f}%")
    table.add_row(
        "Stiffness position limit",
        f"{100.0 * result.stiffness_position_limit_ratio:.1f}%",
    )
    state(context).console.print(table)
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")
    if not result.passed:
        raise typer.Exit(1)


@run_app.command("demo")
def run_demo(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "robotiq_2f85/discrete_force",
    set_values: Annotated[list[str] | None, typer.Option("--set")] = None,
    steps: Annotated[int, typer.Option(min=1)] = 1500,
    run_name: Annotated[str | None, typer.Option()] = None,
) -> None:
    """运行由 profile 驱动的无界面 Robotiq 触觉演示。"""
    try:
        resolved = _composed_run(experiment, set_values)
        configured = resolved.profile
        if configured.control_mode != "position":
            raise ValueError("run demo currently supports position-control profiles; use run grasp")
        run = _run_directory(
            _profile_snapshot(configured),
            "demo",
            resolved.selection.execution.output_root,
            run_name,
            {"steps": steps, "experiment": experiment, "overrides": set_values or []},
            resolved_profile=configured,
        )
        model = load_grasp_model(None, configured.model_path)
        data = mujoco.MjData(model)
        reader = create_tactile_reader(
            model,
            configured.tactile,
            name_resolver=lambda name: f"gripper/{name}",
        )
        actuator = model.actuator(f"gripper/{configured.actuator}").id
        for step in range(steps):
            progress = min(1.0, step / max(1.0, steps / 3.0))
            data.ctrl[actuator] = configured.open_control + progress * (
                configured.closed_control - configured.open_control
            )
            mujoco.mj_step(model, data)
        frame = reader.read(data)
        trace_path = run.artifact_path("trace.csv")
        trace_path.write_text(
            "time_s,left_fz,right_fz\n"
            f"{data.time:.9g},{frame.left[2].sum():.9g},{frame.right[2].sum():.9g}\n",
            encoding="utf-8",
        )
        run.register_artifact(trace_path)
        run.finalize()
    except Exception as error:
        fail(context, error, title="Demo failed")
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")


@compare_app.command("contact")
def compare_contact(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "dm_gripper/force_tracking_default",
    set_values: Annotated[list[str] | None, typer.Option("--set")] = None,
    run_name: Annotated[str | None, typer.Option()] = None,
    taxel: Annotated[str, typer.Option()] = "left:11",
    steps: Annotated[int, typer.Option(min=1)] = 1800,
) -> None:
    """在共享姿态下比较原生网格接触与 MuJoCo mesh-SDF。"""
    try:
        resolved = _composed_run(experiment, set_values)
        if resolved.selection.platform.family != "dm":
            raise ValueError("contact comparison requires a DM experiment")
        profile_snapshot = _profile_snapshot(resolved.profile)
        side, digits = taxel.split(":", maxsplit=1)
        selected = (side, int(digits[0]), int(digits[1]))
        run = _run_directory(
            profile_snapshot,
            "contact-compare",
            resolved.selection.execution.output_root,
            run_name,
            {"taxel": taxel, "steps": steps, "experiment": experiment},
            resolved_profile=resolved.profile,
        )
        csv_path, plot_path, video_path = record_contact_ab(
            output_dir=run.path,
            profile_path=profile_snapshot,
            resolved_profile=resolved.profile,
            taxel=selected,
            cube_half_size=0.003,
            steps=steps,
            stop_above_native_force=15.0,
            fps=30,
            width=960,
            height=540,
            keep_frames=False,
        )
        for artifact in (csv_path, plot_path, plot_path.with_suffix(".pdf"), video_path):
            run.register_artifact(artifact)
        run.finalize()
    except Exception as error:
        fail(context, error, title="Contact comparison failed")
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")


@compare_app.command("tactile")
def compare_tactile(
    context: typer.Context,
    left_experiment: Annotated[
        str, typer.Option("--left-experiment")
    ] = "robotiq_2f85/discrete_force",
    right_experiment: Annotated[
        str, typer.Option("--right-experiment")
    ] = "robotiq_2f85/discrete_force",
    left_set: Annotated[list[str] | None, typer.Option("--left-set")] = None,
    right_set: Annotated[list[str] | None, typer.Option("--right-set")] = None,
    steps: Annotated[int, typer.Option(min=1)] = 1500,
    run_name: Annotated[str | None, typer.Option()] = None,
) -> None:
    """比较两个位置控制触觉 profile 的最终法向载荷。"""
    try:
        left_resolved = _composed_run(left_experiment, left_set)
        right_resolved = _composed_run(right_experiment, right_set)
        left = left_resolved.profile
        right = right_resolved.profile
        run = _run_directory(
            _profile_snapshot(left),
            "tactile-compare",
            left_resolved.selection.execution.output_root,
            run_name,
            {
                "left_experiment": left_experiment,
                "right_experiment": right_experiment,
                "left_overrides": left_set or [],
                "right_overrides": right_set or [],
                "steps": steps,
            },
            resolved_profile=left,
        )
        left_time, left_force, left_right_force = _run_profile_trace(left, steps)
        right_time, right_force, right_right_force = _run_profile_trace(right, steps)
        trace_path = run.artifact_path("trace.csv")
        trace_path.write_text(
            "profile,time_s,left_fz_n,right_fz_n\n"
            f"{left.name},{left_time:.9g},{left_force:.9g},{left_right_force:.9g}\n"
            f"{right.name},{right_time:.9g},{right_force:.9g},{right_right_force:.9g}\n",
            encoding="utf-8",
        )
        metrics_path = run.artifact_path("metrics.json")
        metrics_path.write_text(
            json.dumps(
                {
                    "left_profile": left.name,
                    "right_profile": right.name,
                    "left_total_fz_n": left_force + left_right_force,
                    "right_total_fz_n": right_force + right_right_force,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        run.register_artifact(trace_path)
        run.register_artifact(metrics_path)
        run.finalize()
    except Exception as error:
        fail(context, error, title="Tactile comparison failed")
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")


@view_app.command("grasp")
def view_grasp(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "dm_gripper/force_tracking_default",
    set_values: Annotated[list[str] | None, typer.Option("--set")] = None,
) -> None:
    """打开一个可交互的 DM_Gripper 抓取场景。"""
    try:
        resolved = _composed_run(experiment, set_values)
        model = build_custom_grasp_model(
            resolved.profile,
            object_material=resolved.selection.material.name,
        )
        data = mujoco.MjData(model)
        from mujoco import viewer

        viewer.launch(model, data)
    except Exception as error:
        fail(context, error, title="Viewer failed")


@view_app.command("taxels")
def view_taxels(
    context: typer.Context,
    experiment: Annotated[
        str, typer.Option("--experiment", help="Named experiment under configs/experiment.")
    ] = "robotiq_2f85/discrete_force",
    set_values: Annotated[list[str] | None, typer.Option("--set")] = None,
) -> None:
    """打开一个可交互的 profile 模型，并显示触觉 site。"""
    try:
        configured = _composed_run(experiment, set_values).profile
        model = mujoco.MjModel.from_xml_path(str(configured.model_path))
        data = mujoco.MjData(model)
        from mujoco import viewer

        viewer.launch(model, data)
    except Exception as error:
        fail(context, error, title="Viewer failed")

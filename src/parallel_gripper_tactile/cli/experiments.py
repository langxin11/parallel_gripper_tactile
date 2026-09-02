"""用于仿真、对比与交互检查的 Typer 命令。"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys
from typing import Annotated, Literal

from rich.table import Table
import mujoco
import typer

from ..experiments.contact_compare import record_contact_ab
from ..experiments.force_tracking import (
    ForceTrackingTask,
)
from ..experiments.grasp import run_acceptance
from ..experiments.grasp_video import record_custom_grasp_video
from ..profiles import load_profile
from ..protocols import DisturbanceProtocol
from ..run_artifacts import RunDirectory
from ..runners import execute_force_tracking
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
    profile: Path,
    experiment: str,
    output_root: Path,
    run_name: str | None,
    parameters: dict[str, object],
    *,
    run_prefix: str | None = None,
    run_suffix: str | None = None,
) -> RunDirectory:
    """创建一个输出目录，包含 profile 快照与 manifest。"""
    validated = load_profile(profile)
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


def _run_profile_trace(profile: Path, steps: int) -> tuple[float, float, float]:
    """运行一个位置控制 profile，并返回最终时间与法向载荷。"""
    configured = load_profile(profile)
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
    profile: Annotated[Path, typer.Option("--profile", exists=True, dir_okay=False)],
    output_root: Annotated[Path, typer.Option("--output-root", file_okay=False)] = Path("outputs"),
    run_name: Annotated[str | None, typer.Option()] = None,
    run_prefix: Annotated[str | None, typer.Option("--run-prefix")] = None,
    run_suffix: Annotated[str | None, typer.Option("--run-suffix")] = None,
    disturbance_force: Annotated[float, typer.Option(min=0.0)] = 5.0,
    disturbance_frequency: Annotated[float, typer.Option(min=0.001)] = 2.0,
    target_force: Annotated[float | None, typer.Option(min=0.001)] = None,
    video: Annotated[bool, typer.Option()] = False,
    object_material: Annotated[
        Literal["soft", "medium", "hard", "stiff"], typer.Option("--object-material")
    ] = "hard",
) -> None:
    """运行自研夹爪抓取验收实验。"""
    try:
        run = _run_directory(
            profile,
            "grasp",
            output_root,
            run_name,
            {
                "disturbance_force_n": disturbance_force,
                "disturbance_frequency_hz": disturbance_frequency,
                "target_force_n": target_force,
                "video": video,
                "object_material": object_material,
            },
            run_prefix=run_prefix,
            run_suffix=run_suffix,
        )
        csv_path = run.artifact_path("trace.csv")
        plot_path = run.artifact_path("plot.png")
        protocol = DisturbanceProtocol(
            force_n=disturbance_force, frequency_hz=disturbance_frequency
        )
        result = run_acceptance(
            profile,
            protocol=protocol,
            target_force_n=target_force,
            output_csv=csv_path,
            output_plot=plot_path,
            object_material=object_material,
        )
        run.register_artifact(csv_path)
        run.register_artifact(plot_path)
        metrics_path = run.artifact_path("metrics.json")
        metrics_path.write_text(
            json.dumps(asdict(result), indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run.register_artifact(metrics_path)
        if video:
            video_path = run.artifact_path("video.mp4")
            record_custom_grasp_video(
                profile_path=profile,
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


@run_app.command("force-track")
def run_force_track(
    context: typer.Context,
    profile: Annotated[Path, typer.Option("--profile", exists=True, dir_okay=False)],
    task: Annotated[Path, typer.Option("--task", exists=True, dir_okay=False)],
    output_root: Annotated[Path, typer.Option("--output-root", file_okay=False)] = Path("outputs"),
    run_name: Annotated[str | None, typer.Option()] = None,
    run_prefix: Annotated[str | None, typer.Option("--run-prefix")] = None,
    run_suffix: Annotated[str | None, typer.Option("--run-suffix")] = None,
    viewer: Annotated[
        bool, typer.Option("--viewer", help="Open MuJoCo viewer while running.")
    ] = False,
    render_fps: Annotated[float, typer.Option("--render-fps", min=1.0)] = 30.0,
    realtime_factor: Annotated[float, typer.Option("--realtime-factor", min=0.001)] = 1.0,
    object_material: Annotated[
        Literal["soft", "medium", "hard", "stiff"], typer.Option("--object-material")
    ] = "hard",
    disable_multiccd: Annotated[
        bool,
        typer.Option("--disable-multiccd", help="Use one contact per convex geom pair."),
    ] = False,
    controller_variant: Annotated[
        Literal["pid-only", "pid-torque-ff", "pid-stiffness-ff", "full"],
        typer.Option("--controller-variant"),
    ] = "full",
    sensor_noise_seed: Annotated[int | None, typer.Option("--sensor-noise-seed", min=0)] = None,
) -> None:
    """运行 waypoint 目标法向力跟踪实验。"""
    try:
        tracking_task = ForceTrackingTask.load(task)
        run, result = execute_force_tracking(
            profile=profile,
            task_path=task,
            tracking_task=tracking_task,
            output_root=output_root,
            run_name=run_name,
            run_prefix=run_prefix,
            run_suffix=run_suffix,
            object_material=object_material,
            multiccd_enabled=not disable_multiccd,
            controller_variant=controller_variant,
            sensor_noise_seed=sensor_noise_seed,
            viewer=viewer,
            render_fps=render_fps,
            realtime_factor=realtime_factor,
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
    state(context).console.print(table)
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")
    if not result.passed:
        raise typer.Exit(1)


@run_app.command("demo")
def run_demo(
    context: typer.Context,
    profile: Annotated[Path, typer.Option("--profile", exists=True, dir_okay=False)],
    steps: Annotated[int, typer.Option(min=1)] = 1500,
    output_root: Annotated[Path, typer.Option("--output-root", file_okay=False)] = Path("outputs"),
    run_name: Annotated[str | None, typer.Option()] = None,
) -> None:
    """运行由 profile 驱动的无界面 Robotiq 触觉演示。"""
    try:
        configured = load_profile(profile)
        if configured.control_mode != "position":
            raise ValueError("run demo currently supports position-control profiles; use run grasp")
        run = _run_directory(profile, "demo", output_root, run_name, {"steps": steps})
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
    profile: Annotated[Path, typer.Option("--profile", exists=True, dir_okay=False)],
    output_root: Annotated[Path, typer.Option("--output-root", file_okay=False)] = Path("outputs"),
    run_name: Annotated[str | None, typer.Option()] = None,
    taxel: Annotated[str, typer.Option()] = "left:11",
    steps: Annotated[int, typer.Option(min=1)] = 1800,
) -> None:
    """在共享姿态下比较原生网格接触与 MuJoCo mesh-SDF。"""
    try:
        if load_profile(profile).name != "custom_parallel_gripper":
            raise ValueError("contact comparison requires the custom-gripper profile")
        side, digits = taxel.split(":", maxsplit=1)
        selected = (side, int(digits[0]), int(digits[1]))
        run = _run_directory(
            profile, "contact-compare", output_root, run_name, {"taxel": taxel, "steps": steps}
        )
        csv_path, plot_path, video_path = record_contact_ab(
            output_dir=run.path,
            profile_path=profile,
            taxel=selected,
            cube_half_size=0.003,
            steps=steps,
            stop_above_native_force=15.0,
            fps=30,
            width=960,
            height=540,
            keep_frames=False,
        )
        for artifact in (csv_path, plot_path, video_path):
            run.register_artifact(artifact)
        run.finalize()
    except Exception as error:
        fail(context, error, title="Contact comparison failed")
    state(context).console.print(f"Run: [cyan]{run.path}[/cyan]")


@compare_app.command("tactile")
def compare_tactile(
    context: typer.Context,
    left_profile: Annotated[Path, typer.Option("--left-profile", exists=True, dir_okay=False)],
    right_profile: Annotated[Path, typer.Option("--right-profile", exists=True, dir_okay=False)],
    steps: Annotated[int, typer.Option(min=1)] = 1500,
    output_root: Annotated[Path, typer.Option("--output-root", file_okay=False)] = Path("outputs"),
    run_name: Annotated[str | None, typer.Option()] = None,
) -> None:
    """比较两个位置控制触觉 profile 的最终法向载荷。"""
    try:
        left = load_profile(left_profile)
        right = load_profile(right_profile)
        run = _run_directory(
            left_profile,
            "tactile-compare",
            output_root,
            run_name,
            {
                "left_profile": str(left_profile),
                "right_profile": str(right_profile),
                "steps": steps,
            },
        )
        left_time, left_force, left_right_force = _run_profile_trace(left_profile, steps)
        right_time, right_force, right_right_force = _run_profile_trace(right_profile, steps)
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
    profile: Annotated[Path, typer.Option("--profile", exists=True, dir_okay=False)],
    object_material: Annotated[
        Literal["soft", "medium", "hard", "stiff"], typer.Option("--object-material")
    ] = "hard",
) -> None:
    """打开一个可交互的自研夹爪抓取场景。"""
    try:
        model = build_custom_grasp_model(load_profile(profile), object_material=object_material)
        data = mujoco.MjData(model)
        from mujoco import viewer

        viewer.launch(model, data)
    except Exception as error:
        fail(context, error, title="Viewer failed")


@view_app.command("taxels")
def view_taxels(
    context: typer.Context,
    profile: Annotated[Path, typer.Option("--profile", exists=True, dir_okay=False)],
) -> None:
    """打开一个可交互的 profile 模型，并显示触觉 site。"""
    try:
        configured = load_profile(profile)
        model = mujoco.MjModel.from_xml_path(str(configured.model_path))
        data = mujoco.MjData(model)
        from mujoco import viewer

        viewer.launch(model, data)
    except Exception as error:
        fail(context, error, title="Viewer failed")

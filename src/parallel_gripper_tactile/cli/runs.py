"""用于结构化工实验运行的安全检查与清理命令。"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from rich.table import Table
import typer

from ..run_artifacts import clean_runs, list_runs, plan_clean
from .common import fail, format_value, state


app = typer.Typer(help="Inspect and safely clean structured experiment runs.", no_args_is_help=True)


@app.command("list")
def list_command(
    context: typer.Context,
    output_root: Annotated[
        Path, typer.Option("--output-root", file_okay=False, resolve_path=True)
    ] = Path("outputs"),
) -> None:
    """按从新到旧列出结构化工实验运行，不修改它们。"""
    try:
        runs = list_runs(output_root)
    except Exception as error:
        fail(context, error, title="Unable to list runs")
    table = Table(title=f"Runs in {output_root}")
    table.add_column("Created", no_wrap=True)
    table.add_column("Profile")
    table.add_column("Experiment")
    table.add_column("Path", style="cyan")
    for run in runs:
        manifest = run.manifest
        table.add_row(
            format_value(manifest.created_at.isoformat() if manifest else None),
            format_value(manifest.profile_name if manifest else None),
            format_value(manifest.experiment if manifest else None),
            str(run.path),
        )
    state(context).console.print(table)


@app.command("clean")
def clean_command(
    context: typer.Context,
    older_than_days: Annotated[int | None, typer.Option(min=0)] = None,
    all_runs: Annotated[bool, typer.Option("--all")] = False,
    cache: Annotated[bool, typer.Option()] = False,
    apply: Annotated[bool, typer.Option()] = False,
    output_root: Annotated[
        Path, typer.Option("--output-root", file_okay=False, resolve_path=True)
    ] = Path("outputs"),
) -> None:
    """预览清理目标；仅在设置 ``--apply`` 时才移除它们。"""
    try:
        preview = plan_clean(
            output_root,
            older_than_days=older_than_days,
            all_runs=all_runs,
            cache=cache,
        )
        clean_runs(preview, apply=apply)
    except Exception as error:
        fail(context, error, title="Unable to clean runs")
    label = "Removed" if apply else "Would remove"
    console = state(context).console
    console.print(f"{label} {preview.count} target(s):")
    for target in preview.targets:
        console.print(f"  [cyan]{target}[/cyan]")

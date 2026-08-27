"""parallel-gripper 触觉工具包的 Typer 应用。"""

from __future__ import annotations

from typing import Annotated

from rich.console import Console
import typer

from .. import __version__
from . import assets, experiments, runs, validate
from .common import CliState


app = typer.Typer(
    help="Parallel-gripper tactile simulation, asset, and experiment tools.",
    no_args_is_help=True,
    add_completion=True,
)


@app.callback()
def root_callback(
    context: typer.Context,
    version: Annotated[bool, typer.Option("--version", is_eager=True)] = False,
    verbose: Annotated[bool, typer.Option("--verbose", "-v")] = False,
    no_color: Annotated[bool, typer.Option("--no-color")] = False,
) -> None:
    """在分发命令前配置共享输出。"""
    console = Console(no_color=no_color)
    context.obj = CliState(console=console, verbose=verbose)
    if version:
        console.print(__version__)
        raise typer.Exit()


app.command("validate")(validate.command)
app.add_typer(assets.app, name="assets")
app.add_typer(experiments.run_app, name="run")
app.add_typer(experiments.compare_app, name="compare")
app.add_typer(runs.app, name="runs")
app.add_typer(experiments.view_app, name="view")


def main() -> None:
    """运行标准的 ``pgt`` 命令行应用。"""
    app()


__all__ = ["app", "main"]

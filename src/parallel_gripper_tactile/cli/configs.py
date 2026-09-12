"""配置发现命令。"""

from __future__ import annotations

from typing import Annotated

from rich.table import Table
import typer

from ..research.catalog import ConfigCatalogError, list_configurations
from .common import state


app = typer.Typer(help="查看可选择的科研配置。", no_args_is_help=True)


@app.command("list")
def list_command(
    context: typer.Context,
    group: Annotated[str | None, typer.Argument(help="可选配置组。 ")] = None,
    search: Annotated[str | None, typer.Option("--search", help="按名称或用途过滤。 ")] = None,
) -> None:
    """列出可直接用于 Hydra 选择的配置名称。"""
    cli_state = state(context)
    try:
        entries = list_configurations(group, search=search)
    except ConfigCatalogError as error:
        cli_state.console.print(f"配置发现失败：{error}", style="red")
        raise typer.Exit(code=2) from error

    if not entries:
        scope = f"配置组“{group}”" if group else "全部配置组"
        suffix = f"，过滤条件为“{search}”" if search else ""
        cli_state.console.print(f"{scope}没有匹配的配置{suffix}。")
        return

    table = Table(title="可用科研配置", show_lines=True)
    table.add_column("配置组", style="cyan", no_wrap=True)
    table.add_column("配置名", style="green", no_wrap=False, overflow="fold")
    table.add_column("用途", overflow="fold")
    for entry in entries:
        table.add_row(entry.group, entry.name, entry.purpose or "—")
    cli_state.console.print(table)


__all__ = ["app"]

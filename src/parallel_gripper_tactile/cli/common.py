"""Typer CLI 共用的展示与错误处理辅助函数。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from pydantic import ValidationError
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
import typer


@dataclass(frozen=True, slots=True)
class CliState:
    """由根应用回调配置的全局 CLI 状态。"""

    console: Console
    verbose: bool


def state(context: typer.Context) -> CliState:
    """返回子命令上下文已初始化的根 CLI 状态。"""
    current: typer.Context | None = context
    while current is not None:
        if isinstance(current.obj, CliState):
            return current.obj
        current = current.parent
    return CliState(console=Console(), verbose=False)


def render_validation_error(console: Console, error: ValidationError) -> None:
    """将 Pydantic 字段错误渲染为紧凑、易读的 Rich 表格。"""
    table = Table(title="Profile validation failed", show_lines=True)
    table.add_column("Field", style="cyan", no_wrap=True)
    table.add_column("Problem", style="red")
    table.add_column("Input", style="dim")
    for detail in error.errors(include_url=False):
        location = ".".join(str(part) for part in detail["loc"])
        input_value = repr(detail.get("input", ""))
        table.add_row(location or "<profile>", detail["msg"], input_value[:160])
    console.print(table)


def fail(context: typer.Context, error: Exception, *, title: str = "Command failed") -> None:
    """展示预期的命令错误，并以惯例的退出码 2 退出。"""
    cli_state = state(context)
    if isinstance(error, ValidationError):
        render_validation_error(cli_state.console, error)
    else:
        cli_state.console.print(Panel.fit(str(error), title=title, border_style="red"))
        if cli_state.verbose:
            cli_state.console.print_exception(show_locals=False)
    raise typer.Exit(code=2)


def format_value(value: Any) -> str:
    """为易读的紧凑表格单元格格式化可选值。"""
    return "—" if value is None else str(value)

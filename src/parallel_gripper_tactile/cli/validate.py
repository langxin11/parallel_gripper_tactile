"""直接的 ``pgt validate PROFILE`` 命令。"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from rich.table import Table
import typer

from ..config.profiles import load_profile
from ..validation import validate_profile
from .common import fail, state


def command(
    context: typer.Context,
    profile: Annotated[Path, typer.Argument(exists=True, dir_okay=False, readable=True)],
) -> None:
    """加载、编译并验证一个 YAML 夹爪 profile。"""
    try:
        report = validate_profile(load_profile(profile))
    except Exception as error:  # 预期的配置与模型失败会变成 CLI 诊断信息。
        fail(context, error, title="Profile validation failed")
    table = Table(title="Profile validation")
    table.add_column("Profile", style="cyan")
    table.add_column("Actuator")
    table.add_column("Tactile channels", justify="right")
    table.add_column("Equalities", justify="right")
    table.add_row(
        report.model_name,
        report.actuator,
        str(report.tactile_channels),
        str(report.equalities),
    )
    state(context).console.print(table)

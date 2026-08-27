"""在 ``pgt assets`` 下暴露的资产生成命令。"""

from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Annotated

import typer

from ..asset_tools import (
    DEFAULT_BASE_XML,
    DEFAULT_BOX_TAXEL_OUTPUT_XML,
    DEFAULT_GRID_COLS,
    DEFAULT_GRID_ROWS,
    DEFAULT_TAXEL_OUTPUT_XML,
    DEFAULT_TOUCH_GRID_OUTPUT_XML,
    build_taxel_tree,
    build_touch_grid_tree,
    label_taxel_geoms,
    write_tree,
)
from .common import fail, state


app = typer.Typer(help="Generate and prepare MJCF assets.", no_args_is_help=True)


class TaxelShape(str, Enum):
    """力传感器 taxel 支持的碰撞形状。"""

    sphere = "sphere"
    box = "box"


@app.command("generate-taxels")
def generate_taxels(
    context: typer.Context,
    base_xml: Annotated[
        Path, typer.Option("--base-xml", exists=True, dir_okay=False, readable=True)
    ] = DEFAULT_BASE_XML,
    shape: Annotated[TaxelShape, typer.Option()] = TaxelShape.sphere,
    output_xml: Annotated[Path | None, typer.Option("--output-xml", dir_okay=False)] = None,
) -> None:
    """生成 Robotiq 球体或盒体力 taxel MJCF 资产。"""
    output = output_xml or (
        DEFAULT_BOX_TAXEL_OUTPUT_XML if shape is TaxelShape.box else DEFAULT_TAXEL_OUTPUT_XML
    )
    try:
        write_tree(build_taxel_tree(base_xml, shape.value), output)
    except Exception as error:
        fail(context, error, title="Taxel asset generation failed")
    state(context).console.print(f"Generated [cyan]{output}[/cyan]")


@app.command("generate-touch-grid")
def generate_touch_grid(
    context: typer.Context,
    base_xml: Annotated[
        Path, typer.Option("--base-xml", exists=True, dir_okay=False, readable=True)
    ] = DEFAULT_BASE_XML,
    output_xml: Annotated[Path, typer.Option("--output-xml", dir_okay=False)] = (
        DEFAULT_TOUCH_GRID_OUTPUT_XML
    ),
    rows: Annotated[int, typer.Option(min=1)] = DEFAULT_GRID_ROWS,
    cols: Annotated[int, typer.Option(min=1)] = DEFAULT_GRID_COLS,
) -> None:
    """生成 Robotiq MuJoCo ``touch_grid`` 触觉资产。"""
    try:
        write_tree(build_touch_grid_tree(base_xml, rows=rows, cols=cols), output_xml)
    except Exception as error:
        fail(context, error, title="Touch-grid asset generation failed")
    state(context).console.print(f"Generated [cyan]{output_xml}[/cyan]")


@app.command("prepare-onshape")
def prepare_onshape(
    context: typer.Context,
    input_xml: Annotated[Path, typer.Argument(exists=True, dir_okay=False, readable=True)],
    output_xml: Annotated[Path, typer.Argument(dir_okay=False)],
    left_joint: Annotated[str, typer.Option()] = "left_finger_slide",
    right_joint: Annotated[str, typer.Option()] = "right_finger_slide",
) -> None:
    """为 Onshape 的 MJCF 导出提供稳定的行主序 taxel geom 名称。"""
    try:
        label_taxel_geoms(
            input_xml,
            output_xml,
            left_joint=left_joint,
            right_joint=right_joint,
        )
    except Exception as error:
        fail(context, error, title="Onshape asset preparation failed")
    state(context).console.print(f"Prepared [cyan]{output_xml}[/cyan]")

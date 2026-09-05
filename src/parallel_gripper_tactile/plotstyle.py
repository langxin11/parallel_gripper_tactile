"""共享绘图基础样式：统一实验产物图与论文图的 Matplotlib 外观。

原先 SciencePlots 的加载与 rcParams 设置在十余处绘图入口逐字重复，
现收敛到本模块。`science_pyplot()` 统一应用论文版式与中英文字体；尺寸和双格式导出也由本模块管理。
"""

from __future__ import annotations

from pathlib import Path
import math

from typing import Any

# SciencePlots 的论文级无 LaTeX 样式组合：ieee 给出 Times 衬线与网格，
# no-latex 使绘图不依赖 TeX 发行版。scienceplots 需导入后才注册这些样式表。
BASE_STYLE_SHEETS = ("science", "ieee", "no-latex")

# 基础样式统一补齐的 rcParams：PDF/PS 嵌入 TrueType 字体（文字可复制），
# 位图按出版物 600 DPI 落盘。
BASE_RC_PARAMS: dict[str, Any] = {
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "savefig.dpi": 600,
}

# IEEE 单栏与跨栏图的版面宽度（英寸）。论文图按此宽度设置 figsize，
# 嵌入论文时不缩放，图内字号所见即所得。
COLUMN_WIDTH_IN = 3.5
TEXT_WIDTH_IN = 7.16

# 论文图字体栈与 Typst 模板同源，使用思源宋体与 TeX Gyre Termes，
# 中文字体优先，确保同一文本中的中文与 MathText 混排不丢字。
PAPER_FONT_STACK = ("Noto Serif CJK SC", "TeX Gyre Termes")


def science_pyplot() -> Any:
    """按项目统一样式加载并返回 matplotlib.pyplot。

    Returns:
        应用了基础样式与统一 rcParams 的 matplotlib.pyplot 模块。
    """
    try:
        import matplotlib.pyplot as plt
        import scienceplots  # noqa: F401 -- 导入后注册 SciencePlots 样式。
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error
    plt.style.use(BASE_STYLE_SHEETS)
    plt.rcParams.update(BASE_RC_PARAMS)
    apply_paper_style(plt)
    return plt


def apply_paper_style(plt: Any) -> None:
    """在基础样式之上应用论文单栏版式预设。

    字体与 Typst 论文模板同源，字号按 IEEE 单栏 3.5 in 宽度校准：图内
    8 pt 与论文 8 pt 图注同大小，嵌入后不再因缩放变小。使用时以
    `COLUMN_WIDTH_IN`（单栏）或 `TEXT_WIDTH_IN`（跨栏）作为 figsize 宽度。

    Args:
        plt: `science_pyplot()` 返回的 pyplot 模块。
    """
    plt.rcParams.update(
        {
            "font.family": list(PAPER_FONT_STACK),
            "mathtext.fontset": "dejavuserif",
            "font.serif": list(PAPER_FONT_STACK),
            "font.size": 8,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "legend.fontsize": 7,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.0,
            "lines.markersize": 3,
        }
    )


def paper_figsize(height: float, *, columns: int = 2) -> tuple[float, float]:
    """返回最终论文栏宽下的图像尺寸，单位为英寸。

    Args:
        height: 图像高度，必须为有限正数。
        columns: 单栏使用 1，跨栏使用 2。

    Returns:
        固定栏宽与指定高度组成的尺寸。

    Raises:
        ValueError: 栏数或高度无效。
    """
    if columns not in (1, 2) or not math.isfinite(height) or height <= 0:
        raise ValueError("栏数必须为 1 或 2，高度必须为有限正数。")
    return (COLUMN_WIDTH_IN if columns == 1 else TEXT_WIDTH_IN, height)


def save_publication_figure(figure: Any, path: Path, **kwargs: Any) -> Path:
    """保存论文 PDF 和 600 DPI PNG，并保留其他显式请求的格式。

    保持画布物理尺寸，不使用会改变最终栏宽的紧边界裁切；由绘图入口设置布局。
    不关闭图像，便于调用方继续显示或检查。

    Args:
        figure: 已完成布局的 Matplotlib 图像。
        path: 原有输出路径；同名 PDF 和 PNG 总会生成。
        **kwargs: 其他保存选项，格式、分辨率与边界由本函数统一管理。

    Returns:
        生成的 PDF 路径。
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    options = {k: v for k, v in kwargs.items() if k not in {"dpi", "format", "bbox_inches"}}
    pdf_path = path.with_suffix(".pdf")
    png_path = path.with_suffix(".png")
    # 屏蔽调用方的全局紧裁切设置，保证导出宽度等于设计栏宽。
    import matplotlib as mpl

    with mpl.rc_context({"savefig.bbox": None}):
        figure.savefig(pdf_path, format="pdf", bbox_inches=None, **options)
        figure.savefig(png_path, format="png", dpi=600, bbox_inches=None, **options)
        if path not in (pdf_path, png_path):
            figure.savefig(path, dpi=600, bbox_inches=None, **options)
    return pdf_path

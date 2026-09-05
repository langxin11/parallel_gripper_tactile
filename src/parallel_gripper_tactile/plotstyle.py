"""共享绘图基础样式：统一实验产物图与论文图的 Matplotlib 外观。

原先 SciencePlots 的加载与 rcParams 设置在十余处绘图入口逐字重复，
现收敛到本模块。`science_pyplot()` 复刻既有产物图行为；`apply_paper_style()`
在其上叠加论文单栏版式预设，字体与 Typst 论文模板同源。
"""

from __future__ import annotations

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

# 论文图字体栈与 Typst 模板同源：西文用 Times 风格的 TeX Gyre Termes，
# 中文回退思源宋体（matplotlib 3.7+ 支持字体回退列表）。
PAPER_FONT_STACK = ("TeX Gyre Termes", "Noto Serif CJK SC")


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
            "font.family": "serif",
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

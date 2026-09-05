"""共享绘图样式模块的行为验证。"""

from __future__ import annotations

import matplotlib.pyplot as plt

from parallel_gripper_tactile.plotstyle import (
    COLUMN_WIDTH_IN,
    TEXT_WIDTH_IN,
    apply_paper_style,
    science_pyplot,
)


def test_science_pyplot_applies_base_rc_params() -> None:
    """基础样式应统一字体嵌入与 DPI，并返回 pyplot 模块。"""
    result = science_pyplot()

    assert result is plt
    assert plt.rcParams["pdf.fonttype"] == 42
    assert plt.rcParams["ps.fonttype"] == 42
    assert plt.rcParams["savefig.dpi"] == 600


def test_apply_paper_style_matches_typst_template_fonts() -> None:
    """论文版式应与 Typst 模板字体同源，并按 IEEE 栏宽校准字号。"""
    science_pyplot()
    apply_paper_style(plt)

    assert plt.rcParams["font.family"] == ["serif"]
    assert plt.rcParams["font.serif"] == ["TeX Gyre Termes", "Noto Serif CJK SC"]
    assert plt.rcParams["font.size"] == 8
    assert plt.rcParams["legend.fontsize"] == 7
    assert plt.rcParams["axes.linewidth"] == 0.8


def test_paper_width_constants_follow_ieee_layout() -> None:
    """栏宽常量应与 IEEE 单栏、跨栏版面一致。"""
    assert COLUMN_WIDTH_IN == 3.5
    assert TEXT_WIDTH_IN == 7.16

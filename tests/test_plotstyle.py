"""共享绘图样式模块的行为验证。"""

from __future__ import annotations

import matplotlib.pyplot as plt

import parallel_gripper_tactile.friction_plots as legacy_friction_plots
import parallel_gripper_tactile.plotstyle as legacy_plotstyle
import parallel_gripper_tactile.visualization as visualization
from parallel_gripper_tactile.visualization import friction
from parallel_gripper_tactile.visualization import plotstyle
from parallel_gripper_tactile.plotstyle import (
    COLUMN_WIDTH_IN,
    FULL_WIDTH_FONT_SCALE,
    TEXT_WIDTH_IN,
    apply_paper_style,
    science_pyplot,
)


def test_visualization_preserves_existing_plotstyle_object_identity() -> None:
    """旧 ``plotstyle`` 路径与新 ``visualization`` 入口导出同一对象。"""
    for name in visualization.__all__:
        if hasattr(plotstyle, name):
            assert getattr(legacy_plotstyle, name) is getattr(visualization, name)
    legacy_names = {name for name in dir(legacy_plotstyle) if not name.startswith("_")}
    implementation_names = {name for name in dir(plotstyle) if not name.startswith("_")}
    assert legacy_names == implementation_names
    for name in legacy_names:
        assert getattr(legacy_plotstyle, name) is getattr(plotstyle, name)


def test_visualization_preserves_existing_friction_plot_object_identity() -> None:
    """旧 ``friction_plots`` 路径与新摩擦绘图模块导出同一函数。"""
    assert legacy_friction_plots.plot_summary is friction.plot_summary
    assert legacy_friction_plots.plot_taxel_diagnostics is friction.plot_taxel_diagnostics
    legacy_names = {name for name in dir(legacy_friction_plots) if not name.startswith("_")}
    implementation_names = {name for name in dir(friction) if not name.startswith("_")}
    assert legacy_names == implementation_names
    for name in legacy_names:
        assert getattr(legacy_friction_plots, name) is getattr(friction, name)


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

    assert plt.rcParams["font.family"] == ["Noto Serif CJK SC", "TeX Gyre Termes"]
    assert plt.rcParams["font.serif"] == ["Noto Serif CJK SC", "TeX Gyre Termes"]
    assert plt.rcParams["font.size"] == 9
    assert plt.rcParams["axes.labelsize"] == 9
    assert plt.rcParams["axes.titlesize"] == 10
    assert plt.rcParams["xtick.labelsize"] == 8
    assert plt.rcParams["legend.fontsize"] == 8
    assert plt.rcParams["axes.linewidth"] == 0.9
    assert plt.rcParams["lines.linewidth"] == 1.2
    assert plt.rcParams["lines.markersize"] == 4


def test_full_width_font_scale_enlarges_typography() -> None:
    """跨栏整宽档应等比放大字号与线宽，保持文字相对面板的视觉密度。"""
    import pytest

    science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)

    assert plt.rcParams["font.size"] == pytest.approx(9 * FULL_WIDTH_FONT_SCALE)
    assert plt.rcParams["axes.labelsize"] == pytest.approx(9 * FULL_WIDTH_FONT_SCALE)
    assert plt.rcParams["xtick.labelsize"] == pytest.approx(8 * FULL_WIDTH_FONT_SCALE)
    assert plt.rcParams["legend.fontsize"] == pytest.approx(8 * FULL_WIDTH_FONT_SCALE)
    assert plt.rcParams["axes.linewidth"] == pytest.approx(0.9 * FULL_WIDTH_FONT_SCALE)
    assert plt.rcParams["lines.linewidth"] == pytest.approx(1.2 * FULL_WIDTH_FONT_SCALE)


def test_apply_paper_style_rejects_invalid_font_scale() -> None:
    """无效缩放系数应在创建图像前报告明确错误。"""
    import pytest

    for scale in (0, -1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            apply_paper_style(plt, font_scale=scale)


def test_paper_width_constants_follow_ieee_layout() -> None:
    """栏宽常量应与 IEEE 单栏、跨栏版面一致。"""
    assert COLUMN_WIDTH_IN == 3.5
    assert TEXT_WIDTH_IN == 7.16


def test_publication_export_preserves_width_and_requested_format(tmp_path) -> None:
    """全局紧裁切不能改变导出栏宽，且原请求格式仍然保留。"""
    import re

    from PIL import Image

    from parallel_gripper_tactile.plotstyle import paper_figsize, save_publication_figure

    with plt.rc_context():
        science_pyplot()
        figure, axis = plt.subplots(figsize=paper_figsize(2, columns=1), layout="constrained")
        axis.plot([0, 1], [-1, 1])
        axis.set_xlabel("Time (s)")
        plt.rcParams["savefig.bbox"] = "tight"
        pdf = save_publication_figure(figure, tmp_path / "figure.svg", bbox_inches="tight")
        assert (tmp_path / "figure.svg").is_file()
        with Image.open(tmp_path / "figure.png") as preview:
            assert preview.size == (2100, 1200)
        bounds = re.search(rb"/MediaBox\s*\[\s*0\s+0\s+([\d.]+)\s+([\d.]+)", pdf.read_bytes())
        assert bounds is not None
        assert float(bounds[1]) == 252
        assert float(bounds[2]) == 144
        assert plt.rcParams["savefig.bbox"] == "tight"
        plt.close(figure)


def test_paper_figsize_rejects_invalid_dimensions() -> None:
    """无效尺寸在创建图像前报告明确错误。"""
    import pytest

    from parallel_gripper_tactile.plotstyle import paper_figsize

    for height in (0, -1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            paper_figsize(height)
    with pytest.raises(ValueError):
        paper_figsize(2, columns=3)


def test_chinese_math_label_renders_without_missing_glyphs(tmp_path, caplog) -> None:
    """中文与数学公式混排应在已安装中文字体时完整渲染。"""
    import warnings

    import pytest
    from matplotlib import font_manager

    from parallel_gripper_tactile.plotstyle import paper_figsize, save_publication_figure

    if "Noto Serif CJK SC" not in {font.name for font in font_manager.fontManager.ttflist}:
        pytest.skip("环境未安装论文中文字体。")
    with plt.rc_context(), warnings.catch_warnings(record=True) as caught:
        science_pyplot()
        figure, axis = plt.subplots(figsize=paper_figsize(2, columns=1), layout="constrained")
        axis.plot([0, 1], [-1, 1], label="触觉力")
        axis.set(xlabel="时间（s）", ylabel="力（N）", title="中文公式：$F_z$")
        axis.legend()
        save_publication_figure(figure, tmp_path / "chinese.png")
        plt.close(figure)
    assert not any("Glyph" in str(item.message) for item in caught)
    assert "does not have a glyph" not in caplog.text

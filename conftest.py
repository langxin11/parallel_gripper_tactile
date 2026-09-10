"""全仓 pytest 运行环境与共享 fixture。"""

from __future__ import annotations

import atexit
import os
import shutil
import tempfile
from collections.abc import Generator

import pytest


for variable in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
):
    os.environ.setdefault(variable, "1")


def _is_managed_matplotlib_cache() -> bool:
    """判断字体缓存是否由本测试进程管理。"""
    return os.environ.get("PGT_PYTEST_MANAGED_MPLCONFIGDIR") == "1"


if "MPLCONFIGDIR" not in os.environ or _is_managed_matplotlib_cache():
    _matplotlib_cache = tempfile.mkdtemp(prefix="pgt-pytest-matplotlib-")
    os.environ["MPLCONFIGDIR"] = _matplotlib_cache
    os.environ["PGT_PYTEST_MANAGED_MPLCONFIGDIR"] = "1"
    atexit.register(shutil.rmtree, _matplotlib_cache, ignore_errors=True)


@pytest.fixture
def fast_png_render(monkeypatch: pytest.MonkeyPatch) -> Generator[None]:
    """让内容测试低成本栅格化 PNG，同时保留真实 PDF 渲染。"""
    import matplotlib.pyplot as plt
    from matplotlib.figure import Figure

    plt.get_backend()
    original_savefig = Figure.savefig

    def savefig(figure: Figure, filename: object, *args: object, **kwargs: object) -> None:
        """仅降低 PNG 栅格分辨率，不改变格式、布局和生产代码。"""
        if kwargs.get("format") == "png" or str(filename).lower().endswith(".png"):
            kwargs["dpi"] = 100
        original_savefig(figure, filename, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", savefig)
    yield


@pytest.fixture
def fast_plot_render(fast_png_render: None, monkeypatch: pytest.MonkeyPatch) -> Generator[None]:
    """让合成数据内容测试只完整渲染一次，并保留 PDF 写出分支。"""
    from matplotlib.figure import Figure

    content_savefig = Figure.savefig

    def savefig(figure: Figure, filename: object, *args: object, **kwargs: object) -> None:
        """PNG 渲染真实内容，PDF 以最小画布验证格式写出。"""
        if kwargs.get("format") == "pdf" or str(filename).lower().endswith(".pdf"):
            placeholder = Figure(figsize=(0.1, 0.1))
            content_savefig(placeholder, filename, format="pdf", dpi=72)
            return
        content_savefig(figure, filename, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", savefig)
    yield

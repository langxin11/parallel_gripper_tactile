"""Typst 实验报告模板的编译冒烟测试。"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

pytestmark = pytest.mark.skipif(
    shutil.which("typst") is None,
    reason="本机未安装 Typst CLI，跳过报告编译冒烟测试",
)


def test_template_smoke_report_compiles(tmp_path: Path) -> None:
    """fixture 报告应完整走一遍模板公开能力并编译出非空 PDF。

    覆盖 `reports/template.typ` 的文档骨架、CSV 直读表（含 CRLF 行尾与空串）、
    `metrics.json` 键值表、运行元信息块、子图网格、LaTeX 公式兼容层，以及
    字面量研究数据表（数值列右对齐、布尔中文化与空值占位）；fixture 数据不依赖
    `outputs/` 产物，任何环境都可编译。
    """
    source = REPO_ROOT / "tests" / "fixtures" / "report_smoke.typ"
    output = tmp_path / "report_smoke.pdf"

    result = subprocess.run(
        [
            "typst",
            "compile",
            "--root",
            str(REPO_ROOT),
            str(source),
            str(output),
        ],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )

    assert result.returncode == 0, f"Typst 编译失败：\n{result.stdout}\n{result.stderr}"
    assert output.exists(), "编译成功但未产出 PDF"
    assert output.stat().st_size > 500, "产出的 PDF 异常小"

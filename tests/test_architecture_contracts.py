"""机器检查 docs/architecture.md 声明的依赖规则。

契约定义在 ``pyproject.toml`` 的 ``[tool.importlinter]``；本测试把其并入裸 pytest
门禁，单独执行可用 ``uv run lint-imports`` 复现同一结论。
"""

from __future__ import annotations

from pathlib import Path

import pytest
from importlinter.cli import lint_imports_command

_PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"


def test_import_linter_contracts(capsys: pytest.CaptureFixture[str]) -> None:
    """import-linter 契约必须全部通过。"""
    with pytest.raises(SystemExit) as excinfo:
        lint_imports_command(["--config", str(_PYPROJECT)])
    assert excinfo.value.code == 0, capsys.readouterr().out

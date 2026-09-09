# Contributing

感谢贡献 `parallel-gripper-tactile`。本仓库约定：**说明性文字（docstring、行内注释、文档）
使用简体中文；技术名称、代码标识符、命令行参数与物理量符号保留英文。**

完整约定见 [代码与注释规范](docs/coding-conventions.md)。

## 提交前检查

在仓库根目录执行：

```bash
uv sync --all-packages --all-groups --locked
uv run pre-commit install
```

安装后，每次 `git commit` 会自动执行 Ruff 检查、Ruff 格式检查和完整 pytest。需要手动对全部文件
运行同一组门禁时执行：

```bash
uv run pre-commit run --all-files
```

也可逐项执行：

```bash
uv run ruff check .            # 环境缓存异常时：uv run ruff check --no-cache .
uv run ruff format --check .
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest
```

## 语言与风格要点

- 中文说明文字用全角标点（`，。：；`）；行内代码、标识符、命令行与物理量用半角。
- docstring 采用 Google 风格；`Args:`、`Returns:`、`Raises:` 等节标题保持英文字面值，节内用中文。
- 行内注释写「为什么」或「边界条件」，不写「代码在做什么」。
- 不要改动代码逻辑、profile schema、CLI 行为或数据格式，本仓库的注释与文档应与之保持同步。

## PR 验收清单

- [ ] ruff 与格式检查通过。
- [ ] 测试通过。
- [ ] 新增/修改的注释、docstring、文档为简体中文。
- [ ] 若新增命令或结论涉及时序逻辑，更新对应文档与 `CHANGELOG.md`。

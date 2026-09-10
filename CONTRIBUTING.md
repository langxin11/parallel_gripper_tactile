# Contributing

感谢贡献 `parallel-gripper-tactile`。本仓库约定：**说明性文字（docstring、行内注释、文档）
使用简体中文；技术名称、代码标识符、命令行参数与物理量符号保留英文。**

完整约定见 [代码与注释规范](docs/coding-conventions.md)。
测试分层、并行运行与增量映射见 [测试策略](docs/testing.md)。

## 提交前检查

在仓库根目录执行：

```bash
uv sync --all-packages --all-groups --locked
uv run pre-commit install
```

安装后，每次 `git commit` 会自动执行 Ruff 检查、Ruff 格式检查和并行完整 pytest。pytest worker
数量由 `pytest-xdist` 按当前机器自动确定。需要手动对全部文件运行同一组门禁时执行：

```bash
uv run pre-commit run --all-files
```

也可逐项执行：

```bash
uv run ruff check .            # 环境缓存异常时：uv run ruff check --no-cache .
uv run ruff format --check .
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest
```

开发阶段可按当前改动运行相关测试，或在 24 逻辑核开发机执行已实测的并行全量命令：

```bash
uv run python scripts/test_changed.py
uv run pytest -n 24
```

`uv run pytest --lf` 仅用于重跑上次失败，不是提交门禁。pre-commit 执行
`uv run pytest -n auto`，CI 仍执行裸 `uv run pytest`；两者都运行完整测试集，增量运行不能替代
提交前的权威全量结果。

修改 `configs/research/`、`scripts/research/`、`research/` 编排层或 `studies/lifecycle.py` 时，至少额外
验证配置组合、计划、生命周期状态／失败分类和 study 矩阵测试；`scripts/test_changed.py` 已包含对应
映射。需要手工冒烟时使用计划模式，避免把 Hydra
的 `--cfg job --resolve` 误当作领域校验：

```bash
uv run python scripts/research/run.py execution=plan
uv run python scripts/research/study.py research=force_controller_ablation/study
uv run python scripts/research/study.py research=torque_adrc_tuning/study
```

Hydra/OmegaConf 位于 `research` 依赖组，完整开发安装已包含该组。不要将 Hydra 引入共享控制核或
硬件包，也不要用外层 Multirun 执行正式 study。新增或修改正式 study 时，领域 protocol 必须独占条件
生成，计划和执行传递同一个 `StudyPlan`；不得把科学失败与 Python 异常混为同一失败字段，也不得绕过
产物摘要或 coarse／confirm 谱系校验。

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

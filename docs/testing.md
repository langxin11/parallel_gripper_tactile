# 测试策略

CI 与 pre-commit 均运行完整测试集；增量检查只用于开发反馈，不替代提交门禁。

## 环境与提交检查

```bash
uv sync --all-packages --all-groups --locked
uv run pre-commit install
uv run pre-commit run --all-files
```

每次提交执行 Ruff、格式检查和并行完整 pytest。Ruff 缓存异常时加 `--no-cache`；
全局 pytest 插件干扰时设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。

## 全量测试

CI 的权威命令：

```bash
uv run ruff check .
uv run ruff format --check .
uv run pytest
```

pre-commit 执行 `uv run pytest -n auto`；本地可用 `-n N` 按 CPU 与内存调整 worker 数。
pytest 默认将每个 worker 的 OpenMP／BLAS 线程限制为 1，并隔离 Matplotlib 缓存；显式环境变量优先。
产物测试必须使用 `tmp_path` 或显式临时目录，禁止写入共享 `outputs/`、资产目录或固定文件名。

## 增量测试

```bash
uv run python scripts/test_changed.py --list
uv run python scripts/test_changed.py
uv run python scripts/test_changed.py --base origin/main
```

默认检查工作区；`--base` 同时纳入相对分支的已提交改动，也可直接传文件路径。
映射位于脚本顶部，覆盖包内测试和控制、感知、场景、仿真、可视化的跨模块依赖。
测试基础设施、CI、未映射代码、配置或资产改动保守回退全量；纯文档改动不运行 pytest。
`pytest --lf` 仅重跑本地记录的失败，不能作为全量通过依据。

## 仿真与绘图测试分工

- 物理场景、修改参数对照与 runner 端到端测试完整执行；复用仿真的 fixture 返回深拷贝，避免污染。
- 配置组合、有序矩阵与实际运行测试验证当前行为；历史快照只验证历史文件完整性。
- CI 在安装开发依赖前检查最小环境 CLI，避免掩盖运行依赖缺失。
- 摩擦估计保留四个标准场景和完整仿真—绘图—manifest 链路，以及保守性与保持稳定性断言。
- 绘图内容测试用小型合成数据和低分辨率真实渲染检查统计、布局、缺失数据与产物登记。
- `tests/test_plotstyle.py` 验证 IEEE 栏宽、600 DPI PNG、PDF MediaBox、字体及中文数学符号；
  `tests/test_force_tracking.py` 保留代表性复杂图的真实 600 DPI 契约。

修改公共导出函数、字体策略或图形尺寸时，必须运行上述两个契约测试文件。

## 科研编排改动的额外验证

修改 `configs/research/`、`scripts/research/`、`research/` 或 `studies/lifecycle.py`，须验证配置组合、
计划、生命周期状态／失败分类和 study 矩阵；增量脚本已映射。手工冒烟使用计划模式：

```bash
uv run python scripts/research/run.py execution=plan
uv run python scripts/research/study.py research=force_controller_ablation/study
uv run python scripts/research/study.py research=torque_adrc_tuning/study
```

Hydra 的 `--cfg job --resolve` 只打印配置，不替代领域校验。

## 文档检查

```bash
uv run zensical build --strict
```

检查改动涉及的导航、内部链接与锚点。纯文档本地验证无需 pytest，提交与 CI 门禁仍照常执行。

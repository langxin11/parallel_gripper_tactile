# 测试策略

本仓库把提交门禁和开发反馈分开：CI 执行裸 `uv run pytest`，pre-commit 执行
`uv run pytest -n auto`，开发者也可按改动范围选择增量测试。串行与并行门禁都运行完整测试集，
不减少物理场景、摩擦标准场景和科研结论断言。

## 全量测试

CI 的串行权威命令为：

```bash
uv run ruff check .
uv run ruff format --check .
uv run pytest
```

pre-commit 使用 `uv run pytest -n auto` 运行同一完整测试集，由 `pytest-xdist` 根据当前机器自动
确定 worker 数，避免把开发机的逻辑核数写死到共享配置。

在本仓库当前 24 逻辑核开发机上，实测推荐的并行全量命令为：

```bash
uv run pytest -n 24
```

该选择来自同一测试版本对 `-n 4`、`-n 8` 和 `-n auto` 的实测，而不是假定
`auto` 总是最快；本机 `-n auto` 等价于 24 workers。其他机器先比较 `-n 4`、`-n 8` 与
`-n auto`，再固定适合本机内存和 CPU 的 worker 数。pytest 启动时把每个 worker 的
OpenMP／BLAS 线程默认限制为 1，并为每个进程使用独立 Matplotlib 配置与字体缓存目录，避免外层
进程并行与库内部线程叠加、字体缓存竞争。调用者显式设置的线程或 `MPLCONFIGDIR` 环境变量仍优先。

所有产物测试必须使用 `tmp_path` 或显式临时输出目录，不得把并行测试产物写入共享的
`outputs/`、资产目录或固定文件名。

2026-09-10 在 24 逻辑核开发机上的验收记录如下。`--durations` 仅增加耗时报告，不改变测试选择；
固定 24 workers 的两次稳定性运行未带该报告。

| 命令 | 结果 | 总耗时 |
| --- | --- | ---: |
| 优化前 `uv run pytest --durations=40` | 587 passed，1 skipped | 94.26 s |
| 优化后 `uv run pytest --durations=40` | 593 passed，1 skipped | 66.12 s |
| 优化后 `uv run pytest -n 4 --durations=10` | 593 passed，1 skipped | 44.52 s |
| 优化后 `uv run pytest -n 8 --durations=10` | 593 passed，1 skipped | 40.74 s |
| 优化后 `uv run pytest -n auto --durations=10`（24 workers） | 593 passed，1 skipped | 30.29 s |
| 优化后 `uv run pytest -n 24`（连续运行 1） | 593 passed，1 skipped | 24.81 s |
| 优化后 `uv run pytest -n 24`（连续运行 2） | 593 passed，1 skipped | 24.79 s |

串行结构优化相对基线缩短 29.9%，加速 1.43 倍；推荐并行命令两次平均 24.80 s，相对基线
加速 3.80 倍。24 workers 会让最重单项在竞争下略有变慢，因此不继续增加 worker；8 workers
资源更保守，但本机墙钟比推荐值多约 16 s。

## 增量测试

开发过程中可让透明映射脚本检查当前工作区改动并运行相关测试：

```bash
uv run python scripts/test_changed.py --list
uv run python scripts/test_changed.py
```

检查相对分支基线的已提交改动，同时纳入当前工作区改动：

```bash
uv run python scripts/test_changed.py --base origin/main
```

也可显式给出一个或多个变更模块，便于在编辑后立即反馈：

```bash
uv run python scripts/test_changed.py src/parallel_gripper_tactile/experiments/force_tracking.py
uv run python scripts/test_changed.py packages/dm_grasp_core/src/dm_grasp_core/control.py
```

映射规则位于 `scripts/test_changed.py` 顶部：测试文件映射到自身，workspace 包映射到本包测试目录，
主包和研究脚本先按同名测试前缀选择，并为控制、感知、场景、仿真和可视化补充显式跨模块映射。
测试基础设施、CI 配置、未映射代码、配置或资产改动会保守回退全量测试；纯文档改动不运行 pytest。
这套入口只缩短开发反馈，不替代 pre-commit 的并行完整测试或 CI 的串行完整测试。

常用的直接模块命令如下：

```bash
uv run pytest tests/test_force_tracking*.py
uv run pytest tests/test_friction_estimation*.py tests/test_taxel_friction.py
uv run pytest tests/test_force_scheduling*.py
uv run pytest packages/dm_grasp_core/tests
uv run pytest packages/robotiq_grasp_core/tests
```

`uv run pytest --lf` 只用于修复过程中重跑上一次失败；它依赖本地缓存且会跳过此前通过或未记录的
测试，不能用于提交门禁、CI 结论或声称全量测试通过。

## 仿真与绘图测试分工

- 摩擦估计保留完整“仿真—轨迹处理—绘图—manifest”端到端测试，低、中、高摩擦和双倍噪声
  四个标准场景及保守性、保持稳定性断言全部保留。
- 绘图内容测试使用小型合成数据，验证布局选择、统计值、缺失数据处理、单份 PNG 工件登记和非空
  输出；测试内以低分辨率 PNG 完成真实内容渲染，避免同一内容重复执行昂贵布局。
- `tests/test_plotstyle.py` 集中真实验证 IEEE 栏宽、600 DPI PNG、显式 PDF 请求的 MediaBox、字体嵌入配置和中文
  数学符号；`tests/test_force_tracking.py` 另保留一张代表性复杂力跟踪图的真实 600 DPI 契约。

修改出版导出公共函数、字体策略或图形尺寸时，必须显式运行上述两个契约测试文件，不能只运行使用
轻量渲染 fixture 的内容测试。

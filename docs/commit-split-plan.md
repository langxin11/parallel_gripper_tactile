# 分裂提交方案（待执行）

> 状态：仅方案留档，**未创建任何提交**（用户确认"只保留方案"）。
> 当前分支 `main` @ `f84b841`，工作树 113 项改动（77 修改 + 36 新增）全部未暂存，
> 改动按路径天然可分，无需逐 hunk 拆分。

## 提交顺序（依赖序：资产 → 库 → 脚本 → 测试 → 文档）

### 1. `feat(assets): 更新自研夹爪 CAD 导出（单根节点、真实质量、3×3 Pillars）`

- `assets/grippers/custom_parallel_gripper/**`：STL/part、`parallel_gripper.xml`、
  `parallel_gripper_prepared.xml`（运行时模型，被 profile 引用，必须提交）、
  `config.json`、`README.md`；删除旧 `pillars*.stl`、`frame.*`、`onshape_config.json`
- `configs/custom_parallel_gripper.toml`
- `assets/scenes/grasp_world.xml`（场景视觉/光照）

### 2. `feat(lib): 新增触觉读取器、统一扰动协议与视频工具`

- `src/parallel_gripper_tactile/`：`contact_taxels.py`、`protocols.py`、`video.py` 新增；
  `profiles.py`、`validation.py`、`cli.py`、`__init__.py` 修改

### 3. `refactor(scripts): 共用演示循环，新增自研夹爪实验脚本并归档输出`

- `scripts/**` 全部：`recording.py`（`run_demo_loop`，含 `__init__`/`__post_init__`
  docstring 补齐）、四个 demo、`compare_tactile_models.py`、
  `run_custom_grasp_validation.py`、`record_disturbance_video.py`
  （`width`/`height` 参数说明拆分）、`record_custom_grasp_video.py`
  （模块 docstring 改 `r"""`）、`generate_taxels_xml.py`（补 `shape` 参数说明）、
  `generate_touch_grid_xml.py`（`r"""` + 补 `rows`/`cols` 参数说明）、
  `verify_mujoco.py`、`custom_grasp_scene.py`、`view_custom_grasp_scene.py` 等

### 4. `test: 覆盖接触读取器与自研夹爪链路`

- `tests/**` 全部修改与新增；6 个测试文件的 24 个测试函数补齐中文 docstring

### 5. `docs: 更新 README、工作流与路线图，CI 强制 ruff D 规则`

- `README.md`（含「代码与注释规范」D 豁免收窄说明）、`docs/**`
  （含 `custom-gripper-next-phase.md` 状态版与本文档）、`CHANGELOG.md`、
  `LICENSE`、`pyproject.toml`（license 字段 + ruff D 规则，
  其中 `scripts/`、`tests/` 的 `per-file-ignores` 已收窄为语言相关的
  `D400`/`D401`/`D403`/`D404`/`D415` 五条）、`.gitignore`、`.github/workflows/ci.yml`

## 执行注意事项

- 提交信息使用 conventional 前缀（type/scope 保留英文关键词）+ **中文 subject**，
  与仓库中文文档一致；示例见上文各提交标题（如 `feat(assets): 更新自研夹爪 CAD 导出…`）。
  旧历史（`f84b841` 等）为英文 subject，属历史风格，不强求改写
- 完整校验（ruff + pytest 35 项）在最后一个提交后统一执行；
  中间提交的 pytest 可能因测试文件尚未纳入而部分不绿，属预期
- `robot.pkl` 不提交，已在 `.gitignore` 中忽略（可再生成中间缓存）
- 执行方式：`git add <路径组>` + `git commit`，按上述顺序逐组进行
- 分组以路径为准，不随提交信息文案变化；提交 3/4/5 的实际内容含
  D 豁免收窄后补齐的 docstring（见上文对应列表）

## 变更追踪

| 日期 | 变更 |
|---|---|
| 方案创建后 | `.gitignore` 新增 `robot.pkl` 忽略 |
| 方案创建后 | ruff `D` 豁免收窄为 5 条语言相关规则；`scripts/` 8 处 +
  `tests/` 24 处 docstring 缺口补齐；README 规范段与 CHANGELOG 同步更新 |
| 方案创建后 | commit message 风格确定为 conventional 前缀 + 中文 subject |
  （用户确认），提交示例已同步改写 |

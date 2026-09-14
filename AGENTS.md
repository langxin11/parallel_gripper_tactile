# AGENTS.md

本文件面向 AI 编码代理（Codex、Claude Code 等），汇总在本仓库工作的硬性约定与入口文档。
这里只做索引与摘要；与 `docs/` 下的正式规范冲突时，以正式规范为准。

## 项目定位

MuJoCo 平行夹爪触觉仿真与力控实验库。核心链路：YAML profile 校验 → 场景编译 → 仿真循环 →
触觉/接触力读取 → 控制器（MIT、力跟踪）→ 实验脚本与 study 聚合统计。

## 阅读入口

先读 `CONTRIBUTING.md` 的文档更新判断与本次相关规范，再按改动范围选择专题。

- `docs/architecture.md`：模块职责、仿真循环所有权、依赖规则。
- `docs/workflows.md`：`pgt` 演示／检查、Hydra 科研命令与正式研究执行顺序。
- `docs/research-configuration.md`：Hydra 科研入口、配置组、计划/执行、路径与产物契约。
- `docs/coding-conventions.md`：注释、docstring 与文档书写规范。
- `docs/testing.md`：串行门禁、并行全量、增量映射与绘图测试分工。
- `docs/force-tracking.md`、`docs/control-comparison-ablation.md`：力跟踪控制器与对比研究的方法和结论。
- `docs/adaptive-grasping.md`：自适应抓取路线关系，按需进入具体实验专题。
- `CONTRIBUTING.md`：文档更新判断与 PR 验收清单。

## 硬性约定

- **语言**：说明性文字（注释、docstring、文档、提交信息）一律简体中文，全角标点；
  代码标识符、CLI 参数、物理量符号保留英文。docstring 采用 Google 风格，
  `Args:`/`Returns:`/`Raises:` 等节标题保留英文字面值，节内文字用中文。
- **提交信息**：`type(scope): 中文主题`；type 取 `feat`/`fix`/`docs`/`refactor`/`test`/`chore`/`build`。
- **CHANGELOG**：面向使用者的变更（新命令、新配置字段、行为切换、修复）必须记入
  `CHANGELOG.md` 的 Unreleased 小节，按 Keep a Changelog 分类。
- **边界**：不得随意更改 profile schema、CLI 行为、数据格式与既有实验结论；确需变更时
  同步更新对应文档、测试与 CHANGELOG。
- **正式研究**：条件矩阵由领域 protocol 唯一生成，计划与执行共享同一 `StudyPlan`；公共生命周期只管
  状态、失败、聚合／绘图钩子和产物登记，不得把科学失败、执行异常或阶段谱系合并简化。

## 科研绘图偏好

- 使用 Matplotlib 绘制科研图表时，默认以可直接用于 IEEE 类双栏论文的质量交付，
  无需用户重复提醒；具体投稿模板或用户提供的参考图优先。
- 推荐使用 SciencePlots，先 `import scienceplots` 注册样式，再使用
  `plt.style.context(["science", "ieee", "no-latex"])`；样式名称为 `no-latex`，
  不依赖外部 LaTeX。优先复用项目统一样式与绘图函数。
- 中文图表应在应用样式后配置实际可用的中文字体，例如 Noto Serif CJK SC 或思源宋体，
  并检查中文、数学符号与负号是否正常显示；不假定 SciencePlots 自带字体。
  图中文字语言按交付用途与用户要求确定。
- 按最终单栏或双栏尺寸设计，保证缩放后的字号、线条与标记清晰；使用简洁白底，
  统一字体与数学符号样式，坐标轴标明物理量与单位。
- 同一方法跨图保持相同颜色、线型与标记；兼顾色觉障碍与灰度打印，
  避免仅依赖颜色区分。控制留白与网格，避免图例遮挡、文字重叠和裁切。
- 优先交付矢量 PDF，同时提供 PNG 预览及可复现的绘图代码；
  交付前实际渲染并检查最终尺寸下的可读性，不能仅凭已应用样式判断合格。
- 保持数据与实验结论不变，不为美观擅自平滑、删除异常值或改变统计口径。

## 验证

权威命令、pre-commit／CI 全量门禁和增量检查统一见 [测试策略](docs/testing.md)。
按改动范围完成开发验证，提交前先自查；增量测试与失败重跑不能替代完整提交门禁。

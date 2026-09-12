# 科研报告的组织与维护

本页负责 `reports/` 的目录、编译和维护说明；科研内容均以 `.typ` 保存。
报告围绕“目标力跟踪控制器筛选 → 轻柔接触 → 自适应持握 → 防滑搬运与稳定放置”组织证据。
`docs/` 保存稳定规范，`reports/` 保存文献判断、实验结果与论文工作稿，不自动回写配置或方法结论。

## 前置要求

- Typst CLI ≥ 0.14（在 0.15.0 上验证通过），需能直接执行 `typst` 命令；
- 本地安装中文适配版 `@local/wired-ieee:1.0.0`；实验报告与既有合集采用 IEEE 双栏会议版式，
  西文使用 Times 系的 TeX Gyre Termes，中文回退到 Noto Serif CJK SC；
- 简体中文字体 Noto Serif CJK SC 与 Noto Sans CJK SC：缺字体时编译不报错，
  但对应文字会渲染成方框；
- 首次编译需联网下载 `@preview/mitex` 包（0.2.7 已验证），之后使用本地缓存，
  离线也能编译。

## 目录与编译入口

`reports/` 的报告正文统一使用 Typst，不再维护 Markdown 副本。根目录只保留编译入口、模板与引用资源，
可复用正文放在 `chapters/`，冻结图表放在 `figures/`。

| 入口 | 内容 | 正文来源 |
| --- | --- | --- |
| [`index.typ`](../reports/index.typ) | 报告总目录与阅读路线 | 入口内维护简短导读。 |
| [`literature.typ`](../reports/literature.typ) | 10 篇围绕轻柔抓取的重点论文指南与三篇刚度专题总结 | `chapters/literature/`。 |
| [`experiments.typ`](../reports/experiments.typ) | 位置限幅、速率结构验证与小规模参数调优 | `chapters/experiments/`，按研究先后排列。 |
| [`combined.typ`](../reports/combined.typ) | 既有四部分研究合集 | 保留既有冻结数据块、正文、图表与生成脚本契约。 |

在仓库根分别编译：

```bash
typst compile --root . reports/index.typ
typst compile --root . reports/literature.typ
typst compile --root . reports/experiments.typ
typst compile --root . reports/combined.typ
```

四个 PDF 与入口同名，可从 `index.pdf` 的链接打开。阅读指南和目录采用单栏，长题名与比较表更易阅读；
近期实验及既有合集使用中文适配的 IEEE 双栏，宽数字表跨栏。章节本身不是独立编译入口。
`--root .` 只允许编译器访问仓库内模板、冻结数据和图像，编译不读取 `outputs/` 或外部 Obsidian 文献。
本地笔记只作为链接保留。PDF 是可再生预览，`reports/*.pdf` 仍由 `.gitignore` 排除。
若本地 `.git/info/exclude` 还忽略整个 `reports/`，新源文件也不会自动出现在 Git 状态中；本次不改动该本地规则。

既有 `combined.typ` 保留四个部分：方法说明、研究证据汇总、摩擦感知目标力论文工作稿、
等效接触刚度参考验证。它是阶段性证据快照，不自动代表最新配置。近期三轮研究单列在 `experiments.typ`，
前后轮决策按发生顺序保留，不把后续结论追写成早期研究的既定事实。

报告表格统一采用三线表，不使用竖线或交替底色。合并只把各部分章节标题整体下移一级，
正文、数据与结论边界逐字保留。

合集第二部分的数字**字面写在文件内的数据块里**，其余部分的数字同样全部为字面量：编译不读取
`outputs/`，产物被清理或迁移后报告仍可打开，数据也不会随产物静默变化。数据块由下面的 Python 脚本从
study 产物一次性校验、汇总并冻结；Typst 只负责排版：

```bash
uv run python scripts/reports/study_results_data.py            # 换 run 后刷新数据块
uv run python scripts/reports/study_results_data.py --check    # 校验数据块与产物一致
```

脚本读取的 run 常量写在同一文件顶部；`tests/test_study_results_data.py` 用最小 study 树验证生成逻辑，
并在本机产物存在时校验已提交的数据块没有过期（产物缺失则跳过）。

## LaTeX 公式的双反斜杠约定

公式经 `mitex` 从字符串解析 LaTeX 语法（模板的 `formula()` 已封装居中展示），
而 Typst 字符串中 `\` 是转义符：`\r`、`\t` 会被当成回车、制表符吃掉，公式悄悄残缺
且不报错。因此字符串里的 LaTeX 反斜杠必须双写：

```typst
#formula("\\rho = \\frac{T_L + T_R}{N_L + N_R}")   // 正确：渲染出 ρ
#formula("\rho")    // 错误：\r 被解析为回车转义，公式残缺
```

## 如何更新证据

正式报告不直接绑定源目录。先由 Python 汇总程序读取并校验研究产物，人工审阅中间数据后再冻结到报告；
插图同样复制到 `reports/figures/` 作为受控快照，保证论文不会因清理 `outputs/` 而变化。合集内所有数字
均为字面量，新增内容也不应引入编译期直读。若产物不存在，先重跑对应实验：

```bash
uv run pgt run friction-estimate --experiment dm_gripper/friction_estimation_nominal
uv run pgt run force-schedule --experiment dm_gripper/force_scheduling_dynamic_filling

cp outputs/dm_gripper/friction-estimate/<run>/plot.pdf reports/figures/friction_estimate.pdf
cp outputs/dm_gripper/force-schedule/<run>/plot.pdf reports/figures/force_schedule.pdf
```

合集第二部分的表格与正文数字来自文件内的数据块，换 run 时不需要改报告正文：更新
`scripts/reports/study_results_data.py` 顶部的 run 常量，运行该脚本刷新数据块，并把新 run 的
`figures/*.pdf` 重新复制到 `reports/figures/`（现有快照名为 `controller_metrics.pdf`、
`pid_factorial_effects.pdf`、`adrc_candidate_ranking.pdf`、`estimator_delta_vs_secant.pdf`）。
正式 study 的重跑入口见 [科研配置与实验编排](research-configuration.md)。

## 新增章节与迁移规则

- 旧图按最终版面可读性取舍。九宫格控制器指标图与密集 ADRC 候选排名图已退出合集，相关数值表与结论保留；
  原 PDF 资产暂留作历史快照，不再参与编译。历史摩擦曲线明确标注残差检测器版本，不能解释为当前检测器结果。
- 更新图时使用同一批冻结数据和统计口径；不能以新实验曲线替换旧图后继续沿用旧结果说明。
- 文献正文放在 `reports/chapters/literature/`，实验正文放在 `reports/chapters/experiments/`；由相应入口 `#include`。
- 章节不设置全局 `#show`／页面；因 Typst 文件作用域独立，章节显式导入所需组件，入口统一应用样式。
- `collection-template.typ` 复用既有 `template.typ`；新增 `reading-report`、`experiment-report` 与原生三线表 `review-table`。
- Markdown 迁移后核对段落、表格数字、公式与链接，编译并查看渲染页，再删除旧正文，避免两个版本漂移。
- 新结果新增章节；保持已有运行结果、数据块、统计口径与结论原样，研究先后关系在入口说明。
- 本次迁移对应关系：`literature_reading_guide.md` → `chapters/literature/reading_guide.typ`；
  `三篇核心文献_扼要总结.typ` → `chapters/literature/stiffness_related_work.typ`；
  三份 `stiffness_*.md` → `chapters/experiments/` 下的同名 `.typ`；`reports/README.md` → 本页。

## 模板能力速览

`template.typ` 提供以下组件：

- `report()`：文档骨架与中文字体设置；
- `formula()`：居中展示 LaTeX 公式（`mitex` 兼容层），`formula-box()` 给重点公式加边框；
- `csv-table()`、`metrics-table()`、`run-header()`：仅供模板冒烟测试（`tests/fixtures/report_smoke.typ`），
  不用于新增正式科研报告；合集第三部分的运行信息与指标表已改为字面量；
- `figure-grid()`：把多张矢量 PDF 图组合成网格；
- `data-table()`：渲染字面量记录数组，列定义给出中文表头与有效数字位数，数值列右对齐、布尔转中文、
  空值占位，`span: true` 时跨栏浮动到页顶（IEEE 双栏版式用）。

`fmt-float()` 现在会把尾零补足到该列应有小数位（`0.15` 与 `0.213` 并排显示为 `0.150` 与 `0.213`），
整数按整数显示；已有报告的数字与结论不变，只是显示更整齐。

`tests/test_report_typst.py` 用 `tests/fixtures/` 的迷你数据（含 CRLF CSV）编译 fixture 报告做冒烟测试，
覆盖字面量数据表与其余模板能力；仅当本机装有 Typst CLI 时执行（`shutil.which("typst")` 判定），
CI 无 CLI 环境自动跳过。

# 科研报告的组织与维护

`docs/` 保存当前方法与接口，`reports/` 保存文献判断、实验结果和论文工作稿。报告正文使用 Typst，
新结果新增章节并注明研究条件；不会自动回写配置或覆盖既有实验结论。

## 环境与编译

需要 Typst CLI ≥ 0.14、本地 `@local/wired-ieee:1.0.0`、TeX Gyre Termes 和 Noto Serif／Sans CJK SC
字体；首次编译需获取 `@preview/mitex`。缺字体可能不报错，必须检查渲染页。

| 入口 | 内容 |
| --- | --- |
| `reports/index.typ` | 报告总目录与阅读路线。 |
| `reports/literature.typ` | 文献指南；正文在 `chapters/literature/`。 |
| `reports/experiments.typ` | 实验结果；正文在 `chapters/experiments/`。 |
| `reports/combined.typ` | 已冻结研究合集，按原实验条件解释。 |

在仓库根执行；将 `index.typ` 换成所需入口即可：

```bash
typst compile --root . reports/index.typ
```

PDF 与入口同名，不入库。章节只导入组件并提供正文，页面和全局样式由入口统一设置；
阅读指南采用单栏，实验报告采用 IEEE 双栏，表格使用三线表。

## 数据与图表

报告数字冻结为字面量，图表保存到 `reports/figures/`；编译不直接读取 `outputs/`。
更新时先核对产物的配置、版本、指标与统计口径，再冻结数值和图像；不得换新图后沿用旧结论。
合集数据由 `scripts/reports/study_results_data.py` 顶部指定来源：

```bash
uv run python scripts/reports/study_results_data.py
uv run python scripts/reports/study_results_data.py --check
```

脚本顶部 `STUDIES` 指向 `outputs/studies/` 下的固定 run 目录；该树对应合集冻结时的产物，
**现已清理**，因此脚本当前无法运行（`--check` 会以「缺少产物文件」退出 1）。这不影响报告编译：
合集数字是已冻结的字面量，清理后报告仍然可读。待重跑对应 study 后，需先把 `STUDIES` 的路径
更新为新的 run 目录，再刷新数据块。

已有合集是阶段性证据，不自动代表当前配置。历史摩擦图按其检测器版本解释，不能用作新检测器的验证。
新增章节后编译并检查数字、公式、引用及最终尺寸下的可读性。

## LaTeX 公式的双反斜杠约定

公式经 `mitex` 从字符串解析 LaTeX 语法（模板的 `formula()` 已封装居中展示），
而 Typst 字符串中 `\` 是转义符：`\r`、`\t` 会被当成回车、制表符吃掉，公式悄悄残缺
且不报错。因此字符串里的 LaTeX 反斜杠必须双写：

```typst
#formula("\\rho = \\frac{T_L + T_R}{N_L + N_R}")   // 正确：渲染出 ρ
#formula("\rho")    // 错误：\r 被解析为回车转义，公式残缺
```

## 模板与验证

`template.typ` 提供 `report()`、`formula()`、`formula-box()`、`figure-grid()` 与字面量
`data-table()`；`collection-template.typ` 提供阅读／实验版式和 `review-table`。
`csv-table()`、`metrics-table()`、`run-header()` 仅供模板测试，正式报告使用冻结数据。

`tests/test_report_typst.py` 在 Typst 可用时编译 fixture；`tests/test_study_results_data.py`
验证数据生成，并在来源产物存在时检查冻结值。自动测试不能替代渲染检查。

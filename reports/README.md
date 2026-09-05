# 📊 实验报告与论文工作稿（Typst）

本目录存放用 Typst 编写的实验报告与论文工作稿。两者都是时点性交付物：头部记录所引用 run 的 ID 与
git 提交，正文数值全部程序化读取自 `outputs/` 产物，看到的每个数字都能溯源到具体 run。
`docs/` 只保留可复现的定性结论，两者互补——报告给出某次实验的定量快照，docs 沉淀跨版本
仍然成立的结论；报告不回写 docs/。

## 前置要求

- Typst CLI ≥ 0.14（在 0.15.0 上验证通过），需能直接执行 `typst` 命令；
- 简体中文字体 Noto Serif CJK SC 与 Noto Sans CJK SC：缺字体时编译不报错，
  但对应文字会渲染成方框；
- 首次编译需联网下载 `@preview/mitex` 包（0.2.7 已验证），之后使用本地缓存，
  离线也能编译。

## 编译与查看

在仓库根执行：

```bash
typst compile --root . reports/wired_demo.typ
```

`--root .` 授权 Typst 读取仓库内文件（表格与图直接来自 `outputs/`）。
输出 `reports/wired_demo.pdf`；产物 PDF 不入库，
`reports/*.pdf` 已加入 `.gitignore`。

## LaTeX 公式的双反斜杠约定

公式经 `mitex` 从字符串解析 LaTeX 语法（模板的 `formula()` 已封装居中展示），
而 Typst 字符串中 `\` 是转义符：`\r`、`\t` 会被当成回车、制表符吃掉，公式悄悄残缺
且不报错。因此字符串里的 LaTeX 反斜杠必须双写：

```typst
#formula("\\rho = \\frac{T_L + T_R}{N_L + N_R}")   // 正确：渲染出 ρ
#formula("\rho")    // 错误：\r 被解析为回车转义，公式残缺
```

## 如何换数据源

报告头部的 `#let …-run = "/outputs/…"` 常量声明各数据源 run 的产物目录，改成新
run 的路径后重新编译即可。插图不直接引用 `outputs/`：把所需 run 的 `plot.pdf`
复制到 `reports/figures/` 作为入库快照（换数据时重新复制替换），保证论文的图像
资产固定、不随 `pgt runs clean` 丢失。若产物不存在，先重跑对应实验：

```bash
uv run pgt run friction-estimate --profile configs/custom_parallel_gripper.yaml --task configs/friction_estimation/nominal_friction.yaml
uv run pgt run force-schedule --profile configs/custom_parallel_gripper.yaml --task configs/force_scheduling/dynamic_filling.yaml

cp outputs/custom_parallel_gripper/friction-estimate/<run>/plot.pdf reports/figures/friction_estimate.pdf
cp outputs/custom_parallel_gripper/force-schedule/<run>/plot.pdf reports/figures/force_schedule.pdf
```

## 模板能力速览

`template.typ` 提供以下组件：

- `report()`：文档骨架与中文字体设置；
- `formula()`：居中展示 LaTeX 公式（`mitex` 兼容层），`formula-box()` 给重点公式加边框；
- `csv-table()`：直读 `outputs/` 下 study 的 `summary.csv`、`aggregate.csv` 生成表格；
- `metrics-table()`：读取单次运行的 `metrics.json`，渲染成键值表；
- `run-header()`：读取 `manifest.json`，显示 run ID、git 提交与创建时间；
- `figure-grid()`：把多张矢量 PDF 图组合成网格。

`tests/test_report_typst.py` 用 `tests/fixtures/` 的迷你数据编译 fixture 报告做冒烟
测试；仅当本机装有 Typst CLI 时执行（`shutil.which("typst")` 判定），CI 无 CLI 环境
自动跳过。

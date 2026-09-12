// 报告模板编译冒烟测试的 fixture：以最小数据完整走一遍 template.typ 的公开能力。
// 真实使用见 reports/wired_demo.typ；本文件不依赖 outputs/ 产物，
// 因此在没有运行过实验的环境（如 CI）也能编译。
#import "../../reports/template.typ": *

#show: report.with(title: "模板冒烟测试", subtitle: "fixture 数据")

= 表格能力

`csv-table` 选列渲染（fixture CSV 带 CRLF 行尾与空串）：

#csv-table(
  "/tests/fixtures/report_mini_summary.csv",
  columns: ("variant", "material", "passed", "rmse_n"),
  caption: [CSV 直读表],
)

`metrics-table` 按指定条目顺序渲染键值表：

#metrics-table(
  "/tests/fixtures/report_mini_metrics.json",
  entries: (
    ("ratio", "估计比"),
    ("rmse_n", "跟踪 RMSE"),
    ("passed", "判定"),
  ),
  include-rest: true,
)

= 字面量数据表

`data-table` 直接渲染写在文件里的字面量记录（数值列右对齐、布尔转中文、空值占位）：

#let rows = (
  (variant: "pid", material: "hard", rmse: 0.0407, passed: true, runs: 2),
  (variant: "adrc", material: "hard", rmse: none, passed: false, runs: 1),
)

#data-table(
  rows,
  columns: (
    ("variant", "变体"),
    ("material", "材料"),
    ("rmse", "RMSE（N）"),
    ("passed", "通过"),
    ("runs", "条件数", 0),
  ),
  caption: [字面量记录三线表],
)

= 元信息与图

#run-header("/tests/fixtures/mini_run", [smoke])

#figure-grid(
  ("/tests/fixtures/report_panel_a.png", "/tests/fixtures/report_panel_b.png"),
  caption: [子图网格],
)

= 公式

#formula("f_{ref}=\\operatorname{clip}\\!\\left(\\frac{\\gamma D}{2\\mu},\\ f_{min},\\ f_{max}\\right).")

#formula-box[$J_f(q) \\approx k_("pair") J_c(q)$]

原生 math：$ |f_("ref,k") - f_("ref,k-1")| <= dot(f)_("max") Delta t $

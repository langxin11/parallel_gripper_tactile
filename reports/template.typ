// 实验报告共享模板。
//
// 用法：报告文件 `#import "./template.typ": *` 后用 `#show: report.with(title: …)`
// 包裹正文；在仓库根执行 `typst compile --root . reports/<name>.typ` 编译。
// 数据路径一律以 `/` 开头（相对仓库根）；LaTeX 字符串内的反斜杠必须双写
// （如 `"\\rho"`），否则 `\r`、`\t` 会被 Typst 当成转义符吃掉。
#import "@preview/mitex:0.2.7": mitex

// 西文使用 Times New Roman，中文回退到系统 Noto 宋体系；IEEE 工作稿由
// wired-ieee 使用度量兼容的 TeX Gyre Termes，二者均为 Times 系衬线风格。
#let cn-serif = ("Times New Roman", "Noto Serif CJK SC")

// 仅匹配十进制与科学计数法；不匹配布尔、空串与普通文本。
#let numeric-regex = regex("^-?[0-9]+(\\.[0-9]+)?([eE][+-]?[0-9]+)?$")

// 按有效数字位数格式化浮点数，避免表格里出现 17 位精度。
#let fmt-float(value, digits: 3) = {
  if value == 0 { return "0" }
  let magnitude = calc.floor(calc.log(calc.abs(value), base: 10))
  let decimals = calc.max(0, digits - 1 - magnitude)
  str(calc.round(value, digits: decimals))
}

// 把产物里的原始值转成表格单元格内容：字符串按需转数值，布尔转中文，空串占位。
#let cell-value(value, digits: 3) = {
  if type(value) == str {
    if value == "" { [—] }
    else if value.contains(numeric-regex) { fmt-float(float(value), digits: digits) }
    else { value }
  } else if type(value) == bool {
    if value { [是] } else { [否] }
  } else if type(value) == float {
    fmt-float(value, digits: digits)
  } else {
    repr(value)
  }
}

// 文档骨架：A4 页面、中文排版、编号标题、页码与居中标题块。
#let report(title: "实验报告", subtitle: none, date: none, body) = {
  set text(font: cn-serif, size: 10.5pt, lang: "zh", region: "cn")
  set page(
    paper: "a4",
    margin: (top: 2.5cm, bottom: 2.5cm, x: 2.4cm),
    numbering: "1 / 1",
    footer: context align(center)[
      #set text(size: 9pt, fill: luma(120))
      #counter(page).display("1")
    ],
  )
  set heading(numbering: "1.1")
  set par(justify: true, leading: 0.75em)
  set figure(numbering: "1")
  show figure.caption: set text(size: 9pt)
  show heading.where(level: 1): it => {
    v(0.4em, weak: true)
    text(size: 14pt, weight: "bold", it)
    v(0.4em, weak: true)
  }
  show heading.where(level: 2): it => {
    v(0.3em, weak: true)
    text(size: 12pt, weight: "bold", it)
    v(0.3em, weak: true)
  }

  align(center)[
    #text(size: 18pt, weight: "bold")[#title]
    #if subtitle != none {
      v(6pt)
      text(size: 12pt, fill: luma(100))[#subtitle]
    }
    #v(6pt)
    #text(size: 10pt, fill: luma(100))[
      #if date == none { datetime.today().display("[year] 年 [month] 月 [day] 日") } else { date }
    ]
  ]
  v(1.2em)
  body
}

// 居中展示一条 LaTeX 公式（块级）。注意源串内反斜杠双写。
#let formula(src) = block(above: 1.1em, below: 1.1em, align(center, mitex(src)))

// 给重点公式加边框（对应 LaTeX 的 \boxed 效果）。
#let formula-box(body) = box(
  stroke: 0.6pt + luma(80),
  inset: 8pt,
  radius: 2pt,
  body,
)

// 三线表表头单元格：不使用底色和竖线，仅加粗。
#let header-cell(body) = strong(body)

// 读取 CSV 产物并渲染为带表头的表格。
//
// Args:
//     path: 基于仓库根的 CSV 路径（以 `/` 开头），首行为表头。
//     columns: 选取的列名数组；缺省取全部列。列名不存在时编译报错。
//     digits: 数值列保留的有效数字位数。
//     caption: 表标题；提供时以编号图表形式呈现。
#let csv-table(path, columns: none, digits: 3, caption: none) = {
  let data = csv(path)
  let header = data.first()
  let body-rows = data.slice(1)
  let picked = if columns == none {
    range(header.len())
  } else {
    columns.map(c => {
      let index = header.position(s => s == c)
      if index == none { panic("csv-table：列不存在：" + c) }
      index
    })
  }
  // 按首行数据判定各列对齐：数值列右对齐，其余左对齐。
  let aligns = picked.map(i => {
    if body-rows.len() > 0 and body-rows.first().at(i).contains(numeric-regex) {
      right
    } else {
      left
    }
  })
  let to-cells(row) = picked.map(i => {
    let content = cell-value(row.at(i), digits: digits)
    content
  })
  let body-content = body-rows.map(row => to-cells(row)).flatten()
  let table-content = table(
    columns: picked.len(),
    align: aligns,
    inset: (x: 4pt, y: 3pt),
    stroke: none,
    table.hline(stroke: 0.8pt),
    table.header(..picked.map(i => header-cell(header.at(i)))),
    table.hline(stroke: 0.45pt),
    ..body-content,
    table.hline(stroke: 0.8pt),
  )
  if caption != none {
    figure(kind: table, supplement: [表], caption: caption, table-content)
  } else {
    table-content
  }
}

// 读取单次运行的 metrics.json 并按给定条目顺序渲染键值表。
//
// Args:
//     path: 基于仓库根的 metrics.json 路径。
//     entries: ((字段名, 中文说明), …) 数组，决定展示顺序与释义。
//     include-rest: 是否把未列出的字段追加到表尾（无释义）。
//     digits: 数值保留的有效数字位数。
#let metrics-table(path, entries: (), include-rest: true, digits: 3) = {
  let data = json(path)
  let listed = entries.map(((key, _)) => key)
  let rest = data.keys().filter(key => not listed.contains(key))
  let row-of(key, desc) = (raw(key), cell-value(data.at(key), digits: digits), desc)
  table(
    columns: (auto, auto, 1fr),
    align: (left, right, left),
    inset: (x: 4pt, y: 3pt),
    stroke: none,
    table.hline(stroke: 0.8pt),
    table.header(header-cell([字段]), header-cell([数值]), header-cell([说明])),
    table.hline(stroke: 0.45pt),
    ..entries.map(((key, desc)) => row-of(key, desc)).flatten(),
    ..(if include-rest { rest.map(key => row-of(key, "")) } else { () }).flatten(),
    table.hline(stroke: 0.8pt),
  )
}

// 渲染单次运行的元信息块：run ID、git 提交与脏标记、创建时间。
//
// Args:
//     run-dir: 基于仓库根的运行目录（以 `/` 开头），内含 manifest.json。
//     label: 块首行的实验名称。
#let run-header(run-dir, label) = {
  let manifest = json(run-dir + "/manifest.json")
  let run-id = run-dir.split("/").filter(s => s != "").last()
  let git-state = manifest.at("git", default: none)
  let commit = if git-state != none { git-state.at("commit") } else { "未知" }
  let dirty = if git-state != none and git-state.at("dirty", default: false) { "是" } else { "否" }
  block(
    fill: luma(248),
    inset: 9pt,
    radius: 3pt,
    width: 100%,
    table(
      columns: (auto, 1fr),
      align: (right, left),
      stroke: none,
      inset: (x: 8pt, y: 2pt),
      text(size: 9pt)[*#label*],
      text(size: 9pt)[#raw(run-id)],
      text(size: 9pt)[*git 提交*],
      text(size: 9pt)[#raw(commit.slice(0, 8))（工作区有未提交改动：#dirty）],
      text(size: 9pt)[*创建时间*],
      text(size: 9pt)[#manifest.at("created_at")],
    ),
  )
}

// ——— 研究级数据表：字面量记录 → 三线表 ———
//
// 研究级报告的数字直接写在报告文件里（见 `reports/study_results.typ` 的数据块，
// 由 `scripts/reports/study_results_data.py` 从产物生成），编译不读取 `outputs/`。
// 本组件只负责把这些字面量记录渲染成与 `csv-table` 同款的三线表。

// 判断取值是否按数值列处理：字面量 int／float，或产物 CSV 里的十进制、科学计数法字符串。
#let is-numeric(value) = (
  type(value) == int
    or type(value) == float
    or (type(value) == str and value != "" and value.contains(numeric-regex))
)

// 渲染三线表：列定义决定列顺序、中文表头与各自的有效数字位数。
//
// Args:
//     records: 记录数组，字段名与 `columns` 中的字段名对应；数值列可给字面量
//         int／float，也可给 CSV 字符串。
//     columns: `((字段名, 中文表头), …)` 或 `((字段名, 中文表头, 有效数字), …)`。
//     caption: 表标题；提供时按模板约定编号。
//     digits: 未单独指定时的有效数字位数。
//     span: 是否跨栏浮动到页顶（IEEE 双栏版式用；单栏版式保持 false）。
//     widths: 可选列宽数组，覆盖默认等宽（span 模式为 auto + 1fr）；用于在栏宽内
//         加宽长文本列，避免与相邻数值列相碰。
//     empty: 空串与缺失字段的占位内容。
#let data-table(
  records,
  columns: (),
  caption: none,
  digits: 3,
  span: false,
  widths: none,
  empty: [—],
) = {
  let specs = columns.map(spec => {
    if type(spec) == array and spec.len() == 3 { spec } else { (spec.at(0), spec.at(1), digits) }
  })
  // 数值列右对齐：该列只要出现过数值即按数值列处理。
  let aligns = specs.map(spec => {
    if records.any(r => is-numeric(r.at(spec.at(0), default: ""))) { right } else { left }
  })
  let body-cells = records.map(record => specs.map(spec => {
    let value = record.at(spec.at(0), default: none)
    if value == none or value == "" { empty } else { cell-value(value, digits: spec.at(2)) }
  })).flatten()
  let content = table(
    columns: if widths != none {
      widths
    } else if span {
      (auto,) + range(specs.len() - 1).map(_ => 1fr)
    } else {
      specs.len()
    },
    align: aligns,
    inset: (x: 4pt, y: 3pt),
    stroke: none,
    table.hline(stroke: 0.8pt),
    table.header(..specs.map(spec => header-cell(spec.at(1)))),
    table.hline(stroke: 0.45pt),
    ..body-cells,
    table.hline(stroke: 0.8pt),
  )
  if caption == none { return content }
  let rendered = figure(kind: table, supplement: [表], caption: caption, content)
  if span {
    place(top + center, scope: "parent", float: true, rendered)
  } else {
    rendered
  }
}

// 以网格组合多张图（推荐矢量 PDF）并配总编号标题，子图自动标注 (a)(b)…。
//
// Args:
//     images: 图路径数组（基于仓库根）。
//     caption: 总标题；缺省时只输出网格，不包编号图表。
//     columns: 列数；缺省按最多两列自动排布。
//     gutter: 子图间距。
//     placement: 浮动位置（如 `top`）；缺省原位排布。
//     labels: 子图标签数组；缺省自动 (a)(b)…。
#let figure-grid(
  images,
  caption: none,
  columns: auto,
  gutter: 10pt,
  placement: none,
  labels: auto,
) = {
  let count = images.len()
  let cols = if columns == auto { calc.min(count, 2) } else { columns }
  let label-of(i) = if labels == auto { numbering("(a)", i + 1) } else { labels.at(i) }
  let grid-content = grid(
    columns: range(cols).map(_ => 1fr),
    gutter: gutter,
    ..images.enumerate().map(((i, path)) => align(center)[
      #image(path)
      #v(3pt)
        #text(size: 9pt)[#label-of(i)]
      ]).flatten(),
  )
  if caption != none {
    figure(placement: placement, caption: caption, grid-content)
  } else {
    grid-content
  }
}

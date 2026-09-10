// 摩擦感知目标力调度论文（工作稿）：wired-ieee 中文论文外壳 +
// 复用 reports/template.typ 的产物取数组件。编译：
//   typst compile --root . reports/wired_demo.typ
#import "./template.typ": metrics-table, run-header, formula, formula-box

#let fmt-percent(v) = str(calc.round(v * 100, digits: 1)) + "%"
#let fmt-n(v) = str(calc.round(v, digits: 3))

#let friction-run = "/outputs/dm_gripper/friction-estimate/20260905T112622Z-60d6d822"
#let schedule-run = "/outputs/dm_gripper/force-schedule/20260905T112627Z-cd1f662d"
#let fm = json(friction-run + "/metrics.json")
#let sm = json(schedule-run + "/metrics.json")

// wired-ieee 原版把西文字体硬编码为 TeX Gyre Termes 且无 CJK 回退，本机
// 自动 fallback 会静默丢字/错字（「抓」「并」消失、「计」变形近字）。
// 这里使用打过中文适配补丁的本地分支 @local/wired-ieee:1.0.0（补丁共四处：
// 全局字体列表补 Noto Serif CJK SC、标题行距 0.5em→0.9em、摘要去伪斜体、
// 摘要行距 0.45em→0.55em），改动说明见仓库 README 的 install.sh 本地安装路径。
#import "@local/wired-ieee:1.0.0": ieee
#show: ieee.with(
  title: [摩擦感知目标力调度：基于微滑移探测的保守摩擦估计与闭环验证],
  authors: (
    (name: "肖大亮", organization: "北京理工大学空天学院", email: "2658327508@qq.com"),
  ),
  abstract: [平行夹爪抓取中，夹持力过小导致滑落、过大损伤物体。本文在 MuJoCo 触觉仿真中验证一条摩擦感知力控闭环：以受控微滑移探测聚合两侧触觉法向与切向分量，取临滑前比值分位数并折减，得到保守摩擦下界；再将估计值代入安全系数调度式生成目标法向力。在摩擦系数 0.8 的方物与线性增长切向载荷场景中，估计值为真值的 92.0%，闭环峰值摩擦利用率 75.8%，全程无宏滑移，力跟踪 RMSE 0.0299 N。],
  index-terms: ("触觉传感", "摩擦估计", "力控", "平行夹爪"),
  bibliography: bibliography("wired_demo.bib"),
  lang: "en",
)

= 引言

抓取力控的核心矛盾在于摩擦系数先验未知：保守定值夹持浪费夹持容量并增加损伤风险，而过小则存在滑落风险 @mujoco。现有工作多依赖已知摩擦的模型调度或力矩抖动观测，对触觉阵列的逐侧比值信号利用不足 @manipulation。本文的贡献是给出一条从微滑移探测到保守估计再到目标力调度的完整闭环，并在确定性仿真中量化其保守性与跟踪品质。

= 方法

== 逐侧聚合与摩擦比

第 $s$ 侧触觉 site 的法向合力 $N_s$ 取截断正值，切向合力 $T_s$ 取两轴分量模长，全局摩擦比 $rho$ 定义为：

#formula("N_s=\\max(0,F_{z,s}),\\qquad T_s=\\sqrt{F_{x,s}^2+F_{y,s}^2}")

#formula("\\rho=\\frac{T_L+T_R}{N_L+N_R}.")

探测段以缓慢增速的法向力加载；比值斜率先后越过 arming 与 saturation 阈值且双侧差异在确认窗内持续，即判定临滑。

== 保守下界估计

取临滑前窗口内 $rho_k$ 样本的 $q$ 分位数，乘以折减系数 $eta$ 并截断到先验区间：

#formula-box(formula("\\hat\\mu_{LB}=\\operatorname{clip}\\!\\left(\\eta Q_q\\{\\rho_k\\}_{\\mathrm{pre\\text{-}slip}},\\ \\mu_{min},\\ \\mu_{max}\\right)."))

== 目标力调度

调度器按安全系数 $gamma$ 把切向需求 $D$ 换算为目标法向力，摩擦感知闭环以 $hat(mu)_("LB")$ 替代真值 $mu$：

#formula("f_{ref}=\\operatorname{clip}\\!\\left(\\frac{\\gamma D}{2\\hat\\mu_{LB}},\\ f_{min},\\ f_{max}\\right)")

#formula("\\left|f_{ref,k}-f_{ref,k-1}\\right|\\leq \\dot f_{max}\\Delta t.")

= 实验

仿真基于 DM_Gripper（曲柄滑块传动，触觉阵列 5×5×2 taxel），控制周期 2 ms，NoSlip 迭代 5 次，真值摩擦系数 0.8。估计运行的产物目录与 git 提交如下（数值由编译时程序化读取）：

#run-header(friction-run, [friction-estimate])

摩擦估计运行的量化结果见表 1，全过程曲线见图 1。

#metrics-table(
  friction-run + "/metrics.json",
  entries: (
    ("estimated_friction_coefficient", "保守估计"),
    ("true_friction_coefficient", "仿真真值"),
    ("estimate_ratio", "估计/真值"),
    ("probe_force_at_detection_n", "探测时法向力"),
    ("hold_force_tracking_rmse_n", "保持段 RMSE"),
  ),
  include-rest: false,
)

// 插图约定：所需的产物图复制到 reports/figures/ 作为入库快照（重跑实验后重新
// 复制并替换），论文编译不依赖 outputs/ 中的图像文件。产物图设计宽度 7.16 in
// 与版面文本宽度基本一致，以跨栏浮动嵌入后不做缩放，图内字号所见即所得。
#place(
  top + center,
  scope: "parent",
  float: true,
  figure(
    image("/reports/figures/friction_estimate.pdf", width: 100%),
    caption: [
      摩擦估计运行全程产物图，自上而下：切向载荷指令与触觉剪切响应；载荷—剪切支撑残差（虚线为判定阈值）；保守摩擦系数估计（虚线为仿真真值，仅用于评分）；探测与保持位移及其限位。
    ],
  ),
)

调度运行在 0→2 N 线性增长切向载荷下闭环：峰值摩擦利用率 #fmt-percent(sm.peak_friction_utilization)，最小摩擦裕量 #fmt-n(sm.minimum_friction_margin_n) N，最大切向位移 9.4 μm，无宏滑移；力跟踪 RMSE #fmt-n(sm.force_tracking_rmse_n) N，远低于 0.5 N 阈值。全过程见图 2。

#place(
  top + center,
  scope: "parent",
  float: true,
  figure(
    image("/reports/figures/force_schedule.pdf", width: 100%),
    caption: [
      目标力调度运行全程产物图，自上而下：切向载荷（指令与附加下压力）；平均单侧目标力与实测法向力；摩擦裕量；切向滑移。
    ],
  ),
)

= 结论

微滑移探测能以约 8% 的保守代价可靠估计摩擦下界，并支撑无滑移的目标力调度闭环。结果仅针对硬质物体与线性加载场景；盲估计链路不读取摩擦真值。

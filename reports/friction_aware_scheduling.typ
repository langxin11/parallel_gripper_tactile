// 《摩擦感知目标力调度》实验报告。
// 数据源为两次 `pgt` 运行的产物目录（见下方常量）；所有数值在编译时
// 程序化读取，重跑实验后重新编译即可同步。本报告是时点性交付物，
// 定性结论与适用边界以 docs/force-scheduling.md、docs/friction-estimation.md 为准。
#import "./template.typ": *

#show: report.with(
  title: "摩擦感知目标力调度实验报告",
  subtitle: "微滑移探测、保守摩擦估计与调度闭环验证",
)

// ---- 数据源（换数据时只改这两行）----
#let friction-run = "/outputs/custom_parallel_gripper/friction-estimate/20260905T010111Z-f867ae69"
#let schedule-run = "/outputs/custom_parallel_gripper/force-schedule/20260905T010115Z-1ea30c66"

#let fm = json(friction-run + "/metrics.json")
#let sm = json(schedule-run + "/metrics.json")
#let ft = yaml(friction-run + "/task.yaml")
#let st = yaml(schedule-run + "/task.yaml")

= 摘要

本报告在自研平行夹爪仿真上验证「摩擦感知目标力调度」闭环：先以受控微滑移探测盲估计摩擦系数的保守下界 $hat(mu)_("LB")$，再把估计值代入目标力调度器，在持续增长的切向载荷下维持抓取。两次运行的数值结论：

+ 估计运行：$hat(mu)_("LB") = #fmt-float(fm.estimated_friction_coefficient)$，为仿真真值$mu = #fmt-float(fm.true_friction_coefficient)$ 的 #fmt-float(fm.estimate_ratio * 100, digits: 3)%，满足保守性判据且未触发回退；
+ 调度运行：切向载荷从 0 线性增长至
  #fmt-float(st.downward_load.waypoints.last().force_n) N 的过程中，峰值摩擦利用率
  #fmt-float(sm.peak_friction_utilization * 100)%，最小摩擦裕量
  #fmt-float(sm.minimum_friction_margin_n) N，全程无宏滑移（最大切向位移
  #fmt-float(sm.max_tangential_displacement_m * 1e6) $mu$ m），力跟踪 RMSE
  #fmt-float(sm.force_tracking_rmse_n) N。

以上数值均由本报告编译时从两次运行的 `metrics.json` 与任务快照 `task.yaml` 程序化读取，运行溯源信息见各结果小节开头。

= 方法

== 微滑移探测与保守摩擦估计

探测阶段以缓慢增速的夹持法向力加载物体，同时聚合两侧触觉 site 的法向与切向分量：第 $s$ 侧的法向合力 $N_s$ 取截断正值，切向合力 $T_s$ 取两轴分量的模长，全局摩擦比 $rho$ 为两侧切向之和与法向之和之比：

#formula("N_s=\\max(0,F_{z,s}),\\qquad T_s=\\sqrt{F_{x,s}^2+F_{y,s}^2},\\qquad \\rho=\\frac{T_L+T_R}{N_L+N_R}.")

当比值时间序列出现微滑移征兆（斜率先后越过 arming 与 saturation 阈值，且双侧比值差异在确认窗内持续）即判定临滑；取临滑前窗口内 $rho_k$ 样本的 $q$ 分位数，乘以折减系数 $eta$ 并截断到先验区间，得到保守下界：

#formula-box(
  formula(
    "\\hat\\mu_{LB}=\\operatorname{clip}\\!\\left(\\eta\\, Q_q\\{\\rho_k\\}_{\\mathrm{pre\\text{-}slip}},\\ \\mu_{min},\\ \\mu_{max}\\right),\\qquad 0<\\eta\\leq 1.",
  ),
)

本次任务取分位数 $q = #fmt-float(ft.estimator.estimate_quantile)$、折减系数$eta = #fmt-float(ft.estimator.safety_discount)$。

== 摩擦感知目标力调度

已知摩擦系数时，调度器按安全系数 $gamma$ 把切向需求 $D$ 换算为双侧目标法向力：

#formula("f_{ref}=\\operatorname{clip}\\!\\left(\\frac{\\gamma D}{2\\mu},\\ f_{min},\\ f_{max}\\right).")

摩擦感知闭环把上式中的真值 $mu$ 替换为探测得到的 $hat(mu)_("LB")$，使目标力在估计偏保守时自动留出裕量。相邻控制周期之间还受变化率约束（Typst 原生公式写法示例）：

$ |f_("ref,k") - f_("ref,k-1")| <= dot(f)_("max") Delta t $

= 实验设置

两次运行共用自研平行夹爪 profile 与 `hard` 材质方物，真值摩擦系数$mu = #fmt-float(ft.friction_coefficient)$、质量 #fmt-float(ft.cube_mass_kg) kg、控制周期
#fmt-float(ft.control_period_s * 1000) ms、NoSlip 迭代 #str(ft.solver.noslip_iterations) 次。

== 摩擦估计任务（`nominal_friction_probe`）

#let param-row(name, value, desc) = (name, value, desc)
#table(
  columns: (1.6fr, 1fr, 2.2fr),
  align: (left, right, left),
  inset: (x: 6pt, y: 4pt),
  stroke: 0.4pt + luma(170),
  table.header(header-cell([参数]), header-cell([取值]), header-cell([含义])),
  ..(
    param-row([探测法向力上限], [#fmt-float(ft.probe.max_force_n) N], [加载阶段法向力 ceiling]),
    param-row([法向力增速], [#fmt-float(ft.probe.force_rate_n_s) N/s], [探测段加载速率]),
    param-row([分位数 $q$], fmt-float(ft.estimator.estimate_quantile), [临滑窗口比值分位]),
    param-row([折减系数 $eta$], fmt-float(ft.estimator.safety_discount), [保守性折减]),
    param-row([估计窗口], [#str(ft.estimator.window_size) 样本], [临滑前统计窗口]),
    param-row([摩擦先验区间], [$[#fmt-float(ft.estimator.min_friction_coefficient), #fmt-float(ft.estimator.max_friction_coefficient)]$], [估计值截断范围]),
    param-row([回退摩擦系数], fmt-float(ft.estimator.fallback_friction_coefficient), [探测失败时使用]),
  ).flatten(),
)

== 目标力调度任务（`dynamic_filling`）

切向需求以线性插值沿加载曲线增长，模拟容器逐渐注水：

#table(
  columns: (1.6fr, 1fr, 2.2fr),
  align: (left, right, left),
  inset: (x: 6pt, y: 4pt),
  stroke: 0.4pt + luma(170),
  table.header(header-cell([参数]), header-cell([取值]), header-cell([含义])),
  ..(
    param-row([安全系数 $gamma$], fmt-float(st.scheduler.safety_factor), [目标力对摩擦需求的放大倍数]),
    param-row([法向力区间], [$[#fmt-float(st.scheduler.min_force_n), #fmt-float(st.scheduler.max_force_n)]$ N], [目标力截断范围]),
    param-row([目标力增速上限], [#fmt-float(st.scheduler.max_force_rate_n_s) N/s], [变化率约束]),
    param-row([摩擦下限 $mu_(min)$], fmt-float(st.scheduler.friction_floor), [调度用摩擦兜底值]),
    param-row([加载终点], [#fmt-float(st.downward_load.waypoints.last().force_n) N], [#fmt-float(st.downward_load.waypoints.last().t_s) s 时的附加切向载荷]),
  ).flatten(),
)

= 结果

== 摩擦估计运行

#run-header(friction-run, [friction-estimate])

#metrics-table(
  friction-run + "/metrics.json",
  entries: (
    ("slip_detected", "微滑移征兆被探测到"),
    ("probe_detection_time_s", "探测时刻（自接近段起算）"),
    ("probe_force_at_detection_n", "探测时刻的夹持法向力"),
    ("raw_friction_coefficient", "临滑窗口比值分位（未折减）"),
    ("estimated_friction_coefficient", "保守估计值 μ̂_LB"),
    ("true_friction_coefficient", "仿真真值 μ"),
    ("estimate_ratio", "估计值 / 真值"),
    ("conservatism_passed", "保守性判据（μ̂ 不高于真值加容差）"),
    ("informativeness_passed", "信息量判据（估计显著偏离回退值）"),
    ("using_fallback", "是否回退到默认摩擦系数"),
    ("hold_force_tracking_rmse_n", "保持段力跟踪 RMSE"),
    ("minimum_hold_friction_margin_n", "保持段最小摩擦裕量"),
  ),
  include-rest: true,
)

#figure(
  image(friction-run + "/plot.pdf", width: 86%),
  placement: top,
  caption: [摩擦估计运行全过程：探测加载、比值时间序列、微滑移征兆与保持段力跟踪。],
)

== 目标力调度运行

#run-header(schedule-run, [force-schedule])

#metrics-table(
  schedule-run + "/metrics.json",
  entries: (
    ("mean_target_force_n", "目标法向力均值"),
    ("peak_target_force_n", "目标法向力峰值"),
    ("final_target_force_n", "稳态目标法向力"),
    ("peak_friction_utilization", "峰值摩擦利用率"),
    ("minimum_friction_margin_n", "最小摩擦裕量"),
    ("force_tracking_rmse_n", "力跟踪 RMSE"),
    ("max_tangential_displacement_m", "最大切向位移（滑移判据）"),
    ("target_force_maximum_ratio", "目标力触及上限的时间占比"),
    ("target_force_rate_limited_ratio", "目标力受限速的时间占比"),
    ("slip_passed", "滑移判据"),
  ),
  include-rest: true,
)

#figure(
  image(schedule-run + "/plot.pdf", width: 86%),
  placement: top,
  caption: [目标力调度运行：切向需求增长、调度目标力与实测法向力、摩擦裕量与利用率。],
)

== 结论与适用边界

+ 保守估计成立：$hat(mu)_("LB")$ 低于真值约 #fmt-float((1 - fm.estimate_ratio) * 100)%，保守性、探测与信息量判据全部通过，未触发回退；
+ 闭环调度无滑移：峰值摩擦利用率 #fmt-float(sm.peak_friction_utilization * 100)% 小于 1，最小裕量为正，目标力全程既未触及上限也未受限速约束（两类占比均为
   #fmt-float(sm.target_force_maximum_ratio * 100)%）；
+ 跟踪品质良好：调度段力跟踪 RMSE #fmt-float(sm.force_tracking_rmse_n) N，远低于任务阈值
   #fmt-float(st.metrics.force_rmse_threshold_n) N。

适用边界：结论仅针对 `hard` 材质、线性增长切向载荷与本次探测任务参数成立；摩擦真值仅用于评估，估计与调度链路本身不读取真值。已知摩擦的 oracle 调度对照与参数敏感性见
`docs/force-scheduling.md`；微滑移判据的完整定义见 `docs/friction-estimation.md`。定量数值属时点性结果，重跑实验后以新产物为准。

= 附录：产物清单

两次运行目录内的产物文件（登记于各自 `manifest.json`）：

#let fa = json(friction-run + "/manifest.json").artifacts
#let sa = json(schedule-run + "/manifest.json").artifacts

#text(size: 9pt)[
  *friction-estimate*：#fa.join("，") \
  *force-schedule*：#sa.join("，")
]

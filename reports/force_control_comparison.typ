#import "@preview/mitex:0.2.7": *
#import "./template.typ": formula
#import "@local/wired-ieee:1.0.0": ieee

#show: ieee.with(
  title: [DM_Gripper 法向力控制：控制律、离散约束与验证边界],
  authors: (
    (name: "肖大亮", organization: "北京理工大学空天学院", email: "2658327508@qq.com"),
  ),
  abstract: [本文梳理 DM_Gripper 当前实现的法向力控制方法，包括位置式 PID、历史刚度位置前馈、刚度感知位置增量限幅、DM_Gripper 二阶导纳及二阶直接力矩 ADRC。重点给出统一运动学、离散控制律和实验比较边界。现有证据只支持实现状态与验证范围说明；新限幅参数尚未完成多任务、多随机种子整定。],
  index-terms: ("平行夹爪", "力控制", "导纳控制", "接触刚度", "ADRC"),
  paper-size: "a4",
  lang: "en",
)

= 目的与符号

本文整理当前项目已实现的法向力控制律，作为实验报告的简要方法说明。控制对象为一对手指的平均法向接触力：

#formula("f_n = (F_L + F_R) / 2, \\quad e_f = f_{\\mathrm{ref}} - f_n.")

其中，`f_ref` 为目标力；`F_L`、`F_R` 为两侧法向力。除特别说明外，下文的刚度均指两侧平均力相对闭合位移的等效刚度估计 `k_hat_pair`。本文说明控制结构与验证范围，不以尚未完成的配对实验替代性能结论。

= 共同运动学与力矩基线

以关节角 $q$ 表示开合位置，闭合位移 $c(q)$ 与其雅可比定义为：

#formula("c(q) = \\omega(0) - \\omega(q), \\qquad J_c(q) = c'(q).")

在一个控制周期内作局部线性化，有：

#formula("\\Delta f_n \\approx \\hat{k}_{\\mathrm{pair}} J_c \\Delta q.")

全部位置式控制器最终均进入 MIT 型关节力矩环：

#formula("\\begin{aligned} \\tau &= k_p(q_d-q) + k_d(\\dot q_d-\\dot q) + \\tau_{\\mathrm{ff}}, \\\\ \\tau_{\\mathrm{ff}} &= \\beta f_{\\mathrm{ref}} J_c. \\end{aligned}")

其中 `β` 为可辨识的力矩前馈系数。该项将期望接触力转换为近似负载力矩；位置环只产生小幅 `q_d` 修正。

= 位置式力跟踪

== 基线与历史刚度前馈

标准 PID 产生受行程限制的位置修正：

#formula("\\Delta q_{\\mathrm{PI}} = \\operatorname{sat}_{\\Delta q_{\\max}} \\left(K_P e_f + K_I \\int e_f\\,dt + K_D \\dot e_f\\right).")

历史变体 `pid-stiffness-ff` 在此基础上叠加模型辅助的位置前馈：

#formula("\\Delta q_{\\mathrm{cmd}} = \\operatorname{sat}_{\\Delta q_{\\max}} \\left(\\Delta q_{\\mathrm{PI}} + \\frac{\\alpha e_f}{\\hat{k}_{\\mathrm{pair}} J_c}\\right).")

它用于比较“刚度估计作为位置前馈”这一假设；其增量与误差同向，故并非纯粹的安全约束。

== 刚度感知限幅

新变体 `pid-stiffness-limit` 保留 PID 与力矩前馈，但不再将 `e_f / (k_hat_pair J_c)` 直接加到命令上。刚度估计只决定本周期允许的位移变化量：

#formula("k_{\\mathrm{safe},k} = \\gamma_k \\hat{k}_{\\mathrm{pair},k}, \\quad \\Delta f_{\\mathrm{lim},k} = \\min(\\lvert e_{f,k}\\rvert, \\dot f_{\\mathrm{lim}} \\Delta t).")
#formula("\\Delta q_{\\mathrm{lim},k} = \\frac{\\Delta f_{\\mathrm{lim},k}}{k_{\\mathrm{safe},k} J_{c,k}}.")
#formula("\\begin{aligned} \\Delta q_{\\mathrm{cmd},k} &= \\operatorname{clip}(\\Delta q_{\\mathrm{PI},k}, q_k^-, q_k^+), \\\\ q_k^{\\pm} &= \\Delta q_{\\mathrm{cmd},k-1} \\pm \\Delta q_{\\mathrm{lim},k}. \\end{aligned}")

`γ_k ≥ 1` 是刚度安全系数，`f_dot_lim` 是允许的力变化率。实现中将上式映射为 PID 的动态输出上下界，因此积分项也受到同一边界约束，避免在限幅期间持续累积。当前 profile 的初始值为 `f_dot_lim = 10 N/s`、`γ_k = 1`；它们是保守起点，而非已完成整定的最优参数。

运行记录提供 `stiffness_position_limit_rad`、`stiffness_position_limited` 及 `stiffness_position_limit_ratio`，用于回答限幅是否真正介入，以及介入频率是否合理。该变体目前仅完成运行链路与冒烟验证，尚未纳入默认研究矩阵；应在相同任务与随机种子下优先同 `pid-torque-ff` 配对比较。

= 二阶导纳基线

DMgripper 的共享控制链路采用闭合位移域二阶导纳：

#formula("M_a \\ddot c + B_a \\dot c + K_a c = e_f.")

对离散周期 $Delta t$，采用半隐式积分：

#formula("\\begin{aligned} \\dot c_{k+1} &= \\dot c_k + \\frac{\\Delta t}{M_a}(e_{f,k}-B_a \\dot c_k-K_a c_k), \\\\ c_{k+1} &= c_k + \\Delta t \\dot c_{k+1}. \\end{aligned}")

随后经逆运动学得到关节位置目标，并由位置伺服器执行。`M_a`、`B_a`、`K_a` 分别刻画等效惯性、阻尼与回中趋势。该控制律已具备仿真与 ROS 共用的接口，但当前只作为跨系统基线；还未进入本项目的正式控制器对比矩阵。

= 二阶直接力矩 ADRC

直接力矩路线采用力误差的二阶扩张状态模型：

#formula("\\begin{aligned} \\ddot f_n &= d + b_0 \\tau_{\\mathrm{res}}, \\\\ b_0 &= \\left(\\frac{J_{\\mathrm{eff}}}{\\hat{k}_{\\mathrm{pair}}J_c^2} + \\frac{B_{\\mathrm{eff}}}{\\hat{k}_{\\mathrm{pair}}J_c^2\\Delta t}\\right)^{-1}. \\end{aligned}")

其中 `d` 汇总未建模动力学、摩擦与接触变化。扩张状态观测器估计力、力导数与总扰动，虚拟加速度为：

#formula("v = \\ddot f_{\\mathrm{ref}} + k_p(f_{\\mathrm{ref}}-\\hat f_n) + k_d(\\dot f_{\\mathrm{ref}}-\\widehat{\\dot f_n}).")

残余力矩命令为：

#formula("\\tau_{\\mathrm{res}} = (v - \\hat d) / b_0, \\quad \\tau_{\\mathrm{cmd}} = \\tau_{\\mathrm{ff}} + \\tau_{\\mathrm{res}}.")

限幅后的实际残余力矩会回灌观测器，避免观测器把执行器饱和误判为外部扰动。`adrc-torque` 是当前正式研究矩阵中的主要直接力矩候选；其优势与局限必须按 Step、Sine、Staircase 等任务分别报告，不宜用单一瞬态指标概括。

#colbreak()

= 比较与报告边界

建议将结论分为“已纳入正式矩阵”和“待配对验证”两层：

#figure(
  kind: table,
  caption: [控制器的当前定位与报告边界],
  table(
    columns: (1.05fr, 0.8fr, 1.65fr),
    inset: (x: 3pt, y: 2.5pt),
    stroke: none,
    align: left,
    table.hline(stroke: 0.8pt),
    table.header([*方法*], [*定位*], [*报告边界*]),
    table.hline(stroke: 0.45pt),
    [PID+$tau$-FF], [位置基线], [正式矩阵；新限幅的配对对象。],
    [PID+$k$-FF], [历史消融], [检验刚度前馈，不等同于安全限幅。],
    [PID+$k$-limit], [待验证], [已有链路验证，尚无矩阵统计结论。],
    [ADRC-$tau$], [力矩候选], [正式矩阵；按任务分别比较。],
    [Admittance], [DMGripper 基线], [接口已对齐，未纳入正式矩阵。],
    table.hline(stroke: 0.8pt),
  ),
)

所有方法应使用相同 profile、任务、扰动与随机种子，至少报告 RMSE、MAE、峰值力、上升时间、超调量、调节时间、力矩饱和比例及接触丢失情况。对于刚度感知限幅，还应并列报告限幅触发比例和累计位移限幅量；否则无法判断性能变化来自控制律还是限幅几乎从未生效。

= 复现实验入口

单次轨迹由 `pgt run force-track` 生成，并通过 `--controller-variant` 选择控制器；正式结论应由研究脚本汇总多个随机种子后给出。完整实验口径、参数释义与默认研究矩阵见 `docs/force-tracking.md`、`docs/crank-slider-force-control.md` 和 `docs/control-comparison-ablation.md`。

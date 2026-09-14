# 控制算法对比与消融

本页说明 DMgripper 力控制研究的比较口径与证据边界。控制律见[动态目标力跟踪](force-tracking.md)，
模块职责见[架构](architecture.md)，执行入口与研究顺序见[工作流](workflows.md#formal-study-route)。

## 正式协议

**算法对比**比较完整控制结构；**消融**只改变同一结构中的被测模块。条件矩阵由
`configs/research/<研究名>/study.yaml` 和对应领域 protocol 唯一生成，计划与执行共享同一 `StudyPlan`。
文档不另外维护可执行矩阵；以计划产物确认实际条件，以 run 快照解释既有结果。

| 研究名 | 比较对象与控制变量 |
| --- | --- |
| `force_controller_ablation` | PID 的刚度位置修正与机构力矩前馈组成 2×2 消融；固定默认 waypoint，配对材料与 seed。 |
| `force_controller_selection` | 四个 PID 变体、`pid-stiffness-rate` 与 `adrc-torque`；固定 Step／Ramp／Mixed，配对材料与 seed。 |
| `stiffness_estimator_validation` | 固定 `pid-stiffness-ff`，只替换三种估计器，比较下游力跟踪。 |
| `torque_adrc_tuning` | 扫描测量滤波、控制带宽与观测带宽比，经连续任务约束后确认候选。 |

默认控制器对比排除 `direct-torque`、一阶位置式 `adrc` 和 `pid-stiffness-limit`，它们仍有独立复现入口。
退出原因分别是历史跨任务表现退化、控制导向模型与 MIT 位置闭环阶次不匹配、stiff Step 上出现平台极限环。
不要把历史 study 的成员或运行结果解释为当前矩阵已经执行。

正式接触 preset 为 `medium`、`hard`、`stiff`，它们描述显式 contact pair 的求解器参数，
不是材料弹性模量，也不是系统实测等效刚度。所有比较保持目标、接触参数、控制周期、噪声 seed 和评价时间窗一致，
只改变研究指定因素。执行异常、科学失败和有效结果分别登记，不能通过丢弃失败运行改善排名。

## 指标与解释

误差、饱和和阶跃指标的精确定义见[力跟踪指标](force-tracking.md#force-tracking-metrics)。比较时同时检查：

- RMSE／MAE、偏差、终端误差及阶跃超调／调节时间；Step 的最大瞬时误差常由目标跳变主导，不能单独排名。
- 力矩与位置饱和、力矩抖动、接触恢复和刚度估计有效性；更低 RMSE 伴随更高饱和不自动代表更优。
- 同材料、同 seed 的配对轨迹；PID 消融须四变体齐全才计算 2×2 主效应与交互作用。

Ramp 的卸载终点有 2 s 保持段，终端误差可解释为末端稳态误差。现有每组合三个 seed 的研究只支持工程趋势，
不构成统计显著性结论。汇总指标与图表必须保留所用配置、统计窗口和失败状态。

## 刚度估计器

估计量为平均单侧力相对总闭合行程的局部斜率 \(\hat k_{pair}\)，单位 N/m。默认使用 `window_linear`。

| 方法 | 估计方式 |
| --- | --- |
| `secant_ewma` | 有效参考点间 \(\Delta f/\Delta c\) 的正割线斜率，经限幅与 EWMA 平滑。 |
| `window_linear` | 窗口模型 \(F=a_0+a_1c\) 的斜率 \(a_1\)，再经 EWMA。 |
| `window_quadratic` | 窗口模型 \(F=a_0+a_1c+a_2c^2\) 在当前闭合量处的导数，再经 EWMA。 |

样本数、闭合跨度和力跨度必须满足门限；退化拟合或非正斜率保持上一次估计，输出受正刚度范围约束。
估计器没有显式分离接触阻尼、迟滞和各部件刚度，也没有仅凭滑移或 taxel 集合变化自动冻结的机制。

`stiffness_estimator_validation` 固定刚度位置修正、关闭机构力矩前馈，隔离估计器对控制的影响；
它没有独立刚度参考，不能回答“谁估计得最准”。精度研究应采用独立平衡工作点构造的参考刚度，见
[力控模型的辨识口径](crank-slider-force-control.md#stiffness-identification)。

## 已有证据的适用范围

详细数据与复现来源统一进入[科研报告](reports.md)，本页只保留影响当前选择的结论：

- 位置式 MIT 控制中，机构力矩前馈是稳定的改善来源；刚度加法修正的附加收益有限。
- 已完成研究中，`adrc-torque` 对 Ramp／Mixed 连续参考优于位置式 `full`，Step 误差与超调更高；
  不能据此宣布其对所有任务更优。
- `pid-stiffness-limit` 的平台极限环使其退出默认比较；`pid-stiffness-rate` 的局部调优结果仍需跨频率、
  跨材料确认，局部最优不能直接成为统一控制器结论。
- 固定 `pid-stiffness-ff` 的估计器对比未显示窗口法稳定、可推广的跟踪收益；曲线平滑和平均刚度差异
  都不能替代估计精度或控制收益证据。

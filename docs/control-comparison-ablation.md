# 模型验证、控制对比与消融

本页说明 DMgripper 力控制研究的比较口径与证据边界。控制律见[动态目标力跟踪](force-tracking.md)，
模块职责见[架构](architecture.md)，执行入口与研究顺序见[工作流](workflows.md#formal-study-route)。

## 正式协议

**算法对比**比较完整控制结构；**消融**只改变同一结构中的被测模块。条件矩阵由
`configs/research/<研究名>/study.yaml` 和对应领域 protocol 唯一生成，计划与执行共享同一 `StudyPlan`。
文档不另外维护可执行矩阵；以计划产物确认实际条件，以 run 快照解释既有结果。

| 研究名 | 比较对象与控制变量 |
| --- | --- |
| `force_controller_selection` | PID 基线 `pid-torque-ff`、`pid-only`、`pid-stiffness-rate` 与 `adrc-torque`；固定 Step／Ramp／Mixed，配对材料与 seed。 |
| `torque_adrc_tuning` | 扫描测量滤波、控制带宽与观测带宽比，经连续任务约束后确认候选。 |

默认控制器对比排除 `direct-torque`、一阶位置式 `adrc`、`pid-stiffness-limit`，以及随刚度位置前馈
退役的 `full` 与 `pid-stiffness-ff`；退出原因分别是历史跨任务表现退化、控制导向模型与 MIT 位置闭环
阶次不匹配、stiff Step 平台极限环，以及消融证据显示刚度加法修正附加收益接近于零。
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

已退役的 `stiffness_estimator_validation` 曾固定刚度位置修正、关闭机构力矩前馈，隔离估计器对控制的影响；
它没有独立刚度参考，不能回答“谁估计得最准”。精度研究应采用独立平衡工作点构造的参考刚度，见
[局部刚度辨识](#stiffness-identification)与[平衡工作点参考](#equilibrium-reference)。

控制中使用的刚度与力雅可比定义见[局部接触力模型](force-tracking.md#contact-force-model)，
机构位移与有效工况见[DM 共享控制核](dm-shared-control.md#dm-kinematics)。以下方法用于评价在线估计精度。

### 局部刚度辨识 {#stiffness-identification}

通过缓慢、小幅的试探压入，可拟合局部关系

\[
\Delta f_n\simeq J_f(q)\Delta q,
\]

进而得到 \(\hat J_f\)，或在已知 \(J_c(q)\) 后反算 \(\hat k_{\mathrm{pair}}\)。
这辨识的是“Pillar—物体—机构／接触链路”组合的整体等效 \(k_{\mathrm{pair}}\)，用于前馈、增益调度
和实验比较；它不表示材料弹性模量，也不用于在线反推物体参数。

!!! warning "接触参数不是材料刚度"

    MuJoCo 的响应以编译模型中的显式接触对参数为准，不能仅由 geom 参数推断混合结果，
    也不能把 `solref` 数值当作材料刚度。

### 平衡工作点参考 {#equilibrium-reference}

正式真值研究不把 `solref` 的数值直接当作刚度参考（N/m），而是在相同平均单侧力 \(f_n\) 与总闭合行程
\(c\) 语义下，对加载、卸载两个分支分别采集平衡工作点。内部点使用中心差分：

\[
k_{\mathrm{ref}}(c_i)\simeq
\frac{f_n^{\mathrm{eq}}(c_{i+1})-f_n^{\mathrm{eq}}(c_{i-1})}
{c_{i+1}-c_{i-1}}.
\]

实际闭合网格通常不等距，实现使用三点非均匀插值导数。采样窗口须同时满足闭合跨度与力跨度阈值；
在线估计器独立记录有效比例，未激励的初值不能作为合格估计。在线拟合不参与参考构造。
同一指令偏移的加载／卸载力差仅为路径依赖诊断，并非严格同一实际闭合位置的材料迟滞。
此参考仍是数值近似：正式选型前还需缩小扫描间距、物理步长并收紧求解器容差，检查排序是否稳定。

### 评价指标 {#stiffness-validation-metrics}

跨材料比较以对数 RMSE 为主指标：

\[
\operatorname{RMSE}_{\log k}=\sqrt{\frac{1}{N}\sum_i
\left(\log\hat{k}_i-\log k_{\mathrm{ref},i}\right)^2}.
\]

同时报告相对 RMSE、相对偏差、低估率、估计抖动，以及相邻平衡工作点的力增量预测误差
\(e_{\Delta f}=\Delta f_n-\hat{k}\Delta c\) 的 RMSE 和仅加载分支的正误差 95% 分位数。
这不是一个控制周期内的动态预测误差。力跟踪 RMSE 只用于
评价估计器对下游控制的影响，不能替代上述估计精度指标。

## 已有证据的适用范围

详细数据与复现来源统一进入[科研报告](reports.md)，本页只保留影响当前选择的结论：

- 位置式 MIT 控制中，机构力矩前馈是稳定的改善来源；刚度加法修正的附加收益有限
  （消融 36 条件中 `pid-stiffness-ff` 对 `pid-only` 的 RMSE 差约 2.5e-05 N）。据此刚度位置前馈
  已退役，PID 基线为 `pid-torque-ff`，消融与固定 `pid-stiffness-ff` 的估计器对比研究 concluded 退役。
- 已完成研究中，`adrc-torque` 对 Ramp／Mixed 连续参考优于位置式 `full`，Step 误差与超调更高；
  不能据此宣布其对所有任务更优。
- `pid-stiffness-limit` 的平台极限环使其退出默认比较；`pid-stiffness-rate` 的局部调优结果仍需在统一
  250 Hz 外环下跨材料确认，局部最优不能直接成为统一控制器结论。历史多频率结果只描述当时条件。
- 固定 `pid-stiffness-ff` 的估计器对比未显示窗口法稳定、可推广的跟踪收益；曲线平滑和平均刚度差异
  都不能替代估计精度或控制收益证据。

### 高载荷接触的定性结论 {#collision-geometry-conclusions}

模型文件、变体和生成过程见[DM 资产说明](grippers/dmgripper/index.md#model-selection)，几何布局见
[Pillar 触觉模型](grippers/dmgripper/index.md#tactile-model)。

在高目标力下，原非共面 mesh 与 `multiccd` 同时启用时会出现接触流形切换、活跃接触数跳变和明显更大的
跟踪误差。保留高度差但改用球体、使 mesh 共面，或关闭 `multiccd` 后，接触数与力跟踪都会明显稳定。

因此，当前模型中的高载荷振荡不是“非共面”“mesh”或“multiccd”任一单独因素的必然结果，
而是原非共面 mesh 与多接触点求解方式的交互。该结论描述当前 MuJoCo 模型与任务条件，
仍需通过实物接触试验验证。

| 用途 | 当前配置 |
| --- | --- |
| 常规实验 | 保留高度差球体，启用 `multiccd` |
| 历史复现 | 原 mesh，或在运行时关闭 `multiccd` |

旧模型诊断入口及专属实现已退役。历史诊断产物 `20260904T080635Z-f4cd4ddf` 与上述结论保留，
当前默认球体资产、生成工具和基础模型验证测试继续维护；共面 mesh 与共面球体资产不再保留。
需要完整复现旧诊断时，使用包含原入口及全部碰撞变体的
Git 提交 `ec94b01ff57f6dfea1846611f7c0e7aa75b0ba48`；原始模型修复依据见提交 `37cc30e`。
复现时还须采用对应历史产物记录的物理步长、配置和依赖版本，不将当前 1000 Hz 条件当作旧实验条件。

!!! warning "接触参数的解释边界"

    `solref` 等效接触参数不是独立硅胶形变模型或已完成的实机力学标定，不能据此推导传感器精度，
    也不能直接当作材料刚度。刚度参考的构造见[平衡工作点参考](#equilibrium-reference)。

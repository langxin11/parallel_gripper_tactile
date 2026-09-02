# 🧪 控制算法对比与消融实验规划

本文说明 DM gripper 后续控制算法验证应放在哪个项目中推进，以及如何组织 force tracking
对比实验和消融实验。

当前结论是：短期在 `parallel_gripper_tactile` 中完成 benchmark、viewer、指标与配置化实验；
长期将稳定后的控制器核心沉淀到 `tactile-contact-control`，使其成为不依赖 MuJoCo 资产的算法库。

## 1. 项目分工

| 项目 | 推荐角色 | 不建议承担的内容 |
| --- | --- | --- |
| `parallel_gripper_tactile` | 真实 MJCF、profile、触觉读取、force tracking 任务、viewer、CSV/plot/metrics 产物、消融配置 | 维护大量与仿真无关的通用控制算法抽象 |
| `tactile-contact-control` | PID、ADRC、自适应刚度、扰动观测器等纯控制器实现与单元测试 | 作为当前真实夹爪 MJCF benchmark 的主入口 |
| 硬件/ROS 适配层 | 传感器读取、电机协议、实时通信、安全限位 | 直接承载算法逻辑或实验指标统计 |

当前 `parallel_gripper_tactile` 已经具备真实夹爪模型、触觉读数、结构化运行产物和
`pgt run force-track`，因此更适合作为近期算法对比平台。`tactile-contact-control`
更适合在控制器接口稳定后接收算法核心。

## 2. 术语

“消融”这个词可以使用，但应和“算法对比”区分：

| 名称 | 含义 | 示例 |
| --- | --- | --- |
| 对比实验 | 比较不同控制算法的整体表现 | PID vs ADRC vs 自适应刚度控制 |
| 消融实验 | 在同一算法框架内关闭某个模块，观察该模块贡献 | 关闭力矩前馈、关闭刚度估计、关闭 approach 前馈 |

论文或报告中可以写作“控制算法对比与模块消融实验”。

## 3. 推荐推进路线

### 阶段一：在当前项目固定 benchmark

先保持实验环境统一，把真实 MJCF、触觉读数、目标力曲线和指标统计固定下来。
这一阶段重点不是引入很多算法，而是保证每次运行都能复现。

应优先完成：

1. 固定一组 `configs/force_tracking/*.yaml` 目标力曲线；
2. 固定每组实验的 profile 或 controller 配置快照；
3. 保留 `trace.csv`、`plot.png`、`metrics.json` 和 `manifest.json`；
4. 支持 `--viewer` 观察接触过程，默认仍 headless 批量运行；
5. 明确每个指标的统计时间窗，例如使用 `metrics.ignore_initial_s` 跳过初始过渡。

### 阶段二：稳定控制器接口

当 force tracking 任务稳定后，再把控制器抽象成统一接口：

```text
observation + reference + dt -> command
```

当前项目中这一层接口由 `ForceControlObservation`、`ForceControlReference` 和
`ForceTrackingController.step(...)` 表达。`NormalForceController` 是第一个实现；
后续 ADRC 控制器应实现同一个 `step(...)` 入口。

其中 observation 至少包含当前法向力、双侧法向力、接近目标位置和控制周期；
reference 至少包含目标法向力和接近阶段前馈力；command 至少包含目标位置修正、
MIT 前馈力矩、测量力、滤波力和诊断量。

这一层接口稳定后，PID、ADRC、自适应刚度控制器就可以在同一个仿真任务里互换。

当前第一阶段已经落地：`step.yaml`、`ramp.yaml`、`mixed_waypoints.yaml` 三类标准任务，以及
`controller × task × material × seed` 的显式 comparison schema、`--dry-run` 条件审阅、结构化聚合和
study 级对比图。现阶段矩阵仍只包含四个 PID 系变体；Direct torque 与 ADRC 属于下一阶段。

正式批量研究的接触 preset 已整体上移一档：使用 `medium=(-650,-8)`、`hard=(-1200,-10)` 和
`stiff=(-2500,-15)`。其中日常语义依次更接近 compliant、firm 与 stiff；这些参数是单个显式
contact pair 的求解器参数，不是物体弹性模量或整套系统的实测等效刚度。旧 `soft=(-250,-5)` 只为历史
配置和专项接触建立标定保留，不进入默认消融、控制器对比或诊断矩阵。

### 阶段三：把纯算法沉淀回 `tactile-contact-control`

当控制器接口不再频繁变化时，把与 MuJoCo、MJCF、profile 路径、viewer、plot 无关的算法代码迁回
`tactile-contact-control`。当前项目只保留适配层：

```text
MuJoCo + tactile reader -> controller observation
controller command -> DM/MIT actuator command
```

这样做的好处是：算法可以独立单元测试，仿真项目仍专注于真实模型验证。

### 阶段四：迁移到硬件闭环

硬件阶段不要重新写算法，只替换输入输出适配：

```text
真实触觉传感器 -> controller observation
controller command -> 达妙电机 CAN/串口命令
```

同时保留与仿真相同的 target schedule、日志字段和指标计算方式，便于 sim-to-real 对比。

## 4. 推荐实验矩阵

第一批实验应先做模块消融，确认当前控制结构里每个模块是否真的有贡献。

| 组别 | 配置变化 | 目的 |
| --- | --- | --- |
| Full | 默认配置 | 完整算法基线 |
| PID only | 关闭刚度估计与力矩前馈 | 最朴素反馈控制基线 |
| Direct torque | `MIT kp=0, kd=0`，力误差和模型前馈直接进入 `t_ff` | 对照位置式力控和直接力矩式力控 |
| No approach FF | `approach.feedforward_force_n: 0.0` | 评估低速闭合前馈对接触建立的影响 |
| No torque FF | `torque_feedforward_gain: 0.0` | 评估开度公式力矩前馈的贡献 |
| No stiffness position FF | `position_feedforward_gain: 0.0` | 评估刚度估计用于位置前馈的贡献 |
| No stiffness estimator | `stiffness.enabled: false` | 评估在线刚度估计整体贡献 |

第二批实验再做算法对比：

| 组别 | 控制器 | 对比重点 |
| --- | --- | --- |
| PID | 固定参数 PID | 稳态误差、超调、抗噪性 |
| PID + feedforward | PID 加机构力矩前馈 | 跟踪误差和力矩饱和变化 |
| Adaptive stiffness | PID 加在线刚度估计 | 不同物体刚度下的泛化能力 |
| Direct torque force | `t_ff = τ_force_feedback + τ_model_feedforward` | 不经过位置刚度的力矩式力控 |
| ADRC | 扩张状态观测器控制 | 扰动、模型误差和延迟下的鲁棒性 |

所有组别应使用相同目标力曲线、相同物体、相同接触参数和相同噪声种子。只改变待评估的控制模块。

## 5. 目标力曲线设计

建议至少保留三类 waypoint schedule：

| 曲线 | 用途 |
| --- | --- |
| `step.yaml` | 测试阶跃响应、超调、稳定时间和稳态误差 |
| `ramp.yaml` | 测试平滑加载/卸载能力和迟滞 |
| `mixed_waypoints.yaml` | 综合测试平台段、斜坡段和卸载段 |

设计原则：

1. 目标峰值先保守，确认无明显饱和后再提高；
2. 每个平台段保留 1 到 2 秒，便于统计稳态误差；
3. 斜率逐步提高，避免一开始就让执行器限幅主导结果；
4. 如果主要评估控制器动态响应，使用 `hold` 阶跃；
5. 如果主要评估连续跟踪，使用 `smoothstep`。

`ramp.yaml` 自 2026-09-02 起在匀速卸载终点后附加 2 s 终端保持段，使 `final_error_n` 可以作为
终端稳态误差解读，而不是单纯的动态滞后。

## 6. 指标

每次实验至少比较以下指标：

| 指标 | 用途 |
| --- | --- |
| `rmse_n` | 整体跟踪质量 |
| `mae_n` | 平均绝对误差 |
| `peak_abs_error_n` | 最坏瞬时偏差 |
| `mean_error_n` | 是否存在系统性偏差 |
| `final_error_n` | 结束时误差 |
| `torque_saturation_ratio` | 力矩限幅是否主导结果 |
| `position_saturation_ratio` | 位置修正是否触达限幅 |
| `contact_time_s` | 接触建立速度 |
| `mean_estimated_stiffness_n_per_m` | 等效接触刚度估计是否合理 |
| `rise_time_s` | 阶跃加载的上升时间（90% 阶跃幅值） |
| `overshoot_ratio` | 阶跃超调量与阶跃幅值之比 |
| `settling_time_s` | 进入并保持 ±5% 稳定带的耗时 |

瞬态三项仅在 `hold` 任务存在合格加载阶跃时计算，其余任务输出空值；定义与空值语义见
[动态目标力跟踪](force-tracking.md)的指标表。

如果某个算法 RMSE 更低但饱和比例明显更高，不能直接认为它更好。应同时检查 `trace.csv`
中的力矩、位置修正、接触状态和刚度估计曲线。

对于直接力矩式力控，还应额外关注力矩抖动、力矩变化率、积分项是否 windup，以及脱离接触时是否仍有
持续闭合力矩。该模式建议先在仿真中作为对照组使用，不应直接跳到硬件。

### 6.1 2026-09-01 控制器对比结果

完整 study 位于
[`outputs/studies/force_tracking_controller_comparison/20260901T125358Z-0a777c0f`](../outputs/studies/force_tracking_controller_comparison/20260901T125358Z-0a777c0f/aggregate.csv)。
本次矩阵包含 4 个控制器变体、3 类目标力任务、3 个正式接触 preset 和 3 个噪声 seed，共 108 次运行。
所有运行均成功完成，且力矩与位置饱和比例均为 0，因此当前差异没有被执行器限幅主导。
本次历史 benchmark 使用 `secant_ewma`；study 配置已显式锁定该方法，避免默认估计器更新后改变复现实验。

跨 `medium`、`hard`、`stiff` 三种 preset 对 RMSE 取平均后，相对 `pid-only` 的降幅为：

| 控制器 | Step | Ramp | Mixed |
| --- | ---: | ---: | ---: |
| `pid-torque-ff` | 11.8% | 22.1% | 22.6% |
| `pid-stiffness-ff` | 3.2% | 0.1% | 0.5% |
| `full` | **14.2%** | **22.2%** | **23.1%** |

`full` 在 9 个 task × preset 组合中有 8 个取得最低 RMSE。主要性能增益来自基于机构雅可比的力矩前馈；
刚度位置前馈在 Step 中提供少量额外改善，而在 Ramp 和 Mixed 中未显示明显额外收益。当前只有 3 个 seed，
且 Ramp 中 `full` 与 `pid-torque-ff` 的差异很小，因此不应把这一结果表述为统计显著性结论。

接触时间只由 preset 和噪声 seed 决定；按三个唯一 seed 统计为：

| Preset | 接触时间 |
| --- | ---: |
| `medium` | 1.207 ± 0.025 s |
| `hard` | 1.092 ± 0.006 s |
| `stiff` | 1.058 ± 0.002 s |

更高的接触 preset 在当前相同几何条件下更早达到接触阈值，且 RMSE 整体下降。例如 `full` 的 Step RMSE
由 `medium` 的 0.405 N 降至 `stiff` 的 0.372 N，Ramp 由 0.071 N 降至 0.061 N。这只能解释为
当前显式接触条件下的系统响应趋势，不能推广为真实材料本体越硬，控制效果必然越好。

`full` 在 Ramp 中的平均刚度估计随 preset 呈单调上升：`medium`、`hard`、`stiff` 分别约为
2929、3130、3241 N/m。估计器能够区分相对刚柔趋势，但输出是夹爪—Pillar—物体—接触求解器共同形成的
局部等效刚度，不能解释为材料弹性模量，也不能与单个 contact pair 的 `solref` 数值直接对应。

当前结果还有两个指标边界：

1. Step 的 `peak_abs_error_n` 对所有控制器都约为 5 N，主要来自目标由 1 N 瞬间跳到 6 N 时的初始误差，
   不适合用于控制器排名。2026-09-02 起指标已增加 `rise_time_s`、`overshoot_ratio` 与
   `settling_time_s`，后续控制器对比应同时报告这三项；本节 108-run 基准早于该指标，不含瞬态统计。
2. Ramp 结束时的误差约为 -0.15 至 -0.20 N，该值更接近动态滞后，不能视为严格稳态误差。2026-09-02 起
   `ramp.yaml` 已附加 2 s 终端保持段，复跑后 `final_error_n` 可按终端稳态误差解读；本节基准使用旧版
   6 s 曲线。

### 6.2 历史割线估计器的原理与定位

上述 108-run benchmark 使用的 `secant_ewma` 是轻量级局部割线估计器，不属于先进的概率状态估计或系统
辨识算法。它继续作为计算量小、容易解释的历史工程基线；当前 profile 默认方法已经切换为
`window_linear`。详细力学关系也见[曲柄滑块力控模型](crank-slider-force-control.md)。

对每个控制周期，先用曲柄滑块运动学把电机位置 (q) 转换为总闭合行程 (c(q))，再计算自上一个有效参考点
以来的增量：

\[
\Delta c=c(q_k)-c(q_{k-1}),\qquad
\Delta f=f_{n,k}-f_{n,k-1}.
\]

只有当 \(|\Delta c|\) 和 \(|\Delta f|\) 均超过配置门限，且二者同号时，才构造局部割线样本：

\[
k_{\mathrm{sample}}=
\operatorname{clip}\left(
\left|\frac{\Delta f}{\Delta c}\right|,
k_{\min},k_{\max}
\right).
\]

随后用指数加权移动平均更新估计：

\[
\hat k_k=\hat k_{k-1}+\alpha(k_{\mathrm{sample}}-\hat k_{k-1}).
\]

三种方法共用 `initial=3000 N/m`、`min=250 N/m`、`max=25000 N/m`、`alpha=0.15`、
`min_delta_closure=0.05 mm` 和 `min_delta_force=0.025 N`。估计器在双侧接触确认时以当前位置和滤波力重置；
控制器随后把它换算为 \(\hat J_f=\hat kJ_c(q)\)，并生成受限的位置前馈：

\[
\Delta q_{\mathrm{stiff}}=
\gamma\frac{f_{\mathrm{ref}}-f_n}{\hat kJ_c(q)}.
\]

割线实现的优点是每步只需常数时间和常数内存，带有增量门限、符号检查、上下限与 EWMA，适合实时控制和
历史 benchmark。它的主要局限是：

- 仅使用两个参考点之间的割线，没有利用一段时间窗内的全部样本；
- 不估计置信度或噪声协方差，也没有遗忘因子的正规最小二乘模型；
- 没有显式辨识接触阻尼、迟滞、粘弹性或非线性刚度；
- 无法分离 Pillar、物体、机构和 MuJoCo 接触参数各自的贡献；
- 当前控制器没有依据活跃 taxel 集合变化、滑移等事件专门冻结估计，主要依赖接触状态、门限和符号检查。

因此论文中宜将该基线称为“在线局部割线刚度估计（EWMA-filtered secant estimate）”，而不宜笼统称为先进自适应
辨识。若后续希望提高算法层级，可依次比较滑动窗鲁棒回归、带遗忘因子的递推最小二乘（RLS）、联合估计
刚度与阻尼的 EKF/UKF，以及显式处理接触模式切换的多模型估计器。已有研究中，RLS 可结合残差模型在线拟合
非线性刚度，[Flacco 等](https://doi.org/10.1177/0278364912461813)；也有工作使用双候选力观测器在缺少可靠
接触位置时估计环境刚度，[Online stiffness estimation for robotic tasks with force observers](https://doi.org/10.1016/j.conengprac.2013.11.002)。这些方法模型更完整，但辨识条件、调参与验证成本也更高。

### 6.3 滑动窗刚度估计器对比

在不改变 PID 参数、目标力任务或接触条件的前提下，当前新增独立的刚度估计器对比 protocol。该 protocol 固定
`pid-stiffness-ff`：保留刚度位置前馈、关闭机构力矩前馈，因此结果只比较估计器如何影响位置前馈，而不把力矩前馈
的收益混入结论。比较方法为：

| 方法 | 局部模型 | 当前刚度 |
| --- | --- | --- |
| `secant_ewma` | 相邻有效点的割线 | 割线斜率经 EWMA 平滑 |
| `window_linear` | 最近窗口的 \(F=a_0+a_1c\) | \(a_1\) 经 EWMA 平滑 |
| `window_quadratic` | 最近窗口的 \(F=a_0+a_1c+a_2c^2\) | 当前闭合量处的 \(a_1+2a_2c\)，再经 EWMA 平滑 |

默认 profile 使用 `window_linear`，它利用整段窗口的样本但仍保持线性、易解释和较低计算量；`secant_ewma`
保留为历史工程基线，`window_quadratic` 用于验证是否确实存在对控制有益的局部非线性。窗口估计只在样本数量、
闭合行程跨度与力变化均足够时更新；拟合退化或给出非正刚度时保持上一次估计。所有输出仍裁剪到既有的安全刚度范围。

正式矩阵固定为 3 个估计器 × 3 个目标任务 × 3 个接触 preset × 3 个噪声 seed，共 81 次运行。主指标仍为
RMSE、MAE、最终误差与力矩/位置饱和率；同时从 trace 检查刚度曲线的抖动、是否频繁触及上下限，以及相同 seed
下的力—刚度叠加图。窗口法不能仅因估计曲线更平滑而判优，只有在不增加饱和或显著动态滞后的条件下改善跟踪误差，
才可认为其对控制有效。

#### 6.3.1 2026-09-01 刚度估计器对比结果

完整 study 位于
[`outputs/studies/force_tracking_stiffness_estimator_comparison/20260901T153708Z-7a17f423`](../outputs/studies/force_tracking_stiffness_estimator_comparison/20260901T153708Z-7a17f423/aggregate.csv)。
本次矩阵包含 3 个估计器、3 类目标力任务、3 个正式接触 preset 和 3 个噪声 seed，共 81 次运行；控制器固定为
`pid-stiffness-ff`，即保留刚度位置前馈、关闭机构力矩前馈，因此以下差异只反映估计器对位置前馈的影响。
所有运行均成功完成，且力矩与位置饱和比例在全部 27 个组合中均为 0，当前差异没有被执行器限幅或估计器引入的
饱和主导。

跨 `medium`、`hard`、`stiff` 三种 preset 对 RMSE 取平均后，以 `secant_ewma` 为基准的数值与相对变化为：

| 估计器 | Step | Ramp | Mixed |
| --- | ---: | ---: | ---: |
| `secant_ewma`（基准） | 0.440 N | 0.085 N | 0.111 N |
| `window_linear` | 0.442 N（+0.4%） | 0.086 N（+1.0%） | 0.111 N（+0.1%） |
| `window_quadratic` | 0.441 N（+0.3%） | 0.085 N（+0.1%） | 0.110 N（-0.1%） |

两种窗口法在 Step 与 Ramp 中均未低于基准：`window_linear` 分别高 0.4% 与 1.0%（Ramp 差距主要来自
`medium`，`stiff` 下与基准持平），`window_quadratic` 分别高 0.3% 与 0.1%；Mixed 中三者相差只有 ±0.1%，
差异很小，不应排序，`window_quadratic` 名义上的 -0.1% 也不应视为改善。按 6.3 protocol 的判据，窗口法只有
在不增加饱和或显著动态滞后的条件下改善跟踪误差，才可认为对控制有效；本次没有任何估计器相对基准改善跟踪
误差，因此在当前 `pid-stiffness-ff` 条件下，两种窗口法均未显示出对控制的有效收益，更不能仅凭估计曲线更
平滑而判优。当前只有 3 个 seed，上述差异的绝对值不超过 0.002 N，不应表述为统计显著性结论。

Ramp 卸载末端的 `final_error_n` 不因估计器而异：同一 preset 下三个估计器的差别不超过 0.003 N，`medium`、
`hard`、`stiff` 下分别约为 -0.19、-0.20、-0.18 N。与 6.1 的边界一致，该基准的 ramp 卸载到 1 N 后没有
终端保持段，此值更接近动态滞后，不能视为严格稳态误差（`ramp.yaml` 已于 2026-09-02 附加终端保持段）。

Ramp 任务的平均估计刚度（N/m）随 preset 的变化为：

| 估计器 | `medium` | `hard` | `stiff` |
| --- | ---: | ---: | ---: |
| `secant_ewma` | 2930 | 3117 | 3226 |
| `window_linear` | 1348 | 1268 | 2757 |
| `window_quadratic` | 1630 | 1429 | 4935 |

`secant_ewma` 随 preset 单调上升，数值与 6.1 在 `full` 控制器下报出的历史结果（约 2929、3130、3241 N/m）
几乎一致；两种窗口法的平均估计刚度则与基准相差明显且非单调：`window_linear` 整体更低（1268 至 2757 N/m），
`window_quadratic` 在 `medium`、`hard` 下更低，但在 `stiff` 下升至 4935 N/m，约为基准的 1.5 倍。估计刚度
的这些差异并未转化为跟踪差异（RMSE 相对变化不超过 1.0%），说明当前配置下位置前馈对估计刚度偏差并不敏感，
平均估计刚度的绝对值差异本身不能作为估计器优劣的证据。估计刚度仍是夹爪—Pillar—物体—接触求解器共同形成的
局部等效刚度，不能解释为材料弹性模量；上述数值只描述当前显式接触条件与 `pid-stiffness-ff` 配置下的趋势，
不能推广到真实材料属性或其他控制器组合。

最后一个指标边界：Step 的 `peak_abs_error_n` 对三个估计器均约为 5.0 N，由目标力从 1 N 跳到 6 N 的初始
误差主导，与估计器选择基本无关，不适合用于估计器排名。

## 7. 配置组织建议

当前项目按如下方式组织：

```text
configs/
├── force_tracking/
│   ├── default_waypoints.yaml
│   ├── step.yaml
│   ├── ramp.yaml
│   └── mixed_waypoints.yaml
└── studies/
    ├── force_tracking_ablation.yaml
    ├── force_tracking_controller_comparison.yaml
    └── force_tracking_stiffness_estimator_comparison.yaml
```

控制器变体通过运行时的不可变 profile 副本实现，不复制完整 profile。study schema 只展开条件矩阵，
`scripts/experiments` 直接调用 Python runner；控制器、仿真循环和单次运行产物仍分别由 `control.py`、
`experiments/force_tracking.py` 与 `runners/force_tracking.py` 管理。

## 8. 推荐命令形式

单次带 viewer 检查：

<pre><code class="language-bash">
uv run pgt run force-track \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_tracking/default_waypoints.yaml \
  --viewer
</code></pre>

批量对比先执行 dry-run，校验 profile、三类 task 和完整条件矩阵，不创建输出目录：

<pre><code class="language-bash">
uv run python scripts/experiments/force_tracking_controller_comparison.py \
  --config configs/studies/force_tracking_controller_comparison.yaml \
  --dry-run
</code></pre>

确认矩阵后以 headless 方式执行完整 study：

<pre><code class="language-bash">
uv run python scripts/experiments/force_tracking_controller_comparison.py \
  --config configs/studies/force_tracking_controller_comparison.yaml
</code></pre>

默认配置展开 4 个 PID 系变体 × 3 个 task × 3 个正式接触 preset × 3 个 seed，共 108 个条件。每个条件保留独立
run，study 父目录生成 `summary.csv`、`aggregate.csv`、`summary.json`、对比图和
`study_manifest.json`。脚本顺序调用 runner，不通过 CLI 子进程启动单次实验。

刚度估计器对比同样先 dry-run；它固定 `pid-stiffness-ff`，默认展开 81 个条件：

<pre><code class="language-bash">
uv run python scripts/experiments/force_tracking_stiffness_estimator_comparison.py \
  --config configs/studies/force_tracking_stiffness_estimator_comparison.yaml \
  --dry-run
uv run python scripts/experiments/force_tracking_stiffness_estimator_comparison.py \
  --config configs/studies/force_tracking_stiffness_estimator_comparison.yaml
</code></pre>

## 9. 决策原则

只要问题和真实 MJCF、触觉读数、viewer、run artifacts 或 force tracking 指标有关，就放在
`parallel_gripper_tactile` 中做。

只要问题可以脱离 MuJoCo，只依赖观测、参考目标和控制输出，就应逐步迁移到
`tactile-contact-control` 中做。

这样可以避免两个项目职责混在一起：当前项目负责“实验台是否可信”，旧项目负责“控制算法是否干净可复用”。

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

如果某个算法 RMSE 更低但饱和比例明显更高，不能直接认为它更好。应同时检查 `trace.csv`
中的力矩、位置修正、接触状态和刚度估计曲线。

对于直接力矩式力控，还应额外关注力矩抖动、力矩变化率、积分项是否 windup，以及脱离接触时是否仍有
持续闭合力矩。该模式建议先在仿真中作为对照组使用，不应直接跳到硬件。

### 6.1 2026-09-01 控制器对比结果

完整 study 位于
[`outputs/studies/force_tracking_controller_comparison/20260901T125358Z-0a777c0f`](../outputs/studies/force_tracking_controller_comparison/20260901T125358Z-0a777c0f/aggregate.csv)。
本次矩阵包含 4 个控制器变体、3 类目标力任务、3 个正式接触 preset 和 3 个噪声 seed，共 108 次运行。
所有运行均成功完成，且力矩与位置饱和比例均为 0，因此当前差异没有被执行器限幅主导。

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
   不适合用于控制器排名；后续应增加上升时间、超调量和 ±5% 稳定时间。
2. Ramp 结束时的误差约为 -0.15 至 -0.20 N，但当前卸载到 1 N 后没有终端保持段；该值更接近动态滞后，
   不能视为严格稳态误差。后续应增加 1 至 2 s terminal hold。

### 6.2 当前刚度估计器的原理与定位

当前 `ContactStiffnessEstimator` 是轻量级局部割线估计器，不属于先进的概率状态估计或系统辨识算法。
它适合作为计算量小、容易解释的工程基线，其详细力学关系也见[曲柄滑块力控模型](crank-slider-force-control.md)。

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

当前 profile 使用 `initial=3000 N/m`、`min=250 N/m`、`max=25000 N/m`、`alpha=0.15`、
`min_delta_closure=0.05 mm` 和 `min_delta_force=0.025 N`。估计器在双侧接触确认时以当前位置和滤波力重置；
控制器随后把它换算为 \(\hat J_f=\hat kJ_c(q)\)，并生成受限的位置前馈：

\[
\Delta q_{\mathrm{stiff}}=
\gamma\frac{f_{\mathrm{ref}}-f_n}{\hat kJ_c(q)}.
\]

该实现的优点是每步只需常数时间和常数内存，带有增量门限、符号检查、上下限与 EWMA，适合实时控制和
当前可复现 benchmark。它的主要局限是：

- 仅使用两个参考点之间的割线，没有利用一段时间窗内的全部样本；
- 不估计置信度或噪声协方差，也没有遗忘因子的正规最小二乘模型；
- 没有显式辨识接触阻尼、迟滞、粘弹性或非线性刚度；
- 无法分离 Pillar、物体、机构和 MuJoCo 接触参数各自的贡献；
- 当前控制器没有依据活跃 taxel 集合变化、滑移等事件专门冻结估计，主要依赖接触状态、门限和符号检查。

因此论文中宜称为“在线局部割线刚度估计（EWMA-filtered secant estimate）”，而不宜笼统称为先进自适应
辨识。若后续希望提高算法层级，可依次比较滑动窗鲁棒回归、带遗忘因子的递推最小二乘（RLS）、联合估计
刚度与阻尼的 EKF/UKF，以及显式处理接触模式切换的多模型估计器。已有研究中，RLS 可结合残差模型在线拟合
非线性刚度，[Flacco 等](https://doi.org/10.1177/0278364912461813)；也有工作使用双候选力观测器在缺少可靠
接触位置时估计环境刚度，[Online stiffness estimation for robotic tasks with force observers](https://doi.org/10.1016/j.conengprac.2013.11.002)。这些方法模型更完整，但辨识条件、调参与验证成本也更高。

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
    └── force_tracking_controller_comparison.yaml
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

## 9. 决策原则

只要问题和真实 MJCF、触觉读数、viewer、run artifacts 或 force tracking 指标有关，就放在
`parallel_gripper_tactile` 中做。

只要问题可以脱离 MuJoCo，只依赖观测、参考目标和控制输出，就应逐步迁移到
`tactile-contact-control` 中做。

这样可以避免两个项目职责混在一起：当前项目负责“实验台是否可信”，旧项目负责“控制算法是否干净可复用”。

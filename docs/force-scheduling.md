# 抓取目标力调度：Oracle 与统一自适应

[自适应抓取](adaptive-grasping.md)的已知摩擦参考基线：`force-schedule` 使用场景真值摩擦系数和
切向载荷需求生成平均单侧目标力，再由法向力控制器跟踪。它不估计摩擦，也不自动构成性能上界。

## 运行方法

```bash
uv run pgt run force-schedule --experiment dm_gripper/force_scheduling_gravity_hold
uv run pgt run force-schedule --experiment dm_gripper/force_scheduling_dynamic_filling
```

先建立双侧接触、稳定并撤去支撑，再调度与评价。`gravity_hold` 只抵抗重力；`dynamic_filling`
在物体质心沿重力方向施加 0→2 N 附加力，不改变物体质量。

## 调度公式与力语义

设切向合力需求为 `D`，摩擦系数为 `μ`，安全系数为 `γ`：

\[
f_{ref}=\operatorname{clip}\!\left(\frac{\gamma D}{2\max(\mu,\mu_{floor})},f_{min},f_{max}\right),
\qquad |f_{ref,k}-f_{ref,k-1}|\leq\dot f_{max}\Delta t.
\]

`f_ref` 是平均单侧法向力，双侧理想摩擦容量为 `2μf_n`。`friction_floor` 防止分母退化；
目标受幅值与对称变化率约束。任务在 `configs/task/force_scheduling/` 中定义物体、载荷、调度与验收参数。

标准场景采用 50 g 方块、`μ=0.8`、`γ=1.5`、力范围 `0.5～8 N/侧`、变化率 `1 N/s`。
显式 `solver.noslip_iterations=5` 抑制摩擦锥内数值爬移，不增加摩擦容量；动态载荷下将目标上限
限制为 `0.5 N/侧` 仍会滑落。

## 输出与指标

独占目录 `outputs/dm_gripper/force-schedule/<run>/` 保存输入快照、`effective_parameters.json`、
`trace.csv`、`metrics.json`、`plot.png`、矢量 `plot.pdf` 与 manifest；有效参数声明 `scheduler_kind=oracle`。

trace 记录切向需求、真值摩擦、原始／受限目标、触觉力、摩擦裕量与位移。验收同时检查仿真稳定、
力跟踪 RMSE 和撤支撑后最大切向位移；目标摘要、摩擦利用率与限幅占比用于解释失败。

主图统一为三组曲线，共用撤支撑后的时间轴 `t`（s）：

| 面板 | 数学符号与曲线 | 数据含义 |
| --- | --- | --- |
| 抓力跟踪 | `F_d`、`F_n`（N/侧） | 调度目标、控制器使用的滤波平均单侧触觉法向力。 |
| 切向承载 | `T`、`Dᵍᵗ`、`Cᵍᵗ`（N） | 双侧各自先求切向合力再取模之和；真值载荷需求；真值摩擦容量。 |
| 独立位移评价 | `d_tᵍᵗ`、`d_lim`（mm） | 物体相对撤支撑位置的切向位移模长、验收阈值。 |

图中 `gt` 上标统一表示仿真真值，只供评价，不是传感器读数。失接触后 `T` 与 `Cᵍᵗ`
趋于零，但 `Dᵍᵗ` 仍等于未消失的载荷。原 `friction_margin_n=Cᵍᵗ−Dᵍᵗ` 保留在轨迹中，
不再作为主图独立曲线，避免把负余量误读为实测力。控制观测保持原滤波口径，不额外平滑。
分侧实测切向量新增为 `measured_left_tangential_n`／`measured_right_tangential_n`；
旧轨迹缺少两列时省略 `T`，不从真值计算观测。目标增长率、风险和摩擦候选仍留在诊断轨迹中，
默认关闭的通路不占主图面板。

## 标准场景参考结果

在默认 DMgripper profile、50 g 方块、`hard` 接触和 `μ=0.8` 下，已有标准任务记录为：

| 任务 | 力跟踪 RMSE | 最大切向位移 | 最终目标力 |
| --- | ---: | ---: | ---: |
| 仅重力保持 | 约 `0.009 N` | 约 `0.009 mm` | `0.500 N/侧` |
| 动态注水 | 约 `0.030 N` | 约 `0.009 mm` | 约 `2.335 N/侧` |

这些数值验证了“已知 `μ` 时的负载—目标力—力控”闭环基线，不代表未知材料下的摩擦估计性能。
调度器的纯算法位于 `src/parallel_gripper_tactile/force_scheduling.py`，场景、任务 schema、指标和绘图
位于 `src/parallel_gripper_tactile/experiments/force_scheduling.py`。

## 固定摩擦先验的初步仿真 {: #adaptive-prior }

```bash
uv run pgt run force-schedule --experiment dm_gripper/adaptive_prior
```

该组合复用载荷场景、记录与评价，使用现有 `admittance_unified` 导纳与固定 MIT 增益，
控制周期为 4 ms。它验证上层调度，不代表方案所列真机底层参数已经标定。
`task.adaptive_prior` 非空时启用新策略；省略时保持 Oracle 行为。
左右摩擦先验均为 0.6，场景真实摩擦为 0.8，两者独立配置。

每侧触觉先求切向合力再取模，按实际观测间隔作 50 ms 一阶低通。承载需求为
`safety_factor * max(T_left / mu_left, T_right / mu_right)`，不扣预载基线。
目标只增不减，增长率由剩余目标缺口和滤波切向载荷的正增长率决定，最大为 1 N/s。
首样本直接播种承载量，趋势导数为零；目标上限、下限、安全系数沿用 `scheduler` 配置，
`friction_floor` 仅供 Oracle，先验模式要求两侧摩擦严格为正。

新增 trace 列保留分侧实测切向量、滤波总切向量、载荷趋势、受限需求、调度后剩余缺口与
`capacity_limited`。`raw_target_force_n` 保存未作上下限裁剪的需求；有效配置的
`scheduler_kind=adaptive_prior` 明确在线依据。真值载荷、摩擦裕量与物体位移只供施加载荷和评分。
先验模式的最大位移统计包含撤支撑后的整个阶段；力跟踪 RMSE 仍沿用原忽略窗口。
能力需求一旦超上限，指标 `capacity_limited=true`，总验收失败，裁剪不会被视为需求满足。

初步回归用同一参数覆盖恒定重力、四秒增加 2 N、物体自重 2.5 N 时撤支撑，并复用抓力不足负例。
固定先验基线保留 0.5 N 初始力与 1 N/s 限速，统一撤支撑组合使用下述快速响应候选；
二者仍保留位移未达标的失败边界回归，不放宽验收；低估摩擦或物体已加速滑落时，
接触切向力仍可能低估所需承载。该先验组合本身不开放局部风险或摩擦更新。

## 统一策略的初步验证 {: #unified-adaptive }

多速率仿真支持可选 `task.definition.tactile_fault` 注入，默认 `null`，不影响旧组合。
`start_s` 为撤支撑后的注入起点（默认 0.02 s）；`shear_noise_std_n` 是两侧逐 taxel 切向分量
附加高斯噪声标准差，使用独立随机流；`spike_n`／`spike_frames` 在两侧第一个 taxel 的 Fx
施加一帧或两帧加性毛刺。注入仅修改观测，不向物体施加真实外力。
`drop_frames` 丢弃起点后的指定数量采样，保留序号缺口，不更新 latest；
`jitter=true` 从采集启动起交替使用一个／三个名义周期，1000 Hz 名义设置下为真实 1／3 ms 间隔，
平均有效频率因此为 500 Hz，此压力测试不代表平均频率保持 1000 Hz 的随机抖动。
采样仍落在物理步上，不伪造时间戳。高频日志以 `injected_drop` 标记未采样记录，
相应记录无 taxel 字段；已采样记录的 `injected_spike` 标记毛刺。
短 stale 只冻结目标增长，控制器仍执行原目标；不能据此宣称陈旧反馈下的闭环力控已安全。
真机故障规则和权限保持原样。故障配置用于有限探索，不是正式 study 或完整硬件故障模型。

```bash
uv run pgt run force-schedule --experiment dm_gripper/unified_adaptive
# 自重 2.5 N 的撤支撑工况，1 N 初始抓力、50 N/s 限速；当前位移仍未达标
uv run pgt run force-schedule --experiment dm_gripper/unified_step_load
# 独立快速导纳候选，用于后续滤波与瞬态响应诊断，不替换原基线
uv run pgt run force-schedule --experiment dm_gripper/unified_step_load_fast
# 进一步缩短载荷观测滤波，检查撤支撑后的最初 200 ms
uv run pgt run force-schedule --experiment dm_gripper/unified_step_load_transient
# 独立 1000 Hz 触觉预处理与 250 Hz 控制，保留高频触觉日志
uv run pgt run force-schedule --experiment dm_gripper/unified_step_load_multirate
# 保留相同上层策略，比较位置式 PID＋机构力矩前馈
uv run pgt run force-schedule --experiment dm_gripper/unified_step_load_pid
```

`task.unified_adaptive` 与 `adaptive_prior` 互斥；省略两者仍使用 Oracle。
统一模式独立使用 `unified_adaptive.load` 的摩擦先验、力范围与速率，旧 `scheduler` 不参与目标生成。
共享的 `UnifiedAdaptivePolicy` 在完成预载前只观测，之后按新样本时间推进绝对承载需求。
它只接收双侧各九点三轴力、实测平均单侧力、时间和上一控制周期执行限幅；
场景摩擦、外加载荷和物体位移不进入策略。导纳开启最终 MIT 请求限幅的状态回投。

`unified_step_load_fast` 继承 `unified_step_load`，只将导纳速度上限改为 0.20 rad/s、
力矩前馈比例改为 1.0，其余任务与控制参数不变。该组合仅供仿真瞬态诊断，尚未通过
2 mm 滑移验收，不是全局默认或真机推荐参数。原基线继续保留 0.05 rad/s 与 0.2。
诊断必须额外检查撤支撑后的前 0.2 s；现有验收 RMSE 默认排除这一窗口，不能单独代表接住能力。

`unified_step_load_transient` 继承快速候选，只将承载调度的 `load.filter_tau_s` 从 0.05 s
改为 0.01 s，减少实测切向载荷进入目标生成的滞后；不是修改法向力的 20 Hz 低通。
导纳质量、阻尼、刚度、1 N 初始目标、50 N/s 目标速率上限及 2 mm 判据均保持不变，
局部风险增力和在线摩擦更新仍关闭。回归额外检查从撤支撑开始的完整位移窗口，以及
滤波平均单侧力首次达到 2 N 的时间；首次过阈值不等于稳定时间。
该组合用于当前硬物体、自重 2.5 N 的仿真工况，不自动替换原基线、全局默认或真机参数；
更短滤波对噪声更敏感，跨材料、噪声水平及真机表现仍需独立验证。

`unified_step_load_multirate` 在瞬态候选上显式启用 `task.tactile_sampling`：
`period_s=0.001`、`median_window=3`、`stale_after_s=0.01`、`record_raw=true`。
共享场景默认物理步长为 1 ms，控制周期仍为 4 ms，每个控制周期对应四个触觉采样。
未配置此字段的组合保持原观测调用路径，也使用新的默认物理步长；历史产物与实验结论不回溯改写。
切向分量先做三点因果中值，再聚合分侧合力模，
最后使用 `unified_adaptive.load.filter_tau_s` 在采样侧进行唯一一次低通；法向反馈保持不变。
首帧播种中值历史；长间断或坏帧后重建历史，不把不连续数据拼成载荷导数。

多速率组合只允许统一策略，可显式开启风险增力／摩擦更新；采样与控制周期须为物理步长的整数倍，
采样周期不得大于控制周期。仿真记录实际采样时间，不回填不存在的历史帧。
多速率 `trace.csv` 按控制周期记录，追加 `sensor_sequence_id`、`sensor_time_s`、`control_time_s`、
`sensor_age_s`、`sensor_stale`、`sensor_valid`、丢帧／事件累计与分侧滤波承载。
旧 `time_s` 仍是执行该请求后一个物理步的评价时刻，分析控制时序应使用 `control_time_s`。
可选 `tactile.jsonl` 按采样周期保留原始 taxel、滤波承载与风险／摩擦旁路诊断，并登记到 manifest。
坏帧累计计数跨最新快照保留，非有限原始分量在严格 JSON 中写为 `null`，同时明确标记样本无效。
指标仍从所有物理步计算，不因控制日志降采样改变原评分；前 200 ms 需另作完整窗口诊断。

可用 `--set task.definition.tactile_sampling.median_window=1` 旁路中值，或改
`period_s=0.004` 比较 250/250 Hz；比较 5 ms 低通时使用
`--set task.definition.unified_adaptive.load.filter_tau_s=0.005`。
这些都是独立探索覆盖，不自动改变原瞬态候选，亦不代表异常、材料和真机评测已经全部完成。

`unified_step_load_pid` 是仿真专用对照：保持同一任务、先验、1 N 初始力、50 N/s 目标限速、
20 Hz 法向低通、4 ms 外环、MIT 增益、协议力矩上限与配对 seed，仅替换下层完整控制结构。
复用历史 `pid-torque-ff` 的位置式 PID；启用 `window_linear` 以进入原有模型前馈路径，
但刚度位置前馈增益严格为零，估计值不进入上层调度。不能只切换配置名称而关闭实际前馈。

两组每个物理步均重新施加保持的 MIT 请求。PID 保留原有积分限幅，上层消费位置修正、
协议位置及力矩触边诊断；并不宣称它具有导纳式状态回投。两组的内部运动约束和前馈比例不同：
导纳速度请求上限为 0.05 rad/s、前馈比例为 0.2；PID 的位置修正上限为 0.15 rad、
模型前馈比例为 1，没有同等的逐周期位置目标速度限制。因此这是完整配置对照，不是仅控制律的公平消融，
也不是完全相同的目标曲线回放；反馈不同会产生不同的自适应目标轨迹。
新增 `motor_position_rad`、`motor_velocity_rad_s`、`requested_position_rad`、
`force_feedforward_torque_n_m`、`execution_limited` 用于检查这一边界。真机统一模式仍仅支持导纳。

观测器使用接触滞回、因果时间窗、加载趋势、剪切重分配、法向增力排除、持续确认和冷却。
风险事件只消费一次，并受步长、累计增量、次数和持续时间约束；零承载缺口时也能抬高目标。
显式配置 `risk_repeat_interval_s` 后，持续满确认时间的风险可按该间隔继续增力，
无需等待风险解除再生成独立事件；续增不构成新的摩擦证据。默认 `null` 保留旧单事件行为。
持续风险的确认标志由采样侧生成，避免控制抽取漏过短暂证据中断后错误认定持续风险。
已确认的大步请求通过 `risk_rate_n_s` 快速完成，但始终受总目标变化率和力上限约束。
重复、陈旧、坏帧或执行限幅不生成新步数，不补算冻结期间增力。
采样快照短期锁存最近事件及候选，避免 1000/250 Hz 抽取遗漏；接触变化、坏帧、长间断或
超过采样新鲜度窗口即清除，控制侧只消费一次，不逐个补发漏过的历史事件。
摩擦候选额外要求相关触点剪切份额下降且局部力比不再增长，取事件前有效比值中位数；
质量分数仅是有效触点覆盖率，不是统计置信度或真实摩擦的保证。
较低候选经折减后可即时接受，提高需多个独立一致事件；低质量或越界候选不覆盖已有估计，
接触变化及过期回退到先验。不能把普通稳态切法向力比直接解释为真实静摩擦系数。

`dm_gripper/unified_step_load_risk` 是独立实验候选：开启两条权限，风险窗口 40 ms、确认 12 ms，
每步 1 N、续增间隔 100 ms、风险附加速率 50 N/s，累计预算 12 N／12 步／20 s，
保留仿真原 8 N 上限。它显式开启 `observer.allow_steady_load_risk`，允许稳载下局部剪切
重分配形成风险，但仍排除明显卸载与主动法向增力；恒定载荷本身不触发。
这些为待验证参数，纯力信号没有局部变化时仍可能漏检持续滑动，不能声称一定接住真实物体。
该候选同时启用 `observer.stable_contact_subset`：仅比较窗口内始终存在的触点，双侧各至少两个，
边缘点进出不清空整个窗口，也不直接算作局部重分配；不足时仍重建窗口。
此模式的切向／法向趋势由同一窗口斜率给出，不使用噪声敏感的单采样间隔瞬时差分；
代价是卸载或主动增力证据撤销最多延迟一个窗口，需在目标硬件上验证误触发。
摩擦质量是共同触点数除以九，候选阈值 0.4 要求至少四点，不是统计置信度。
设备绝对时间的浮点减法采用纳秒级窗口容差，避免整周期窗口被反复误判为预热。

默认 `risk_enabled=false`、`friction_update_enabled=false`，观测仅诊断。
受控事件注入验证权限、去重、预算和摩擦状态逻辑，不验证真实传感器起滑识别。
仿真复用恒载、缓增载、自重 2.5 N 的撤支撑及承载不足负例，未扩大为完整材料或重量评测。
`unified_step_load` 使用约 0.254842 kg 物体，在默认重力加速度 9.81 m/s² 下自重为 2.5 N；
质量与几何共同决定惯性，全程无额外向下外力。夹爪进入力跟踪并经过原有 0.3 s 预载等待后，
关闭支撑接触，撤支撑时刻定义为场景时间零，随后观察 4 s。
该组合初始平均单侧抓力 `load.min_force_n=1.0`，目标速率上限 `load.max_force_rate_n_s=50.0`；
接近前馈 `approach.feedforward_force_n=1.0` 是另一个量，不等同于接触后实测预载。
滤波、导纳、MIT、8 N 目标上限及 2 mm 位移阈值保持不变。高限速仅是仿真候选，不是实际增力保证，
目标仍受缺口增益与载荷趋势限制。常规 `unified_adaptive` 和真机 YAML 保留原设置。
这是半瓶水总重量的刚性方块近似，不包含瓶体变形和液体晃动。

该入口已纠正早期“50 g 方块额外施加 2.5 N”的建模。旧运行快照仍保留原始输入和结果，
不能作为本工况的证据；本工况也不再叠加原先的 50 g 自重。

trace 的 `adaptive_*` 列记录风险、事件号、触点掩码、分侧摩擦及候选、质量／更新原因、
调度缺口、跟踪误差、执行限幅和失败原因。原始需求仍在 `raw_target_force_n`。
持续能力不足、执行受限、接触无效或耗尽风险预算会锁存 `failure_reason`；长采样间断立即失败。
仿真失败后冻结目标增长并继续评分，结果不能通过；真机故障处置见
[通用抓取实验](dmgripper-experiments.md#unified-adaptive)。

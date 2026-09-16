# 动态目标力跟踪

`force-track` 先建立双侧接触，再跟踪 waypoint 定义的平均单侧法向力。单次运行、viewer 和批量执行见
[工作流](workflows.md)，对比协议见[控制器对比](control-comparison-ablation.md)。
默认实验 `dm_gripper/force_tracking_default` 使用 `adrc_torque`、`window_linear`、Ramp 和 `hard` 接触 preset；
具体组合以 `configs/experiment/dm_gripper/force_tracking_default.yaml` 为准。

## 接触与目标曲线

所有 DMgripper 力控制器共享接近、接触速度过渡、力跟踪三阶段状态机。双侧力达到
`contact_threshold_n` 并满足确认步数／稳定时间后进入过渡；丢失接触按 `release_policy`、
释放阈值和确认步数重新接近。状态机拒绝非有限输入，但没有自动识别滑移或 taxel 集合切换的机制。

任务 `approach.duration_s` 生成接近轨迹，`timeout_s` 限制等待时间，
`settle_after_contact_s` 控制接触后的静置。`feedforward_force_n` 的单位是平均单侧力 N，
通过当前闭合雅可比映射为 \(\tau_{\mathrm{ff}}=f_{\mathrm{ff}}J_c(q)\)，不是固定电机力矩。
接近时间不占用目标曲线的跟踪时间；`release_support_on_tracking=true` 可在跟踪时撤掉支撑。
PID／导纳的统一参数比较见[共享控制核](dm-shared-control.md)。

`reference.waypoints` 由严格递增的 `t_s`（s）与非负的 `force_n`（N）组成，最后一个时刻决定跟踪时长。
`hold` 用于阶跃，`linear` 用于匀速加载／卸载，`smoothstep` 用于平滑过渡；对应标准任务为
`step.yaml`、`ramp.yaml`、`mixed.yaml`。Ramp 在卸载至 1 N 后保持 2 s，以统计终端稳态误差。
比较控制器时固定同一任务与 `metrics.ignore_initial_s` 评价时间窗。

## 控制律

所有目标与误差均使用 \(f_n=(F_L+F_R)/2\)。运动学、等效刚度和静态力矩映射见
[力控模型](crank-slider-force-control.md)。控制算法位于 `packages/dm_grasp_core`，
`src/parallel_gripper_tactile/control.py` 负责配置与 MuJoCo 适配。

### 位置式 PID 与刚度变体

`pid-only`、`pid-torque-ff`、`pid-stiffness-ff`、`full` 分别选择 PID、机构力矩前馈与刚度位置修正的组合。
`pid-stiffness-limit` 关闭刚度加法修正，改用刚度约束位置偏置增量；该变体仅保留专项实验，
不在默认正式比较矩阵中。`pid-stiffness-rate` 是独立的速率式控制变体。

这里的 PID 输出是相对每周期实测位置的**位置偏置**，不是速度命令或相邻周期指令增量。令
\(e_k=F_{\mathrm{ref},k}-F_{n,k}\)、外环周期为 \(T_c\)，则未限幅输出可写为

\[
\delta q_k^\ast=K_p e_k+K_i\sum_{i=0}^{k}e_iT_c-K_d\frac{F_{n,k}-F_{n,k-1}}{T_c}.
\]

普通 PID 将该输出与可选刚度位置修正相加，再裁剪到
\([-\delta q_{\max},+\delta q_{\max}]\)，最终使用
\(q_{\mathrm{ref},k}=q_{\mathrm{real},k}+\delta q_k\)。位置参考取当期反馈，
既不是固定接触位置，也不是上一周期位置指令；限幅约束当前位置附近的偏置，不约束累计闭合行程。
MIT 内环与后端仍分别执行原有机械行程、速度及力矩限制，力矩前馈开关与增益保持独立。

共享核的 `max_position_adjustment=None` 可单独关闭 PID 的固定偏置及积分幅值限幅；
后端请求受限且误差继续推向饱和方向时停止本周期积分，反向误差仍允许积分消退。
真机通过 `controller.pid.max_position_adjustment_rad: null` 启用；现有仿真 profile 的有限上限
保持原配置。机械行程和力矩等后端保护不随该选项关闭。

刚度位置限幅变体先计算

\[
\Delta F_{\mathrm{allow},k}=\min\!\left(|e_k|,\dot F_{\mathrm{lim}}T_c\right),\qquad
\Delta q_{\mathrm{lim},k}=\frac{\Delta F_{\mathrm{allow},k}}
{\gamma\,\hat k_{c,k}\,J_c(q_k)},
\]

再把 \(\delta q_k^\ast\) 裁剪到
\([\delta q_{k-1}-\Delta q_{\mathrm{lim},k},\delta q_{k-1}+\Delta q_{\mathrm{lim},k}]\)
与全局位置偏置范围的交集，再加到当期实测位置。它只限制偏置变化，不限制实测位置变化造成的
指令位移。因此只有积分项、微分项和限幅换算显式使用
\(T_c\)；不存在“PID 先输出速度，再乘 \(T_c\)”这一步。

此语义从当前 Unreleased 版本起生效；此前固定接触位置参考的运行产物和研究结论保留为历史证据，
不能当作新控制律的性能验证。一阶 LADRC 与刚度速率变体仍将累计位置修正加到固定接触位置。

`--set controller=dm_gripper/pid_stiffness_rate` 是独立的速率式实验变体，不改变上述位置式 PID。
它令 PID 输出期望力变化率，再用在线刚度和机构雅可比换算为电机角速度：

\[
\begin{aligned}
e_k &= F_{\mathrm{ref},k}-F_{n,k},\\
\dot F_k^\ast &= K_P e_k+K_I\xi_k-K_D\frac{F_{n,k}-F_{n,k-1}}{T_c},\\
\xi_k &= \xi_{k-1}+e_kT_c,\\
\dot q_k &= \operatorname{clip}\!\left(
\frac{\operatorname{clip}(\dot F_k^\ast,-\dot F_{\max},\dot F_{\max})}
{\hat k_{c,k}J_c(q_k)},-\dot q_{\max},\dot q_{\max}\right),\\
\delta q_k &= \operatorname{clip}\!\left(
\delta q_{k-1}+\dot q_kT_c,-\delta q_{\max},\delta q_{\max}\right).
\end{aligned}
\]

此时 \(K_P\) 的单位为 \(\mathrm{s}^{-1}\)，\(K_I\) 为 \(\mathrm{s}^{-2}\)，\(K_D\) 无量纲；
`T_c` 显式进入积分、微分与位置更新，反馈力变化率限幅只约束反馈支路，不是实际接触力变化率的硬约束。

位置式 PID 也以实际 \(T_c\) 计算积分与微分；采样保持、离散相位、量化及接触动力学仍使闭环性能受频率影响。

### 直接力矩 MB-ADRC

`controller=dm_gripper/adrc_torque` 在接近阶段保留 MIT 位置伺服，跟踪阶段旁路 MIT `kp/kd`，
由三状态 current LESO 估计力、力变化率与残差扰动。控制导向模型为：

\[
\ddot F=f_{res}+b_0\tau_{res},\qquad
b_0=\operatorname{clip}\!\left(s_b\frac{\hat k_{pair}J_c(q)}{I_{eq}},b_{min},b_{max}\right).
\]

机构前馈 \(\tau_{model}=F_{ref}J_c(q)\) 承担名义静态力矩；LESO 的已知输入为最终实际力矩减去同周期
模型前馈，避免重复补偿。控制律内部以 \(\omega_c^2\)、\(2\omega_c\) 提供名义 PD 动态，
并使用参考力的一、二阶导数。切换时按上一周期实际力矩初始化扰动，调度 `b0` 时同步缩放扰动状态；
力矩经变化率、量化与幅值限制后的实际值反馈给下一周期 LESO。

LESO 使用独立 40 Hz 低通，不复用 PID、刚度估计及指标的 20 Hz 公共低通。
默认 \(f_c=40\) Hz、\(\omega_c=60\) rad/s、\(\omega_o=240\) rad/s 是仿真参考点，
不代表所有任务最优或实机标定。硬件迁移需重新辨识输出轴惯量、闭合行程—力斜率和力矩—力动态。
该路径要求机构几何与刚度估计，和一阶 `adrc`、`torque_feedback_gain > 0` 互斥。

`adrc_torque_td` 另加 180 rad/s 临界阻尼线性 TD，只整形 `hold` 等参考跳变，模型前馈同步使用整形参考；
`linear`、`smoothstep` 使用解析导数并旁路 TD。它不改变测量链路，也不进入默认比较矩阵。
`direct_torque` 与一阶位置式 `adrc` 保留独立复现入口：前者直接输出反馈力矩，后者保留 MIT 位置环，
由 LADRC 闭合速度积分生成位置修正。

### 公共低通

公共法向力滤波器按实际外环周期离散化：

\[
F_{f,k}=F_{f,k-1}+\alpha(F_{n,k}-F_{f,k-1}),\qquad
\alpha=1-e^{-2\pi f_cT_c}.
\]

它不是固定延迟 \(1/f_c\)。其精确离散频率响应为

\[
H(e^{j\omega})=\frac{\alpha}{1-(1-\alpha)e^{-j\omega}},\qquad
\omega=2\pi fT_c.
\]

例如 20 Hz 截止频率、4 ms 外环周期下，对 8.3 Hz 信号的离散相位滞后约为 17.1°，等效约
5.7 ms；不能把截止频率的倒数 50 ms 当作固定延迟。

## 输出与时间语义

run 保存 `profile.yaml`、`task.yaml`、`effective_parameters.json`、`trace.parquet`、
`metrics.json` 和 `manifest.json`。有效参数快照用于确认实际组合与运行时覆盖。
默认只生成 `plots/tracking.png`；diagnostic 模式或科学失败增加触觉与控制器诊断图。
重绘与论文导出见[工作流](workflows.md)，产物／路径约定见[科研配置](research-configuration.md)。

常规 trace 默认按普通控制器 100 Hz、直接力矩 ADRC 250 Hz 存储；首尾、状态／限幅切换、waypoint
前后 0.2 s 保留完整控制频率。指标与初次绘图使用降采样前数据；重绘读取原指标，不重新统计。
`execution.trace_sample_period_s` 必须不小于且为控制周期的整数倍，设为控制周期可保留全频；
事件窗口由 `execution.trace_event_window_s` 配置。旧 `trace.csv` 仍可读取。

trace schema v2 区分目标、测量／滤波力、位置／速度命令、控制分解与实际执行器力矩。
`motor_torque_n_m` 和 `commanded_torque_n_m` 是命令力矩，`actuator_torque_n_m` 来自 MuJoCo
`qfrc_actuator`。逐 taxel `left/right_taxel_fx/fy/fz_<row>_<col>` 始终记录，细节图按需开启。

`stiffness_valid` 表示本次接触 reset 后至少完成过一次有效估计更新；估计保持期间仍为有效，
不表示当前周期有新更新，也不代表材料真值。未有效时绘图使用 NaN；缺字段时按 capability 省略，
不伪造 `K_hat`。

时间列按控制边界定义：`control_time_s` 是最近外环步之前的时间，用于目标、滤波力和估计器；
`command_time_s` 是最近 MIT 命令应用步之前的时间，用于关节反馈和命令；
`reference_start_time_s` 是精确任务起点；原 `time_s` 仍是物理步后的触觉时间。

## 指标解读 {#force-tracking-metrics}

`metrics.json` 可用于快速比较不同配置：

| 指标 | 解读 |
| --- | --- |
| `rmse_n` | 均方根误差，反映整体跟踪质量。 |
| `mae_n` | 平均绝对误差，比 RMSE 对尖峰不那么敏感。 |
| `peak_abs_error_n` | 最大绝对误差，用于观察最坏瞬时偏差。 |
| `mean_error_n` | 平均有符号误差，用于判断是否存在系统性偏大或偏小。 |
| `final_error_n` | 结束时的力误差。 |
| `torque_saturation_ratio` | 力矩命令触达限幅的比例。 |
| `position_saturation_ratio` | 位置命令触达限幅的比例。 |
| `mean_estimated_stiffness_n_per_m` | 跟踪阶段平均等效接触刚度。 |
| `stiffness_position_limit_ratio` | 评价区间内刚度感知位置边界实际触发的周期比例。 |
| `rise_time_s` | 加载阶跃后滤波力首次达到阶跃前平台加 90% 阶跃幅值的耗时；非 `hold` 任务、无合格阶跃或窗口内未达到时为 `null`。 |
| `overshoot_ratio` | 阶跃平台窗口内滤波力峰值超出目标平台的幅值与阶跃幅值之比（下限为 0）；无法判定时为 `null`。 |
| `settling_time_s` | 滤波力进入并保持 ±5% 阶跃幅值稳定带的首个时刻相对阶跃起点的耗时；窗口末尾仍未稳定或无法判定时为 `null`。 |
| `simulation_stable` | 仿真是否保持稳定。 |

如果 RMSE 高但饱和比例低，通常优先检查控制器增益、刚度估计滤波和前馈比例。
如果饱和比例高，说明目标力、目标变化速度或执行器限幅之间不匹配，应先降低目标曲线强度或放慢变化速度。
如果刚度估计大幅跳变，应检查是否发生接触柱集合变化、滑移、脱离或目标力变化过快。

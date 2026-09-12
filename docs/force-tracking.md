# 🎯 动态目标力跟踪任务

`force-track` 用于测试 DM_Gripper 对时变法向力目标的跟踪能力。它把一次实验拆成接触接近和目标力跟踪两个阶段：
先以低速闭合建立双侧接触，再按照 waypoint 定义的时间曲线跟踪目标力。

<pre><code class="language-bash">
uv run pgt run force-track \
  --set task=force_tracking/default_waypoints
</code></pre>

需要实时观察 MuJoCo 场景时加 `--viewer`：

<pre><code class="language-bash">
uv run pgt run force-track \
  --set task=force_tracking/default_waypoints \
  --set execution.viewer=true
</code></pre>

默认 viewer 按 1 倍实时速度播放。若需要慢放或改变刷新率，可使用 `--realtime-factor`
和 `--render-fps`。

`--set execution.multiccd_enabled=false` 只用于碰撞流形诊断：它保留 model 组指向的碰撞模型，但将 MuJoCo 的
`multiccd` 求解选项关闭。默认 profile 已采用共面球体碰撞近似，常规力跟踪不需要这个开关。

默认任务配置位于 `configs/task/force_tracking/default_waypoints.yaml`。标准控制器比较还提供
`step.yaml`、`ramp.yaml` 与 `mixed.yaml`，分别使用 `hold`、`linear` 和 `smoothstep`
插值。任务配置只描述流程和目标力曲线；夹爪结构、执行器限幅、触觉传感器和控制器参数仍来自
统一组合得到的 platform、model、controller 与 estimator 片段。

## 1. 两阶段流程

另有独立的 [DMgripper 导纳基线](dm-shared-control.md)，通过
`--experiment dm_gripper/force_tracking_admittance` 选择。它包含接近、接触速度过渡和跟踪，
使用当前 ROS 2 的共享导纳核，不进入本页历史 PID/ADRC 默认比较矩阵。
全部 DMgripper 力控制器现在共用三阶段双侧接触状态机。接近轨迹与前馈统一由任务的
`approach.duration_s` 和 `approach.feedforward_force_n` 提供，`approach.timeout_s` 控制等待上限；
导纳配置中的 `approach_*` 字段只供未启用公共状态机的核心兼容调用使用。

PID 与导纳的隔离对比使用
`experiment=dm_gripper/force_tracking_pid_unified` 和
`experiment=dm_gripper/force_tracking_admittance_unified`。两者共同选择
`dm_unified_ramp`：目标力沿用 PID Ramp 的 `1→3→6→1 N`，外环周期统一为 4 ms；
MIT 增益、6 s 线性关节接近轨迹、接近前馈、接触阈值和公共状态机参数也逐项相同；
若 8 s 内仍未建立双侧接触则任务失败，避免预接触阶段长时间空跑。
因此 `track_reference` 中只有 PID 与二阶导纳跟踪律不同。统一入口的双侧有效接触要求任一侧
持续 100 ms 不高于 0.05 N 时重新接近；瞬时掉力和普通跟踪误差不触发状态切换。

### 接近阶段

接近阶段由 `approach` 配置段控制。该阶段夹爪按低速闭合，直到确认左右两侧都稳定接触物体。
此时目标力曲线的计时还没有开始，因此接触建立时间不会挤占后续 waypoint 的跟踪时间。

`feedforward_force_n` 是接近阶段使用的名义平均单侧法向力前馈，单位为 N。它不是固定电机力矩。
控制器会根据当前关节角动态计算闭合行程雅可比 \(J_c(q)\)，再映射成 MIT 力矩前馈：

\[
\tau_{\mathrm{ff}}(q)\simeq f_{\mathrm{ff}}J_c(q)
\]

例如默认的 `feedforward_force_n: 1.0` 表示平均单侧法向力前馈为 1 N，也就是理想对称接触下总法向力约 2 N。
由于 \(J_c(q)\) 随关节角变化，最终写入 `t_ff` 的力矩也会随当前姿态变化。

接近阶段常用字段如下：

| 字段 | 含义 |
| --- | --- |
| `duration_s` | 接近阶段名义持续时间，用于生成低速闭合轨迹。 |
| `timeout_s` | 最长等待接触时间，超过后任务失败退出。 |
| `settle_after_contact_s` | 双侧接触确认后的静置时间，用来让接触力和滤波器稳定。 |
| `feedforward_force_n` | 接近阶段名义平均单侧法向力前馈，会动态换算为当前关节角下的力矩前馈。 |

### 接触与信号状态

控制器仅在左右接触都连续达到 profile 中 `contact_threshold_n` 且满足确认步数时进入跟踪；
此时 `F_L`、`F_R` 和 `f_n=(F_L+F_R)/2` 均为有效测量。单侧接触时保持接近/保守反馈，
不更新 `k_pair`；左右两侧都低于释放阈值并满足确认步数后，退出跟踪并重新接近。出现缺失、
非有限值、符号/坐标约定不一致或接触集合快速变化等信号异常时，冻结刚度更新和依赖刚度的
前馈，记录异常并退回保守控制，直到重新确认双侧接触。

### 跟踪阶段

进入跟踪阶段后，任务时间 `tracking_time_s` 从 0 开始。控制器读取目标力 \(f_{\mathrm{ref}}(t)\)，
结合触觉测得的平均单侧法向力 \(f_n=(F_L+F_R)/2\)、接触刚度估计、位置修正和力矩前馈生成执行器命令。

默认为位置式力控：PID 位置修正与刚度位置前馈修正目标位置，模型力矩前馈进入 MIT `t_ff`。
`--set controller=dm_gripper/pid_stiffness_limit` 保留 PID 和机构力矩前馈，但关闭刚度位置前馈；在线刚度只把
`position_limit_force_rate_n_s·dt` 的允许预测力变化换算成每周期位置目标增量上限，并通过动态 PID
输出边界抑制积分 windup。刚度安全系数由 `position_limit_stiffness_safety_factor` 配置。该变体已进入
当前默认正式比较矩阵，与历史 `pid-stiffness-ff`、`full` 同时保留。

这里的 PID 输出是相对接触位置的**位置修正**，不是速度命令。令
\(e_k=F_{\mathrm{ref},k}-F_{n,k}\)、外环周期为 \(T_c\)，则未限幅输出可写为

\[
\delta q_k^\ast=K_p e_k+K_i\sum_{i=0}^{k}e_iT_c-K_d\frac{F_{n,k}-F_{n,k-1}}{T_c}.
\]

刚度位置限幅先计算

\[
\Delta F_{\mathrm{allow},k}=\min\!\left(|e_k|,\dot F_{\mathrm{lim}}T_c\right),\qquad
\Delta q_{\mathrm{lim},k}=\frac{\Delta F_{\mathrm{allow},k}}
{\gamma\,\hat k_{c,k}\,J_c(q_k)},
\]

再把 \(\delta q_k^\ast\) 裁剪到
\([\delta q_{k-1}-\Delta q_{\mathrm{lim},k},\delta q_{k-1}+\Delta q_{\mathrm{lim},k}]\)
与全局位置修正范围的交集，最终令
\(q_{\mathrm{ref},k}=q_{\mathrm{contact}}+\delta q_k\)。因此只有积分项、微分项和限幅换算显式使用
\(T_c\)；不存在“PID 先输出速度，再乘 \(T_c\)”这一步。

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
`T_c` 显式进入积分、微分与位置更新，因此控制律参数不再隐含“每周期位移”的含义。首轮结构验证固定
\(K_I=K_D=0\)、\(K_P=20\ \mathrm{s}^{-1}\)、\(\dot F_{\max}=50\ \mathrm{N/s}\) 和
\(\dot q_{\max}=0.5\ \mathrm{rad/s}\)。模型力矩前馈仍保持可比，因此
\(\dot F_{\max}\) 是反馈支路命令边界，不是实际接触力变化率的硬约束。

### 20 Hz 公共低通的离散延迟

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

以 \(f_c=20\ \mathrm{Hz}\)、\(T_c=4\ \mathrm{ms}\) 和已观察到的
\(f\approx8.3\ \mathrm{Hz}\) 为例，连续模型近似给出 \(-22.6^\circ\) 与 7.5 ms；代码所用离散实现
给出约 \(-17.1^\circ\) 与 5.7 ms，即约 1.4 个控制周期。因而“20 Hz 滤波增加相位滞后”成立，
但把它表述为固定 50 ms 延迟或 1.9 个实际控制周期都不准确。

若把 profile 字段 `control.force.torque_feedback_gain` 设为大于 0（`direct-torque` 控制器变体
即取 1.0，可用 `--set controller=dm_gripper/direct_torque` 运行），跟踪阶段切换为直接力矩式对照：
力误差直接进入 MIT 前馈力矩，PID 与刚度位置修正置零，MIT 位置环 kp/kd 逐周期覆盖为 0；
接近与释放阶段不受影响，仍走共享的位置伺服轨迹。

若配置 `control.force.adrc` 段（`adrc` 控制器变体自动注入默认参数，可用
`--set controller=dm_gripper/adrc` 运行），跟踪阶段切换为一阶线性自抗扰（LADRC）外环：
扩张状态观测器（LESO）估计滤波力与总扰动，控制律输出闭合速度命令 `u`（m/s），经当前
闭合雅可比换算为电机角速度后逐周期积分成持久的位置修正（裁剪到
`max_position_adjustment`），以此替换 PID 位置修正与刚度位置前馈。
与 `direct-torque` 的本质区别在于 LADRC 不旁路位置环：MIT kp/kd 保持 profile 值，
位置弹簧阻尼照常参与力矩合成；模型力矩前馈仍走 `torque_feedforward_gain` 路径，
刚度估计器照常运行以保持 trace 中刚度曲线可比。`adrc` 与 `torque_feedback_gain > 0`
互斥，同时启用会在控制器构造时抛出 `ValueError`。

`--set controller=dm_gripper/adrc_torque` 启用二阶直接力矩 MB-ADRC。接近阶段继续使用组合 profile 中的
MIT `kp/kd` 建立稳定接触；进入跟踪后仅逐周期旁路 MIT `kp/kd`，由三状态 current LESO 估计
法向力、力变化率与残差总扰动，控制律自身以 `ω_c²` 和 `2ω_c` 提供名义 PD 动态并直接输出电机力矩。
目标曲线的一、二阶导数分别作为速度和加速度前馈进入控制律。

该路径使用以下控制导向模型：

\[
\ddot F=f_{res}+b_0\tau_{res},\qquad
b_0=\operatorname{clip}\!\left(
s_b\frac{\hat K J_c(q)}{I_{eq}}, b_{min}, b_{max}
\right)
\]

机构前馈 `τ_model=F_ref·J_c(q)` 负责名义静态夹持力，LESO 的输入使用执行器实际总力矩减去同周期
`τ_model` 后的残差，因此不会把同一前馈再次当成总扰动补偿。力矩先经过变化率限制，再经过 MIT
量化和幅值限制；下一周期 LESO 使用最终实际力矩。切换时扰动状态按接近阶段最后一个实际力矩初始化，
在线调度 `b0` 时同步缩放扰动状态，使补偿力矩连续。

LESO 不直接使用完全原始的触觉力，也不复用 PID、刚度估计与评价指标使用的 20 Hz 公共低通。
它使用独立的 40 Hz 一阶低通做轻度预处理：该通道只负责抑制高频尖峰，力与力变化率的主要估计
仍由 LESO 完成。当前 40 Hz 截止频率是小范围仿真扫描得到的工程起点：连续任务表现稳定且未观察到
力矩饱和，但仍应与控制器带宽、传感器噪声和闭环时延联合复核。实机必须依据传感器采样率、噪声谱和
闭环时延重新标定。

profile 可选段 `control.force.torque_adrc` 提供 `equivalent_inertia_kg_m2`、`input_gain_scale`、
`controller_bandwidth_rad_s`、`observer_bandwidth_rad_s`、
`measurement_filter_cutoff_hz`、输入增益上下界和 `max_torque_rate_n_m_s`。默认值是当前 MuJoCo
模型的仿真起点，不是实机标定结果；迁移到硬件前必须
通过自由空间 `τ→q̈`、准静态 `c→F` 和接触状态 `τ→F` 三组辨识重新确认惯量、增益尺度与带宽。
`torque_adrc` 要求启用机构几何与接触刚度估计，并与一阶 `adrc`、`torque_feedback_gain > 0` 互斥。
当前仿真默认值 `fc=40 Hz、ωc=60 rad/s、ωo=240 rad/s` 是专用调参 study 在连续任务约束下确认的
可行参考点。它改善连续参考跟踪，但 Step 超调仍是需要单独权衡和继续调参的指标；不应把它描述为对所有
任务都更优的通用参数组合。

`--set controller=dm_gripper/adrc_torque_td` 是独立的工程增强对照：它在 `adrc-torque` 前增加带宽为
180 rad/s 的临界阻尼线性 TD，将参考力整形成连续的力、力变化率和力加速度；机构模型前馈同步使用
整形后的参考，避免原始阶跃绕过 TD。论文式 `adrc-torque` 仍不启用 TD，二者除此之外使用完全相同
的 LESO、模型前馈、刚度调度和限幅参数。TD 只处理参考信号，不改变“触觉力轻滤波后进入 ESO”的
测量链路，也不进入默认正式对比矩阵。Linear 与 smoothstep 已有解析导数，TD 对这两类轨迹直接
同步并旁路，只整形 `hold` 等零导数参考中的离散跳变，避免给连续任务重复增加相位滞后。
该带宽只是在 `medium`、seed 0 小范围扫描中选出的仿真起点，尚未进入默认正式矩阵或形成跨材料结论。

一阶位置式 `adrc` 保留用于历史复现，但不再进入默认控制器对比矩阵：它把一阶 LADRC 外包在保留
MIT 阻抗的位置环之外，控制导向模型与实际闭环阶次不匹配。二阶直接力矩 `adrc-torque` 的专用调参
study 扫描 `measurement_filter_cutoff_hz`、`ωc` 和 `ωo/ωc`，并拒绝滤波截止频率低于
`ωo/(2π)` 的候选。粗扫后的候选必须在 Ramp、Mixed 的 RMSE 不超过当前基线 110%、力矩饱和不超过
1% 的前提下，按 Step 超调、Step RMSE、Mixed RMSE 排序；随后在三种材料、三个 seed 上确认。

正式计划与执行使用 Hydra study 入口；默认先生成计划，加入 `execution=study_run` 才推进仿真：

```bash
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study study.stage=confirm \
  study.coarse_study_dir=/absolute/path/to/coarse-study
```

coarse 固定生成 102 条有序条件。confirm 会校验 coarse 的研究类型、阶段、科学配置哈希、完成状态、
执行异常计数以及排名文件的 schema 与 SHA-256 摘要，再按可行排名选择候选；不能把其他配置或未完成
coarse 的排名混入确认统计。研究只通过 Hydra 正式入口执行，避免路径参数绕过统一 profile 组合。

若需要测试撤掉支撑后的真实夹持能力，可以把 `release_support_on_tracking` 设为 `true`。
若只想先评估力控曲线本身，保持默认支撑更利于排除掉落和姿态变化的干扰。

## 2. Waypoint 目标力曲线

目标力曲线由 `reference.waypoints` 描述。每个 waypoint 包含：

| 字段 | 含义 |
| --- | --- |
| `t_s` | 跟踪阶段内的时间，单位 s。 |
| `force_n` | 该时刻的目标平均单侧法向力，单位 N。 |

waypoint 的 `t_s` 应严格递增，`force_n` 应为非负值。任务总跟踪时长由最后一个 waypoint 的
`t_s` 决定。

插值方式由 `reference.interpolation` 控制：

| 方式 | 适用场景 |
| --- | --- |
| `hold` | 阶跃目标，用于观察上升时间、超调和稳态误差。 |
| `linear` | 斜坡目标，用于测试可重复的匀速加载和卸载。 |
| `smoothstep` | 平滑过渡目标，适合默认综合测试，能减少目标曲线拐点处的瞬时冲击。 |

建议在一条测试曲线中同时保留爬升段、平台段和卸载段。平台段可以观察稳态偏差和刚度估计是否漂移；
卸载段可以暴露积分残留、迟滞和接触状态变化带来的问题。其中 `ramp.yaml` 在 6 s 匀速卸载结束后
附加 2 s 终端保持段（保持 1 N），用于统计终端稳态误差。

## 3. 配置建议

设计目标力曲线时，优先从保守目标开始：

1. 第一段目标力不要过高，例如从 2 N 左右开始；
2. 斜率不要太陡，先确认 `torque_saturation_ratio` 和 `position_saturation_ratio` 接近 0；
3. 每个平台至少保留 1 到 2 秒，方便比较稳态误差；
4. 若主要测试控制器动态响应，可使用 `hold` 阶跃；若主要测试平稳跟踪，可使用 `smoothstep`；
5. 逐步提高峰值力和变化速度，而不是一次性给出接近执行器上限的目标。

`metrics.ignore_initial_s` 可用于跳过跟踪阶段刚开始的过渡过程，只统计稳定后的误差。
如果需要比较不同控制参数，保持同一个 task YAML 不变，只替换 profile 中的控制器参数。

## 4. 输出产物

每次运行会在实验输出目录中保存配置快照、逐步数据和评价指标。常见文件包括：

| 文件 | 内容 |
| --- | --- |
| `profile.yaml` | 本次运行使用的夹爪 profile 快照。 |
| `task.yaml` | 本次运行使用的 force tracking task 快照。 |
| `effective_parameters.json` | 解析后的完整 profile、task 与本次实际生效的运行时覆盖。 |
| `trace.parquet` | 使用 Zstd 压缩、事件感知降采样的状态、目标力、测量力和控制量。 |
| `plots/tracking.png` | 600 DPI 目标力与滤波力单面板 PNG。 |
| `plots/tactile.png` | 600 DPI 双侧法向力与切向力 PNG。 |
| `plots/controller.png` | 600 DPI 关节、力矩及可用控制分解 PNG。 |
| `metrics.json` | 跟踪误差、饱和比例、接触时间等摘要指标。 |
| `manifest.json` | 运行命令、时间戳和产物索引。 |

profile、task 等人工输入继续采用 YAML。`effective_parameters.json` 是 force-track run 的有效参数快照，
用于区分原始输入与解析、覆盖后的实际运行语义。默认 `trace.parquet` 对常规区段进行事件感知降采样：
普通控制器为 100 Hz，`adrc-torque` / `adrc-torque-td` 为 250 Hz；首尾样本、阶段与控制状态切换、
限幅状态变化以及 waypoint 前后 0.2 s 保留完整控制频率。误差指标和图像均在降采样前计算/生成，
不会因存储采样率改变。实际采样周期和事件窗口记录在 Parquet metadata、manifest 与
`effective_parameters.json` 中。读取接口继续兼容旧 CSV API 及历史 `trace.csv` 产物。
需要覆盖默认策略时，可使用 `--trace-period` 和 `--event-window`；采样周期必须不小于
且为任务控制周期的整数倍，设置为控制周期等价于全频记录常规区段。

单次 force-track runner 将完整频率行通过 `on_result(full_rows, result)` 回调交给
`visualization/force_tracking.py`。默认只生成三张图：`tracking.png` 为 `F_ref` 与 `F_filt`
（缺失时回退 `meas`），`tactile.png` 为 `F_{nL}`／`F_{nR}` 与 `F_{tL}`／`F_{tR}`，
`controller.png` 为 `q_des`／`q`、`dq_des`／`dq`、`tau_cmd` 与 MuJoCo 执行的 `tau_act`。
存在有效字段时，控制器图还加入 `K_hat`、导纳 `x_a`／`dx_a`、ADRC 扰动和位置修正分解；
缺字段按 capability 省略。所有轴都标明物理量和单位（例如 `t (s)`），使用 SciencePlots 与
MathText；waypoint 只在线性参考曲线上放置 marker，关键接触事件才使用细灰色竖线。默认不生成
误差、滞回、limits 或 state 面板。PNG 为 600 DPI，默认不自动生成 PDF。

可用以下命令从既有 run 重绘。命令优先读取 `trace.parquet`，不存在时回退 `trace.csv`；指标始终
读取原 `metrics.json`，不从降采样数据重算。`effective_parameters.json` 优先，旧 `task.yaml` 可用；
若只有旧 profile，不把它当作有效参数，避免旧 profile 覆盖组合解析结果。

<pre><code class="language-bash">
uv run python scripts/research/render.py &lt;run-directory&gt; [--tactile-detail] \
  [--format png|pdf|both] [--output-dir 新目录]
</code></pre>

默认重绘写入原 run 下独占的 `plots/replots/&lt;UTC&gt;-&lt;id&gt;/`，拒绝覆盖已存在的
`--output-dir`，并写入独立的 `rendering_manifest.json`，登记 source hashes、实际图像和 notes。
它不修改原 `trace`、`metrics` 或 run manifest，因此不会破坏 study 摘要。默认 `tactile_detail` 为关闭；
开启后可为左右触觉阵列分别绘制空间对应的 3×3 taxel 小倍图，且只显示接触确认后的数据。每个面板
对应一个 `T_{r,c}`：左轴显示 `Fz`，右轴显示 `Fx`／`Fy`；九个面板共享法向量程和切向量程，避免
切向小信号被法向力淹没，也不采用各 taxel 独立缩放而破坏空间比较。PDF 仅在
显式请求 `--format pdf` 或 `both` 时生成。

`trace.parquet` 中最常用的列包括：

| 列 | 含义 |
| --- | --- |
| `phase` | 当前阶段，通常为接近或跟踪。 |
| `tracking_time_s` | 跟踪阶段时间，接近阶段不计入。 |
| `target_normal_force_n` | 当前目标平均单侧法向力。 |
| `measured_normal_force_n` | 触觉测得的平均单侧法向力。 |
| `filtered_normal_force_n` | 低通滤波后的平均单侧法向力。 |
| `torque_adrc_measurement_n` | 仅供直接力矩 ADRC 使用的轻度预处理触觉力；默认 40 Hz。 |
| `tracking_error_n` | 目标力减测量力。 |
| `force_feedforward_torque_n_m` | 根据目标力和机构雅可比计算的力矩前馈。 |
| `mit_feedforward_torque_n_m` | 最终送入 MIT 命令的前馈力矩。 |
| `target_force_rate_n_s`、`target_force_acceleration_n_s2` | 由 waypoint 插值得到的目标力一、二阶导数。 |
| `torque_adrc_estimated_force_n` | 二阶直接力矩 LESO 估计的法向力。 |
| `torque_adrc_estimated_force_rate_n_s` | LESO 估计的法向力变化率。 |
| `torque_adrc_estimated_disturbance_n_s2` | LESO 估计的残差总扰动。 |
| `torque_adrc_reference_force_n` 及其 rate/acceleration 列 | 直接力矩 ADRC 实际使用的参考状态；TD 变体下为整形结果。 |
| `torque_adrc_raw_torque_n_m`、`torque_adrc_limited_torque_n_m` | 变化率/幅值限制前后的直接力矩命令。 |
| `torque_adrc_residual_torque_n_m` | 扣除机构模型前馈后的 LADRC 残差控制力矩。 |
| `torque_adrc_input_gain_n_per_n_m_s2` | 按在线刚度、雅可比和名义惯量调度后的 `b0`。 |
| `torque_adrc_rate_limited`、`torque_adrc_amplitude_limited` | 本周期是否触发 ADRC 力矩变化率或幅值限制。 |
| `estimated_contact_stiffness_n_per_m` | 在线估计的整体等效刚度 `k_pair`。 |
| `closure_jacobian_m_per_rad` | 当前关节角下的闭合行程雅可比 \(J_c(q)\)。 |
| `aperture_m` | 由开度公式计算的当前夹爪开口。 |
| `stiffness_position_limit_rad` | 刚度感知变体本周期允许的 PID 位置目标最大变化量。 |
| `stiffness_position_limited` | 本周期 PID 输出是否触及刚度感知动态边界。 |
| `stiffness_rate_force_command_n_s` | 速率式变体裁剪后的反馈力变化率命令 \(\dot F_k\)。 |
| `stiffness_rate_joint_velocity_rad_s` | 经 \(\hat k_cJ_c(q)\) 映射后的电机角速度命令 \(\dot q_k\)。 |
| `stiffness_rate_force_limited` | 本周期是否触及反馈力变化率边界。 |
| `stiffness_rate_joint_velocity_limited` | 本周期是否触及角速度或累计位置修正边界。 |

### 4.1 trace schema v2 与时间语义

force-track trace schema 升级为版本 2。既有字段值、控制律和指标口径保持不变；其中
`motor_torque_n_m` 明确表示命令力矩，不是实测力矩。新增字段包括：

- `desired_position_rad`、`desired_velocity_rad_s`、`commanded_torque_n_m`、`actuator_torque_n_m`；后者来自
  MuJoCo `qfrc_actuator`；
- `stiffness_valid`、`admittance_displacement_m`、`admittance_velocity_m_s`；
- `left/right_tangential_force_n`，以及 `left/right_taxel_fx/fy/fz_<row>_<col>`。

`stiffness_valid` 表示本次接触 reset 后至少完成过一次有效估计更新；估计保持期间仍为有效，
不表示当前周期有新更新，也不代表材料真值。未有效时绘图使用 NaN；缺字段时按 capability 省略，
不伪造 `K_hat`。

时间列按控制边界定义：`control_time_s` 是最近外环步之前的时间，用于目标、滤波力和估计器；
`command_time_s` 是最近 MIT 命令应用步之前的时间，用于关节反馈和命令；
`reference_start_time_s` 是精确任务起点；原 `time_s` 仍是物理步后的触觉时间。

runner 参数 `tactile_detail=False`。逐 taxel 字段始终写入 trace；细节图可通过 Python runner 的
`tactile_detail=True` 或重绘 CLI 的 `--tactile-detail` 开启。降采样不能恢复高频信息，指标仍在完整频率数据上统一计算。

兼容的 Python `run_force_tracking(output_plot=path)` 调用仍保留；只有显式传入单个路径时才写出
单张 tracking 图，不改变 runner 默认的三张图布局。

## 5. 指标解读

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

## 6. 与力控模型的关系

`force-track` 使用的前馈力矩和刚度调度来自[曲柄滑块力控模型](crank-slider-force-control.md)。
其中开度公式用于计算闭合行程和 \(J_c(q)\)，再把目标法向力映射到位置修正和力矩前馈。

需要注意，在线估计得到的是“Pillar—物体—机构/接触链路”共同形成的整体等效 `k_pair`，
不是材料弹性模量，也不用于反推物体材料参数。它只服务于控制器前馈、增益调度和实验比较；
实机传感器直接输出力，仿真则从 taxel/Pillar 接触读取后统一为 `F_L`、`F_R` 和 `f_n`。

## 7. 碰撞几何诊断

高载荷振荡的五条件对照由独立 diagnosis protocol 保存，避免把因果诊断开关混入默认任务：

<pre><code class="language-bash">
uv run python scripts/research/study.py \
  research=archive/model_bug_diagnosis/study \
  study.phase=collision-geometry \
  execution=study_run
</code></pre>

该阶段分别控制 Pillar 高度共面性、mesh/sphere 拓扑和 `multiccd`。当前证据表明，问题来自原始非共面
mesh 与 `multiccd` 的组合导致接触流形在 18 与 72 个活跃接触之间切换；并不能归因于 PID、平均单侧
力语义、非共面性、mesh 拓扑或“72 个接触”中的任一单独因素。默认碰撞近似和完整结果见
[触觉读数约定](tactile-conventions.md#collision-geometry-conclusions)。

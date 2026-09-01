# 🎯 动态目标力跟踪任务

`force-track` 用于测试自研夹爪对时变法向力目标的跟踪能力。它把一次实验拆成接触接近和目标力跟踪两个阶段：
先以低速闭合建立双侧接触，再按照 waypoint 定义的时间曲线跟踪目标力。

<pre><code class="language-bash">
uv run pgt run force-track \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_tracking/default_waypoints.yaml
</code></pre>

需要实时观察 MuJoCo 场景时加 `--viewer`：

<pre><code class="language-bash">
uv run pgt run force-track \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_tracking/default_waypoints.yaml \
  --viewer
</code></pre>

默认 viewer 按 1 倍实时速度播放。若需要慢放或改变刷新率，可使用 `--realtime-factor`
和 `--render-fps`。

`--disable-multiccd` 只用于碰撞流形诊断：它保留 profile 指向的碰撞模型，但将 MuJoCo 的
`multiccd` 求解选项关闭。默认 profile 已采用共面球体碰撞近似，常规力跟踪不需要这个开关。

默认任务配置位于 `configs/force_tracking/default_waypoints.yaml`。标准控制器比较还提供
`step.yaml`、`ramp.yaml` 与 `mixed_waypoints.yaml`，分别使用 `hold`、`linear` 和 `smoothstep`
插值。任务配置只描述流程和目标力曲线；夹爪结构、执行器限幅、触觉传感器和控制器参数仍来自
`--profile` 指定的 YAML。

## 1. 两阶段流程

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
卸载段可以暴露积分残留、迟滞和接触状态变化带来的问题。

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
| `trace.csv` | 每个控制周期的状态、目标力、测量力和控制量。 |
| `plot.png` | 目标力与测量力曲线图。 |
| `metrics.json` | 跟踪误差、饱和比例、接触时间等摘要指标。 |
| `manifest.json` | 运行命令、时间戳和产物索引。 |

`trace.csv` 中最常用的列包括：

| 列 | 含义 |
| --- | --- |
| `phase` | 当前阶段，通常为接近或跟踪。 |
| `tracking_time_s` | 跟踪阶段时间，接近阶段不计入。 |
| `target_normal_force_n` | 当前目标平均单侧法向力。 |
| `measured_normal_force_n` | 触觉测得的平均单侧法向力。 |
| `filtered_normal_force_n` | 低通滤波后的平均单侧法向力。 |
| `tracking_error_n` | 目标力减测量力。 |
| `force_feedforward_torque_n_m` | 根据目标力和机构雅可比计算的力矩前馈。 |
| `mit_feedforward_torque_n_m` | 最终送入 MIT 命令的前馈力矩。 |
| `estimated_contact_stiffness_n_per_m` | 在线估计的整体等效刚度 `k_pair`。 |
| `closure_jacobian_m_per_rad` | 当前关节角下的闭合行程雅可比 \(J_c(q)\)。 |
| `aperture_m` | 由开度公式计算的当前夹爪开口。 |

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
uv run python scripts/experiments/force_tracking_diagnosis.py \
  --config configs/studies/force_tracking_diagnosis.yaml \
  --phase collision-geometry
</code></pre>

该阶段分别控制 Pillar 高度共面性、mesh/sphere 拓扑和 `multiccd`。当前证据表明，问题来自原始非共面
mesh 与 `multiccd` 的组合导致接触流形在 18 与 72 个活跃接触之间切换；并不能归因于 PID、平均单侧
力语义、非共面性、mesh 拓扑或“72 个接触”中的任一单独因素。默认碰撞近似和完整结果见
[触觉读数约定](tactile-conventions.md#2026-08-31-ab)。

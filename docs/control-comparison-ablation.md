# 控制算法对比与消融实验规划

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

## 7. 配置组织建议

短期可以在当前项目中按如下方式组织：

```text
configs/
├── force_tracking/
│   ├── default_waypoints.yaml
│   ├── step.yaml
│   ├── ramp.yaml
│   └── mixed_waypoints.yaml
└── controller_ablations/
    ├── full.yaml
    ├── pid_only.yaml
    ├── no_approach_ff.yaml
    ├── no_torque_ff.yaml
    ├── no_stiffness_position_ff.yaml
    └── no_stiffness_estimator.yaml
```

如果当前 CLI 还不支持单独加载 controller override，可以先复制完整 profile 形成可运行配置；
等实验矩阵稳定后，再增加 `--controller-config` 或 `--override`，减少重复配置。

## 8. 推荐命令形式

单次带 viewer 检查：

<pre><code class="language-bash">
uv run pgt run force-track \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_tracking/default_waypoints.yaml \
  --viewer
</code></pre>

批量对比时不建议打开 viewer，应使用 headless：

<pre><code class="language-bash">
uv run pgt run force-track \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_tracking/default_waypoints.yaml \
  --run-name full_default
</code></pre>

后续若加入 sweep 命令，理想形式可以是：

<pre><code class="language-bash">
uv run pgt compare force-track \
  --profiles configs/controller_ablations/*.yaml \
  --task configs/force_tracking/mixed_waypoints.yaml
</code></pre>

该命令尚未实现，当前应先用单次运行产物保证指标和配置字段稳定。

## 9. 决策原则

只要问题和真实 MJCF、触觉读数、viewer、run artifacts 或 force tracking 指标有关，就放在
`parallel_gripper_tactile` 中做。

只要问题可以脱离 MuJoCo，只依赖观测、参考目标和控制输出，就应逐步迁移到
`tactile-contact-control` 中做。

这样可以避免两个项目职责混在一起：当前项目负责“实验台是否可信”，旧项目负责“控制算法是否干净可复用”。

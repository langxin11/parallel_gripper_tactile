# 🧮 Oracle 抓取目标力调度

`force-schedule` 根据物体需要抵抗的切向载荷和已知摩擦系数，在线生成平均单侧法向力目标，
再复用现有 `NormalForceController` 完成接触建立与力跟踪。它用于回答“如果摩擦系数已知，目标力
应如何随负载变化”这一基线问题。

当前实现是 **oracle 基线**：任务配置直接提供 MuJoCo 场景使用的真值摩擦系数 `μ`，调度器也读取
同一个真值。它没有估计摩擦系数，不能作为“在线摩擦估计已经实现”的证据。盲估计与估计值调度已由
独立的 [`friction-estimate`](friction-estimation.md) 实验实现；两个入口刻意分开，以便直接比较 oracle
上界和摩擦估计误差、收敛速度及失效保护。

## 运行方法

仅重力保持 50 g 方块：

<pre><code class="language-bash">
uv run pgt run force-schedule \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_scheduling/gravity_hold.yaml
</code></pre>

沿重力方向把附加载荷从 0 线性增加到 2 N，模拟容器逐渐注水：

<pre><code class="language-bash">
uv run pgt run force-schedule \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_scheduling/dynamic_filling.yaml
</code></pre>

两个任务都先低速闭合并确认双侧接触，稳定后撤去临时支撑，随后才开始目标力调度和滑移评价。

## 调度公式与力语义

设 `D` 为两侧摩擦共同抵抗的切向合力需求，`μ` 为滑动摩擦系数，`γ` 为不小于 1 的安全系数。
对称二指抓取中，两侧理想摩擦容量为 `2μf_n`，因此平均单侧目标法向力为：

\[
f_{ref}=\operatorname{clip}\!\left(\frac{\gamma D}{2\mu},\ f_{min},\ f_{max}\right).
\]

实现还使用 `friction_floor` 防止 `μ` 过小时分母退化，并对相邻控制周期的目标施加对称变化率限制：

\[
\left|f_{ref,k}-f_{ref,k-1}\right|\leq \dot f_{max}\Delta t.
\]

这里的 `f_ref` 始终是**平均单侧法向力**，不是左右两侧力之和。当前场景的切向需求为方块重力
与沿重力方向附加载荷的合力大小；动态注水任务不会改变物体质量，而是在物体质心施加 0→2 N
的附加力。

## 标准场景

| 任务 | 场景 | 目标力行为 |
| --- | --- | --- |
| `gravity_hold.yaml` | 50 g 方块，仅抵抗重力 | 理论值略低于下限，最终由 `min_force_n=0.5 N` 限制 |
| `dynamic_filling.yaml` | 重力之外增加 0→2 N 向下外力 | 目标随载荷上升，最终约为 `2.335 N/侧` |

标准任务使用 `μ=0.8`、`γ=1.5`、`f_min=0.5 N`、`f_max=8 N` 和
`max_force_rate_n_s=1 N/s`。任务文件还可配置接近阶段、物体质量、接触材料、载荷 waypoint、
控制周期、滑移阈值和力跟踪 RMSE 门限；未知字段和非法范围会在仿真开始前被拒绝。

## `noslip_iterations` 的使用边界

两个标准任务都在 task 中显式设置：

<pre><code class="language-yaml">
solver:
  noslip_iterations: 5
</code></pre>

该后处理用于抑制软约束求解器在摩擦锥内的长时数值爬移，使准静态保持结果不被累计数值误差主导。
它不会提高摩擦系数，也不应掩盖物理上不足的夹持力：把动态注水任务的最大目标力限制为
`0.5 N/侧` 时，物体仍会滑落，摩擦裕量也会变为负值。

## 输出与指标

每次运行写入独占目录：

```text
outputs/custom_parallel_gripper/force-schedule/<UTC timestamp>-<id>/
├── manifest.json
├── profile.yaml
├── task.yaml
├── effective_parameters.json
├── trace.csv
├── metrics.json
├── plot.png
└── plot.pdf
```

`effective_parameters.json` 会把 `scheduler_kind` 记录为 `oracle`。`trace.csv` 包含真值摩擦系数、
切向需求、原始/受限目标力、变化率、触觉测量、摩擦裕量、摩擦利用率和物体切向位移；图像对应展示
载荷、目标/测量力、摩擦裕量和滑移。`metrics.json` 的主要验收量包括：

- `force_tracking_rmse_n`：平均单侧目标力的跟踪 RMSE；
- `max_tangential_displacement_m`：撤去支撑后的最大切向位移；
- `mean_target_force_n`、`peak_target_force_n`、`final_target_force_n`：调度目标摘要；
- `minimum_friction_margin_n`、`peak_friction_utilization`：摩擦容量诊断；
- `target_force_maximum_ratio`、`target_force_rate_limited_ratio`：目标上限和变化率限幅占比；
- `simulation_stable`、`slip_passed`、`force_tracking_passed`：分项验收结果。

## 当前验证结果

在默认自研夹爪 profile、50 g 方块、`hard` 接触和 `μ=0.8` 下，当前标准任务结果为：

| 任务 | 力跟踪 RMSE | 最大切向位移 | 最终目标力 |
| --- | ---: | ---: | ---: |
| 仅重力保持 | 约 `0.009 N` | 约 `0.009 mm` | `0.500 N/侧` |
| 动态注水 | 约 `0.030 N` | 约 `0.009 mm` | 约 `2.335 N/侧` |

这些数值验证了“已知 `μ` 时的负载—目标力—力控”闭环基线，不代表未知材料下的摩擦估计性能。
调度器的纯算法位于 `src/parallel_gripper_tactile/force_scheduling.py`，场景、任务 schema、指标和绘图
位于 `src/parallel_gripper_tactile/experiments/force_scheduling.py`。

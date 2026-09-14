# Oracle 抓取目标力调度

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
`trace.csv`、`metrics.json`、`plot.png` 与 manifest；有效参数声明 `scheduler_kind=oracle`。

trace 记录切向需求、真值摩擦、原始／受限目标、触觉力、摩擦裕量与位移。验收同时检查仿真稳定、
力跟踪 RMSE 和撤支撑后最大切向位移；目标摘要、摩擦利用率与限幅占比用于解释失败。

## 标准场景参考结果

在默认 DMgripper profile、50 g 方块、`hard` 接触和 `μ=0.8` 下，已有标准任务记录为：

| 任务 | 力跟踪 RMSE | 最大切向位移 | 最终目标力 |
| --- | ---: | ---: | ---: |
| 仅重力保持 | 约 `0.009 N` | 约 `0.009 mm` | `0.500 N/侧` |
| 动态注水 | 约 `0.030 N` | 约 `0.009 mm` | 约 `2.335 N/侧` |

这些数值验证了“已知 `μ` 时的负载—目标力—力控”闭环基线，不代表未知材料下的摩擦估计性能。
调度器的纯算法位于 `src/parallel_gripper_tactile/force_scheduling.py`，场景、任务 schema、指标和绘图
位于 `src/parallel_gripper_tactile/experiments/force_scheduling.py`。

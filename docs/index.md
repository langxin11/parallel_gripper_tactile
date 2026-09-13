# 🧭 平行夹爪指尖触觉仿真

本工具包使用经过校验的 YAML profile 与 `pgt` 命令行接口。新用户可先完成下面的快速开始，
再查看[常用工作流](workflows.md)；无需先阅读 CAD 或控制模型文档。

```bash
uv sync --all-packages --locked
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt run demo --set model=robotiq_2f85/touch_grid_3x3
uv run pgt runs list
```

专题阅读入口：

1. [动态目标力跟踪](force-tracking.md)：运行两阶段基准并理解接触状态；
2. [Hydra 科研配置与实验编排](research-configuration.md)：组合配置、审阅计划并执行正式研究；
3. [Oracle 抓取目标力调度](force-scheduling.md)：根据已知摩擦系数和切向载荷生成目标力；
4. [微滑移探测与保守摩擦估计](friction-estimation.md)：触觉检测、估计边界与当前结果；
5. [切向扰动下的触觉增力](tangential-disturbance.md)：稳定夹持后以触觉变化有限增力的仿真边界；
6. [Robotiq 2F-85 离散力控制](discrete-force-control.md)：用 `ΔF_tick` 实现整数控制；
7. [控制算法对比与消融](control-comparison-ablation.md)：在一致任务下比较结果。

模型与接口背景：

- [触觉读数约定](tactile-conventions.md)：统一仿真与实机到 `F_L`、`F_R`、`f_n` 的输入语义；
- [Onshape to Robot：曲柄滑块夹爪 MJCF 导出](onshape-export-upgrade.md)：CAD 命名、闭环与导出验收；
- [曲柄滑块力控模型](crank-slider-force-control.md)：`f_n`、`k_pair`、雅可比和限幅。

工程状态与电机/执行器基线见[DM_Gripper 配置基线与状态](custom-gripper-next-phase.md)。
文献指南、近期实验与研究合集的目录及编译入口见[科研报告的组织与维护](reports.md)。
配置目录与目的导向研究的设计、验收和迁移记录见[配置重构 Goal](configuration-refactor-goal.md)。

下一阶段的目录整理、分夹爪控制核与真机接入见[纯 Python 真机联调计划](python-hardware-roadmap.md)。
基础力跟踪与动态增力已收敛为通用抓取实验，操作方法见[DMgripper 通用抓取实验](dmgripper-experiments.md)；重构过程与验收记录见[DMgripper 通用抓取实验重构实施计划](dmgripper-unified-experiments-plan.md)。

<pre><code class="language-bash">
uv sync
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt validate configs/dm_gripper.yaml
uv run pgt run demo
uv run pgt run force-track --set task=force_tracking/default_waypoints
uv run pgt run force-schedule --experiment dm_gripper/force_scheduling_gravity_hold
uv run pgt run tangential-disturbance --experiment dm_gripper/tangential_disturbance
uv run pgt run friction-estimate --experiment dm_gripper/friction_estimation_nominal
uv run pgt run discrete-force --experiment robotiq_2f85/discrete_force
</code></pre>

参见[常用工作流](workflows.md)、[动态目标力跟踪](force-tracking.md)、
[Hydra 科研配置与实验编排](research-configuration.md)、
[Oracle 抓取目标力调度](force-scheduling.md)、
[微滑移探测与保守摩擦估计](friction-estimation.md)、
[切向扰动下的触觉增力](tangential-disturbance.md)、
[Robotiq 2F-85 离散力控制](discrete-force-control.md)、
[控制算法对比与消融](control-comparison-ablation.md)与[项目架构](architecture.md)。

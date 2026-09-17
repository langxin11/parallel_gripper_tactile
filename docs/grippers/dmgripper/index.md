# DMgripper

DMgripper 使用单电机驱动曲柄滑块机构，左右 Pillar 提供接触力观测。
仿真与真机共用 `dm_grasp_core` 的纯计算能力，各自管理运行时与设备生命周期。

## 先选择任务

| 任务 | 阅读入口 |
| --- | --- |
| 选择模型或检查碰撞变体 | [模型资产](#model-selection)、[Pillar 触觉模型](#tactile-model) |
| 维护 CAD 与模型资产 | [资产维护入口](#asset-source) |
| 理解关节角、开度、雅可比与 MIT 请求 | [控制基础与共享核](../../dm-shared-control.md) |
| 选择仿真平台与执行器限幅 | [配置与执行器基线](../../dm-shared-control.md#actuator-baseline) |
| 跟踪目标力、选择 PID／ADRC／导纳 | [动态目标力跟踪](../../force-tracking.md)、[共享控制核](../../dm-shared-control.md) |
| 根据载荷和触觉反馈调整抓力 | [自适应抓取](../../adaptive-grasping.md) |
| 运行真机任务与检查设备状态 | [通用抓取实验](../../dmgripper-experiments.md) |
| 复现模型、刚度或控制器研究 | [模型验证、控制对比与消融](../../control-comparison-ablation.md) |

## 最小仿真与计划

```bash
uv run python scripts/research/run.py \
  experiment=dm_gripper/force_tracking_admittance execution=plan
uv run pgt run force-track --experiment dm_gripper/force_tracking_admittance
```

第一条命令校验组合并生成计划；第二条执行一次导纳力跟踪。
查看模型时使用 `uv run pgt view grasp --set model=dm_gripper/height_spheres`。

## 能力分层

| 层次 | 当前实现 | 进一步阅读 |
| --- | --- | --- |
| 资产 | MJCF、网格、闭环约束、Pillar 碰撞变体 | [模型资产](#model-selection) |
| 测量 | 按接触几何聚合逐 taxel 力，统一局部坐标与法向符号 | [触觉接口与读取](../../tactile-conventions.md) |
| 控制 | MIT 请求、运动学、PID、LADRC、导纳与刚度估计 | [共享控制核](../../dm-shared-control.md) |
| 运行时 | 仿真物理循环与真机设备会话分别拥有生命周期 | [项目架构](../../architecture.md) |
| 研究 | 领域 protocol 生成条件矩阵，计划与执行共享 `StudyPlan` | [科研配置](../../research-configuration.md) |

!!! warning "统一反馈力定义"

    控制主量为平均单侧法向力 `f_n=(F_L+F_R)/2`。
    比较配置、曲线与刚度时，先确认没有把它与双侧总力混用。

## 资产维护入口 {#asset-source}

模型维护从仓库中的 `assets/grippers/dm_gripper/README.md` 开始。
该文件链接到同目录的 Onshape 导出、后处理与验收教程；完整流程随资产维护。

<!-- 模型与触觉正文随资产维护。 -->

--8<-- "assets/grippers/dm_gripper/README.md:content"

[onshape]: #asset-source
[tactile]: #tactile-model
[dm-guide]: #
[validation]: ../../control-comparison-ablation.md#collision-geometry-conclusions
[motor-model]: motor-model.md
[tactile-contract]: ../../tactile-conventions.md
[tx-tactile-contract]: ../../tactile-conventions.md
[tx-onshape]: #asset-source
[tx-validation]: ../../control-comparison-ablation.md#collision-geometry-conclusions

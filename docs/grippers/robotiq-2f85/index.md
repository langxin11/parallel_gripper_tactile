# Robotiq 2F-85

本入口连接 Robotiq 的模型、触觉与整数命令控制。
仿真使用 tendon 执行器；离散力控制在 `0～255` 整数命令空间运行。

## 先选择任务

| 任务 | 阅读入口 |
| --- | --- |
| 查看基础模型、触觉变体或重新生成资产 | [模型资产](#model-selection) |
| 理解离散 taxel、插件通道与 FOV | [触觉模型](#tactile-model) |
| 对齐左右力、坐标和日志语义 | [触觉接口与读取](../../tactile-conventions.md) |
| 运行整数命令力跟踪与消融研究 | [离散力控制](../../discrete-force-control.md) |
| 组合实验配置与检查运行产物 | [科研配置](../../research-configuration.md)、[常用工作流](../../workflows.md) |

## 最小仿真

```bash
uv run pgt run demo --set model=robotiq_2f85/touch_grid_3x3
uv run pgt run discrete-force --experiment robotiq_2f85/discrete_force
```

前者读取触觉，后者运行离散力控制；每次运行的目录由终端 `Run:` 行给出。
完整研究的矩阵、运行命令与指标见[离散力控制](../../discrete-force-control.md)。

## 控制能力与硬件边界

| 层次 | 当前实现 | 验证边界 |
| --- | --- | --- |
| 仿真模型 | 离散 force sensor 与 `touch_grid` 变体 | 按所选模型确认触觉覆盖与读数 |
| 共享控制核 | `robotiq_grasp_core`：稳定判定、tick 增益、HOLD 与整数动作决策 | 不依赖 DM 控制核 |
| 仿真实验 | 量化 PI、固定步长、自适应死区、预测和动态步长变体 | 研究当前 MuJoCo 条件下的法向力稳定 |
| 硬件适配 | `robotiq_hardware`：位置命令后端与单步控制衔接 | 测试使用 fake backend，尚未完成真实设备验证 |

!!! note "接入真实设备前"

    当前硬件包要求调用方管理连接、激活与生命周期。
    包内 `packages/robotiq_hardware/README.md` 是后端接入说明；仿真结果不证明设备已完成标定。

<!-- 模型与触觉正文随资产维护。 -->

--8<-- "assets/grippers/robotiq_2f85/README.md:content"

[tactile]: #tactile-model
[robotiq-guide]: #
[tx-tactile-contract]: ../../tactile-conventions.md

# 平行夹爪触觉仿真与力控实验

本项目提供 MuJoCo 夹爪模型、统一触觉接口、力控制器和可复现的实验编排。
先运行最小示例，再按所使用的夹爪进入对应文档。

## 快速开始

在仓库根目录执行。需要 Python 3.12 或更新版本以及 `uv`。

```bash
uv sync --all-packages --locked
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt run demo --set model=robotiq_2f85/touch_grid_3x3
uv run pgt runs list
```

演示默认无界面；终端的 `Run:` 行给出 `outputs/` 下的独占目录。
`trace.csv` 保存本次演示的最终触觉力，`manifest.json` 登记输入与产物。

!!! tip "接下来选择夹爪"

    Robotiq 的整数命令与 DMgripper 的 MIT 请求具有不同语义。
    从下面对应入口选择模型、控制器和实验配置。

## 夹爪与模型

| 夹爪 | 控制与模型特点 | 阅读入口 |
| --- | --- | --- |
| **Robotiq 2F-85** | tendon 执行器、离散 taxel／`touch_grid`、整数命令力控制 | [模型、控制与实验](grippers/robotiq-2f85/index.md) |
| **DMgripper** | 曲柄滑块机构、Pillar 接触力、MIT 内环与法向力外环 | [模型、控制与实验](grippers/dmgripper/index.md) |

维护 CAD、碰撞几何或模型生成流程时，进入[资产维护约定](architecture.md#asset-maintenance)。

## 按任务查找

| 你要完成的任务 | 阅读入口 |
| --- | --- |
| 运行一次实验、查看模型或管理产物 | [常用工作流](workflows.md) |
| 组合配置、检查计划、执行正式研究 | [科研配置与实验编排](research-configuration.md) |
| 理解触觉数组、单位、坐标和反馈力 | [触觉接口与读取](tactile-conventions.md) |
| 跟踪目标力或调整抓力 | [DM 力跟踪](force-tracking.md)、[自适应抓取](adaptive-grasping.md)、[Robotiq 离散力控制](discrete-force-control.md) |
| 评价模型、刚度估计器或控制器 | [模型验证、控制对比与消融](control-comparison-ablation.md) |
| 阅读实验结果与维护报告 | [科研报告](reports.md) |
| 修改实现与验证改动 | [项目架构](architecture.md)、[代码规范](coding-conventions.md)、[测试策略](testing.md) |

运行结果与方法的适用条件由对应专题说明；模型能够加载不等于实机性能已经得到验证。

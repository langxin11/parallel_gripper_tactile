# 🧭 平行夹爪指尖触觉仿真

本工具包使用经过校验的 YAML profile 与 `pgt` 命令行接口。新用户可先完成下面的快速开始，
再查看[常用工作流](workflows.md)；无需先阅读 CAD 或控制模型文档。

```bash
uv sync --all-packages --locked
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt run demo --set model=robotiq_2f85/touch_grid_3x3
uv run pgt runs list
```

## 按任务阅读

| 你要做什么 | 阅读入口 |
| --- | --- |
| 运行演示、实验或正式研究 | [常用工作流](workflows.md) |
| 理解配置组合、计划校验和研究产物 | [科研配置与实验编排](research-configuration.md) |
| 跟踪给定目标力、比较控制器 | [动态目标力跟踪](force-tracking.md)、[控制算法对比与消融](control-comparison-ablation.md) |
| 根据载荷与触觉反馈调整抓力 | [自适应抓取](adaptive-grasping.md) |
| 使用 Robotiq 整数命令控制力 | [Robotiq 离散力控制](discrete-force-control.md) |
| 配置或操作 DMgripper | [配置与执行器基线](dmgripper-configuration.md)、[通用抓取实验](dmgripper-experiments.md) |
| 阅读实验结果或维护报告 | [科研报告的组织与维护](reports.md) |

## 模型与开发背景

- [触觉读数约定](tactile-conventions.md)：双侧法向力、切向力与坐标语义。
- [Onshape 导出](onshape-export-upgrade.md)、[曲柄滑块力控模型](crank-slider-force-control.md)：模型来源与力学关系。
- [项目架构](architecture.md)、[DMgripper 共享控制核](dm-shared-control.md)：模块职责与依赖边界。
- [代码与注释规范](coding-conventions.md)、[测试策略](testing.md)：修改与验证约定。

# 🤖 Parallel Gripper Tactile

MuJoCo 二指平行夹爪触觉仿真工具包。它使用经过 schema 校验的 YAML profile、统一的 `pgt` 命令行接口，
并为每次实验保存可复现的运行产物。

## 安装与最小运行

```bash
uv sync --all-packages --locked
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt run demo --set model=robotiq_2f85/touch_grid_3x3
```

运行默认无界面。结果会写入 `outputs/` 下的独占目录，可用以下命令查看产物和交互场景：

```bash
uv run pgt runs list
uv run pgt view grasp
```

终端的 `Run:` 行给出本次目录；这个最小演示的 `trace.csv` 保存最终触觉力，`manifest.json` 登记输入与产物。
需要完整力跟踪曲线时运行下面的 `force-track`，再打开其目录中的 `plots/tracking.png` 并查看 `metrics.json`。

常用单次实验还包括：

```bash
uv run pgt run grasp --video
uv run pgt run force-track --set task=force_tracking/step
uv run pgt run force-schedule --experiment dm_gripper/force_scheduling_oracle
uv run pgt run friction-estimate --experiment dm_gripper/friction_estimation
uv run pgt run discrete-force --experiment robotiq_2f85/discrete_force
```

资源生成与诊断命令包括 `pgt assets generate-taxels`、`pgt assets generate-touch-grid`、
`pgt compare tactile`、`pgt compare contact` 和 `pgt view taxels`；参数与示例见[常用工作流](docs/workflows.md)。

`pgt configs list [GROUP]` 用于查看可用配置组（如 `experiment`、`controller`、`scheduler`、`task`、`research`），
也可用 `--search TEXT` 按文本搜索：

```bash
uv run pgt configs list experiment
uv run pgt configs list --search friction
```

## CLI 与 Hydra 的边界

`pgt` 面向演示、模型查看、设备检查和单次预设运行；`scripts/research` 下的 Hydra 入口面向组合探索和
正式 study。两条入口共享 Python runner 与运行产物格式，但科研矩阵、计划／执行模式和阶段谱系由 Hydra
及 study 管理。研究进度会在终端自动显示已处理／总数、科学失败、执行异常及 study 目录；配置组、覆盖规则、
产物结构和恢复语义见[Hydra 科研配置与实验编排](docs/research-configuration.md)。

计划模式先校验领域组合并写出计划，确认后再执行：

```bash
uv run python scripts/research/study.py \
  research=force_controller_selection/study
uv run python scripts/research/study.py \
  research=force_controller_selection/study execution=study_run
```

正式研究的矩阵、执行顺序、`refinement`／`confirmation` 流程和数值结论统一维护在[常用工作流](docs/workflows.md)
及各专题文档中；study 不会自动回写候选参数。

## 按夹爪进入

| 夹爪 | 模型、控制与实验 | 资产维护 |
| --- | --- | --- |
| **Robotiq 2F-85** | [使用入口](docs/grippers/robotiq-2f85/index.md)：触觉变体、整数命令控制与离散力研究 | [模型资产](assets/grippers/robotiq_2f85/README.md) |
| **DMgripper** | [使用入口](docs/grippers/dmgripper/index.md)：Pillar 触觉、MIT 控制、力跟踪与真机任务 | [模型资产](assets/grippers/dm_gripper/README.md) |

## 文档导航

| 你要完成的任务 | 阅读入口 |
| --- | --- |
| 从第一次运行开始 | [文档首页](docs/index.md)、[常用工作流](docs/workflows.md) |
| 理解接口与控制原理 | [公共触觉接口](docs/tactile-conventions.md)、[DM 控制基础](docs/dm-shared-control.md)、[Robotiq 离散力控制](docs/discrete-force-control.md) |
| 实验配置与结果复现 | [科研配置](docs/research-configuration.md)、[控制对比与消融](docs/control-comparison-ablation.md)、[科研报告](docs/reports.md) |
| 维护 CAD、MJCF 与触觉模型 | [资产维护约定](assets/README.md) |

## 开发入口

架构、测试、配置与报告规范分别见[项目架构](docs/architecture.md)、[测试规范](docs/testing.md)、
[研究配置](docs/research-configuration.md)、[代码与注释规范](docs/coding-conventions.md)和
[CONTRIBUTING](CONTRIBUTING.md)。

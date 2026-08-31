# Parallel Gripper Tactile

MuJoCo 二指平行夹爪触觉仿真：使用通过 schema 校验的 YAML profile、单一 `pgt` 命令，以及
可复现的逐次运行产物（run artifacts）。

## 快速开始

```bash
uv sync
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt validate configs/custom_parallel_gripper.yaml
uv run pgt run demo --profile configs/robotiq_2f85.yaml
uv run pgt run grasp --profile configs/custom_parallel_gripper.yaml
uv run pgt run force-track --profile configs/custom_parallel_gripper.yaml --task configs/force_tracking/default_waypoints.yaml
# 自研夹爪可选 soft / medium / hard 显式触觉接触材料（默认 hard）
uv run pgt run grasp --profile configs/custom_parallel_gripper.yaml --object-material soft
uv run pgt run force-track --profile configs/custom_parallel_gripper.yaml --task configs/force_tracking/default_waypoints.yaml --object-material medium
uv run pgt run force-track --profile configs/custom_parallel_gripper.yaml --task configs/force_tracking/default_waypoints.yaml --controller-variant full --object-material hard --sensor-noise-seed 0
```

Profile 仅使用 YAML。它们是不可变的 Pydantic v2 模型：未知字段、非法控制限幅、空/多文档输入、
未解析的模型路径都会在仿真开始前失败。相对模型路径以 profile 所在目录为基准解析。

## 命令

```text
pgt validate PROFILE
pgt assets generate-taxels [--shape sphere|box]
pgt assets generate-touch-grid
pgt assets prepare-onshape INPUT OUTPUT
pgt run demo --profile PROFILE
pgt run grasp --profile PROFILE [--video] [--object-material soft|medium|hard]
pgt run force-track --profile PROFILE --task TASK.yaml [--viewer] [--object-material soft|medium|hard]
pgt compare tactile --left-profile A --right-profile B
pgt compare contact --profile PROFILE
pgt view taxels --profile PROFILE
pgt view grasp --profile PROFILE [--object-material soft|medium|hard]
pgt runs list
pgt runs clean (--older-than-days N | --all | --cache) [--apply]
```

仿真默认无界面（headless）。`pgt run force-track --viewer` 会在运行 waypoint
目标力跟踪任务时同步打开 MuJoCo GUI；`pgt view` 会打开静态交互检查场景。Typer 通过
`pgt --install-completion` 提供 shell 补全。

`pgt run grasp` 和 `force-track` 支持 `--run-prefix` 与
`--run-suffix`。它们会保留自动生成的 UTC 时间戳和短 ID，例如
`trial-20260830T104726Z-6dc7385b`；原有 `--run-name` 仍用于指定完整、不可改写的目录名，
不能与前缀或后缀同时使用。

力控消融的四个跟踪阶段变体为：

| 变体 | PID | 刚度位置前馈 | 力矩前馈 |
| --- | --- | --- | --- |
| `pid-only` | 开 | 关 | 关 |
| `pid-torque-ff` | 开 | 关 | 开 |
| `pid-stiffness-ff` | 开 | 开 | 关 |
| `full` | 开 | 开 | 开 |

所有变体共享相同的接近阶段和 waypoint 任务。

## Research studies

`pgt` 提供确定参数下的单次实验；`scripts/experiments` 保存 controller、材料和重复次数组成的
多次科研 protocol。运行力跟踪消融研究：

```bash
uv run python scripts/experiments/force_tracking_ablation.py \
    --config configs/studies/force_tracking_ablation.yaml
```

该 study 直接调用 Python runner，而非通过子进程调用 CLI。它会在 `outputs/studies` 创建独立父目录，
保存 study 配置、逐次结果、聚合统计和每个子 run 的可复现工件。

## Profiles

| Profile | 控制 | 触觉后端 |
| --- | --- | --- |
| `robotiq_2f85.yaml` | position | `force_sensor` |
| `robotiq_2f85_box.yaml` | position | box 的 `force_sensor` |
| `robotiq_2f85_touch_grid.yaml` | position | `touch_grid` |
| `custom_parallel_gripper.yaml` | MIT 力矩 + 法向力外环 | `contact_geom` |

自研夹爪的 Pillars 有意使用等效软接触，而非独立的可变形硅胶体：
`solref="-1200 -10"`、`solimp="0.75 0.95 0.0025 0.5 2"`。当前仿真接触响应约为
`1200 N/m`；产品量程换算只能作为设计背景，不能替代仓库模型参数。

## 运行产物

每个产出结果（result-producing）的命令都会创建独占目录：

```text
outputs/<profile>/<experiment>/<UTC timestamp>-<id>/
├── manifest.json
├── profile.yaml
├── trace.csv
├── metrics.json
├── plot.png
└── video.mp4
```

`manifest.json` 只列出实际产出的产物，并记录 profile 哈希、Git 状态、依赖版本、参数与创建时间。
`pgt runs clean` 默认只预览将删除的目标；需要删除时加 `--apply`。

## 架构

```text
config → tactile / scenes / simulation → experiments / analysis / io → CLI
```

`SimulationSession` 独占 `mj_step`；控制时钟与采样时钟相互独立。所有触觉读取器在不可变坐标系中
返回局部 `(3, rows, cols)` 力数组，压缩以正的 `Fz` 表示。

## 接触模型决策

实验性的 Drake/Hydroelastic 桥接已从主线移除。它未经标定、对本模型无材料准确性收益、存在穿透限制，
且每次求值约 0.510 ms——约为 mesh-SDF 比较路径的 30 倍。原生 MuJoCo 接触与 mesh-SDF 仍可用于
A/B 实验。

## 开发

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest
uv run ruff check .
uv run ruff format --check .
```

见[代码与注释规范](docs/coding-conventions.md)与 [CONTRIBUTING](CONTRIBUTING.md)。

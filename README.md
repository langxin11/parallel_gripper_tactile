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
pgt run grasp --profile PROFILE [--video]
pgt compare tactile --left-profile A --right-profile B
pgt compare contact --profile PROFILE
pgt view taxels --profile PROFILE
pgt view grasp --profile PROFILE
pgt runs list
pgt runs clean (--older-than-days N | --all | --cache) [--apply]
```

仿真默认无界面（headless）。`pgt view` 会打开显式的 MuJoCo GUI。Typer 通过
`pgt --install-completion` 提供 shell 补全。

## Profiles

| Profile | 控制 | 触觉后端 |
| --- | --- | --- |
| `robotiq_2f85.yaml` | position | `force_sensor` |
| `robotiq_2f85_box.yaml` | position | box 的 `force_sensor` |
| `robotiq_2f85_touch_grid.yaml` | position | `touch_grid` |
| `custom_parallel_gripper.yaml` | MIT 力矩 + 法向力外环 | `contact_geom` |

自研夹爪的 Pillars 有意使用等效软接触，而非独立的可变形硅胶体：
`solref="-6000 -10"`、`solimp="0.75 0.95 0.0025 0.5 2"`。这表示由单根 Pillar
15 N / 2.5 mm 满量程换算得到的约 `6 kN/m` 名义法向刚度，并带有 2.5 mm 的柔顺过渡。

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

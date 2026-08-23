# 平行夹爪指尖触觉仿真

面向 MuJoCo 的多平行夹爪触觉资产与实验工具。项目同时保留 Robotiq 2F-85
参考模型和自研曲柄滑块平行夹爪，并通过类型化 profile 统一模型路径、执行器、
控制范围、安装位姿与触觉阵列命名。

## 快速开始

```bash
uv sync --all-groups
uv run pgt-check configs/robotiq_2f85.toml
uv run pgt-check configs/custom_parallel_gripper.toml
uv run pytest
```

查看自研夹爪：

```bash
uv run -m mujoco.viewer \
  --mjcf assets/grippers/custom_parallel_gripper/scene.xml
```

现有 Robotiq taxel、touch-grid、抓取、扰动、Rerun 和视频脚本继续可用：

```bash
uv run scripts/run_cube_grasp_demo.py --auto-close
uv run scripts/run_touch_grid_demo.py --auto-close
uv run scripts/compare_tactile_models.py --disturbance
uv run scripts/record_disturbance_video.py
```

自研夹爪的 `base` 是预留给转接法兰（未来可连接 FR3 等机械臂）的安装根。下面的验收
场景仅在运行时将该根刚性固定到 profile 的 mount frame，不会修改夹爪机构或 CAD：

```bash
uv run scripts/run_custom_grasp_validation.py \
  --output-csv outputs/custom_gripper/validation/custom_gripper_grasp.csv

uv run scripts/record_custom_grasp_video.py \
  --output outputs/custom_gripper/disturbance_video/custom_gripper_disturbance.mp4
```

运行下面的交互式查看器可直接检查 profile 中 `mount.pos/quat` 施加后的整体朝向；
它加载与验收完全相同的物块、支撑板和安装 frame。使用 `--closed` 可查看闭合姿态：

```bash
uv run scripts/view_custom_grasp_scene.py
uv run scripts/view_custom_grasp_scene.py --closed
uv run scripts/view_custom_grasp_scene.py --frame body
```

夹爪通过 mount frame 横向安装：滑轨局部 X 与本体局部 Z 均平行地面。按自研夹爪实际指尖
几何，测试块为 `6 × 25 × 25 mm`，在 YZ 接触面上形成 `25 × 25 mm`（大于 `24 × 24 mm`）的接触面。
它在水平侧向夹持姿态下依次验证：撤去支撑板后的无外力静态保持，以及与 Robotiq 相同的
`1.0 s` 闭合、`0.5 s` 带支撑稳定、`0.5 s` 无支撑稳定、`5 N / 2 Hz / 1.0 s` 世界 Y 向
切向扰动和 `0.5 s` 恢复。两项默认均以 YZ 接触面内位移不超过 `2 mm` 为通过条件。

## 支持的夹爪

| Profile | 执行器 | 触觉表示 | 状态 |
| --- | --- | --- | --- |
| `robotiq_2f85.toml` | `fingers_actuator` | 3×3 force taxel | 参考基线 |
| `custom_parallel_gripper.toml` | `gripper_drive` | 左右各 3×3 Pillars 接触 geom | 接入中 |

自研夹爪的 18 个 Pillars STL 就是实际触觉测量位置和唯一主动碰撞几何，按每侧
滑块局部坐标命名为 `left/right_taxel_geom_00` 到 `22`。它们不是叠加在指尖上的
近似球体。

Pillar 碰撞采用等效软接触：`solref="-6000 -10"`（对应法向标称刚度约 `6 kN/m`）与
`solimp="0.75 0.95 0.0025 0.5 2"`。该值由单柱 `15 N / 2.5 mm` 满量程换算，并保留
`2.5 mm` 的柔顺过渡；它是接触层的等效模型，不等同于具有独立三轴弹性自由度的实体硅胶柱。

## 项目结构

```text
assets/
  grippers/
    custom_parallel_gripper/   Onshape 导出的自研夹爪与 STL
    robotiq_2f85/              Robotiq 参考资产
  objects/                     被抓物体
  scenes/                      公共实验场景
configs/                       夹爪 profile
src/parallel_gripper_tactile/  可复用核心库与验证 CLI
scripts/                       生成、演示、记录和迁移工具
tests/                         模型契约与数值工具测试
docs/                          架构、触觉约定和工作流
outputs/
  robotiq/                      Robotiq 实验结果（taxel/touch_grid 演示、比较、扰动视频）
  custom_gripper/               自研夹爪结果（验收、扰动视频、参数扫描实验）
```

`outputs/` 按夹爪与实验类别分子目录，脚本默认输出路径已指向对应子目录；
扫描类实验请把 `--output-csv`/`--output` 显式指向 `experiments/` 下的文件。

新夹爪应通过 profile 接入，不应在公共脚本中新增 `data.ctrl[0]`、固定控制范围或
特定模型路径。详细设计见[项目架构](docs/architecture.md)。

## 代码与注释规范

- 注释语言：`src/` 公共库的 docstring 使用英文；`scripts/` 与 `tests/` 的
  docstring 与行内注释使用中文（与 README、docs、CLI 帮助和终端输出一致）。
  公共库与脚本边界上的中英混用视为规范违规。
- docstring 风格：Google 约定（pyproject 中已配置 ruff `D` 规则）。`scripts/`
  与 `tests/` 仅豁免与英文语法/ASCII 标点绑定的 5 条规则
  （`D400`/`D401`/`D403`/`D404`/`D415`）——中文 docstring 的祈使语气、
  首词大写与全角句号 `。` 会触发这 5 条误报；其余格式类 D 规则（缺
  docstring、空行、引号、多行摘要位置等）对中文同样生效，由 ruff 自动强制。
- 注释必须与实现同步：描述 API 行为（如读取初始状态还是仿真状态）、渲染
  原语（如 `mjv_connector`）时，修改实现后必须同步更新 docstring。
- 实验时序与扰动波形统一使用 `parallel_gripper_tactile.DisturbanceProtocol`；
  新增实验脚本不得再自建协议类。共享的渲染/编码工具位于
  `parallel_gripper_tactile.video`。

## Onshape 模型维护

自研夹爪的源模型仍由 Onshape 维护。重新导出后不要直接覆盖经过验证的资产；先导出
到临时目录，再运行：

```bash
uv run scripts/prepare_onshape_export.py RAW.xml PREPARED.xml
uv run pgt-check configs/custom_parallel_gripper.toml
```

完整的配合命名、质量、闭环、碰撞、力矩与导出升级清单见
[Onshape 导出与升级建议](docs/onshape-export-upgrade.md)。

从当前模型走向完整 Pillars 触觉反馈实验的实施顺序、接口设计、测试和验收条件见
[自研平行夹爪下一阶段实施路线图](docs/custom-gripper-next-phase.md)。

Robotiq 触觉模型的坐标系和符号约定见[触觉读数约定](docs/tactile-conventions.md)，
常用实验命令见[工作流](docs/workflows.md)。

## 许可证

本项目采用 [Apache License 2.0](LICENSE)。

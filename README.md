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

## 支持的夹爪

| Profile | 执行器 | 触觉表示 | 状态 |
| --- | --- | --- | --- |
| `robotiq_2f85.toml` | `fingers_actuator` | 3×3 force taxel | 参考基线 |
| `custom_parallel_gripper.toml` | `gripper_drive` | 左右各 3×3 Pillars 接触 geom | 接入中 |

自研夹爪的 18 个 Pillars STL 就是实际触觉测量位置和唯一主动碰撞几何，按每侧
滑块局部坐标命名为 `left/right_taxel_geom_00` 到 `22`。它们不是叠加在指尖上的
近似球体。

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
```

新夹爪应通过 profile 接入，不应在公共脚本中新增 `data.ctrl[0]`、固定控制范围或
特定模型路径。详细设计见[项目架构](docs/architecture.md)。

## Onshape 模型维护

自研夹爪的源模型仍由 Onshape 维护。重新导出后不要直接覆盖经过验证的资产；先导出
到临时目录，再运行：

```bash
uv run scripts/prepare_onshape_export.py RAW.xml PREPARED.xml
uv run pgt-check configs/custom_parallel_gripper.toml
```

完整的配合命名、质量、闭环、碰撞、力矩与导出升级清单见
[Onshape 导出与升级建议](docs/onshape-export-upgrade.md)。

Robotiq 触觉模型的坐标系和符号约定见[触觉读数约定](docs/tactile-conventions.md)，
常用实验命令见[工作流](docs/workflows.md)。

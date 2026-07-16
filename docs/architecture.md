# 项目结构与场景组合

本仓库将夹爪、环境和被抓物体拆开维护，并在运行时组合成 MuJoCo 模型。

```text
grasp_world.xml     环境：地面、承托板、相机、光照和天空盒
2f85_*.xml          夹爪：机构，以及 taxel 或 touch_grid 触觉模型
target_cube.xml     实体：可自由运动的方块
       │
       └── scripts/grasp_scene.py ── MjSpec.attach() ──> 可运行模型
```

`scripts/grasp_scene.py` 是组合的唯一入口。默认的交互脚本
`view_taxels.py`、`run_cube_grasp_demo.py` 和 `run_touch_grid_demo.py` 都通过它加载模型。
两个抓取演示再将读数整理为统一的 `(3, rows, cols)` 触觉帧，交给
`scripts/recording.py` 同时驱动 CSV 与 Rerun 输出。

## attach 与命名

组合时，夹爪和方块分别使用 `gripper/`、`cube/` 前缀。因此，原始夹爪中名为
`left_taxel_force_00` 的传感器，在默认抓取场景中是
`gripper/left_taxel_force_00`；方块 body 是 `cube/target_cube`。

这样可以安全地添加更多实体而不发生名称冲突。脚本通过
`gripper_name_in_model()` 兼容这两种情况：默认的组合模型使用带前缀名称，传入
`--scene PATH` 的完整外部 MJCF 则可继续使用其原有名称。

## 两种运行方式

默认运行时组合场景：

```bash
uv run scripts/run_cube_grasp_demo.py --auto-close
```

若已有包含环境、夹爪和对象的完整 MJCF，可跳过组合：

```bash
uv run scripts/run_cube_grasp_demo.py --scene path/to/complete_scene.xml
```

用 `--gripper-xml PATH` 替换默认夹爪资产，例如切换到 3×3 `touch_grid`：

```bash
uv run scripts/run_touch_grid_demo.py \
  --gripper-xml assets/robotiq_2f85/2f85_touch_grid_3x3.xml
```

## 资产职责

- `assets/robotiq_2f85/2f85.xml`：上游的基础夹爪；不修改。
- `scripts/generate_taxels_xml.py`：生成 3×3 离散 taxel 的派生夹爪。
- `scripts/generate_touch_grid_xml.py`：生成 `touch_grid` 派生夹爪，可设置网格分辨率。
- `scripts/recording.py`：统一触觉帧、CSV 合力记录以及 Rerun 实时/回放输出。
- `assets/scenes/grasp_world.xml`：默认抓取环境。天空盒和地面纹理由 MuJoCo 程序化生成。
- `assets/objects/target_cube.xml`：默认的自由方块。

生成得到的 `2f85_taxels.xml` 与 `2f85_taxels_box.xml` 不应手工编辑；应修改生成脚本后重新生成。
球形版本保留为默认触觉近似；平面 box 版本用于和 3×3 touch-grid 做同几何 A/B 比较。

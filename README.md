# Robotiq 2F-85 Tactile

独立维护 MuJoCo 中为 Robotiq 2F-85 两个指尖添加触觉传感器的最小资产仓库。

## 内容与边界

- `assets/robotiq_2f85/2f85.xml`：未修改的基础夹爪模型。
- `scripts/generate_taxels_xml.py`：从基础模型生成带 18 个离散 taxel 的派生 MJCF。
- `assets/robotiq_2f85/2f85_taxels.xml`：生成结果；不可手工编辑。
- `assets/robotiq_2f85/2f85_touch_grid.xml`：每侧 32×32、三通道的 `touch_grid` 版本；生成器也支持自定义分辨率。
- `assets/scenes/grasp_world.xml`：抓取环境（地面、薄板、相机与程序化星空）。
- `assets/objects/target_cube.xml`：可自由运动的方块实体。
- `scripts/grasp_scene.py`：使用 `MjSpec.attach()` 在运行时组合环境、夹爪和方块。
- `scripts/check_mjcf.py`：使用 MuJoCo 编译资产的验证工具。
- `scripts/view_taxels.py`：在 MuJoCo viewer 中目视检查两侧 taxel 的位置与尺寸。
- `scripts/report_taxels.py`：将传感器读数按左右两个 3×3 网格输出。
- `scripts/run_cube_grasp_demo.py`：闭合夹爪并打印两侧 taxel 合力。
- `scripts/run_touch_grid_demo.py`：用 OpenCV 将切向力画为箭头、法向压力画为绿→红颜色。

这里的默认实现是每个指尖 3×3 球形 taxel。每个 taxel 用一个球形接触 geom 与一个局部坐标系对齐的 `force` sensor 表示；此外，每侧还提供 `*_pad_force` 和 `*_pad_torque`。

不包含抓取策略、强化学习环境、真实触觉相机驱动或 Shadow Hand 专用的观测包装代码。

## 使用

```bash
uv run scripts/generate_taxels_xml.py
uv run scripts/generate_touch_grid_xml.py
uv run scripts/generate_touch_grid_xml.py --rows 3 --cols 3 \
  --output-xml assets/robotiq_2f85/2f85_touch_grid_3x3.xml
uv run pytest
uv run scripts/check_mjcf.py
uv run scripts/view_taxels.py
uv run scripts/report_taxels.py
uv run scripts/run_cube_grasp_demo.py
uv run scripts/run_touch_grid_demo.py
uv run scripts/run_touch_grid_demo.py \
  --gripper-xml assets/robotiq_2f85/2f85_touch_grid_3x3.xml
```

详细用法、记录和绘图命令见 [常用工作流](docs/workflows.md)。场景资产如何运行时拼接、
名称前缀如何变化见 [项目结构](docs/architecture.md)。关于 taxel / touch_grid 的坐标系、
力的作用对象和 `Fz` 符号，见 [触觉读数约定](docs/tactile-conventions.md)。

## 设计参数

默认 taxel 使用 `solimp="0.90 0.95 0.002"` 和 `solref="0.015 1"`。生成脚本的关键参数为
`TAXEL_GRID`、`TAXEL_RADIUS` 与 `MIDDLE_ROW_TO_TOP_EDGE`，均在 `left_pad` 与 `right_pad`
局部坐标系下定义，单位为米。

# Robotiq 2F-85 Tactile

独立维护 MuJoCo 中为 Robotiq 2F-85 两个指尖添加触觉传感器的最小资产仓库。

## 内容与边界

- `assets/robotiq_2f85/2f85.xml`：未修改的基础夹爪模型。
- `scripts/generate_taxels_xml.py`：从基础模型生成带 18 个离散 taxel 的派生 MJCF。
- `assets/robotiq_2f85/2f85_taxels.xml`：生成结果；不可手工编辑。
- `assets/robotiq_2f85/2f85_touch_grid.xml`：每侧 32×32、三通道的 `touch_grid` 版本。
- `assets/scenes/cube_grasp.xml`：水平夹爪闭合抓取居中正方体的生成场景。
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
uv run scripts/generate_cube_grasp_scene.py
uv run scripts/generate_cube_grasp_scene.py --gripper-xml assets/robotiq_2f85/2f85_touch_grid.xml --output-xml assets/scenes/cube_grasp_touch_grid.xml
uv run pytest
uv run --extra sim scripts/check_mjcf.py
uv run --extra sim scripts/view_taxels.py
uv run --extra sim scripts/report_taxels.py
uv run --extra sim scripts/run_cube_grasp_demo.py
uv run --extra sim --extra viz scripts/run_touch_grid_demo.py
```

两个闭合演示默认以实时速度启动 MuJoCo viewer。自动化验证或无图形环境可传入
`--no-viewer`；`run_touch_grid_demo.py` 另可传入 `--no-window` 关闭 OpenCV 面板。
默认是手动控制模式：在 viewer 的 **Control** 面板中调节 `fingers_actuator`，即可边闭合
边查看 taxel 或 OpenCV 触觉图。使用 `--auto-close` 才会按预设轨迹自动闭合。

在 viewer 中开启 `Sites` 与 `Contact points` 显示，即可检查 taxel 的局部坐标朝向、球形
接触体位置及接触点。`report_taxels.py` 的输出顺序与 XML 的 `00` 到 `22` 行优先命名一致。
`cube_grasp.xml` 固定夹爪为水平方向，并关闭重力以避免物块在闭合前掉落；它用于验证触觉接触，
不是抬升或滑移实验场景。场景包含世界 ``z=0`` 的水平地面；夹爪根节点绕 X 轴旋转 90°，
使指长轴沿世界 Y 轴、两指的触觉法向沿世界 ±X。地面使用无限延展的经典棋盘纹理。

## 接触模型

默认 taxel 使用 ``solimp="0.90 0.95 0.002"`` 和 ``solref="0.015 1"``，即允许约毫米级过渡的
软化刚体接触。这足以调节接触力上升速度与数值稳定性，但不会模拟硅胶的横向扩散、压痕或真实材料形变。
若研究这些效应，应另行引入 MuJoCo flex、有限元/降阶软体模型，或使用 `touch_grid` 一类专门的触觉传感器插件。

生成脚本的关键参数位于 `TAXEL_GRID`、`TAXEL_RADIUS` 与 `MIDDLE_ROW_TO_TOP_EDGE`。它们都在 `left_pad` 与 `right_pad` 的局部坐标系下定义，长度单位为米。

## 命名约定

- site：`left_taxel_site_00` 至 `right_taxel_site_22`
- taxel 力：`left_taxel_force_00` 至 `right_taxel_force_22`
- 指尖合力/力矩：`left_pad_force`、`left_pad_torque`、`right_pad_force`、`right_pad_torque`

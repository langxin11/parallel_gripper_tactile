# Robotiq 2F-85 Tactile

独立维护 MuJoCo 中为 Robotiq 2F-85 两个指尖添加触觉传感器的最小资产仓库。

## 内容与边界

- `assets/robotiq_2f85/2f85.xml`：未修改的基础夹爪模型。
- `scripts/generate_taxels_xml.py`：从基础模型生成带 18 个离散 taxel 的派生 MJCF。
- `assets/robotiq_2f85/2f85_taxels.xml`：生成结果；不可手工编辑。
- `assets/robotiq_2f85/2f85_touch_grid.xml`：每侧 32×32、三通道的 `touch_grid` 版本；生成器也支持自定义分辨率。
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
uv run scripts/generate_touch_grid_xml.py --rows 3 --cols 3 \
  --output-xml assets/robotiq_2f85/2f85_touch_grid_3x3.xml
uv run scripts/generate_cube_grasp_scene.py \
  --gripper-xml assets/robotiq_2f85/2f85_touch_grid_3x3.xml \
  --output-xml assets/scenes/cube_grasp_touch_grid_3x3.xml
uv run scripts/generate_cube_grasp_scene.py
uv run scripts/generate_cube_grasp_scene.py --gripper-xml assets/robotiq_2f85/2f85_touch_grid.xml --output-xml assets/scenes/cube_grasp_touch_grid.xml
uv run pytest
uv run scripts/check_mjcf.py
uv run scripts/view_taxels.py
uv run scripts/report_taxels.py
uv run scripts/run_cube_grasp_demo.py
uv run scripts/run_touch_grid_demo.py
```

两个闭合演示默认以实时速度启动 MuJoCo viewer。自动化验证或无图形环境可传入
`--no-viewer`；`run_touch_grid_demo.py` 另可传入 `--no-window` 关闭 OpenCV 面板。
默认是手动控制模式：在 viewer 的 **Control** 面板中调节 `fingers_actuator`，即可边闭合
边查看 taxel 或 OpenCV 触觉图。使用 `--auto-close` 才会按预设轨迹自动闭合。
三个带 viewer 的脚本均默认以 60 FPS 刷新；可传入 `--render-fps 30` 调整显示帧率，而物理仿真
仍按 MJCF 的 `timestep` 推进。

两个抓取演示均可在每个物理步调用记录钩子，将控制量与左右三维合力写为 CSV。例如：

```bash
uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer \
  --record-csv outputs/taxel_forces.csv --record-every 5
uv run scripts/run_touch_grid_demo.py --auto-close --no-viewer --no-window \
  --record-csv outputs/touch_grid_forces.csv --record-every 5
```

字段为 `step`、`time_s`、`control`、左右的 `fx/fy/fz`。三维力均在各自触觉 site 局部坐标系中；
taxel 版本为该侧 9 个 taxel 力之和，touch_grid 版本为该侧所有格点之和。
对于 taxel，MuJoCo `force` sensor 记录的是 taxel 子 body 施加给 pad 父 body 的相互作用力，
并非 pad 的全部外力；当前 site 的 +Z 指向表面外侧，故压缩时原始 `Fz` 为负，正压力应取 `-Fz`。

使用 SciencePlots 的 `science` 风格绘制记录曲线：

```bash
uv run scripts/plot_forces.py outputs/taxel_forces.csv
uv run scripts/plot_forces.py outputs/touch_grid_forces.csv \
  --output outputs/touch_grid_forces.pdf
```

默认生成与 CSV 同名的 PNG，图中依次显示控制量、左侧三维力和右侧三维力。绘图使用 `science`
与 `no-latex` 组合样式，故无需本机安装 TeX。

`view_taxels.py` 默认会在每个 site 上显示局部 XYZ 坐标系标架，因此可直接检查 taxel
力传感器的读数坐标系。默认以 60 FPS 实时渲染，并按模型 `timestep` 推进物理，因此可在
**Control** 面板操作夹爪并观察接触；`--render-fps 30` 可调整显示帧率，`--no-physics` 可保持
静态，`--no-site-frames` 可关闭标架。仍可在 viewer 中开启 `Sites` 与 `Contact points` 显示，以
检查球形接触体位置及接触点。`report_taxels.py` 的输出顺序与 XML 的 `00` 到 `22` 行优先命名一致。
`cube_grasp.xml` 固定夹爪为水平方向，并启用默认重力。方块下方的固定薄板承托它，避免在闭合前
掉落，同时使夹取过程中的触觉读数包含重力影响。场景包含世界 ``z=0`` 的水平地面；夹爪根节点绕 X 轴旋转 90°，
使指长轴沿世界 Y 轴、两指的触觉法向沿世界 ±X。地面使用无限延展的经典棋盘纹理，背景使用
MuJoCo 内置渐变与随机白点生成的星空天空盒。
交互演示以自由相机启动，可用鼠标旋转、平移和缩放视角；默认构图聚焦夹爪与方块。

## 接触模型

默认 taxel 使用 ``solimp="0.90 0.95 0.002"`` 和 ``solref="0.015 1"``，即允许约毫米级过渡的
软化刚体接触。这足以调节接触力上升速度与数值稳定性，但不会模拟硅胶的横向扩散、压痕或真实材料形变。
若研究这些效应，应另行引入 MuJoCo flex、有限元/降阶软体模型，或使用 `touch_grid` 一类专门的触觉传感器插件。

生成脚本的关键参数位于 `TAXEL_GRID`、`TAXEL_RADIUS` 与 `MIDDLE_ROW_TO_TOP_EDGE`。它们都在 `left_pad` 与 `right_pad` 的局部坐标系下定义，长度单位为米。

## 命名约定

- site：`left_taxel_site_00` 至 `right_taxel_site_22`
- taxel 力：`left_taxel_force_00` 至 `right_taxel_force_22`
- 指尖合力/力矩：`left_pad_force`、`left_pad_torque`、`right_pad_force`、`right_pad_torque`

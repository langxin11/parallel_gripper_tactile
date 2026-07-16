# 常用工作流

所有命令均从仓库根目录执行。项目使用 Python 3.12，由 `uv` 自动管理环境。

## 检查与查看资产

```bash
uv run scripts/generate_taxels_xml.py
uv run scripts/generate_touch_grid_xml.py
uv run scripts/check_mjcf.py
uv run scripts/view_taxels.py
uv run scripts/report_taxels.py
```

`view_taxels.py` 默认显示 site 坐标系标架；`--no-site-frames` 可关闭。默认 60 FPS 渲染，
`--render-fps 30` 可降低显示刷新率，不改变模型物理 `timestep`。`--no-physics` 用于静态检查位置。

## 运行抓取演示

```bash
# 离散 taxel，自动闭合
uv run scripts/run_cube_grasp_demo.py --auto-close

# touch_grid，自动闭合
uv run scripts/run_touch_grid_demo.py --auto-close

# 无图形环境
uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer
uv run scripts/run_touch_grid_demo.py --auto-close --no-viewer --no-window
```

不传 `--auto-close` 时，可在 MuJoCo viewer 的 **Control** 面板调节 `fingers_actuator`。
两个演示可用 `--render-fps` 控制渲染刷新率。

## 记录与绘图

记录钩子在每个物理步可采样控制量和左右合力；`--record-every` 指定每隔多少物理步写一行：

```bash
uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer \
  --record-csv outputs/taxel_forces.csv --record-every 5

uv run scripts/run_touch_grid_demo.py --auto-close --no-viewer --no-window \
  --record-csv outputs/touch_grid_forces.csv --record-every 5

uv run scripts/plot_forces.py outputs/taxel_forces.csv
uv run scripts/plot_forces.py outputs/touch_grid_forces.csv \
  --output outputs/touch_grid_forces.pdf
```

CSV 包含 `step`、`time_s`、`control` 及左右的 `fx/fy/fz`。绘图脚本使用 SciencePlots 的
`science` 和 `no-latex` 风格，不要求安装 TeX。

## 切换触觉模型或场景

```bash
# 使用 3×3 touch_grid 资产
uv run scripts/run_touch_grid_demo.py \
  --gripper-xml assets/robotiq_2f85/2f85_touch_grid_3x3.xml

# 使用完整的外部 MJCF，跳过默认运行时场景组合
uv run scripts/run_cube_grasp_demo.py --scene path/to/complete_scene.xml
```

更多关于运行时组合和名称前缀见 [项目结构](architecture.md)；传感器力方向与坐标系见
[触觉读数约定](tactile-conventions.md)。

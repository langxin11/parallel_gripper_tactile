# 常用工作流

所有命令均从仓库根目录执行。项目使用 Python 3.12，由 `uv` 自动管理环境。

## 检查与查看资产

```bash
uv run scripts/generate_taxels_xml.py
uv run scripts/generate_taxels_xml.py --shape box
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
uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer --no-rerun
uv run scripts/run_touch_grid_demo.py --auto-close --no-viewer --no-rerun
```

不传 `--auto-close` 时，可在 MuJoCo viewer 的 **Control** 面板调节 `fingers_actuator`。
两个演示默认还会启动 Rerun 触觉仪表盘：左压力、右压力、左切向和右切向四个等宽、等高空间
视图紧凑排列在同一行；压力图保持传感器原始分辨率，切向箭头最多聚合为 8×8，并用网格线标出
各显示区域。合力与控制量按仿真时间绘制。
`--no-rerun` 只关闭 Rerun，不影响 MuJoCo viewer；
`--render-fps` 控制 MuJoCo 渲染刷新率。

## 记录与绘图

记录钩子在每个物理步可采样控制量和左右合力；`--record-every` 指定每隔多少物理步写一行：

```bash
uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer --no-rerun \
  --record-csv outputs/taxel_forces.csv --record-every 5

uv run scripts/run_touch_grid_demo.py --auto-close --no-viewer --no-rerun \
  --record-csv outputs/touch_grid_forces.csv --record-every 5

uv run scripts/plot_forces.py outputs/taxel_forces.csv
uv run scripts/plot_forces.py outputs/touch_grid_forces.csv \
  --output outputs/touch_grid_forces.pdf
```

CSV 包含 `step`、`time_s`、`control` 及左右的 `fx/fy/fz`。taxel 与 touch-grid 都统一记录
“物体施加给触觉表面”的力，在各自 site 局部坐标系中表达，压缩时 `Fz > 0`。绘图脚本使用
SciencePlots 的 `science` 和 `no-latex` 风格，不要求安装 TeX。

Rerun 的 `.rrd` 会保存左右完整三通道触觉网格、显示用箭头、合力与控制量；只有显式传入
`--record-rrd` 才会落盘。`--rerun-hz` 按仿真时间控制采样频率，默认 100 Hz；频率不随场景
`timestep` 改变，且不能超过 MuJoCo 物理频率：

```bash
# 实时观察并同时保存
uv run scripts/run_touch_grid_demo.py --auto-close \
  --record-rrd outputs/touch_grid.rrd

# 不启动任何窗口，仅生成 CSV 和 RRD
uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer --no-rerun \
  --record-csv outputs/taxel_forces.csv \
  --record-rrd outputs/taxel.rrd

# 回放
uv run rerun outputs/taxel.rrd
```

即使传入 `--no-rerun`，`--record-rrd` 仍会使用文件 sink 正常记录。CSV 继续作为稳定、紧凑的
合力数据格式；网格级回放使用 RRD。

## 切换触觉模型或场景

```bash
# 使用 3×3 touch_grid 资产
uv run scripts/run_touch_grid_demo.py \
  --gripper-xml assets/grippers/robotiq_2f85/2f85_touch_grid_3x3.xml

# 使用完整的外部 MJCF，跳过默认运行时场景组合
uv run scripts/run_cube_grasp_demo.py --scene path/to/complete_scene.xml
```

更多关于运行时组合和名称前缀见 [项目结构](architecture.md)；传感器力方向与坐标系见
[触觉读数约定](tactile-conventions.md)。

## 公平比较 box taxel 与 touch_grid

```bash
# 单独查看或运行 box taxel
uv run scripts/view_taxels.py \
  --gripper-xml assets/grippers/robotiq_2f85/2f85_taxels_box.xml
uv run scripts/run_cube_grasp_demo.py --auto-close \
  --gripper-xml assets/grippers/robotiq_2f85/2f85_taxels_box.xml

# 自动运行同条件 A/B 比较
uv run scripts/compare_tactile_models.py
uv run scripts/compare_tactile_models.py \
  --record-every 5 \
  --output-csv outputs/tactile_model_comparison.csv \
  --output-plot outputs/tactile_model_comparison.png

# 抓稳后撤去支撑，并施加默认 5 N、2 Hz、1 s 的世界 Y 向正弦扰动
uv run scripts/compare_tactile_models.py --disturbance \
  --record-every 5 \
  --output-csv outputs/tactile_disturbance_comparison.csv \
  --output-plot outputs/tactile_disturbance_comparison.png
```

比较固定使用 3×3 平面 box taxel 与 3×3 touch-grid：二者具有相同的 pad 外形、碰撞分块、
接触参数、物体初态和闭合轨迹。联合 CSV 保留左右三维力，曲线图叠加正法向压力。终端报告最后
20% 仿真的稳态均值、峰值和对称相对误差；默认任一侧稳态误差超过 10% 即返回失败。

`--disturbance` 启用固定时序的切向稳定性实验：闭合 1.0 s、带支撑稳定 0.5 s、关闭支撑板碰撞、
无支撑稳定 0.5 s、扰动 1.0 s、恢复 0.5 s。外力通过 `xfrc_applied` 施加在方块质心，方向固定为
世界坐标系 +Y；`--steps` 在该模式下不生效，步数由总时长和物理 `timestep` 自动决定。常用调节项为：

```bash
uv run scripts/compare_tactile_models.py --disturbance \
  --disturbance-force 5 \
  --disturbance-frequency 2 \
  --disturbance-duration 1 \
  --support-settle-duration 0.5 \
  --release-settle-duration 0.5 \
  --recovery-duration 0.5 \
  --slip-threshold 0.002
```

扰动 CSV 在原有局部三维力之外增加阶段、世界系外力、左右世界系力、世界系合力以及方块位姿和
速度。图中同时显示控制量、外力、局部切向力、世界 Y 合力和方块在世界 YZ 切平面内的位移。
终端用两种模型世界 Y 合力的 NRMSE 衡量响应一致性，并以扰动开始时的位置为基准计算扰动及
恢复阶段的最大切向位移和速度；默认位移超过 2 mm 判为滑移。

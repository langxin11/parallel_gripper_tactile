# 常用工作流

所有命令从仓库根目录运行。项目使用 `uv` 管理 Python 3.12 环境和开发工具。

## 初始化与回归检查

```bash
uv sync --all-groups
uv run pgt-check configs/robotiq_2f85.toml
uv run pgt-check configs/custom_parallel_gripper.toml
uv run pytest
```

在修改 Python 代码后，还应运行：

```bash
uv run ruff check .
uv run ruff format --check .
```

## Robotiq 参考模型

生成或检查离散 taxel 与 `touch_grid` 派生资产：

```bash
uv run scripts/generate_taxels_xml.py
uv run scripts/generate_taxels_xml.py --shape box
uv run scripts/generate_touch_grid_xml.py
uv run scripts/check_mjcf.py
uv run scripts/view_taxels.py
uv run scripts/report_taxels.py
```

运行离散 taxel 或 `touch_grid` 抓取演示：

```bash
uv run scripts/run_cube_grasp_demo.py --auto-close
uv run scripts/run_touch_grid_demo.py --auto-close
```

无图形环境下记录 CSV 或 RRD：

```bash
uv run scripts/run_cube_grasp_demo.py --auto-close --no-viewer --no-rerun \
  --record-csv outputs/robotiq/taxel_demo/taxel_forces.csv \
  --record-rrd outputs/robotiq/taxel_demo/taxel.rrd

uv run scripts/run_touch_grid_demo.py --auto-close --no-viewer --no-rerun \
  --record-csv outputs/robotiq/touch_grid_demo/touch_grid_forces.csv

uv run rerun outputs/robotiq/taxel_demo/taxel.rrd
```

## 触觉模型比较

`compare_tactile_models.py` 使用 3×3 平面 box taxel 与 3×3 `touch_grid` 进行同条件比较。
二者共享 pad 外形、接触参数、物体初态和闭合轨迹。

```bash
uv run scripts/compare_tactile_models.py \
  --output-csv outputs/robotiq/comparison/tactile_model_comparison.csv \
  --output-plot outputs/robotiq/comparison/tactile_model_comparison.png

uv run scripts/compare_tactile_models.py --disturbance \
  --output-csv outputs/robotiq/comparison/tactile_disturbance_comparison.csv \
  --output-plot outputs/robotiq/comparison/tactile_disturbance_comparison.png
```

扰动实验会在带支撑稳定后移除支撑碰撞，避免支撑摩擦参与抗扰；随后向方块质心施加世界
Y 方向的正弦力。结果中的左右局部力会额外转换为世界系，以便比较合力与滑移响应。

## 自研夹爪：资产与姿态检查

自研资产应首先通过结构、闭环、碰撞过滤和运动扫掠检查：

```bash
uv run scripts/verify_mujoco.py
uv run pgt-check configs/custom_parallel_gripper.toml
```

在实际抓取场景中查看 profile 的 mount 安装位姿：

```bash
uv run scripts/view_custom_grasp_scene.py
uv run scripts/view_custom_grasp_scene.py --closed
uv run scripts/view_custom_grasp_scene.py --frame body
```

单独检查某个 Pillar 的接触读数：

```bash
uv run scripts/run_custom_gripper_tactile_demo.py --taxel left:11 --auto-close
```

## 自研夹爪：抓取验收与视频

运行无支撑保持、法向力跟踪和切向扰动验收：

```bash
uv run scripts/run_custom_grasp_validation.py \
  --output-csv outputs/custom_gripper/validation/custom_gripper_grasp.csv \
  --output-plot outputs/custom_gripper/validation/custom_gripper_grasp.png
```

默认测试块质量为 `50 g`，默认扰动为世界 Y 方向 `5 N`、`2 Hz` 正弦力。退出码和控制台结果
反映保持、扰动、法向力跟踪和数值稳定性检查；不要把一次仿真通过解读为实机性能保证。

录制视频：

```bash
uv run scripts/record_custom_grasp_video.py \
  --output outputs/custom_gripper/disturbance_video/custom_gripper_disturbance.mp4

uv run scripts/record_custom_grasp_video.py --force 0 \
  --output outputs/custom_gripper/disturbance_video/custom_gripper_zero_disturbance.mp4
```

视频与验收采用相同的“MIT 预接触 + simple-pid 法向力外环”控制器，画面显示控制状态及
`Fn=实测值/目标值`。无桌面显示的环境在命令前添加 `MUJOCO_GL=egl`。对应 CSV/曲线使用
`run_custom_grasp_validation.py` 的 `--output-csv/--output-plot` 写入同一目录；默认目标总
法向力为 `8 N`、方块质量为 `50 g`。

## 更新 Onshape 导出

```bash
uv run scripts/prepare_onshape_export.py RAW.xml PREPARED.xml
uv run scripts/verify_mujoco.py --mjcf PREPARED.xml
```

确认检查通过后，才更新 profile 指向的 `parallel_gripper_prepared.xml`，并重新运行
`pgt-check` 和完整测试。细节见[Onshape 导出与升级](onshape-export-upgrade.md)。

## 文档站

```bash
uv run zensical serve
uv run zensical build
```

`serve` 会监视 `docs/` 和 `zensical.toml`，保存后自动刷新预览；`build` 将静态站点写入
被 Git 忽略的 `site/` 目录。

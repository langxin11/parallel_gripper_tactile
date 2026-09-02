# Changelog

本项目的所有重要变更都记录在此文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [Unreleased]

### 新增

- `direct-torque` 控制器变体（`--controller-variant direct-torque`）：位置式 MIT 力控的
  直接力矩式对照。跟踪阶段力误差经 `torque_feedback_gain` 直接进入 MIT 前馈力矩，
  MIT kp/kd 由控制器逐周期覆盖为 0（接近阶段仍共享 profile 位置伺服增益），PID 与
  刚度位置修正诊断为 0、刚度估计器照常运行；`force_tracking_controller_comparison`
  默认矩阵由 108 扩至 135 条
- profile 新增字段 `control.force.torque_feedback_gain`（默认 0.0，保持位置式行为；
  大于 0 时跟踪阶段启用直接力矩式力控）
- 三类标准力跟踪任务 `step.yaml`、`ramp.yaml` 与 `mixed_waypoints.yaml`，分别覆盖阶跃、线性加载/卸载和
  平台—平滑斜坡综合测试
- `force_tracking_controller_comparison` study：固定展开 controller × task × material × seed，支持
  `--dry-run` 审阅矩阵，并生成聚合指标、饱和比例、消融增量和同 seed 轨迹对比图
- `force_tracking_stiffness_estimator_comparison` study：固定 `pid-stiffness-ff`，比较
  `secant_ewma`、`window_linear` 与 `window_quadratic` 三种刚度估计器，并输出聚合结果和同 seed
  力—刚度对比图
- 新增 `stiff=(-2500,-15)` 显式接触 preset；默认批量研究改用 `medium/hard/stiff`，原
  `soft=(-250,-5)` 仅保留用于兼容和专项标定
- `docs/custom-gripper-next-phase.md`：自研平行夹爪下一阶段实施路线图（Pillars 触觉反馈实验）
- `parallel_gripper_tactile.protocols`：统一的实验时序协议
  `DisturbanceProtocol`（含支撑释放、无支撑保持、切向扰动状态设置）
- `parallel_gripper_tactile.video`：渲染级力箭头、像素保存与 MP4 编码共享工具
- `recording.run_demo_loop`：两个触觉演示共用的驱动循环（viewer 节流、物理推进、采样与记录）
- `AGENTS.md`：面向 AI 编码代理的仓库协作约定入口，汇总语言策略、提交信息规范与验证门禁
- `metrics.json` 新增 `rise_time_s`、`overshoot_ratio`、`settling_time_s` 三个阶跃瞬态指标；
  仅在 `hold` 任务存在合格加载阶跃（跳变不低于 1 N、平台段不低于 0.5 s）时计算，
  无法判定时输出 `null`，study 聚合按 NaN 感知口径统计

### 变更

- `ramp.yaml` 任务在卸载终点后新增 2 s 终端保持段（保持 1 N），tracking 时长由 6 s 延长至
  8 s，用于终端稳态误差统计

### 修复

- `run_custom_grasp_validation`：首步即失稳时不再因空数据抛 `IndexError`，
  而是以仿真失败状态返回
- `report_taxels`：docstring 明确只读取模型初始状态，避免误导

### 重构

- 三份重复的扰动协议类收敛为 `DisturbanceProtocol`；`release_settle` 相位
  更名为 `unsupported_hold`，比较脚本 CLI 参数改为 `--hold-duration`
- `run_cube_grasp_demo` 与 `run_touch_grid_demo` 共用驱动循环；
  `run_touch_grid_demo` 新增 `--close-control`，消除硬编码控制量
- 箭头/像素/编码工具收敛到 `parallel_gripper_tactile.video`
- `docs/custom-gripper-next-phase.md` 按实现进度重写：各阶段标注
  ✅/⏳/⬜ 状态，修正 `ContactTaxelReader` API 与 `outputs/` 实际布局，
  更新下一次开发建议为 drive 行程复核
- 新脚本注释统一为中文（与 `scripts/`、docs 一致），`verify_mujoco.py`
  移入 `scripts/` 并修正默认模型路径
- `validate_profile` 增加 contact_geom 模式下的 site 存在性校验；
  `load_profile` 改为向上探测仓库根；`tactile_center_in_base` 由 profile
  推导中央 taxel
- `outputs/` 按夹爪与实验类别分子目录（`robotiq/{taxel_demo,touch_grid_demo,
  comparison,disturbance_video}`、`custom_gripper/{validation,disturbance_video,
  experiments}`），脚本默认输出路径与 README、docs 同步更新
- ruff `D` 规则豁免收窄：`scripts/` 与 `tests/` 仅豁免语言相关的 5 条
  （`D400`/`D401`/`D403`/`D404`/`D415`），其余格式类 docstring 检查
  自动生效，并补齐暴露的 32 处 docstring 缺口

## [0.2.0] - 2026-08-19

### 新增

- 触觉仿真泛化为多平行夹爪：类型化 profile（`configs/*.toml`）统一模型路径、执行器、
  控制范围、安装位姿与触觉阵列命名
- 自研曲柄滑块平行夹爪（`custom_parallel_gripper`）接入，左右各 3×3 Pillars 触觉接触 geom
- `pgt-check` 验证 CLI 与 Onshape 导出准备脚本（`prepare_onshape_export.py`）
- 可比触觉传感器模型（taxel 力 / touch-grid / Pillars 接触）与切向扰动基准
- Rerun 触觉仪表盘与对应工作流文档
- 扰动视频录制与发表级绘图脚本（`record_disturbance_video.py`）
- 运行时场景组合（`grasp_scene.py`）与增强重力抓取演示
- 触觉录制与可视化工具（`recording.py`）

### 变更

- 运行时统一为 Python 3.12（`uv`），依赖锁定 `uv.lock`

### 文档

- `docs/architecture.md`、`docs/workflows.md`、`docs/tactile-conventions.md`、
  `docs/onshape-export-upgrade.md`

## [0.1.0] - 2026-07-16

### 新增

- Robotiq 2F-85 指尖触觉资产仓库（taxel 3×3 与 touch-grid 高分辨率触觉模型）
- 方块闭合抓取触觉场景（无限棋盘地面、水平地面侧向抓取）
- viewer 手动控制夹爪与默认实时显示抓取演示
- taxel 检查工具（`view_taxels.py`）

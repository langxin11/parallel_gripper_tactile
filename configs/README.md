# 配置组合入口

人工配置按所有权拆分，由 Hydra 只在入口层组合，再交给 Pydantic 领域模型做完整校验。runner 只消费
组合后冻结的 profile 和 task，不读取片段或旧完整 profile 覆盖结果。

## 单次实验

默认 DM Torque ADRC + Ramp + hard 计划：

```bash
uv run python scripts/research/run.py execution=plan
```

执行时去掉 `execution=plan`。常见变化只替换一个配置组：

```bash
# 使用保留的原始 mesh 复现历史碰撞行为。
uv run python scripts/research/run.py model=dm_gripper/original_mesh execution=plan

# 换 PID 与估计器。
uv run python scripts/research/run.py \
  controller=dm_gripper/full estimator=window_quadratic execution=plan

# 使用统一导纳组合；该 experiment 会同时选择相容的 controller、estimator 和 task。
uv run python scripts/research/run.py \
  experiment=dm_gripper/force_tracking_admittance_unified execution=plan
```

配置组职责如下：

- `platform/`：设备家族、后端、机构、安装和硬边界；
- `model/`：MJCF、碰撞近似、触觉布局和传感器噪声标定；
- `controller/`：完整控制律、MIT 可调增益、滤波及接触状态参数；DM 变体共享
  `dm_gripper/_base.yaml` 公共基线，变体文件只声明 `name` 与差异字段，模块启停差异
  由 `configure_force_controller` 在组合后按 `controller.name` 派生；
- `estimator/`：刚度估计方法和参数，或显式 `none`；
- `task/`：目标曲线、时序、扰动和任务验收参数；
- `material/`：接触材料 preset；
- `execution/`：计划／执行、输出、viewer、记录和求解选项；
- `experiment/`：只选择上述已有组并命名常用组合。

`model=dm_gripper/{height_spheres,original_mesh}` 分别选择当前默认球体代理与保留的原始 mesh。
`multiccd` 不属于 model，使用 `execution.multiccd_enabled=false` 单独切换。一阶 `adrc` 与
`direct_torque` 两个历史复现配置入口已移除，不进入默认正式控制器对比；变体派生能力
仍保留在代码层，历史数据复现以当时的 git 版本为准。

根目录的 `dm_gripper.yaml` 与 `robotiq_2f85.yaml` 只保留为独立 profile schema 示例和底层 Python API
的兼容默认值，不是组合入口的参数来源。派生完整 profile 已删除；单次实验和正式研究均从上述配置组
构造对象，正式研究直接验证并冻结组合结果。

其余单次实验也从同一个 `run.yaml` 组合，并共享严格领域解析：

```bash
# 已知摩擦力调度与动态注水。
uv run python scripts/research/run.py \
  experiment=dm_gripper/force_scheduling_gravity_hold execution=plan
uv run python scripts/research/run.py \
  experiment=dm_gripper/force_scheduling_dynamic_filling execution=plan

# 摩擦估计；可用 task=friction_estimation/high_friction 等替换任务。
uv run python scripts/research/run.py \
  experiment=dm_gripper/friction_estimation_nominal execution=plan

# Robotiq 离散力；只替换 model 即可检查另外两种触觉布局。
uv run python scripts/research/run.py \
  experiment=robotiq_2f85/discrete_force execution=plan
uv run python scripts/research/run.py \
  experiment=robotiq_2f85/discrete_force \
  model=robotiq_2f85/touch_grid_3x3 execution=plan
```

`pgt` 的运行、查看和比较命令使用相同的 experiment 名，并以可重复的 `--set` 传入 Hydra 覆盖：

```bash
uv run pgt run force-track \
  --experiment dm_gripper/force_tracking_default \
  --set controller=dm_gripper/full --set task=force_tracking/step
uv run pgt run force-schedule \
  --experiment dm_gripper/force_scheduling_gravity_hold
uv run pgt run friction-estimate \
  --experiment dm_gripper/friction_estimation_nominal
uv run pgt run discrete-force \
  --experiment robotiq_2f85/discrete_force
uv run pgt view taxels \
  --experiment robotiq_2f85/discrete_force \
  --set model=robotiq_2f85/touch_grid_3x3
uv run pgt compare tactile \
  --left-set model=robotiq_2f85/box_force_sensor \
  --right-set model=robotiq_2f85/touch_grid_3x3
```

旧 `--profile`／`--task` 和控制器、材料、seed 等重复科学参数已由 `--experiment`／`--set`
替代；输出根目录由 `execution.output_root` 唯一负责。抓取、接触比较和视频命令仍保留自身的实验时序或
渲染选项，但其 profile 与材料来自同一组合结果。

## 研究入口

正式研究通过统一 `study.yaml + research=<purpose>/study` 选择。每个目的目录中的 `study.yaml` 同时保存
研究问题、预期决策、准入／停止／排除依据、profile 的命名 experiment 组合和唯一领域矩阵，不再跳转到
第二份 domain YAML。计划与执行都接收该组合产生的同一冻结 profile：

活跃研究只在 `study.profile` 中选择基础 experiment 和确有必要的 controller、estimator 或 model 覆盖；
任务与 seed 由 `study.definition` 的条件矩阵唯一拥有，输出目录由 `execution.output_root` 唯一拥有。
因此 `study.definition` 不再重复保存完整 profile 路径或 `output_root`。

推荐的科学决策顺序是“默认 `window_linear` 合理性检查、刚度位置限幅三臂验证、刚度速率控制频率验证与 PID 模块消融 → 刚度速率参数调优、Torque ADRC coarse／confirm 和独立导纳调优
→ 人工审查并冻结配置 → 最终控制器比较”。局部起滑与 Robotiq 离散力属于独立研究。这个顺序不改变
配置所有权：各 Study 不会自动回写优胜参数，只有 Torque ADRC 的 `coarse → confirm` 构成程序强制的
谱系依赖。

```bash
# 基础组件验证。
uv run python scripts/research/study.py research=stiffness_ground_truth_validation/study
uv run python scripts/research/study.py research=force_tracking_stiffness_limit_pilot/study
uv run python scripts/research/study.py research=force_tracking_stiffness_rate_validation/study
uv run python scripts/research/study.py research=force_controller_ablation/study

# 参数调优；速率调优保留 pid-torque-ff 性能基线，ADRC confirm 必须绑定已完成 coarse 的绝对目录。
uv run python scripts/research/study.py research=force_tracking_stiffness_rate_tuning/study
uv run python scripts/research/study.py research=force_tracking_stiffness_rate_refinement/study
uv run python scripts/research/study.py research=force_tracking_stiffness_rate_confirmation/study
uv run python scripts/research/study.py research=torque_adrc_tuning/study
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study study.stage=confirm \
  study.coarse_study_dir=/absolute/path/to/coarse-study

# 人工冻结候选配置后再生成最终控制器比较计划。
uv run python scripts/research/study.py research=force_controller_selection/study

# 独立研究。
uv run python scripts/research/study.py research=friction_local_slip_validation/study
uv run python scripts/research/study.py research=robotiq_discrete_force_validation/study
```

以上命令默认只生成计划；显式追加 `execution=study_run` 才执行仿真。执行时可再追加
`execution.workers=8`，按条件使用 MuJoCo CPU 多进程；计划、manifest、聚合和绘图仍由父进程统一管理。
完整命令顺序与决策门见
[`docs/workflows.md`](../docs/workflows.md#推荐的正式研究执行顺序)，硬依赖、配置冻结和生命周期语义见
[`docs/research-configuration.md`](../docs/research-configuration.md#推荐执行路线与决策门)。

旧碰撞／接触模型诊断入口及专属实现已退役，历史结论与复现版本见
[`模型验证结论`](../docs/control-comparison-ablation.md#collision-geometry-conclusions)。
当前字段所有权、组合规则和执行语义见 `docs/research-configuration.md`；历史迁移快照保持不变。

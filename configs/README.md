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
# 换碰撞模型。
uv run python scripts/research/run.py model=dm_gripper/flat_spheres execution=plan

# 换 PID 与估计器。
uv run python scripts/research/run.py \
  controller=dm_gripper/full estimator=window_quadratic execution=plan

# 使用完整导纳组合；该 experiment 会同时选择相容的 controller、estimator 和 task。
uv run python scripts/research/run.py \
  experiment=dm_gripper/force_tracking_admittance execution=plan
```

配置组职责如下：

- `platform/`：设备家族、后端、机构、安装和硬边界；
- `model/`：MJCF、碰撞近似、触觉布局和传感器噪声标定；
- `controller/`：完整控制律、MIT 可调增益、滤波及接触状态参数；
- `estimator/`：刚度估计方法和参数，或显式 `none`；
- `task/`：目标曲线、时序、扰动和任务验收参数；
- `material/`：接触材料 preset；
- `execution/`：计划／执行、输出、viewer、记录和求解选项；
- `experiment/`：只选择上述已有组并命名常用组合。

`model=dm_gripper/{height_spheres,flat_spheres,coplanar_mesh,original_mesh}` 覆盖四个现有碰撞资源。
`multiccd` 不属于 model，使用 `execution.multiccd_enabled=false` 单独切换。`direct_torque` 与一阶
`adrc` 仅供独立历史复现，不进入默认正式控制器对比。

根目录的 `dm_gripper.yaml` 与 `robotiq_2f85.yaml` 只保留为独立 profile schema 示例和底层 Python API
的兼容默认值，不是组合入口的参数来源。派生完整 profile 已删除；单次实验和正式研究均从上述配置组
构造对象，正式研究解析还会逐字段检查组合基础对象与兼容默认值一致。

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

```bash
# 默认控制器选型，计划为 162 条；direct-torque 与一阶 ADRC 已有证据退出。
uv run python scripts/research/study.py

# PID 模块消融、刚度估计器验证、局部起滑验证。
uv run python scripts/research/study.py research=force_controller_ablation/study
uv run python scripts/research/study.py research=stiffness_estimator_validation/study
uv run python scripts/research/study.py research=friction_local_slip_validation/study

# 两阶段 Torque ADRC 共用一个权威定义，confirm 必须绑定 coarse 谱系。
uv run python scripts/research/study.py research=torque_adrc_tuning/study
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study study.stage=confirm \
  study.coarse_study_dir=/absolute/path/to/coarse-study

# 显式执行时选择统一 execution 组。
uv run python scripts/research/study.py \
  research=robotiq_discrete_force_validation/study execution=study_run
```

已完成使命的碰撞／接触模型诊断只保留在
`research/archive/model_bug_diagnosis/study.yaml`，默认入口和正式控制器矩阵都不引用它。迁移基线、字段
所有权和完整新旧映射见 `docs/configuration-migration-baseline.md`。

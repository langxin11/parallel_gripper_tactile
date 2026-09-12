# 🧪 Hydra 科研配置与实验编排

科研入口与面向演示的 `pgt` 分工如下：

- `pgt` 用于模型查看、设备检查、交互演示和已知参数的一次性运行；
- `scripts/research/run.py` 用于组合配置、计划检查、单次实验和探索性 Multirun；
- `scripts/research/study.py` 用于带固定条件矩阵、配对关系、重复运行、失败账本和聚合统计的正式研究。

三个入口最终调用同一 Python runner 和实验内核，不通过子进程互相调用。Hydra 只存在于科研入口与
`research/` 编排层，不进入控制循环、共享控制核或硬件基础包。

## 安装科研依赖

```bash
uv sync --all-packages --all-groups --locked
```

Hydra 与 OmegaConf 位于独立的 `research` 依赖组；`dev` 包含该组，因此完整开发环境无需另行选择。

## 单次实验

默认 DM 力跟踪组合：

```bash
uv run python scripts/research/run.py
```

显式选择控制器、估计器、任务、材料和 seed：

```bash
uv run python scripts/research/run.py \
  controller=dm_gripper/adrc_torque \
  estimator=window_linear \
  task=force_tracking/ramp \
  material=hard \
  seed=0
```

DMgripper 共享导纳仿真链路使用 experiment 组合，同时选择相容的控制器、估计器和任务：

```bash
uv run python scripts/research/run.py \
  experiment=dm_gripper/force_tracking_admittance
```

探索性参数组合使用 Hydra 原生 Multirun。每个组合拥有独立的 Hydra 外层目录和内部 run artifacts：

```bash
uv run python scripts/research/run.py -m \
  material=medium,hard,stiff \
  seed=0,1,2
```

## 计划模式

计划模式会完成 OmegaConf 插值、严格 Pydantic 校验、资源路径检查、profile 资源与触觉布局检查、
组件兼容性检查和 MuJoCo scene 编译，但不会创建 `MjData`、推进仿真或访问设备：

```bash
uv run python scripts/research/run.py \
  execution=plan \
  controller=dm_gripper/full \
  task=force_tracking/step \
  material=medium \
  seed=0
```

输出包含 `effective_configuration.json`、`composition_provenance.json`、`plan.json` 和 Hydra 的
`.hydra/` 启动快照。`--cfg job --resolve` 仅打印 OmegaConf 组合结果，不替代上述领域计划检查。

## 正式研究

### 推荐执行路线与决策门

正式 study 的配置相互独立，但从科学决策角度，推荐按以下路线组织新一轮完整实验：

```text
刚度参考真值验证 ──────────┐
估计器下游敏感性 ──────────┤
PID 模块消融 ──────────────┼→ 人工审查／冻结 PID-ADRC 候选 → 最终控制器比较
Torque ADRC coarse → confirm ┘

共享导纳调优 → 冻结共享导纳基线（不进入上述选型）
局部起滑验证、Robotiq 离散力验证：独立研究
```

各阶段含义如下：

| 阶段 | Study | 决策作用 |
| --- | --- | --- |
| 基础组件验证 | `stiffness_ground_truth_validation` | 以准静态中心差分参考比较三种刚度估计器的精度与低估风险。 |
| 基础组件验证 | `stiffness_estimator_validation` | 检查三种估计器对下游力跟踪的敏感性；不作为估计精度真值。 |
| 基础组件验证 | `force_controller_ablation` | 判断 PID、刚度位置前馈和力矩前馈的贡献。 |
| 参数调优 | `torque_adrc_tuning` | 先 coarse 搜索可行域，再 confirm 验证前五候选。 |
| 参数调优 | `dm_admittance_tuning` | 调整共享导纳接近和接触切换参数；作为独立基线。 |
| 决策门 | 人工审查 | 审查摘要、配对统计和失败记录；必要时更新配置、测试和文档并提交。 |
| 最终比较 | `force_controller_selection` | 只比较决策门之后已经冻结在配置中的控制器。 |
| 独立研究 | `friction_local_slip_validation` | 验证局部起滑检测，不阻塞力控制器选型。 |
| 独立研究 | `robotiq_discrete_force_validation` | 验证另一夹爪的整数命令控制，不依赖 DM 研究。 |

这一路线中只有 Torque ADRC 的 `coarse → confirm` 是入口强制校验的硬依赖；其他箭头是推荐的
科学决策顺序，不是程序调用依赖。计划模式也不是执行模式的前置文件依赖，但正式运行前应先审阅计划。
Study 不会把排名第一的参数自动写入另一个 Study；如果前置结果改变候选配置，必须先人工更新并冻结，
再启动最终控制器比较，否则最终矩阵仍会使用当前 YAML 中的固定参数。归档的模型 bug 诊断已经完成使命，
不进入这条路线。

控制器对比计划固定为当前权威 study YAML 中的 6 个控制器 × 3 个任务 × 3 个材料 × 3 个 seed，
共 162 个有序条件：

```bash
uv run python scripts/research/study.py \
  research=force_controller_selection/study
```

确认计划后执行：

```bash
uv run python scripts/research/study.py \
  research=force_controller_selection/study \
  execution=study_run
```

PID 模块消融计划为 4 个控制器 × 3 个材料 × 3 个 seed，共 36 个有序条件：

```bash
uv run python scripts/research/study.py \
  research=force_controller_ablation/study
uv run python scripts/research/study.py \
  research=force_controller_ablation/study \
  execution=study_run
```

局部起滑验证、刚度参考真值、估计器下游敏感性、DM 导纳调参与 Robotiq 离散力 study 用法相同，矩阵分别为
5 场景 × 3 seed（15 条）、3 估计器 × 3 材料 × 3 seed（27 条）、3 估计器 × 3 任务 × 3 材料 × 3 seed
（81 条）、16 候选 × 1 材料 × 2 seed
（32 条）与 5 控制器 × 4 材料 × 3 噪声 × 1 seed（60 条）：

```bash
uv run python scripts/research/study.py \
  research=friction_local_slip_validation/study
uv run python scripts/research/study.py \
  research=stiffness_ground_truth_validation/study
uv run python scripts/research/study.py \
  research=stiffness_estimator_validation/study
uv run python scripts/research/study.py \
  research=dm_admittance_tuning/study
uv run python scripts/research/study.py \
  research=robotiq_discrete_force_validation/study \
  execution=study_run
```

因果诊断研究一次调用执行一个 phase，`study.phase` 必选（10 个 phase 共 39 条条件）；原
`--phase all` 由逐 phase 循环替代：

```bash
uv run python scripts/research/study.py \
  research=archive/model_bug_diagnosis/study \
  study.phase=collision-geometry \
  execution=study_run
```

Torque ADRC 调参也由同一正式入口承载。正式 coarse 为 34 个满足测量带宽约束的候选 × 3 个任务 ×
`medium` × seed 0，共 102 条；confirm 从已完成 coarse 的可行排名取前五，并在缺席时追加基线，随后对
所选候选执行 3 个任务 × 3 个材料 × 3 个 seed：

```bash
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study \
  execution=study_run
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study study.stage=confirm \
  study.coarse_study_dir=/absolute/path/to/coarse-study
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study study.stage=confirm \
  study.coarse_study_dir=/absolute/path/to/coarse-study \
  execution=study_run
```

confirm 不只检查目录是否存在，还要求 coarse manifest 的生命周期 schema、研究类型、阶段、完成状态、
研究定义哈希和无执行异常均匹配，并验证 `candidate_ranking.csv` 已登记、内容摘要一致、候选全集及排名
schema 正确。任一项不一致都会在任何 confirm 子 run 前失败。

每个 `configs/research/<purpose>/study.yaml` 同时保存研究元数据与唯一条件矩阵；Hydra 只选择研究方案
和执行模式，不再通过 selector 跳转到第二份 domain YAML。入口明确拒绝
`-m`、`--multirun` 与 `hydra.mode=MULTIRUN`，并在任何子实验前失败，避免把同一矩阵重复展开。
计划和执行持有同一个冻结 `StudyPlan`；每个条件带稳定 `condition_id`、科学参数、配对键和基线角色。
每个研究还用 `study.profile.experiment` 与 `study.profile.overrides` 选择基础 profile，复用单次实验的组合
服务；解析、预检、执行与有效配置快照持有同一冻结对象。归档模型诊断因需改写历史模型路径而保留原始
profile 来源字段，但其基础对象仍会通过组合入口预检。

活跃研究的 profile 以 `study.profile.experiment` 组合结果为唯一来源。`study.profile.overrides` 只保存
矩阵共享且确实改变基础 profile 的 controller、estimator 或 model 选择，不再用 task、seed 或 execution
充当组合占位。任务与 seed 归 `study.definition` 的矩阵所有，研究目录归
`execution.output_root` 所有；领域定义不再重复保存旧完整 profile 路径和 `output_root`。研究计划中的
profile 摘要来自实际冻结的组合对象，而不是兼容 profile 文件。归档模型诊断因需要改写历史模型资源，
继续显式保留原始 profile 路径，并在解析时执行新旧等价检查。

## 生命周期、状态与失败分类

公共生命周期仅统一“已校验计划→逐条件执行→失败记录→聚合→研究专属绘图→产物登记”，不改变任务、
控制器、指标、统计公式或仿真循环。`study_manifest.json` 的状态为：

- `planned`：已完成预检，但没有创建 `MjData`、推进仿真或登记 run；
- `running`：正在逐条件执行，manifest 在每个条件完成后由父进程更新；
- `partial`：至少一个条件形成正常 run，同时至少一个条件发生 Python 执行异常；
- `completed`：所有条件都形成正常 run；其中仍可包含科学验收失败；
- `failed`：没有任何正常 run，或聚合／绘图阶段无法完成。

`condition_results` 将 `completed`、`scientific_failure` 和 `execution_error` 分开记录。兼容字段 `runs`
包含所有正常 run，`failed_runs` 只包含 `passed=false` 的科学失败，`failed_conditions` 只包含没有形成
正常 run 的 Python 异常。入口、配置、预检、聚合与绘图异常另存于 `lifecycle_failures`，不会混入
`failed_runs`。所有登记产物同时保存 SHA-256 摘要。

## 配置组及所有权

| 配置组 | 负责参数 | 不负责参数 |
| --- | --- | --- |
| `platform` | 设备家族、仿真后端、基础 profile | 控制算法、任务曲线 |
| `controller` | 控制器名称及该算法独有参数 | 估计器和材料 |
| `estimator` | 刚度估计方法或显式关闭 | 控制器增益 |
| `task` | 任务家族及 task 文件 | profile 或材料 |
| `material` | 接触材料 preset | 控制时序 |
| `execution` | 计划/执行、输出、条件进程数、viewer、记录与只读恢复检测 | 科学条件 |
| `research` | 研究问题、决策、准入／停止／排除依据、阶段谱系及唯一矩阵 | 外层笛卡尔积 |

优先级从低到高依次为：配置组默认值、根 preset 的 `_self_` 值、命令行覆盖。Hydra 对列表采用整表
替换，不进行元素级拼接；正式矩阵列表只在目的目录的 `study.yaml` 中维护。配置模型使用 `extra="forbid"`，未知字段、
拼写错误和缺失值会在仿真前失败。控制器组整体替换；领域解析器还会清空其他算法的 `adrc`、
`torque_adrc`、`admittance` 与直接力矩反馈字段，随后对完整 profile 重新执行 Pydantic 校验。

当前声明 DM 与 Robotiq 仿真组合。导纳必须配 `estimator=none`；非导纳控制器中仅 `pid-only` 允许显式关闭估计器；
Torque ADRC 参数只能随 `adrc-torque` 或 `adrc-torque-td` 出现。选择 platform 不会打开串口、连接设备、
使能电机或发送命令；尚未支持的硬件组合会被 schema 拒绝。

## 路径、产物与复现

Hydra 配置搜索路径由入口脚本确定；profile、task、study 和输出根目录中的相对路径则统一以代码仓库根
解析，不依赖调用时的当前目录。profile 内部 MJCF 等相对资源仍按 profile 文件所在目录解析，保持旧语义。

Hydra 拥有一次科研调用的外层目录，其中保存组合来源、选择、覆盖参数、Git 状态和领域有效配置；
`RunDirectory` 继续拥有内部单次实验目录、manifest、trace、metrics 与图。正式 study 在外层目录中保存
完整计划、实际完成记录、逐条件异常及聚合结果。单条件异常会登记到 `failed_conditions.json` 并继续其余
条件；若配置或预检阻止研究启动，入口仍写出 `setup_failure.json` 和 `state=failed` 的生命周期 manifest。

科学配置哈希排除时间戳、Hydra 输出目录、domain `output_root` 和绝对运行目录；它包含有序条件、完整
控制器参数（含默认值）、任务与 profile 内容摘要、材料、seed、统计／排序／阶段规则及 confirm 的 coarse
谱系。因此仅改变 cwd 或输出位置不会改变哈希，任何科学参数或输入资源内容变化都会改变。可在
`execution.recovery_source` 指向既有 study，生成 `recovery_assessment.json`：只有研究类型、阶段、哈希和
`condition_id` 均匹配且状态为 `completed` 的条件才列为已有成功。本阶段明确
`automatic_resume_enabled=false`，不会自动跳过、续跑或跨配置聚合。

实际执行与 `effective_parameters.json` 使用同一个冻结 `GripperProfile` 和 `ForceTrackingTask` 对象，
底层不会重新读取原始 profile 覆盖 Hydra 结果。

## 迁移边界

本阶段已贯通 DM 单次力跟踪、DM 共享导纳、正式控制器对比、PID 模块消融、Torque ADRC 两阶段调参、
摩擦局部起滑、刚度参考真值、刚度估计器下游敏感性、DM 导纳调参、Robotiq 离散力和因果诊断（单 phase 入口）。原
`scripts/experiments/` 研究入口已删除，统一使用 Hydra 正式入口；可复用矩阵展开与聚合实现仍位于包内
protocol。

迁移中保留的语义边界：导纳调参的候选排名与 raw 物理力峰值口径、估计器对比的 secant 基线与公共
seed 叠加、Robotiq 离散力的 HOLD/再激活时序与逐平台指标、诊断的单因素条件构造与数值/分类双横轴
绘图均原样保留；旧 protocol 私有的 `max_workers` 和 `--jobs` 已统一为 `execution.workers`。当
`execution.workers>1` 时，公共生命周期使用 `spawn` 条件级 CPU 多进程；每个进程独占 run 目录，父进程
独占 manifest、聚合与绘图。该调度参数不进入科学配置哈希，也不引入自动 resume、Optuna、Ray、MLflow
或分布式执行框架。

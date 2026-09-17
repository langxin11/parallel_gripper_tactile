# 🧪 Hydra 科研配置与实验编排

操作命令与研究顺序见[常用工作流](workflows.md)。本页只定义配置、计划、生命周期和复现契约。
`pgt`、科研单次与 study 最终调用同一 Python runner；Hydra 不进入控制循环或硬件基础包。

## 安装科研依赖

完整环境安装见[测试策略](testing.md)。仅运行实验可用 `uv sync --locked --no-default-groups`；
Hydra 与 OmegaConf 属于运行依赖。

## 单次实验

配置名是组目录下的相对路径：科研入口使用 `controller=dm_gripper/pid_only`，
`pgt run` 使用 `--set controller=dm_gripper/pid_only`。`pgt configs list` 只发现 YAML 元数据，
不保证跨组组合兼容。`run.py -m` 的每个探索性组合有独立外层目录和内部 run。

## 计划模式

`execution=plan` 完成插值、严格 Pydantic 校验、路径与 profile 资源检查、触觉布局与组件兼容检查、
MuJoCo scene 编译，不创建 `MjData`、推进时间或访问设备。输出包含 `effective_configuration.json`、
`composition_provenance.json`、`plan.json` 与 `.hydra/` 快照。
`--cfg job --resolve` 仅打印组合结果，不能替代领域检查。

## 正式研究

### 推荐执行路线与决策门

执行顺序见[正式研究路线](workflows.md#formal-study-route)。默认生成计划，`execution=study_run` 执行；
计划文件不是执行的前置依赖，但正式运行前应审阅相同参数的计划。各研究不会自动传递最优参数。

每个 `configs/research/<purpose>/study.yaml` 保存研究元数据和唯一条件矩阵，领域 protocol 唯一展开矩阵。
入口拒绝 `-m`、`--multirun` 与 `hydra.mode=MULTIRUN`。计划与执行共享冻结的 `StudyPlan`，
条件包含稳定 `condition_id`、科学参数、配对键和基线角色。

活跃研究以 `study.profile.experiment` 的组合结果为唯一基础 profile；`study.profile.overrides` 仅保存
矩阵共享且改变基础 profile 的 controller、estimator 或 model 选择。任务与 seed 归
`study.definition`，研究目录归 `execution.output_root`。解析、预检、执行和快照持有同一冻结对象，
摘要来自实际组合对象。归档模型诊断因改写历史资源而保留原 profile 路径，并执行等价检查。

Torque ADRC 的 `coarse → confirm` 强制校验谱系：coarse manifest 的生命周期 schema、研究类型、阶段、
完成状态、研究定义哈希和无执行异常必须匹配；`candidate_ranking.csv` 必须已登记且摘要、候选全集与
排名 schema 正确。confirm 按可行排名取前五，缺席时追加基线；校验失败发生在任何 confirm 子 run 之前。

## 生命周期、状态与失败分类

公共生命周期仅负责计划、执行、失败记录、聚合／绘图钩子和产物登记；科学条件、统计公式与仿真循环归领域模块。

| `study_manifest.json` 状态 | 含义 |
| --- | --- |
| `planned` | 已预检，未创建 `MjData`、推进仿真或登记 run。 |
| `running` | 条件正在执行，父进程逐条件更新 manifest。 |
| `partial` | 至少形成一个正常 run，且存在 Python 执行异常。 |
| `completed` | 所有条件均形成正常 run，仍可能有科学验收失败。 |
| `failed` | 没有正常 run，或聚合／绘图无法完成。 |

`condition_results` 区分 `completed`、`scientific_failure`、`execution_error`。
`runs` 包含所有正常 run；`failed_runs` 仅含 `passed=false` 的科学失败；`failed_conditions` 仅含未形成
正常 run 的 Python 异常。入口、配置、预检及聚合／绘图异常归 `lifecycle_failures`。登记产物均保存 SHA-256。

`execution.workers` 默认为 `1`；大于 `1` 时采用 `spawn` 条件级 CPU 多进程，worker 独占 run 目录，
父进程独占 manifest、聚合与绘图，并按计划顺序保存结果，保证配对和统计输入不受完成顺序影响。
进度中的“已处理”包含三类条件结果，不等于验收通过数。可选 `on_progress` 在父进程写入 manifest 后
接收不可变 `StudyProgress`；回调异常仅告警，不中断研究，也不发送到 worker。

## 出图模式 {#plot-modes}

`execution.plot_mode=summary|diagnostic` 默认为 `summary`，写入有效配置与 manifest，
不改变条件矩阵、指标、验收或科学哈希。

- 单次力跟踪默认生成 `tracking.png`，`diagnostic` 增加触觉与控制器诊断。
- 力跟踪和局部起滑 study 按计划预定最小 seed 保留逐次诊断；其他成功 run 保存轨迹、指标和输入。
  科学失败保留诊断，局部起滑负例按自身验收规则处理。代表 seed 不按误差或完成顺序选择。
- 跨条件聚合判定候选不可行，不等于单条件科学失败，不触发自动补图；需分析时选 `diagnostic`。
- 汇总默认省略简单基线差值图和 MAE 补充面板；配对消融、饱和及刚度验证指标保留。
  `diagnostic` 恢复补充图，但无有效基线时不生成空差值图。其他实验必要主图保持保留。
- 运行时图读取完整频率数据；保存轨迹仍按采样策略处理，事后重绘不能恢复已丢失的瞬态。
  无有效轨迹的执行异常只保留已有输入与失败账本。

## 配置组及所有权

| 配置组 | 所有权 |
| --- | --- |
| `platform` | 设备家族、后端、基础 profile。 |
| `model` | MJCF、碰撞几何与触觉布局。 |
| `controller` | 算法及独有参数。 |
| `estimator` | 刚度估计方法或显式关闭。 |
| `task` | 任务家族和曲线。 |
| `material` | 接触材料。 |
| `execution` | 计划／执行、目录、进程数、viewer、记录与恢复检查。 |
| `experiment` | 常用配置组合与有意覆盖。 |
| `research` | 研究问题、决策、准入／停止／排除规则、谱系及唯一矩阵。 |

优先级：组默认值 < 根 preset 的 `_self_` < 命令行。列表整表替换，不拼接；矩阵列表仅在研究定义中维护。
配置使用 `extra="forbid"`，未知字段与缺失值在仿真前失败。控制器组整体替换，解析器清除其他算法的
`adrc`、`torque_adrc`、`admittance` 和直接力矩反馈字段，再校验完整 profile。

导纳必须配 `estimator=none`；非导纳仅 `pid-only` 可显式关闭估计器。Torque ADRC 参数仅可随
`adrc-torque` 或 `adrc-torque-td` 出现。platform 选择不连接或使能设备，未支持的硬件组合由 schema 拒绝。

切向扰动仅支持 DM 的 `full`／`pid-only`，拒绝 viewer、导纳与 ADRC；material 写入任务的
`object_material`。解析先构造最终 profile，再按实际材料编译 scene，检查 MIT 法向控制及
`control_period_s` 不小于物理步长。它支持单次与探索性 Multirun，未定义正式 study。

共享仿真场景默认使用 `0.001 s` 物理步长，显式多速率触觉默认同为 `0.001 s`；
控制周期仍由任务配置独立指定。采样与读取边界见[公共触觉契约](tactile-conventions.md#readers)。
历史产物保留其原有时序条件，不因默认值调整而重新解释。

## 路径、产物与复现

配置搜索路径由入口确定；profile、task、study 和输出根目录的相对路径统一基于仓库根，
profile 内的 MJCF 等资源基于该 profile 所在目录，不依赖调用 cwd。

Hydra 外层目录保存组合来源、覆盖、Git 状态、领域有效配置及 study 计划与聚合；`RunDirectory`
拥有内部 run 的 manifest、trace、metrics 和图。单条件异常写入 `failed_conditions.json` 后继续；
启动前失败仍写入 `setup_failure.json` 和失败状态的生命周期 manifest。
实际运行与 `effective_parameters.json` 使用同一冻结 `GripperProfile` 和 `ForceTrackingTask`，不重读原文件覆盖配置。

科学哈希包含有序条件、完整控制器参数及默认值、任务与 profile 内容摘要、材料、seed、统计／排名／阶段
规则和 confirm 的 coarse 谱系；排除时间戳、输出位置、绝对运行目录、workers、绘图模式及进度观察。
改变 cwd 或输出位置不改变哈希，改变科学参数或输入资源内容会改变。

`execution.recovery_source` 仅生成 `recovery_assessment.json`。研究类型、阶段、哈希、`condition_id`
均匹配且状态为 `completed` 的条件才列为已有成功；`automatic_resume_enabled=false`，不自动跳过、续跑或跨配置聚合。

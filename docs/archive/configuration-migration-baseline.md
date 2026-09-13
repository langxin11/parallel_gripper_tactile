# 配置重构历史基线

> **状态：历史兼容基线。** 本文冻结配置重构实施时的输入、入口和研究矩阵，供回归测试解释；
> 当前配置方法以 [`../research-configuration.md`](../research-configuration.md) 和仓库
> `configs/README.md` 为准。

本文记录 [`configuration-refactor-goal.md`](configuration-refactor-goal.md) 迭代 1 的迁移基线。
机器可读权威快照为
`tests/baselines/configuration_refactor_v1.json`，其 SHA-256 为
`555e1d0945c5628b01cb6ed7ec494a0f04b3e047044d2b9728b81637fbaace96`。快照由重构前实现生成，
固定在 Git 提交 `65201b5f711c9aafbdec28a14bc8a76db5a3d59f`；其中 `${REPOSITORY_ROOT}` 表示捕获时的
仓库根，比较时必须先做同样的路径规范化。

快照包含 80 份 YAML 的解析值与内容摘要、6 份 profile 的完整 Pydantic 默认值、7 个 MJCF 资源摘要、
15 份可复用 task 的完整领域值、2 个单次 Hydra preset 的组合结果，以及所有静态正式研究和诊断
各 phase 的完整有序条件。Torque ADRC confirm 依赖 coarse 排名，快照保留其完整 domain 定义和
“可行前五名，必要时追加 baseline”的动态选择契约，不伪造尚不存在的排名产物。

## 字段所有权

最终 `GripperProfile` 仍是 runner 消费的冻结领域对象；下表规定人工输入片段的唯一所有者。组合服务
负责把片段合成为该对象，不允许 runner 再读取另一份 YAML 覆盖结果。

| 字段或语义 | 新所有者 | 迁移约束 |
| --- | --- | --- |
| `family`、`backend`、`control.mode/actuator/open/closed`、`mount` | `platform` | 机构、安装和后端能力不随控制器变化 |
| `control.mit.p_min/p_max/v_max/t_max`、`control.force.geometry` | `platform` | 设备硬边界和机构几何优先于可调限制 |
| `model.path`、完整 `tactile`、传感器噪声标准差 | `model` | MJCF 与相容的 taxel 布局、测量模型必须一起选择和校验 |
| `control.mit.kp/kd/t_ff` | `controller` | MIT 可调增益属于控制器，不复制设备量程 |
| 接触／释放阈值与确认步数、PID 增益、位置修正、控制滤波 | `controller` | 控制状态切换和滤波语义不随 task 或材料隐式变化 |
| `admittance`、`torque_adrc`、直接力矩反馈字段 | `controller` | 选择其他算法时必须整体清除无关算法字段 |
| 刚度估计的启用、方法、范围、滤波和采样门限 | `estimator` | `none` 显式关闭；相容性由组合后的领域校验保证 |
| 刚度位置／力矩前馈增益、刚度感知位置限幅 | `controller` | 这些字段决定控制律如何使用估计值，不属于估计方法 |
| `target_n`、waypoint、时序、扰动和任务验收阈值 | `task` | profile 中的旧 `target_n` 迁为实验默认 task 值，不形成第二权威来源 |
| 材料刚度／阻尼 preset | `material` | 不覆盖 task、controller 或 model |
| seed、计划／执行、输出、viewer、记录和 `multiccd` | `execution` 或实验／研究选择 | seed 是条件轴；`multiccd` 是求解执行选项，不属于碰撞资源 |
| 条件轴、配对键、基线角色、停止规则、阶段与谱系 | `research/<purpose>` | 只能由对应领域 protocol 展开为一个 `StudyPlan` |

### DM 导纳完整差异

`configs/dm_gripper_admittance.yaml` 相对 `configs/dm_gripper.yaml` 的全部有效差异如下；未列字段逐值相同。
这张表是迁移等价的最低比较集合，但验收仍比较完整对象。

| 差异 | 旧默认 → 旧导纳 | 新所有者 |
| --- | --- | --- |
| `name` | `dm_gripper` → `dm_gripper_admittance` | `experiment` 组合名 |
| `control.mit.kp` | `20.0` → `10.0` | `controller/dm_gripper/admittance` |
| `control.mit.kd` | `0.63793536` → `5.0` | `controller/dm_gripper/admittance` |
| `control.force.target_n` | `8.0` → `1.0` | `task/force_tracking/dm_admittance_ramp` 的初始目标 |
| `control.force.contact_threshold_n` | `0.15` → `1.0` | `controller/dm_gripper/admittance` |
| `control.force.kp/ki` | `0.016/0.2` → `0.0/0.0` | `controller/dm_gripper/admittance` |
| `control.force.filter_cutoff_hz` | `20.0` → `2.0` | `controller/dm_gripper/admittance` |
| `control.force.stiffness.enabled` | `true` → `false` | `estimator/none` |
| 导纳动力学、接近／过渡曲线、可调速度与前馈限幅 | `null` → 13 个显式字段 | `controller/dm_gripper/admittance` |
| 导纳位置上下限、闭合方向 | `null` → `0/π/2/+1` | `platform/dm_gripper/simulation`，组合时注入控制器 |
| `admittance.mit_torque_limit_nm` | `null` → `4.0` | 不设第二权威值；由 platform `mit.t_max` 派生并校验 |

迁移初期曾同时保留恒定 1 N 的 `dm_admittance.yaml` 和低力 Ramp；统一状态机稳定后，旧恒力任务已删除，
单次导纳与正式调参统一使用 `dm_admittance_ramp.yaml`。profile 的 `target_n` 仍不能代替任务曲线。

### Task 时间语义

- 力跟踪 reference 从“接触确认且 settle 完成”后计时，循环预算为
  `approach.timeout + settle + reference.duration + control_period`；`approach.duration` 不再额外相加。
  六个旧 task 的预算依次为：default `7.702 s`、mixed `10.702 s`、ramp `11.202 s`、step
  `7.202 s`、DM 导纳 hold `46.004 s`、DM 导纳 ramp `48.004 s`。
- 力调度的向下载荷从稳定接触、settle 完成并撤去支撑后计时；gravity hold 与 dynamic filling 的预算
  分别为 `6.302 s` 和 `10.302 s`，二者控制周期均为 `2 ms`。
- 摩擦估计保持“接触 settle → probe settle → probe → recovery → schedule load”的阶段顺序；探测确认可
  提前结束 probe。各 task 的完整阶段参数和预算已展开在机器快照，迁移不得改变其相对计时起点。
- Robotiq 的 `duration_s=24` 从仿真开始，是硬总时长；`18.5 s` reference 从首次进入
  `ADJUST/HOLD` 后开始。`1/30 s` 控制周期与 `0.01 s` 记录周期是两个独立时钟。

### 碰撞与 Robotiq 变体

DM 默认与 `flat_spheres` 的完整领域差异只有 `name` 和 `model.path`。四个目标模型的资源映射为：

| 目标 model | 旧来源 | 求解语义 |
| --- | --- | --- |
| `dm_gripper/height_spheres` | `dm_gripper.yaml` 的 `parallel_gripper_height_sphere_collision.xml` | 默认，`multiccd=true` |
| `dm_gripper/flat_spheres` | `dm_gripper_flat_spheres.yaml` 的 `parallel_gripper_flat_sphere_collision.xml` | 诊断／独立复现 |
| `dm_gripper/coplanar_mesh` | diagnosis 的 `parallel_gripper_coplanar_mesh_collision.xml` | 诊断归档，不进默认矩阵 |
| `dm_gripper/original_mesh` | diagnosis 的 `parallel_gripper_prepared.xml` | 诊断归档；可与 `multiccd=false` 独立复现 |

Robotiq 三个旧 profile 分别迁为 `model/robotiq_2f85/sphere_force_sensor`、`box_force_sensor` 和
`touch_grid_3x3`。标准 profile 的 `closed=255`，后两者为 `220`；该模型相关可达边界必须由
model 能力校验解释，不能在切换模型时静默沿用。前两者是 `force_sensor` 3×3 前缀布局，后者是
由 MJCF 推导行列的 `touch_grid` 双传感器布局。

## 重构时的新旧路径映射

下表记录重构实施时采用的新旧路径映射，不表示旧路径仍可用于当前入口。

| 旧路径 | 目标路径或处置 |
| --- | --- |
| `configs/dm_gripper.yaml` | 由 `platform/dm_gripper/simulation` + `model/dm_gripper/height_spheres` + controller + estimator 组合替代；根文件仅保留为底层 API 兼容默认值 |
| `configs/dm_gripper_admittance.yaml` | 删除派生整份 profile；差异分配到 `controller/dm_gripper/admittance`、`estimator/none` 和导纳 task |
| `configs/dm_gripper_flat_spheres.yaml` | 删除派生整份 profile；改选 `model/dm_gripper/flat_spheres` |
| `configs/robotiq_2f85.yaml` | `platform/robotiq_2f85/simulation` + `model/robotiq_2f85/sphere_force_sensor`；根文件仅保留为底层 API 兼容默认值 |
| `configs/robotiq_2f85_box.yaml` | 同一 platform + `model/robotiq_2f85/box_force_sensor` |
| `configs/robotiq_2f85_touch_grid.yaml` | 同一 platform + `model/robotiq_2f85/touch_grid_3x3` |
| `configs/force_tracking/{default_waypoints,step,ramp,mixed_waypoints,dm_admittance,dm_admittance_ramp}.yaml` | `configs/task/force_tracking/` 下保留正式任务；`mixed_waypoints` 收敛为 `mixed`，旧恒力 `dm_admittance` 后续由低力 Ramp 取代 |
| `configs/force_scheduling/{gravity_hold,dynamic_filling}.yaml` | `configs/task/force_scheduling/` 下同名文件 |
| `configs/friction_estimation/{low_friction,nominal_friction,high_friction,noisy_friction,no_slip_low_probe,hardware_scale_nominal}.yaml` | `configs/task/friction_estimation/` 下同名文件 |
| `configs/discrete_force/robotiq_delta_f_tick.yaml` | `configs/task/discrete_force/robotiq_delta_f_tick.yaml` |
| `configs/research/platform/dm/{simulation,admittance_simulation}.yaml` | 前者迁为 `platform/dm_gripper/simulation`；后者消除，导纳不再伪装成另一平台 |
| `configs/research/controller/dm/*.yaml`（10 份） | `configs/controller/dm_gripper/`；`adrc`、`direct_torque` 仅供独立复现选择 |
| `configs/research/estimator/{none,secant_ewma,window_linear,window_quadratic}.yaml` | `configs/estimator/` 下同名文件 |
| `configs/research/task/*.yaml`（5 份纯路径包装） | 删除；根组合直接选择 `configs/task/<family>/...` |
| `configs/research/material/{soft,medium,hard,stiff}.yaml` | `configs/material/` 下同名文件 |
| `configs/research/execution/{plan,run}.yaml` 与 `study_execution/{plan,run}.yaml` | 合并为 `configs/execution/{plan,run}.yaml`，由根入口约束单次／study 差异 |
| `configs/research/dm_force_track.yaml`、`dm_admittance.yaml` | 常用选择迁入 `experiment/dm_gripper/force_tracking_{default,admittance}.yaml`；统一入口为 `run.yaml` |
| `configs/studies/*.yaml` 与 `configs/research/study/*.yaml` | 每对合并为一个 `configs/research/<purpose>/study.yaml`，不再保留纯路径 selector |
| `configs/research/*study 根 preset` | 删除独立根包装；统一入口为 `study.yaml`，通过 `research=<purpose>` 选择 |
| `configs/studies/smoke/*.yaml` | 移入测试 fixture，不属于人工正式研究目录 |

原先未有 Hydra preset 的力调度、摩擦估计、Robotiq 离散力、查看、抓取、接触比较和视频入口均已新增
`experiment/<device>/<purpose>` 组合；原 CLI 路径参数已移除。

## 重构前入口与引用盘点

| 类别 | 快照时入口／引用 | 迁移要求 |
| --- | --- | --- |
| Hydra 单次 | `scripts/research/run.py`，仅 `dm_force_track`、`dm_admittance` | 改为 `--config-name run experiment=...` |
| Hydra 正式研究 | `scripts/research/study.py` 与 9 个根 preset | 改为 `--config-name study research=...` |
| 兼容研究脚本 | `force_tracking_controller_comparison.py`、`force_tracking_ablation.py`、`force_tracking_torque_adrc_tuning.py` | 已删除；统一使用 `scripts/research/study.py research=<purpose>/study` |
| CLI／实验默认 profile | `scenes/custom.py`、`custom_demo.py`、force tracking/scheduling/friction、grasp、video | 全部改为统一组合服务产出的冻结对象 |
| 演示脚本 | `scripts/demos/record_experiment_demos.py` | profile/task 路径参数改为组合选择 |
| 测试和增量映射 | `tests/` 中旧路径引用、`scripts/test_changed.py` 的旧 study 映射 | 随每个闭环迁移；最终不得残留失效引用 |

## 研究准入、退出与归档

| 目标 purpose | 旧定义与矩阵 | 分类 | 问题、决策与依据 |
| --- | --- | --- | --- |
| `force_controller_selection` | controller comparison，6×3×3×3 = 162 | 正式保留 | 在任务和材料间选择仍有竞争力的控制器；保留 PID 四变体、stiffness-limit、二阶 torque ADRC |
| `force_controller_ablation` | PID ablation，4×1×3×3 = 36 | 正式保留 | 判断力矩前馈、刚度前馈及交互作用；完整同材料同 seed 配对 |
| `torque_adrc_tuning/coarse` | 34×3×1×1 = 102 | 正式保留 | 先按测量带宽约束筛选可行域 |
| `torque_adrc_tuning/confirm` | 动态候选×3×3×3，通常 135，追加缺席 baseline 时最多 162 | 正式保留 | 验证 coarse 前五名并保持 coarse manifest／ranking 摘要谱系 |
| `stiffness_estimator_validation` | 3×3×3×3 = 81 | 正式保留 | 在固定控制器下比较三个估计器；`secant_ewma` 是历史基线 |
| `dm_admittance_tuning` | 16×1×2 = 32 | 正式保留 | 只回答导纳接近与接触切换参数选择，不并入控制器选型 |
| `robotiq_discrete_force_validation` | 5×4×3×1 = 60 | 正式保留 | 比较离散命令控制器的 HOLD／再激活和逐平台指标 |
| `friction_local_slip_validation` | 5 场景×3 seed = 15 | 淘汰候选，待重建 | 旧多 seed 结论属于旧探测终止时刻和旧载荷调度；当前文档明确其不适用于新检测器 |
| `archive/model_bug_diagnosis` | diagnosis 10 phase 共 39；其中碰撞几何 5 条 | 诊断归档 | 已定位非共面 mesh 与 multiccd 的交互并切换默认模型；保留配置和结论，不进默认自动化 |
| `independent_reproduction/direct_torque` | 历史控制器对照 | 独立复现 | 历史结果显示更快但明显欠阻尼；不回答当前默认选型问题 |
| `independent_reproduction/position_adrc` | 一阶位置式 ADRC | 独立复现 | 控制导向模型与位置弹簧闭环阶次不匹配，已退出正式矩阵 |

诊断配置中的非碰撞 phase 也随整项归档，因为它们服务的是已完成的模型／控制链路因果排查，而不是一个
新的正式决策。若未来形成新问题，应新建目的明确的研究定义并重新评审准入，不得直接把归档矩阵复活。

退出与归档的证据锚点如下：

- `direct-torque` 的 135-run 历史复跑记录在提交
  `85bada4daaf6bde8a9a0f1a374ef811d5f6f60a9`；Step／Ramp／Mixed 相对 `pid-only` 的 RMSE 分别恶化
  37.7%／204.5%／121.0%，且不是饱和造成。提交
  `dd3f6e6cabf57883ae190514af836a826c177714` 将其移出正式矩阵。该证据只支持“当前基线欠阻尼”，
  不代表充分调参后的直接力矩架构上限。
- 碰撞诊断原始汇总为
  `outputs/studies/force_tracking_diagnosis/collision-geometry/20260904T080635Z-f4cd4ddf/summary.csv`。
  `original-mesh-multiccd` 的 RMSE 为 `0.2054 N`、接触塌陷 17 次，其余四个单因素控制 RMSE 为
  `0.0885～0.0945 N` 且无塌陷。提交 `37cc30e39e9364abfc46485af250f06703098287` 切换默认模型；
  正式结论见 `docs/tactile-conventions.md#collision-geometry-conclusions`。
- 旧 Torque ADRC coarse 排名位于
  `outputs/studies/force_tracking_torque_adrc_tuning/coarse/20260904T080731Z-87b82464/candidate_ranking.csv`。
  该产物早于当前生命周期 schema，缺少现行 confirm 所需的状态和科学哈希，只作为历史证据保留，
  不能宣称自动兼容或直接恢复。
- 局部起滑退出依据为 `docs/friction-estimation.md`“历史结论的适用范围”：旧 15 条矩阵的探测终止时刻和
  载荷调度已改变，不能作为当前检测器的正式泛化结论。

## 基线验收

迁移前捕获工具只服务一次性冻结，旧输入删除后不再作为运行工具保留。使用以下命令核对入库证据：

```bash
sha256sum tests/baselines/configuration_refactor_v1.json
```

后续等价测试必须以已入库快照为期望值，不能调用新组合实现生成期望值。
允许的差异只有本文显式映射的来源路径、组合名和已评审矩阵裁剪；每个允许差异都要单独断言，不能删除
字段或放宽数值容差来让比较通过。

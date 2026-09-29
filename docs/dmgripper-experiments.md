# DMgripper 分阶段真机抓取

本轮按物体 1（奶龙玩偶）、物体 2（水瓶）、物体 3（硬质方形盒子）的顺序开展。
每个物体依次完成 **人工选择预载力 → 人为滑移扰动标定 → 冻结参数的自适应抓取**。
这里的“预接触力”指双侧接触后建立的初始预载力；真正尚未接触时不存在接触力。
程序负责跟踪和记录，人决定合适的预载、起滑时间及最终接受的摩擦先验。

默认只检查配置。`--execute` 才连接 DM4310P 与双侧 PapillArray。
运行前核对机械限位、编码器零偏、回零方向和设备急停，并能承接物体。
预检会按需自动张开回零，故启动时先移开物体并保持传感器空载；到 `ready` 后再放入物体。

## 物体与条件

| 编号 | 对象 | 配置 | 需要固定并记录的条件 |
| --- | --- | --- | --- |
| 1 | 奶龙玩偶 | `configs/hardware/dmgripper/object_1.yaml` | 抓取部位、朝向、填充状态、是否由外部支撑。 |
| 2 | 水瓶 | `configs/hardware/dmgripper/object_2.yaml` | 同一瓶体、瓶盖状态、水量或总质量、抓取高度与朝向。 |
| 3 | 硬质方形盒子 | `configs/hardware/dmgripper/object_3.yaml` | 盒体表面、总质量、接触面及抓取位置。 |

三个模板都从 `stage: preload` 开始，数值只是试验候选，左右摩擦先验尚未标定。
玩偶以 `0.5 N/侧` 作为首个候选，在已确定的物体边界内人工比较 `0.5–1.0 N/侧`；
另外两类物体须根据实际质量和耐受重新试选。名称不会暗中改变控制器。
用 `metadata.condition` 记录实际条件，不把不同水量、接触面或姿态的标定混为一组。

## 阶段一：人工确定预载力

```sh
# 检查候选配置，不连接设备。
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/object_1.yaml

# 空载启动；完成 ready 后放入物体，在终端输入 start。
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/object_1.yaml --bias --execute
```

`preload` 阶段在建立双侧接触后平滑到达 `reference.initial_force_n`，然后保持固定目标。
它不通过刚度、深度或未经确认的摩擦估计自动增加目标。一次试验结束，先承接物体，输入
`release` 并按 Enter，等待回位与失能；在两次试验之间修改候选力，避免把多次试选混成一次已标定试验。

选择依据是实测左右力、持续接触、物体变形和操作者观察。预载建立门槛仅用于推进状态，
不等于“最佳底力”或“可稳定承重”。如果左右明显不平衡，先调整接触位置。
记录是否完全悬空、有无滑移与变形，以及观察时长；手托物体时不能据此宣称独立承重。

接受一个预载后，在该物体 YAML 保留选定的 `reference.initial_force_n`，填写
`reference.preload_source` 为对应运行目录／试验标识及选择说明，然后改为 `stage: friction`。
`friction` 和 `adaptive` 均要求这项来源，避免把候选模板冒充已经完成的标定。
切换阶段时同步修改 `metadata.task_name`，例如 `object-1-friction`、`object-1-adaptive`，
便于在输出目录中区分三类试验；已完成运行的配置快照保持原样。

## 阶段二：人为滑移扰动与摩擦候选

使用同一物体、接触条件和已选择的预载，重新空载启动。
`stage: friction` 仍使用固定目标力；进入 `active` 后，操作者缓慢施加切向扰动，观察物体
相对于左右指面的运动。在明确观察到滑动时输入相应命令并按 Enter：

| 命令 | 人工观察含义 |
| --- | --- |
| `slip-left` | 确认左侧发生相对滑动。 |
| `slip-right` | 确认右侧发生相对滑动。 |
| `slip-both` | 确认两侧都发生相对滑动。 |

这些命令只标记事件，不自行改变目标力，也不代表程序已独立检测起滑。
若只确认一侧，另一侧保留未知，不能复制已测一侧的值凑齐标定。
持续失接触、原始过力、压缩行程、触觉失鲜及电机故障保护仍有效。
一次清晰扰动可用于产生首个候选；没有清楚观察到起滑，便没有本次起滑标定依据。

正常释放并结束记录后执行：

```sh
uv run --package dmgripper-experiments dmgripper-calibrate-friction \
  outputs/real/<本次标定运行目录>

# 若人工按键有反应延迟，可结合 MCAP 或同步视频修正时间；时间为本次实验 time_s。
uv run --package dmgripper-experiments dmgripper-calibrate-friction \
  outputs/real/<本次标定运行目录> \
  --slip-time-s 12.3 --side left --window-s 0.2 \
  --output outputs/real/<本次标定运行目录>/friction_candidate_reviewed.json
```

默认输出 `friction_candidate.json`，已有文件不覆盖。分析使用起滑前窗口内分侧实测合力：

\[
T_i=\sqrt{F_{x,i}^2+F_{y,i}^2},\qquad r_i=T_i/F_{z,i},\quad i\in\{L,R\}.
\]

每个窗口取比值的中位数，并保存样本数、实际样本时间范围、极值与标准差；多次标记再取
各窗口候选的中位数。候选文件保留来源、物体条件、预载、事件侧别及人工时间修正，未知侧为 `null`。
分析按流读取 MCAP，只保留选中短窗口的触觉样本。窗口内标记侧失接触或三轴力无效时拒绝生成候选，
需要缩短窗口或重新确认时间。窗口和统计量是公开的取样口径，不能消除按键延迟或传感器误差。
不要把长时间滑动后的比值当作起滑瞬间静摩擦；窗口也不能跨越空载、释放或另一段接触。

本工具得到的是**该抓取条件下的有效摩擦／承载先验候选**。
局部起滑时的整侧比值通常不等于滑动触点的材料摩擦系数；柔软玩偶和曲面水瓶也可能存在
形变与接触力矩影响。操作者查看时序并接受候选后，将数值写入
`reference.friction.left_mu`、`right_mu`，用 `reference.friction.source` 记录候选文件和选择说明。
工具不自动修改 YAML，不暗中乘固定折减系数，也不将一次扰动称为普适材料标定。

## 阶段三：冻结先验的自适应抓取 {#unified-adaptive}

确认 `reference.initial_force_n`、`reference.preload_source`、左右 `mu` 及
`reference.friction.source` 后，将 `stage` 改为 `adaptive`。
缺少任一侧先验或标定来源时，配置检查失败；不会回退到未经标定的 `0.1`。
整个自适应试验保持这组预载与摩擦值不变，参数快照随每次运行保存。

目标调度沿用分侧载荷公式：

\[
F_{\mathrm{raw}}=\gamma\max(T_L/\mu_L,T_R/\mu_R),
\qquad F_{\mathrm{goal}}=\operatorname{clip}(F_{\mathrm{raw}},F_0,F_{\max}).
\]

其中 `F_0` 是选定预载，`gamma` 是显式安全倍率。目标再经增／退力速率限制，交给固定参数的
MIT＋导纳力跟踪。每侧先求切向合力再取模，不能用逐触点切向模之和替代。
论文中的安全倍率和最小力不能直接照搬成这三个物体的耐受标定。

`capacity_limited` 表示按冻结先验计算的需求超过配置目标上限；它保留为诊断、事件与运行摘要，
目标仍受限，不再因持续 `0.5 s` 就自动宣称实际滑落。真实失接触、过力和通信异常仍触发保护。
操作者依据是否悬空、滑移、损伤及扰动后的恢复评价抓取结果。
`manifest.status: completed` 仅表示运行正常释放和收尾；没有人工评价时，科学结果仍是未评估。

## 人工命令与设备生命周期

输入 `start`／`s`、`status`、`release`／`r` 后按 Enter；起滑标记仅用于摩擦标定阶段。
`--terminal plain` 可切换为文本终端；`--terminal auto` 自动选择显示方式。
有限时长且 `lifecycle.on_finished: return` 的组合会自动启动并自动回位，因此现场人工放物体的
流程应使用交互运行，不启用这个组合。

正常顺序为 `preparing` → `ready` → 必要的 `homing` → `approach` →
`contact_transition` → `preload` → `active` → 人工 `release` → `returning` → `completed`。
这里运行状态 `preload` 和整场试验 `stage: preload` 不同：前者只指初始力建立过程，后者表示
建立后仍持续保持固定力的试选任务。

预检先检查电机反馈与 MIT 模式，必要时受限回零并失能，再验证触觉。
`--bias` 在首个完整包后只发一次清零命令，清除旧缓冲与滤波状态，等待 `2 s` 后验证空载。
`0.5 s` 窗口中左右滤波法向力绝对值的均值都须不超过 `0.1 N`；最多等待 `5 s`。
非有限值、坏帧、原始过力等检查独立生效。清零无效时先检查设备零点与安装，不用未知偏置做标定。

命令位置限制为 `[0, pi/2] rad`，反馈范围默认两端各扩展 `0.05 rad`；home 默认 `0 rad`、
容差 `0.03 rad`。反馈余量不会扩大命令行程。接触后的总闭合增量受
`safety.max_contact_compression_m` 约束，同时检查实际位置、命令和下一步预测。
该总压缩量含物体、指垫和机构变形，不是单独的材料压入量。

| 故障 | 处理 |
| --- | --- |
| 使能前预检失败 | 退出；已尝试使能时尽力失能。 |
| 使能后触觉、失接触、过力、控制／记录故障，且 DM 仍可控 | 停止继续闭合，建立 `fault_holding`，等待人工承接后 `release`。 |
| DM 反馈或通信丢失、短写、电机故障、意外失能、反馈越界、保持失败 | 尽力失能并退出。 |

故障释放只依赖 DM，受限回到 home 后确认失能。即使回位成功，原始故障仍保存在 manifest。
`Ctrl+C` 立即尽力失能清理，可能松开物体，不能当作常规 `release`。
软件动作受通信与固件时延影响，不替代现场硬件急停。

## MCAP 记录与时间回放

每次运行保存 `config.json`、`recording.mcap` 和 `manifest.json`，结束后按可用轨迹生成图。
真机实验与独立 `papillarray-record` 共用 ZSTD 分块压缩、消息索引和内嵌 JSON Schema。
`/tactile` 保存双侧与逐触点数据，`/trace` 保存目标、实测力和电机轨迹，`/events` 保存阶段、
人工起滑标记和故障。三个主题由同一起始时钟映射为 UTC 纳秒时间，不采用后台落盘时刻。

在 Foxglove 打开或拖入 MCAP，添加 Plot 面板后选择：

- `/trace.measured_force_n`、`/trace.target_force_n`：平均单侧实测力与目标。
- `/tactile.left_force_n`、`/tactile.right_force_n`：双侧法向力。
- `/tactile.left_taxel_forces_n[0][2]`：左侧触点 0 的法向力。

Raw Messages 面板选择 `/events`。默认 Log time 可与控制、触觉曲线同步查看。
设备 `timestamp_us`、`raw_timestamp_us`、主机 `received_at_s` 和控制 `time_s` 均保留。
格式原理见 [Foxglove 自定义数据](https://docs.foxglove.dev/docs/getting-started/custom/custom-schema-encodings)。

时序经有界队列送往单个后台写线程，不双写 CSV／JSONL、不截断原始浮点。
默认 `recording.tactile_stride: 4`，在设备 `1000 Hz` 下记录约 `250 Hz`；首包和异常包仍保留。
需要全速标定记录时显式设为 `1`，同一组比较保持一致。默认最长 `600 s`、压缩文件软上限
`512 MiB`，触发后按受限释放收尾并记录原因；最终块及索引可略超体积上限。
关闭会排空队列并完成索引；强制杀进程不保证最后未落盘的数据和索引完整。

旧 JSONL／CSV 运行数据与旧统一自适应、粒子路线 YAML 不做兼容。
已清理的探索性运行不作为新的三物体标定来源。需要复现历史方法时使用对应 Git 版本与其数据。

## 模块取舍与文献边界

| 模块 | 本次真机路线 | 理由 |
| --- | --- | --- |
| 人工选预载、起滑标定、冻结分侧摩擦调力 | 保留 | 可明确区分试选、标定与执行，参数来源可核对。 |
| 深度到摩擦的经验对数曲线 | 移除 | 没有验证当前映射及参数的直接证据，总闭合量也不是材料摩擦的唯一观测。 |
| 粒子摩擦后验、纯力启发式事件在线改摩擦 | 移除 | 当前似然、事件判据和误差未经本硬件标定；与文献里的检测器及观测模型不同。 |
| 刚度自动选底力、在线改变导纳质量／阻尼 | 移除 | 通用研究不能直接证明当前拼接控制律的效果或稳定性。 |
| 刚度估计 | 可选诊断，默认关闭 | 不参与当前控制，也不解释为材料弹性模量。 |
| 过力、行程、失接触、设备故障保护 | 保留 | 属于独立执行边界。 |

Khamis 等的 ICRA 2021 原始工作支持分侧载荷与摩擦调力，并说明局部起滑时整侧比值的局限；
其条件不能直接覆盖玩偶柔顺接触和瓶体曲面。[作者原稿](https://contactile.com/wp-content/uploads/2022/01/KhamisEtAl2021_ICRA2021_Manuscript_preprint.pdf)。
纯力时序检测也有学习研究，但其训练标签与验证不能替代本仓库的启发式阈值验收。
[Wang 等原文](https://arxiv.org/abs/2307.04011)。

“从本轮移除”不代表整个研究方向没有价值；仿真共享核与历史报告仍保留原适用条件，
没有把候选算法删除出历史，也没有把本轮简化路线称为已完成真机验证。

# DMgripper 通用抓取实验

`dmgripper-run` 是 DM4310P 平行夹爪与双侧 PapillArray 的通用真机抓取入口。它不使用 Hydra，
也不运行 MuJoCo；Hydra 配置和 `pgt` 入口仍只用于仿真研究。默认数值只是便于启动和审阅的配置，
尚未经真机验证，不应视为硬件验收参数。

执行实验前必须准备承接容器、确认硬件急停可用，并使操作者始终能够承接被夹物。软件的
`fault_holding`、回位和尽力失能都受 USB、串口、USB2CAN 与固件时延限制，不能替代硬件急停或
功能安全互锁；任何异常失能或保持丢失都可能使物体松脱。

## 生命周期

一次运行共用一条生命周期：`preparing`（预检与零力验证）→ `ready`（等待 `start`）→
必要时 `homing`（受限自动回零）→ `approach`（受限闭合接近）→
`contact_transition`（平滑衰减接近速度）→ `preload`（建立
初始抓力并学习基线）→ `active`（目标策略启用）→ `holding`（任务计时完成，保持抓握）→
`returning`（显式 `release` 后受限张开）→ `completed`。使能前收到 `release` 记为
`cancelled`，不发送任何运动命令。使能后的可保持实验故障进入非终态 `fault_holding`；不可保持的
底层故障进入 `fault` 并立即尽力失能。运行阶段与实验结果分开：从 `fault_holding` 人工释放并成功
回位后，阶段可以到达 `completed`，本次实验结果仍是 `failed`，原始故障也不会被改写。

- 在 `ready` 阶段输入 `s`/`start`、`status` 或 `r`/`release` 后必须按 `Enter` 提交。快捷输入在输入层展开为完整命令；运行时收到 `start`
  后先向终端与 `events.jsonl` 写入 `command_received`，再完成电机连接、反馈／机械行程／MIT 模式
  检查与使能确认；使能报文发送前即登记“可能已使能”，确认丢失仍会尽力失能，只有确认成功才
  发送接近命令。未知命令会显示可用命令，后台标准输入读取失败会明确终止并进入安全清理。
- `preload` 的目标：曲线模式取曲线首值，动态模式取初始抓力；确认双侧接触且平均力
  连续高于 `目标-preload_tolerance_n` 后进入 `active`，不再设置预载高侧稳定窗口。动态策略在进入 `active` 时冻结基线，
  此后的载荷变化不再移动基线。
- 失接触判据（`any_side`／`both_sides`）与处理动作（`fault`／`reapproach`）分开配置；默认
  `any_side + fault`。第一版仅曲线模式支持 `reapproach`：任务时间在重接近期间暂停，恢复接触
  后先稳定到暂停点目标再继续剩余曲线，接触段编号递增。
- 正常任务结束默认 `on_finished: hold`，保持闭环抓握等待显式释放；动态模式在 `holding`
  阶段继续响应新的载荷增长。`return` 必须在配置中明确选择。

## 启动位置与自动回零

DM 的目标命令工作范围与反馈安全范围彼此独立。当前命令仍严格限制在 `[0, pi/2] rad`；编码器反馈
则允许在两端各超出 `hardware.feedback_position_margin_rad`，默认安全范围为
`[-0.05, pi/2+0.05] rad`。扩展范围只用于判断反馈是否仍可控，不能用于放宽目标命令。

当前默认 `hardware.home_position_rad=0.0`、`hardware.home_tolerance_rad=0.03`、
`hardware.feedback_position_margin_rad=0.05`。输入 `start` 后，运行时依次打开 DM、检查反馈与故障、
确认 MIT 模式和开始前失能状态，再使能并检查当前位置：位于 home 容差内时直接进入 `approach`；
否则先复用受限回位速度、加速度和加加速度执行 `homing`，达标后才开始接近。回零命令始终位于命令
工作范围，且回零只依赖有效 DM 反馈，不依赖触觉接触状态。反馈越出扩展安全范围时禁止自动运动；
回零超时但 DM 命令与反馈仍健康时进入 `fault_holding`。

若编码器反馈略低于 `0` 且已超出 home 容差，负角度本身不能作为合法目标发送；首个 home 位置目标
会投影到工作范围下界 `0`，但 MIT 合成力矩仍受回位力矩限幅约束。该边界情形必须在首次真机验收中
以无负载、小零偏逐步确认，不能把反馈安全余量理解为允许发送负角度命令。

这三个默认值是待真机验收的保守起点，不是已经证明的机械安全参数。首次执行前必须核对编码器零偏、
真实硬限位、回零方向、容差和安全余量。

实验配置同时把关节速度、跟踪／回位力矩限幅和 MIT `kp`／`kd` 限制在 DM4310P 协议量程内；硬件
会话在每次编码前再次拒绝越界字段，不允许底层协议的静默饱和改变共享控制核已检查的力矩语义。

## 使能前零力验证

默认启用 `lifecycle.verify_zero_force`。预检期间电机保持未使能。传入 `--bias` 后的真实顺序为：
建立稳定数据流并收到首个完整有效包，只发送一次 bias，丢弃且不记录 bias 前包，重置双侧滤波器和
设备时间／包计数基线，等待 bias 后的新包，再等待 `timing.tactile_bias_settle_s`，最后从 settle 后的
新包开始零力窗口验证。验证窗口不会混入 bias 前数据。

随后在 `lifecycle.zero_force_stable_s` 时间窗口内，分别计算双侧滤波全局 `Fz`
绝对值的均值；仅当左右均值均不大于 `lifecycle.zero_force_threshold_n` 时，才会通过零力门禁并进入
`ready`。

每帧仍会硬性检查传感器数量、全局三轴力、逐侧 taxel 数量与结构、每个 taxel 的三轴有限性、包计数
前进规则、设备时间递增和数据新鲜度；taxel 数在同一次采集会话中变化也会阻断启动。逐 taxel 力幅值
本身不使用未经标定的统一阈值，也不参与零力通过判定。门禁通过时，`zero_force_diagnostics` 事件会
记录逐侧 taxel 数、三轴均值和标准差、最大绝对残余力及其 taxel 编号、轴向和有符号值，供离线标定。

当均值门禁通过时，若整个验证期内任一侧滤波 `Fz` 峰值达到 `lifecycle.contact_on_n`，运行时会在
进入 `ready` 前向终端和 `events.jsonl` 输出一次结构化 `warning`（包含最大峰值、触发侧和告警阈值）。
峰值不会重置均值窗口，也不会单独阻止进入 `ready`。该告警不表示可以带载运行；应检查传感器、夹具
和线缆是否仍有接触或预紧。

这项调整不改变其他阻断性保护：触觉非有限值、数据过期、原始法向力超过
`safety.force_ceiling_n`、以及双侧原始法向力不平衡仍会使预检失败。进入运行阶段后的接触、失接触、
过力与退出失能保护也保持不变。

## 故障保持与人工释放

实验失败后是否保持取决于 DM 通道是否仍可控，不能把所有异常归为同一种策略。

- 使能前的配置、输出目录、触觉启动／零力门禁、DM 打开、初始反馈、MIT 模式或开始前状态错误不会
  使能电机；已发送过使能请求但未确认时只尽力失能，不声称进入保持。
- 使能后的触觉过期、断流、非有限或时序异常，接触寻找失败、preload 超时、持续失接触、原始过力或
  双侧严重不平衡，控制算法／目标策略／记录异常，以及 DM 仍健康时的 homing 或 returning 超时，
  会停止继续闭合和目标更新，并尝试进入 `fault_holding`。
- DM 命令短写、通信或新反馈丢失、电机故障或意外失能、反馈越出扩展安全范围、无法建立／继续保持，
  以及故障保持期间失去 DM 控制权都不可保持，会立即尽力失能和关闭设备。

只有当前位置保持命令成功发送并收到新的有效使能反馈后，运行时才记录已经进入 `fault_holding`。
保持请求由最新有效反馈生成并投影到命令工作范围，强制 `dq=0`、前馈力矩为零，并继续经过位置、增益
与合成力矩限制。故障保持不再读取或依赖已经失效的触觉数据，只持续检查 DM 命令与反馈。操作者应先
承接物体，再输入 `release`；`status` 只报告状态，未知命令只告警。

`release` 后使用同一条 DM-only 受限轨迹返回 home，随后才尝试失能。回位未在时限内到达 home、但
仍能重新建立保持时，会返回 `fault_holding` 并要求操作者检查后再次输入 `release`；保持或回位期间
DM 通道失效则记为 `hold_lost` 并立即尽力失能。即使最终回位和失能成功，原始实验结果仍为
`failed`。`Ctrl+C` 是明确的紧急软件退出，会跳过普通人工释放等待并立即进入尽力失能清理；它不是
`release` 的替代命令。

## 目标力来源

`reference` 段二选一：

- `curve`：`hold`／`linear`／`smoothstep` waypoint 曲线。首点必须是零时刻，时间严格递增；
  任务时长取末节点。曲线模式允许减力，但最小力不得低于失接触阈值；含下降段的曲线与导纳
  `prevent_unloading` 互斥，计划阶段报错。
- `adaptive`：复用共享核 `TactileDisturbancePolicy`。切向输入为两侧各自 `hypot(Fx,Fy)` 之和，
  法向目标为平均单侧力；只在新触觉观测到来时更新，持续恒定载荷不会无界增力。
  权威真机配置保留导纳 `prevent_unloading`，并把预载高侧容差单独设为 `0.3 N`；因此 `0.5 N`
  初始目标允许在平均力 `0.35–0.8 N` 内稳定，但低侧要求及机械限位、失接触、过力保护不变。

## 控制器

`controller.kind` 独立选择 `admittance`、`pid` 或一阶位置型 LADRC `adrc`。导纳消费外层滤波
力，死区与单向闭合是导纳专属参数；PID／LADRC 把原始力交给共享核 `begin_tracking`／
`step_tracking` 接口，由核心内部做唯一一次低通。trace 同时记录原始三轴力、外层滤波力与控制
使用力，滤波时延可追溯。

`preload` 超时会报告目标、当前双侧力、当前均值和允许区间；若导纳因 `prevent_unloading=true` 无法
纠正过高抓力，错误中也会直接标明该配置线索。

## 刚度估计

`estimation` 默认 `enabled: true`、`method: window_linear`，但控制消费为 `none`：估计量、
有效性、更新时刻与原因（初值、样本不足、激励不足、拟合退化、保持旧值）只写入 trace 与终端，
不改变控制命令。控制器显式选择 `stiffness_consumption: feedforward`（仅 PID／LADRC）时才
消费估计前馈；估计初值与门限沿用仿真验证起点，尚不是真机辨识值。

## 配置与 dry-run

配置合并顺序：dataclass 默认值 → 严格 YAML → Tyro 命令行覆盖 → 完整验证与冻结。YAML 只
允许已登记字段；未知字段、非有限数值与不相容组合一律拒绝。`--execute`、`--bias` 与
`--output` 是操作参数，不写入 YAML。

```sh
# 只验证配置并输出计划
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/force_curve.yaml

# 交互终端执行；运行时输入 s/start、status、r/release，并按 Enter 提交
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/adaptive_grip.yaml --bias --execute

# 在 YAML 之上选择控制器
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/adaptive_grip.yaml --controller.kind pid
```

非交互执行只允许无人值守组合（`auto_start=true` 且 `on_finished=return`）。

`terminal.mode=auto` 会同时检查 TTY 和 Rich 的终端能力；`TERM=dumb`／`unknown`、重定向输出或
Rich 运行时失败时使用纯文本。Rich 模式由唯一 Live 区域按 `terminal.refresh_hz` 合并更新预检、
快照和事件；底部固定 `命令>` 输入区接管逐字符显示与退格，不与动态面板争抢终端回显，退出时恢复
原终端属性。纯文本快照同样按该频率合并，可显式传入 `--terminal.mode plain` 只改变显示模式。
Rich 结束后会在动态区域之外输出最终状态、失能确认和运行目录。主故障或 `Ctrl+C` 同时伴随清理
故障时，CLI 保留原始错误，同时在标准错误中醒目标出清理失败和运行记录目录。

## 记录与重绘

输出目录为 `outputs/real/<task_name>/<object_name>/<UTC时间戳>-<run_id>/`，包含：

```text
├── config.json      # 输入路径、元数据与完整有效配置
├── events.jsonl     # 阶段、接触、目标启用、故障等结构化事件
├── tactile.jsonl    # 全局力、逐 taxel 三轴原始力及包连续性诊断
├── trace.csv        # 控制周期 trace（schema dmgripper-experiment/v1）
├── manifest.json    # 状态、失能确认、原始故障、清理故障与产物清单
└── plot.pdf / plot.png
```

`tactile.jsonl` 在原有快照字段上兼容新增 `left_taxel_forces_n`、`right_taxel_forces_n`、
`counter_event` 和 `counter_gap`；旧读取器可以忽略这些新增字段。逐 taxel 数值不会擅自平滑、删点或
改变全局 `Fz` 的零力统计口径。

manifest 保留原有 `error`，同时为失败运行新增兼容字段 `primary_error`、`fault_phase`、
`fault_holding_entered`、`fault_holding_duration_s` 和 `fault_resolution`。其中
`fault_resolution` 可区分 `released`、`forced_disable` 与 `hold_lost`；`disable_confirmed` 继续区分
`true`／`false`／`not_applicable`，只有读回失能状态才写 `true`，发送过失能命令并不等于已确认。
设备正常结束但绘图失败时记为 `post_processing_error`，不掩盖已完成的控制结果。
失能、串口关闭、触觉采集停止或终端显示退出中的任一清理故障会把运行标记为 `failed` 并使
命令返回非零状态；原始运行故障仍保留为主错误，清理故障单独列入 `cleanup_errors`。失能设备
操作与确认事件记录分开处理，`disable_confirmed` 不会因显示或记录失败被误写为 `false`。

```sh
# 重绘（默认写回源目录并登记 manifest）
uv run --package dmgripper-experiments dmgripper-plot <运行目录>
# 离线重绘：写入独占 repaint 目录，保留源 trace 与原 manifest
uv run --package dmgripper-experiments dmgripper-plot --repaint <运行目录>
```

历史 cup 记录（v1／v2）继续可读：其 `state` 列按 `phase` 解释，可用同一绘图命令重绘。

## 迁移

旧入口 `dmgripper-force-demo` 与 `dmgripper-cup` 打印迁移提示并拒绝执行；旧场景交互不会
映射成自动阶段推进。`dmgripper-cup-plot` 是通用历史读取器的薄别名。旧 cup trace 的字段与阶段
语义见[历史数据说明](archive/dmgripper-cup.md)。

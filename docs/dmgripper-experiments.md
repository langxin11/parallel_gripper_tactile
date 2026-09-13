# DMgripper 通用抓取实验

`dmgripper-run` 是 DM4310P 平行夹爪与双侧 PapillArray 的通用真机抓取入口。它不使用 Hydra，
也不运行 MuJoCo；Hydra 配置和 `pgt` 入口仍只用于仿真研究。默认数值只是便于启动和审阅的配置，
尚未经真机验证，不应视为硬件验收参数。

执行实验前必须准备承接容器、确认急停可用，并使操作者始终能够承接被夹物。任何异常导致的尽力
失能都可能使物体松脱。

## 生命周期

一次运行共用一条生命周期：`preparing`（预检与零力验证）→ `ready`（等待 `start`）→
`approach`（受限闭合接近）→ `contact_transition`（平滑衰减接近速度）→ `preload`（建立
初始抓力并学习基线）→ `active`（目标策略启用）→ `holding`（任务计时完成，保持抓握）→
`returning`（显式 `release` 后受限张开）→ `completed`。使能前收到 `release` 记为
`cancelled`，不发送任何运动命令；故障进入 `fault`，保留原因并尽力失能。

- 在 `ready` 阶段输入 `start`、`status` 或 `release` 后必须按 `Enter` 提交。运行时收到 `start`
  后先向终端与 `events.jsonl` 写入 `command_received`，再完成电机连接、反馈／机械行程／MIT 模式
  检查与使能确认；使能报文发送前即登记“可能已使能”，确认丢失仍会尽力失能，只有确认成功才
  发送接近命令。未知命令会显示可用命令，后台标准输入读取失败会明确终止并进入安全清理。
- `preload` 的目标：曲线模式取曲线首值，动态模式取初始抓力；平均力持续位于
  `[目标-preload_tolerance_n, 目标+preload_overforce_tolerance_n]` 后进入 `active`。上下容差
  分开配置，允许只接纳温和的高侧过冲而不放宽低侧抓力要求。动态策略在进入 `active` 时冻结基线，
  此后的载荷变化不再移动基线。
- 失接触判据（`any_side`／`both_sides`）与处理动作（`fault`／`reapproach`）分开配置；默认
  `any_side + fault`。第一版仅曲线模式支持 `reapproach`：任务时间在重接近期间暂停，恢复接触
  后先稳定到暂停点目标再继续剩余曲线，接触段编号递增。
- 正常任务结束默认 `on_finished: hold`，保持闭环抓握等待显式释放；动态模式在 `holding`
  阶段继续响应新的载荷增长。`return` 必须在配置中明确选择。

## 使能前零力验证

默认启用 `lifecycle.verify_zero_force`。预检期间电机保持未使能；若传入 `--bias`，运行时会先完成
触觉清零和稳定等待。随后在 `lifecycle.zero_force_stable_s` 时间窗口内，分别计算双侧滤波 `Fz`
绝对值的均值；仅当左右均值均不大于 `lifecycle.zero_force_threshold_n` 时，才会通过零力门禁并进入
`ready`。

当均值门禁通过时，若整个验证期内任一侧滤波 `Fz` 峰值达到 `lifecycle.contact_on_n`，运行时会在
进入 `ready` 前向终端和 `events.jsonl` 输出一次结构化 `warning`（包含最大峰值、触发侧和告警阈值）。
峰值不会重置均值窗口，也不会单独阻止进入 `ready`。该告警不表示可以带载运行；应检查传感器、夹具
和线缆是否仍有接触或预紧。

这项调整不改变其他阻断性保护：触觉非有限值、数据过期、原始法向力超过
`safety.force_ceiling_n`、以及双侧原始法向力不平衡仍会使预检失败。进入运行阶段后的接触、失接触、
过力与退出失能保护也保持不变。

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

# 交互终端执行；运行时输入 start、status、release，并按 Enter 提交
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/adaptive_grip.yaml --bias --execute

# 在 YAML 之上选择控制器
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/adaptive_grip.yaml --controller.kind pid
```

非交互执行只允许无人值守组合（`auto_start=true` 且 `on_finished=return`）。

`terminal.mode=auto` 会同时检查 TTY 和 Rich 的终端能力；`TERM=dumb`／`unknown`、重定向输出或
Rich 运行时失败时使用纯文本，避免动态控制序列导致提示不可见。Rich 模式关闭库内自动刷新线程，
预检与 `ready` 使用静态面板，首个控制快照产生后才由实验显示线程按 `terminal.refresh_hz` 启动
Live 合并更新，并保持输入光标可见。纯文本快照同样按该频率合并。若运行期输入回显仍受终端动态
重绘影响，可显式传入 `--terminal.mode plain`；这只改变显示，不改变控制或记录。
Rich 结束后会在动态区域之外输出最终状态、失能确认和运行目录。主故障或 `Ctrl+C` 同时伴随清理
故障时，CLI 保留原始错误，同时在标准错误中醒目标出清理失败和运行记录目录。

## 记录与重绘

输出目录为 `outputs/real/<task_name>/<object_name>/<UTC时间戳>-<run_id>/`，包含：

```text
├── config.json      # 输入路径、元数据与完整有效配置
├── events.jsonl     # 阶段、接触、目标启用、故障等结构化事件
├── tactile.jsonl    # 原始触觉包
├── trace.csv        # 控制周期 trace（schema dmgripper-experiment/v1）
├── manifest.json    # 状态、失能确认、原始故障、清理故障与产物清单
└── plot.pdf / plot.png
```

manifest 的 `disable_confirmed` 区分 `true`／`false`／`not_applicable`（使能前取消）。
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

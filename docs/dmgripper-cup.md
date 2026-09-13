# DMgripper 真机倒水实验（历史文档）

> **状态：历史文档。** `dmgripper-cup` 入口已被通用抓取实验 `dmgripper-run` 取代并拒绝执行。
> 本文仅用于解释历史 v1／v2 cup 运行记录（阶段名、交互语义与 trace 字段）；当前操作方法见
> [`dmgripper-experiments.md`](dmgripper-experiments.md)。历史 `pour` 时间不应解释为新
> `active` 任务时间。

`dmgripper-cup` 是 DM4310P 平行夹爪与双侧 PapillArray 的真机倒水流程入口。它不使用
Hydra，也不运行 MuJoCo；Hydra 配置和 `pgt` 入口仍只用于仿真研究。倒水流程的默认数值只是便于
启动和审阅的配置，尚未经真机验证，不应视为硬件验收参数。

执行实验前必须准备承接容器、确认急停可用，并使操作者始终能够托住杯子。任何异常导致的尽力失能都
可能使杯子松脱。

## 配置与 dry-run

配置按以下顺序合并：`CupConfig` dataclass 默认值、严格 YAML 文件、Tyro 命令行覆盖。YAML 只允许
已登记的配置字段和嵌套字段；未知字段、非有限数值、用字符串代替数值，以及将布尔值当作数值都会被
拒绝。`--execute`、`--bias` 和 `--output` 是操作参数，不能写入 YAML。

不带 `--execute` 时命令只输出解析后的配置、端口和输出目录，不导入真机运行时，也不访问设备：

```sh
uv run --package dmgripper-experiments dmgripper-cup \
  --config configs/hardware/dmgripper/cup.yaml
```

可在 YAML 之上选择外层控制器并覆盖嵌套字段：

```sh
uv run --package dmgripper-experiments dmgripper-cup \
  --config configs/hardware/dmgripper/cup.yaml \
  --controller pid --control.target-force-n 0.6
```

`--controller` 只接受 `admittance`、`pid` 或 `adrc`。三种控制器的外层力调节不同，但电机内层都使用
MIT 协议。默认配置可改为 `--controller admittance` 或 `--controller adrc`，并可继续用 Tyro 的
点号参数覆盖任意配置字段，例如 `--grip.max-target-force-n 1.2`。配置会拒绝超过力上限、低于接触确认
阈值或违反抓握安全关系的组合。

常用字段的位置与单位如下。参数在启动时固定；切换控制器需要结束当前实验后重新启动。

| YAML 字段 | 含义 |
| --- | --- |
| `controller` | `admittance`、`pid` 或一阶位置型 LADRC `adrc` |
| `control.target_force_n` | 初始平均单侧法向目标力，默认 0.5 N |
| `grip.max_target_force_n` | 平均单侧目标上限，默认 1.5 N |
| `control.force_ceiling_n` | 任意一侧原始法向力的绝对值保护上限，默认 2 N |
| `grip.max_force_rate_n_s` | 目标力增长速率上限，默认 0.5 N/s |
| `grip.force_deadband_n` | 导纳力误差死区，默认 0.1 N；进入死区后冻结期望闭合量与虚拟速度 |
| `grip.prevent_unloading` | 导纳抓握阶段禁止反向卸载，默认启用；显式 `release` 不受影响 |
| `control.tracking_duration_s` | 从提示倒水开始计时，默认 10 s，可覆盖为 30 s |
| `grip.stable_time_s` | 手托及撤手后的稳定确认时间，默认 2 s |
| `pid.*`、`adrc.*` | 所选外环的参数 |
| `control.admittance_*` | 导纳质量、阻尼和刚度 |
| `control.mit_kp`、`control.mit_kd` | 共用 MIT 内环增益 |

切向输入为两侧各自 `hypot(Fx, Fy)` 的和，法向目标为平均单侧力，两者不可混为总夹持力。
策略保留手托阶段的切向基线，撤手及倒水引起的切向增长会抬高受限目标，停止加水不会主动卸力。
这是触觉载荷增长触发的工程策略，不能仅由这些曲线确认物体没有滑移；本实验没有物体位移测量。

默认导纳路径面向存在打印传动回差的夹爪：平均单侧力低于目标死区下沿时继续闭合；进入死区后
冻结导纳位移并将虚拟速度清零；力高于死区上沿时保持已经建立的闭合量，不因小幅过冲反向穿越
传动间隙。原始单侧力超过保护上限或左右失衡仍会故障失能，正常张开只由操作者托住杯子后输入
`release` 触发。PID 和 ADRC 路径不应用这组导纳约束。

## 执行流程

只有显式传入 `--execute` 才允许使能电机。`--bias` 只应在传感器和夹爪完全空载时使用，用于执行前的
触觉清零；它不能代替人工确认零力、急停和承接措施。
运行时先建立数据流并收到首个有效包，再向 PapillArray 发送 `z\n`、清空协议半包并重置 10 Hz
低通状态；随后保持空载等待 2 s，再开始连续零力验证。这与 ROS 2 运行中通过服务向已经配置好的
串口线程发送 bias 的时序等价，不需要在 `z\n` 后重新发送采样率命令。

```sh
uv run --package dmgripper-experiments dmgripper-cup \
  --config configs/hardware/dmgripper/cup.yaml \
  --controller admittance --bias --execute \
  --output outputs/real/cup_trial_001
```

执行需要交互终端，标准输入关闭会触发故障清理。运行时按下列交互顺序推进。命令从标准输入读取完整一行，控制循环不会因等待输入而停止心跳。

1. 程序先对 10 Hz 低通后的双侧 `Fz` 检查窗口均值，窗口默认 0.5 s，可通过
   `control.zero_force_stable_s` 配置；均值不超过 `control.zero_force_threshold_n`，且窗口内任一峰值
   均未达到 `control.contact_on_n` 才会通过。这两个阈值必须满足零力阈值小于接触阈值。
   原始三轴力继续用于有限性和过力保护。确认无负载、急停和承接容器均就绪后，手托杯子放入夹爪并输入
   `ready`，开始闭合。
2. 力稳定后，程序提示可以撤手；此时切向触觉驱动的增力已启用。完全撤手后输入第二次 `ready`。
3. 再次稳定后，程序提示开始倒水。倒水时长结束后夹爪仍会保持，不会自动回位。
4. 先重新托住杯子，再输入 `release`，程序才回位并失能。任意阶段输入 `status` 可查看当前状态。

若出现异常或需要中止，优先使用急停并承接杯子。`Ctrl-C` 会交给运行时执行退出清理，但不保证夹持力能
持续保持。

## 记录与重绘

每次执行写入独占输出目录，包含：

- `config.json`：实际使用的配置。
- `events.jsonl`：阶段和故障等结构化事件。
- `tactile.jsonl`：原始触觉采样记录。
- `trace.csv`：控制周期 trace。
- `manifest.json`：运行状态和产物清单。
- `plot.pdf`、`plot.png`：结束时生成的控制诊断图。

`trace.csv` schema v2 新增 `force_deadband_active` 和 `unloading_blocked`，分别表示本周期是否因
进入力死区而冻结，以及是否阻止了导纳反向卸载。旧 schema v1 记录仍可由绘图命令读取。

可基于已有 `trace.csv` 独立重绘诊断图：

```sh
uv run --package dmgripper-experiments dmgripper-cup-plot \
  --directory outputs/real/cup_trial_001
```

该命令只读取记录目录，不连接真机。

触觉工作线程逐包保存采样，控制循环以默认 100 Hz 使用最新快照；两者的时间戳分别保留，
并不代表所有触觉包都参与了控制。`time_s` 是实验启动后的主机单调时间，
`tactile_received_at_s` 是主机接收时间，`tactile_timestamp_us` 是设备时间；
`position_rad` 等为该次命令返回的反馈，`q_des_rad` 等为协议量化前的受限请求。
记录逐行刷新，不使用无界后台积压队列；存储或通信阻塞导致超时会终止实验。
故障时保留已写入数据和失败 manifest，可以使用独立绘图命令分析，正常关闭设备后才自动出图。

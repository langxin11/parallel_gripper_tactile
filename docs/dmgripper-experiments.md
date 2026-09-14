# DMgripper 通用抓取实验

`dmgripper-run` 控制 DM4310P 平行夹爪与双侧 PapillArray，不使用 Hydra 或 MuJoCo。
默认参数尚未经真机验证。运行前必须核对机械限位、编码器零偏、回零方向和设备急停，准备承接容器，
并始终能够承接物体。软件保持、回位和尽力失能受通信与固件时延限制，不能替代硬件急停；失能可能松脱物体。

## 配置与执行

配置按 dataclass 默认值 → 严格 YAML → Tyro 参数覆盖合并，再完整验证并冻结；拒绝未知字段、
非有限数值和不相容组合。`--execute`、`--bias`、`--output` 为操作参数，不写入 YAML。

```sh
# 验证配置并输出计划，不连接设备
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/force_curve.yaml

# 交互执行动态增力任务
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/adaptive_grip.yaml --bias --execute

# 覆盖控制器，仍只输出计划
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/adaptive_grip.yaml --controller.kind pid
```

输入 `s`／`start`、`status`、`r`／`release` 后按 Enter 提交。非交互执行只接受
`auto_start=true` 且 `on_finished=return`。`terminal.mode=auto` 自动选择动态面板或纯文本；
可用 `--terminal.mode plain` 强制纯文本。显示模式不改变控制行为，最终输出包含状态、失能确认和运行目录。

## 生命周期与预载

正常阶段为 `preparing` → `ready` → 必要时 `homing` → `approach` →
`contact_transition` → `preload` → `active` → `holding` → 显式释放后的 `returning` → `completed`。

- `preparing` 完成触觉预检与零力验证；`ready` 等待 `start`，随后检查 DM 反馈、机械范围、
  MIT 模式与开始前失能状态，确认使能成功才运动。使能前 `release` 记为 `cancelled`，不发送运动命令。
- `preload` 取曲线首值或动态初始抓力。双侧确认接触且平均力连续达到
  `目标-preload_tolerance_n`，持续 `preload_stable_time_s` 后进入 `active`；**不设高侧稳定窗口**。
  动态策略此时冻结预载基线，后续载荷不移动基线。
- 失接触判据 `any_side`／`both_sides` 与动作 `fault`／`reapproach` 独立，默认
  `any_side + fault`。只有曲线模式支持重接近：暂停任务时间，恢复接触并稳定到暂停点目标后继续，接触段编号递增。
- 默认 `on_finished: hold`，任务结束仍闭环抓握并等待 `release`；动态模式继续响应载荷增长。
  自动回位必须显式选择 `return`。

## 启动位置与运动边界

命令位置严格限制在 `[0, pi/2] rad`；反馈安全范围为两端各扩展
`hardware.feedback_position_margin_rad`，默认 `[-0.05, pi/2+0.05] rad`。反馈余量不放宽命令范围。

默认 home 为 `0.0 rad`、容差为 `0.03 rad`。使能后若不在 home 容差内，先按受限速度、加速度和
加加速度回零，再接近；回零只依赖有效 DM 反馈。越出反馈安全范围时禁止自动运动。
轻微负反馈对应的首个位置目标投影到 `0`，合成力矩仍受限；首次验收须无负载、小零偏逐步确认。

配置验证与硬件编码均检查 DM4310P 的速度、力矩、`kp`／`kd` 量程，拒绝越界字段，
避免协议静默饱和改变已检查的力矩语义。

## 使能前零力验证

默认启用 `lifecycle.verify_zero_force`，期间电机保持失能。使用 `--bias` 时，先取得完整有效包，
只发送一次 bias，丢弃 bias 前数据并重置滤波与时间／包计数基线，等待新包和
`timing.tactile_bias_settle_s`，再用 settle 后新包验证。

`lifecycle.zero_force_stable_s` 窗口内，左右滤波全局 `Fz` 绝对值的均值都不得超过
`lifecycle.zero_force_threshold_n`。若均值通过但任一侧峰值达到 `contact_on_n`，输出结构化
`warning`，不重置窗口或单独阻止启动；操作者仍应排查接触或预紧。

每帧检查传感器与 taxel 数量／结构、三轴有限性、包计数、设备时间和新鲜度；原始法向过力与
双侧严重不平衡同样阻断启动。逐 taxel 幅值不使用未标定阈值，也不参与零力均值门禁；
`zero_force_diagnostics` 记录数量、均值、标准差和最大残余力位置，供离线标定。

## 故障保持与人工释放

故障处理取决于 DM 通道是否仍可控：

| 情形 | 行为 |
|---|---|
| 使能前预检失败 | 不使能；已发使能请求但确认丢失时，只尽力失能 |
| 使能后触觉、接触、过力、不平衡、控制／记录错误，或 DM 健康时回零／回位超时 | 停止闭合与目标更新，尝试 `fault_holding` |
| DM 通信／反馈丢失、命令短写、电机故障、意外失能、反馈越界或无法保持 | `fault`，立即尽力失能并关闭设备 |

只有保持命令发送成功且收到新的有效使能反馈后，才确认 `fault_holding`。保持目标取最新反馈并
投影到命令范围，`dq=0`、前馈力矩为零，仍受位置、增益和合成力矩限制；不再依赖触觉，只检查 DM。

操作者先承接物体，再输入 `release`：运行受限 DM-only 回位，到 home 后尝试失能。
回位超时但可重新保持时返回 `fault_holding`，等待检查后再次释放；保持或回位失去 DM 控制权则记为
`hold_lost` 并尽力失能。`Ctrl+C` 跳过普通释放等待，立即尽力失能清理，不能代替 `release`。

运行阶段与结果独立：故障后即使释放成功、阶段到达 `completed`，实验结果仍为 `failed`，原始故障保留。

## 目标与控制器

`reference` 二选一：

- `curve`：`hold`／`linear`／`smoothstep` waypoint，首点时间为零且时间严格递增，末点定义时长。
  允许下降段，但最低目标不得低于失接触阈值；下降段与导纳 `prevent_unloading` 互斥。
- `adaptive`：共享核 `TactileDisturbancePolicy`；切向输入为双侧各自 `hypot(Fx,Fy)` 之和，
  法向目标为平均单侧力，只在新触觉观测时更新，恒定载荷不会无界增力。
  当前配置初始目标 `0.5 N`、低侧容差 `0.15 N`，预载最低线为 `0.35 N`，没有高侧窗口；
  `prevent_unloading=true` 不主动卸载，过力保护仍独立生效。

`controller.kind` 可选 `admittance`、`pid`、一阶位置型 LADRC `adrc`。导纳使用外层滤波力；
死区与单向闭合仅属于导纳。PID／LADRC 将原始力交给共享核 `begin_tracking`／`step_tracking`，
内部只低通一次。trace 分别记录原始、外层滤波和控制使用力。

刚度估计默认 `enabled: true`、`method: window_linear`，`stiffness_consumption: none`，
只记录数值、有效性与原因。仅 PID／LADRC 显式选择 `feedforward` 才消费估计；默认估计参数尚未经真机辨识。

## 记录与重绘

输出目录为 `outputs/real/<task_name>/<object_name>/<UTC时间戳>-<run_id>/`：

| 产物 | 内容 |
|---|---|
| `config.json` | 输入路径、元数据与有效配置 |
| `events.jsonl` | 阶段、接触、目标启用、故障及诊断 |
| `tactile.jsonl` | 全局力、逐 taxel 原始三轴力、包连续性 |
| `trace.csv` | 控制周期记录，schema 为 `dmgripper-experiment/v1` |
| `manifest.json` | 结果、原始／清理故障、失能确认、产物清单 |
| `plot.pdf`／`plot.png` | 结果图 |

`fault_resolution` 区分 `released`、`forced_disable`、`hold_lost`；`disable_confirmed` 为
`true`／`false`／`not_applicable`，只有读回失能状态才写 `true`。任何设备或终端清理故障均使运行
失败并返回非零，原始错误保留，清理错误单列为 `cleanup_errors`。单独绘图失败记为
`post_processing_error`，不改写已完成的控制结果。

```sh
# 写回源目录并更新 manifest
uv run --package dmgripper-experiments dmgripper-plot <运行目录>
# 写入独占 repaint 目录，保留源记录
uv run --package dmgripper-experiments dmgripper-plot --repaint <运行目录>
```

历史 cup v1／v2 仍可用同一命令重绘，读取器将旧 `state` 列解释为 `phase`。

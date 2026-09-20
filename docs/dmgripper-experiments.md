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
`auto_start=true`、`on_finished=return` 且任务时长有限。`terminal.mode=auto` 自动选择动态面板或纯文本；
可用 `--terminal.mode plain` 强制纯文本。显示模式不改变控制行为，最终输出包含状态、失能确认和运行目录。

### 通用自适应抓取入口

后续水瓶（空瓶、0.25／0.5 水量）、舵机、耳机仓、海绵和长方体任务统一使用
`configs/hardware/dmgripper/unified_adaptive.yaml`。物体名称只用于输出目录与元数据，
不会暗中改变控制参数；空瓶与不同水量分别记录即可，不需要复制一份控制配置。
旧的 `adaptive_grip.yaml` 及 `unified_adaptive_fast.yaml` 保留原有行为供历史实验复现，
不再作为新任务的默认入口。曲线实验仍使用独立曲线配置。

```sh
# 离线检查完整配置，尚不访问设备。
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/unified_adaptive.yaml \
  --metadata.object-name water-bottle-empty

# 执行；换物体只修改记录名称，力限按实际物体耐受覆盖。
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/unified_adaptive.yaml \
  --metadata.object-name earbud-case --bias --execute
```

统一配置同时启用在线摩擦更新与 `window_linear` 等效接触刚度估计。后者记录
`stiffness_n_per_m`、`stiffness_valid` 和原因，不直接修改控制增益；稳定持物且没有足够
位移激励时可以没有有效刚度估计，不能把此时的数值当作物体材料弹性模量。
`reference.adaptive.duration_s: null` 表示 active 没有任务截止时间，持续运行至
输入 `release` 并按 Enter；受限回位完成后才正常失能退出。有限时长配置继续支持原行为。
触觉失鲜、预载、回位与通信超时仍用于检测故障，不属于任务执行时间限制。

MIT 跟踪 `kp=4.0`、`kd=0.7`，较库默认 `2.0／0.5` 小幅提高，回位参数独立。
初始力 `1 N/侧`，切向承载增大时目标以 `10 N/s` 上升，承载减小时以 `1 N/s` 下降；
目标上限为 `30 N/侧`，原始过力线为 `40 N/侧`。增力速率由
`reference.adaptive.max_force_rate_n_s` 与 `load.max_force_rate_n_s` 共同声明，减力速率由
`load.max_force_decrease_rate_n_s` 单独声明；省略后兼容旧配置，按增力速率对称卸力。
上限只是实验边界，不代表海绵、空瓶或耳机仓都能承受该力。修改目标上限时应同步
`reference.adaptive.unified.load.max_force_n` 与 `safety.max_target_force_n`，
并保持其低于 `safety.force_ceiling_n`。启动摩擦先验为 `0.3`，安全倍率 `1.5`，候选折减 `0.8`，
同一连续接触段内保留已接受估计；允许目标随载荷下降而受限降低。

### 双终端手柄旁路实验

只验证触觉与自主逐触点摩擦估计时，不必运行完整抓取生命周期。终端 A 用独立
`papillarray-record` 在空载下 bias 并持续记录；终端 B 用 `dmgripper-teleop` 手动闭合夹爪。
两个进程分别独占触觉串口和 DM 串口，不交换控制状态：

```sh
# 终端 A：启动时确保两个传感器完全无负载
uv run --package papillarray-hardware papillarray-record \
  --port /dev/papillarray --bias \
  --estimate-friction \
  --contact-on 0.15 --contact-off 0.10 \
  --output outputs/real/papillarray/manual-friction-01

# 终端 B：记录器收到 bias 后首个有效包再启动，窗口内仍需人工使能
uv run --package dmgripper-hardware dmgripper-teleop \
  --port /dev/dmj4310_can --execute
```

`--estimate-friction` 以逐触点接触集合和双侧低通法向力窗口判断稳定，并在启动时冻结参考接触集合；
活动期允许扰动引起的边缘触点变化，仅在整侧失去有效接触、失鲜、缺包或退出时停止。该路径只读取
逐 pillar `Fx/Fy/Fz`，不启动原厂滑移服务，也不发送 `S\n`／`s\n`。终端 A 显示“自主逐 pillar
摩擦估计已启动”后再沿触觉面缓慢施加切向扰动。结束时先在手柄端张开并确认失能，再对终端 A
按 `Ctrl-C`；无需按 Enter。记录器在独占运行目录写入 `tactile.jsonl`、`events.jsonl`、
`config.json` 和 `manifest.json`。单 pillar `Fz` 测量误差尺度为 `0.05 N` 时，默认接触进入／退出
门槛取 `0.15/0.10 N`；门槛以下数据仍记录但不作为参考触点。

## 生命周期与预载

### 原厂滑移短时辨识会话 {#native-slip-session}

可选 `lifecycle.native_slip` 在 `preload`／`active` 阶段启用原厂滑移服务旁路。
省略或设为 `null` 时不发送原厂启停命令。启用时必须显式填写 `max_duration_s`；没有设备通用
的最长运行默认值，应根据当前固件限制与真机经验选定。其余门槛为可配置的待标定起点，
不是原厂推荐参数。配置示意如下，数值仅用于说明字段：

```yaml
lifecycle:
  native_slip:
    max_duration_s: 2.0
    stable_duration_s: 0.3
    force_tolerance_n: 0.15
    force_rate_n_s: 0.5
    contact_on_n: 0.05
    contact_off_n: 0.025
    sample_timeout_s: 0.05
    confirmation_timeout_s: 0.2
    cooldown_s: 2.0
    estimate_expiry_s: 5.0
    min_estimates_per_side: 1
    max_sessions: 1
```

启动要求双侧有效触点数量达标、接触集合稳定，每侧实测法向力位于目标误差范围内，
法向力及目标变化率均低于门槛，并连续满足 `stable_duration_s`。逐点接触使用进入／退出滞回；
重复快照不累积稳定时间，缺包、失鲜、接触集合变化会重置稳定窗口。初始预载的既有达标判据
不替代这一额外门槛。原厂服务建立的接触参考仅属于本次会话。

会话具有递增编号。采集线程独占串口启停，发送启动后需由新包确认双侧激活；启动确认等待也计入
`max_duration_s`，不得反复刷新租期。每侧取得指定数量的新 `SLIPPED` 转换及有效正摩擦估计后
提前请求停止；超时、接触变化、失鲜、退出允许阶段、异常或人工中断同样请求停止。
原厂状态一直保持 `SLIPPED` 不算多个独立事件，缺失扩展不视为恢复或停止。
停止发送后必须由后续包的双侧非激活状态确认；确认超时报告失败，退出路径仍尽力补发停止命令。

到期检查在采集线程逐包读取前后执行，读取超时后也会执行；不依赖控制循环继续推进。
活动会话的单包等待预算缩短到下一个租期／确认期限，并临时缩短底层串口读取超时，避免沿用普通
采集的长等待预算。命令 I/O 不持有状态锁，不阻塞控制线程查询状态。这是主机软件租期，停止发送
仍有至多一个底层读取周期的预算超调、线程调度和命令 I/O 延迟，不保证设备在精确期限内
关闭。进程退出或通信中断时，主机无法保证固件停止；应结合设备行为验收，不能将命令发送当作确认。
启动与停止确认状态及原因进入日志，超时没有有效估计的会话不得解释为辨识成功。

有效逐点结果停止后保留到 `estimate_expiry_s`，接触变化或观测间断时撤销。默认 `max_sessions=1`，
不会周期性重启；显式允许更多会话时，须无有效结果、完成冷却并重新通过稳定窗口。
只对真正取得证据的 pillar 保存估计，未滑触点保持未知，不把传感器级估计复制给每个触点。

记录器 1.5.0 的 `tactile.jsonl` 保留逐点三轴力、位移（mm）、原厂完整滑移状态、逐点／传感器级
摩擦估计、推荐抓力和同包会话编号／状态；非有限估计记录为 `null`，缺失扩展为空数组。
独立记录器的自主模式在 `events.jsonl` 记录 `own_friction_state` 和逐点
`own_friction_estimate`。自主路径冻结稳定接触时的参考 pillar，
仅从逐点 `Fx/Fy/Fz` 时序检测“力比先上升、后饱和或剪切重分配”的持续事件；原厂状态和估计不进入
自主判据。确认后对候选前 `0.2 s` 窗口取 80% 分位得到 `raw_mu`，再乘 `0.8` 并裁剪到
`[0.05,2.0]` 得到 `conservative_mu`。普通粘着力比不生成估计。

自主阈值尚未真机标定，不能据此宣称检测准确率或估计误差已经通过实物验收。
辨识会话只记录自主候选和原厂推荐抓力，不在线改变目标调度。若显式启用保留的原厂对照路径，
同侧同 pillar 两条
路径均产生正有限结果后仍可记录配对误差，但这不是当前推荐实验流程。

### 快速多速率真机候选

`configs/hardware/dmgripper/unified_adaptive_fast.yaml` 是独立候选，不替换旧配置。
初始平均单侧目标为 1 N，上限为 30 N／侧，增力上限 50 N/s；导纳速度上限 0.20 rad/s，
质量 0.2 kg、阻尼 15 N·s/m、刚度 1 N/m、力矩前馈比例 1.0，MIT 力矩上限仍为 4 N·m。
`controller.admittance.feedforward_ratio` 默认统一为 1.0；
死区设为 0，`prevent_unloading=true` 按真机实验要求阻止导纳减小闭合量；显式释放仍走独立流程。
此项与允许反向纠偏的仿真候选不同，不保证实际抓力单调，也不能主动卸去过冲；原始过力保护保留。
法向低通显式设为 20 Hz，切向链为逐 taxel 中值3＋10 ms 低通。

30 N 是调度上限而不是固定目标；候选过力保护为 35 N／侧，**不是已验证安全值**。
必须按机构、传感器和物体允许值核对；9 个 taxel 的量程不能简单求和作为允许抓力。
旧配置不能证明这一快速候选已经安全，首次执行需保留物体承接与硬件急停。

```sh
# 离线校验：不连接设备
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/unified_adaptive_fast.yaml

# 核对保护阈值与设备条件后交互执行；清零时触觉必须无外载
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/unified_adaptive_fast.yaml --bias --execute
```

输入 `start` 后执行接近和 1 N 预载，等待进入 `active` 再转移物体支撑载荷；
程序不会自动撤去外部支撑。`holding` 仍自适应，输入 `release` 才按既有流程释放回位。
预载仍沿用低侧持续达标规则，并非双向误差稳定判定。此快速候选现显式开启实验性风险增力与摩擦更新：
每步请求增加 1 N，持续确认风险下每 100 ms 最多续增一次，总目标变化率仍受 50 N/s 限制。
风险累计预算为 12 N／12 步／20 s，不能无限加力；预算耗尽且风险持续会报告失败并按既有流程处置。
窗口 40 ms、确认 12 ms；窗口内共同接触点用于比较，避免边缘点反复进出使检测持续预热。
摩擦质量阈值 0.4 对应至少四个共同触点，候选折减后使用；这只是工程覆盖率，不是统计置信度。
相同事件只更新一次摩擦，持续风险续增不提供新的独立摩擦证据。

多速率配置只支持 1000 Hz 采样／250 Hz 控制。预处理在采集线程每包执行，目标调度每控制周期执行；
重复或年龄超过 10 ms 的快照冻结目标增长，100 ms 硬超时仍触发原保护。接收年龄不等于
传感器到主机的总延迟；串口积压和日志性能仍须从实测评估，不保证主机硬实时。
非法原始数据、过力和量程饱和不能被中值滤波隐藏，采集异常仍会传播为统一模式致命故障。
真机保留现有逐包原始日志，嵌套 `processed` 保存采样侧结果；`record_raw` 当前不关闭真机审计日志。
记录器 1.3.0 追加 `sensor_*` 时间、序号、年龄、丢帧和事件列。两类时钟分别记录，不做同步精度承诺。

正常阶段为 `preparing` → `ready` → 必要时 `homing` → `approach` →
`contact_transition` → `preload` → `active` → `holding` → 显式释放后的 `returning` → `completed`。

- `preparing` 先做电机预检：打开 DM 串口、校验反馈与 MIT 模式、确认失能态（上次异常退出残留的
  使能态先显式失能一次再确认）；位置不在 home 容差内时按受限回位参数自动回零，完成后回到失能态。
  操作者约定运行前已移开物体，预检回零因此可以解除上次故障残留的闭合位置，避免零力验证被卡死。
  预检失败按执行异常直接失能退出，不进入故障保持。随后完成触觉预检与零力验证；
  `ready` 等待 `start`（期间电机保持失能），收到后复查反馈与失能态、确认使能成功才运动，
  漂移出 home 容差时先回零再接近。使能前 `release` 记为 `cancelled`，不发送运动命令。
- `preload` 取曲线首值或动态初始抓力。双侧确认接触且平均力连续达到
  `目标-preload_tolerance_n`，持续 `preload_stable_time_s` 后进入 `active`；**不设高侧稳定窗口**。
  设置 `preload_min_force_ratio` 后，改为要求最终目标乘以该比例，并覆盖绝对容限，适用于只需
  确认抓力已建立、不要求预载精确稳态的任务。
  旧动态策略此时冻结预载基线，后续载荷不移动基线；统一策略不从绝对承载中扣除预载载荷。
- 失接触判据 `any_side`／`both_sides` 与动作 `fault`／`reapproach` 独立，默认
  `any_side + fault`。只有曲线模式支持重接近：暂停任务时间，恢复接触并稳定到暂停点目标后继续，接触段编号递增。
- 默认 `on_finished: hold`，任务结束仍闭环抓握并等待 `release`；动态模式继续响应载荷增长。
  自动回位必须显式选择 `return`。

## 启动位置与运动边界

命令位置严格限制在 `[0, pi/2] rad`；反馈安全范围为两端各扩展
`hardware.feedback_position_margin_rad`，默认 `[-0.05, pi/2+0.05] rad`。反馈余量不放宽命令范围。

默认 home 为 `0.0 rad`、容差为 `0.03 rad`。预检阶段发现不在 home 容差内时先按受限速度、加速度和
加加速度回零，完成后失能等待启动；`start` 后若又漂移出容差，会再次受限回零后才接近。
两处回零都只依赖有效 DM 反馈。越出反馈安全范围时禁止自动运动。
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
双侧严重不平衡同样阻断启动。旧模式的逐 taxel 幅值不使用未标定阈值，也不参与零力均值门禁；
`zero_force_diagnostics` 记录数量、均值、标准差和最大残余力位置，供离线标定。

部分 PapillArray 固件的微秒计时器每 \(2^{32}\) μs（约 71.6 分钟）回绕。
采集会话仅在包计数正常前进、设备时间跨越该边界且模差不超过一秒时展开回绕，
使滤波、控制与回放使用的 `timestamp_us` 保持连续；重复包、普通时间回退和计数异常仍报错。
`tactile.jsonl` 同时记录 `raw_timestamp_us` 与 `timestamp_wrap_count`，保留设备原值和回绕诊断。
bias 后同时重置展开基线；主机接收时间的新鲜度检查不受影响。记录器版本为 1.4.0，旧记录保持不变。

## 故障保持与人工释放

旧模式故障处理取决于 DM 通道是否仍可控；统一模式还有下文所述的独立触觉保护：

| 情形 | 行为 |
|---|---|
| 使能前预检失败 | 不使能；已发使能请求但确认丢失时，只尽力失能 |
| 使能后触觉、接触、过力、不平衡、控制／记录错误，或 DM 健康时回零／回位超时 | 停止闭合与目标更新，尝试 `fault_holding` |
| DM 通信／反馈丢失、命令短写、电机故障、意外失能、反馈越界或无法保持 | `fault`，立即尽力失能并关闭设备 |

只有保持命令发送成功且收到新的有效使能反馈后，才确认 `fault_holding`。保持目标取最新反馈并
投影到命令范围，`dq=0`、前馈力矩为零，仍受位置、增益和合成力矩限制；不再依赖触觉，只检查 DM。

操作者先承接物体，再输入 `release`：运行受限 DM-only 回位，到 home 后尝试失能。
回位超时但可重新保持时返回 `fault_holding`，等待检查后再次释放；保持或回位失去 DM 控制权则记为
`hold_lost` 并尽力失能。回位规划失败时继续持位，可检查后重试 `release`；输入通道异常也不
直接失能，记录一次同类告警并继续持位，输入恢复后可释放。若终端永久丢失，则需要人工
承接物体后通过可用的中断渠道处理，程序不会伪造一次 `release`。`Ctrl+C` 跳过普通释放等待，立即尽力失能清理，不能代替 `release`。

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

PID 使用每周期电机实测位置作为参考：`q_des = q_real + position_adjustment`。
`controller.pid.max_position_adjustment_rad` 限制的是相对当前反馈的位置偏置，
不是接触后的累计闭合行程；最终请求仍经过原有速度、机械行程与 MIT 合成力矩限幅。
该变化与仿真共用控制核；一阶 LADRC 仍使用固定接触参考。

将 `controller.pid.max_position_adjustment_rad` 设为 `null` 可关闭 PID 的固定位置偏置及积分幅值
限幅，使修正量能够超过旧的 ±0.15 rad；机械行程、逐周期速度、MIT 总力矩与触觉过力保护继续生效。
此模式在后端位置／速度或力矩饱和且误差继续推向饱和方向时回退本周期积分，反向误差仍允许积分消退。
原有限数值配置保持既有行为。该开关只取消 PID 的固定偏置上限；一阶 LADRC 不支持无界累计行程，
切换到 LADRC 时，PID 的 `null` 配置按其原默认 0.15 rad 上限处理。

PID 的独立模型力矩前馈由 `controller.pid.torque_feedforward_gain` 控制：

\[
\tau_{\mathrm{ff}}=g_{\mathrm{ff}}J_c(q_{\mathrm{real}})F_{\mathrm{target}}.
\]

目标力为平均单侧力，`J_c` 为总闭合行程对电机角度的雅可比；不额外乘二。
显式数值须位于 0～1，`1` 为完整模型前馈、`0` 或 `null` 关闭该 PID 模型项。
该字段只作用于 PID，不影响导纳或一阶 LADRC，也不依赖在线刚度估计。

在 YAML 中配置 PID 前馈比例 `1.0`、位置偏置上限 `null` 后，运行时追加 `--controller.kind pid`
即可使用该组合；即使刚度估计关闭或尚无有效估计也会产生前馈。
预载、运行与正常保持阶段使用当期目标力，接近／回位保持原有策略。前馈与 PID 的 MIT 合成力矩
仍受原有力矩限幅约束；`trace.csv` 的 `tau_ff_nm` 记录最终前馈请求。

刚度估计默认 `enabled: true`、`method: window_linear`，只记录数值、有效性与原因，
不生成控制量。独立 PID 模型力矩前馈不消费估计。默认估计参数尚未经真机辨识。

## 统一自适应试运行与旁路回放 {: #unified-adaptive }

```sh
# 只验证配置，不连接、不使能
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/unified_adaptive.yaml

# 仅回放一段连续抓取记录，输出文件必须尚不存在
uv run --package dmgripper-experiments python -m dmgripper_experiments.replay \
  <抓取段的tactile.jsonl> <新的diagnostic.csv> \
  --config configs/hardware/dmgripper/unified_adaptive.yaml
```

`reference.adaptive.unified` 非空时替代旧剪切增量策略，保持 `adaptive` 生命周期和交互入口。
它要求双侧恰好九点，按设备时间戳只消费新观测；直接使用采集层局部三轴力，
部署前必须核对左右身份、坐标、正压缩符号、触点量程与摩擦先验。适配器不自动推断这些标定。
算法和仿真边界见[自适应调度器验证](force-scheduling.md#adaptive-scheduler)。

统一配置的当前参数见上文“通用自适应抓取入口”。仅支持导纳控制，启用执行限幅回投；
共享 `load` 的最低力、上限与速率必须与生命周期／保护配置一致。
库默认仍关闭风险与摩擦权限；启用它们需要相应验收标志，或显式设置
`experimental_closed_loop=true` 授权未验收的受控闭环实验。通用 YAML 使用后一种方式，
不会伪造 `risk_validation_passed` 或 `friction_validation_passed`。
软件与仿真验证不等于真实起滑识别、摩擦准确性或止滑能力已经通过验收。

使能后的触觉／控制故障停止自适应与继续闭合；只要 DM 命令和新鲜使能反馈仍正常，
进入 `fault_holding` 等待人工承接物体并输入 `release`，不因科学失败或触觉故障直接退出。
故障保持不依赖已经失效的触觉闭环，也不保证恒定抓力。DM 通信丢失、实际电机故障或持位
命令失败时无法保证电机保持，沿用失控降级与尽力失能；`Ctrl+C` 仍为立即清理的人工中断。

回放始终撤销两条控制权限，目标不推进，也不模拟执行器；按原始设备时间产生相同的因果诊断。
重复时间戳忽略，回退／坏输入明确报错，已写部分结果可能保留，不可当作完整验收记录。
多次抓取需分段单独回放。检测提前量和误触发仍需独立位移参考、负例及未参与调参的记录；
该工具不自行给出起滑真值、摩擦准确性或闭环性能结论。

## 抓取中的在线摩擦估计 {: #online-friction-adaptive }

统一自适应入口不读取历史摩擦辨识结果。它以左右相同的 `0.3` 启动先验建立
抓取，在 active 阶段由逐触点纯力风险事件生成分侧摩擦候选；低于当前状态的保守候选立即接受，
提高摩擦需要三次独立一致事件。`friction_expiry_s: null` 使已接受的估计在当前连续接触段内
保持；接触集合变化、数据失效或新抓取开始时仍重置为启动先验。执行入口与命令见上文
“通用自适应抓取入口”与“统一自适应试运行”。

在线链路为风险事件产生候选、候选通过覆盖率和范围门禁、更新 `adaptive_left_mu`／
`adaptive_right_mu`，随后共享调度器按

\[
T_s=\left\|\sum_i(F_{x,s,i},F_{y,s,i})\right\|,\qquad
f_{\mathrm{raw}}=\gamma\max\left(\frac{T_L}{\hat\mu_L},\frac{T_R}{\hat\mu_R}\right)
\]

重新计算平均单侧法向目标。该公式使用二维切向合力模，不要求预先知道加载方向。目标可双向变化：
升降速率与上下限由 `reference.adaptive` 与共享 `load` 配置约束，当前统一配置增力 `10 N/s`、
退力 `1 N/s`，最低保持 `1.0 N/侧`；风险事件只更新摩擦状态，
不再叠加固定增力预算或触发 `risk_budget_exhausted`。导纳控制允许跟随下降目标受限张开。
没有可信事件时，
trace 中的摩擦值仍是启动先验，不能解释为已经估计完成；`lower_accepted`、`raise_accepted` 才表示
控制状态接受了新候选。`adaptive_left_candidate`／`adaptive_right_candidate` 保存当次候选，
候选由事件前整侧切向合力与法向合力之比生成，逐触点局部比值只定位发生重分配的区域。
质量值是直接支持事件的受影响触点比例，不是统计置信度。稳定无风险样本形成
`adaptive_*_mu_lower_bound`（各侧有效触点切向矢量和的模除以法向力之和）；显著低于该下界的候选以 `below_observed_lower_bound` 拒绝。
每个有效触点也积累局部 `hypot(Fx,Fy)/Fz` 下界，左右各侧当前有效触点历史下界的最大值
分别记录为 `adaptive_left_taxel_mu_lower_bound` 与 `adaptive_right_taxel_mu_lower_bound`。
共同摩擦系数假设下局部最大值可提供更强下界；存在异质接触时，它仅约束对应局部摩擦，
不能直接否决整侧等效摩擦候选。全局与局部均来自同一组触觉数据，不增加独立事件计数，
也不会只凭下界直接提高控制采用的摩擦值。下界成立依赖粘着假设和有效力测量，
“未检测到风险”不等于已经通过独立位移参考证明没有滑移。
启用在线摩擦更新的运行图会自动增加摩擦面板：
阶梯线表示控制器实际采用的左右 `adaptive_*_mu`，空心圆表示已接受的原始候选，叉号表示未通过
门禁的候选，点线表示当前接触段已经观测的无滑移利用率下界。

当前统一入口使用 `1.0 N/侧` 初始力、`30 N/侧` 目标上限与 `40 N/侧` 原始过力保护。
这些只是受控试验边界；跟踪缺口持续超过 `failure_timeout_s` 时策略进入故障保持，
不会自动突破力限。
纯力事件可能漏掉无明显重分配的同步滑动，也可能把接触重排当作风险；检测提前量、摩擦误差和
物体位移仍需独立参考验证。

### Niu 等人的粒子滤波能否移植

可以复用随机游走预测、贝叶斯加权和有效样本量触发重采样的框架，但当前通用配置尚未启用
粒子滤波。[原文式（10）与算法 1](https://arxiv.org/html/2602.02026v1) 使用
`F_MER,t ≈ μ F_MER,n (1-cf_target)`，隐含实际接触系数接近控制目标的条件。
单凭稳定粘着时的力比不能辨识唯一摩擦系数；粒子滤波也不能凭空增加可观测性。
如果无条件将该等式用于当前闭环，后验可能只是收敛到控制器自己的假设。

适合本项目的移植方式是：稳定粘着样本用带测量不确定度的单边下界似然；有独立起滑证据时，
才用边界附近的力比作双边观测。局部与全局来自同一组力数据，应处理相关性，不能将它们
当作多个独立证据反复收紧后验。先以旁路回放验证覆盖率、估计偏差和接触变化后的恢复，
再考虑用保守后验分位数参与调度；保持力限和变化率约束。

原文 `F_MER,t` 是局部切向力**模的积分**，当前承载调度使用**切向合力的模**，两者在
触点剪切方向不同时不相等。PapillArray 的柱力也不能直接视为 Tac3D 面元力密度，需先明确
接触面法向与空间离散权重，再移植该观测模型。

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

记录器 1.7.0 在现有 schema 上保留并追加 `adaptive_*` 列：风险、事件、有效掩码、摩擦候选／质量、
更新原因、需求与跟踪缺口、执行限幅及失败原因；旧目标模式对应列为空。
原始需求仍保留于 `target_raw_force_n`，观测状态变化与新事件同步写入 `events.jsonl`。

`fault_resolution` 区分 `released`、`forced_disable`、`hold_lost`；`disable_confirmed` 为
`true`／`false`／`not_applicable`，只有读回失能状态才写 `true`。任何设备或终端清理故障均使运行
失败并返回非零，原始错误保留，清理错误单列为 `cleanup_errors`。单独绘图失败记为
`post_processing_error`，不改写已完成的控制结果。

```sh
# 写回源目录并更新 manifest
uv run --package dmgripper-experiments dmgripper-plot <运行目录>
# 写入独占 repaint 目录，保留源记录
uv run --package dmgripper-experiments dmgripper-plot --repaint <运行目录>
# 窗口化评价图：只绘 active 段的任务时间 0–12 s，横轴切换为任务时间
uv run --package dmgripper-experiments dmgripper-plot --repaint \
  --task-time 0:12 <运行目录>
# 或按阶段白名单截窗（可组合），窗口条件写入重绘 manifest
uv run --package dmgripper-experiments dmgripper-plot --repaint \
  --phase preload,active <运行目录>
```

历史 cup v1／v2 仍可用同一命令重绘，读取器将旧 `state` 列解释为 `phase`。
窗口参数必须配合 `--repaint`，避免覆盖源目录的整段图；`--task-time` 只保留
任务时钟推进的行，适合以曲线起点为零的评价图，`--phase` 保留所选阶段的全部行
（横轴仍为运行时间）。未知阶段名或倒置区间直接报错，窗口内无数据时视同无可绘制数据。

### 单向合成力矩与被动卸力

通用配置启用 `controller.closing_torque_only: true`，接近、接触过渡、预载、active 与正常
holding 阶段按最新反馈约束 MIT 合成力矩的闭合方向分量处于零至力矩上限之间。
保留 `prevent_unloading: false`，允许导纳降低正向支撑，由物体与触觉垫反力推动被动退让；
不保证摩擦或自锁机构能够卸至任意目标力。限幅自动回投导纳状态，避免负力矩请求积累。
接触过渡保留接近阶段的力前馈，避免接触确认时突然清零前馈。
显式 release、回零及故障位置保持仍使用独立的双向位置控制，不把单向跟踪限制带入回位。

此约束针对主机计算的未量化命令，不是固件级单向电流限幅。协议量化、反馈延迟及两次通信
之间的运动仍可能使电机反馈力矩短暂为负；不能宣称实际力矩绝不为负或绝不会脱离接触。
需要严格的驱动器级保证时，须由固件提供合成力矩单向限幅。当前改动未做真机验收。

统一配置还启用 `controller.zero_tracking_velocity: true`。预载、active 与 holding 阶段继续由
导纳积分闭合位移并更新 MIT 位置参考，但将 MIT 目标速度固定为零。速度项因此只对实测速度
提供阻尼，不再把导纳速度直接注入内环；接近、接触过渡、故障保持与 `release` 回位仍保留其
各自的速度轨迹。该开关需要与非零 `feedforward_ratio` 一起在真机上评估，不能替代位置、速度
或力矩限幅。

### 平滑建立预载

统一配置的法向测量力使用 `timing.tactile_cutoff_hz: 20.0` 的一阶低通。
`lifecycle.preload_force_rate_n_s: 1.0` 将预载改为从接触过渡末的滤波平均单侧力开始，
按五次曲线升至初始抓力，首尾目标变化率与加速度均为零，峰值变化率不超过 `1 N/s`。
起点裁剪在接触阈值与最终预载目标之间；已经超过目标时不再额外升力。
例如从 `1.15 N` 升至 `4 N` 约需 `5.34 s`，然后才累计最终预载稳定时间。统一配置以
`preload_min_force_ratio: 0.75` 与 `preload_stable_time_s: 0.5` 作为粗预载门槛：对 `4 N`
目标，双侧仍接触且平均力不低于 `3 N` 连续 `0.5 s` 后即可进入 active，而非等待窄误差带。
升力包含在预载超时内，升力与稳定等待无法在超时内完成时明确报告故障。
旧配置省略该字段或设为 `null` 仍使用常值预载目标；active 阶段的增力／卸力速率独立。
此次仅改变滤波与预载建立方式，导纳质量、阻尼及 MIT 增益保持现值，效果需真机记录确认。

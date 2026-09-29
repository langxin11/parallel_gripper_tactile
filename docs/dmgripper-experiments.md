# DMgripper 通用抓取实验

`dmgripper-run` 控制 DM4310P 平行夹爪与双侧 PapillArray，不使用 Hydra 或 MuJoCo。
默认参数尚未经真机验证。运行前必须核对机械限位、编码器零偏、回零方向和设备急停，准备承接容器，
并始终能够承接物体。软件保持、回位和尽力失能受通信与固件时延限制，不能替代硬件急停；失能可能松脱物体。

## 配置与执行

配置按 dataclass 默认值 → 严格 YAML → Tyro 参数覆盖合并，再完整验证并冻结；拒绝未知字段、
非有限数值和不相容组合。默认值即主力 `unified_adaptive` profile，YAML 只写实验身份与真差异，
派生量（调度包络上限、最低力、速率边界）由 `safety` 与初始目标在构造期生成，不存在第二个旋钮。
`--execute`、`--bias`、`--output`、`--terminal`、`--terminal-refresh-hz` 为操作参数，不写入 YAML。

```sh
# 验证配置并输出计划，不连接设备
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/unified_adaptive.yaml

# 交互执行
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/unified_adaptive.yaml --bias --execute

# 覆盖单个字段，仍只输出计划
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/unified_adaptive.yaml \
  --metadata.object-name water-bottle-empty
```

输入 `s`／`start`、`status`、`r`／`release` 后按 Enter 提交。非交互执行只接受
有限任务时长且 `on_finished=return` 的组合（该组合同时使 `ready` 阶段自动启动，不再有独立开关）。
`--terminal auto` 自动选择动态面板或纯文本；可用 `--terminal plain` 强制纯文本。
显示模式不改变控制行为，最终输出包含状态、失能确认和运行目录。启动后事件流首先输出
`safety_limits`：最终生效限幅表（目标上限、原始过力线、增／退力速率、压缩行程与派生的统一
调度包络），真机启动即可审计“配置里写的”与“实际执行的”是否一致。

### 通用自适应抓取入口

水瓶（空瓶、0.25／0.5 水量）、舵机、耳机仓、海绵和长方体任务统一使用
`configs/hardware/dmgripper/unified_adaptive.yaml`。物体名称只用于输出目录与元数据，
不会暗中改变控制参数；空瓶与不同水量分别记录即可，不需要复制一份控制配置。
粒子摩擦后验的受控试验使用 `unified_adaptive_particle.yaml`，它只写与主力的真差异
（100 Hz 控制、刚度预载、粒子估计器与导纳外环速度边界），其余继承默认值。

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
`reference.duration_s: null` 表示 active 没有任务截止时间，持续运行至
输入 `release` 并按 Enter；受限回位完成后才正常失能退出。有限时长配置继续支持原行为。
触觉失鲜、预载、回位与通信超时仍用于检测故障，不属于任务执行时间限制。

MIT 跟踪采用候选 `kp=10.0`、`kd=2.0`，回位参数独立；该选择不表示已完成实机阻尼标定。
初始力 `1 N/侧`，切向承载增大时目标以 `10 N/s` 上升，承载减小时以 `1 N/s` 下降；
目标上限为 `30 N/侧`，原始过力线为 `40 N/侧`。增／退力速率与目标上限只在
`safety` 定义一次；统一调度包络的最低力取 `reference.initial_force_n`、上限取
`safety.max_target_force_n`，构造期派生，YAML 中不存在重复旋钮。
上限只是实验边界，不代表海绵、空瓶或耳机仓都能承受该力；修改上限只需改
`safety.max_target_force_n`，并保持其低于 `safety.force_ceiling_n`。
启动摩擦先验为 `0.1`，安全倍率 `1.5`，候选折减 `0.8`，
同一连续接触段内保留已接受估计；允许目标随载荷下降而受限降低。

历史上的 `adaptive_grip.yaml`、`force_curve.yaml`、`unified_adaptive_fast.yaml` 与其
承载的旧路径（曲线任务、切向增量策略、pid／adrc 控制器、原厂滑移与自主旁路会话、
多速率预处理、risk_step 阶梯增力）已随配置重构退役；如需复现历史实验，回退到对应
git 版本。

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

### 独立记录器的原厂滑移与自主摩擦字段

`dmgripper-run` 的 `lifecycle.native_slip`／`own_friction` 旁路会话已随配置重构退役，
在线链路只保留统一摩擦估计。独立记录器 `papillarray-record` 仍支持
`--native-slip` 与 `--estimate-friction`：其 `tactile.jsonl`（记录器 1.5.0）保留逐点三轴力、
位移（mm）、原厂完整滑移状态、逐点／传感器级摩擦估计、推荐抓力和同包会话编号／状态；
非有限估计记录为 `null`，缺失扩展为空数组。`events.jsonl` 记录 `own_friction_state` 和逐点
`own_friction_estimate`。自主路径冻结稳定接触时的参考 pillar，
仅从逐点 `Fx/Fy/Fz` 时序检测“力比先上升、后饱和或剪切重分配”的持续事件；原厂状态和估计不进入
自主判据。确认后对候选前 `0.2 s` 窗口取 80% 分位得到 `raw_mu`，再乘 `0.8` 并裁剪到
`[0.05,2.0]` 得到 `conservative_mu`。普通粘着力比不生成估计。
自主阈值尚未真机标定，不能据此宣称检测准确率或估计误差已经通过实物验收。

正常阶段为 `preparing` → `ready` → 必要时 `homing` → `approach` →
`contact_transition` → `preload` → `active` → `holding` → 显式释放后的 `returning` → `completed`。
终端将这些内部阶段归并为预检、等待启动、建立抓力、正式运行、故障保持和释放结束；
`events.jsonl` 与 trace 仍保留原始阶段名供诊断。使能后的 `homing`、`approach`、
`contact_transition`、`preload`、`active` 和 `holding` 均接受 `release`，统一转入受限回位。
统一自适应配置的 `duration_s: null` 使 `holding` 在正常路径不可达；失接触无论发生在哪一侧
一律按故障处理，不存在重新接近分支。

- `preparing` 先做电机预检：打开 DM 串口、校验反馈与 MIT 模式、确认失能态（上次异常退出残留的
  使能态先显式失能一次再确认）；位置不在 home 容差内时按受限回位参数自动回零，完成后回到失能态。
  操作者约定运行前已移开物体，预检回零因此可以解除上次故障残留的闭合位置，避免零力验证被卡死。
  预检失败按执行异常直接失能退出，不进入故障保持。随后完成触觉预检与零力验证；
  `ready` 等待 `start`（期间电机保持失能），收到后复查反馈与失能态、确认使能成功才运动，
  漂移出 home 容差时先回零再接近。使能前 `release` 记为 `cancelled`，不发送运动命令。
- `preload` 取初始抓力。双侧确认接触且平均力连续达到最终目标乘以
  `preload_min_force_ratio`（默认 `0.75`），持续 `preload_stable_time_s` 后进入 `active`；
  **不设高侧稳定窗口**。该比例门槛适用于只需确认抓力已建立、不要求预载精确稳态的任务；
  统一策略不从绝对承载中扣除预载载荷。
- 任一侧失接触即故障（常量语义）。重新接近的摩擦基线语义未定义，不提供该分支。
- 默认 `on_finished: hold`，任务结束仍闭环抓握并等待 `release`，统一自适应目标继续响应载荷增长。
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
只发送一次 bias，丢弃 bias 前数据并重置滤波与时间／包计数基线，等待新包和固定的 `2 s` settle
常数，再用 settle 后新包验证。

零力验证在 `0.5 s` 稳定窗口（超时上限 `5 s`）内检查：左右滤波全局 `Fz` 绝对值的均值都不得超过
`0.1 N` 常数阈值。若均值通过但任一侧峰值达到 `lifecycle.contact_on_n`，输出结构化
`warning`，不重置窗口或单独阻止启动；操作者仍应排查接触或预紧。

每帧检查传感器与 taxel 数量／结构、三轴有限性、包计数、设备时间和新鲜度；原始法向过力与
双侧严重不平衡同样阻断启动。逐 taxel 幅值不使用未标定阈值，也不参与零力均值门禁；
`zero_force_diagnostics` 记录数量、均值、标准差和最大残余力位置，供离线标定。

部分 PapillArray 固件的微秒计时器每 \(2^{32}\) μs（约 71.6 分钟）回绕。
采集会话仅在包计数正常前进、设备时间跨越该边界且模差不超过一秒时展开回绕，
使滤波、控制与回放使用的 `timestamp_us` 保持连续；重复包、普通时间回退和计数异常仍报错。
`tactile.jsonl` 同时记录 `raw_timestamp_us` 与 `timestamp_wrap_count`，保留设备原值和回绕诊断。
bias 后同时重置展开基线；主机接收时间的新鲜度检查不受影响。记录器版本为 1.4.0，旧记录保持不变。

### 记录体积与速率策略（记录器 1.12.0）

`tactile.jsonl` 每行浮点截断到 9 位有效数字（float32 可表示精度）；传感器上行在协议层即为
float32，更高位数只复制运算舍入噪声。`received_at_s` 是主机单调钟，保留 float64 全精度；
行内分隔符收紧为无空格。读取端仍按行 `json.loads` 解析，旧记录保持不变，可直接混用。

触觉流由记录器内的专用写线程异步落盘：采集线程只做有限性校验并入队，序列化与写盘按
256 行或 0.25 s 批量提交，磁盘延迟不进入触觉观测链路。flush 只保证进程退出（含异常收尾）后
已提交数据可读；进程被杀死时最多丢失一个提交周期内的队列与缓冲，断电与内核崩溃不在保证
范围。写线程失败会在下一次写入或收尾时以链式异常上抛；收尾本为成功时升级为失败并保留
原始错误，不把缺数据的运行静默记为完成。

`recording.tactile_stride` 按设备包计数抽样正常包；首个包与出现间隙／回绕的异常包始终保留，
硬件丢包仍可由 `packet_counter` 差值审计。当前真机配置取 4，即 1000 Hz 里记录约 250 Hz；
需要全速率（噪声频谱等）时改为 1，体积约增大 4 倍。同一研究内应保持记录速率一致，
新旧速率的数据不宜直接混入同一对比。

`recording.max_duration_s`（默认 600 s）与 `recording.max_tactile_mib`（默认 512 MiB）触发后
按 release 语义受限回位正常收尾：事件流记录 `recording_limit`，manifest 的 `stop_reason`
说明原因。两者设为 `null` 可关闭；忘记停止不再无限写盘。

存量旧记录可用 `scripts/maintenance/reencode_compress_tactile.py` 就地迁移为
`tactile.jsonl.gz`：重编码与新记录同语义（float32 截断、紧凑分隔符，行数与速率不变），
gzip 无损压缩，逐行校验与 MD5 全量比对通过才原子替换。回放工具
（`dmgripper-experiments` 的 `replay` 与 `scripts/analysis/replay_pillar_friction.py`）
按扩展名透明读取 `.gz`；其余消费方可用 `gzip.open(path, "rt", encoding="utf-8")`
或 `zcat` 等价访问。

## 故障保持与人工释放

故障处理取决于 DM 通道是否仍可控；统一自适应模式还有下文所述的独立触觉保护：

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

目标力只有统一自适应一条路径：共享核 `TactileDisturbancePolicy` 以双侧各自 `hypot(Fx,Fy)` 之和为
切向输入、平均单侧力为法向目标，只在新触觉观测时更新，恒定载荷不会无界增力；
`prevent_unloading=false` 不主动卸载，过力保护仍独立生效。
导纳是唯一控制器，使用外层滤波力；死区与单向闭合仅属于导纳。
trace 分别记录原始、外层滤波和控制使用力。

`estimation` 只保留 `enabled` 开关：估计方法固定为 `window_linear`，窗口界限与平滑系数沉为
内部常数，只记录数值、有效性与原因，不生成控制量。默认估计参数尚未经真机辨识。

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

`reference.unified` 是统一自适应策略的硬件视图，保持既有生命周期和交互入口。
它要求双侧恰好九点，按设备时间戳只消费新观测；直接使用采集层局部三轴力，
部署前必须核对左右身份、坐标、正压缩符号、触点量程与摩擦先验。适配器不自动推断这些标定。
算法和仿真边界见[自适应调度器验证](force-scheduling.md#adaptive-scheduler)。

统一配置的当前参数见上文“通用自适应抓取入口”。仅支持导纳控制，启用执行限幅回投；
共享调度的最低力、上限与速率由 `safety` 与初始目标在构造期派生（`unified_core_config`）。
硬件视图固定开启风险检测与摩擦更新，仍要求显式 `experimental_closed_loop=true`
授权未验收的受控闭环实验。
软件与仿真验证不等于真实起滑识别、摩擦准确性或止滑能力已经通过验收。

使能后的触觉／控制故障停止自适应与继续闭合；只要 DM 命令和新鲜使能反馈仍正常，
进入 `fault_holding` 等待人工承接物体并输入 `release`，不因科学失败或触觉故障直接退出。
故障保持不依赖已经失效的触觉闭环，也不保证恒定抓力。DM 通信丢失、实际电机故障或持位
命令失败时无法保证电机保持，沿用失控降级与尽力失能；`Ctrl+C` 仍为立即清理的人工中断。

回放始终撤销两条控制权限，目标不推进，也不模拟执行器；按原始设备时间产生相同的因果诊断。
重复时间戳忽略，回退／坏输入明确报错，已写部分结果可能保留，不可当作完整验收记录。
多次抓取需分段单独回放。检测提前量和误触发仍需独立位移参考、负例及未参与调参的记录；
该工具不自行给出起滑真值、摩擦准确性或闭环性能结论。

### 深度条件对数先验 {#depth-friction-prior}

统一硬件视图以 `unified.estimator` 判别估计路线：`depth_prior` 必须携带深度先验块，
`particle` 可选叠加深度先验作粒子初始化，`classic` 不带该块、即固定先验路径。
`unified_adaptive.yaml` 与 `unified_adaptive_particle.yaml` 显式开启
`reference.unified.depth_friction_prior`。
这是一条经验先验，不是由深度测得的真实摩擦或有统计保证的摩擦下界。

摩擦先验以首次双侧滤波法向力均达到 `lifecycle.contact_on_n` 时的实测关节角 `q_c` 为基线，
通过现有非线性曲柄滑块模型换算两指总闭合量 `c(q)`：

\[
\delta(q)=\max(0,c(q)-c(q_c))=\max(0,a(q_c)-a(q)),
\]

其中 `a(q)` 是两指总开度。压缩量来自命令发送前配对的实测角度，不用目标角度或固定雅可比
近似，也不消费传感器 pillar 位移。它表示物体、指垫和机构变形合计造成的两指总闭合增量，
不能辨识左右侧各自压缩量或物体单独压缩量。左右先验使用同一 `δ`，触点覆盖与摩擦证据仍分侧处理。
摩擦先验在一次抓取中只锁定一个双侧接触关节基线，接触确认、触点变化和短暂脱离不会重新归零，避免绕过累计压缩限制。
共享核保留可选逐触点位移入口供原调用兼容，但真机路径以关节总压缩量为准。

\[
x=\operatorname{clip}\left(\frac{\delta-\delta_0}{\delta_1-\delta_0},0,1\right),
\qquad
\mu_{\mathrm{prior}}=\mu_0+(\mu_1-\mu_0)\frac{\ln(1+\kappa x)}{\ln(1+\kappa)}.
\]

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `min_friction`／`max_friction` | `0.1`／`0.8`（当前 `unified_adaptive.yaml`）；粒子配置仍为 `0.1`／`0.6` | 仅限定先验曲线，不限制在线后验。 |
| `onset_depth_m`／`saturation_depth_m` | `0.0002`／`0.002` | 低于起点保持下限，超过终点保持上限。 |
| `curvature` | `2.0` | 对数曲率，越大则前段增长越快。 |
| `min_contact_taxels` | `4` | 每侧至少四个有效承力触点。 |
| `stable_time_s`／`depth_tolerance_m` | `0.2 s`／`0.0001 m` | 深度裁剪到曲线有效区间后，相对窗口锚点超出容差时重新确认。 |
| `max_increase_per_s` | 当前配置为 `null`；粒子配置为 `0.05 s⁻¹` | `null` 取消控制 μ 上调限速；正数设置上调速率上限。 |

曲线确认使用 `clip(δ, δ₀, δ₁)`：超过饱和深度后，候选已经恒定，继续压缩不再重置
确认计时；离开饱和区、覆盖不足、风险或执行受阻仍按原门控重新确认。原始深度照常记录，
最大压缩行程保护不使用这个裁剪值。曲线参数是待标定候选。预载期间计算候选但不采用；active 内低风险稳定接触、新鲜数据、
执行未受阻及深度覆盖同时满足，持续确认后锁定本接触段先验。之后深度变化仅更新曲线诊断，
不逐帧重置后验。若此前尚无有效摩擦证据，锁定值初始化经典状态；粒子模式再以锁定值作为
`prior_mean` 初始化一次。已有有效事件（包括尚未改变控制 μ 的粒子事件和经典待确认事件）
或已接受的下界，不被随后更高的深度先验覆盖。现有粒子初始化为高斯样本裁剪，`prior_mean`
是裁剪前位置参数，实际样本均值可能不同，须看后验诊断。

估计状态与最终采用值分开保存。控制 μ 仅在先验已锁定、当前覆盖有效且接触低风险时上调；当前配置取消 μ 上调限速，确认后直接采用估计状态值，
更低证据可立即降低采用值。接触变化、观察失效、缺包或估计过期清除锁定并回退到 `0.1`；
深度缺失、覆盖不足或风险上升禁止退力。条件恢复后目标下降仍受既有 `1 N/s` 限制。
深度曲线不改变初始力、刚度预载或力上限；MIT 跟踪增益作为独立配置采用 `10／2`。

记录器 `1.11.0` 保留 `adaptive_{left,right}_depth_prior_depth_m`、`candidate`、`value`、
`locked`、`reason`，分别表示当前深度、曲线候选、锁定先验、锁定状态和原因。
`adaptive_left_mu`／`adaptive_right_mu` 始终是实际进入承载公式的系数，不能当作真实摩擦真值。
上述范围只限制先验，后续估计仍受统一 `0.05–2.0` 范围限制。
旁路回放显式关闭粒子与深度控制路径，不重建深度闭环采用轨迹。

### 最大压缩行程保护 {#max-contact-compression}

`safety.max_contact_compression_m` 在主力配置取 `0.040`，粒子受控试验配置收紧为 `0.020`；
表示**从首次任一侧接触起，两指合计最多再闭合一个上限值**，不是每侧一份限值。
此保护独立于摩擦先验 `2 mm` 饱和点；先验达到上限不代表允许继续无限压缩。
该限制由 `safety` 定义并始终生效。

保护基线早于或等于摩擦先验的双侧接触基线：前者防止单侧接触阶段绕过限值，后者避免
把物体平移当作摩擦先验的压入证据。运行时从首次接触即检查实测压缩量，覆盖接近确认、接触过渡、预载、active 和 holding。
测量达到上限时停止正常闭合；每次发送前还检查目标关节角及从最新实测角度出发的
一步速度预测，超过上限的命令不发送。统一模式通过既有故障保持路径等待人工 `release`；
正常释放与故障回位不被该限制阻断，不自动松夹。`compression_limit` 事件记录原因与压缩量。

保护基于主机采样与运动学，不能保证惯性、通信延迟和模型误差下的实际运动绝不超出配置上限；
它不代替硬件限位及原始力保护。实际限制还受机械开度、力上限共同约束；粒子候选的刚度预载
另有 `3 mm` 预载闭合上限，较严格的限制先起作用。

trace 追加 `contact_closure_m`、`contact_compression_m`、`contact_compression_limit_m`，
记录保护基线、实测总压缩量及配置限值，单位均为 m；`depth_prior_contact_closure_m`
另记录摩擦先验的双侧接触基线，摩擦深度诊断继续保留。

### 当前跟踪参数与摩擦采用路径

以下数值对应当前通用配置，粒子候选配置的控制参数并不完全相同。

| 参数 | 通用配置 | 对跟踪的影响 |
| --- | --- | --- |
| `controller.admittance.force_deadband_n` | `0.1 N` | 误差进入死区后冻结导纳状态，小力稳态不继续追求零误差。 |
| `controller.mit_kp`／`mit_kd` | `10.0`／`2.0` | 内环对位置参考的响应及速度阻尼。 |
| `controller.zero_tracking_velocity` | `true` | MIT 目标速度为零，速度项只阻尼实测速度。 |
| 导纳 `mass_kg`／`damping_ns_m`／`stiffness_n_m` | `0.2`／`15`／`1` | 决定外环响应速度、阻尼和恢复作用。 |
| `feedforward_ratio` | `1.0` | 完整机构模型前馈，受模型误差和力矩限幅影响。 |
| 控制频率／法向力低通 | `250 Hz`／`20 Hz` | 影响测量噪声、反馈延迟与动态跟踪。 |
| `unified.friction.filter_tau_s`／`gap_gain_per_s`／`load_rate_gain` | `0.05 s`／`8 s⁻¹`／`1` | 决定载荷变成法向目标的速度。 |
| 增力／减力速率 | `10`／`1 N/s` | 约束目标变化速度，不是稳态误差允差。 |
| `unified.tracking_error_n` | `0.3 N` | 执行受限且误差过大时触发保护，不是力控死区。 |

粒子候选当前使用 `100 Hz`、MIT `10／2`、`zero_tracking_velocity=false`；
还启用刚度预载及导纳质量／阻尼自适应（带宽 `10 rad/s`、阻尼比 `1`、平滑 `0.2 s`、
相对变化率 `1 s⁻¹`），并限制闭合／张开速度为 `10／5 mm/s`、加速度 `50 mm/s²`。
这些会影响动态滞后与振荡，不能把两份配置的表现差异全部归因于粒子滤波。

两条模式均保留有效低风险观察的整侧利用率下界更新；它不要求三个风险事件。
经典模式将合格事件候选乘 `0.8`，较低值立即接受，较高值需三次独立一致事件。
粒子模式关闭经典事件更新，只在合格事件后用后验 `10%` 分位（结合已有下界与最小摩擦约束）
下调摩擦状态；稳定帧不会直接通过粒子后验上调控制 μ。启用深度先验后，这些估计路径的
最终上调还要经过深度就绪、接触质量及 `0.05/s` 速率限制，较低证据不被先验抹除。

## 抓取中的在线摩擦估计 {: #online-friction-adaptive }

统一自适应入口不读取历史摩擦辨识结果。它以左右相同的 `0.1` 启动先验建立
抓取，在 active 阶段由逐触点纯力风险事件生成分侧摩擦候选；低于当前状态的保守候选立即接受，
经典事件候选提高摩擦需要三次独立一致事件，稳定观察的整侧下界另可更新摩擦状态。`friction_expiry_s: null` 使已接受的估计在当前连续接触段内
保持；接触集合变化、数据失效或新抓取开始时仍重置为启动先验。执行入口与命令见上文
“通用自适应抓取入口”与“统一自适应试运行”。

在线链路为风险事件产生候选、候选通过覆盖率和范围门禁、更新 `adaptive_left_mu`／
`adaptive_right_mu`，随后共享调度器按

\[
T_s=\left\|\sum_i(F_{x,s,i},F_{y,s,i})\right\|,\qquad
f_{\mathrm{raw}}=\gamma\max\left(\frac{T_L}{\hat\mu_L},\frac{T_R}{\hat\mu_R}\right)
\]

重新计算平均单侧法向目标。该公式使用二维切向合力模，不要求预先知道加载方向。目标可双向变化：
升降速率与上下限由 `safety` 单源定义、构造期派生，当前统一配置增力 `10 N/s`、
退力 `1 N/s`，最低保持 `1.0 N/侧`；风险事件只更新摩擦状态，
不再叠加固定增力预算或触发 `risk_budget_exhausted`。导纳控制允许跟随下降目标受限张开。
没有可信事件时，摩擦也可能由深度条件先验或稳定观察下界更新，不能据此解释为真实摩擦已知；
`lower_accepted`、`raise_accepted` 表示控制状态接受了新事件候选。`adaptive_left_candidate`／`adaptive_right_candidate` 保存当次候选，
候选由事件前整侧切向合力与法向合力之比生成，逐触点局部比值只定位发生重分配的区域。
事件候选由直接支持事件的受影响触点数整数门控（`min_event_taxels`，如取 `3` 即至少三个触点），
不是统计置信度。稳定无风险样本形成
`adaptive_*_mu_lower_bound`（各侧有效触点切向矢量和的模除以法向力之和）；显著低于该下界的候选以 `below_observed_lower_bound` 拒绝。
每个有效触点也积累局部 `hypot(Fx,Fy)/Fz` 下界，左右各侧当前有效触点历史下界的最大值
分别记录为 `adaptive_left_taxel_mu_lower_bound` 与 `adaptive_right_taxel_mu_lower_bound`。
共同摩擦系数假设下局部最大值可提供更强下界；存在异质接触时，它仅约束对应局部摩擦，
不能直接否决整侧等效摩擦候选。全局与局部均来自同一组触觉数据，不增加独立事件计数，
稳定低风险的整侧下界可提高估计状态，局部下界仅作诊断；启用深度先验后最终采用值仍受上调门控。下界成立依赖粘着假设和有效力测量，
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
| `config.json` | 输入路径、元数据、有效配置及与默认配置的净差异 `delta` 小节 |
| `events.jsonl` | 阶段、接触、目标启用、故障及诊断 |
| `tactile.jsonl` | 全局力、逐 taxel 原始三轴力、包连续性 |
| `trace.csv` | 控制周期记录，schema 为 `dmgripper-experiment/v1` |
| `manifest.json` | 结果、原始／清理故障、失能确认、产物清单 |
| `plot.pdf`／`plot.png` | 结果图 |

启动后事件流首先写入 `safety_limits` 限幅表：最终生效的目标上限、原始过力线、增／退力速率、
压缩行程与派生的统一调度包络，离线审计可将它与 `config.json` 的 `delta` 小节逐项对照。

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
该字段设为 `null` 时保留常值预载目标；active 阶段的增力／卸力速率独立。
此次仅改变滤波与预载建立方式，导纳质量、阻尼及 MIT 增益保持现值，效果需真机记录确认。

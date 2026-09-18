# dmgripper-experiments 0.2.0

DM4310P 与双侧 PapillArray 的通用纯 Python 真机抓取实验包。它不依赖 ROS 或 MuJoCo，把基础
力跟踪与原倒水实验收敛为同一条生命周期：预检、就绪、必要时受限回零、受限接近、接触确认、
速度过渡、初始抓力稳定、正式运行、保持抓握、显式释放回位和结束。目标力来自给定时间曲线或触觉动态增力，控制器
在二阶导纳、PID 与一阶位置型 LADRC 中独立选择，互不绑定。

先审阅计划（默认 dry-run，不导入运行时、不打开设备）：

```sh
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/force_curve.yaml
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/adaptive_grip.yaml
```

确认传感器完全无负载、急停可用后，在交互终端执行（运行时输入 `s`/`start`、`status`、`r`/`release`，
每条命令都需按 `Enter` 提交）：

```sh
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/adaptive_grip.yaml --bias --execute
```

交互终端的实时面板底部保留固定 `命令>` 输入区；输入内容、退格与回车提交不受状态刷新干扰。
`s` 和 `r` 分别是 `start` 与 `release` 的快捷输入，两者仍需按 `Enter` 确认。

使能前的零力验证会逐帧阻断 taxel 缺失、数量变化、非有限值和包序异常，并将逐 taxel 残余统计写入
诊断；是否通过仍以 `lifecycle.zero_force_stable_s` 窗口中双侧滤波 `Fz` 绝对值的均值为门禁：左右
均值均不大于 `lifecycle.zero_force_threshold_n` 才会进入 `ready`。均值门禁通过后，若整个验证期内任一
滤波峰值达到 `lifecycle.contact_on_n`，终端和 `events.jsonl` 会输出一次结构化 `warning`（最大峰值、
触发侧、告警阈值），不会重置窗口或单独阻止使能；这不是带载运行许可，仍应检查传感器是否完全空载。
触觉有效性／新鲜度、原始法向力上限和双侧不平衡保护，以及运行期的接触、过力、失接触和尽力失能保护
均不受影响。

非交互执行只允许明确的无人值守组合：`lifecycle.auto_start=true` 且 `lifecycle.on_finished=return`。
正常结束默认保持抓握（`on_finished: hold`），等待用户显式 `release` 后才受限张开回位并失能。
使能后若实验级故障发生且 DM 反馈与命令通道仍健康，运行进入 `fault_holding`，在当前位置受限保持并
等待操作者承接物体后输入 `release`；通信丢失、电机故障、意外失能、反馈越界、保持失败和 `Ctrl+C`
等无法保持的故障会立即尽力失能。两类路径都把原始故障、保持结果与清理故障分开写入 manifest。
控制过程即使已经结束，只要失能、设备关闭、采集停止或终端收尾失败，本次运行仍标记为
`failed` 并返回非零状态；失能确认只由真实设备操作结果决定，不受事件显示失败影响。

输出按 `outputs/real/<task_name>/<object_name>/<UTC时间戳>-<run_id>/` 组织，包含有效配置、
事件 JSONL、原始触觉 JSONL、控制 trace CSV 与 manifest；正常结束后自动生成 `plot.pdf`／
`plot.png`。离线重绘不覆盖历史原件：

```sh
uv run --package dmgripper-experiments dmgripper-plot --repaint <运行目录>
```

`terminal.mode=auto` 只在 Rich 判定为兼容且非 dumb 的交互终端中启用动态面板，否则自动使用
纯文本；`terminal.mode=json` 输出无 ANSI 的 JSON 行。Rich 面板展示阶段、任务时间、左右法向力、
目标力、切向力、刚度估计及有效性、开度、限幅状态、控制周期与触觉年龄。界面明确显示可用命令，
收到 `start` 后会先显示并记录确认，再进行电机连接、检查和使能；未知命令会给出警告。Rich 由单一
刷新线程驱动，预检与 `ready` 使用不移动光标的静态面板，产生首个控制快照后才启动 Live；纯文本
快照也按 `terminal.refresh_hz` 合并输出。
Rich 面板停止后会额外输出一行稳定摘要，包含最终状态、失能确认和运行目录；若主故障或 `Ctrl+C`
同时伴随清理失败，终端会直接显示清理警告和 manifest 所在目录。终端刷新不参与控制时钟。

等效接触刚度估计只用于诊断：估计量、有效性、更新时刻与原因写入 trace，不改变控制命令。
估计初值与门限沿用仿真验证起点，尚不是真机辨识值。

PID／LADRC 路径把原始力交给共享核，由核心内部做唯一一次低通（trace 同时记录原始力、外层
滤波力与控制使用力）；导纳路径沿用外层滤波力，死区与单向闭合是导纳专属参数。曲线模式含下降
段时与导纳 `prevent_unloading` 互斥，计划阶段直接报错。
预载在确认双侧接触后，只要平均力连续高于
`目标-preload_tolerance_n` 即可进入正式阶段，不设高侧稳定窗口。

## 迁移说明

旧入口 `dmgripper-force-demo` 与 `dmgripper-cup` 已被取代：调用时打印迁移提示并拒绝执行，
不会把旧场景交互映射成自动阶段推进。`dmgripper-cup-plot` 保留为通用历史读取器的薄别名，
可重绘历史 v1／v2 cup trace（`state` 列自动按 `phase` 解释）。历史运行目录继续可读、可重绘。
操作细节见 [`docs/dmgripper-experiments.md`](../../docs/dmgripper-experiments.md)。

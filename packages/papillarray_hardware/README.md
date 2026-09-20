# papillarray-hardware 0.1.0

Contactile PapillArray 的 PTS v2.0 纯 Python 同步串口采集边界。它只处理字节流、协议帧和设备
控制命令；不包含 ROS、MuJoCo、DM4310P、Robotiq 或任何夹爪控制算法。因此 DM 与 Robotiq 的
上层控制律可以共用同一份触觉观测，而不会互相依赖。

## 边界与安全性

- `PapillArraySerialClient` 构造时不会导入 PySerial 或访问设备；只有显式调用 `open()` 才会
  创建串口。测试可注入内存 fake serial。
- 默认串口配置为 udev 别名 `/dev/papillarray`、`115200 baud`、`1000 Hz`。可选采样率严格限于
  `100`、`250`、`500`、`1000 Hz`；默认要求每包报告 `2` 个传感器，数量不符时拒绝上送。
- `configure_stream()` 才会写入采样率命令；`clear_bias()` 会写入设备清零／偏置清除命令
  `z\n`，调用前必须确保传感器无负载。该包不会自动执行清零。
- `PtsStreamReader` 处理半包、前导噪声、坏校验和和异常长度，并保留控制器的 `packet_counter`
  与 `timestamp_us`。Type 1／3／4／5／6 被解码；未定义布局的 Type 7 仅以原始 `bytes`
  保存，绝不猜测其含义。
- `TactileWorker` 在单一后台线程中组合串口客户端、包完整性跟踪和双侧全局 `Fz` 低通，
  `TactileSnapshot` 同时保留左右逐 taxel 三轴原始力。采集会话拒绝空 taxel、shape 或有限性
  错误、运行中 taxel 数变化、重复／乱序计数器及非递增设备时间；合法缺包和 uint32 回绕
  作为 `counter_event`／`counter_gap` 诊断随快照上送。
- 请求 bias 时，采集会话先等待首个完整有效包，再只发送一次 `z\n`，并清空协议与操作系统串口
  接收缓冲；bias 前包不发布也不记录。发送后重置滤波和设备时间／计数器基线，但保留本次会话
  已经确认的 taxel 拓扑。
- 顶层索引与首个数据块之间允许 Controller v2.0 实际输出的全零对齐填充；非零的未声明
  字节仍按协议结构错误拒绝。
- Type 3／4／5／6 的嵌套索引均以去除起止标志后的整段帧数据起点为绝对基址，不以所属
  Type 块或传感器块为基址；索引头、条目边界与未声明间隙均会严格校验。
- 上层应为实机循环额外实现观测新鲜度、丢包统计、执行器互锁和急停策略。

## 最小使用方式

```python
from papillarray_hardware import PapillArraySerialClient, PapillArraySerialConfig

config = PapillArraySerialConfig()
with PapillArraySerialClient(config) as tactile:
    tactile.configure_stream()
    packet = tactile.read_packet()
    print(packet.packet_counter, packet.timestamp_us)
```

真实硬件接入前，应先确认设备端口和供电，且在具备急停与无负载条件下才执行
`clear_bias()`。

## 只读采集探针

安装该 workspace 后，可用下列命令采集有限个包并输出 JSON Lines：

```sh
uv run --package papillarray-hardware papillarray-probe
```

`--port` 默认使用 `/dev/papillarray`，仍可显式覆盖；`--baud`、`--rate`、`--expected-sensors`、`--count`、`--timeout` 和
`--packet-timeout` 分别默认为 `115200`、`1000`、`2`、`10`、`1` 秒和 `3` 秒。`--timeout` 是
一次底层串口 `read()` 的超时；`--packet-timeout` 是等待一个校验通过 PTS 包的总时限，必须不小于
`--timeout`。Controller 在首次收到 `f<rate>\n` 配置命令后可能存在启动延迟：一次空读取不会立即
令探针失败，而会在 `--packet-timeout` 到达前继续读取。即使设备持续输出噪声、坏帧或半包也会在总
时限退出。由于底层 `read()` 最多可阻塞 `--timeout` 秒，实际总等待可能比 `--packet-timeout` 晚至多
一个单次读取周期。例如单传感器部署可显式写为：

```sh
uv run --package papillarray-hardware papillarray-probe \
  --expected-sensors 1 --count 20 --packet-timeout 3
```

探针显式打开端口后只会发送一次采样率配置命令 `f<rate>\n`，绝不会发送清零 `z\n` 或滑动
检测 `S\n`／`s\n` 命令。每一行包含主机单调时钟接收时间、设备包计数器和时间戳、每个传感器的
全局力／力矩和 pillar 数。`counter_event` 依次标记 `first`、`consecutive`、`gap`、`wrap`、
`duplicate` 或 `out_of_order`；前进或回绕时 `counter_gap` 是可解释的缺失包数，首次、重复和乱序
时为 `null`。计数器按无符号 32 位半模规则判定，因而重复／乱序不会被误报为巨量丢包。传感器数
不符、协议错误或超时会在标准错误给出诊断并以非零状态退出。

若探针超时，标准错误会输出一次等待的协议诊断，包括“接收字节”“空读取”“起始标志”“结束标志”“候选帧”、
“校验失败”“结构失败”“超长丢弃”“最后协议错误”和最多 64 字节的“原始十六进制预览”。例如
持续收到非 PTS 数据时，可预期类似：

```text
PapillArray 探针超时：等待有效 PTS 包超过总时限。协议诊断：接收字节=…，起始标志=0，…，原始十六进制预览=…。
```

这类诊断只读取设备输出，探针仍只发送 `f<rate>\n`，不会执行清零或滑动检测命令。

## 独立记录与手柄实验

`papillarray-record` 不依赖夹爪控制进程，可与 `dmgripper-teleop` 分别运行在两个终端。
记录器以独占目录保存 `tactile.jsonl`、`events.jsonl`、`config.json` 和 `manifest.json`。
显式传入 `--bias` 时，采集线程在首个完整包后发送一次清零命令；启动时传感器必须完全无负载。

显式传入 `--estimate-friction` 后，记录器根据双侧逐触点接触集合和全局法向力变化率判断稳定接触，
连续满足 `--stable-duration` 后启动一次自主估计。启动时冻结参考接触集合；活动期允许扰动引起的
边缘触点变化；整侧失去有效接触持续 `30 ms`、失鲜、缺包或退出时停止。该模式不调用原厂滑移服务，
不会发送 `S\n`／`s\n`。按单 pillar `Fz` 测量误差尺度 `0.05 N`，默认逐触点接触进入／退出门槛
设为 `0.15/0.10 N`；较小信号仍完整记录，但不进入参考接触集合。

自主估计器不读取原厂状态或原厂摩擦值：对每个参考
pillar 计算 `rho=hypot(Fx,Fy)/Fz`，只在比值先上升、随后饱和或发生剪切重分配并持续确认时，
冻结候选开始前 `0.2 s` 窗口的 80% 分位数。`raw_mu` 保留未经安全折减的实验估计，
`conservative_mu=clip(0.8*raw_mu,0.05,2.0)` 预留控制裕量。普通粘着阶段的 `rho` 只表示已用
摩擦比例，不产生估计。`events.jsonl` 保存 `own_friction_state` 和 `own_friction_estimate`；
当前命令只记录候选，不向夹爪发送目标力。

```sh
# 终端 A：空载启动；收到 bias 后首个有效包再用手柄闭合接触
uv run --package papillarray-hardware papillarray-record \
  --port /dev/papillarray --bias \
  --estimate-friction \
  --contact-on 0.15 --contact-off 0.10 \
  --output outputs/real/papillarray/manual-friction-01

# 终端 B：独立控制夹爪，不访问触觉串口
uv run --package dmgripper-hardware dmgripper-teleop \
  --port /dev/dmj4310_can --execute
```

终端 A 显示“自主逐 pillar 摩擦估计已启动”后，再沿触觉面缓慢施加切向扰动。整个实验结束时
先用手柄张开并失能，再在终端 A 按 `Ctrl-C`；无需按 Enter。`Ctrl-C` 只结束自主估计与记录，
不能替代夹爪端的失能操作。

`--native-slip` 与 `--native-slip-duration` 仅保留给需要复现实验或做原厂对照的场景；日常自主估计
不要传入这两个参数。

## 示例脚本

探针 CLI 用于快速验证设备是否存活；`examples/` 下的脚本用于演示上层应如何正确使用这一采集边界
（显式生命周期、控制命令时序、新鲜度检查与丢包统计）。默认端口均为 udev 别名 `/dev/papillarray`。

### 读取触觉数据包

`read_packets.py` 是"最小使用方式"的可运行教学版，读取有限个包并打印指定传感器的全局力表格：

```sh
uv run --package papillarray-hardware python packages/papillarray_hardware/examples/read_packets.py --port /dev/papillarray
```

### 滑动检测

`slip_detection.py` 演示探针刻意不发送的滑动检测启停命令，启动滑动检测后打印每传感器的
pillar 滑动状态与目标抓握力。注意：该示例会向设备发送 `S\n`／`s\n` 命令：

```sh
uv run --package papillarray-hardware python packages/papillarray_hardware/examples/slip_detection.py --port /dev/papillarray
```

### 持续监测与丢包统计

`monitor.py` 是最接近驱动主循环的同步教学版，演示观测新鲜度检查与按无符号 32 位半模规则统计丢包。
启动预热会丢弃设备 FIFO 残留的陈旧包——观察到计数器大步跳变且跳变后恢复连续才作为统计起点，缺包数
因此不含上次会话与本次运行之间的间隔。注意：只有显式传入 `--bias` 时才会发送 `z\n` 清零命令，且发送前传感器必须完全无负载：

```sh
uv run --package papillarray-hardware python packages/papillarray_hardware/examples/monitor.py --port /dev/papillarray
```

## 协议来源与许可

协议布局与控制命令以下列远程实现为参考，链接固定指向与记录的 SHA-256 对应的提交：

- [`papillarray_serial_driver/protocol.py`](https://github.com/langxin11/contactile-papillarray-ros2/blob/3dd143c19c4c88c69f0fe2a5e8c60aa37f0156f3/papillarray_serial_driver/papillarray_serial_driver/protocol.py)
- [`papillarray_serial_driver/serial_worker.py`](https://github.com/langxin11/contactile-papillarray-ros2/blob/3dd143c19c4c88c69f0fe2a5e8c60aa37f0156f3/papillarray_serial_driver/papillarray_serial_driver/serial_worker.py)

上述参考文件在本次实现时的 SHA-256 分别为
`ddacbdd228705b170c3e358e48507ec245c95adc31c94042641e23a0542423f1` 与
`4e12495ec91494a0c3918b0f6001ae28393182776678ac9099ebd1531dacd66f`。

参考包声明为 `Proprietary`。本包同样为 `Proprietary`，不声称开源，也未复制 Apache License。
发布或向仓库外分发前，应另行确认 Contactile 协议与参考实现的授权范围。

## 本地验证

```sh
uv run ruff check packages/papillarray_hardware
uv run ruff format --check packages/papillarray_hardware
uv run --package papillarray-hardware pytest packages/papillarray_hardware/tests
```

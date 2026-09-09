# papillarray-hardware 0.1.0

Contactile PapillArray 的 PTS v2.0 纯 Python 同步串口采集边界。它只处理字节流、协议帧和设备
控制命令；不包含 ROS、MuJoCo、DM4310P、Robotiq 或任何夹爪控制算法。因此 DM 与 Robotiq 的
上层控制律可以共用同一份触觉观测，而不会互相依赖。

## 边界与安全性

- `PapillArraySerialClient` 构造时不会导入 PySerial 或访问设备；只有显式调用 `open()` 才会
  创建串口。测试可注入内存 fake serial。
- 默认串口配置为 `/dev/ttyACM0`、`115200 baud`、`500 Hz`。可选采样率严格限于
  `100`、`250`、`500`、`1000 Hz`；默认要求每包报告 `2` 个传感器，数量不符时拒绝上送。
- `configure_stream()` 才会写入采样率命令；`clear_bias()` 会写入设备清零／偏置清除命令
  `z\n`，调用前必须确保传感器无负载。该包不会自动执行清零。
- `PtsStreamReader` 处理半包、前导噪声、坏校验和和异常长度，并保留控制器的 `packet_counter`
  与 `timestamp_us`。Type 1／3／4／5／6 被解码；未定义布局的 Type 7 仅以原始 `bytes`
  保存，绝不猜测其含义。
- 上层应为实机循环额外实现观测新鲜度、丢包统计、执行器互锁和急停策略。

## 最小使用方式

```python
from papillarray_hardware import PapillArraySerialClient, PapillArraySerialConfig

config = PapillArraySerialConfig(port="/dev/ttyACM0")
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
uv run --package papillarray-hardware papillarray-probe --port /dev/ttyACM0
```

`--port` 必填；`--baud`、`--rate`、`--expected-sensors`、`--count` 和 `--timeout` 分别默认
为 `115200`、`500`、`2`、`10` 和 `1` 秒。例如单传感器部署可显式写为：

```sh
uv run --package papillarray-hardware papillarray-probe \
  --port /dev/ttyACM0 --expected-sensors 1 --count 20
```

探针显式打开端口后只会发送一次采样率配置命令 `f<rate>\n`，绝不会发送清零 `z\n` 或滑动
检测 `S\n`／`s\n` 命令。每一行包含主机单调时钟接收时间、设备包计数器和时间戳、每个传感器的
全局力／力矩和 pillar 数。`counter_event` 依次标记 `first`、`consecutive`、`gap`、`wrap`、
`duplicate` 或 `out_of_order`；前进或回绕时 `counter_gap` 是可解释的缺失包数，首次、重复和乱序
时为 `null`。计数器按无符号 32 位半模规则判定，因而重复／乱序不会被误报为巨量丢包。传感器数
不符、协议错误或超时会在标准错误给出诊断并以非零状态退出。

## 协议来源与许可

协议布局与控制命令以本机下列已有实现为参考：

- `/home/xiaodaliang/workspace/maintained/tactile_grasp_ros2/src/contactile-papillarray-ros2/papillarray_serial_driver/papillarray_serial_driver/protocol.py`
- `/home/xiaodaliang/workspace/maintained/tactile_grasp_ros2/src/contactile-papillarray-ros2/papillarray_serial_driver/papillarray_serial_driver/serial_worker.py`

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

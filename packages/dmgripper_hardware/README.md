# dmgripper-hardware 0.1.0

DM4310P 与 USB2CAN 的纯 Python 硬件基础包。当前版本包含可离线验证的协议编码、反馈解码、
分段接收帧重组、内存 `FakeTransport`、延迟打开的 `PySerialTransport`、只读
`DmStateRefresher`，以及默认 dry-run 的受限小步运动探针；没有 CAN、ROS、MuJoCo、触觉采集或
Robotiq 依赖。

## 边界与安全性

- `Usb2CanProtocol` 只返回或解释 `bytes`，不会打开端口，也不会向设备发送命令。
- `PySerialTransport` 在构造时不会导入 PySerial 或打开端口；只有调用方显式 `open` 才会
  创建串口。其工厂可注入，测试只使用内存 fake，不访问 `/dev`。
- `Usb2CanDeviceConfig` 严格校验端点、波特率、不同且非零的电机／主机 CAN ID，以及有限正的
  刷新超时。默认波特率为 `921600`。
- `MotorLimits` 没有通用默认值。当前夹爪的 `make_dm4310p_gripper_config()` 固化用户提供的、
  无 I/O 的部署配置：电机协议量程为位置 `[-1.7, 1.7] rad`、速度 `[-8, 8] rad/s`、
  力矩 `[-4, 4] N·m`；机械关节行程为 `[0, pi/2] rad`，角度增大方向为闭合方向。
- 电机协议量程和机械关节行程是两个不同边界。`validate_joint_position()` 对机械目标越界直接
  抛出 `ValueError`，不做静默 clamp；构造配置时也会检查机械行程位于协议位置量程内。
- `DmStateRefresher.refresh_once()` 唯一允许写入 `make_feedback_request()` 创建的状态刷新帧；
  它会按总超时读取、分帧、忽略噪声与无关帧并返回目标反馈。它不实现使能、失能、置零、控制
  报文或寄存器读写；上层必须单独实现互锁、反馈新鲜度与急停策略。
- 未实现 USB2CANFD，也不对其帧格式或接口做任何假设。

## 受限小步运动探针

`dmgripper-motion-probe` 用于通过纯 Python 对首次 DM4310P 联调执行受限的三阶段动作。它先读取
初始反馈，拒绝故障或机械行程外的状态；随后记录精确初始位置作为保持和回位
目标，并按显式配置的正步长设置闭合方向的阻抗平衡点。机械行程始终是
`[0, pi/2] rad`，越界目标直接
拒绝，不作截断。

默认运行是 dry-run：只发送状态查询和控制模式寄存器 `10` 的只读请求，并输出 JSON Lines 计划，
不会使能、发送 MIT 命令、写控制模式或失能。计划同时列出初始位置／速度／力矩／状态码、读回模式、
`kp`／`kd`、闭合步长与阶段持续时间，供执行前人工核对：

```sh
uv run --package dmgripper-hardware dmgripper-motion-probe \
  --port /dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00
```

`--execute` 会拒绝非 MIT 模式（寄存器 `10` 必须为 `1`），并且使能后必须收到新的
`status_code=1` 反馈；不会隐式切换控制模式。每条使能或 MIT 命令后只等待并解析该命令产生的
一条反馈，不再附加状态查询，避免反馈队列积压。MIT 的 `q_des` 是阻抗虚拟平衡点，不是必须到达的
轨迹 waypoint：保持、闭合、回位均在 `--stage-duration`（默认 `1 s`）内持续发送目标；只要通信、
模式、使能状态与机械位置持续正常，位置误差不会判为失败。执行 JSON 仍报告 `target_reached` 与位置
误差，供诊断而非作为通过条件。

成功执行时，三条阶段 JSON 后还会输出一条
`{"mode":"execute","event":"complete","disable_confirmed":true}`。它只表示三阶段运行结束且已读回
`status_code=0`，不表示最终机械位置已经回到初始位置；应从 `return` 阶段记录的
`final_position_rad` 与 `position_error_rad` 判断回位误差。

闭合推进可用 `--mit-kp` 在协议 `(0, 500]` 内调整，默认仍为 `2.0`；`--closing-step` 接受任意正有限
数值，默认 `0.03 rad`。`kd` 固定为 `0.5`，不允许前馈力矩。最终闭合目标必须通过机械行程
`[0, pi/2] rad` 校验，越界直接拒绝且不截断。速度与力矩完整记录在每阶段 JSON 中，但不使用人为阈值
中止正常实验；故障状态、机械位置越界、通信读写超时、模式／使能／最终失能确认失败仍为硬失败。

`PySerialTransport` 会在每次控制或只读请求写入期间临时设为设备 `timeout_s`，并在结束后恢复原有
`write_timeout`；这使 PySerial 层的写入等待有界，但不替代外部急停或硬件功能安全。以下 `--execute`
仍仅适合无负载、急停可用的受控验收：

```sh
uv run --package dmgripper-hardware dmgripper-motion-probe \
  --port /dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00 \
  --mit-kp 5.0 --closing-step 0.06 --stage-duration 1.0 \
  --execute
```

执行顺序严格为：使能并以初始反馈位置保持，按配置步长设置闭合阻抗平衡点，再将阻抗平衡点恢复为初始位置，最后
失能。MIT 命令的默认值参考 `dm_force_tracking.yaml` 中的 `mit_kp=2.0`、`mit_kd=0.5`；
`kp` 可由命令行显式调整，不添加前馈力矩。
正常结束时失能帧短写会使命令以非零状态失败；通信超时、故障、机械范围越界和 Ctrl-C
均会尽力发送已验证的 `CMD_DISABLE`，随后以一帧有界状态查询确认 `status_code=0`，最后关闭串口。

该探针不是急停、功能安全互锁或硬实时控制器：保护反应仍受 USB、串口、USB2CAN 和固件时延限制。
现有协议来源只验证了 `CMD_ENABLE`、`CMD_DISABLE` 和 MIT 帧，没有独立、可证明的停止帧，因此本包
不会臆造停止 CAN 报文或执行置零。

## 只读状态探针

从仓库根目录可直接运行有限次数的 DM4310P 状态探针：

```sh
uv run --package dmgripper-hardware dmgripper-state-probe \
  --port /dev/ttyUSB0 --count 10 --interval 0.1 --timeout 0.05
```

命令输出稳定的 JSON Lines，包含主机单调时钟接收时间、序号、位置、速度、力矩、状态码、
故障标记和 `within_mechanical_range`。探针固定使用 `motor_id=1`、`master_id=17`、
`921600` 波特率以及 `make_dm4310p_gripper_config()` 的协议量程；每次只通过
`DmStateRefresher` 发送状态查询帧。机械行程外的反馈只会标记为 `false`，不会截断反馈、
改变目标或发送任何运动命令。

安全警告：这是只读查询，不是急停或安全互锁。首次连接前请在无负载、急停可用的条件下核对
串口端点、CAN ID、固件量程和 USB2CAN 接线；确认设备不会因状态查询而产生意外动作。发生
异常或按下 Ctrl-C 时，探针会关闭传输并以非零状态退出。

## 使用示例

```python
from dmgripper_hardware import MitCommand, Usb2CanProtocol, make_dm4310p_gripper_config

deployment = make_dm4310p_gripper_config("/dev/ttyUSB0")
target_joint_rad = deployment.validate_joint_position(0.2)
protocol = Usb2CanProtocol(deployment.motor_limits)
packet = protocol.make_mit_packet(
    deployment.motor_id,
    MitCommand(target_joint_rad, 0.0, 0.0, 0.0, 0.0),
)

# packet 仅是 30 字节；此处不发生 I/O。
assert len(packet) == 30
```

`DmMitCommandAdapter.prepare()` 可把 `dm_grasp_core.MITCommand` 显式映射为独立协议命令并
生成 30 字节帧。返回的 `PreparedMitCommand` 同时保留核心请求、协议对象和最终字节，便于审计；
该过程不持有传输对象，也不会发送帧。核心与协议的同名命令类型不会混用。

## 只读状态刷新示例

下例显示显式生命周期。真正使用 `PySerialTransport` 前，应先在无负载、急停可用的条件下核对
端点、CAN ID 和量程；构造刷新器本身不会执行 I/O。

```python
from dmgripper_hardware import (
    DmStateRefresher,
    MotorLimits,
    PySerialTransport,
    Usb2CanDeviceConfig,
    Usb2CanProtocol,
)

config = Usb2CanDeviceConfig("/dev/ttyUSB0", motor_id=1, master_id=17, timeout_s=0.05)
protocol = Usb2CanProtocol(MotorLimits(-1.7, 1.7, -8.0, 8.0, -4.0, 4.0))
refresher = DmStateRefresher(config, protocol, PySerialTransport())

with refresher:
    feedback = refresher.refresh_once()
```

## 协议来源

实现以仓库中已有的 ROS 2 C++ 适配器为直接来源，数值量化、字节序、30/16 字节帧布局、
CAN ID 偏移和反馈解析与下列文件保持一致：

- `tactile_grasp_ros2/src/dm_gripper_control/dmj4310_driver_cpp/include/dmj4310_driver_cpp/usb2can_protocol.hpp`，SHA-256：`e073cb8f390449406896714ad545c0974db88f37d276853aafa8fe557cec49d2`。
- `tactile_grasp_ros2/src/dm_gripper_control/dmj4310_driver_cpp/src/usb2can_protocol.cpp`，SHA-256：`b654c7f43af533f3090fd30f277ed5b51647840b5abb5f0ad214f6926f7dce76`。
- `tactile_grasp_ros2/src/dm_gripper_control/dmj4310_driver_cpp/src/hardware_io.cpp`，SHA-256：`cd0bb11f3db24ba681428937e1f978bcddb4181b6e51fe01ea1ebc3931609380`。

另以固定达妙官方 SDK `DM_CAN.py` 的 `send_data_frame`、MIT 打包、参数帧与 16 字节接收帧
提取逻辑交叉核对：

- 上游仓库 `dmBots/motor-sdk`，固定提交 `0b2ede457bdbf0882e29ab9958ab8fda047b7f4a`；
  本地参考文件 `tactile_grasp_ros2/src/dm_gripper_control/dmj4310_driver/dmj4310_driver/vendor/DM_CAN.py`，
  SHA-256：`78f1d6bc14805f8d5ca5c5a3399c57efaecc055e77dacf838864a076de92a75b`。

参考 ROS 2 部署配置使用 `motor_id=1`、`master_id=17` 和 `[-1.7, 1.7] rad`、`[-8, 8] rad/s`、
`[-4, 4] N·m`。这些值已经过离线协议向量核对，但尚未在当前实机上验收。

## 未验证的实机假设

当前 DM4310P 与 USB2CAN 的只读状态刷新已完成真机通信核验；受限运动探针尚未完成真机验收。
USB2CANFD 仍未接入。首次执行 `--execute` 前仍须在无负载、急停可用的条件下核验实际串口设备路径与
波特率、从机／主机 CAN ID、固件 PMAX／VMAX／TMAX、当前固件是否接受上述 USB2CAN 帧封装，以及
反馈状态码的设备表现。

## 本地验证

```sh
uv run ruff check packages/dmgripper_hardware
uv run ruff format --check packages/dmgripper_hardware
PYTHONPATH=packages/dmgripper_hardware/src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  uv run pytest packages/dmgripper_hardware/tests
```

## 许可

Apache-2.0。`LICENSE` 复制自承载仓库的许可证。达妙官方 `DM_CAN.py` 未被复制到本包；其上游
许可证为 MIT，来源与固定提交记录于参考 ROS 2 包的 `vendor/UPSTREAM.md`。

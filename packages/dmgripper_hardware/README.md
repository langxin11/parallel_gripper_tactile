# dmgripper-hardware 0.1.0

DM4310P 与 USB2CAN 的纯 Python 硬件基础包。当前版本包含可离线验证的协议编码、反馈解码、
分段接收帧重组、内存 `FakeTransport`、延迟打开的 `PySerialTransport`，以及只读
`DmStateRefresher`；没有 CAN、ROS、MuJoCo、触觉采集或 Robotiq 依赖。

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

本包尚未接入当前 DM4310P、USB2CAN 或 USB2CANFD，故以下内容仍需先用只读状态刷新在无负载、
急停可用的条件下核验：实际串口设备路径与波特率、从机／主机 CAN ID、固件 PMAX／VMAX／TMAX、
当前固件是否接受上述 USB2CAN 帧封装，以及反馈状态码的设备表现。

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

# dmgripper-hardware 0.1.0

DM4310P 与 USB2CAN 的纯 Python 硬件基础包。当前版本只包含可离线验证的协议编码、反馈解码、
分段接收帧重组和内存 `FakeTransport`；没有串口、CAN、ROS、MuJoCo、触觉采集或 Robotiq 依赖。

## 边界与安全性

- `Usb2CanProtocol` 只返回或解释 `bytes`，不会打开端口，也不会向设备发送命令。
- `ByteTransport` 是未来真实串口适配器的最小接口；本批次只实现不接触硬件的
  `FakeTransport`。因此安装本包或运行其测试不会访问 USB2CAN。
- `MotorLimits` 没有默认值。实际的 PMAX、VMAX、TMAX 必须由调用方按固件读回值或已验证部署
  配置显式传入，不能仅根据 DM4310P 型号猜测。
- 本包保留 MIT、位置速度、速度、力位混合、位置速度 CSP、速度 CSP、力矩 CSP、状态刷新与
  寄存器报文的协议格式，但不实现使能、失能、控制模式切换或寄存器写入的执行流程。上层硬件
  适配器必须单独实现互锁、时效检查、反馈新鲜度与急停策略。
- 未实现 USB2CANFD，也不对其帧格式或接口做任何假设。

## 使用示例

```python
from dmgripper_hardware import MitCommand, MotorLimits, Usb2CanProtocol

limits = MotorLimits(-1.7, 1.7, -8.0, 8.0, -4.0, 4.0)
protocol = Usb2CanProtocol(limits)
packet = protocol.make_mit_packet(0x01, MitCommand(0.0, 0.0, 0.0, 0.0, 0.0))

# packet 仅是 30 字节；此处不发生 I/O。
assert len(packet) == 30
```

## 协议来源

实现以仓库中已验证的 ROS 2 C++ 适配器为直接来源，数值量化、字节序、30/16 字节帧布局、
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
`[-4, 4] N·m`，但这些只是该设备的已验证配置，不构成本包默认值或对当前实机的保证。

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

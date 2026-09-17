# 串口设备别名规则

本目录保存当前实验台的 udev 部署规则，为 PapillArray Controller、达妙 USB2CAN 和 Robotiq 2F85 的 USB／RS485 转换器创建稳定的设备别名。规则绑定当前硬件的 USB 供应商标识、产品标识和序列号，属于本实验台的部署特定项，不是通用硬件配置。

## 当前别名

| 设备 | 别名 | 供应商标识 | 产品标识 | 序列号 |
| --- | --- | --- | --- | --- |
| PapillArray Controller | `/dev/papillarray` | `16c0` | `0483` | `15954100` |
| 达妙 USB2CAN | `/dev/dmj4310_can` | `2e88` | `4603` | `00000000050C` |
| Robotiq 2F85 USB／RS485（FTDI） | `/dev/robotiq2f85` | `0403` | `6015` | `DAASND29` |

Robotiq 别名绑定 USB／RS485 转换器，不识别下游夹爪型号或 Modbus 地址。更换转换器时需更新序列号；将该转换器接到其他设备时，别名仍然相同。

## 安装与验证

将规则文件复制到系统 udev 规则目录，然后重新加载规则并触发设备重新识别：

```bash
sudo cp deploy/udev/99-parallel-gripper-tactile.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

如果触发后别名没有出现，请在停止相关设备程序后重新插入对应 USB 设备。检查已连接设备的别名是否存在：

```bash
ls -l /dev/papillarray /dev/dmj4310_can /dev/robotiq2f85
```

规则将设备权限设置为 `dialout` 组可读写（`0660`）。运行采集或电机程序的用户需要加入该组；加入后请重新登录，或重新启动当前会话使组权限生效：

```bash
sudo usermod -aG dialout "$USER"
```

Robotiq 别名生效后，在仓库根目录启动遥控窗口：

```bash
uv run --package robotiq-hardware --extra teleop robotiq-teleop --port /dev/robotiq2f85
```

窗口连接后仍需手动激活；激活会产生设备运动，操作步骤见 [Robotiq 硬件说明](../../packages/robotiq_hardware/README.md)。

## 核对设备属性

更换 USB 接口或排查别名未生成时，可以先查看设备的 udev 属性，并核对 `idVendor`、`idProduct` 和 `serial`：

```bash
udevadm info --query=property --name=/dev/ttyACM0
udevadm info --query=property --name=/dev/ttyACM1
udevadm info --query=property --name=/dev/ttyUSB0
```

实际设备节点可能不是 `ttyACM0`、`ttyACM1` 或 `ttyUSB0`，应根据 `ls -l /dev/serial/by-id/` 的结果替换命令中的路径。

更换控制器、USB2CAN 或其他硬件后，必须核对新的 USB 属性，并同步更新规则中的序列号；不要仅依赖随插拔顺序变化的 `ttyACM`／`ttyUSB` 编号。

# 串口设备别名规则

本目录保存当前实验台的 udev 部署规则，为 PapillArray Controller 和达妙 USB2CAN 创建稳定的设备别名。规则绑定当前硬件的 USB 供应商标识、产品标识和序列号，属于本实验台的部署特定项，不是通用硬件配置。

## 当前别名

| 设备 | 别名 | 供应商标识 | 产品标识 | 序列号 |
| --- | --- | --- | --- | --- |
| PapillArray Controller | `/dev/papillarray` | `16c0` | `0483` | `15954100` |
| 达妙 USB2CAN | `/dev/dmj4310_can` | `2e88` | `4603` | `00000000050C` |

## 安装与验证

将规则文件复制到系统 udev 规则目录，然后重新加载规则并触发设备重新识别：

```bash
sudo cp deploy/udev/99-parallel-gripper-tactile.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

如果触发后别名没有出现，请拔出并重新插入两个设备。检查别名是否存在：

```bash
ls -l /dev/papillarray /dev/dmj4310_can
```

规则将设备权限设置为 `dialout` 组可读写（`0660`）。运行采集或电机程序的用户需要加入该组；加入后请重新登录，或重新启动当前会话使组权限生效：

```bash
sudo usermod -aG dialout "$USER"
```

## 核对设备属性

更换 USB 接口或排查别名未生成时，可以先查看设备的 udev 属性，并核对 `idVendor`、`idProduct` 和 `serial`：

```bash
udevadm info --query=property --name=/dev/ttyACM0
udevadm info --query=property --name=/dev/ttyACM1
```

实际设备节点可能不是 `ttyACM0` 或 `ttyACM1`，应根据 `ls -l /dev/serial/by-id/` 的结果替换命令中的路径。

更换控制器、USB2CAN 或其他硬件后，必须核对新的 USB 属性，并同步更新规则中的序列号；不要仅依赖随插拔顺序变化的 `ttyACM` 编号。

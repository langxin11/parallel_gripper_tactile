# Robotiq Hardware

Robotiq 2F85 的纯 Python 硬件适配边界。基础适配器把经过校验的整数位置命令交给后端；可选遥控入口管理 USB／RS485 会话和鼠标／手柄输入。不包含 ROS、MuJoCo、DM 电机或触觉采集依赖。

## USB／RS485 鼠标与手柄遥控

在仓库根目录运行。先用模拟模式核对鼠标和 TC-G50 手柄按钮，此模式不打开串口：

```bash
uv run --package robotiq-hardware --extra teleop robotiq-teleop --dry-run
```

将 TC-G50 通过 USB 或系统蓝牙配对接入电脑；系统识别为游戏手柄后，窗口显示实际按下的按钮编号。默认按钮 0 打开、按钮 1 闭合，
不同手柄模式的编号可能不同，不预设 A／B 标识与编号的对应关系。需要时指定
`--open-button 4 --close-button 5`，并先在模拟模式核对。连接多个手柄时用
`--joystick-index 0` 选择枚举索引。窗口未识别手柄时仍可使用鼠标。

夹爪按设备手册供电并接入 USB／RS485 后，先查询串口路径：

```bash
ls -l /dev/serial/by-id/
```

使用实际路径替换下列示例；推荐使用稳定的 `/dev/serial/by-id/...` 路径。
设备默认参数为 115200、8N1、地址 9，可用 `--baudrate` 和 `--device-id` 覆盖。

```bash
uv run --package robotiq-hardware --extra teleop robotiq-teleop --port /dev/ttyUSB0
```

串口权限不足时，检查设备所属组并为当前用户配置该组权限，重新登录后重试；不要用 root
启动整个控制窗口。窗口需要本地图形桌面，不能在无显示设备的 SSH 会话中直接显示。

1. 等待窗口显示 `connected`，此时仅连接和读取反馈，不自动激活或运动。
2. 清空夹爪运动范围，再点击“激活（夹爪将运动）”。驱动的激活过程可能完整开闭，
   期间不能使用鼠标／手柄打断自检；此前已激活的夹爪不会被强制重置。
3. 显示 `ready` 后先释放输入。短按“打开”或“闭合”走一步，长按连续运动，松开停止连续运动；
   鼠标与手柄采用相同规则，短按已登记的一步可完成。
4. STOP、空格、输入冲突、窗口失焦或手柄断开会停止；恢复控制前先释放全部输入。
   退出时尝试停止并关闭串口。故障后不自动重连或恢复运动，排查后重新启动窗口。

面板顶部显示模拟／真机状态和手柄名称，中间显示当前位置与目标位置。
进度条表示 0～255 位置编码，目标标记表示已提交的目标；未知值显示为破折号，均不代表毫米。
激活完成后按钮收起，停止按钮始终保留；故障详情在独立错误区显示。

### 短按单步、长按连续

只有一种操作模式，无需选择或追加启动参数。按下打开先将目标减 1，按下闭合先将目标加 1；
持续按住达到 0.4 s 后，发送打开终点 0 或闭合终点 255，按配置速度连续运动。
**松开会停止连续运动；短按只让已登记的一小步完成。**

连续短按以成功提交的目标累加，始终限制在 0～255 内；初次操作以及连续运动停止后的
下一次按下，都从实时位置反馈建立单步基准。这表示命令分辨率，不保证真机每次产生准确、
可测的机械位移。长按从单步升级为终点运动后，不再重复发送终点命令。

STOP、空格、失焦、输入冲突、手柄断开、心跳超时和退出会取消未发单步并尝试停止，
恢复后重新从反馈建立基准。串口忙时仅保留最新按下沿，不积压点击队列；不要用于批量计步。
旧命令中的 `--motion-mode jog` 或 `--motion-mode continuous` 仍可解析并显示迁移提示，
两者均执行当前统一行为，不再切换模式。

默认 `--speed 30 --force 30`，两者均是 0～255 的寄存器值，**不是 mm/s 和 N**；
低寄存器值不代表零夹持力。显示的位置为 0～255 编码，不是毫米。
当前只支持开闭位置遥控，没有触觉采集、力闭环或毫米标定。

会话显式 `start()` 后由唯一后台线程访问设备；窗口每帧提交输入心跳，超过 250 ms
没有新输入时尝试停止并锁定到输入释放。串口请求等待设置为 150 ms 且不重试；
这些是软件参数，不是已验证的物理停止延迟。通信断开、驱动阻塞或进程被强制终止时，
软件不能保证停机，需保留可直接断开设备电源的方式。

该入口使用固定版本驱动的原子 `move(..., start=True)` 恢复运动，避免停止后先运行旧目标；
原有 `PyRobotiqGripper3312Backend` 的 `start=False` 契约保持不变。
依赖接口核对自 [3.3.12 连接说明](https://pyrobotiqgripper.readthedocs.io/en/3.3.12/connection/)
与 [API 文档](https://pyrobotiqgripper.readthedocs.io/en/3.3.12/api/robotiqgripper/)，
并通过固定版本驱动加假串口、假设备与窗口事件验证；**尚未完成真机及 TC-G50 实物验证**。

## 基础适配器边界

`Robotiq2F85Hardware` 要求调用方注入一个 `RobotiqPositionBackend`。后端是本包定义的最小协议：

```python
from robotiq_hardware import Robotiq2F85Hardware

gripper = Robotiq2F85Hardware(backend)
receipt = gripper.move(128)  # 只接受整数 0–255
assert receipt.requested_position == 128
assert gripper.read_position() == 128
gripper.open()               # 发送 0
gripper.close()              # 发送 255
```

位置命令越界会被拒绝，不会静默裁剪；`True` 和 `False` 也不会被当作整数命令接受。

## pyrobotiqgripper 3.3.12 集成

可选依赖精确固定为 `pyrobotiqgripper==3.3.12`。版本化文档：[pyrobotiqgripper 3.3.12 API](https://pypi.org/project/pyrobotiqgripper/3.3.12/)。版本化 API 证据确认已连接、已激活对象提供无参数 `position()` 位置读取和 `move(...)` 位置命令；`PyRobotiqGripper3312Backend` 只使用这两个接口：

```python
from robotiq_hardware import PyRobotiqGripper3312Backend, Robotiq2F85Hardware

# gripper 必须由调用方完成连接和激活；本包不会自动执行这些步骤。
backend = PyRobotiqGripper3312Backend(gripper, speed=255, force=255)
hardware = Robotiq2F85Hardware(backend)
hardware.move(128)
hardware.read_position()
```

每次调用底层 `move` 都显式传入 `wait=False`，保证位置命令不等待完整运动结束。`readStatus=False` 是显式默认值，用于避免控制周期附加状态读取的阻塞边界；需要读取状态时可构造 `PyRobotiqGripper3312Backend(..., read_status=True)`。`refreshStatus=False` 和 `start=False` 也由适配器显式传入。

`read_position()` 会严格拒绝布尔值、非整数和超出 0–255 的反馈；底层 `position()` 异常会原样透传。`move()` 成功后返回只包含 `requested_position` 的不可变命令收据；读反馈与发命令是两个独立接口。

该后端尚未经过真实 Robotiq 设备验证。连接、激活、串口参数和设备生命周期全部由调用方负责；适配器不会自动 connect、activate、start 或 stop。测试只使用 fake gripper，不会发送真实设备命令。

`run_discrete_control_step()` 把 `robotiq_grasp_core` 的一次 `observe`／`decide` 与绝对位置命令
衔接起来。零增量不发送命令；非零增量只有在后端成功返回后才调用 `action_applied`。发送失败时
取消核心中的 pending 动作并原样抛出异常。该单步不会隐式读取位置反馈，反馈采样由调用方独立调度。

测试使用 fake backend，不会发送真实设备命令。

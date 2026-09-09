# Robotiq Hardware

Robotiq 2F85 的纯 Python 硬件适配边界。这个包只负责把经过校验的整数位置命令交给后端，不包含 ROS、MuJoCo、DM 电机或触觉采集依赖。

## 当前边界

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

测试使用 fake backend，不会发送真实设备命令。

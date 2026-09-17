# dmgripper-hardware 0.1.0

DM4310P 与 USB2CAN 的纯 Python 硬件基础包。当前版本包含可离线验证的协议编码、反馈解码、
分段接收帧重组、内存 `FakeTransport`、延迟打开的 `PySerialTransport`、只读
`DmStateRefresher`、可复用 `DmSession`，默认 dry-run 的受限小步运动探针，以及可选的独立摇杆遥控入口；没有 CAN、ROS、
MuJoCo、触觉采集或 Robotiq 依赖。

## 边界与安全性

- `Usb2CanProtocol` 只返回或解释 `bytes`，不会打开端口，也不会向设备发送命令。
- `PySerialTransport` 在构造时不会导入 PySerial 或打开端口；只有调用方显式 `open` 才会
  创建串口。其工厂可注入，测试只使用内存 fake，不访问 `/dev`。
- `Usb2CanDeviceConfig` 严格校验端点、波特率、不同且非零的电机／主机 CAN ID，以及有限正的
  刷新超时。默认波特率为 `921600`。
- `MotorLimits` 没有通用默认值。当前夹爪的 `make_dm4310p_gripper_config()` 固化用户提供的、
  无 I/O 的部署配置：电机协议量程为位置 `[-1.7, 1.7] rad`、速度 `[-8, 8] rad/s`、
  力矩 `[-4, 4] N·m`；目标命令工作范围为 `[0, pi/2] rad`，角度增大方向为闭合方向。
- 电机协议量程、命令工作范围和反馈安全范围是三个不同边界。`validate_joint_position()` 对工作范围外
  的目标命令直接抛出 `ValueError`；`validate_feedback_position()` 只允许反馈在工作范围两端各扩展
  `feedback_position_margin_rad`。默认余量为 `0.05 rad`，即反馈范围
  `[-0.05, pi/2+0.05] rad`，但目标命令范围仍是 `[0, pi/2] rad`。构造配置时会检查两个范围都位于
  协议位置量程内。
- `DmStateRefresher.refresh_once()` 唯一允许写入 `make_feedback_request()` 创建的状态刷新帧；
  它会按总超时读取、分帧、忽略噪声与无关帧并返回目标反馈。它不实现使能、失能、置零、控制
  报文或寄存器读写；上层必须单独实现互锁、反馈新鲜度与急停策略。
- 未实现 USB2CANFD，也不对其帧格式或接口做任何假设。

## 可复用 DM 会话

`DmSession` 由单个调用线程独占，提供显式 `open`、`inspect`、`require_disabled`、`enable`、
`command`、`hold`、`disable` 和 `close`。构造只组装协议与传输对象，不执行设备 I/O；每次命令都要求
完整写入，并用该命令产生的新反馈检查电机故障、使能状态和扩展反馈安全范围。

`command` 只接受 `[0, pi/2] rad` 工作范围内的目标。`hold` 要求已经存在最新有效的使能反馈，目标
位置必须等于该反馈在工作范围内的投影（允许一个协议量化步误差），并强制目标速度和前馈力矩均为
零，再复用普通命令与反馈检查。会话会在编码前拒绝超出协议速度、力矩、`kp` 或 `kd` 范围的请求，
避免协议静默饱和改变上层已经验证的合成力矩。`disable` 只有在新反馈明确报告 `status_code=0` 时
才算确认失能；直接接收失能命令自身的回复，不追加状态查询，避免重复启停时反馈错位。

会话不决定何时自动回零、何种实验故障可以保持、何时接受人工 `release`，也不创建运行目录或
manifest；这些策略由上层入口决定：抓取实验属于 `dmgripper_experiments`，独立手柄遥控属于
`dmgripper-teleop`。默认反馈安全余量尚未经当前真机验收，软件范围检查
和位置保持也不能替代硬件急停、机械限位或物体承接措施。

## 独立手柄遥控

`dmgripper-teleop` 单独控制连续关节角，不需要 PapillArray、抓取实验或 Robotiq。
TC-G50 可用蓝牙或 USB 接入；先由系统识别为游戏手柄。默认运行完全离线的交互模拟，
**不打开串口**，与下文会读取真实设备的 motion-probe 默认 dry-run 不同：

```sh
uv run --package dmgripper-hardware --extra teleop dmgripper-teleop
```

默认读取第一个手柄的纵轴 `--axis 1`：上推打开、下推闭合；偏转越大，目标角推进越快。
蓝牙和 USB 模式的轴编号可能不同，先在模拟窗口核对轴值和方向；可用 `--axis`、
`--joystick-index` 和 `--invert-axis` 调整。窗口未识别手柄时不会接受使能。

面板分区显示电机反馈和最近 MIT 命令：反馈包括关节位置 `q`（rad）、速度 `v`（rad/s）、
力矩 `tau`（N·m）；命令包括 `q_des`、`v_des`、`tau_ff`、`kp`（N·m/rad）和
`kd`（N·m·s/rad）。反馈量直接取电机回传，不用目标值或前馈值代替。
命令区显示最近一次发送并收到有效反馈的命令（协议量化前），尚未发送或已确认失能时显示 `—`。
发生故障时数值可能停留在最后一次反馈，应结合设备状态判断。离线模拟的速度、力矩反馈为零，
不代表真实电机响应。
三张反馈卡片与命令区采用大号数字，顶部输入条显示摇杆方向和幅度；使能按钮会直接提示
“先连接手柄”或“摇杆回中后使能”等当前条件。

真机启动使用显式执行参数，端口默认 `/dev/dmj4310_can`：

```sh
uv run --package dmgripper-hardware --extra teleop dmgripper-teleop --execute
```

启动后检查 MIT 模式，并要求电机已经失能；不会自动切换模式、回零、置零或接管已使能设备。
摇杆回中、窗口有焦点后点击“使能”，初始目标取当前反馈角在命令行程内的投影。摇杆控制的是目标角变化速度，
目标角积分后交给 MIT 阻抗控制；实际角度由反馈显示，不能把目标速度当作实际机械速度保证。
回中（进入摇杆死区）时暂停目标更新，保留最后的目标角，不重设为当前反馈角；再次同向推动时继续推进，
反向推动则先进入下面的释放过渡。
保持期间持续发送同一角目标，阻抗保持允许受力后存在角度误差，也不保证恒定夹持力。
STOP、空格、失焦、手柄断连或输入超时会失能，**失能不等于位置保持，负载可能释放**；
恢复后先回中并重新手动使能。通信／电机故障后会退出设备会话，需排查后重启。

默认参数为 `--max-speed 0.2` rad/s、`--deadzone 0.12`、`--mit-kp 2.0`、
`--mit-kd 0.5`。死区外线性映射偏转量；角目标始终限制在 `[0, pi/2]` rad 内，增大为闭合。
默认不限制目标与反馈的角度偏差：实际关节受阻时，推动摇杆仍继续改变目标角，
通过 `kp * (q_des - q)` 累积位置项力矩；回中保留该目标，反向推动会平滑释放原方向的偏差。
如需主动限制目标领先量，可显式指定 `--max-lead 0.05`；省略该参数即关闭领先限制。
命令采用零目标速度，靠角目标斜坡推进。增益默认值沿用受限运动探针，摇杆速度、死区
与可选领先限制参数尚未经当前真机验收。

`--feedforward-force` 设置平均单侧等效前馈目标力大小，单位 N，默认 `1`，开闭默认共用此大小。
`--opening-feedforward-force` 可单独覆盖张开等效目标力，设为 `0` 只关闭张开前馈；
未单独覆盖时，`--feedforward-force 0` 同时关闭两个方向的前馈。
使用与项目真机实验相同的曲柄滑块几何（初始角 `pi/4`、曲柄半径 `0.03 m`、连杆长度 `0.04 m`、
偏置 `0.021213203435596423 m`），每周期按当前反馈角计算总闭合行程雅可比 `J_c(q)`，
闭合前馈为 `tau_ff = +J_c(q) * F_close`，张开为 `tau_ff = -J_c(q) * F_open`。
雅可比已经包含两指运动，平均单侧力不再乘 2；例如 `q=pi/4` 时 `J_c=0.06 m/rad`，
每侧等效 `1 N` 对应闭合 `+0.06 N·m`、张开 `-0.06 N·m`。张开参数描述开向等效助力，
不表示实际存在单侧拉力或测得了接触力。
参数接受非负有限数值，符号由开闭状态决定；`--invert-axis` 只改变操作映射，张开仍为负、闭合仍为正。
换算力矩按部署协议 `[-4, 4] N·m` 限幅；该范围不是推荐工作力矩或总输出力矩上限。
面板显示目标力和最近已确认命令中的前馈力矩，不代表测得的实际力或实际力矩。
前馈在首次使能时为零，开闭两方向均从零渐入；非零比例下仍按反馈角持续计算，
包括摇杆回中及达到目标角限位后。
实际力矩还叠加位置误差和速度阻尼项，因此回中后仍可能运动，也不保证恒定夹持力。
STOP／失焦／断连仍请求失能。离线模拟只验证命令与交互，不模拟前馈力矩的物理效果。

方向切换采用“释放旧偏差，再建立新方向”的状态机，方向记忆跨越摇杆回中：

| 操作 | 目标角与前馈 |
| --- | --- |
| 首次闭合／持续闭合 | 目标角按摇杆积分，闭合前馈比例用五次曲线渐入至 1；同向可继续累积位置误差。 |
| 首次张开／持续张开 | 目标角按摇杆减小，张开前馈比例用五次曲线渐入至 -1。 |
| 张开与闭合之间反转 | 入口保持原目标及原方向前馈；随后把目标平滑混合到最新反馈角，原方向前馈归零；释放结束后再沿新方向积分，从零渐入新方向前馈，不直接翻转力矩符号。 |
| 回中保持 | 冻结目标和当前前馈比例，暂停过渡计时；不会把尚未完成的前馈自动加到满值或继续卸力。 |

`--switch-duration 0.25` 指定每次释放与新方向前馈渐入各自的时长（秒）。释放阶段回中后恢复时，
从保持的输出重新开始完整平滑区间，避免暂停期间反馈移动造成目标跳变；前馈渐入则从暂停进度继续。
切换中再次反向也从当时输出重新释放，不恢复更早的目标。界面会显示“正在释放原方向力矩”。
`--max-speed` 只限制常规摇杆积分速度，释放目标按过渡时长收敛，较大偏差下可能比常规积分更快；
这些是命令过渡时长，不保证实际电机在相同时间内完成卸力或反转。

设备会话由唯一后台线程拥有。运行中每条控制命令消费其新反馈，不附加轮询；
名义周期 10 ms，单次串口等待 50 ms，周期超过 100 ms 则故障退出，输入超过 250 ms
未刷新则失能。上述时间是软件阈值，不构成硬实时或物理停机时间保证。窗口退出时尝试失能并关闭串口，
失败会明确报告“失能未确认”。首次使用仍需无负载、机械行程可用并保留外部急停。

当前通过假会话、角目标测试和窗口事件验证；模拟会话直接跟随目标，不模拟动力学或接触力，
尚未完成真机手柄联调。

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
uv run --package dmgripper-hardware dmgripper-motion-probe
```

`dmgripper-motion-probe` 与 `dmgripper-state-probe` 默认使用 udev 别名
`/dev/dmj4310_can`；可通过 `--port` 显式覆盖。

`--execute` 会拒绝非 MIT 模式（寄存器 `10` 必须为 `1`），并且使能后必须收到新的
`status_code=1` 反馈；不会隐式切换控制模式。每条使能、失能或 MIT 命令后只等待并解析该命令产生的
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
  --mit-kp 5.0 --closing-step 0.06 --stage-duration 1.0 \
  --execute
```

执行顺序严格为：使能并以初始反馈位置保持，按配置步长设置闭合阻抗平衡点，再将阻抗平衡点恢复为初始位置，最后
失能。MIT 命令的默认值参考 `dm_force_tracking.yaml` 中的 `mit_kp=2.0`、`mit_kd=0.5`；
`kp` 可由命令行显式调整，不添加前馈力矩。
正常结束时失能帧短写会使命令以非零状态失败；通信超时、故障、机械范围越界和 Ctrl-C
均会尽力发送已验证的 `CMD_DISABLE`，随后在限定超时内接收该命令自身的反馈，确认 `status_code=0`，
最后关闭串口。

该探针不是急停、功能安全互锁或硬实时控制器：保护反应仍受 USB、串口、USB2CAN 和固件时延限制。
现有协议来源只验证了 `CMD_ENABLE`、`CMD_DISABLE` 和 MIT 帧，没有独立、可证明的停止帧，因此本包
不会臆造停止 CAN 报文或执行置零。

## 只读状态探针

从仓库根目录可直接运行有限次数的 DM4310P 状态探针：

```sh
uv run --package dmgripper-hardware dmgripper-state-probe \
  --count 10 --interval 0.1 --timeout 0.05
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

deployment = make_dm4310p_gripper_config("/dev/dmj4310_can")
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

config = Usb2CanDeviceConfig("/dev/dmj4310_can", motor_id=1, master_id=17, timeout_s=0.05)
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
`[-4, 4] N·m`。这些协议值已经过离线向量核对；新增的 `0.03 rad` home 容差和 `0.05 rad` 反馈
安全余量仍只是实验层的保守起点，尚未在当前真机上验收。

## 未验证的实机假设

当前 DM4310P 与 USB2CAN 的只读状态刷新已完成真机通信核验；受限运动探针、自动回零、故障保持、
home 容差和扩展反馈安全范围尚未完成真机验收。
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

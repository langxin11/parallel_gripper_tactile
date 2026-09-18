# dm-grasp-core 0.1.0

DMgripper 的共享 Python 控制核，要求 Python >=3.12；运行依赖仅限
numpy 与 simple-pid 两个纯计算库，不包含 ROS、MuJoCo、串口或模型路径。
安装本目录后，两端使用 `from dm_grasp_core import ...`；版本为 `__version__`。

## 子域边界

- `control/`：曲柄滑块运动学、二阶导纳状态、MIT 合成力矩约束、达妙协议
  量化与法向力跟踪（PID、一阶 LADRC、直接力矩、二阶力矩 LADRC 外环）。
- `grasp/`：minimum-jerk 接近／接触过渡，以及导纳状态到 MIT 命令的映射。
- `tactile/`：仅提供双侧法向力零窗口和稳定接触判定；不包含传感器解析、滤波、标定或设备 I/O。

顶层 `dm_grasp_core` 继续导出全部既有公共对象。兼容期内，
`dm_grasp_core.control` 和 `dm_grasp_core.command` 仍可导入，且导出与新子域完全相同的对象。

## 控制约定

- 力反馈为左右法向力的平均值（N），不是两侧之和。
- 导纳位移是总闭合行程（m）；角位置 rad，角速度 rad/s，力矩 N·m。
- `SecondOrderAdmittance` 使用半隐式 Euler，`step_admittance` 原地更新其状态。
- `MITCommandConfig` 依次包含位置上下限、速度上限、闭合方向（±1）、kp、kd、前馈比例、前馈力矩上限、合成力矩上限。
- `build_mit_command` 使用目标构型雅可比映射速度、实测构型雅可比映射力前馈。先限制合成力矩，再限制机械角；约束无法同时满足时抛出 `ValueError`。
- `step_admittance` 顺序为平均力误差积分、按参考闭合行程与机械范围限制状态、按实测雅可比限制速度、构建 MIT 命令。

就导纳命令路径而言，调用者保留设备参数验证、输入新鲜度和有效性判断、目标力上限、控制时间步裁剪、接触过渡状态机、失接触策略、使能/失能与设备通信；这里没有新增这些策略，出错时导纳状态可能已被更新，与迁移前节点相同。接触确认器亦保留原输入约定，不应直接接收未经检查的 NaN 传感器读数。`control/` 下的法向力跟踪模块自带接近/跟踪/释放状态机与 MIT 协议量化，但其设备参数验证、命令下发与安全策略同样由 `MITTorqueInner` 的实现方保留。

## 安装与测试

```sh
python -m pip install ./packages/dm_grasp_core
PYTHONPATH=packages/dm_grasp_core/src python -m pytest packages/dm_grasp_core/tests
```

测试使用独立数学期望覆盖正反闭合方向、规则与不规则时间步、限幅、复位和无效输入。
测试不导入 ROS，也不依赖仓库外的消息包或节点实现。

## 来源与许可

Apache-2.0。原 `dm_gripper_control/package.xml` 声明 Apache-2.0；本目录 `LICENSE` 原样复制自承载仓库 `parallel_gripper_tactile/LICENSE`，保留其中 Copyright 2026 langxin11。ROS 控制包没有独立 LICENSE 文件。

原 `control.py` 的八项公共算法来自 `tactile_grasp_ros2/src/dm_gripper_control/dm_gripper_control/dm_gripper_control/control.py`，仅移除机械端点搜索类并调整模块说明与导入；现按职责置于 `control/`、`grasp/` 与 `tactile/`。原 `command.py` 从 `force_tracking_node.py` 的 `_build_command` 和 FORCE_TRACKING 积分段抽取，现实现置于 `grasp/command.py`，改用显式参数和 dataclass 返回值，保留数学顺序和限幅行为。没有从不再维护的实验或传感器原型仓库复制实现。

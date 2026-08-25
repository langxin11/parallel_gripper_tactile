# 项目架构

本项目把 MuJoCo 资产、夹爪差异和实验流程分开管理，使 Robotiq 2F-85 参考模型与
自研曲柄滑块平行夹爪能在同一套验证与记录约定下工作。

```text
MJCF / STL 资产
        ↓
TOML profile（模型、控制、安装、触觉布局）
        ↓
Python 核心库（profile、控制、触觉读取、验证、实验协议）
        ↓
场景与实验脚本（查看、抓取、扰动、记录、绘图）
```

## 资产与 profile

`configs/` 中的 profile 是脚本选择夹爪的入口：

| Profile | 控制方式 | 触觉后端 | 主要资产 |
| --- | --- | --- | --- |
| `robotiq_2f85.toml` | MuJoCo 位置执行器 | 3×3 `force_sensor` | `2f85_taxels.xml` |
| `custom_parallel_gripper.toml` | Python MIT 力矩内环 | 3×3 `contact_geom` | `parallel_gripper_prepared.xml` |

每个 profile 都声明模型路径、执行器名称、开闭控制端点、安装位姿和左右触觉阵列的
命名规则。自研夹爪还声明 `control.mit` 的位置、速度与力矩限制，以及 `control.force`
的法向力外环参数。

`pgt-check` 会编译 profile 引用的 MJCF，并检查执行器、控制范围和全部触觉通道。新资产
或 profile 改动应先通过该检查，而不是等到长时间仿真才发现名称或坐标错误。

## 核心库

| 模块 | 职责 |
| --- | --- |
| `profiles.py` | 读取并校验 TOML profile，解析相对模型路径。 |
| `validation.py` | 编译 MJCF，验证执行器与触觉布局的契约。 |
| `contact_taxels.py` | 按命名的 Pillar 碰撞 geom 聚合接触力，输出左右 3×3 力阵列。 |
| `control.py` | MIT 力矩内环和基于 `simple-pid` 的法向力外环。 |
| `protocols.py` | 共享闭合、撤支撑、切向扰动与恢复的时序。 |
| `video.py` | MuJoCo 离屏渲染与视频编码工具。 |

`MITTorqueController` 将位置、速度和前馈力矩转换为受限的 motor 力矩；
`NormalForceController` 先以位置接近，确认左右指尖连续接触后，再以总法向力跟踪
profile 中的目标值。它们只用于声明 `mit_torque` 的 profile。

## 场景与实验

Robotiq 的场景组合位于 `scripts/grasp_scene.py`，自研夹爪的固定安装抓取场景位于
`scripts/custom_grasp_scene.py`。后者会把 `base` 固定在 profile 的 mount frame；这只是在
运行时的安装方式，不会修改 CAD 或基础 MJCF。

所有带撤支撑和扰动的实验复用 `DisturbanceProtocol`：闭合、带支撑稳定、无支撑保持、
切向正弦扰动和恢复。扰动力通过 `xfrc_applied` 施加到方块质心；默认方向为世界坐标系
Y 轴，滑移在世界 YZ 接触平面内度量。

## 资产维护边界

- Robotiq 的离散 taxel 与 `touch_grid` 派生 XML 由生成脚本维护。
- 自研夹爪的 `parallel_gripper_prepared.xml` 是 profile 所引用的可运行资产；重新导出后
  必须先经过 `prepare_onshape_export.py` 和 `verify_mujoco.py`。
- 实验初始条件、验收阈值和控制时序属于脚本或 profile，不应直接写进基础夹爪资产。

更多触觉力的坐标系和符号见[触觉读数约定](tactile-conventions.md)，可执行命令见
[常用工作流](workflows.md)。

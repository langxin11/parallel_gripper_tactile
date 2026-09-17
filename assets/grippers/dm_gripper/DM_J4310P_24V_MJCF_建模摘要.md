<!-- --8<-- [start:content] -->
# DM-J4310P-2EC（24 V）在 MuJoCo / MJCF 中的抽象建模

> **资料范围：**本页保留建模时采用的厂家参数、实机截图记录和输出轴换算假设，
> 不是对当前在线设备的实时读取。数值来源见文末“资料依据”；其中截图未附采集日期与完整原始记录。
> 实验采用的配置以所选 profile、Hydra 组合与实际加载的 MJCF 为准。
> 本页维护电机模型，机构与外环控制的权威说明分别见[共享控制核][shared-control]和[力跟踪][force-tracking]。

## 1. 建模目标

面向“ADRC + 曲柄滑块夹爪 + 指尖触觉/接触力”的仿真，当前将 DM-J4310P-2EC 抽象为：

> **减速器输出轴上的受限力矩执行器**

不展开 FOC、电流环、三相电磁过程和热模型。实机 MIT 模式可用：

```text
kp = 0, kd = 0, t_ff = τ_cmd
```

因此控制接口可对应为：

```text
实机：ADRC → τ_cmd → MIT t_ff → 内部电流环 → 输出轴
仿真：ADRC → τ_cmd → MuJoCo <motor> → 输出轴 joint
```

---

## 2. 24 V 型号厂家参数

| 参数 | 数值 |
|---|---:|
| 额定电压 | 24 V |
| 额定输出转矩 | 3.5 N·m |
| 峰值输出转矩 | 12.5 N·m |
| 额定输出转速 | 120 rpm ≈ 12.566 rad/s |
| 空载最大输出转速 | 200 rpm ≈ 20.944 rad/s |
| 减速比 | 10:1 |
| 极对数 | 14 |
| 整机质量 | 约 0.325 kg |

---

## 3. 当前实机读取参数

### 电机辨识参数

| 参数 | 实机值 |
|---|---:|
| 相电阻 Rs | 640.5491 mΩ |
| 相电感 Ls | 222.3364 μH |
| 磁链 ψf | 0.004362747 Wb |
| 粘滞系数 | 0.0002532329 |
| 转动惯量 | 2.074755×10⁻⁵ kg·m² |

### 当前驱动配置

| 参数 | 当前值 |
|---|---:|
| VBUS | 24.0170 V |
| Imax | 20.522388 A |
| 控制模式 | MIT Mode |
| PMAX | 1.7 rad |
| VMAX | 8 rad/s |
| TMAX | 4 N·m |
| 减速比 | 10 |
| 齿轮系数 | 1 |
| CAN | 1 Mbps |

说明：

- `TMAX=4` 是当前 MIT 指令映射范围，不是物理峰值；
- `VMAX=8` 是当前通信映射范围，已按夹爪实验需要低于 24 V 电机实际最高转速；
- `PMAX`、`VMAX`、`TMAX` 都是驱动器 MIT 协议的可配置映射/命令范围，可以按实验需要
  人为收紧，但不得超过电机、减速器和夹爪机构的实际极限；
- 物理峰值仍按 12.5 N·m；
- 24 V 空载最大输出转速仍按约 20.944 rad/s。

本仓库采用更保守的夹爪侧配置：

```text
P_MIN = 0 rad
P_MAX = 1.7 rad            # 当前达妙 PMAX，与夹爪可用开度范围匹配
V_MAX = 8 rad/s            # 当前达妙 VMAX
T_MAX = 4 N·m              # 当前达妙 TMAX，略高于额定 3.5 N·m
```

其中 `P_MAX` 不是电机可累计旋转角度，而是本夹爪曲柄关节允许的控制目标上限。

---

## 4. MJCF 参数换算

界面把 `Inertia`、`Damp` 列为“电机参数”，减速比单独列出。
**当前暂按它们为转子侧辨识参数处理。** 说明书没有明确写“转子侧”，因此这是建模假设。

理想减速器下：

$$
J_out = N^2 J_m
$$

$$
B_out = N^2 B_m
$$

取 `N=10`：

```text
J_out = 0.002074755 kg·m²
B_out = 0.02532329 N·m·s/rad
```

因此当前建议：

```text
armature = 0.002074755
damping  = 0.02532329
```

`frictionloss` 暂无实机辨识值，可先用经验初值：

```text
frictionloss = 0.04 N·m
```

---

## 5. 推荐 MJCF

```xml
<mujoco model="gripper_dm4310p">

  <default>
    <default class="dm4310p_24v">
      <joint
        armature="0.002074755"
        damping="0.02532329"
        frictionloss="0.04"/>
    </default>
  </default>

  <worldbody>
    <body name="gripper_base">

      <!-- 电机固定部分质量主要并入 gripper_base -->

      <body name="crank">
        <joint
          name="dm4310_output_joint"
          type="hinge"
          axis="0 0 1"
          class="dm4310p_24v"/>
      </body>

    </body>
  </worldbody>

  <actuator>
    <motor
      name="dm4310_torque"
      joint="dm4310_output_joint"
      gear="1"
      ctrllimited="true"
      ctrlrange="-4 4"
      forcelimited="true"
      forcerange="-4 4"/>
  </actuator>

</mujoco>
```

仓库中的 `parallel_gripper.xml` 与 `parallel_gripper_prepared.xml` 已采用这一结构。
`ctrlrange` 与 `forcerange` 均对齐当前实机 MIT 的 `TMAX=4 N·m`，厂家 12.5 N·m 峰值仅作为电机能力参考。

### 参数含义

- `gear="1"`：joint 已定义在减速器输出轴；厂家转矩/转速也是减速后数据，不再重复乘 10。
- `ctrlrange="-4 4"`：匹配当前实机 `TMAX=4`。
- `forcerange="-4 4"`：让 MuJoCo 执行器最终输出也不超过当前 MIT 力矩范围。
- `armature`：当前按转子惯量经 10:1 减速器反射到输出轴。
- `damping`：当前按转子侧粘滞系数经 10:1 减速器反射到输出轴。
- `frictionloss=0.04`：经验初值，后续用低速/静摩擦实验替换。

---

## 6. 当前建议参数

| MJCF 参数 | 当前值 |
|---|---:|
| actuator | `<motor>` |
| gear | 1 |
| ctrlrange | ±4 N·m |
| forcerange | ±4 N·m |
| armature | 0.002074755 kg·m² |
| damping | 0.02532329 N·m·s/rad |
| frictionloss | 0.04 N·m（经验值） |
| 最大输出转速参考 | 20.944 rad/s |
| 连续输出转矩参考 | 3.5 N·m |

### Python MIT 控制回路

MJCF `<motor>` 的 `ctrl` 单位是输出轴力矩。位置和速度闭环由 Python 显式计算：

$$
\tau_{cmd} = \operatorname{clip}\left(
K_p(p_{des}-p) + K_d(v_{des}-v) + t_{ff},
-T_{MAX}, T_{MAX}
\right)
$$

控制器还会把 `p_des` 限制到 `[P_MIN, P_MAX]`，把 `v_des` 限制到
`[-V_MAX, V_MAX]`，并把 `t_ff` 单独限制到 `±T_MAX`。当前默认值为：

| MIT 参数 | 当前值 |
|---|---:|
| P_MIN / P_MAX | 0 / 1.7 rad |
| V_MAX | 8 rad/s |
| T_MAX | 4 N·m |
| Kp | 20 N·m/rad |
| Kd | 0.63793536 N·m·s/rad |
| t_ff | 0 N·m |

### 达妙 MIT 协议量化

达妙电机在 CAN/串口数据帧中不会直接传输浮点物理量，而是先映射到固定 bit 数的无符号整数：

$$
u=\frac{x-x_{\min}}{x_{\max}-x_{\min}}(2^N-1)
$$

仿真控制器会先按该公式量化，再解码回物理量参与 MIT 力矩计算，从而保留实机协议的分辨率影响。
当前配置下的分辨率为：

| 物理量 | 编码范围 | bit 数 | 分辨率 |
|---|---:|---:|---:|
| Position | 0..1.7 rad | 16 | 2.594e-5 rad |
| Velocity | -8..8 rad/s | 12 | 3.907e-3 rad/s |
| Torque / t_ff | -4..4 N·m | 12 | 1.954e-3 N·m |
| Stiffness / Kp | 0..500 N·m/rad | 12 | 0.1221 N·m/rad |
| Damping / Kd | 0..5 N·m·s/rad | 12 | 0.001221 N·m·s/rad |

其中 Position 使用 profile 的 `[P_MIN, P_MAX]`；Velocity 和 Torque 分别使用
`[-V_MAX, V_MAX]` 与 `[-T_MAX, T_MAX]`。Stiffness 和 Damping 使用达妙协议固定范围，
因此 profile 中 `kp` 必须在 `0..500`，`kd` 必须在 `0..5`。

`configs/dm_gripper.yaml` 保留基础 profile；科研运行还会按所选 Hydra 配置组合控制器参数。
MIT 算法位于 `packages/dm_grasp_core/src/dm_grasp_core/control/mit.py`，
`src/parallel_gripper_tactile/control.py` 负责配置与 MuJoCo 适配。
命令范围由配置给出，但 `T_MAX` 必须同时不超过 `<motor ctrlrange>` 和 `<motor forcerange>`；
提高命令范围时需要核对模型限幅。

### 预接触与目标法向力跟踪

DM 力控制器在 MIT 内环之外组织接近、接触过渡和力跟踪。左右指尖的有效法向合力分别记为
`F_L`、`F_R`，控制主量为**平均单侧力**：

$$
f_n = \frac{F_L+F_R}{2}
$$

目标力、跟踪误差和在线等效刚度均采用这一语义；双侧总力 `F_L+F_R` 只作为派生量。
测量坐标系、符号和聚合规则见[触觉接口约定][tactile-contract]。

具体 PID、刚度辅助反馈、力矩前馈与 ADRC 的公式由[力跟踪控制律][force-control-laws]统一维护；
关节角、开度和闭合雅可比由[共享控制核][shared-control]维护。
接触确认、丢失接触处理、目标曲线与滤波参数以所选控制器／任务配置为准。
本资产摘要不再维护另一套外环公式、默认目标或调参值。

---

## 7. 后续实验修正项

1. 确认 `Inertia`、`Damp` 是否明确为转子侧参数；
2. 辨识低速库仑摩擦和静摩擦；
3. 对齐实机输出轴零位与仿真 `qpos=0`；
4. 在夹爪负载下重新评估等效阻尼与摩擦；
5. 必要时再加入电流、电压和热限制。

---

## 资料依据

1. 《DM-J4310P-2EC 减速电机使用说明书 V1.1》，达妙科技，2025-11-21。
2. 当前实机串口助手截图：24 V 供电、MIT 模式、电机辨识参数及驱动配置。

<!-- --8<-- [end:content] -->

[shared-control]: ../../../docs/dm-shared-control.md
[force-tracking]: ../../../docs/force-tracking.md
[tactile-contract]: ../../../docs/tactile-conventions.md
[force-control-laws]: ../../../docs/force-tracking.md#force-control-laws
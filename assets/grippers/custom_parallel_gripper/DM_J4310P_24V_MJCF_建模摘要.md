# DM-J4310P-2EC（24 V）在 MuJoCo / MJCF 中的抽象建模

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
| PMAX | 12.5 rad |
| VMAX | 30 rad/s |
| TMAX | 10 N·m |
| 减速比 | 10 |
| 齿轮系数 | 1 |
| CAN | 1 Mbps |

说明：

- `TMAX=10` 是当前 MIT 指令映射范围，不是物理峰值；
- `VMAX=30` 是通信映射范围，不代表 24 V 电机实际最高转速；
- `PMAX`、`VMAX`、`TMAX` 都是驱动器 MIT 协议的可配置映射/命令范围，可以按实验需要
  人为收紧，但不得超过电机、减速器和夹爪机构的实际极限；
- 物理峰值仍按 12.5 N·m；
- 24 V 空载最大输出转速仍按约 20.944 rad/s。

本仓库采用更保守的夹爪侧配置：

```text
P_MIN = 0 rad
P_MAX = 1.570796 rad       # 受夹爪主动关节机械范围限制
V_MAX = 20.943951 rad/s    # 不超过 24 V 空载输出转速
T_MAX = 10 N·m             # 当前 MIT 命令范围，小于 12.5 N·m 厂家峰值
```

其中 `P_MAX` 不是电机可累计旋转角度，而是本夹爪曲柄关节允许的控制目标上限。

---

## 4. MJCF 参数换算

界面把 `Inertia`、`Damp` 列为“电机参数”，减速比单独列出。
**当前暂按它们为转子侧辨识参数处理。** 说明书没有明确写“转子侧”，因此这是建模假设。

理想减速器下：

```math
J_out = N^2 J_m
```

```math
B_out = N^2 B_m
```

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
      ctrlrange="-10 10"
      forcelimited="true"
      forcerange="-12.5 12.5"/>
  </actuator>

</mujoco>
```

仓库中的 `parallel_gripper.xml` 与 `parallel_gripper_prepared.xml` 已采用这一结构。
`forcerange` 是模型的最终厂家峰值保护，Python 控制器的 `T_MAX` 还会先做一层较保守的命令限幅。

### 参数含义

- `gear="1"`：joint 已定义在减速器输出轴；厂家转矩/转速也是减速后数据，不再重复乘 10。
- `ctrlrange="-10 10"`：匹配当前实机 `TMAX=10`。
- `forcerange="-12.5 12.5"`：表示厂家峰值输出能力。
- `armature`：当前按转子惯量经 10:1 减速器反射到输出轴。
- `damping`：当前按转子侧粘滞系数经 10:1 减速器反射到输出轴。
- `frictionloss=0.04`：经验初值，后续用低速/静摩擦实验替换。

---

## 6. `dcmotor.pdf` 的作用

MuJoCo 的 DC motor 模型对后续高保真建模有帮助，尤其是：

- 电压—电流—反电势关系；
- 转矩常数；
- 电阻、电感；
- 转矩—转速包络；
- 减速器反射惯量；
- 粘滞摩擦、库仑摩擦；
- 电流与热饱和。

但当前不建议直接用 `<dcmotor>` 替代 `<motor>`：

1. 实机控制接口是 MIT 的 `t_ff`，驱动器内部已有电流环；
2. 当前研究重点是 ADRC、机构动力学和接触力；
3. 手册给出的 `Rs/Ls` 是三相 BLDC 的相参数，不能未经换算直接作为等效 DC 参数；
4. 厂家给的是额定/峰值转矩，并未明确给出 MuJoCo `dcmotor` 所需的堵转转矩定义。

因此当前采用**输出轴受限力矩源**。若以后研究电压、电流、热和转矩—转速包络，再建立第二版 `<dcmotor>` 模型。

---

## 7. 当前建议参数

| MJCF 参数 | 当前值 |
|---|---:|
| actuator | `<motor>` |
| gear | 1 |
| ctrlrange | ±10 N·m |
| forcerange | ±12.5 N·m |
| armature | 0.002074755 kg·m² |
| damping | 0.02532329 N·m·s/rad |
| frictionloss | 0.04 N·m（经验值） |
| 最大输出转速参考 | 20.944 rad/s |
| 连续输出转矩参考 | 3.5 N·m |

### Python MIT 控制回路

MJCF `<motor>` 的 `ctrl` 单位是输出轴力矩。位置和速度闭环由 Python 显式计算：

```math
\tau_{cmd} = \operatorname{clip}\left(
K_p(p_{des}-p) + K_d(v_{des}-v) + t_{ff},
-T_{MAX}, T_{MAX}
\right)
```

控制器还会把 `p_des` 限制到 `[P_MIN, P_MAX]`，把 `v_des` 限制到
`[-V_MAX, V_MAX]`，并把 `t_ff` 单独限制到 `±T_MAX`。当前默认值为：

| MIT 参数 | 当前值 |
|---|---:|
| P_MIN / P_MAX | 0 / 1.570796 rad |
| V_MAX | 20.943951 rad/s |
| T_MAX | 10 N·m |
| Kp | 20 N·m/rad |
| Kd | 0.63793536 N·m·s/rad |
| t_ff | 0 N·m |

这些参数统一配置在 `configs/custom_parallel_gripper.toml`；控制实现位于
`src/parallel_gripper_tactile/control.py`。因此修改 MIT 命令范围不需要改 MJCF，但
`T_MAX` 必须同时不超过 `<motor ctrlrange>` 和 `<motor forcerange>`。

### 预接触与目标法向力跟踪

抓取控制在 MIT 内环之外增加 `simple-pid` 法向力外环。预接触阶段按位置斜坡低速闭合；
左右两侧 taxel 的法向力均超过接触阈值并保持若干仿真步后，记录当前输出轴位置
`p_contact` 并切换到力跟踪：

```math
F_n = \sum_i F_{n,i},\qquad
\Delta p = PID(F_{target}-F_n),\qquad
p_{des}=p_{contact}+\operatorname{clip}(\Delta p,-\Delta p_{max},\Delta p_{max})
```

总法向力先经过一阶低通滤波。`simple-pid` 的输出限幅同时实现积分抗饱和，随后仍由
MIT 内环和 `T_MAX` 执行最终力矩保护。当前默认 `F_target=8 N`，配置位于
`configs/custom_parallel_gripper.toml` 的 `[control.force]`。

---

## 8. 后续实验修正项

1. 确认 `Inertia`、`Damp` 是否明确为转子侧参数；
2. 辨识低速库仑摩擦和静摩擦；
3. 对齐实机输出轴零位与仿真 `qpos=0`；
4. 在夹爪负载下重新评估等效阻尼与摩擦；
5. 必要时再加入电流、电压和热限制。

---

## 资料依据

1. 《DM-J4310P-2EC 减速电机使用说明书 V1.1》，达妙科技，2025-11-21。
2. 当前实机串口助手截图：24 V 供电、MIT 模式、电机辨识参数及驱动配置。
3. MuJoCo DC Motor Model：<https://mujoco.readthedocs.io/en/latest/_static/dcmotor.pdf>

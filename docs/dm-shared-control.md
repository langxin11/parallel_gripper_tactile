# DMgripper 共享控制核

`packages/dm_grasp_core` 是独立包 `dm-grasp-core`，运行依赖仅为 numpy 与 simple-pid。
算法只接收数值，不访问设备、时钟、ROS 消息或 MuJoCo。仿真适配层
`parallel_gripper_tactile.control` 负责执行器绑定、profile 转换和名称再导出；
真机入口见 [DMgripper 通用抓取实验](dmgripper-experiments.md)。各后端分别拥有设备生命周期。

本文说明 DM 专用的机构映射和共享控制接口。Robotiq 使用独立控制核，不套用本文的曲柄滑块关系。

| 阅读目的 | 对应内容 |
| --- | --- |
| 从电机关节角换算夹爪开度 | [机构运动学](#dm-kinematics) |
| 核对执行器范围与配置基线 | [执行器基线](#actuator-baseline) |
| 确认局部接触模型的适用条件 | [模型适用边界](#contact-model-boundaries) |
| 选择目标力控制方法 | [动态目标力跟踪](force-tracking.md#contact-force-model) |
| 评价在线刚度估计的准确性 | [刚度辨识与验证](control-comparison-ablation.md#stiffness-identification) |

## 机构运动学与基控制设计 {#dm-kinematics}

`CrankSliderKinematics` 为 DM 控制器提供总开度 `aperture`、总闭合行程 `closure`、
闭合雅可比 `closure_jacobian` 和机械范围内的反解 `position_for_closure`。
运动学作为独立组件复用，由接近轨迹、导纳和法向力控制消费，不是一种独立的力控制器。

以曲柄角 `q`（rad）表示驱动关节位置。两指开口宽度为：

\[
\omega(q)=2\left[r\cos(q+\theta_0)+
\sqrt{l^2-\left(r\sin(q+\theta_0)-e\right)^2}\right]
\]

其中：

\[
\theta_0=\frac{\pi}{4},\qquad r=0.03\ \mathrm{m},\qquad
l=0.04\ \mathrm{m},\qquad e=\frac{0.03}{\sqrt2}\ \mathrm{m},
\qquad q\in[0,1.7].
\]

定义从张开初始位置起算的总闭合行程：

\[
c(q)=\omega(0)-\omega(q).
\]

令 \(\alpha=q+\theta_0\)，并记

\[
s(q)=\sqrt{l^2-\left(r\sin\alpha-e\right)^2},
\]

则总闭合行程相对于曲柄角的运动学雅可比为：

\[
\boxed{
J_c(q)=\frac{\partial c}{\partial q}=
2r\left[\sin\alpha+
\frac{\left(r\sin\alpha-e\right)\cos\alpha}{s(q)}\right]
}
\]

单位为 m/rad。在给定行程中 \(J_c(q)>0\)，即增加 `q` 会闭合夹爪。

| 曲柄角 | 开口宽度 \(\omega\) | 闭合行程雅可比 \(J_c\) |
| --- | ---: | ---: |
| \(q=0\) | 122.43 mm | 42.43 mm/rad |
| \(q=\pi/4\) | 78.05 mm | 60.00 mm/rad |
| \(q=\pi/2\) | 37.57 mm | 42.43 mm/rad |
| \(q=1.7\) | 32.25 mm | 30.49 mm/rad |

因此，夹爪在中间开口附近具有更大的位移传动比；固定的关节位置增益会随姿态表现出不同的力控增益。

以上参数描述本文的 DM 机构；实际控制的位置、速度与力矩边界由组合配置决定。
反解在给定机械范围内裁剪目标闭合行程；无效几何或非单调范围会报错。
实现位于 `packages/dm_grasp_core/src/dm_grasp_core/control/kinematics.py`。

## 局部接触模型的适用边界 {#contact-model-boundaries}

运动学关系描述机构位置；将它用于接触力与力矩映射时，还需要以下局部接触假设。

!!! warning "接触变化时退回保守反馈"

    力控局部模型适用于左右手指同步闭合、物体大致居中、接触集合在一个控制周期内不变化的准静态阶段。
    接触建立、脱离、滑移或新的 Pillar 加入接触时，模型的导数不连续，应退回保守反馈控制。

[局部力控模型](force-tracking.md#contact-force-model)不应直接用于以下情况：

- 物体明显偏心，左右法向力不平衡；
- 接触面曲率大，接触法向随位姿快速旋转；
- 接触柱集合频繁增减；
- 材料有显著黏弹性、塑性或加载／卸载滞回；
- 控制带宽接近机构柔性模态或传感器延迟主导的频段。

在这些场景中，应将上述关系视为前馈和增益调度的先验，并持续使用触觉闭环来保证最终力跟踪。

## 核心接口与职责

- 曲柄滑块正反解、雅可比、五次接近轨迹、双侧接触确认和速度过渡。
- 二阶导纳 `step_admittance`：平均双侧力误差驱动闭合位移，先按机构雅可比裁剪速度再积分，
  同时限制内部状态、目标构型速度和 MIT 合成力矩。
  统一自适应模式显式启用 `saturation_feedback`，把最终限幅后的 MIT 位置／速度请求回投导纳状态，
  以 `execution_limited` 反馈目标调度；默认关闭以保持旧控制轨迹。此处不包含协议量化或实测响应补偿。
- `MITCommandConfig`、五字段 `MITCommand`、`build_mit_command`：生成量化前 MIT 请求；
  `MITTorqueModel`：协议量化与合成力矩计算。
- 法向外环 `begin_tracking`／`step_tracking`：PID、一阶 LADRC、直接力矩、二阶 LADRC 路径，
  经 `MITTorqueInner` 注入电机访问；另提供三种在线接触刚度估计。
  PID 可通过 `NormalForceConfig.pid_torque_feedforward_gain` 显式覆盖模型力矩前馈，
  按当前机构雅可比与目标力计算，不依赖刚度估计；`None` 保持既有仿真配置的前馈选择。

导纳方程为：

\[
M\ddot c+B\dot c+Kc=f_{target}-\frac{f_{left}+f_{right}}{2}.
\]

共享接触状态机负责 `approach` → `contact_transition` → `force_tracking` 及持续失接触后的
重接近；具体控制器决定跟踪阶段的 MIT 请求。目标曲线从跟踪建立后计时，接近不占用任务时间。
共享算法不意味着后端的使能、bias、故障与释放策略相同。

## 配置与执行器基线 {#actuator-baseline}

DMgripper 仿真实验由 `platform`、`model`、`controller`、`estimator`、`task`、`material` 与
`execution` 配置组组合；`configs/dm_gripper.yaml` 只保留为独立 profile schema 示例和底层
Python API 的兼容默认值，不是实验组合入口。模型选择见[DMgripper 入口](grippers/dmgripper/index.md)，
CAD 重建步骤随资产维护，位置见[资产维护入口](grippers/dmgripper/index.md#asset-source)。

```bash
uv run pgt validate configs/dm_gripper.yaml
uv run python scripts/research/run.py execution=plan
uv run pgt view grasp --set model=dm_gripper/height_spheres
```

Pillars 使用等效接触参数 `solref="-1200 -10"` 与
`solimp="0.75 0.95 0.0025 0.5 2"`；它们不是独立的硅胶有限元模型。力控主量是平均单侧
法向力 `f_n=(F_L+F_R)/2`，在线 `k_pair` 是 Pillar—物体—机构／接触链路的组合等效刚度，
只用于前馈、增益调度和实验比较。

### 电机与执行器一致性

项目 MIT 映射固定为 PMAX=`1.7 rad`、VMAX=`8 rad/s`、TMAX=`4.0 N·m`。修改这些范围时，
必须同步更新电机寄存器、上位机 profile 的量化范围，以及 MuJoCo actuator 的 `ctrlrange` 和
`forcerange`；只改其中一处会让相同的 CAN 位域对应错误的物理量。prepared MJCF 当前对
`gripper_drive` 使用 ±4 N·m 的 `ctrlrange` 与 `forcerange`。

Onshape 原始导出中的 position actuator 与 `forcerange=12.5` 可用于保留导出信息或短时能力
核对，但 `12.5 N·m` 不等于本项目的连续使用限幅；持续控制以 profile 与 prepared MJCF 的 ±4 N·m
为准。

### 修改后的验收

每次修改 CAD 导出、profile 或控制参数后，至少运行本节的 profile 验证和 Viewer 检查。
然后使用相同的 force-tracking task 比较 `f_n` 跟踪误差、限幅比例、接触状态和 `k_pair` 轨迹；
不要将历史计划或单次截图当作配置事实来源。

## 仿真入口

```sh
# PID／导纳使用共同 task 与平台的对比组合；动画追加 --set execution.viewer=true
uv run python scripts/research/run.py experiment=dm_gripper/force_tracking_pid_unified
uv run python scripts/research/run.py experiment=dm_gripper/force_tracking_admittance_unified
uv run pgt run force-track --experiment dm_gripper/force_tracking_admittance_unified
```

参数由当前 controller、platform、task 组合确定，不将某次调优数值写成通用默认值。
PID 统一入口继承导纳统一入口的共同实验设置，再覆盖控制器与实验标识；平台、任务、材料与记录周期只维护一份。
统一组合隔离控制律差异，不保证两种方法均已调优；饱和或跟踪失败须保留为实验结果。
正式研究执行约定见[运行流程](workflows.md)。

## 时序与量化边界

导纳外环只在配置的控制周期积分；仿真每个物理步用最新实测位置、速度重新计算上一 MIT 请求的
合成力矩，以表示电机内环。不能将一次合成力矩保持整个外环周期当作等价实现。

`DMAdmittanceController.last_requested_command` 为量化前请求；trace 的 `control`、
`mit_feedforward_torque_n_m`、`motor_torque_n_m` 分别为量化后位置、前馈和仿真合成力矩，
均不能解释为电机实测力矩。导纳 trace 按外环周期保存，快照记录核心版本与执行器应用方式。

当前仿真平台使用位置编码区间 `[0,1.7]` 与 Python `round`；真机部署协议使用 `[-1.7,1.7]`
与非负编码值的 `floor(x+0.5)`。两者位置区间及恰好半整数时的取整规则不同。
共享请求对齐不等于协议字节或物理行为等价。更改量化规则须显式记录并保留既有实验基线。

## 构建与验证

```sh
uv build --package dm-grasp-core --wheel
```

其他环境应安装构建产物，记录包版本、源码提交及 wheel 校验值，不跨仓复制算法文件。
安装路径和部署流程由目标工作区维护，本仓不假定外部 ROS 工作区的目录或依赖锁定状态。

常规门禁见[测试策略](testing.md)。共享核使用仓库内的独立数学期望验证控制公式、状态更新、
限幅、复位、正反方向和不规则时间步，不再加载仓库外 ROS 节点或维护迁移期固定轨迹。
MuJoCo 烟雾仅证明能进入跟踪并产生有限输出，不构成硬件稳定性或抓取性能结论。

# DMgripper 共享控制核

`packages/dm_grasp_core` 是独立包 `dm-grasp-core`，运行依赖仅为 numpy 与 simple-pid。
算法只接收数值，不访问设备、时钟、ROS 消息或 MuJoCo。仿真适配层
`parallel_gripper_tactile.control` 负责执行器绑定、profile 转换和名称再导出；
真机入口见 [DMgripper 通用抓取实验](dmgripper-experiments.md)。各后端分别拥有设备生命周期。

## 核心接口与职责

- 曲柄滑块正反解、雅可比、五次接近轨迹、双侧接触确认和速度过渡。
- 二阶导纳 `step_admittance`：平均双侧力误差驱动闭合位移，先按机构雅可比裁剪速度再积分，
  同时限制内部状态、目标构型速度和 MIT 合成力矩。
- `MITCommandConfig`、五字段 `MITCommand`、`build_mit_command`：生成量化前 MIT 请求；
  `MITTorqueModel`：协议量化与合成力矩计算。
- 法向外环 `begin_tracking`／`step_tracking`：PID、一阶 LADRC、直接力矩、二阶 LADRC 路径，
  经 `MITTorqueInner` 注入电机访问；另提供三种在线接触刚度估计。

导纳方程为：

\[
M\ddot c+B\dot c+Kc=f_{target}-\frac{f_{left}+f_{right}}{2}.
\]

共享接触状态机负责 `approach` → `contact_transition` → `force_tracking` 及持续失接触后的
重接近；具体控制器决定跟踪阶段的 MIT 请求。目标曲线从跟踪建立后计时，接近不占用任务时间。
共享算法不意味着后端的使能、bias、故障与释放策略相同。

## 仿真入口

```sh
uv run pgt run force-track --experiment dm_gripper/force_tracking_admittance
# 动画追加 --set execution.viewer=true

# PID／导纳使用共同 task 与平台的对比组合
uv run python scripts/research/run.py experiment=dm_gripper/force_tracking_pid_unified
uv run python scripts/research/run.py experiment=dm_gripper/force_tracking_admittance_unified

# 导纳候选研究：默认只计划，执行追加 execution=study_run
uv run python scripts/research/study.py research=dm_admittance_tuning/study
```

参数由当前 controller、platform、task 组合确定，不将某次调优数值写成通用默认值。
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

常规门禁见[测试策略](testing.md)。共享核固定回归数据
`packages/dm_grasp_core/tests/golden_tracking.json` 保存迁移前输出及来源 SHA256；
生成器须读取保存的原始实现，不能用当前实现刷新期望值。

可选 ROS 对齐检查：

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest tests/test_dm_ros_alignment.py
```

需先安装 ROS 和消息包，并将目标 `dm_gripper_control` 加入 `PYTHONPATH`；缺少环境时明确跳过。
测试以实际 `_control_tick` 方法和内存输入／发布器比较跟踪阶段五字段请求及导纳状态，不创建 ROS 节点，
不验证设备保护。MuJoCo 烟雾也仅证明能进入跟踪并产生有限输出，不构成硬件稳定性或抓取性能结论。

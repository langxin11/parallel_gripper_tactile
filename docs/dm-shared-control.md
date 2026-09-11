# DMgripper 共享控制核

`packages/dm_grasp_core` 是独立构建的 `dm-grasp-core==0.1.0`，运行依赖仅限 numpy 与
simple-pid 两个纯计算库，不包含 ROS、MuJoCo、串口或模型路径。
仿真通过 uv workspace 使用它，ROS 2 的 `dm_gripper_control` 安装同版本 wheel。
算法修复只改此包；两端的输入适配、调度和设备生命周期分别维护。
仿真侧 `parallel_gripper_tactile.control` 是该核的适配层：负责 MIT 执行器绑定、
profile 配置转换与公共名称再导出，不重复实现控制律。

## 已共享的内容

- 曲柄滑块正/反解与雅可比、五次接近轨迹、接触确认和速度过渡。
- 二阶导纳：目标力与平均双侧反馈力之差沿闭合方向驱动虚拟位移：

\[
M\ddot c+B\dot c+Kc=f_{target}-\frac{f_{left}+f_{right}}{2}.
\]

- 导纳内部状态限幅、目标构型速度逆映射、实测构型力矩前馈和 MIT 合成力矩约束。
- 达妙 MIT 协议量化与力矩命令计算（`MITTorqueModel`）、三种方法的在线接触刚度估计、
  二阶直接力矩 LADRC，以及法向力外环状态机与 PID、一阶 LADRC、直接力矩、
  二阶 LADRC 四条跟踪路径（电机访问经 `MITTorqueInner` 协议注入）。
- 显式 `MITCommandConfig`、五字段 `MITCommand`、`build_mit_command` 与
  `step_admittance`。算法只接收数值，不读取时钟、设备、ROS 消息或 MuJoCo 数据。

`dm_gripper_control.control` 保留原导入路径，实际重导出共享实现。ROS 标定、使能、
bias、输入有效性、新鲜度、HOLD/FAULT 和失能策略保留现有行为。仿真接触阶段使用
相同算法原语，但不模拟 ROS 的设备生命周期；共同回放验证范围是跟踪阶段 MIT 请求。
仿真侧全部 DM 力控外环（PID、刚度估计与两条 ADRC）已随上述模块迁入核心；
`parallel_gripper_tactile.control` 只保留执行器绑定、profile 配置转换与名称再导出。

## 运行仿真

在 `parallel_gripper_tactile` 根目录执行：

```bash
uv sync --all-groups
uv run pgt run force-track \
  --experiment dm_gripper/force_tracking_admittance
```

需要查看动画时添加 `--viewer`。示例为 4 ms 外环、1 N 平均单侧目标和 1 N
双侧接触进入阈值；MIT 使用 ROS 当前参考值 kp=10、kd=5。平均单侧力经过 2 Hz
一阶低通后进入导纳，原始双侧力仍用于接触与释放判定。原 `CONTROLLER_VARIANTS`
默认实验矩阵不含本变体。

接触进入和退出使用滞回：两侧达到 1 N 后进入接触过渡；跟踪阶段只有任一侧
连续 `release_confirm_steps` 个周期不高于 `release_threshold_n` 才重新接近。示例中
释放阈值为 0.05 N，25 个 4 ms 周期对应 100 ms，瞬时单侧掉力不会重置导纳。

示例导纳参数为 `M=0.20 kg`、`B=15 N·s/m`、`K=1 N/m`，接近速度和跟踪速度
上限均为 `0.05 rad/s`，接触过渡为 50 ms，接近前馈为 0.5 N。参数由
1.0→1.4→1.0 N 的 Ramp 任务筛选，最低目标与接触阈值一致。调参排序使用物理步进后
左右触觉侧力的平均值，并包含切入跟踪的首个瞬态，避免低通或忽略窗口掩盖冲击。
在 hard/explicit 接触、固定噪声种子 0 和 1 下，物理力峰值绝对误差平均为
0.086 N，忽略最初 0.2 s 后的物理力 RMSE 为 0.022 N，滤波跟踪 RMSE 为
0.007 N；Ramp 阶段的 `force_tracking` 占比为 100%，且未触发位置或力矩饱和。
低峰值的代价是该场景从全开位置建立 1 N 接触约需 41.5 s；这仍是仿真调参结果，
不代表已通过实机安全验收。
可用下列 Hydra 正式入口重跑候选；执行命令可追加 `execution.workers=8` 使用 CPU 多进程：

```bash
uv run python scripts/research/study.py \
  research=dm_admittance_tuning/study
uv run python scripts/research/study.py \
  research=dm_admittance_tuning/study \
  execution=study_run
```

`control.force.admittance` 只由 `admittance` 入口使用，和 ADRC/直接力矩反馈互斥。
接近轨迹、前馈和接触过渡由此段配置；任务的接近超时、参考曲线、控制周期和接触后
等待仍有效。`control.force.geometry` 及 `control.mit` 提供机构几何与内环参数。
使用不带导纳配置的普通 profile 时，该变体注入导纳默认值，但沿用该 profile 的
MIT 参数；要重现实机基线，应使用专门的示例 profile。

## PID／导纳统一对比

控制律隔离对比不改写上述 ROS 对齐基线，而使用两条独立组合：

```bash
uv run python scripts/research/run.py \
  experiment=dm_gripper/force_tracking_pid_unified
uv run python scripts/research/run.py \
  experiment=dm_gripper/force_tracking_admittance_unified
```

两条组合共享 PID Ramp 目标、4 ms 外环、`kp=20`、`kd=0.63793536`、6 s 线性关节接近、
1 N 接近前馈和公共双侧接触状态机。状态依次为 `approach`、`contact_transition`、
`force_tracking`：双侧连续 5 个周期达到 0.15 N 后，用 50 ms 五次曲线将接近期望速度降至零；
跟踪中任一侧连续 25 个周期不高于 0.05 N，状态机才返回 `approach`。目标力曲线只在
`force_tracking` 建立后开始计时，接近耗时不会占用 Ramp。

公共状态机属于 `dm-grasp-core`，不包含 PID 或导纳方程。统一入口下，具体控制器只决定
跟踪阶段如何把同一平均单侧力误差转换为 MIT 请求。现有导纳参数尚未针对 6 N Ramp 调优，
因此其饱和或跟踪失败应作为实验结果保留。

DMgripper 的其他 PID、刚度前馈、直接力矩及 ADRC 配置也启用同一公共状态机；它们所用的
接近时长、等待上限和目标曲线仍由各自选择的 task 决定。旧导纳实验不再由内部低速轨迹接管，
因此不会再出现约 41 s 的预接触等待。

共享二阶导纳在更新位移前先按当前机构雅可比裁剪速度，再积分位移；仿真适配器不再在
积分完成后才补做角速度裁剪。这一顺序用于避免单个外环周期生成越过限速边界的位置跳变。

## 两层控制周期与量化边界

ROS 以外环频率发送 q/dq/kp/kd/tau_ff，由电机内部控制器持续计算力矩。仿真新导纳
变体在每个物理步用最新 q/dq 重新计算上一 MIT 请求的合成力矩，导纳积分仍只在
4 ms 外环时钟触发。若把一次合成力矩保持 4 ms，不能等价模拟 kp=10、kd=5 的电机
内环；迁移烟雾测试曾暴露此差异。旧 PID/ADRC 的时序不因此改变。

`DMAdmittanceController.last_requested_command` 是共享核的量化前请求。
既有 trace 的 `control`、`mit_feedforward_torque_n_m` 和 `motor_torque_n_m`
仍表示仿真量化后的位置、前馈和合成力矩；不得把它们当作电机实测力矩。
导纳 trace 默认按外环周期保存，运行快照记录核心版本和执行器应用方式。

本阶段刻意保留原仿真量化器：其位置区间 `[0,1.7]` 和取整方法，与当前官方 SDK
`[-1.7,1.7]` 对称区间及截断不同。已验证的是共享请求一致；协议字节对齐须另作
显式变更并保留原实验基线，不能称本阶段已实现全部仿真实机等价。

## 独立构建与安装

在仿真仓根目录构建。下例使用本机系统 Python 已安装的 setuptools/wheel，不联网：

```bash
uv build --package dm-grasp-core --wheel --offline \
  --python /usr/bin/python3 --no-build-isolation
sha256sum dist/dm_grasp_core-0.1.0-py3-none-any.whl
```

在 ROS 工作区根目录安装到原有 `.venv`，再走工作区构建脚本：

```bash
uv pip install --python .venv/bin/python --no-index --no-deps \
  ../parallel_gripper_tactile/dist/dm_grasp_core-0.1.0-py3-none-any.whl
./scripts/build_ros2.sh
```

ROS 包的 `install_requires` 不代替上述安装。保留每次验收过的 wheel、SHA256、
核心版本及源码提交或工作树快照，升级版本后才切换实机；不要跨仓复制 Python 文件。

当前 ROS 工作区已将验收后的 wheel 保存在 DM 子仓 `wheels/`，并以工作区
`pyproject.toml`/`uv.lock` 固定该本地发行包，因此日常 `uv sync --locked` 也能
恢复控制核。未来发布新版本时同步更新发行包和版本约束；开发中的源文件改动不会
自动替换实机使用的 wheel。

## 验证

默认仿真测试包括共享核及其迁移前固定输出：

```bash
uv run ruff check .
uv run ruff format --check .
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest
```

`packages/dm_grasp_core/tests/golden_tracking.json` 固化迁移前 ROS 代码的 240 步
输出，并记录两份来源文件 SHA256；覆盖方向、2/4 ms、不规则步长和复位/限幅场景。
生成器必须读取保存的迁移前代码，不能拿新实现刷新期望值。

两端共同回放需加载 ROS、已构建的 `dm_gripper_msgs` 和 `papillarray_interfaces`，
并令 `PYTHONPATH` 包含 ROS 的 `dm_gripper_control` 包目录。然后在仿真仓执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest tests/test_dm_ros_alignment.py
```

该测试不创建 ROS 节点或通信实体：实际 `_control_tick` 方法使用内存输入和发布器，
与仿真适配器比较 2 ms、4 ms、不规则周期共 540 步的五字段请求及导纳状态。
缺少 ROS 环境时此组集成测试明确跳过。节点保护由 ROS 仓的 safety 测试单独验证。
MuJoCo 烟雾只证明代码可进入跟踪并产生有限输出，不构成硬件稳定性或抓取性能结论。

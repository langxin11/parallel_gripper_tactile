# DMgripper 共享导纳控制核

`packages/dm_grasp_core` 是独立构建的 `dm-grasp-core==0.1.0`，只有 Python 标准库依赖。
仿真通过 uv workspace 使用它，ROS 2 的 `dm_gripper_control` 安装同版本 wheel。
算法修复只改此包；两端的输入适配、调度和设备生命周期分别维护。

## 已共享的内容

- 曲柄滑块正/反解与雅可比、五次接近轨迹、接触确认和速度过渡。
- 二阶导纳：`M*c_ddot + B*c_dot + K*c = F_target - (F_left+F_right)/2`。
- 导纳内部状态限幅、目标构型速度逆映射、实测构型力矩前馈和 MIT 合成力矩约束。
- 显式 `MITCommandConfig`、五字段 `MITCommand`、`build_mit_command` 与
  `step_admittance`。算法只接收数值，不读取时钟、设备、ROS 消息或 MuJoCo 数据。

`dm_gripper_control.control` 保留原导入路径，实际重导出共享实现。ROS 标定、使能、
bias、输入有效性、新鲜度、HOLD/FAULT 和失能策略保留现有行为。仿真接触阶段使用
相同算法原语，但不模拟 ROS 的设备生命周期；共同回放验证范围是跟踪阶段 MIT 请求。
PID、刚度估计及 ADRC 暂时保留在历史仿真实现中，没有全部迁入新包。

## 运行仿真

在 `parallel_gripper_tactile` 根目录执行：

```bash
uv sync --all-groups
uv run pgt run force-track \
  --profile configs/custom_parallel_gripper_admittance.yaml \
  --task configs/force_tracking/dm_admittance.yaml \
  --controller-variant admittance
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
可用下列 Hydra 正式入口重跑候选（串行执行）：

```bash
uv run python scripts/research/study.py \
  --config-name dm_admittance_tuning
uv run python scripts/research/study.py \
  --config-name dm_admittance_tuning \
  study_execution=run
```

`control.force.admittance` 只由 `admittance` 入口使用，和 ADRC/直接力矩反馈互斥。
接近轨迹、前馈和接触过渡由此段配置；任务的接近超时、参考曲线、控制周期和接触后
等待仍有效。`control.force.geometry` 及 `control.mit` 提供机构几何与内环参数。
使用不带导纳配置的普通 profile 时，该变体注入导纳默认值，但沿用该 profile 的
MIT 参数；要重现实机基线，应使用专门的示例 profile。

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

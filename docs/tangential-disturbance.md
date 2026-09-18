# 切向扰动下的触觉增力

所属主题：[自适应抓取](adaptive-grasping.md)。三条路线的关系与选择见主题总览。

`tangential-disturbance` 是 DMgripper 的纯仿真实验。它检验在稳定夹持后，控制器能否只依据触觉力的变化提高法向目标，以抵抗世界 `YZ` 接触平面内的外加载荷。它不是在线摩擦估计实验，也不把外载、物体位移、速度或真实摩擦系数输入控制律。

可选 `force_ratio` 参考 [Gentle Grasping](https://imec-publications.be/server/api/core/bitstreams/1a8e6d21-1ce1-4a84-aa50-8a44aaaf28a7/content)
的变化量比值判据，但传感器、模型与控制器不同，不构成完整复现或真实微滑证明。

## 运行边界与时序

实验只接受仿真 DM `pid-torque-ff` 或 `pid-only` 组合。两者使用现有的 MIT 法向 PID 外环；外环在 `control_period_s` 到期时读取触觉并生成目标，MIT 内环则在每个 MuJoCo 物理步用最新关节状态保持执行该目标。`admittance`、ADRC 变体和 viewer 会在计划预检时拒绝。

本实验固定使用 `multiccd_enabled=true`，不接受显式轨迹降采样周期；
`execution.trace_sample_period_s=null` 表示按完整物理步保存。

阶段顺序如下：

1. 关节接近并确认法向力进入跟踪状态。
2. 法向力在初始目标附近连续稳定后撤去支撑。
3. 进行 `initial_hold_s` 的无外载保持；只有持续力跟踪、法向误差与位移均合格才开始扰动。
4. 施加 `ramp`、`step` 或 `pulse` 切向载荷，并在恢复阶段保持最终法向目标。

法向目标只允许增加，回落保持阶段也不会主动降低。策略的上限、最大增长率与控制周期均写入 task，因此参数调整时必须以有效 task 快照为准。

## 检测器与策略

默认检测器 `shear_increase` 先对每侧 taxel 的三轴力求合力，再取该侧切向模，
最后将两侧切向模相加并滤波得到 `S`。稳定保持末刻的滤波值为 `S0`，据此构造有限目标包络：

\[
F_\mathrm{envelope}=\min\!\left(F_\max,\;F_0+g\max(0,S-S_0)\right).
\]

它是工程上的有限增力规则，不估计摩擦系数，也不会因持续载荷在每个控制周期无界增加。包络可随剪切卸载下降，但实际请求与 MIT 目标均保持只增不减。可选 `force_ratio` 只使用左侧局部 `y` 合力与左侧法向力；法向分母不足时该比值标为无效，而非填充为零。

三种增力策略为：

- `constant`：始终维持初始目标；
- `fixed_step`：触发确认后按固定步长增加；
- `dynamic_step`：触发确认后按剪切变化和闭合量缩放步长。

`shear_increase` 的 `alpha` 由切向增量相对阈值的评分映射至 `[0,1]`；
`force_ratio` 的 `alpha` 为有效比值绝对值裁剪至 `[0,1]`。闭合量由夹爪编码器和机构
运动学换算，再以 `closure_scale_m` 归一化；它是位移代理，不是材料刚度真值。
`dynamic_step` 的 `exp(alpha)+exp(-normalized_closure)` 是为仿真状态量明确改写的无量纲表达；它不是论文公式的逐项实现。原文中按一秒法向均值上调的机制未启用，底层 MIT PID 也替代了论文中的导纳控制路径。

## 任务、指标和产物

任务组为 `tangential_disturbance/ramp`、`step`、`pulse`；`policy`、`disturbance`、`metrics`
与周期以有效 task 为准。真值摩擦只用于场景，Hydra `material` 写入 task 的 `object_material`。

`passed` 同时要求数值稳定、完整完成、初始保持合格、扰动阶段最大切向位移未超过滑移阈值，以及恢复窗口成立。峰值实际法向力、最终实际法向力与最大切向位移均只在 `disturbance` 阶段评分；初始保持的位移以独立的 `initial_hold_displacement_m` 评估。恢复从最后一次载荷变化之后开始计时：物体切向速度、实际法向力相对目标的误差和控制状态必须连续满足 `recovery_dwell_s`，并一直维持到扰动结束。`recovery_time_s` 是该连续窗口开始相对最后载荷变化的时间；它为空即不通过。

runner 保存全物理频率 `trace.csv`、指标、输入快照、有效参数、`plot.png`／`plot.pdf` 与 manifest。
两种出图模式均显示初始保持与扰动阶段的法向力、切向量、离线外载／位移和触发点；空轨迹不绘图。

## 当前回归结果

2026 年 9 月 12 日，固定 `full`、`hard`、50 g 方块、真实接触摩擦 0.8、
初始平均单侧力 0.8 N、附加载荷 1.5 N，三种策略与三种扰动各运行 seed 0、1、2，
共 27 次。seed 0 参与响应参数选择，seed 1、2 用于同场景回归，不能据此宣称跨材料泛化。
下表为各三个 seed 的最大切向位移范围；阈值保持 2 mm。

| 扰动 | 固定目标 | 固定步长 | 动态步长 |
| --- | ---: | ---: | ---: |
| 斜坡 | 179.08～183.09 mm | 0.0022～0.0024 mm | 0.0022～0.0024 mm |
| 阶跃 | 185.36～191.41 mm | 2.93～2.97 mm | 1.50～1.56 mm |
| 脉冲 | 185.36～191.41 mm | 2.93～2.98 mm | 1.50～1.57 mm |

动态步长在这九个条件中均满足完整验收，阶跃后约 0.27～0.28 s 进入最终稳定窗口；
固定步长在阶跃和脉冲中恢复了保持，但位移已超过阈值，因此保留为科学失败。
固定目标对照发生滑落，说明相同摩擦求解设置未掩盖夹持能力不足。
斜坡的微米级位移属于当前接触求解器的仿真结果，不代表实机检测分辨率。
这些结果针对给定步长和响应参数，不证明动态步长普遍优于更快的固定步长。

另有零附加载荷负例验证不发生目标力持续上爬，初始握力不足和初始保持中失去跟踪的负例
验证不会进入扰动并伪造成功。`force_ratio` 通过符号与零分母单元验证，尚无此矩阵下的
滑移检出或恢复性能结论；不要把默认策略的成功率移用于它。

## 命令

```bash
uv run pgt run tangential-disturbance --experiment dm_gripper/tangential_disturbance
```

切换纯 PID 使用 `--set controller=dm_gripper/pid_only --set estimator=none`。
科研入口可通过 `experiment=dm_gripper/tangential_disturbance execution=plan` 校验组合；
探索性 Multirun 可扫描 task、`task.definition.policy.strategy` 与 seed，尚无正式 study。
MIT 接收法向外环生成的关节请求；法向目标不直接作为电机力矩。仿真结果不替代实机验证。

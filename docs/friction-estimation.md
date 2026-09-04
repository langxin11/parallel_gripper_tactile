# 🧭 微滑移探测与保守摩擦估计

`friction-estimate` 在 oracle 目标力调度基线之上增加一条盲估计链路：固定预抓取法向力，沿世界
`+Y` 缓慢增加切向探测载荷，从左右逐 taxel 三轴触觉力中检测初始滑移，冻结保守摩擦系数下界
`μ_hat`，撤去探测载荷，再把 `μ_hat` 交给现有目标力调度器抵抗重力和动态附加载荷。

## 能力边界

估计器只接收以下量：

- 已知切向探测载荷；
- 左右触觉面的法向合力和剪切合力大小；
- 当前控制周期。

真实摩擦系数、物体位移和速度不进入估计器。它们只在仿真结束后用于评价估计误差、保守性和探测
位移。每次运行的 `effective_parameters.json` 会记录
`oracle_signals_used_by_estimator: []`。

当前 MuJoCo 模型提供逐 taxel 接触力，但没有真实硅胶内部形变、柱体微振动或视觉标记。因此本实现是
**力域初始滑移代理**：它验证探测状态机、保守下界和后续力控闭环，不能等同于实机 PapillArray
微滑移算法。文献中的典型做法同样在检测到初始滑移时记录切向/法向力比；真正的局部起滑检测则通常
依赖柱体形变、振动或分布式形变场（[Tremblay 与 Cutkosky，1993](https://bdml.stanford.edu/oldweb/touch/publications/tremblay_icra93.pdf)、
[Khamis 等，2018](https://contactile.com/wp-content/uploads/2022/01/KhamisEtAl2017_PapillArray_ProofOfConcept_preprint.pdf)、
[Sui 等，2021](https://ieeexplore.ieee.org/document/9565930/)）。

## 运行方法

名义摩擦场景：

```bash
uv run pgt run friction-estimate \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/friction_estimation/nominal_friction.yaml
```

标准 task 还包括：

- `low_friction.yaml`：`μ=0.35`；
- `high_friction.yaml`：`μ=1.2`；
- `noisy_friction.yaml`：`μ=0.8`，逐 taxel 测量噪声放大两倍。

## 阶段机

```mermaid
flowchart LR
  A[接近并确认接触] --> B[撤去支撑并稳定]
  B --> C[世界 +Y 切向载荷斜坡]
  C -->|检测到力域起滑| D[冻结保守 μ 下界]
  C -->|达到探测上限| E[显式回退 μ]
  D --> F[撤去探测载荷并恢复]
  E --> F
  F --> G[用估计或回退 μ 调度目标力]
  G --> H[重力/动态增载保持验收]
```

探测方向选世界 `+Y`，与世界 `-Z` 重力正交。每一侧先对 taxel 三轴力逐分量求和，再计算局部
剪切模，避免把可相互抵消的内部剪切错误累加：

\[
N_s=\max(0,F_{z,s}),\qquad
T_s=\sqrt{F_{x,s}^2+F_{y,s}^2},\qquad
\rho=\frac{T_L+T_R}{N_L+N_R}.
\]

## 探测与保守估计

探测器并行使用两个触觉判据：

1. 已知切向需求与触觉剪切支撑出现持续失配；
2. `ρ` 随探测载荷明显上升后趋于饱和，并且左右两侧摩擦比保持一致。

任一判据持续超过 `mismatch_confirm_s` 才锁存起滑，候选起滑期间的样本不再写入估计窗口，避免滑动
后的动力学污染静摩擦极限。确认后取起滑前窗口中 `ρ` 的高分位数并乘安全折减：

\[
\hat\mu_{LB}
=\operatorname{clip}\!\left(
  \eta Q_q\{\rho_k\}_{pre-slip},\ \mu_{min},\ \mu_{max}
\right),\qquad 0<\eta\leq1.
\]

标准场景使用 `q=0.85`、`η=0.9`；两倍噪声场景把 `η` 收紧到 `0.85`。高分位用于靠近摩擦极限，
折减用于吸收噪声和模型误差。如果达到
探测上限、接触不足或样本不足，则返回配置的 `fallback_friction_coefficient`，并把
`using_fallback` 置为真；不会把未起滑时的普通静摩擦利用率冒充真实 `μ`。

估计完成后的平均单侧目标力仍使用：

\[
f_{ref}=\operatorname{clip}\!\left(\frac{\gamma D}{2\hat\mu_{LB}},
f_{min},f_{max}\right).
\]

由于 `μ_hat` 是下界，同一安全系数下得到的目标力不小于使用真值的 oracle 目标。

## MuJoCo 求解条件

正式 task 必须显式设置 `noslip_iterations: 5`。实测关闭该后处理时，软接触在摩擦锥内的数值爬移
会让 `μ=0.2/0.5/0.8` 都在约 `0.52 N` 探测载荷下达到位移阈值，摩擦系数不可辨识；设置为 5 后，
起滑载荷和触觉比才随真实摩擦变化。提高到 20 的结果与 5 接近。MuJoCo 官方也说明 NoSlip
后处理用于抑制软接触模型的慢滑移；它不会阻止超过摩擦锥后的滑动
（[MuJoCo Modeling 文档](https://mujoco.readthedocs.io/en/latest/modeling.html#slow-slippage)）。

## 当前标准场景结果

默认 profile、`hard` 接触和固定噪声种子下：

| 场景 | 真值 μ | 保守估计 μ | 估计/真值 | 探测最大位移 | 后续保持最大位移 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 低摩擦 | 0.35 | 0.310 | 88.6% | 0.006 mm | 0.002 mm |
| 名义摩擦 | 0.80 | 0.736 | 92.0% | 0.893 mm | 0.011 mm |
| 高摩擦 | 1.20 | 1.109 | 92.4% | 1.318 mm | 0.017 mm |
| 两倍噪声 | 0.80 | 0.714 | 89.2% | 0.114 mm | 0.007 mm |

四个场景的估计均不高于真值，后续动态载荷保持的真实摩擦裕量为正。上述结论只覆盖当前模型、任务和
噪声种子；迁移硬件前还需要多 seed 统计、真实切向加载标定和失败回退试验。

## 运行产物

每次运行保存 `profile.yaml`、`task.yaml`、`effective_parameters.json`、`trace.csv`、`metrics.json`、
`plot.png`、`plot.pdf` 和 `manifest.json`。`trace.csv` 同时记录触觉输入、残差、利用率、估计值、
探测载荷、目标力与物体运动；真实 `μ`、摩擦容量和物体运动字段仅用于离线评分。

# Robotiq 2F-85 离散力控制

## 实现范围

`pgt run discrete-force` 把 Robotiq 2F-85 的 tendon actuator 视为 `0～255` 的离散命令空间。
所有控制变体（包括 PI 对照）最终都经过相同的四舍五入和限幅，只会输出整数动作：

\[
\Delta u\in\{0,\pm1,\ldots,\pm n_{\max}\}。
\]

其中 `Δu=0` 是明确的 `HOLD` 决策；实现不会把任意很小的连续量强制改成一个 tick。当前阶段只研究
法向力稳定，不读取或控制滑移。

## 状态机与局部估计

状态机按 `APPROACH → WAIT_STABLE → ADJUST → HOLD` 运行。超过安全力上限时进入 `RELEASE`，
释放动作始终为 `-1 tick`。接近阶段使用 3 tick 动作和独立的 50 ms 动作间隔，避免执行器尚未响应时
连续排入多个粗动作。

每次调节动作真正经过可配置延迟并写入 actuator 后，控制器才开始计时。最近 20 个滤波力样本的范围
小于 `0.10 N`，且动作后至少经过 `0.20 s`，才接受新的稳定力。正常调节动作的稳定前后力差按实际动作
tick 数归一化，并用 EWMA 更新。正常 `ADJUST` 的夹紧与松开动作共用同一局部模型；只要力变化与
动作方向同号，二者都可成为有效样本，安全 `RELEASE` 样本始终排除：

\[
\widehat{\Delta F}_{tick,k}
=(1-\lambda)\widehat{\Delta F}_{tick,k-1}
+\lambda\frac{\bar F_k-\bar F_{k-1}}{\Delta u_k}。
\]

首批三个有效样本积累完成前只允许单 tick 调节。是否拆分正负方向模型由后续数据中的系统性差异决定，
第一版不预先增加模型复杂度。

## 控制器消融

| 变体 | 命令空间 | 死区 | 一步预测 | 动态步长 |
| --- | --- | --- | --- | --- |
| `quantized-pi` | 整数 | 固定 | 否 | PI 内部连续计算，输出统一量化 |
| `fixed-step` | 整数 | 固定 | 否 | 固定 ±1 |
| `adaptive-deadband` | 整数 | `ΔF_tick` + 噪声 | 否 | 固定 ±1 |
| `predictive` | 整数 | 自适应 | 是 | 固定 ±1 |
| `dynamic-step` | 整数 | 自适应 | 是 | 正常调节 0、±1～±3；安全释放 -1 |

自适应变体使用：

\[
\epsilon_{hold}=\max(c_\sigma\sigma_F,\tfrac12\widehat{\Delta F}_{tick}),
\qquad
\epsilon_{react}=\epsilon_{hold}+\beta\widehat{\Delta F}_{tick}。
\]

`HOLD` 中的偏差必须连续超过再激活阈值 0.20 s，才会返回 `ADJUST`。预测变体显式比较
`{-1, 0, +1}`；动态变体在模型可靠后比较 `{-3, -2, -1, 0, +1, +2, +3}`，并先剔除越界和
不安全候选。候选代价由预测误差与随动作幅值增长的保守裕量组成；并列时优先较小动作，因此
`HOLD` 是真正参与最优选择的候选，而不是事后阈值补丁。

夹紧候选还要满足 `max_force_n - force_margin_n` 的安全预测；没有可靠局部估计时动作空间限制为
`{-1, 0, +1}`。旧任务中的 `dynamic_step.eta` 仅为配置兼容保留，不再决定正式算法的步长。

## 单次运行

```bash
uv run pgt run discrete-force \
  --experiment robotiq_2f85/discrete_force
```

CLI 使用可重复的 `--set` 选择控制器、材料、task 字段和 `seed`，例如
`--set controller=robotiq_2f85/predictive --set material=hard --set seed=1`。
接触前后均使用 30 Hz 控制时钟；在 500 Hz 物理步长上由仿真时间调度器交替落到相邻物理步，长期平均
周期严格保持为 1/30 s。接近阶段另受 50 ms 动作间隔限制，因此接近动作频率不超过 20 Hz。
主任务在接触稳定后启动 `2→4→6→8→6→4→2 N` 目标曲线，首个平台为在线辨识保留 2.5 s，
其余目标包含 1.5～2.0 s 平台，相邻
平台用 1.0 s 线性过渡；安全上限为 10 N。这样一次运行即可验证 HOLD 再激活、加载与卸载、局部增益
随压缩变化及动态步长，而无需为三个固定目标重复接近和标定。四种刚度使用显式触觉球—方块接触
pair；在 `u=255` 的当前模型中，稳定可达力约为 9.69、11.89、19.12 和 31.69 N。接触本身仍具有 MuJoCo
`solimp` 非线性，可用于观察局部增益随压缩程度变化，但这些档位不对应真实材料杨氏模量。

每次运行保存 `task.yaml`、`effective_parameters.json`、`trace.csv.gz`、`metrics.json`、600 DPI PNG 和
manifest。常规 trace 由独立 `record_period_s` 定时器采样，默认 100 Hz 并使用 gzip 压缩；
动作生效、动作重新稳定、状态切换及首尾行强制保留。仿真不会为每个 500 Hz 物理步构造或保存完整记录；
安全超限、峰值和有限性等必须覆盖物理步的量以在线标量方式累计，绘图使用事件增强 trace，RMSE、
HOLD 占比与逐平台统计仍使用均匀的 30 Hz 控制周期样本，避免记录频率或额外事件行改变统计权重。
trace 除双侧力、原始/含噪/滤波力、状态、动作、稳定性、局部增益、阈值和预测量外，还保存请求位置
`u_request`、实际写入命令 `u_actual_command`、等价机械位置 `p_actual`、位置残差，以及稳定动作的
`Δu/Δp/Δe_position/ΔF`、`rho_p`、模型有效性和每个有限候选的代价。

指标按方案的优先级同时报告安全超限及持续时间、RELEASE 次数、动作次数、总移动量、平均非零步长、
反转、相邻位置振荡、稳定时间、HOLD 占比、峰值力、峰值过冲、稳态 MAE、RMSE、预测 MAE、平均实测
单 tick 增益和平均位置响应比。稳态 MAE 对末段逐点绝对误差取平均，不允许目标两侧的振荡误差互相
抵消。七个平台另写入逐平台误差、动作、反转、相邻位置振荡、HOLD 占比与
稳定判定。`passed` 要求仿真有限、未超过安全力、建立接触、所有平台均形成至少 0.5 s 的持续 HOLD，
且整条命令轨迹为整数。PI 的积分器和比例项可以保留小数，但其最终 actuator 命令与其他控制器共用
`round + clip(0, 255)` 执行接口；动作次数、总移动量和反转次数均按实际发生的整数命令变化统计。

## 完整 study

```bash
uv run python scripts/research/study.py \
  research=robotiq_discrete_force_validation/study
uv run python scripts/research/study.py \
  research=robotiq_discrete_force_validation/study \
  execution=study_run
```

正式 study 统一串行执行，不提供并行度参数。条件之间相互独立且种子固定，串行结果与历史
并行逐字节一致。

默认矩阵为 5 个控制器 × 4 种刚度 × 3 种噪声，共 60 个条件。每个条件均运行完整主曲线。study
同时输出逐次、逐平台和聚合 CSV/Parquet、JSON 摘要、控制器总览/消融链/逐平台比较 PNG、
子 run 目录及 study manifest。增加噪声重复时，
只需提高 study 中 `seeds.count`，矩阵展开和聚合逻辑不变。

当前参数是 MuJoCo 验证起点，不是实机标定结果。迁移硬件前必须重新确认命令延迟、传感器噪声、稳定
窗口、力安全上限与局部 tick 增益范围。

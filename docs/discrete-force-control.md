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
小于 `0.10 N`，且动作后至少经过 `0.20 s`，才接受新的稳定力。夹紧动作的稳定前后力差按实际动作
tick 数归一化，并用 EWMA 更新：

\[
\widehat{\Delta F}_{tick,k}
=(1-\lambda)\widehat{\Delta F}_{tick,k-1}
+\lambda\frac{\bar F_k-\bar F_{k-1}}{\Delta u_k}。
\]

首批三个有效样本积累完成前只允许单 tick 调节。释放动作不更新第一版的夹紧方向局部模型。

## 控制器消融

| 变体 | 命令空间 | 死区 | 一步预测 | 动态步长 |
| --- | --- | --- | --- | --- |
| `quantized-pi` | 整数 | 固定 | 否 | PI 内部连续计算，输出统一量化 |
| `fixed-step` | 整数 | 固定 | 否 | 固定 ±1 |
| `adaptive-deadband` | 整数 | `ΔF_tick` + 噪声 | 否 | 固定 ±1 |
| `predictive` | 整数 | 自适应 | 是 | 固定 ±1 |
| `dynamic-step` | 整数 | 自适应 | 是 | 夹紧 1～3、释放 1 |

自适应变体使用：

\[
\epsilon_{hold}=\max(c_\sigma\sigma_F,\alpha\widehat{\Delta F}_{tick}),
\qquad
\epsilon_{react}=\epsilon_{hold}+\beta\widehat{\Delta F}_{tick}。
\]

`HOLD` 中的偏差必须连续超过再激活阈值 0.20 s，才会返回 `ADJUST`。预测变体仅在预测误差连同
噪声/tick 裕量严格小于当前误差时动作。动态步长为：

\[
n=\operatorname{clip}\left(\left\lfloor
\eta\frac{|e|}{\widehat{\Delta F}_{tick}}\right\rfloor,1,3\right)。
\]

夹紧前还会按 `max_force_n - force_margin_n` 计算安全步长；没有可靠局部估计时只允许单 tick。

## 单次运行

```bash
uv run pgt run discrete-force \
  --profile configs/robotiq_2f85.yaml \
  --task configs/discrete_force/robotiq_delta_f_tick.yaml
```

CLI 可覆盖 `--controller-variant`、`--object-material`、`--force-noise-std` 和 `--noise-seed`。
接触前后均使用 30 Hz 控制时钟；在 500 Hz 物理步长上由仿真时间调度器交替落到相邻物理步，长期平均
周期严格保持为 1/30 s。接近阶段另受 50 ms 动作间隔限制，因此接近动作频率不超过 20 Hz。
主任务在接触稳定后启动 `2→4→6→8→6→4→2 N` 目标曲线，首个平台为在线辨识保留 2.5 s，
其余目标包含 1.5～2.0 s 平台，相邻
平台用 1.0 s 线性过渡；安全上限为 10 N。这样一次运行即可验证 HOLD 再激活、加载与卸载、局部增益
随压缩变化及动态步长，而无需为三个固定目标重复接近和标定。四种刚度使用显式触觉球—方块接触
pair；在 `u=255` 的当前模型中，稳定可达力约为 9.69、11.89、19.12 和 31.69 N。接触本身仍具有 MuJoCo
`solimp` 非线性，可用于观察局部增益随压缩程度变化，但这些档位不对应真实材料杨氏模量。

每次运行保存 `task.yaml`、`effective_parameters.json`、`trace.csv.gz`、`metrics.json`、600 DPI PNG、
矢量 PDF 和 manifest。trace 按控制周期（30 Hz）采样并 gzip 压缩，动作生效行强制保留；
指标与逐平台判定仍基于全速率物理步序列计算，不受采样影响。trace 包含双侧力、原始/含噪/滤波力、
状态、实际与请求动作、稳定性、`ΔF_tick`、自适应阈值、预测量、动作数和反转数。

指标按方案的优先级同时报告安全超限、动作次数、总移动量、反转、相邻位置振荡、稳定时间、HOLD
占比、峰值力、稳态 MAE、RMSE 和预测 MAE。稳态 MAE 对末段逐点绝对误差取平均，不允许目标两侧的
振荡误差互相抵消。七个平台另写入逐平台误差、动作、反转、HOLD 占比与
稳定判定。`passed` 要求仿真有限、未超过安全力、建立接触、所有平台均形成至少 0.5 s 的持续 HOLD，
且整条命令轨迹为整数。PI 的积分器和比例项可以保留小数，但其最终 actuator 命令与其他控制器共用
`round + clip(0, 255)` 执行接口；动作次数、总移动量和反转次数均按实际发生的整数命令变化统计。

## 完整 study

```bash
uv run python scripts/experiments/robotiq_discrete_force.py \
  --config configs/studies/robotiq_discrete_force.yaml \
  --dry-run
uv run python scripts/experiments/robotiq_discrete_force.py \
  --config configs/studies/robotiq_discrete_force.yaml
```

`--jobs N` 可用 N 个进程并行执行条件（默认 1 为串行）。条件之间相互独立且种子固定，
并行结果与串行逐字节一致；条件耗时以本机绘图与仿真为主，60 条件矩阵在本机 12 进程约 33 s。

默认矩阵为 5 个控制器 × 4 种刚度 × 3 种噪声，共 60 个条件。每个条件均运行完整主曲线。study
同时输出逐次、逐平台和聚合 CSV/Parquet、JSON 摘要、控制器总览/消融链/逐平台比较图 PDF/PNG、
子 run 目录及 study manifest。增加噪声重复时，
只需提高 study 中 `seeds.count`，矩阵展开和聚合逻辑不变。

当前参数是 MuJoCo 验证起点，不是实机标定结果。迁移硬件前必须重新确认命令延迟、传感器噪声、稳定
窗口、力安全上限与局部 tick 增益范围。

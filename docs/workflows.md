# 🚀 常用工作流

单次运行使用 `pgt`；需要固定条件矩阵、重复试验和聚合统计时使用 `scripts/experiments`。
两类入口共享 Python runner 与产物格式。人工编写的 profile、task 与 study 配置均为 YAML，并在模型编译前完成校验。

## 单次运行与交互检查

<pre><code class="language-bash">
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt assets generate-taxels --shape box
uv run pgt assets generate-touch-grid
uv run pgt run demo --profile configs/robotiq_2f85_touch_grid.yaml
uv run pgt compare tactile \
  --left-profile configs/robotiq_2f85_box.yaml \
  --right-profile configs/robotiq_2f85_touch_grid.yaml
uv run pgt run grasp --profile configs/custom_parallel_gripper.yaml --video
uv run pgt run force-track \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_tracking/default_waypoints.yaml
uv run pgt run force-schedule \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_scheduling/gravity_hold.yaml
uv run pgt run force-schedule \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_scheduling/dynamic_filling.yaml
uv run pgt run friction-estimate \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/friction_estimation/nominal_friction.yaml
uv run pgt compare contact --profile configs/custom_parallel_gripper.yaml
</code></pre>

`--disable-multiccd` 是保留原碰撞模型、仅限制 convex geom pair 接触数的诊断开关；默认 profile 已使用
稳定的共面球体碰撞近似，常规实验无需添加该开关。

`force-schedule` 复用法向力控制器，根据切向载荷和 task 中已知的摩擦系数生成平均单侧目标力。
`gravity_hold.yaml` 只验证撤去支撑后的重力保持；`dynamic_filling.yaml` 以沿重力方向的 0→2 N
附加力模拟注水。当前实现是读取真值 `μ` 的 oracle 基线，不包含在线摩擦系数估计。两个 task 均显式
设置 `noslip_iterations=5` 以抑制摩擦锥内的长时数值爬移；目标力固定为不足的 `0.5 N/侧` 时仍会
滑落，因此该求解设置不会掩盖摩擦容量不足。公式、task 字段、输出列和当前验收结果见
[Oracle 抓取目标力调度](force-scheduling.md)。

`friction-estimate` 在固定预抓取力下沿世界 `+Y` 缓慢加载，用逐 taxel 三轴触觉合力的摩擦比饱和
与载荷—支撑失配检测力域初始滑移。估计器不读取真实 `μ`、物体位移或速度；确认后冻结保守
`μ` 下界并驱动同一个目标力调度器。低、中、高摩擦和两倍噪声 task、回退语义及当前结果见
[微滑移探测与保守摩擦估计](friction-estimation.md)。

## 多条件研究

控制器 × 材料 × 噪声种子的消融 protocol：

<pre><code class="language-bash">
uv run python scripts/experiments/force_tracking_ablation.py \
  --config configs/studies/force_tracking_ablation.yaml
</code></pre>

跨三类目标曲线的规范化控制器对比先审阅矩阵，再运行完整 protocol：

<pre><code class="language-bash">
uv run python scripts/experiments/force_tracking_controller_comparison.py \
  --config configs/studies/force_tracking_controller_comparison.yaml \
  --dry-run
uv run python scripts/experiments/force_tracking_controller_comparison.py \
  --config configs/studies/force_tracking_controller_comparison.yaml
</code></pre>

默认矩阵为 6 个控制器变体 × 3 个任务 × 3 个正式接触 preset × 3 个 seed，共 162 个条件；其中包含
四个 PID 系变体、`direct-torque` 与二阶直接力矩 `adrc-torque`。一阶位置式 `adrc` 因控制导向模型
阶次不匹配而保留为历史复现入口，不再参加默认正式对比。三个 preset 为
`medium=(-650,-8)`、`hard=(-1200,-10)` 与 `stiff=(-2500,-15)`。原
`soft=(-250,-5)` 不进入默认矩阵。study 完成后会同时输出 `summary.csv`、`summary.parquet`，以及适用时的
`aggregate.csv`、`aggregate.parquet`；diagnosis study 只有 summary。图仍保存在 `figures/` 中的 PNG 与 PDF。

图表按 study 的科学问题组织：控制器对比展示误差、饱和、相对 Full 增量和同 seed 轨迹；PID 消融展示
材料分组指标以及完整 2×2 配对的主效应/交互作用；ADRC 调参展示候选排序、约束可行域和参数—性能关系；
刚度估计器对比展示相对 secant 的增量和力/刚度轨迹；因果诊断展示扫描变量—诊断指标曲线与有效轨迹叠加。
所有 study 图同时输出 600 DPI PNG 和矢量 PDF。

二阶直接力矩 ADRC 的测量轻滤波和控制/观测器带宽采用两阶段调参：粗扫先固定 `medium` 与一个 seed，
确认阶段再在三种 preset 与三个 seed 上复验。确认阶段读取粗扫目录中的可行候选排名：

<pre><code class="language-bash">
uv run python scripts/experiments/force_tracking_torque_adrc_tuning.py \
  --config configs/studies/force_tracking_torque_adrc_tuning.yaml \
  --stage coarse \
  --dry-run
uv run python scripts/experiments/force_tracking_torque_adrc_tuning.py \
  --config configs/studies/force_tracking_torque_adrc_tuning.yaml \
  --stage coarse
uv run python scripts/experiments/force_tracking_torque_adrc_tuning.py \
  --config configs/studies/force_tracking_torque_adrc_tuning.yaml \
  --stage confirm \
  --coarse-study-dir outputs/studies/force_tracking_torque_adrc_tuning/coarse/&lt;粗扫目录&gt;
</code></pre>

按阶段执行因果诊断；碰撞几何阶段包含原 mesh、两种球体、共面 mesh 和关闭 `multiccd` 五个条件：

<pre><code class="language-bash">
uv run python scripts/experiments/force_tracking_diagnosis.py \
  --config configs/studies/force_tracking_diagnosis.yaml \
  --phase collision-geometry
</code></pre>

研究脚本直接调用 `parallel_gripper_tactile.runners.execute_force_tracking`，不会启动 CLI 子进程。
每个条件生成独立 run。study 父目录同时保存人工输入 `study.yaml` 和路径、默认值均已解析的
`study.resolved.json`。force-track run 同时保留 YAML 输入快照，并写入包含完整解析 profile、task 和实际
运行时覆盖的 `effective_parameters.json`。时序数据默认以 Zstd 压缩的 `trace.parquet` 保存：普通控制器
常规区段为 100 Hz，直接力矩 ADRC 为 250 Hz；阶段/控制状态/限幅状态变化以及 waypoint 前后 0.2 s
保留完整控制频率。指标和图像使用未降采样数据。旧 CSV API 与历史 CSV 产物仍兼容读取。study 父目录另存 `study.yaml` 和逐次 summary；
消融和控制器对比研究还生成 CSV 与 Parquet 两种聚合统计，控制器对比图统一登记到 `study_manifest.json`。

动态目标力跟踪任务的配置、两阶段流程和指标解读见[动态目标力跟踪](force-tracking.md)。
[Oracle 抓取目标力调度](force-scheduling.md)说明已知摩擦系数下的目标力调度基线。
控制算法对比、消融矩阵和项目分工见[控制算法对比与消融](control-comparison-ablation.md)。
碰撞几何对照的结论和使用边界见[触觉读数约定](tactile-conventions.md#collision-geometry-conclusions)。

用 `pgt runs list` 查看既有产物。用 `pgt runs clean --all` 预览要删除的目标；确认目标后
再加 `--apply`。

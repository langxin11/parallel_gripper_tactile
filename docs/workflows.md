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
uv run pgt run discrete-force \
  --profile configs/robotiq_2f85.yaml \
  --task configs/discrete_force/robotiq_delta_f_tick.yaml
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

`friction-estimate` 在固定预抓取力下沿世界 `+Y` 缓慢加载，仅从逐 taxel 三轴触觉的利用率趋势、
空间重分布和双侧不对称性生成接触变化评分。检测器不读取外部载荷、真实 `μ`、位移或速度；
确认后冻结摩擦候选，保持阶段根据实测切向力调度法向目标。当前结果和能力边界见
[微滑移探测与保守摩擦估计](friction-estimation.md)。
运行同时输出经过接触滞回筛选的逐 taxel 局部摩擦利用率，以及左右触觉面的峰值分布图；这些局部量
当前用于诊断和验证，不参与目标力计算。
`hardware_scale_nominal.yaml` 另以目标传感器单 taxel 的 `0.05 N` 标称分辨率为依据，采用
`0.5 N/0.25 N` 接触滞回阈值和 `4 N/侧` 预载；原有 `0.05 N/0.025 N` 仅保留为低力仿真基线。

纯力局部起滑的多种子正例与低探测载荷负例 study：

<pre><code class="language-bash">
uv run python scripts/experiments/friction_estimation_local_slip.py \
  --config configs/studies/friction_estimation_local_slip.yaml
</code></pre>

study 输出逐次与聚合 CSV/Parquet、检测时刻与局部候选摩擦比图，以及每个条件的完整单次运行产物。
纯触觉总体检测改变了探测终止时刻，历史局部 study 通过率不能沿用，必须重新运行。
事件门槛检查正例检测和负例误报；控制候选门槛另要求局部估计/真值位于 `[0.55, 1.02]`，防止把
“检测到局部变化”误写成“局部摩擦估计已经可以接管目标力”。

`discrete-force` 使用 Robotiq 2F-85 的 `0～255` 整数 tendon 命令，在线估计稳定动作前后的
`ΔF_tick`，并驱动自适应 HOLD 死区、再激活滞回、一步预测、动态步长和安全释放。主任务在一次抓取
内运行 `2→4→6→8→6→4→2 N` 平台—过渡曲线；五种控制器、四种接触刚度和三个噪声等级组成
60 条件 study：

<pre><code class="language-bash">
uv run python scripts/experiments/robotiq_discrete_force.py \
  --config configs/studies/robotiq_discrete_force.yaml --dry-run
uv run python scripts/experiments/robotiq_discrete_force.py \
  --config configs/studies/robotiq_discrete_force.yaml
</code></pre>

完整矩阵默认自动并行，进程数取可用 CPU、条件数量与 12 的最小值；资源受限时可用 `--jobs 1`
强制串行，或用 `--jobs N` 指定进程数。单次运行中的物理、控制和记录时钟分别配置；默认是
500 Hz 物理、30 Hz 控制和 100 Hz 常规记录，并额外保留关键事件。

算法、trace 字段、验收口径和当前四档可达力见
[Robotiq 2F-85 离散力控制](discrete-force-control.md)。

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

## 实验报告与论文工作稿（Typst）

`reports/` 目录用 Typst 编写实验报告与论文工作稿。两者都是时点性交付物：头部记录所引用
run 的 ID 与 git 提交，数值全部程序化读取自 `outputs/` 产物，与 docs/ 只保留可复现定性
结论的约定互补；不回写 docs/。

在仓库根编译《摩擦感知目标力调度》论文工作稿：

<pre><code class="language-bash">
typst compile --root . reports/wired_demo.typ
</code></pre>

产物为 `reports/wired_demo.pdf`，不入库（`reports/*.pdf` 已加入 `.gitignore`）。
前置要求：Typst CLI ≥ 0.14（0.15.0 已验证）；Noto Serif/Sans CJK SC 简体中文字体，
缺字体渲染成方框但编译不报错；首次编译需联网下载 `@preview/mitex` 包，之后走本地缓存。

工作稿的数据源是 `friction-estimate` 与 `force-schedule` 两次运行的产物目录，路径写在
文件头部常量里。更换数据源时改头部的 `#let …-run = "/outputs/…"` 常量，并把新 run 的
`plot.pdf` 复制到 `reports/figures/` 替换入库快照（插图不直接引用 `outputs/`，图像资产
固定、不随 `pgt runs clean` 丢失）；产物不存在则先重跑对应实验（命令见上文）。

报告内的 LaTeX 公式经 `mitex` 兼容，Typst 字符串中反斜杠须双写（如 `"\\rho"`），否则
`\r`、`\t` 会被当转义符吃掉。`tests/test_report_typst.py` 用 `tests/fixtures/` 迷你数据
编译 fixture 报告做冒烟测试，本机装有 Typst CLI 时才执行、CI 无 CLI 环境自动跳过。
模板组件说明详见 [`reports/README.md`](../reports/README.md)。

## 演示视频录制

摩擦估计与 Ramp 力跟踪实验支持把 MuJoCo 场景和右侧实时曲线面板合成为
16:9 MP4 演示，适合组会口头报告。画面为论文式仪表盘：左侧为整机构视角的
MuJoCo 场景（摩擦演示叠加当前切向外加载荷箭头），右侧三行曲线
（MathText 数学符号，配色语义固定：灰虚线=目标、蓝=实测、橙=外部载荷、
绿=摩擦极限、紫=估计 μ̂、红虚线=真值/事件）带顶部窄 phase bar 与逐帧
同步的时间游标；左上 HUD 采用「实验名 / 状态 / 关键测量 / 辅助量」四层
结构。录制走实验循环的可选逐帧回调，只读快照、不产生实验 trace；需要
ffmpeg，无界面环境需在导入 mujoco 前设置 `MUJOCO_GL=egl`。

```bash
# 同时录制摩擦估计与 Ramp 力跟踪两个演示
uv run python scripts/demos/record_experiment_demos.py --demo both
# 只录制 Ramp 力跟踪，固定触觉噪声种子保证可复现
uv run python scripts/demos/record_experiment_demos.py --demo ramp --noise-seed 0 --fps 60
```

产物默认写入 `outputs/demos/force_tracking_ramp.mp4` 与
`outputs/demos/friction_estimation_with_curves.mp4`（均为 1600×900 @30 fps，
可通过 `--width/--height/--fps/--panel-width/--output-dir` 调整，不入库）。
每次录制还会把对应真实事件的关键帧导出到
`outputs/demos/preview/`（Ramp：start/mid/end；摩擦：滑移前/检测瞬间/
自适应增载后）供人工验收。录制器入口为
`parallel_gripper_tactile.experiments.demo_videos` 中的
`record_force_tracking_ramp_video` 与 `record_friction_demo_video`。

## 论文图导出

科研图默认使用 `science`、`ieee`、`no-latex` 组合，不需要外部 LaTeX。
字体使用 TeX Gyre Termes 和 Noto Serif CJK SC，运行环境应安装这两种字体，中文需检查缺字警告。
新增绘图入口复用 `plotstyle` 公共模块，以 `paper_figsize(height, columns=1)` 创建 3.5 英寸单栏图，
默认 `columns=2` 创建 7.16 英寸跨栏图；高度由面板数量确定。创建时启用约束布局，
通过 `save_publication_figure(figure, path)` 输出同名 PDF 和 600 DPI PNG。其他显式请求格式也会保留。
导出不使用紧边界裁切，以免改变最终栏宽。多面板长图仍需根据目标期刊页高拆分或安排到补充材料。
历史产物不会自动覆盖；需重新执行绘图才能应用新样式。交付前按最终尺寸检查中文、负号、图例与裁切。

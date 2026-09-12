# 🚀 常用工作流

演示、查看和设备检查使用 `pgt`；科研组合、探索运行与正式研究统一使用 `scripts/research`。
旧的控制器对比、PID 消融与 Torque ADRC 调参研究脚本已经删除，当前请使用对应的 Hydra study 入口。
所有入口共享 Python runner 与产物格式，不会通过子进程调用 `pgt`。

## 单次运行与交互检查

<pre><code class="language-bash">
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt assets generate-taxels --shape box
uv run pgt assets generate-touch-grid
uv run pgt run demo --set model=robotiq_2f85/touch_grid_3x3
uv run pgt compare tactile \
  --left-set model=robotiq_2f85/box_force_sensor \
  --right-set model=robotiq_2f85/touch_grid_3x3
uv run pgt run grasp --video
uv run pgt run force-track --set task=force_tracking/default_waypoints
uv run pgt run force-schedule --experiment dm_gripper/force_scheduling_gravity_hold
uv run pgt run force-schedule --experiment dm_gripper/force_scheduling_dynamic_filling
uv run pgt run friction-estimate --experiment dm_gripper/friction_estimation_nominal
uv run pgt run discrete-force --experiment robotiq_2f85/discrete_force
uv run pgt compare contact
</code></pre>

`--set execution.multiccd_enabled=false` 是保留所选碰撞模型、仅限制 convex geom pair 接触数的诊断
覆盖；默认 model 已使用稳定的高度球体碰撞近似，常规实验无需添加该覆盖。

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

纯力局部起滑的多种子正例与低探测载荷负例 study（默认先生成计划，`execution=study_run` 执行）：

<pre><code class="language-bash">
uv run python scripts/research/study.py \
  research=friction_local_slip_validation/study
uv run python scripts/research/study.py \
  research=friction_local_slip_validation/study \
  execution=study_run
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
uv run python scripts/research/study.py \
  research=robotiq_discrete_force_validation/study
uv run python scripts/research/study.py \
  research=robotiq_discrete_force_validation/study \
  execution=study_run
</code></pre>

正式 study 可用 `execution.workers=N` 在 CPU 上并行执行独立条件。单次运行中的物理、控制和记录时钟分别配置；默认是
500 Hz 物理、30 Hz 控制和 100 Hz 常规记录，并额外保留关键事件。

算法、trace 字段、验收口径和当前四档可达力见
[Robotiq 2F-85 离散力控制](discrete-force-control.md)。

## Hydra 科研运行

科研依赖随完整开发环境安装：

<pre><code class="language-bash">
uv sync --all-packages --all-groups --locked
</code></pre>

组合并执行一个确定的 DM 力跟踪实验：

<pre><code class="language-bash">
uv run python scripts/research/run.py \
  controller=dm_gripper/adrc_torque \
  estimator=window_linear \
  task=force_tracking/ramp \
  material=hard \
  seed=0
</code></pre>

将 `execution=plan` 加入同一命令会执行完整领域校验、scene 编译并保存计划，但不推进仿真。
DM 共享导纳使用 `experiment=dm_gripper/force_tracking_admittance`。探索性组合可使用原生 Multirun：

<pre><code class="language-bash">
uv run python scripts/research/run.py -m \
  material=medium,hard,stiff \
  seed=0,1,2
</code></pre>

正式控制器对比、PID 模块消融、Torque ADRC 两阶段调参、局部起滑、刚度估计器对比、DM 导纳调参、
Robotiq 离散力与因果诊断由 study 自身展开权威 YAML 中的矩阵；默认只生成计划，显式选择
`execution=study_run` 才会执行。除下表 preset 外，`friction_estimation_local_slip`、
`force_tracking_stiffness_estimator_comparison`、`dm_admittance_tuning`、`robotiq_discrete_force`
与 `force_tracking_diagnosis`（须再指定 `study.phase=<phase>`）用法相同：

<pre><code class="language-bash">
uv run python scripts/research/study.py \
  research=force_controller_selection/study
uv run python scripts/research/study.py \
  research=force_controller_selection/study \
  execution=study_run
uv run python scripts/research/study.py \
  research=force_controller_ablation/study
uv run python scripts/research/study.py \
  research=force_controller_ablation/study \
  execution=study_run
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study study.stage=confirm \
  study.coarse_study_dir=/absolute/path/to/coarse-study
</code></pre>

study 入口拒绝外层 `-m`，从而保证计划、配对统计与实际执行只展开一次。配置组、覆盖优先级、列表
替换、非法组合、路径与产物语义见 [Hydra 科研配置与实验编排](research-configuration.md)。

`execution.workers` 默认为 `1`，大于 `1` 时使用 `spawn` 启动条件级 CPU 进程。每个 worker 独占一个
run 目录，只负责 MuJoCo 仿真和条件指标；父进程按计划顺序更新 `study_manifest.json`，并在所有条件结束后
统一聚合和绘图。并行完成顺序不会改变 `condition_results`、配对关系或统计输入顺序。建议先比较
`N=1,4,8` 的耗时与内存占用，不要直接使用全部逻辑核。

## 推荐的正式研究执行顺序

新一轮完整研究建议逐项执行并在阶段之间审查产物，不要把全部命令串联为一个长任务。下列命令均为
实际执行；去掉末尾的 `execution=study_run` 即为只生成计划。计划模式不是程序上的必经步骤，但正式运行前
应先用相同参数生成并审阅计划。

第一阶段验证基础组件和控制结构。默认使用 `window_linear`；历史估计器比较只表示控制器执行指标，
不作为主线前置条件。先检查准静态参考的量级，再依次运行位置限幅和速率控制结构验证，最后执行 PID 模块消融：

```bash
uv run python scripts/research/study.py \
  research=stiffness_ground_truth_validation/study \
  execution=study_run \
  execution.workers=8
uv run python scripts/research/study.py \
  research=force_tracking_stiffness_limit_pilot/study \
  execution=study_run \
  execution.workers=8
uv run python scripts/research/study.py \
  research=force_tracking_stiffness_rate_validation/study \
  execution=study_run \
  execution.workers=8
uv run python scripts/research/study.py \
  research=force_controller_ablation/study \
  execution=study_run
```

第二阶段先调优并确认刚度速率控制器，再调优二阶直接力矩 ADRC。刚度速率确认固定调优排名第一的
\(K_P=30\ \mathrm{s^{-1}}\)、\(\dot F_{\max}=70\ \mathrm{N/s}\)，以 54 条条件检查频率和材料泛化；
ADRC 的 `confirm` 是唯一具有强制谱系依赖的阶段，必须使用已完成 coarse 后终端打印的绝对目录：

```bash
uv run python scripts/research/study.py \
  research=force_tracking_stiffness_rate_tuning/study \
  execution=study_run \
  execution.workers=8
uv run python scripts/research/study.py \
  research=force_tracking_stiffness_rate_refinement/study \
  execution=study_run \
  execution.workers=8
uv run python scripts/research/study.py \
  research=force_tracking_stiffness_rate_confirmation/study \
  execution=study_run \
  execution.workers=8
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study \
  execution=study_run

COARSE_DIR=/absolute/path/to/completed-coarse-study
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study \
  study.stage=confirm \
  study.coarse_study_dir="$COARSE_DIR" \
  execution=study_run
```

共享导纳调优不进入默认 PID／ADRC 选型矩阵，可作为独立基线在同一阶段运行：

```bash
uv run python scripts/research/study.py \
  research=dm_admittance_tuning/study \
  execution=study_run
```

速率调优只负责筛选候选，确认结果通过后才能冻结到最终比较。完成上述研究后必须设置人工决策门：
检查 `study_manifest.json` 的生命周期状态、科学失败与执行异常，
审阅 `summary.csv`、聚合表、候选排名和图表。若最优估计器或控制参数不同于当前 YAML，应先更新配置、
测试和文档并形成提交；Study 不会自动把最优参数传给下一项研究。配置冻结后再运行最终控制器比较：

```bash
uv run python scripts/research/study.py \
  research=force_controller_selection/study \
  execution=study_run
```

局部起滑和 Robotiq 离散力不阻塞 DM 控制器选型，最后按各自研究问题独立运行：

```bash
uv run python scripts/research/study.py \
  research=friction_local_slip_validation/study \
  execution=study_run
uv run python scripts/research/study.py \
  research=robotiq_discrete_force_validation/study \
  execution=study_run
```

所有正式产物写入 `outputs/research/studies/`。旧 `outputs/studies/` 只包含兼容时期的历史结果，不是当前
Hydra study 入口的输出位置。归档的模型 bug 诊断不属于上述路线，只在复现历史接触问题时显式运行。

## 多条件研究与产物约定

默认矩阵为 6 个控制器变体 × 3 个任务 × 3 个正式接触 preset × 3 个 seed，共 162 个条件；其中包含
四个 PID 2×2 变体、`pid-stiffness-rate` 与二阶直接力矩 `adrc-torque`。`pid-stiffness-limit` 因
stiff Step 平台极限环退出最终矩阵，只保留专项复现；`direct-torque` 和一阶位置式
`adrc` 均保留为独立/历史复现入口，不参加当前默认正式对比；后者因控制导向模型
阶次不匹配而保留为历史复现入口，不再参加默认正式对比。三个 preset 为
`medium=(-650,-8)`、`hard=(-1200,-10)` 与 `stiff=(-2500,-15)`。原
`soft=(-250,-5)` 不进入默认矩阵。study 完成后会同时输出 `summary.csv`、`summary.parquet`，以及适用时的
`aggregate.csv`、`aggregate.parquet`；diagnosis study 只有 summary。图默认以 PNG 保存在 `figures/` 中。

图表按 study 的科学问题组织：控制器对比展示误差、饱和、相对 Full 增量和同 seed 轨迹；速率调优展示
\(K_P\)、\(\dot F_{\max}\) 与 RMSE、\(\sigma_F\)、超调之间的热图；PID 消融展示
材料分组指标以及完整 2×2 配对的主效应/交互作用；ADRC 调参展示候选排序、约束可行域和参数—性能关系；
刚度估计器对比展示相对 secant 的增量和力/刚度轨迹；因果诊断展示扫描变量—诊断指标曲线与有效轨迹叠加。
所有 study 图默认输出一份 600 DPI PNG。

二阶直接力矩 ADRC 的测量轻滤波和控制／观测器带宽采用两阶段调参：粗扫先固定 `medium` 与一个 seed，
确认阶段再在三种 preset 与三个 seed 上复验。计划、coarse 执行与 confirm 都使用 Hydra 正式入口：

<pre><code class="language-bash">
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study execution=study_run
uv run python scripts/research/study.py \
  research=torque_adrc_tuning/study study.stage=confirm \
  study.coarse_study_dir=/absolute/path/to/coarse-study execution=study_run
</code></pre>

因果诊断按单 phase 一次调用执行，替代原 `--phase all` 循环；碰撞几何阶段包含原 mesh、两种球体、
共面 mesh 和关闭 `multiccd` 五个条件：

<pre><code class="language-bash">
uv run python scripts/research/study.py \
  research=archive/model_bug_diagnosis/study \
  study.phase=collision-geometry
uv run python scripts/research/study.py \
  research=archive/model_bug_diagnosis/study \
  study.phase=collision-geometry \
  execution=study_run
for phase in reproducibility controllers materials force-scale contact-model \
            collision-geometry force-semantics position-limit integral-gain filter-cutoff; do
  uv run python scripts/research/study.py \
    research=archive/model_bug_diagnosis/study \
    "study.phase=${phase}" execution=study_run
done
</code></pre>

所有研究 protocol 直接调用 `parallel_gripper_tactile.runners` 中的 runner，不会启动 CLI 子进程。
每个条件生成独立 run。study 父目录同时保存人工输入 `study.yaml` 和路径、默认值均已解析的
`study.resolved.json`。force-track run 同时保留 YAML 输入快照，并写入包含完整解析 profile、task 和实际
运行时覆盖的 `effective_parameters.json`。时序数据默认以 Zstd 压缩的 `trace.parquet` 保存：普通控制器
常规区段为 100 Hz，直接力矩 ADRC 为 250 Hz；阶段/控制状态/限幅状态变化以及 waypoint 前后 0.2 s
保留完整控制频率。指标和图像使用未降采样数据。旧 CSV API 与历史 CSV 产物仍兼容读取。study 父目录另存 `study.yaml` 和逐次 summary；
消融、控制器对比和 Torque ADRC 研究生成相应 CSV／Parquet 聚合与研究专属图，并统一登记到
`study_manifest.json`。manifest 还记录 `planned/running/partial/completed/failed` 状态、三类条件结果、
科学配置哈希和产物摘要；`execution.recovery_source` 只生成可恢复性报告，本阶段不会自动续跑。

动态目标力跟踪任务的配置、两阶段流程和指标解读见[动态目标力跟踪](force-tracking.md)。
[force-track](force-tracking.md) 单次运行默认登记 `plots/tracking.png`、`plots/tactile.png` 和
`plots/controller.png`；既有其他实验的 `plot.png` 产物约定不变。需要从已有目录重绘时使用：

<pre><code class="language-bash">
uv run python scripts/research/render.py &lt;run-directory&gt; [--tactile-detail] \
  [--format png|pdf|both] [--output-dir 新目录]
</code></pre>

重绘读取 Parquet（缺失时回退 CSV），指标沿用原 `metrics.json`，默认写入独占的
`plots/replots/&lt;UTC&gt;-&lt;id&gt;/` 并登记独立 `rendering_manifest.json`；不会覆盖已有输出目录，也不修改原
trace、metrics 或 run manifest。PDF 仅在显式请求时生成。
[Oracle 抓取目标力调度](force-scheduling.md)说明已知摩擦系数下的目标力调度基线。
控制算法对比、消融矩阵和项目分工见[控制算法对比与消融](control-comparison-ablation.md)。
碰撞几何对照的结论和使用边界见[触觉读数约定](tactile-conventions.md#collision-geometry-conclusions)。

用 `pgt runs list` 查看既有产物。用 `pgt runs clean --all` 预览要删除的目标；确认目标后
再加 `--apply`。

## 实验报告与论文工作稿（Typst）

`reports/` 目录用 Typst 编写实验报告与论文工作稿。报告按科研问题组织证据，运行编号与 Git 提交保留在
研究产物中，不进入正文。数值先由 Python 校验、汇总并冻结，Typst 编译不直接读取 `outputs/`；`docs/`
只保留已经稳定的方法、接口与适用边界，报告结论经人工决策后才进入文档和配置。

在仓库根编译报告总目录、文献指南、近期实验和既有研究合集：

<pre><code class="language-bash">
typst compile --root . reports/index.typ
typst compile --root . reports/literature.typ
typst compile --root . reports/experiments.typ
typst compile --root . reports/combined.typ
</code></pre>

产物与各入口同名，从 `reports/index.pdf` 可打开其他三份 PDF；预览不入库（`reports/*.pdf` 已加入 `.gitignore`）。
前置要求：Typst CLI ≥ 0.14（0.15.0 已验证）；Noto Serif/Sans CJK SC 简体中文字体，
缺字体渲染成方框但编译不报错；首次编译需联网下载 `@preview/mitex` 包，之后走本地缓存。

数值先通过 `scripts/reports/study_results_data.py` 校验并冻结到合集数据块，插图固定在
`reports/figures/`。原独立报告、论文工作稿与 Markdown 初步证据已并入合集，论文工作稿编译期
直读的数字也已冻结为字面量；新增报告内容一律使用字面量数据块。

报告内的 LaTeX 公式经 `mitex` 兼容，Typst 字符串中反斜杠须双写（如 `"\\rho"`），否则
`\r`、`\t` 会被当转义符吃掉。`tests/test_report_typst.py` 用 `tests/fixtures/` 迷你数据
编译 fixture 报告做冒烟测试，本机装有 Typst CLI 时才执行、CI 无 CLI 环境自动跳过。
报告正文统一使用 `.typ`，文献和近期实验章节分别放在 `reports/chapters/literature/` 与
`reports/chapters/experiments/`。目录、迁移关系与模板组件详见[报告维护说明](reports.md)。

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
`outputs/demos/friction_estimation_with_curves.mp4`（均默认为 1920×1080 @30 fps，
使用 H.264 CRF 18 编码；可通过 `--width/--height/--fps/--panel-width/--output-dir`
调整，不入库）。右侧实时面板调用 `plotstyle.science_pyplot()`，与论文图共用
SciencePlots IEEE 样式及字体配置。
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
通过 `save_publication_figure(figure, path)` 按 `path` 的扩展名输出一份图像；默认实验图为 600 DPI PNG。
导出不使用紧边界裁切，以免改变最终栏宽。多面板长图仍需根据目标期刊页高拆分或安排到补充材料。
历史产物不会自动覆盖；需重新执行绘图才能应用新样式。交付前按最终尺寸检查中文、负号、图例与裁切。

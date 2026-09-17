# 🚀 常用工作流

`pgt` 用于演示与检查；`scripts/research/run.py` 用于组合实验；`study.py` 用于正式研究。
环境与门禁见[测试策略](testing.md)，配置契约见[科研配置规范](research-configuration.md)。

## 单次运行与交互检查

先从 [Robotiq 2F-85](grippers/robotiq-2f85/index.md) 或 [DMgripper](grippers/dmgripper/index.md)
确认支持的模型与控制方式。两种夹爪使用不同执行器语义。

### Robotiq：触觉演示与对照

```bash
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt configs list
uv run pgt run demo --set model=robotiq_2f85/touch_grid_3x3
uv run pgt compare tactile --left-set model=robotiq_2f85/box_force_sensor --right-set model=robotiq_2f85/touch_grid_3x3
```

### DMgripper：抓取与力跟踪

```bash
uv run pgt run grasp --video
uv run pgt run force-track --set task=force_tracking/default_waypoints
```

其他实验按主题进入：[自适应抓取](adaptive-grasping.md)、[离散力控制](discrete-force-control.md)。
资产生成与导出见[资产维护约定](architecture.md#asset-maintenance)及各模型页面；运行参数与管理子命令可用
`uv run pgt --help` 逐级查看。
`execution.multiccd_enabled=false` 仅用于接触数量诊断，常规实验无需添加。

## Hydra 科研运行

```bash
uv run python scripts/research/run.py \
  controller=dm_gripper/adrc_torque estimator=window_linear \
  task=force_tracking/ramp material=hard seed=0 execution=plan
```

去掉 `execution=plan` 即执行；计划会校验领域配置并编译 scene，但不推进仿真。
共享导纳选择 `experiment=dm_gripper/force_tracking_admittance`。
探索性组合可使用 `run.py -m material=medium,hard,stiff seed=0,1,2`；
切向扰动支持单次和 Multirun，尚无正式 study。

正式 study 使用统一命令，把 `research` 替换为下表中的研究名加 `/study`：

```bash
uv run python scripts/research/study.py research=force_controller_selection/study
uv run python scripts/research/study.py research=force_controller_selection/study \
  execution=study_run execution.workers=8
```

默认只生成计划，执行前用相同参数审阅计划。`execution.workers` 默认为 `1`，大于 `1` 时并行执行
独立条件；按机器资源选择数量。study 拒绝外层 `-m`，矩阵由研究定义唯一生成。

## 推荐的正式研究执行顺序 {#formal-study-route}

各阶段之间审查产物；候选参数经人工冻结后再进入最终比较。Study 不会自动向下一项研究传递最优参数。

| 顺序 | `research` 名称（省略 `/study`） | 决策作用 |
| --- | --- | --- |
| 1 | `stiffness_ground_truth_validation` | 检查默认估计器的量级、有效性与参考边界。 |
| 2 | `force_tracking_stiffness_limit_pilot` | 比较无限幅、在线限幅与准静态参考限幅。 |
| 3 | `force_tracking_stiffness_rate_validation` | 检查控制结构的极限环与外环频率敏感性。 |
| 4 | `force_controller_ablation` | 检查 PID、刚度位置前馈与力矩前馈的贡献。 |
| 5 | `force_tracking_stiffness_rate_tuning` → `force_tracking_stiffness_rate_refinement` → `force_tracking_stiffness_rate_confirmation` | 初筛、最坏工况再调优、跨频率与材料确认。 |
| 6 | `torque_adrc_tuning` | coarse 筛选，再由 confirm 复验候选。 |
| 决策门 | 人工审查 | 检查状态、科学失败、执行异常、排名与配对统计；更新并提交配置、测试和文档。 |
| 7 | `force_controller_selection` | 比较已经冻结的控制器。 |
| 独立基线 | `dm_admittance_tuning` | 调整导纳接近与接触切换，不进入默认 PID／ADRC 矩阵。 |
| 独立研究 | `friction_local_slip_validation`、`robotiq_discrete_force_validation` | 分别验证局部起滑与整数命令控制，不阻塞 DM 选型。 |

`stiffness_estimator_validation` 仅比较估计器接入控制后的执行指标，不作为刚度精度或默认方法选型依据。
Torque ADRC 的 confirm 是入口强制要求上游谱系的阶段，必须指定已完成 coarse 的绝对目录：

```bash
uv run python scripts/research/study.py research=torque_adrc_tuning/study \
  study.stage=confirm study.coarse_study_dir=/absolute/path/to/completed-coarse-study \
  execution=study_run
```

矩阵、准入规则与科学结论见[控制算法对比与消融](control-comparison-ablation.md)，条件列表以
`configs/research/<purpose>/study.yaml` 及生成计划为准。

## 多条件研究与产物约定

正式产物位于 `outputs/research/studies/`；每个条件有独立 run。父目录保存 `study.yaml`、
`study.resolved.json`、summary、适用的 aggregate 与研究图，统一登记到 `study_manifest.json`。
状态、失败分类、科学哈希与只读恢复检查见[科研配置规范](research-configuration.md)。

力跟踪 run 保存输入快照、`effective_parameters.json`、`metrics.json` 与 Zstd 压缩的 `trace.parquet`。
常规轨迹为 100 Hz，直接力矩 ADRC 为 250 Hz；状态变化和 waypoint 前后 0.2 s 保留完整控制频率。
指标与运行时图像使用未降采样数据；[出图模式](research-configuration.md#plot-modes)控制图像数量。

从已有目录重绘：

```bash
uv run python scripts/research/render.py <run-directory> --format both
```

可加 `--tactile-detail` 或 `--output-dir`。重绘读取 Parquet，缺失时回退 CSV，沿用原指标；默认写入
独占的 `plots/replots/<UTC>-<id>/` 并登记 `rendering_manifest.json`，不覆盖原产物。PDF 需显式请求。
用 `pgt runs list` 查看产物；`pgt runs clean --all` 先预览，确认后加 `--apply` 删除。

## 实验报告与论文工作稿（Typst）

编译、冻结数据与章节维护见[科研报告](reports.md)。新结果进入报告，稳定方法与边界经审查后再更新专题。

## 演示视频录制

```bash
uv run python scripts/demos/record_experiment_demos.py --demo both
```

可选 `--demo ramp --noise-seed 0 --fps 60`。需要 ffmpeg；无界面环境在导入 MuJoCo 前设置
`MUJOCO_GL=egl`。视频合成场景与实时曲线，默认 1920×1080、30 fps，写入 `outputs/demos/`；
关键事件预览帧位于其 `preview/` 子目录。尺寸与输出位置见命令帮助。

## 论文图导出

复用 `plotstyle`：`science`、`ieee`、`no-latex` 样式，TeX Gyre Termes 与 Noto Serif CJK SC 字体。
`paper_figsize(height, columns=1)` 为 3.5 英寸单栏，`columns=2` 为 7.16 英寸跨栏。
使用约束布局和 `save_publication_figure(figure, path)`，按扩展名输出；实验默认单份 600 DPI PNG。
不使用改变栏宽的紧边界裁切；交付前实际检查最终尺寸下的中文、负号、图例与裁切。

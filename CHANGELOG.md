# Changelog

本项目的所有重要变更都记录在此文件中。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [Unreleased]

### 新增

- DMgripper 遥控新增张开方向雅可比前馈：负力矩平滑渐入，默认等效大小与闭合相同；
  可通过 `--opening-feedforward-force` 单独调整或设为 0 关闭，回中保持当前比例，
  双向反转先释放原方向前馈至零再渐入新方向，面板同时显示开闭设定。

- DMgripper 遥控窗口调整为仪表盘布局：三张反馈卡片、独立命令区、摇杆输入条和更大的数值；
  使能按钮直接提示未连接手柄、需要回中等原因，保持原有输入互锁和控制行为。

- DMgripper 遥控支持双向平滑切换：反转先释放旧方向位置偏差及前馈，再沿新方向推进；
  开闭前馈按方向渐入，首次使能前馈为零。新增 `--switch-duration`（默认 0.25 s），
  回中保持目标与前馈比例并暂停过渡；释放恢复及快速再反向从当前输出重新平滑接续。

- DMgripper 遥控面板放大数值与说明字体，分区显示电机关节位置／速度／力矩反馈，
  以及最近 MIT 命令的 `kp`、`kd`、`q_des`、`v_des`、`tau_ff`；未发命令或已确认失能时清空命令显示。

- `dmgripper-teleop` 新增 `--feedforward-force` 平均单侧目标力参数（默认 1 N，0 关闭）：
  按当前反馈角的机构闭合雅可比实时换算 MIT 前馈，按协议 4 N·m 上限限幅，
  面板显示目标力和最近已确认的力矩命令；按开闭状态渐变叠加，失能流程不变。

- 新增独立 `dmgripper-teleop` 手柄角位置遥控：摇杆偏转映射目标角速度，回中保留最后目标角保持，
  支持中心死区、角限位和可选目标领先限制；默认不限制目标与反馈的角度偏差，
  允许受阻时继续通过位置误差产生力矩；失焦／断连／心跳失鲜失能。默认完全离线模拟，
  `--execute` 连接真机后仍需手动使能，不依赖触觉，尚未完成真实设备联调。

- Robotiq 遥控面板新增中文状态、手柄名称、大号位置与目标编码、开合进度条，
  调整激活与停止按钮布局；保留原有短按、长按及安全取消行为。

- 实验台 udev 规则新增 `/dev/robotiq2f85` 串口别名，绑定序列号为 `DAASND29` 的 FTDI USB／RS485 转换器。

- 新增可选 `robotiq-teleop` USB／RS485 鼠标和手柄开闭入口：显式激活、短按目标 ±1、
  持续按住 0.4 s 后连续开闭，松开停止连续运动；显式停止和输入失效取消待发单步。
  按钮编号可配置，并提供不访问设备的 `--dry-run` 模式。旧 `--motion-mode` 参数仅兼容解析，
  不再切换行为。尚未完成真实夹爪及 TC-G50 实物验证，不包含触觉力闭环。

- 真机 PID 新增独立模型力矩前馈 `controller.pid.torque_feedforward_gain`：显式 0～1
  按实时机构雅可比与平均单侧目标力生成前馈，不依赖刚度估计，也不额外引入位置修正。
  显式值覆盖旧模型项避免重复叠加，省略／`null` 保持旧行为；共享核新增对应可选参数，其他控制器不变。
  水瓶曲线配置为 PID 启用完整模型前馈，保持刚度仅诊断和原有速度／力矩／过力保护。

- `dmgripper-plot` 支持 `--phase`／`--task-time` 窗口化评价图（须配合 `--repaint`，不覆盖源目录整段图）：
  `--task-time 起:止` 只保留任务时钟推进的行并把横轴切换为任务时间，`--phase` 按阶段白名单截窗；
  窗口条件写入重绘 manifest 以便追溯，未知阶段或倒置区间直接报错。
- 新增水瓶真机候选配置 `water_bottle_curve.yaml`（2→4→6→8 N，每级 2 s 平台＋1.5 s 平滑过渡，
  总时长 12 s，目标上限 9 N／过力保护 10 N）与 `water_bottle.yaml`（低载荷统一自适应，单侧
  ≤1.5 N）；均未经真实设备安全验收，PET 瓶高力级与满瓶低力级支撑要求见配置注释。
- 多速率统一策略支持风险事件短期锁存交付、可选持续风险大步续增及分侧在线摩擦更新。
  新增 `unified_step_load_risk` 仿真候选；快速真机配置显式实验授权，不伪造风险／摩擦验收通过。
  低质量摩擦候选不再覆盖已有估计；保留目标限幅、风险预算、失鲜冻结和独立硬件保护。
  稳载局部重分配可显式纳入风险，纯力候选不代表已验证的起滑识别或真实摩擦。
- 真机统一策略可选接入采集线程内多速率预处理，原始与处理状态同包发布；新增
  `unified_adaptive_fast.yaml` 独立快速候选（1 N 预载、30 N／侧目标上限、35 N／侧候选过力保护）。
  新增导纳力矩前馈比例配置，默认零保持旧行为；记录器 1.3.0 追加采样时序诊断。
  原始过力／量程异常仍失能；快速配置现显式开放实验性风险与摩擦闭环，参数尚未经真实设备安全验收。
- 多速率力调度仿真新增可选 `tactile_fault`：切向高斯噪声、单／双帧毛刺、丢帧与真实采样间隔抖动，
  保存注入标记及序号缺口，复用闭环测试核对陈旧数据冻结和恢复增力边界；默认关闭，不修改真机行为。
- 新增共享触觉预处理与最新状态接口，以及 `dm_gripper/unified_step_load_multirate` 仿真组合：
  独立 1000 Hz 切向中值／低通及风险旁路、250 Hz 目标调度／控制，避免重复低通；
  增加采样序号、年龄、丢帧和事件诊断，可选登记 `tactile.jsonl` 高频日志。
  历史组合保持原路径；新接口暂不接入真机，不改变现有非法数据／超时保护规则。
- 新增 `dm_gripper/unified_step_load_transient`，在快速导纳候选上仅将载荷滤波时间常数
  缩短至 10 ms；复用现有回归补充前 200 ms 响应及全窗口位移检查，不启用风险增力或摩擦更新，
  不改全局默认、真机配置或原验收阈值。
- 新增独立仿真候选 `dm_gripper/unified_step_load_fast`，继承撤支撑基线，仅将导纳速度上限
  设为 0.20 rad/s、力矩前馈比例设为 1.0；用于后续瞬态诊断，不替换默认或真机配置，不放宽滑移验收。
- 新增仿真对照 `dm_gripper/unified_step_load_pid`，位置式 PID＋机构力矩前馈接入相同统一调度；
  增加位置、速度、前馈和执行限幅诊断。仅统一模式 PID 改为每物理步施加保持请求，旧 Oracle 路径不变；
  内部运动约束与导纳不同，结果不作为控制律单因素消融，真机权限不变。
- 新增 `dm_gripper/unified_step_load`：物体总质量约 255 g、自重 2.5 N，预载后撤去支撑，
  不施加额外外力；纠正早期使用轻方块外力阶跃代替重量的建模，保留旧运行快照。
  以现有参数化用例检查质量、重力和外力语义，并记录原控制参数下的失败边界，不放宽验收阈值。
- 新增固定分侧摩擦先验的自适应承载调度共享核，以及 `dm_gripper/adaptive_prior` 仿真组合：
  由实测切向合力生成绝对需求，以缺口和因果滤波载荷趋势连续增力；保留能力超限诊断。
  复用现有导纳与 MIT 适配，仅供初步仿真验证。
- 新增 `dm_gripper/unified_adaptive` 仿真与 `configs/hardware/dmgripper/unified_adaptive.yaml`
  真机配置，复用逐触点风险、分侧摩擦状态、有界事件增力与导纳抗饱和；默认仅摩擦先验承载闭环，
  风险／摩擦更新还需显式验收门禁。统一模式触觉保护失败不进入故障保持，科学失败保持仍检查触觉健康。
  新增 `python -m dmgripper_experiments.replay` 旁路回放入口和 `adaptive_*` 诊断列，记录器版本为 1.2.0；
  软件与仿真验证不代表真实局部滑移识别或真机稳定性已经通过验收。

- DMgripper 真机链路新增分层硬件会话与运行时安全阶段：`papillarray_hardware` 提供带包／传感器／
  taxel 完整性诊断的通用采集会话，`dmgripper_hardware` 提供显式 DM 会话和独立命令／反馈范围校验，
  `dmgripper_experiments` 新增按启动位置选择的 `homing` 与非终态 `fault_holding`。共享
  `dm_grasp_core` 仍只维护运动学、轨迹、MIT 请求和控制算法，不访问设备或实验记录。
- 新增通用抓取实验入口 `dmgripper-run` 与离线重绘命令 `dmgripper-plot`：基础力跟踪与 cup
  专用实现收敛为同一条生命周期（预检、就绪、接近、接触过渡、初始抓力稳定、运行、保持、释放
  回位），目标力支持 waypoint 时间曲线与触觉动态增力，控制器在导纳、PID 与一阶 LADRC 中独立
  选择。输出按任务／物体／运行编号组织，trace 使用新 schema `dmgripper-experiment/v1`，
  记录原始／滤波／控制使用力、目标导数、刚度诊断与真实时序。交互终端默认 Rich 面板，重定向
  退化纯文本，`terminal.mode=json` 输出无 ANSI 的 JSON 行；终端刷新不参与控制时钟。
- 共享核 `dm_grasp_core` 新增目标力曲线纯计算（`grasp/reference.py`）与法向力控制器的外部
  生命周期跟踪接口（`begin_tracking`／`step_tracking`）：生命周期归运行时所有，控制器不再
  通过内部接触状态机隐式驱动；接触刚度估计器新增诊断快照（值、有效性、单次消费的更新事件、
  更新时刻与原因），仿真 `ForceReference` 委托同一插值实现。
- 通用实验包新增 `examples/`（曲线、动态增力与刚度诊断示例，默认离线）与 `scripts/plot_run.py`
  薄工具，以及 `configs/hardware/dmgripper/force_curve.yaml`、`adaptive_grip.yaml` 权威配置。
- `scripts/test_changed.py` 的 workspace 包分支升级为消费者映射：共享核改动同时覆盖真机实验包
  与仿真力跟踪回归，实验包与设备包改动覆盖对应契约测试。

### 变更

- PID 基线控制器切换为 `pid-torque-ff`（PID 位置修正＋机构力矩前馈），退役刚度位置前馈：
  消融证据显示刚度加法修正的附加收益接近于零（RMSE 差约 2.5e-05 N）。基线配置
  `position_feedforward_gain` 置 0，`full` 与 `pid-stiffness-ff` 变体改为代码内自包含派生
  （行为与历史逐字段一致），引用 `dm_gripper/full` 的切向扰动、摩擦估计、力调度实验与各研究
  预检基线同步切换；直连 API 默认变体同步改为 `pid-torque-ff`。

- DM controller 配置组统合公共基线：变体文件经组内 defaults 继承 `_base.yaml`，只声明
  `name` 与差异字段，并按变体角色规范注释；组合后的 resolved 配置与重构前逐字段一致，
  行为不变。变体间的模块启停与增益差异仍由 `configure_force_controller` 按 `controller.name`
  派生，修改基座 `stiffness_control` 等字段不会改变 PID 消融变体的实际行为。

- DM 仿真任务与代码默认控制周期统一为 4 ms（250 Hz），删除同轨迹的 `step_125hz`、
  `step_250hz` 频率副本，并将刚度速率验证、调优和确认矩阵收敛为固定 250 Hz；Robotiq 离散力控制
  保持独立 30 Hz 决策周期。DM 普通 trace 默认同步调整为 8 ms（125 Hz），确保每两个控制周期
  记录一次。噪声摩擦探测趋势窗、硬件量程候选的一致性门槛与切向扰动剪切门槛同步按 250 Hz
  重校准；历史多频率报告与既有产物保留原时序含义。

- 压缩统一力调度配置链：将快速导纳与短承载滤波覆盖直接并入 `unified_step_load_multirate`，
  保持现有基线、多速率和风险候选的有效参数；PID／导纳统一力跟踪入口复用共同实验设置。

- 共享仿真场景默认物理步长由 2 ms 调整为 1 ms（1000 Hz），仿真多速率触觉默认采样周期
  同步为 1 ms（1000 Hz）；滤波时间常数和失鲜阈值保持不变。历史研究产物与实验结论
  不回溯改写；新运行使用新的物理步长与采样条件。
- PapillArray 驱动、采集 CLI／示例及 DMgripper 真机实验默认请求频率由 500 Hz 提高为
  1000 Hz；真机多速率配置同步为 1 ms，控制频率与滤波时间常数不变。

- 文档按 Robotiq 2F-85 与 DMgripper 建立独立入口；拆分公共触觉接口、模型布局与验证依据，
  将曲柄滑块内容并入 DM 控制基础、力跟踪与刚度验证。资产正文随模型维护并由站点引用，
  明确独立性验收标准，修正 DM 资产说明中总力与平均单侧力、接触参数与材料刚度的混淆，
  移除未采用的 DC 电机扩展建模专题及其资料引用。
  夹爪入口合并模型与触觉布局，短篇配置、读取和验证说明并入主题小节，以页内目录减少阅读跳转。
  Onshape 导出教程仅随资产维护，移除主站导航与转引页，主文档保留资产维护入口。


- 水瓶曲线 PID 将 `max_position_adjustment_rad` 设为 `null`，取消固定位置偏置与积分幅值
  上限，使反馈修正可超过原 ±0.15 rad。共享核支持 PID 可选无固定上限，并根据后端位置／速度／
  力矩饱和停止同向积分；机械行程、速度、总力矩和触觉过力保护保持生效。
  有限数值配置保留旧行为，LADRC／刚度速率控制器仍要求有限累计行程上限。

- 共享位置式 PID 跟踪改用每周期实测位置作为参考：`q_des = q_real + position_adjustment`，
  同步影响仿真与真机 PID 及其刚度位置／力矩前馈变体。位置修正限幅改为当前反馈附近的偏置范围，
  不再限制接触后的累计行程；保留原 PID 增益、积分限幅、前馈开关与后端保护。
  一阶 LADRC、刚度速率、直接力矩和导纳控制律不变；既有固定接触参考研究结果仅代表历史版本。

- DMgripper 真机启动流程调整：`preparing` 阶段先做电机预检——打开 DM 串口、校验反馈与 MIT 模式、
  确认失能态（上次异常退出残留的使能态先显式失能一次再确认），位置不在 home 容差内时按受限回位
  参数自动回零后回到失能态，再执行触觉预检与零力验证。操作者约定运行前已移开物体，预检回零因此
  可以解除上次故障残留的闭合位置，不再被零力验证卡死；电机问题也在等待 `start` 前暴露。预检失败
  直接失能退出、不进入故障保持；零力验证期间电机保持失能、`start` 后复查再使能的既有语义不变。
- 快速真机候选 `unified_adaptive_fast` 改为 `prevent_unloading=true`：导纳跟踪期间禁止减小
  虚拟闭合量，显式释放流程不变；不保证实际抓力单调，不取消原始过力保护。
- `dm_gripper/unified_step_load` 的初始平均单侧目标改为 1 N、目标限速改为 50 N/s，
  用于隔离上层限速影响的仿真候选；滤波、导纳、MIT 与真机参数保持不变，位移验收线仍为 2 mm。
- 目标力调度主图改为抓力跟踪、实测切向承载与显式真值参考、独立位移评价三个面板；
  摩擦余量只保留在轨迹中，新增分侧实测切向记录并同时输出／登记 PNG 和矢量 PDF。
- 压缩当前文档，删除已完成的配置重构计划、迁移基线说明和旧 cup 独立说明，清理导航与失效引用；
  操作指南保留当前入口，历史变更通过 Git 追溯，实验数据与代码保持不变。

- 文档新增“自适应抓取”主题总览，统一 Oracle 调度、触觉增力与触觉摩擦能力估计的阅读入口；
  保留独立实验协议与证据边界。首页、测试说明与研究路线去重，贡献指南增加文档更新判断。

- DMgripper 命令工作范围继续保持 `[0, pi/2] rad`，反馈安全范围新增两端独立余量；默认
  `home_position_rad=0.0`、`home_tolerance_rad=0.03`、`feedback_position_margin_rad=0.05`。
  输入 `start` 后，当前位置不在 home 容差内会先执行受限自动回零；反馈越出扩展安全范围时禁止
  自动运动。实验配置和 DM 会话同时拒绝超出协议速度、力矩、`kp`、`kd` 量程的请求，防止协议
  静默饱和改变共享核的合成力矩检查。上述默认值尚待真机验收。
- DMgripper 使能后的触觉、接触、控制、记录及可恢复轨迹超时故障在 DM 通道健康时改为当前位置
  受限保持，等待操作者承接物体并输入 `release`；随后以不依赖触觉的 DM-only 轨迹回到 home，再
  确认失能。DM 通信／反馈丢失、短写、电机故障、意外失能、反馈越界或保持丢失仍立即尽力失能；
  `Ctrl+C` 保持紧急退出语义。故障后即使释放成功，运行结果仍为 `failed`。
- `dmgripper-experiment/v1` 的 `tactile.jsonl` 兼容新增双侧逐 taxel 三轴原始力与包连续性诊断；
  manifest 保留 `error` 并新增 `primary_error`、`fault_phase`、`fault_holding_entered`、
  `fault_holding_duration_s`、`fault_resolution` 和始终存在的 `cleanup_errors`，区分人工释放、强制
  失能与保持丢失。
- 修复 `dmgripper-run` 的交互启动与 Rich 终端兼容性：命令输入统一忽略首尾空白和大小写，明确提示
  按 `Enter`，收到 `start` 后在打开电机前记录并显示确认，未知命令和后台输入异常不再静默；Rich
  改为单一刷新所有者，预检和 `ready` 使用静态面板，首个控制快照后才启动 Live 并保持光标可见；
  `auto` 对 dumb 或重定向终端回退纯文本，运行时显示失败也会恢复事件输出。纯文本快照按配置刷新率
  合并，避免按控制频率刷屏。
  退出清理故障现在会使运行失败并返回非零状态，且设备失能结果不再被后续事件输出故障覆盖。
  Rich 结束后固定输出状态、失能确认和运行目录；中断伴随清理失败时直接在终端显示附加警告。
- `dmgripper-run` 的使能前零力验证改为只以双侧滤波全局 `Fz` 窗口均值为门禁；逐 taxel 数据只用于
  传感器数量、结构、有限性和运行中数量一致性的硬检查，以及残余均值、标准差、最大值和位置诊断，
  不凭空增加统一幅值门限。请求 bias 时先收到首个完整有效包，再只发送一次 bias；bias 前数据丢弃，
  滤波与设备时序基线重置，settle 后的新包才进入验证窗口。全局峰值告警、原始法向力上限、双侧
  不平衡及其他运行期安全保护不变。
- `dmgripper_experiments` 的 PID／LADRC 路径改为把原始力交给共享核、由核心内部做唯一一次
  低通（此前外层滤波后再滤波一次）；导纳路径维持外层滤波力输入。trace 同时记录三层力信息，
  滤波时延可追溯。等效接触刚度估计默认启用但只诊断，控制消费需显式
  `controller.stiffness_consumption: feedforward`。
- `dmgripper-force-demo` 与 `dmgripper-cup` 旧入口改为迁移提示并拒绝执行，不再把旧场景交互
  映射成自动阶段推进；`dmgripper-cup-plot` 保留为通用历史读取器薄别名，历史 v1／v2 cup
  trace 仍可重绘。旧 `cup_*` 专用实现与基础状态机移除，包版本升至 0.2.0。
- `dmgripper-cup` 导纳路径新增面向打印传动回差的力死区与单向闭合保持：误差进入死区时冻结
  导纳位移和虚拟速度，力偏高时禁止期望位置反向，正常张开仅由显式 `release` 触发；过力与
  左右失衡保护保持优先。倒水 trace 升级为 schema v2，新增死区激活与卸载阻止诊断字段。

- 新增 `dmgripper-cup` 真机倒水交互入口及 `dmgripper-cup-plot` 离线重绘命令：配置按 dataclass
  默认值、严格 YAML、Tyro 覆盖合并；默认 dry-run，只有显式 `--execute` 才进入真机运行时。运行记录
  写入配置、事件、触觉、控制 trace、manifest 和 PDF／PNG 诊断图。默认参数尚未经真机验收。

- 新增 DM 纯仿真 `tangential-disturbance` 单次实验：稳定预载、撤支撑和初始保持合格后施加 ramp／step／pulse
  切向载荷；支持 `full`／`pid-only` 的 MIT 法向 PID 路径、constant／fixed_step／dynamic_step 只增目标策略及
  shear_increase／force_ratio 检测器。runner 保存全频 CSV、指标及同 stem PNG／PDF；Hydra 支持单次计划与探索性
  Multirun，尚未定义正式 study。
- 新增 `pgt configs list [GROUP] [--search TEXT]`，从现有 YAML 元数据发现配置名与用途，无需先运行仿真。
- 正式 study 终端显示已处理／总条件数、科学失败、执行异常、状态和目录；Python 入口支持可选
  `on_progress` 观察回调，仅在父进程账本落盘后通知，计划模式只显示条件数与目录。

- 文献指南补入 DENSE 采集与弱标签边界，限定 Tactile Genesis 的背景借鉴范围，并按用户实际可用性排除 DIFFTACTILE。

- 文献指南改以轻柔抓取和目标法向力调节为主线，新增 Liu 2025 及变化量比值验证建议；按输入约束排除依赖触觉图像或稠密视觉重建的候选。

- 优化研究合集的旧图取舍：移除标签密集的控制器九宫格与 ADRC 排名图，保留数值表及结论，
  并明确旧摩擦曲线对应历史残差检测器。
- 新增 Typst 报告总目录、文献指南与近期实验入口；报告正文按文献和实验拆入 `reports/chapters/`，
  Markdown 正文迁移为原生 Typst，编译维护说明移至 `docs/reports.md`，保留既有合集与冻结数据。
- 新增 `pid-stiffness-rate` 实验控制器：PID 输出期望力变化率，经在线 `window_linear` 刚度与机构闭合
  雅可比映射为关节速度，再按实际外环周期积分为位置修正；同时新增 125／250／500 Hz、三控制器、三 seed
  的 `force_tracking_stiffness_rate_validation` 配对研究及速率／速度限幅 trace 诊断量。
- 新增 `force_tracking_stiffness_rate_tuning` 小规模调优：固定 250 Hz stiff Step、`K_I=K_D=0` 和
  `window_linear`，比较九组 `K_P × 最大力变化率` 候选及 `pid-torque-ff` 性能基线，按平台力标准差、
  超调约束和 RMSE 排名，并生成物理符号参数热图。
- 新增 `force_tracking_stiffness_rate_confirmation` 参数确认：固定二次调优选出的
  `K_P=20 s^-1`、`max_force_rate_n_s=50 N/s`，覆盖 500／250／125 Hz 与三种正式材料的 54 条
  配对条件，并登记实测最大正力增长率。
- 新增 `force_tracking_stiffness_rate_refinement` 二次调优：针对首轮确认中 500 Hz medium／hard 的
  超调失败，以 42 条配对条件扫描 `K_P={20,25,30} s^-1` 与 `max_force_rate_n_s={50,70} N/s`；
  多材料热图与候选排名统一采用最坏工况口径。
- 新增 `force_tracking_stiffness_limit` 三臂配对研究：固定 `window_linear`，比较无位置限幅、在线刚度限幅与
  准静态参考刚度限幅，并报告接触窗口峰值、最大正力增长率、跟踪误差和限幅介入率；pilot 固定 MuJoCo
  物理步长 2 ms、外环周期 4 ms，并将允许力变化率提高到 50 N/s。
- 新增 `stiffness_ground_truth_validation` 正式研究：通过加载／卸载平衡工作点的分支内中心差分建立
  控制器语义下的等效接触刚度参考，并按估计器、接触材料和噪声 seed 聚合对数 RMSE、相对偏差、低估率、
  估计抖动与相邻平衡点加载力增量低估指标；支持条件级 CPU 多进程。
  扫描检查双侧接触、末段力／闭合波动及估计有效比例，未激励初值不计为合格估计。
- 新增研究报告合集 `reports/combined.typ`：单一编译入口产出一份便于连续阅读的 IEEE 双栏合集 PDF，
  按方法说明、研究证据汇总、论文工作稿与初步证据四部分组织；研究证据部分每项正式研究一节，给出
  科学问题、三线表、关键产物图与结论表述，覆盖控制器选型、PID 2×2 消融、Torque ADRC 两阶段调参、
  刚度估计器对比、DM 导纳整定、Robotiq 离散力控制与已退出的摩擦局部起滑矩阵。报告数据以字面量写在
  文件内的数据块中，编译不读取 `outputs/`；`scripts/reports/study_results_data.py` 从 study 产物生成并
  校验该数据块，报告模板同步新增渲染字面量记录的 `data-table()` 组件。
- 正式 study 新增统一的 `execution.workers`：执行模式可用 `spawn` 条件级 MuJoCo CPU 多进程，父进程
  独占 manifest、聚合与绘图，并保持 `StudyPlan` 结果顺序；默认值 `1` 保持原串行行为。
- 新增 force-track 单次绘图层与重绘命令：runner 通过 `on_result(full_rows, result)` 生成
  `plots/tracking.png`、`plots/tactile.png` 和 `plots/controller.png`，并登记实际产物；新增
  `scripts/research/render.py` 的独占重绘目录、`rendering_manifest.json`、Parquet 优先读取和可选逐 taxel 细节。
- 新增 force-track trace schema v2 字段：期望位置／速度、命令与执行力矩、刚度有效性、导纳状态、双侧切向力和
  逐 taxel 三轴力，并明确 `control_time_s`、`command_time_s`、`reference_start_time_s` 的边界时间语义。

- 新增 DMgripper PID／二阶导纳统一对比入口：两者共享 PID Ramp 目标、4 ms 外环、MIT 增益、
  线性关节接近轨迹、双侧接触确认、50 ms 速度过渡及任一侧持续掉力后的重接近语义。
- `dm-grasp-core` 新增双侧接触公共状态机，将接触确认、速度过渡与可配置的掉力判据从具体力控制律中解耦。
- `dm-grasp-core` 新增 DM 力控纯算法模块：达妙 MIT 协议量化与力矩命令（`MITTorqueModel`）、
  三种方法的在线接触刚度估计、二阶直接力矩 LADRC，以及法向力外环状态机与 PID、一阶 LADRC、
  直接力矩、二阶 LADRC 四条跟踪路径（电机访问经 `MITTorqueInner` 协议注入，配置为显式
  frozen dataclass）；新增对应核心单元测试。
- 新增配置重构迁移基线：冻结重构前全部 YAML、profile、task、模型资源摘要、单次组合和正式研究有序矩阵，
  并记录字段所有权、新旧路径、研究准入／退出与诊断归档依据，供后续组合实现做独立等价比较。
- 新增 Hydra 原生科研单次与正式 study 入口，以及按平台、控制器、估计器、任务、材料和研究方案组织的
  可复用配置组；计划模式执行完整领域与资源校验但不推进仿真，单次入口支持原生串行 Multirun。
- 新增科研调用的有效配置、组合选择/覆盖/来源、Git 状态和失败条件溯源；正式控制器对比与 PID 消融
  复用各自唯一权威矩阵，并在调度前拒绝外层 Multirun。
- 新增类型化 `StudyPlan`、条件结果和公共生命周期 manifest，区分
  `planned/running/partial/completed/failed` 状态、科学验收失败、执行异常及聚合／绘图失败；登记产物
  SHA-256，并提供不自动续跑的既有条件与可恢复性报告。
- Torque ADRC coarse／confirm 两阶段调参迁入 Hydra 正式 study：保留 102 条 coarse 权威矩阵及动态
  confirm 选择规则，confirm 严格验证 coarse 类型、阶段、科学配置哈希和排名摘要。
- 摩擦局部起滑（15 条）、刚度估计器对比（81 条）、DM 导纳调参（32 条）、Robotiq 离散力（60 条）与
  因果诊断（10 个 phase 共 39 条）迁入 Hydra 正式 study 入口：条件矩阵、统计口径、排名规则与产物
  文件名保持不变，manifest 升级为公共生命周期状态机并登记产物 SHA-256；新输出位于
  `outputs/research/studies/<kind>/`（诊断多一层 phase 目录）。
- 因果诊断新增 `study.phase` 选择字段（仅诊断研究可用、必填），一次调用执行一个 phase 并使 phase
  进入科学配置哈希；原 `--phase all` 由逐 phase 调用替代。
- 文档站点导航补充通用抓取实验、DMgripper 共享控制核与执行器基线，推荐路线编号顺延补齐；
  删除已被实现取代的纯 Python 真机联调路线图和通用抓取重构实施计划，将配置迁移快照与旧 cup
  trace 说明明确标为历史资料，并修正控制核归属、viewer 参数和 Onshape 模型组合等过时描述。

### 修复

- 修复 DMgripper STOP 后再次使能的反馈错位：失能命令直接接收自身回复，不再追加状态查询；
  保留使能／失能状态确认及失败报告。
- `robotiq-teleop` 在控制窗口运行期间按 Ctrl+C 正常退出，不再打印中断调用栈；
  保留停止与断开清理，其他程序异常仍正常报告。

- 修复 PapillArray 微秒时钟在 uint32 边界回绕时误报“设备时间未严格递增”：
  仅对包计数前进且跨边界模差不超过一秒的回绕展开连续设备时间，避免滤波和下游时序中断。
  重复、乱序及其他时间回退仍拒绝；记录器 1.4.0 在触觉日志中追加原始时间与回绕次数，
  保留既有实验记录，不改变主机新鲜度和设备保护。

- 真机实验结果图的数学字体从 SciencePlots 默认的 CM 改为 STIX：CM 系 `cm*.ttf` 的 head 表时间戳
  低于 Unix 纪元，PDF 嵌入时 fontTools 会对每个数学字体输出两条无害告警，污染真机终端输出。
  STIX 同为衬线数学字体，观感与 CM 几乎一致；仿真侧统一样式不受影响（仍为 dejavuserif）。
- 通用抓取结果图将动态增力 trigger 改为切向力曲线下层的绿色半透明背景区；时间、力、位置、力矩与刚度坐标轴改为标准数学符号，图例改用简短方位词或物理量符号。
- Rich 实时面板新增固定底部命令输入区，统一 `start`、`status` 和 `release` 的键入、退格与回车体验；新增需要回车确认的 `s`/`r` 快捷输入，并在退出时恢复终端属性。
- `preload` 稳定判据改为确认双侧接触且平均力持续高于最低预载线，删除高侧稳定窗口；
  动态增力与曲线目标统一使用 `safety.max_target_force_n`，双侧力差不再作为硬停机条件。
- 旧 `dmgripper-cup` 过渡实现曾把使能前零力门禁改为滤波双侧 `Fz` 的 0.5 s 窗口均值和接触阈值
  峰值保护；该入口现已由 `dmgripper-run` 取代，通用入口的最终行为以上述“均值门禁、峰值告警”为准。

- 修正 `dmgripper-cup` 将 `0.1 N` 零力阈值误用于逐包原始三轴合力的问题：启动门禁现与 ROS 2
  一致使用 10 Hz 低通后的双侧 `Fz`，原始三轴力继续承担有效性与过力保护；零力期限耗尽时输出
  末值和最长稳定时间，不再误报为触觉快照超时。

- 缺少 Full／secant 基线或有效对比时不再生成空差值图；摩擦无确认事件的重复空面板合并说明，
  Robotiq 反向与振荡计数改为并列展示，避免将重叠事件误读为可相加类别。

- Hydra／OmegaConf 改为主包运行依赖，修复未安装开发依赖时 CLI 导入失败；保留空的 `research`
  依赖组兼容旧安装命令，并在 CI 增加最小运行环境冒烟检查。
- 修正 README 的旧摩擦估计描述和工作流的旧脚本保留说明，首页改为最小运行与产物查看入口，
  文档导航补齐科研配置、测试策略和报告维护。

### 移除

- 移除 `controller=dm_gripper/full` 与 `controller=dm_gripper/pid_stiffness_ff` 两个 CLI
  配置入口（刚度位置前馈退役）；变体派生与行为测试保留在代码层，历史研究复现以当时
  git 版本或内存派生为准。
- 移除 `controller=dm_gripper/adrc_torque_td` CLI 配置入口：线性 TD 是带微分状态输出的
  二阶临界阻尼参考低通，解析参考（smoothstep／linear）本身就是零额外相位滞后的替代，
  且该变体从未进入任何正式研究矩阵；变体派生与行为测试仍保留在代码层，
  `adrc_torque` 默认控制器不受影响。
- 退役独立导纳调参研究线：正式执行 `dm_admittance_tuning` 32 条件全部稳定且完整跟踪
  （force_tracking 占比均为 1.0，满足停止条件），结论为接近速度 0.1 rad/s 组全面优于
  0.05 组（原始侧力峰值 0.707 对 0.731 N、RMSE 0.043 对 0.139 N），接近／接触切换参数
  不敏感；产物保留于 `outputs/research/studies/dm_admittance_tuning/`。
  随之移除 `controller=dm_gripper/admittance`（旧调参入口）、
  `experiment=dm_gripper/force_tracking_admittance`、`dm_admittance_ramp` 任务、研究协议
  与专属实现；统一导纳线 `admittance_unified` 不受影响。
- 移除一阶位置式 ADRC 与直接力矩两个历史复现配置入口
  （`controller=dm_gripper/adrc`、`controller=dm_gripper/direct_torque`）；
  两者不进入默认正式矩阵，变体派生能力仍保留在代码层，历史数据复现以当时的 git 版本为准。
- 移除与 `pid_only` 逐字段相同的冗余控制器配置 `controller=dm_gripper/pid_only_unified`；
  统一 PID 力跟踪实验 `experiment=dm_gripper/force_tracking_pid_unified` 改用
  `controller=dm_gripper/pid_only`，组合结果不变。
- 移除依赖仓库外 ROS 节点的 `test_dm_ros_alignment.py`，以及共享控制核迁移期的
  `golden_tracking.json` 和生成脚本；共享核继续由仓库内独立数学期望、边界测试和仿真集成验证。
- 移除不再使用的 DMgripper 共面 mesh、共面球体配置与 MJCF 资产；模型生成脚本收敛为只生成
  当前默认的保留高度差球体模型。原始 mesh 继续作为历史行为复现入口，旧诊断结论与版本指针保留。
- 移除 `unified_step_load_fast`、`unified_step_load_transient` 中间实验入口，当前候选使用
  `unified_step_load_multirate`；移除 `research=archive/model_bug_diagnosis/study`、专属诊断协议、
  Python 导出及 `study.phase` 字段。历史报告、产物与冻结快照保留，复现版本见模型验证文档。

- 删除仅供旧导纳单次实验使用的恒定 1 N `dm_admittance` task；导纳 experiment、仿真测试与正式调参
  统一使用 `dm_admittance_ramp`，不再维护两套专用预接触时序。
- 删除 `scripts/experiments/` 下全部 8 个专项研究旧入口；其实现迁入 `studies/protocols/` 包内并由
  Hydra 正式入口复用，避免直接路径参数绕过统一组合解析。
- 移除 `dm_admittance_tuning` 的 `max_workers` 字段与 `robotiq_discrete_force` 的 `--jobs`/
  `--dry-run` 参数：并行度改由公共 `execution.workers` 负责，计划审阅改用默认的
  `execution=study_plan`。
- 移除已并入合集 `reports/combined.typ` 的独立文档入口：`force_control_comparison.typ`、
  `study_results.typ`、`wired_demo.typ` 与 `stiffness_reference_pilot.md`；`template.typ`、
  `wired_demo.bib` 与 `reports/figures/` 快照继续由合集使用，论文工作稿编译期直读产物的数字
  冻结为字面量。

### 变更

- 单次运行与正式研究新增 `execution.plot_mode=summary|diagnostic`，默认汇总模式；力跟踪单次默认
  仅生成跟踪主图，研究按预定最小 seed 保留逐次诊断，科学失败仍保留诊断。完整诊断可显式恢复，
  trace、指标、科学配置哈希和条件矩阵不变。
- 研究汇总省略简单基线差值与 MAE 补充面板，ADRC 确认阶段参数关系和速率确认振荡幅值转为诊断图；
  导纳调参补充既有排名与约束汇总，抓取、载荷调度、离散控制和摩擦快照精简重复展示。

- 正式研究改用显式适配器注册表集中解析、预检和协议分发，保留领域矩阵、科学哈希与阶段谱系。
- 合并静态迁移快照检查，参数化研究计划测试，复用同参数的名义摩擦仿真基准；保留原科学断言与端到端覆盖。

- 最终控制器比较移除出现 stiff Step 平台极限环的 `pid-stiffness-limit`，加入
  `pid-stiffness-rate`；`pid-torque-ff` 继续作为性能基线，矩阵规模保持 162 条，并统一固定
  `window_linear` 刚度估计器。

- 科研报告改为围绕“目标力跟踪 → 轻柔接触 → 自适应持握 → 防滑搬运与稳定放置”组织证据；运行编号与
  Git 提交退出正文，正式报告数据由 Python 汇总后冻结，Typst 编译不直接读取原始研究产物。
- 报告模板的 `fmt-float()` 补足有效数字尾零（`0.15` 与 `0.213` 并排时显示为 `0.150` 与 `0.213`），
  整数按整数显示，既有报告的数字与结论不变，只是表格内同一列的显示更整齐。
- DMgripper 单次力跟踪诊断图组包含三张 600 DPI PNG：目标／滤波力、双侧触觉力和控制器状态；复用
  SciencePlots／MathText，默认不再生成误差、滞回、limits 或 state 面板，也不自动生成 PDF。waypoint 仅在线性
  参考曲线上使用 marker，关键接触事件使用细灰竖线；既有指标与控制律保持不变。
- 可选逐 taxel 细节图改为空间对应的 3×3 小倍图；每个面板保留 `Fx`／`Fy`／`Fz` 原始单位，法向与
  切向分别使用跨 taxel 统一量程，便于定位偏载和局部异常。

- 绘图导出改为只保存调用方请求的一种格式；现有单次实验与正式 study 默认保留 600 DPI PNG，
  不再自动生成同名 PDF 或在 manifest 中重复登记。
- 清理正式研究配置的跨层重复字段：活跃研究以 experiment 组合为唯一 profile 来源，以 execution 组为
  唯一输出目录来源，并移除用于占位的 task、seed、execution 覆盖；科学哈希改为摘要实际冻结的组合
  profile。归档模型诊断继续保留有注释的历史 profile 例外。
- DMgripper PID／二阶导纳统一对比入口的预接触线性关节轨迹由 25 s 缩短为 6 s，
  双侧接触等待上限由 30 s 缩短为 8 s，避免正式采集前长时间空跑。其余 DMgripper 力控制器
  也统一启用相同的双侧接触状态机；旧导纳实验的接近阶段改由 task 轨迹控制，不再使用内部
  低速轨迹产生约 41 s 的预接触等待。
- 力跟踪循环在启用公共状态机时显式传递接近关节速度，使 PID 与导纳的接触过渡从相同运动状态开始；
  统一对比入口只替换跟踪阶段的 PID／导纳控制律。
- DM 力控算法迁移至 `dm_grasp_core`：仿真侧 `parallel_gripper_tactile.control` 收敛为适配层
  （MIT 执行器绑定、profile 配置转换与公共名称再导出），导入路径、类签名与数值行为不变；
  曲柄滑块运动学公式收敛到核心唯一维护（核心版闭合雅可比增加工作区间正值校验）；
  `dm-grasp-core` 运行依赖新增 numpy 与 simple-pid。
- 单次 DM 力跟踪迁至统一 `configs/run.yaml`：platform、四种碰撞 model、全部 PID／ADRC／导纳 controller、
  四种 estimator、六个力跟踪 task、material、execution 与常用 experiment 直接组合为冻结领域对象；
  `experiment=dm_gripper/force_tracking_admittance` 完整保留专用 MIT 增益、接触阈值、滤波和 estimator 关闭。
- 力跟踪 runner 在收到已解析对象时直接保存该对象的 profile／task YAML，不再把组合片段误作完整快照；
  运行算法和产物名称不变。
- 力调度、摩擦估计、Robotiq 离散力及 `pgt` 的抓取、视频、查看和接触／触觉比较入口改为消费同一
  experiment 组合服务；CLI 以 `--experiment` 和可重复 `--set` 替代旧 `--profile`、`--task` 及重复的
  科学参数，输出位置统一由 `execution.output_root` 配置。
- 正式研究迁至统一 `configs/study.yaml` 与目的导向的 `configs/research/<purpose>/study.yaml`；研究元数据、
  准入／停止／排除依据和领域矩阵由同一文件提供。Torque ADRC coarse／confirm 共用一份定义，模型 bug
  诊断移入 `research/archive/`，默认控制器选型明确排除已有 135-run 负面证据的 `direct-torque`。
- 正式研究的 profile 也改由命名 experiment 与 Hydra 覆盖组合；删除 DM 导纳、DM 平面球和 Robotiq
  box／touch-grid 的派生完整 profile，以及旧 task、研究 selector、兼容研究脚本和重复片段目录。迁移前
  参数由冻结基线保留。
- pre-commit 的完整 pytest 门禁改用 `pytest-xdist` 自动确定 worker 数并行执行；CI 继续执行串行
  完整测试，测试选择与验收口径不变。
- force-track runner 可直接接收同一份已校验冻结 profile，避免配置解析后再次读取原始文件；控制器切换
  会清除其他算法专用字段并重新执行完整 profile 与资源校验。控制器对比、消融和 Torque ADRC 调参
  实现迁入包内 protocol。

- 7 个研究脚本的 `aggregate_rows` 手写聚合统一收敛到 `studies/aggregation.py` 的声明式
  polars 聚合层（列规格加 `aggregate_records` 解释器），三份逐字节相同的 `_transient_stats`
  副本一并合并；输出列名、列序、行序与数值口径不变（均值完全一致，标准差与
  `statistics.stdev` 存在不超过 1e-16 相对偏差的浮点舍入差异），新增
  `tests/test_study_aggregation.py` 十八项回归锚点。

## [0.4.0] - 2026-09-10

### 变更

- 测试开发依赖新增 `pytest-xdist`，并隔离每个 worker 的 Matplotlib 字体缓存、默认限制
  OpenMP／BLAS 内部线程；保留裸 pytest 提交门禁，同时提供按变更模块选择相关测试的透明增量入口。
- 重复科研绘图测试改用小型合成数据的轻量渲染，真实 600 DPI、PDF 页面尺寸、字体和代表性复杂图
  集中由渲染契约测试验证；完整摩擦估计端到端与全部标准摩擦场景、科研验收结论保持不变。

- PapillArray 与 DM USB2CAN 的纯 Python 探针分别默认使用 `/dev/papillarray`
  和 `/dev/dmj4310_can`，同时保留 `--port` 显式覆盖；仓库新增当前实验台的 udev
  规则与安装说明。
- `dmgripper-motion-probe` 在三阶段成功且最终失能读回确认后，额外输出
  `event=complete`、`disable_confirmed=true` 的 terminal JSON；该事件不推断机械回位误差。
- `dmgripper-motion-probe` 的 MIT 阶段改为固定 `--stage-duration` 运行：`q_des` 作为阻抗平衡点，
  未达到目标位置仅记录诊断、不再判失败；JSON 增加阶段位置范围、最大绝对速度／力矩、命令次数和
  实际耗时，通信／状态／机械边界失败仍会失能。
- `dmgripper-motion-probe` 的 `--mit-kp` 改按 MIT 协议允许 `(0, 500]`；`--closing-step` 接受任意
  正有限值，二者不再由人为力矩阈值限制，最终目标仍严格拒绝机械行程外数值。
- `papillarray-probe` 新增单包总等待时限与有界协议诊断；即使串口持续返回噪声、坏帧或半包，
  也会自动退出并报告接收字节、起止标志、候选帧、校验／结构失败和原始十六进制预览。
- PapillArray PTS 顶层索引解析兼容真实 Controller v2.0 在索引表与首个数据块之间加入的
  全零对齐填充，同时继续拒绝非零的未声明数据。
- PapillArray 读取器在首次采样配置后的总等待时限内允许多次串口空读取，并记录空读取次数；
  不再因 Controller 启动延迟在第一次底层读取超时时提前退出。
- `dmgripper-hardware` 新增当前 DM4310P 夹爪部署配置：CAN ID 为 `1/17`，协议量程为
  `±1.7 rad`、`±8 rad/s`、`±4 N·m`，机械行程另设为 `[0, pi/2] rad` 且角度增大为闭合；
  机械目标越界直接拒绝，不由协议量化层静默饱和。
- 仿真步进会话迁入 `simulation/session.py`，摩擦估计、触觉滑移与逐点摩擦算法迁入
  `perception/`；原模块路径保留完整兼容导出，仿真循环与算法数值行为不变。
- `dmgripper-hardware` 新增延迟打开的 PySerial 传输和仅发送状态查询帧的单次反馈刷新；
  `robotiq-hardware` 新增严格校验的 `position()` 反馈读取与不可变命令收据。两条路径均不自动
  连接、激活或启动设备。
- 两个硬件包分别依赖自身控制核心：DM 新增从共享 MIT 请求到 USB2CAN 帧的纯离线适配，
  Robotiq 新增离散控制器到绝对位置命令的单步编排；后端发送失败不会登记为已执行动作。
- `dm-grasp-core` 按 `control`、`grasp` 与 `tactile` 子域整理内部实现，并保留顶层及
  `dm_grasp_core.control`、`dm_grasp_core.command` 兼容入口；算法公式、固定轨迹与版本号不变。

- 演示视频默认升级为 1920×1080 @30 fps 与 H.264 CRF 18；实时曲线面板复用
  `plotstyle.science_pyplot()` 的 SciencePlots IEEE 样式。自定义力箭头先经
  `mjv_initGeom()` 初始化，再由 `mjv_connector()` 设置起终点，避免复用槽位残留。

- Robotiq 离散力控制升级为 v2：正常正负调节动作共用单 tick 力增益模型，自适应量化死区采用
  `0.5·ΔF_tick`，predictive 与 dynamic-step 分别显式枚举 `{-1,0,+1}` 和 `{-3,…,+3}` 候选；
  新增请求/执行/机械位置、稳定动作增量与候选代价诊断。常规轨迹按独立记录时钟加关键事件记录，
  仅安全峰值等必要量在 500 Hz 物理循环中在线累计，不再构造全量物理步记录。
- Robotiq 离散力 trace 新增独立 `record_period_s` 时钟，默认以 100 Hz 常规采样并继续强制保留关键
  事件；完整 study 默认按可用 CPU 自动并行，最多使用 12 个工作进程，`--jobs 1` 可回退串行。
- `friction-estimate` 改为纯触觉特征评分检测，接口排除外部载荷、真实摩擦与物体运动；候选前窗口生成
  冻结摩擦候选，保持阶段使用实测切向力调度。新增 `tactile_slip` 配置及全部证据 trace/指标，旧检测
  参数兼容读取但不再生效。主图改为五面板，逐点图增加时间热图与空间快照，明确离线评分量。
- 硬件阈值场景探测速度降至 `1 N/s`，适配新检测延迟；旧多种子旁路结论标记为历史结果。

- 论文工作稿插图改为复制到 `reports/figures/` 的入库快照：编译不再读取 `outputs/` 中的
  图像文件，图像资产固定、不随 `pgt runs clean` 丢失；换数据源时重新复制替换。

- 科研绘图统一应用 SciencePlots 无 LaTeX 论文样式与中英文字体，使用固定论文栏宽，
  统一导出矢量 PDF 和 600 DPI PNG；抓取与接触对比运行新增 PDF 产物登记，原有显式输出格式继续保留。
- 论文图轴标签提高到 9 pt、标题提高到 10 pt、刻度与图例提高到 8 pt，并同步增强线宽与标记尺寸，
  改善双栏图在屏幕审阅和论文最终尺寸下的可读性。
- 跨栏整宽时间序列图（摩擦估计、力调度、抓取等单列纵排面板）放大字号一档：
  绘图入口通过 `science_pyplot(font_scale=FULL_WIDTH_FONT_SCALE)` 选用整宽档，字号与线宽
  等比放大 1.2 倍，保持文字相对 7.16 in 宽面板的视觉密度与单栏图一致；单栏图与多列网格不变。
- 主包、研究脚本与功能测试的内部导入全部从根路径兼容层迁至规范路径（`config.profiles`、
  `artifacts.run_artifacts`、`visualization.*`、`perception.*` 与 `robotiq_grasp_core`）；
  公共 API、实验行为与产物格式不变。

### 新增

- 新增 `dmgripper-force-demo` 纯 Python 真机入口：复用控制核的 minimum-jerk 闭合量轨迹完成
  预接触，双侧 PapillArray `+Fz` 稳定接触后平滑切换到二阶导纳力跟踪，正常回到机械零位并
  确认失能；接触进入与退出均采用持续时间和滞回判定，并记录基础 CSV。触觉首次配置或清零后的
  单包超时会自动重试，首包总等待与运行中新鲜度分别约束；bias 流程先确认数据流、再清零并等待
  `2 s`，零力验证和控制复用 ROS 2 的 `10 Hz` 低通非负法向力语义。正常回位超时按实际
  minimum-jerk 规划时长自动扩展，并在轨迹结束后按位置容差确认到达；预接触与回位轨迹改为
  闭合量空间规划并通过运动学逆解为关节命令，回位默认最大闭合量速度为 `0.012 m/s`，并提供
  命令行覆盖。回位 MIT 增益单独提高为 `kp=10`、`kd=0.5`，以克服零位附近的静摩擦。
  实际反馈进入 `0.02 rad` 容差后才确认完成并失能。

- `dmgripper-motion-probe` 提供默认 dry-run 的 DM4310P 受限阻抗联调：先只读验证初始状态，
  仅在显式 `--execute` 后按“当前位置保持、闭合平衡点、恢复初始平衡点”执行；每阶段
  检查机械位置、故障和通信超时，记录速度与力矩，所有结束路径均尽力失能，且不包含置零或
  未经验证的停止帧。
- 新增 `papillarray-probe` 与 `dmgripper-state-probe` 两个有限次数、JSON Lines 输出的纯 Python
  真机探针。前者只配置触觉采样率并读取 PTS 包，默认不执行清零或滑动检测命令；后者只发送
  DM 状态查询帧，不包含使能、置零或运动命令。
- 新增独立 `papillarray-hardware==0.1.0` workspace 成员：参考现有专有驱动提取 PTS v2.0
  协议解析、字节流重同步和显式生命周期同步串口客户端；保留包计数与设备时间戳，Type 7
  仅保存原始字节，并将设备清零／偏置清除作为显式命令而非传感器标定流程。
- 新增 `dmgripper-hardware==0.1.0` workspace 成员：基于现有达妙官方 USB2CAN 实现提供
  无设备 I/O 的 DM4310P 协议编解码、串口分帧器、显式固件量程和 fake transport；尚不包含
  真实串口、USB2CANFD、设备使能或运动流程。
- 新增 `robotiq-hardware==0.1.0` workspace 成员：严格校验 `0～255` 整数位置命令，并提供
  `pyrobotiqgripper==3.3.12` 的可选非阻塞适配器；连接和激活仍由调用方显式负责。
- 新增独立 uv workspace 成员 `robotiq-grasp-core==0.1.0`，承载 Robotiq 整数量化离散力控制、
  单 tick 力增益、HOLD／再激活与有限动作预测；仿真旧导入路径保留兼容出口，且该核心不依赖
  ROS、MuJoCo、profile 或仿真主包。
- 新增平行夹爪力控制方法说明报告，采用中文适配的 IEEE 双栏会议版式和 Times 系西文字体，集中整理位置式 PID、刚度感知位置增量限幅、二阶导纳与二阶直接力矩 ADRC 的控制律及验证边界；报告表格统一为三线表。
- 新增 `pid-stiffness-limit` 力跟踪变体：保留 PID 与机构力矩前馈，关闭基于同一力误差的刚度位置前馈，
  改用在线刚度和机构雅可比把允许力变化率换算为 PID 位置目标的周期增量边界；trace 与 metrics 记录
  边界值、触发状态和触发比例。既有 `pid-stiffness-ff`、`full` 及默认比较矩阵保持不变。
- 新增独立 `dm-grasp-core==0.1.0`，与 ROS 2 DMgripper 共用二阶导纳、运动学、
  平滑接近/接触过渡及 MIT 请求映射；仿真以 uv workspace 引用，ROS 安装固定 wheel。
- `pgt run force-track` 新增显式 `--controller-variant admittance`、可选
  `control.force.admittance` 配置及 4 ms / 1 N 示例。仅该变体在每个物理步重算
  MIT 内环，外环按任务周期运行；跟踪失接触采用 0.05 N/25 周期确认，避免瞬时单侧
  掉力重复触发接近；默认比较矩阵和旧控制器行为不变。
- 新增 DMgripper Ramp 导纳调参入口，候选使用独立输出目录和进程并行，
  结果按仿真稳定性、完整力跟踪、物理步进侧力峰值和 RMSE 确定性排名；
  示例参数由 `M=0.02 kg, B=0.2 N·s/m, K=1 N/m` 调整为
  `M=0.20 kg, B=15 N·s/m, K=1 N/m`。共享导纳改为先限速再积分，仿真接入力
  低通；示例统一采用 1 N 接触阈值和不低于 1 N 的 Ramp，并将滤波截止频率、接近/跟踪
  速度上限、接触过渡和接近前馈分别调整为 2 Hz、0.05 rad/s、50 ms 和 0.5 N。
- 新增迁移前固定轨迹回归、可选 ROS/仿真共同回放测试；导纳运行产物记录核心版本与
  执行器应用方式。共享请求与仿真量化结果分别说明，尚不宣称协议字节或硬件动态等价。

- 新增 `pgt run discrete-force` Robotiq 2F-85 离散力控制实验：命令严格限制为 `0～255`，离散变体
  保留真正的零动作 HOLD，并实现稳定窗口、统一双向 `ΔF_tick` EWMA、自适应死区与再激活滞回、
  一步预测、最多 ±3 tick 有限候选动作、安全预测及单 tick RELEASE；运行保存按独立周期（默认 100 Hz）
  采样、动作生效行强制保留的 gzip 压缩诊断 trace（`trace.csv.gz`）、指标和论文图。
- Robotiq 离散力控制默认使用 30 Hz 独立控制时钟；无需控制周期与 500 Hz 物理步长整除。
- 新增统一整数执行接口的量化 PI、固定单步、自适应死区、一步预测、动态步长五级消融；以一次抓取内
  `2→4→6→8→6→4→2 N` 的平台—过渡曲线为主任务，将 study 收敛为 4 种显式接触刚度 × 3 个
  噪声等级 × 5 个控制器共 60 条件，并输出逐次、逐平台及聚合 CSV/Parquet、JSON、比较图和子 run。

- 新增 `hardware_scale_nominal.yaml` 硬件量程对齐摩擦估计场景：按单 taxel `0.05 N` 标称分辨率采用
  `0.5 N/0.25 N` 接触滞回阈值，并同步将预载调整为 `4 N/侧`，避免只提高阈值造成有效触点丢失。
- 摩擦估计运行新增逐 taxel 接触滞回筛选与局部摩擦利用率旁路诊断：trace 保存双侧逐点接触掩码、
  `T_i/N_i` 和汇总量，并新增矢量 PDF 与 PNG 热图；现有总体摩擦估计和目标力调度语义保持不变。
- 新增纯力局部起滑旁路状态机：以局部摩擦比先上升后饱和和同侧剪切空间重分配为持续证据，锁存
  事件前保守局部摩擦候选；仿真速度和位移不进入检测器，局部结果暂不驱动目标力。
- 新增纯力局部起滑多种子 study 与低探测载荷负例，汇总检测率、误报率、相对总体判据的提前量和
  局部候选摩擦下界；摩擦估计 runner 支持记录传感器噪声 seed 覆盖。
- 新增 `pgt run friction-estimate` 力域初始滑移探测与保守摩擦估计：以世界 `+Y` 慢速切向探测，
  结合触觉摩擦比饱和和载荷—剪切支撑持续失配判据，从起滑前窗口生成安全折减的 `μ` 下界；估计器
  不读取真实摩擦系数、物体位移或速度，检测失败时显式使用保守回退值
- 新增低、中、高摩擦与两倍传感噪声四个标准辨识 task；估计或回退值直接驱动现有目标力调度器，
  运行产物记录 oracle 隔离声明、探测诊断、保守性、探测位移和后续动态载荷保持指标
- 新增 `docs/friction-estimation.md`，说明当前实现是受触觉力观测能力限制的初始滑移代理，给出算法、
  `noslip_iterations` 辨识前提、标准场景结果和迁移硬件前的适用边界
- 新增 `pgt run force-schedule` oracle 抓取目标力调度：按
  `f_ref=clip(γD/(2μ), f_min, f_max)` 从切向载荷和已知真值摩擦系数生成平均单侧目标力，支持目标力
  变化率限制；提供仅重力保持和 0→2 N 动态注水两个标准 task、可复现运行产物、摩擦裕量与滑移指标
- 目标力调度 task 显式配置 `noslip_iterations=5`，抑制摩擦锥内长时数值爬移；目标力不足的反例仍会
  滑落，避免把求解器后处理误解为额外摩擦或防滑控制
- 新增 `docs/force-scheduling.md`，说明 oracle 边界、平均单侧力公式、标准场景、输出字段与当前验证结果
- 新增 `reports/` Typst 模板与《摩擦感知目标力调度》论文工作稿 `wired_demo.typ`：模板提供
  产物指标表、run 元信息块、矢量插图与 mitex 公式组件，工作稿经本地中文适配的 wired-ieee
  排版；在仓库根用 `typst compile --root . reports/wired_demo.typ` 编译，产物 PDF 不入库
- 新增 `tests/test_report_typst.py` 编译冒烟测试：用 `tests/fixtures/` 迷你数据编译 fixture 报告，
  本机装有 Typst CLI 时自动执行、CI 无 CLI 环境自动跳过
- 摩擦估计与 Ramp 力跟踪实验循环新增可选逐帧 `on_frame` 渲染回调：按视频帧率
  （`render_fps`）回调最新采样行与模型/数据快照，默认关闭，不改变仿真与产物；
  配套新增离屏演示录制 `scripts/demos/record_experiment_demos.py`，把 MuJoCo
  场景与实时曲线面板合成为 16:9 MP4（摩擦估计、Ramp 力跟踪两个演示，产物默认
  `outputs/demos/`，无界面环境需在导入 mujoco 前设置 `MUJOCO_GL=egl`）
- 主包新增 `polars` 与 `scipy` 依赖，作为后续 study 跨条件聚合与显著性检验的基础
- 新增 import-linter 架构契约（`uv run lint-imports` 可单独执行，并由
  `tests/test_architecture_contracts.py` 并入 pytest 门禁）：机器检查主包
  入口→study→runner→实验→领域→共享的分层、场景与控制的相互隔离、以及各共享包
  与设备包的独立性；`tests/test_compat_deprecation.py` 同步校验兼容层警告与转发等价性

### 弃用

- 根路径兼容导出模块（`profiles`、`run_artifacts`、`plotstyle`、`friction_plots`、
  `taxel_friction`、`tactile_slip`、`friction_estimation`、`discrete_force_control`）导入时
  发出 `DeprecationWarning`，计划于 0.4.0 移除；请改用各自规范路径。

### 移除

- 8 个根路径兼容导出模块（`profiles`、`run_artifacts`、`plotstyle`、`friction_plots`、
  `taxel_friction`、`tactile_slip`、`friction_estimation`、`discrete_force_control`）与
  对应兼容性测试在 0.4.0 删除，请使用各自规范路径（`config.profiles`、
  `artifacts.run_artifacts`、`visualization.*`、`perception.*`、`robotiq_grasp_core`）。

## [0.3.0] - 2026-09-04

### 新增

- force-track run 新增 `effective_parameters.json`：记录解析后的完整 profile、task 与实际运行时覆盖，
  在保留 YAML 人工输入快照的同时提供机器可读的有效参数复现依据
- study 新增 `study.resolved.json`，保存路径和默认值均已解析的机器可读配置快照
- force-track 默认时序产物改为 Zstd 压缩的 `trace.parquet`，并采用事件感知降采样：普通控制器常规
  区段为 100 Hz，直接力矩 ADRC 为 250 Hz，状态变化与 waypoint 邻域保留完整控制频率；指标和绘图
  仍使用完整频率数据；新增 `--trace-period` 与 `--event-window` 覆盖参数。读取接口继续
  兼容旧 CSV API 与历史 `trace.csv` 产物。study 的 `summary` 与
  （适用时的）`aggregate` 同时输出 CSV 和 Parquet，diagnosis study 仅输出 summary
- 单次 force-track 按 Step、Ramp、Mixed/Smoothstep 的实验目的分别生成瞬态误差、加载—卸载滞后和
  waypoint 误差诊断；PID 消融、控制器对比、ADRC 调参、刚度估计器对比和因果诊断分别生成专用图表。
  所有图统一输出 600 DPI PNG 和矢量 PDF，并登记到 manifest
- 新增 `adrc-torque-td` 工程对照变体：在论文式二阶直接力矩 ADRC 前加入临界阻尼线性 TD，
  同步整形反馈参考和机构模型前馈，并在 trace 中记录实际使用的参考力、变化率与加速度；原
  `adrc-torque` 保持无 TD，不改变触觉测量进入 LESO 的轻滤波链路，也不扩大默认正式矩阵
- `adrc-torque` 默认观测器带宽由 180 调整为 240 rad/s，与三种接触 preset、三个 seed 的
  调参确认结果一致；专用调参配置同步把 `ωo/ωc=4` 设为新基线
- 二阶直接力矩 `adrc-torque` 新增两阶段调参 study：
  `force_tracking_torque_adrc_tuning.py` 以独立 `TorqueAdrcControl` 覆盖扫描轻度测量滤波、
  控制器带宽和观测器带宽比例；粗扫以连续任务 RMSE 与力矩饱和作为约束，优先降低 Step 超调，
  确认阶段在三种接触 preset、三个噪声 seed 上复验，并记录每个候选的精确参数和排序结果
- 默认控制器对比矩阵移除一阶位置式 `adrc`：该实现仍保留用于历史复现，但不再作为正式 benchmark
  变体；默认条件数由 189 调整为 162
- `adrc-torque` 控制器变体（`--controller-variant adrc-torque`）：接近阶段保留 MIT
  阻抗，跟踪阶段旁路 MIT `kp/kd`，由二阶离散 current LESO 与名义 PD 直接输出
  电机力矩；机构雅可比前馈承担名义静态夹持力，LESO 只观察实际总力矩扣除模型
  前馈后的残差，并使用量化、变化率和幅值限制后的实际力矩更新；目标力一、二阶
  导数进入控制律；LESO 使用独立 40 Hz 一阶轻滤波测量，不复用 PID 与指标的
  20 Hz 公共滤波，trace 新增该测量、LESO 状态、`b0`、原始/受限/残差力矩及限幅诊断；
  默认控制器对比矩阵由 162 扩至 189 条
- profile 新增可选配置段 `control.force.torque_adrc`（`TorqueAdrcControl`）：按在线
  接触刚度、闭合雅可比、名义等效惯量和输入增益尺度调度 `b0`，支持输入增益上下界、
  控制/观测器带宽、测量轻滤波截止频率与力矩变化率限制；该路径要求启用机构几何和
  接触刚度估计，并与一阶 `adrc`、`torque_feedback_gain > 0` 互斥
- `adrc` 控制器变体（`--controller-variant adrc`）：跟踪阶段以一阶线性自抗扰
  （LADRC）外环替换 PID 位置修正——扩张状态观测器估计滤波力与总扰动，控制律输出
  闭合速度并逐周期积分成位置修正（裁剪到 `max_position_adjustment`）；MIT kp/kd
  不被覆盖（位置弹簧阻尼保留，与 `direct-torque` 的本质区别），模型力矩前馈照常、
  刚度估计器照常运行；`force_tracking_controller_comparison` 默认矩阵由 135 扩至
  162 条
- profile 新增可选配置段 `control.force.adrc`（`AdrcControl`：`b0_n_per_m`、
  `controller_bandwidth_rad_s`、`observer_bandwidth_rad_s`、
  `max_closing_velocity_m_s`；默认 `None` 表示不启用，与
  `control.force.torque_feedback_gain > 0` 互斥，同时启用会在控制器构造时抛
  `ValueError`；默认带宽 (40, 120) rad/s 在 step 任务、默认 profile 上按
  “饱和比例不超 5% 中 rmse 最小”从五档候选中选定）
- `direct-torque` 控制器变体（`--controller-variant direct-torque`）：位置式 MIT 力控的
  直接力矩式对照。跟踪阶段力误差经 `torque_feedback_gain` 直接进入 MIT 前馈力矩，
  MIT kp/kd 由控制器逐周期覆盖为 0（接近阶段仍共享 profile 位置伺服增益），PID 与
  刚度位置修正诊断为 0、刚度估计器照常运行；`force_tracking_controller_comparison`
  默认矩阵由 108 扩至 135 条
- profile 新增字段 `control.force.torque_feedback_gain`（默认 0.0，保持位置式行为；
  大于 0 时跟踪阶段启用直接力矩式力控）
- 三类标准力跟踪任务 `step.yaml`、`ramp.yaml` 与 `mixed_waypoints.yaml`，分别覆盖阶跃、线性加载/卸载和
  平台—平滑斜坡综合测试
- `force_tracking_controller_comparison` study：固定展开 controller × task × material × seed，支持
  `--dry-run` 审阅矩阵，并生成聚合指标、饱和比例、消融增量和同 seed 轨迹对比图
- `force_tracking_stiffness_estimator_comparison` study：固定 `pid-stiffness-ff`，比较
  `secant_ewma`、`window_linear` 与 `window_quadratic` 三种刚度估计器，并输出聚合结果和同 seed
  力—刚度对比图
- 新增 `stiff=(-2500,-15)` 显式接触 preset；默认批量研究改用 `medium/hard/stiff`，原
  `soft=(-250,-5)` 仅保留用于兼容和专项标定
- `docs/custom-gripper-next-phase.md`：DM_Gripper 下一阶段实施路线图（Pillars 触觉反馈实验）
- `parallel_gripper_tactile.protocols`：统一的实验时序协议
  `DisturbanceProtocol`（含支撑释放、无支撑保持、切向扰动状态设置）
- `parallel_gripper_tactile.video`：渲染级力箭头、像素保存与 MP4 编码共享工具
- `recording.run_demo_loop`：两个触觉演示共用的驱动循环（viewer 节流、物理推进、采样与记录）
- `AGENTS.md`：面向 AI 编码代理的仓库协作约定入口，汇总语言策略、提交信息规范与验证门禁
- `metrics.json` 新增 `rise_time_s`、`overshoot_ratio`、`settling_time_s` 三个阶跃瞬态指标；
  仅在 `hold` 任务存在合格加载阶跃（跳变不低于 1 N、平台段不低于 0.5 s）时计算，
  无法判定时输出 `null`，study 聚合按 NaN 感知口径统计

### 变更

- `ramp.yaml` 任务在卸载终点后新增 2 s 终端保持段（保持 1 N），tracking 时长由 6 s 延长至
  8 s，用于终端稳态误差统计

### 修复

- 文档站点：`tactile-conventions` 的「高载荷接触的定性结论」小节标题经上次改写后丢失原锚点，
  导致 `architecture`、`workflows`、`force-tracking` 三处跨文档链接失效；现通过 `attr_list`
  为该小节挂显式锚点 `#collision-geometry-conclusions` 并更新三处链接，标题后续再改名也不会断链
- `run_custom_grasp_validation`：首步即失稳时不再因空数据抛 `IndexError`，
  而是以仿真失败状态返回
- `report_taxels`：docstring 明确只读取模型初始状态，避免误导

### 重构

- 三份重复的扰动协议类收敛为 `DisturbanceProtocol`；`release_settle` 相位
  更名为 `unsupported_hold`，比较脚本 CLI 参数改为 `--hold-duration`
- `run_cube_grasp_demo` 与 `run_touch_grid_demo` 共用驱动循环；
  `run_touch_grid_demo` 新增 `--close-control`，消除硬编码控制量
- 箭头/像素/编码工具收敛到 `parallel_gripper_tactile.video`
- `docs/custom-gripper-next-phase.md` 按实现进度重写：各阶段标注
  ✅/⏳/⬜ 状态，修正 `ContactTaxelReader` API 与 `outputs/` 实际布局，
  更新下一次开发建议为 drive 行程复核
- 新脚本注释统一为中文（与 `scripts/`、docs 一致），`verify_mujoco.py`
  移入 `scripts/` 并修正默认模型路径
- `validate_profile` 增加 contact_geom 模式下的 site 存在性校验；
  `load_profile` 改为向上探测仓库根；`tactile_center_in_base` 由 profile
  推导中央 taxel
- `outputs/` 按夹爪与实验类别分子目录（`robotiq/{taxel_demo,touch_grid_demo,
  comparison,disturbance_video}`、`custom_gripper/{validation,disturbance_video,
  experiments}`），脚本默认输出路径与 README、docs 同步更新
- ruff `D` 规则豁免收窄：`scripts/` 与 `tests/` 仅豁免语言相关的 5 条
  （`D400`/`D401`/`D403`/`D404`/`D415`），其余格式类 docstring 检查
  自动生效，并补齐暴露的 32 处 docstring 缺口

## [0.2.0] - 2026-08-19

### 新增

- 触觉仿真泛化为多平行夹爪：类型化 profile（`configs/*.toml`）统一模型路径、执行器、
  控制范围、安装位姿与触觉阵列命名
- DM_Gripper（`custom_parallel_gripper`）接入，左右各 3×3 Pillars 触觉接触 geom
- `pgt-check` 验证 CLI 与 Onshape 导出准备脚本（`prepare_onshape_export.py`）
- 可比触觉传感器模型（taxel 力 / touch-grid / Pillars 接触）与切向扰动基准
- Rerun 触觉仪表盘与对应工作流文档
- 扰动视频录制与发表级绘图脚本（`record_disturbance_video.py`）
- 运行时场景组合（`grasp_scene.py`）与增强重力抓取演示
- 触觉录制与可视化工具（`recording.py`）

### 变更

- 运行时统一为 Python 3.12（`uv`），依赖锁定 `uv.lock`

### 文档

- `docs/architecture.md`、`docs/workflows.md`、`docs/tactile-conventions.md`、
  `docs/onshape-export-upgrade.md`

## [0.1.0] - 2026-07-16

### 新增

- Robotiq 2F-85 指尖触觉资产仓库（taxel 3×3 与 touch-grid 高分辨率触觉模型）
- 方块闭合抓取触觉场景（无限棋盘地面、水平地面侧向抓取）
- viewer 手动控制夹爪与默认实时显示抓取演示
- taxel 检查工具（`view_taxels.py`）

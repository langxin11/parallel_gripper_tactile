# 📌 自研夹爪配置基线与状态

自研夹爪完全由 `configs/custom_parallel_gripper.yaml` 配置：MJCF 来源、法兰安装、MIT 限幅、
法向力控制器，以及 3×3 接触几何触觉布局。

<pre><code class="language-bash">
uv run pgt validate configs/custom_parallel_gripper.yaml
uv run pgt run grasp --profile configs/custom_parallel_gripper.yaml
uv run pgt view grasp --profile configs/custom_parallel_gripper.yaml
</code></pre>

Pillars 使用等效接触参数 `solref="-1200 -10"` 与
`solimp="0.75 0.95 0.0025 0.5 2"`；它们不是独立的硅胶有限元模型。力控主量是平均单侧
法向力 `f_n=(F_L+F_R)/2`，在线 `k_pair` 是 Pillar—物体—机构/接触链路的组合等效刚度，
只用于前馈、增益调度和实验比较。

## 电机与执行器一致性

项目 MIT 映射固定为 PMAX=`1.7 rad`、VMAX=`8 rad/s`、TMAX=`4.0 N·m`。修改这些范围时，
必须同步更新电机寄存器、上位机 profile 的量化范围，以及 MuJoCo actuator 的 `ctrlrange` 和
`forcerange`；只改其中一处会让相同的 CAN 位域对应错误的物理量。prepared MJCF 当前对
`gripper_drive` 使用 ±4 N·m 的 `ctrlrange` 与 `forcerange`。

Onshape 原始导出中的 position actuator 与 `forcerange=12.5` 可用于保留导出信息或短时能力
核对，但 `12.5 N·m` 不等于本项目的连续使用限幅；持续控制以 profile 与 prepared MJCF 的 ±4 N·m
为准。

## 当前验收状态

每次修改 CAD 导出、profile 或控制参数后，至少运行本页开头的 profile 验证和 Viewer 检查。
然后使用相同的 force-tracking task 比较 `f_n` 跟踪误差、限幅比例、接触状态和 `k_pair` 轨迹；
不要将历史计划或单次截图当作配置事实来源。

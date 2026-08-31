# 🧩 Onshape to Robot：曲柄滑块夹爪 MJCF 导出

本页只保留当前工程可复现流程；截图、探索记录和操作历史保留在飞书导出文档。

## CAD 命名与闭环

在 Assembly 的 Mate feature 上命名：`dof_*` 导出普通运动自由度，`fix_*` 合并固定刚体，
`closing_*` 标记需恢复的运动学闭环，`frame_*` 导出为 MJCF `<site>`。`*_inv` 用于明确反转
关节轴方向。曲柄滑块模型中应仅有 `dof_gripper_drive` 作为主动关节。

对 `closing_*` 的 Revolute Mate，导出器应生成中心点和 Z 轴偏移点各一对的 `<connect>`，
以约束销轴中心与轴线方向，同时保留绕轴转动。Mate Connector 的 Z 轴必须沿 Revolute 的转轴；
Slider 的 Z 轴必须沿导轨方向。导出后先核对该映射与实际 XML，而不要只依据 CAD 命名推断。

## 导出、后处理与验收

Onshape to Robot 可将 `dof_gripper_drive` 导出为 position actuator；项目实际使用的 actuator
类型、`ctrlrange`/`forcerange` 和连续控制限幅必须以仓库 profile 与 prepared MJCF 为准。原始导出中
出现的 `forcerange=12.5` 只是导出基线或峰值能力参考，不是项目持续控制限幅。

每次重新导出后，用 `pgt assets prepare-onshape` 为 3×3 Pillar 碰撞 geom 标注稳定的行优先
（row-major）名称：

<pre><code class="language-bash">
uv run pgt assets prepare-onshape RAW.xml PREPARED.xml
</code></pre>

把 `configs/custom_parallel_gripper.yaml` 指向整理后的资产，然后执行：

<pre><code class="language-bash">
uv run pgt validate configs/custom_parallel_gripper.yaml
uv run pgt view grasp --profile configs/custom_parallel_gripper.yaml
</code></pre>

该整理命令保留模型几何，只赋予接触几何触觉读取器所需的稳定
`left_taxel_geom_RC` / `right_taxel_geom_RC` 碰撞名称。验收时检查：仅有预期主动 actuator、每个
`closing_*` Revolute 闭环有中心与 `_z` 的 connect 对、`frame_*` 已成为 site、左右手指能平行开合，
并且在 Viewer 中没有明显的闭环撕裂或持续数值振荡。求解器、equality 和接触参数必须从当前模型读取；
任何调整都应经上述验证，而不承诺“最佳”闭链稳定性。

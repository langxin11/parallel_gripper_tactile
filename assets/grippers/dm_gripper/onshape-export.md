<!-- --8<-- [start:content] -->
# DMgripper：Onshape 导出、后处理与验收

本页维护当前模型的 CAD 命名、导出输入和验收流程。模型变体与运行入口见
[DMgripper 资产说明][model]，Pillar 坐标与读数映射见[触觉资产说明][tactile]。

> 导出会改写模型与网格。先检查工作区中需要保留的修改；导出、触觉命名和实验加载是三个不同步骤。
> 原始导出不自动具备当前控制器要求的执行器类型与全部物理参数。

## 1．准备 Assembly

第一个实例会被 exporter 当作基座，应让 `base` 排在第一位，并用 Onshape 的 **Fixed** feature 固定到世界。
按下列刚体组织 Assembly，避免标准件意外带来额外自由度。

| 实例或子装配 | MJCF 结果 | 组织方式 |
| --- | --- | --- |
| `gripper_base` | 固定 base body | 定子、连接架和左右导轨用 Fastened 合并。 |
| `drive_crank` | 转动 body | 电机转子与驱动曲柄合为一个刚体。 |
| `left_link`、`right_link` | 左右连杆 body | 分别保留转动连接。 |
| `left_finger`、`right_finger` | 左右滑台 body | 滑台与指体分别固定合并。 |

需要稳定的 link 名称时，可在实例上设置 `link_gripper_base`、`link_drive_crank`、
`link_left_finger` 等具名 Mate Connector。

## 2．命名关节、闭环与坐标系

名称应设置在 Assembly 的 **Mate feature** 上。`dof_` 导出普通关节，`fix_` 合并固定刚体，
`closing_` 导出闭环 equality，`frame_` Mate Connector 导出为 site 并移除该前缀。

| 两端刚体 | Mate 类型 | 建议名称 |
| --- | --- | --- |
| base ↔ drive crank | Revolute | `dof_gripper_drive`。 |
| rotor ↔ drive crank | Fastened | `fix_crank_to_rotor`。 |
| drive crank ↔ left link | Revolute | `dof_left_link_crank_pin`。 |
| base ↔ left finger | Slider | `dof_left_finger_slide`。 |
| left link ↔ left finger | Revolute | `closing_left_link_finger_pin`。 |
| drive crank ↔ right link | Revolute | `dof_right_link_crank_pin`。 |
| base ↔ right finger | Slider | `dof_right_finger_slide`。 |
| right link ↔ right finger | Revolute | `closing_right_link_finger_pin`。 |

- Revolute 两端 Mate Connector 原点位于销轴中心，Z 轴沿转轴。
- Slider 的 Z 轴沿导轨运动方向，行程限位包含默认姿态。
- 被动曲柄销可设置宽松限位，例如 `-180°` 至 `180°`；默认姿态不得存在不合理干涉或预紧。
- 开合方向相反时，可用 `dof_gripper_drive_inv` 翻转轴向，导出关节仍为 `gripper_drive`。
- 左右触觉坐标系用 `frame_left_taxel_00...22` 和 `frame_right_taxel_00...22` 命名。

> `closing_` 不用于 Slider 副。当前流程在“连杆—滑块”的 Revolute 副处断开闭环，
> 不要同时给该副再加 `dof_`。每个 Revolute 闭环必须有中心点与 `_z` 点两条 connect。

## 3．配置并导出

`config.json` 记录 workspace、Assembly 标签页、输出名称和碰撞白名单。
先确认 URL 指向要导出的 Assembly；重新导出需要独立安装 `onshape-to-robot[mujoco]` 并配置 Onshape 凭据。
在具备这些依赖的环境中，从仓库根目录执行：

```bash
uv run onshape-to-robot assets/grippers/dm_gripper
uv run onshape-to-robot-mujoco assets/grippers/dm_gripper
```

`joint_properties` 使用移除 `dof_` 后的关节名。只有 `gripper_drive` 为主动关节，
四个被动关节均设为 `"actuated": false`。

当前碰撞白名单为：

```json
"ignore": {
  "*": "collision",
  "!Pillars*": "collision",
  "!base": "collision",
  "!stator": "collision",
  "!bracket*": "collision",
  "!*MGN9 Rail": "collision"
}
```

原始模型有左右各九个 Pillar 碰撞 mesh，另有 base、MGN9 rail、stator 和 bracket 四个外壳碰撞 mesh。
motor、crank 和传动标准件只用于可视化，避免内部传动碰撞。

**执行器必须另行核对。**当前导出配置声明 position actuator；当前 prepared 模型使用 `<motor>`，
输入为输出轴 N·m，限幅为 `±4 N·m`。`prepare-onshape` 不执行这项转换，也不自动恢复主模型的电机参数。
原始导出或旧版本中的峰值 `forcerange` 不能替代项目持续控制限幅，依据见[电机建模摘要][motor-model]。

## 4．命名触觉碰撞并生成默认球体模型

先对原始导出执行后处理：

```bash
uv run pgt assets prepare-onshape \
  assets/grippers/dm_gripper/parallel_gripper.xml \
  assets/grippers/dm_gripper/parallel_gripper_prepared.xml
```

该命令只为碰撞 geom 赋予稳定的 `left_taxel_geom_RC`、`right_taxel_geom_RC` 名称，不修改几何。
当 18 个具名 taxel site 齐全时，使用 geom 与 site 位置的最小距离一对一分配；缺失 site 时有位置排序回退，
但触觉读取器仍需要完整的对应 site，因此回退不等于模型通过了运行时验收。

默认球体模型生成脚本仍位于根目录 `scripts/`：

```bash
uv run python scripts/prepare_height_sphere_collision_model.py \
  --source assets/grippers/dm_gripper/parallel_gripper_prepared.xml \
  --output assets/grippers/dm_gripper/parallel_gripper_height_sphere_collision.xml
```

脚本将 18 个 Pillar 碰撞 geom 替换为半径 `0.0028 m` 的球体，并逐点采用对应 taxel site
位置，因此保留高度差。脚本不承担 motor 转换或电机参数恢复，输出继承输入模型的机构与
执行器定义。共面 mesh 与共面球体变体已经退役；历史实验依据见[模型验证][validation]。

## 5．接入并验收

让 `configs/model/dm_gripper/<name>.yaml` 的 `model.path` 指向整理后的资产，再用相同 model
组合执行领域与资源校验、打开 Viewer。以下以现有默认变体为例：

```bash
uv run python scripts/research/run.py \
  model=dm_gripper/height_spheres execution=plan
uv run pgt view grasp --set model=dm_gripper/height_spheres
```

| 检查范围 | 通过条件 |
| --- | --- |
| 关节与执行器 | 五个预期关节存在，仅 `gripper_drive` 有执行器，类型、输入单位与限幅符合当前控制器。 |
| 闭环 | 左右各有中心与 `_z` 的 connect 对，共四条。 |
| 碰撞 | 原 mesh 基线有 18 个 Pillar 和四个外壳碰撞 mesh；变体按声明替换 Pillar 碰撞几何。 |
| 触觉 | 18 个稳定 geom 名与对应 site 齐全，且没有意外的 `frame_freejoint`。 |
| 运动 | 低、中、高目标位置下左右手指平行开合，无非有限状态、明显跳变、闭环撕裂或持续振荡。 |

Viewer 中按 `3` 显示或隐藏 collision group 3，按 `C` 显示接触点，按 `F` 显示接触力。
求解器、equality 和接触参数以实际加载模型为准；任何调整都需要上述检查，不预设通用的“最佳参数”。

## 排障

| 症状 | 优先检查 |
| --- | --- |
| 重新导出后触觉 geom 名消失 | 再执行后处理，并确认 `_z` 闭环约束没有丢失。 |
| 模型发散或闭环松软 | Mate Connector 原点、Z 轴、默认姿态；核对后再调整 equality 的 `solref`、`solimp`。 |
| Onshape 中可动，导出后卡住 | 关节轴向，以及 mate limits 是否包含默认姿态。 |
| 出现额外自由度 | 转子与曲柄、滑台与指体，以及 frame 与 base 是否固定或合并。 |
| 运行控制器时输出异常 | 是否误用 position actuator，控制输入单位及限幅是否匹配。 |
| 高载荷接触抖动 | 查看[模型验证与接触边界][validation]，区分机构错误与碰撞几何／求解器交互。 |
<!-- --8<-- [end:content] -->

[model]: README.md
[tactile]: README.md#tactile-model
[motor-model]: DM_J4310P_24V_MJCF_建模摘要.md
[validation]: ../../../docs/control-comparison-ablation.md#collision-geometry-conclusions

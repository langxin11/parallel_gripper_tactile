# Onshape 导出与升级建议

本文记录自研曲柄滑块平行夹爪从 Onshape 到 MuJoCo 的稳定契约，避免模型重新导出后
与代码、文档和触觉通道再次失配。

## 当前模型契约

Onshape 文档：

```text
document:  ee6a871f9e00fc227503180b
workspace: aa505c6adc3fefb88a217362
element:   16b75a7cc71174b7cd5960db
```

建议保留的配合名称：

```text
fix_stator_base
dof_gripper_drive
fix_crank_to_rotor
fix_bracket_to_stator
fix_guide_to_bracket
fix_left_finger_slider
closing_left_finger_pin
dof_left_link_crank_pin
dof_right_link_crank_pin
fix_right_finger_slider
closing_right_link_finger_pin
dof_left_finger_slide
dof_right_finger_slide
```

名称约定：`fix_*` 表示刚性连接，`dof_*` 表示导出的树内自由度，`closing_*` 表示
通过 MuJoCo equality 闭合的树外连接。

## 必须优先修正

### 1. 消除第二个根节点

当前导出仍可能报告：

```text
Found 2 root nodes: base, frame
```

并给 `frame` 生成 `freejoint` 和极小占位质量。在无重力验证中不明显，但装入真实抓取
场景后它会成为独立自由物体。若 frame 是结构件，应在 Onshape 增加 `fix_frame_base`；
若只是参考几何，应抑制或从仿真导出中排除。

完成标准：导出日志只剩一个 root node，MJCF 中不存在 `frame_freejoint`。

### 2. 补充被动转动副范围

为以下配合设置覆盖完整机构运动、但不过度宽松的限制：

```text
dof_left_link_crank_pin
dof_right_link_crank_pin
```

可先用 `-180°～180°` 消除无界警告，再根据运动包络收紧。不要给闭环销轴增加 actuator。

### 3. 复核主动关节机械范围

当前导出约为 `[-0.1056, 1.9888] rad`，而高端附近会受到闭环机构和接触约束影响。
profile 暂采用 `-0.05～1.85 rad` 作为实验控制范围。最终范围应由 Onshape 干涉检查、
实机限位和 MuJoCo 扫描共同确定。

## 质量与材料

所有参与动力学的零件都应在 Onshape 指定材料。建议起点：

- 结构铝件、支架、曲柄：6061-T6 铝，密度约 `2700 kg/m³`；
- 钢制导轨、轴承、紧固件：钢，密度约 `7850 kg/m³`；
- 塑料外壳或打印件：按实际材料选择，例如 PLA 约 `1240 kg/m³`；
- 电机、滑块、轴承等采购件：优先填写厂家质量，而不是仅按包络体积估算；
- Pillars 若代表硅胶/弹性触觉柱：使用实际配方密度；缺数据时可从
  `1050～1200 kg/m³` 做灵敏度分析。

质量警告的正确修复位置是 CAD/BOM。不要依赖导出器生成的 `1e-9 kg` 占位惯量作为
正式动力学参数。

## 驱动器

`dof_gripper_drive` 是唯一控制自由度。当前 MuJoCo 使用位置执行器：

```text
actuator:  gripper_drive
kp:        20
forcerange: ±12.5 N·m
```

DM-J4310P-2EC 的 `12.5 N·m` 是峰值输出，适合短时限幅；连续夹持与热稳态实验应使用
额定 `3.5 N·m`，或加入持续时间/温升降额模型。控制量是目标角度，不是夹持力。

## 碰撞与触觉

只让 18 个 Pillars STL 参与接触：

```json
"ignore": {
  "*": "collision",
  "!Pillars*": "collision"
}
```

左右各 9 个 Pillars 是实际 3×3 触觉测量位置。导出器不会产生稳定的 geom 实例名，
因此每次导出后运行：

```bash
uv run scripts/prepare_onshape_export.py RAW.xml PREPARED.xml
```

脚本根据两个滑块 body 中碰撞 geom 的局部中心排序并写入：

```text
left_taxel_geom_00  ... left_taxel_geom_22
right_taxel_geom_00 ... right_taxel_geom_22
```

不要额外叠加球形 taxel，否则会改变首次接触位置和真实 Pillars 几何。

## 闭环 equality

两条物理闭环各导出两条 `connect` equality：一条约束连接点重合，另一条 `_z`
约束轴向。因此 XML 中四条 equality 对应两个闭环，不是四个独立机构闭环。

保留：

```text
closing_left_finger_pin
closing_left_finger_pin_z
closing_right_link_finger_pin
closing_right_link_finger_pin_z
```

## 推荐导出流程

1. 在 Onshape 检查单根节点、配合范围、材料、质量和干涉。
2. 导出到临时目录，不直接覆盖仓库中的已验证模型。
3. 保存导出日志，并确认 5 个自由度、2 个物理闭环和 18 个 Pillars。
4. 运行 `prepare_onshape_export.py` 恢复稳定触觉名称。
5. 对比旧新 XML 的关节范围、质量、惯量、执行器、equality 和碰撞数量。
6. 更新 `assets/grippers/custom_parallel_gripper/`。
7. 运行 `uv run pgt-check configs/custom_parallel_gripper.toml` 和完整测试。
8. 在 viewer 中检查碰撞组、接触点、接触力与完整开闭行程。

只有上述验证全部通过后，新的导出资产才应提交到主分支。

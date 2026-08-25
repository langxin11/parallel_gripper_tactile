# Onshape 导出与升级

本页定义自研曲柄滑块平行夹爪从 Onshape 导出到仓库资产的验证流程。目标是让新的 CAD
导出在质量、闭环、关节、碰撞和 3×3 Pillars 触觉命名上保持与 Python 代码兼容。

## 受版本控制的运行资产

`configs/custom_parallel_gripper.toml` 引用：

```text
assets/grippers/custom_parallel_gripper/parallel_gripper_prepared.xml
```

该文件是脚本和测试实际加载的 MJCF。不要用未经检查的新导出直接覆盖它；先在临时位置完成
后处理和验证。

## 导出要求

- 机构保持单一根节点；参考 frame 不应以独立自由体导出。
- `gripper_drive` 是主动输出轴，并与 profile 中的 MIT 控制范围相容。
- 被动销轴与滑块的关节范围应反映真实机械包络。
- 两条机构闭环以四条 MuJoCo `connect` equality 表示；每个物理销轴使用一对约束。
- 所有动力学零件都应具备真实或可追溯的质量与惯量，不能依赖极小占位质量。
- 左右各 9 个 Pillars 是指尖唯一的触觉碰撞通道；不要另行叠加球形 taxel。

电机的物理保护范围由 MJCF 的 `<motor>` 给出，常用命令范围由 profile 的
`control.mit` 给出。两者都应在修改导出或驱动参数后重新检查。

## 推荐流程

1. 从 Onshape 导出到临时文件，不直接覆盖仓库资产。
2. 确认 mesh、质量、关节、闭环和碰撞过滤符合设计。
3. 运行后处理，为 Pillar 碰撞 geom 恢复行优先的稳定名称：

   ```bash
   uv run scripts/prepare_onshape_export.py RAW.xml PREPARED.xml
   ```

4. 使用新文件进行模型检查：

   ```bash
   uv run scripts/verify_mujoco.py --mjcf PREPARED.xml
   ```

5. 比对新旧 XML 的关节范围、惯量、执行器、equality 与主动碰撞 geom，再将通过验证的
   文件更新为 `parallel_gripper_prepared.xml`。
6. 运行项目契约检查和回归测试：

   ```bash
   uv run pgt-check configs/custom_parallel_gripper.toml
   uv run pytest
   ```

7. 使用查看器检查实际安装姿态和开闭过程：

   ```bash
   uv run scripts/view_custom_grasp_scene.py
   uv run scripts/view_custom_grasp_scene.py --closed
   ```

## Pillar 命名契约

后处理后的碰撞 geom 必须按滑块局部坐标行优先命名：

```text
left_taxel_geom_00  ... left_taxel_geom_22
right_taxel_geom_00 ... right_taxel_geom_22
```

每个 geom 还需要对应的 `left/right_taxel_00...22` site。`ContactTaxelReader` 依此解析
通道；命名、网格尺寸或 site 缺失会由 `pgt-check` 及测试尽早报告。

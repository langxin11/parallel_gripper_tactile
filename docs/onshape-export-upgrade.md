# Onshape 导出与整理

每次从 Onshape 重新导出后，用 `pgt assets prepare-onshape` 为 3×3 Pillar 碰撞 geom 标注
稳定的行优先（row-major）名称：

```bash
uv run pgt assets prepare-onshape RAW.xml PREPARED.xml
```

把 `configs/custom_parallel_gripper.yaml` 指向整理后的资产，然后执行：

```bash
uv run pgt validate configs/custom_parallel_gripper.yaml
uv run pgt view grasp --profile configs/custom_parallel_gripper.yaml
```

该整理命令保留模型几何，只赋予接触几何触觉读取器所需的稳定
`left_taxel_geom_RC` / `right_taxel_geom_RC` 碰撞名称。

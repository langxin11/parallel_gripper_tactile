# 自研夹爪状态与后续工作

自研夹爪完全由 `configs/custom_parallel_gripper.yaml` 配置：MJCF 来源、法兰安装、MIT 限幅、
法向力控制器，以及 3×3 接触几何触觉布局。

```bash
uv run pgt validate configs/custom_parallel_gripper.yaml
uv run pgt run grasp --profile configs/custom_parallel_gripper.yaml
uv run pgt view grasp --profile configs/custom_parallel_gripper.yaml
```

Pillars 使用保留的等效接触参数 `solref="-6000 -10"` 与
`solimp="0.75 0.95 0.0025 0.5 2"`；它们不是独立的硅胶有限元模型。

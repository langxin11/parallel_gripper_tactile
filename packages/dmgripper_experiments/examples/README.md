# dmgripper-experiments 示例

示例调用与 CLI 相同的公共接口，默认全部离线：只验证配置、打印计划，
不创建设备、不使能电机、不生成正式运行产物。

- `force_curve.py`：给定时间力曲线任务示例（恒定与上升段）。
- `adaptive_grip.py`：触觉动态增力任务示例。
- `stiffness_diagnostics.py`：离线读取既有运行目录的刚度估计诊断。

从仓库运行：

```bash
uv run --package dmgripper-experiments python packages/dmgripper_experiments/examples/force_curve.py
uv run --package dmgripper-experiments python packages/dmgripper_experiments/examples/adaptive_grip.py
uv run --package dmgripper-experiments python packages/dmgripper_experiments/examples/stiffness_diagnostics.py <运行目录>
```

正式 YAML 只在 `configs/hardware/dmgripper/` 维护一份权威副本；示例
不复制完整配置。

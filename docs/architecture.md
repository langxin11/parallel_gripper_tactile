# 项目架构

仓库使用单向依赖图：

```text
config → tactile / scenes / simulation → experiments / analysis / io → CLI
```

`parallel_gripper_tactile.config` 由 `profiles.py` 中的 Pydantic 模型表示。YAML profile 是
冻结的（frozen）、拒绝未知字段、按 `mode` 区分控制与触觉后端，并以自身所在目录解析路径。

`tactile.py` 定义力传感器、接触几何与 touch-grid 传感器共用的读取器协议。每个读取器返回局部
`(3, rows, cols)` 数组，正的 `Fz` 表示压缩。

`simulation.py` 独占 MuJoCo 步进，并维护相互独立的物理、控制、采样与渲染节奏。实验产出共享
样本，因此无界面校验、绘图与录制使用同一条状态轨迹。

CLI 只负责展示。它创建独占运行目录并生成 Rich 诊断；不提供被包内模块消费的 API。

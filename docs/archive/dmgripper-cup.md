# DMgripper 真机倒水实验（历史数据说明）

> **状态：历史兼容说明。** `dmgripper-cup` 已被 `dmgripper-run` 取代并拒绝执行。
> 当前实验方法见 [`../dmgripper-experiments.md`](../dmgripper-experiments.md)。

本页只解释旧 cup v1／v2 运行目录，不能用于启动真机实验。旧流程中的 `pour` 表示操作者第二次确认
撤手后开始的定时阶段；它不等同于通用生命周期的 `active`，也不能与新实验的任务时间直接比较。

## Trace 兼容语义

- 旧 trace 使用 `state`，当前读取器在缺少 `phase` 时以同值回填。
- v2 的 `force_deadband_active` 表示导纳进入力死区后冻结期望闭合量与虚拟速度；
  `unloading_blocked` 表示阻止了导纳反向卸载。v1 没有这两个字段。
- `time_s` 是旧实验启动后的主机单调时间，`tactile_received_at_s` 是主机接收时间，
  `tactile_timestamp_us` 是设备时间；这些时间源不可互换。
- `position_rad` 等字段来自对应命令返回的反馈，`q_des_rad` 等字段是协议量化前的受限请求，
  不能把请求值解释为电机实际执行结果。

当前通用绘图器仍可读取这些记录：

```sh
uv run --package dmgripper-experiments dmgripper-plot --repaint <历史运行目录>
```

兼容别名 `dmgripper-cup-plot <历史运行目录>` 会在源目录生成图像；新分析优先使用上面的
`dmgripper-plot --repaint`，以免改写历史目录。

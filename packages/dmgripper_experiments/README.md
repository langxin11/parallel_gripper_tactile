# DMgripper 真机抓取实验

按物体 1 奶龙玩偶、物体 2 水瓶、物体 3 硬质方盒依次执行人工预载、
摩擦候选与冻结先验自适应试验。各物体模板均是未经标定的 `preload` 候选。
完整流程见[真机实验说明](../../docs/dmgripper-experiments.md)。

在仓库根目录离线检查配置：

```bash
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/object_1.yaml
```

设备与急停准备就绪、传感器完全无负载后，在交互终端运行：

```bash
uv run --package dmgripper-experiments dmgripper-run \
  --config configs/hardware/dmgripper/object_1.yaml --bias --execute
```

终端输入 `start` 启动，`release` 请求受限回位。`friction` 阶段只有在
`active`／`holding` 期间接受 `slip-left`、`slip-right`、`slip-both` 人工起滑标记。
离线候选命令是 `dmgripper-calibrate-friction RUN_DIR`；可用
`--window-s 0.2` 修改取样窗口，或用 `--slip-time-s SEC --side left|right|both`
显式修正标记时间。默认独占写入 `RUN_DIR/friction_candidate.json`，不改 YAML。
候选是该抓取条件的有效承载比，不是材料真摩擦系数；未知侧保持空值。

每次运行目录包含 `config.json`、统一 ZSTD indexed `recording.mcap`、
`manifest.json` 和诊断图。MCAP 的 `/tactile`、`/trace`、`/events` 可按
统一时间轴查看。`completed` 只表示释放、回位和收尾完成；人工抓取评价
在 manifest 中仍标为未评估。

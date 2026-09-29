# 真机实验离线示例

先从仓库根目录检查物体 1 的预载候选配置：

```bash
uv run --package dmgripper-experiments dmgripper-run --config configs/hardware/dmgripper/object_1.yaml
```

完成独立的人工预载试验并记录接受依据后，在同一物体配置中将 `stage` 改为
`friction`，填写 `reference.preload_source`。在真机实验的 `active`／`holding`
阶段，观察物体相对滑动时输入 `slip-left`、`slip-right` 或 `slip-both`。
离线生成候选：

```bash
uv run --package dmgripper-experiments dmgripper-calibrate-friction outputs/real/<运行目录>
```

如需修正按键反应时间，使用 `--slip-time-s <运行相对秒数> --side left|right|both`。
候选只代表该物体、抓取位置和条件下的整侧有效承载比；人工接受后，
在 `adaptive` 阶段填写左右 `reference.friction.left_mu`、`right_mu`、`source`。
未观察到滑动的一侧保持空值，不推定为另一侧相同。

已有运行的刚度诊断可使用 `stiffness_diagnostics.py <运行目录>` 离线查看。

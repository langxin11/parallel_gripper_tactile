# 🚀 常用工作流

使用单一 `pgt` 入口。Profile 为 YAML，并在模型编译前完成校验。

<pre><code class="language-bash">
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt assets generate-taxels --shape box
uv run pgt assets generate-touch-grid
uv run pgt run demo --profile configs/robotiq_2f85_touch_grid.yaml
uv run pgt compare tactile \
  --left-profile configs/robotiq_2f85_box.yaml \
  --right-profile configs/robotiq_2f85_touch_grid.yaml
uv run pgt run grasp --profile configs/custom_parallel_gripper.yaml --video
uv run pgt run force-track \
  --profile configs/custom_parallel_gripper.yaml \
  --task configs/force_tracking/default_waypoints.yaml
uv run pgt compare contact --profile configs/custom_parallel_gripper.yaml
</code></pre>

动态目标力跟踪任务的配置、两阶段流程和指标解读见[动态目标力跟踪](force-tracking.md)。
控制算法对比、消融矩阵和项目分工见[控制算法对比与消融](control-comparison-ablation.md)。

用 `pgt runs list` 查看既有产物。用 `pgt runs clean --all` 预览要删除的目标；确认目标后
再加 `--apply`。

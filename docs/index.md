# 平行夹爪指尖触觉仿真

本工具包使用经过校验的 YAML profile 与 `pgt` 命令行接口。建议按以下路线阅读并执行：

1. [触觉读数约定](tactile-conventions.md)：统一仿真与实机到 `F_L`、`F_R`、`f_n` 的输入语义；
2. [Onshape to Robot：曲柄滑块夹爪 MJCF 导出](onshape-export-upgrade.md)：确认 CAD 命名、闭环与导出验收；
3. [曲柄滑块力控模型](crank-slider-force-control.md)：了解 `f_n`、`k_pair`、雅可比和限幅；
4. [动态目标力跟踪](force-tracking.md)：运行两阶段基准并理解接触状态；
5. [控制算法对比与消融](control-comparison-ablation.md)：在一致任务下比较结果。

工程状态与电机/执行器基线见[自研夹爪配置基线与状态](custom-gripper-next-phase.md)。

<pre><code class="language-bash">
uv sync
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt validate configs/custom_parallel_gripper.yaml
uv run pgt run demo --profile configs/robotiq_2f85.yaml
uv run pgt run force-track --profile configs/custom_parallel_gripper.yaml --task configs/force_tracking/default_waypoints.yaml
</code></pre>

参见[常用工作流](workflows.md)、[动态目标力跟踪](force-tracking.md)、
[控制算法对比与消融](control-comparison-ablation.md)与[项目架构](architecture.md)。

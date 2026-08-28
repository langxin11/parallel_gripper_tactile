# 平行夹爪指尖触觉仿真

本工具包使用经过校验的 YAML profile 与 `pgt` 命令行接口。

<pre><code class="language-bash">
uv sync
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt validate configs/custom_parallel_gripper.yaml
uv run pgt run demo --profile configs/robotiq_2f85.yaml
</code></pre>

参见[常用工作流](workflows.md)与[项目架构](architecture.md)。

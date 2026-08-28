# 代码与注释规范

本页规定仓库的注释、docstring 与文档书写规范，目标是让中文开发者以一致、可维护的方式继续开发。
它与静态检查工具（ruff）共同构成提交前门禁：工具管格式，规范管语言与表达。

## 语言策略

**说明性文字一律使用简体中文；技术性名称保留英文。** 具体分工：

| 类别 | 语言 | 示例 |
| --- | --- | --- |
| 模块/类/函数 docstring 的说明文字 | 中文 | `"""从 MuJoCo 数据中读取当前触觉力。"""` |
| 行内 `#` 注释 | 中文 | `# 只在样本时钟到点时采样，避免重复记录。` |
| 代码标识符、API、类/函数名 | 英文 | `SimulationSession`、`read()`、`sample_period_s` |
| 命令行参数与子命令 | 英文 | `pgt run grasp`、`--profile`、`--video` |
| 物理量符号、单位、`solref`/`solimp` 等求解器参数 | 英文 | `Fz`、`solref="-1200 -10"` |
| MuJoCo/资产对象名 | 英文 | `left_taxel_geom_00`、`touch_grid` |

原则：中文读起来是"这句话什么意思"，英文标识符是"程序里就是这个名字"。两者不混写。
首次出现的陌生术语可附中文括注，例如：`mesh-SDF（网格有符号距离场）`。

## docstring 规范

采用 **Google 风格**（`project.toml` 中 `[tool.ruff.lint.pydocstyle] convention = "google"`）。

### 结构

- 模块、类、函数都应有 docstring；模块级一句话概括模块职责。
- 摘要行直接陈述，无需重复函数名或冠以 "This function…"。
- 内容较多时，摘要后空一行再写详细说明；详细说明也可用中文。
- 参数、返回、异常用 Google 节标题，**节标题保持英文字面值**（pydocstyle 依赖）：

  <pre><code class="language-python">
  def sample(self, dt_s: float) -> SampleT:
      """返回一个采样。

      Args:
          dt_s: 距上次采样的时间步长（秒）。

      Returns:
          按样本时钟到点时的实验采样。
      """
  </code></pre>

  `Args:`、`Returns:`、`Raises:`、`Yields:`、`Note:` 等标题必须原样保留英文，节内文字用中文。

### 标点

- 中文说明文字使用全角标点：`，。：；（）`。
- 行内出现的代码、标识符、命令行、物理量用半角标点，并保留在反引号或代码体内。
- 不应在中文句子中间嵌入英文整句；术语与符号除外。

## 行内注释规范

- 说明「为什么」或「边界条件」，而不是「代码在做什么」。
- `#` 号后加一个空格；注释与被注释代码保持合理对齐与缩进。
- 简短、克制；一行能说清的不拆多行。
- 用简体中文书写，统一全角标点。

## 与静态检查的配合

ruff 已对 `src/**` 与 `tests/**` 豁免 `D400/D401/D403/D404/D415`。这几条规则针对英文语法与
ASCII 标点（例如请求式语气、首词大写、句末英文句号），对中文 docstring 会误报，因此豁免；
**其余格式类 `D` 规则仍由 CI 强制执行**（涵盖 docstring 存在性、首行摘要、缩进、节标题拼写等）。

中文 docstring 的核心约定（首行摘要、Google 节标题、多行空行分隔）仍然遵循 Google 风格，
只是不要求英文语法层面的首词大写 / 句末英文句号。

## 提交前检查

<pre><code class="language-bash">
uv run ruff check .            # 环境缓存异常时：uv run ruff check --no-cache .
uv run ruff format --check .
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest
</code></pre>

版本管理、更新日志与发布按 [Keep a Changelog](https://keepachangelog.com/zh-CN/) 与
[语义化版本](https://semver.org/lang/zh-CN/) 执行。

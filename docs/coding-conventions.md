# ✍️ 代码与注释规范

## 语言策略

说明文字、注释与 docstring 使用简体中文和全角标点；标识符、API、CLI 参数、物理量、单位及资产名
保留英文原名。代码内使用半角标点，正文中的代码放在反引号内；陌生术语首次出现可附中文解释。

## docstring 规范

采用 `pyproject.toml` 指定的 Google 风格。

### 结构

模块、类、函数应有 docstring；首行直接概括职责，不重复名称。详细说明与摘要空行分隔。
`Args:`、`Returns:`、`Raises:`、`Yields:`、`Note:` 等节标题保留英文，节内说明用中文。

```python
def sample(self, dt_s: float) -> SampleT:
    """返回当前采样。

    Args:
        dt_s: 距上次采样的时间步长（秒）。

    Returns:
        样本时钟到点时的实验采样。
    """
```

### 标点

中文使用 `，。：；（）`；代码、命令和物理量保留自身半角标点。除术语和符号外，不在中文句中嵌入英文整句。

## 行内注释规范

解释原因与边界，不复述代码；简短、克制。`#` 后留一个空格，保持正确缩进，使用中文全角标点。

## 与静态检查的配合

Ruff 豁免中文代码区域的 `D400/D401/D403/D404/D415`，避免英文语法与 ASCII 标点误报；
其他 docstring 格式检查仍生效，范围以 `pyproject.toml` 为准。

## 验证与维护入口

门禁见[测试策略](testing.md)，文档更新判断见仓库根目录 `CONTRIBUTING.md`。
更新日志与版本遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/) 和
[语义化版本](https://semver.org/lang/zh-CN/)。

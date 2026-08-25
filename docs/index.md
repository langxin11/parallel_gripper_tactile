# 平行夹爪指尖触觉仿真

面向 MuJoCo 的多平行夹爪触觉资产与实验工具。项目包含 Robotiq 2F-85
参考模型、自研曲柄滑块平行夹爪，以及用于统一加载、控制、触觉读取和验证的 Python 库。

## 快速开始

从仓库根目录安装项目及开发工具：

```bash
uv sync --all-groups
```

检查两个夹爪 profile：

```bash
uv run pgt-check configs/robotiq_2f85.toml
uv run pgt-check configs/custom_parallel_gripper.toml
```

运行测试：

```bash
uv run pytest
```

## 文档站开发

启动本地文档站：

```bash
uv run zensical serve
```

服务启动后访问终端显示的本地地址。编辑 `docs/` 中的 Markdown 文件并保存后，
预览会自动热重载。

构建可部署的静态站点：

```bash
uv run zensical build
```

默认构建输出为 `site/`，该目录已生成且不应提交到版本控制。

## 内容导航

- [项目架构](architecture.md)：资产、profile、核心库和实验脚本的分层关系。
- [触觉读数约定](tactile-conventions.md)：坐标系、符号与张量表示。
- [常用工作流](workflows.md)：生成、仿真、记录与可视化命令。
- [Onshape 导出与升级](onshape-export-upgrade.md)：资产导出及验证流程。
- [自研夹爪状态与后续工作](custom-gripper-next-phase.md)：当前能力边界和后续工作。

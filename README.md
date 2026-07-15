# Robotiq 2F-85 Tactile

独立维护 MuJoCo 中为 Robotiq 2F-85 两个指尖添加触觉传感器的最小资产仓库。

## 内容与边界

- `assets/robotiq_2f85/2f85.xml`：未修改的基础夹爪模型。
- `scripts/generate_taxels_xml.py`：从基础模型生成带 18 个离散 taxel 的派生 MJCF。
- `assets/robotiq_2f85/2f85_taxels.xml`：生成结果；不可手工编辑。
- `scripts/check_mjcf.py`：使用 MuJoCo 编译资产的验证工具。

这里的默认实现是每个指尖 3×3 球形 taxel。每个 taxel 用一个球形接触 geom 与一个局部坐标系对齐的 `force` sensor 表示；此外，每侧还提供 `*_pad_force` 和 `*_pad_torque`。

不包含抓取策略、强化学习环境、真实触觉相机驱动或 Shadow Hand 专用的观测包装代码。

## 使用

```bash
uv run scripts/generate_taxels_xml.py
uv run pytest
uv run --extra sim scripts/check_mjcf.py
```

生成脚本的关键参数位于 `TAXEL_GRID`、`TAXEL_RADIUS` 与 `MIDDLE_ROW_TO_TOP_EDGE`。它们都在 `left_pad` 与 `right_pad` 的局部坐标系下定义，长度单位为米。

## 命名约定

- site：`left_taxel_site_00` 至 `right_taxel_site_22`
- taxel 力：`left_taxel_force_00` 至 `right_taxel_force_22`
- 指尖合力/力矩：`left_pad_force`、`left_pad_torque`、`right_pad_force`、`right_pad_torque`

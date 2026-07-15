# Robotiq 2F-85 Tactile

独立维护 MuJoCo 中为 Robotiq 2F-85 两个指尖添加触觉传感器的最小资产仓库。

## 内容与边界

- `assets/robotiq_2f85/2f85.xml`：未修改的基础夹爪模型。
- `scripts/generate_taxels_xml.py`：从基础模型生成带 18 个离散 taxel 的派生 MJCF。
- `assets/robotiq_2f85/2f85_taxels.xml`：生成结果；不可手工编辑。
- `scripts/check_mjcf.py`：使用 MuJoCo 编译资产的验证工具。
- `scripts/view_taxels.py`：在 MuJoCo viewer 中目视检查两侧 taxel 的位置与尺寸。
- `scripts/report_taxels.py`：将传感器读数按左右两个 3×3 网格输出。

这里的默认实现是每个指尖 3×3 球形 taxel。每个 taxel 用一个球形接触 geom 与一个局部坐标系对齐的 `force` sensor 表示；此外，每侧还提供 `*_pad_force` 和 `*_pad_torque`。

不包含抓取策略、强化学习环境、真实触觉相机驱动或 Shadow Hand 专用的观测包装代码。

## 使用

```bash
uv run scripts/generate_taxels_xml.py
uv run pytest
uv run --extra sim scripts/check_mjcf.py
uv run --extra sim scripts/view_taxels.py
uv run --extra sim scripts/report_taxels.py
```

在 viewer 中开启 `Sites` 与 `Contact points` 显示，即可检查 taxel 的局部坐标朝向、球形
接触体位置及接触点。`report_taxels.py` 的输出顺序与 XML 的 `00` 到 `22` 行优先命名一致。

生成脚本的关键参数位于 `TAXEL_GRID`、`TAXEL_RADIUS` 与 `MIDDLE_ROW_TO_TOP_EDGE`。它们都在 `left_pad` 与 `right_pad` 的局部坐标系下定义，长度单位为米。

## 命名约定

- site：`left_taxel_site_00` 至 `right_taxel_site_22`
- taxel 力：`left_taxel_force_00` 至 `right_taxel_force_22`
- 指尖合力/力矩：`left_pad_force`、`left_pad_torque`、`right_pad_force`、`right_pad_torque`

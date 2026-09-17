# Robotiq 2F-85 模型资产

本目录以 `2f85.xml` 为基础模型，保存离散力传感器 taxel 和 MuJoCo `touch_grid` 变体。
具体布局与读数转换见[触觉资产说明][tactile]；控制方式与实验入口见[Robotiq 2F-85 使用入口][robotiq-guide]。

<!-- --8<-- [start:content] -->
## 来源与许可

仓库保存了基础 MJCF 和 `assets/` 下的 STL，但本目录未记录可核验的上游版本、获取过程或独立许可声明。
派生触觉模型由项目资产生成器生成；不能仅从文件名或结构推定基础模型的来源。

> 仓库根目录的项目许可证不能代替缺失的第三方资产授权记录。上游来源、版本与许可仍需补齐。

## 模型变体 {#model-selection}

| Hydra `model` | MJCF 文件 | 触觉表示 |
| --- | --- | --- |
| `robotiq_2f85/sphere_force_sensor` | `2f85_taxels.xml` | 每侧 3×3 球形 taxel，各有 `force` sensor。 |
| `robotiq_2f85/box_force_sensor` | `2f85_taxels_box.xml` | 每侧 3×3 盒形 taxel，覆盖分块 pad。 |
| `robotiq_2f85/touch_grid_3x3` | `2f85_touch_grid_3x3.xml` | 每侧 3×3 插件输出，用于触觉对照。 |
| 无对应的现有命名配置 | `2f85_touch_grid.xml` | 生成命令默认的每侧 32×32 插件输出。 |

基础模型保留 tendon 与 `fingers_actuator` 的控制映射，`ctrlrange` 为 `[0, 255]`。
该控制量不能当作 DMgripper 的 N·m 力矩输入；使用各自模型配置和控制器。

## 生成流程

在**仓库根目录**执行以下命令。生成命令会写入指定输出文件；重新生成前先检查工作区中是否有需要保留的模型修改。

```bash
uv run pgt assets generate-taxels --shape sphere
uv run pgt assets generate-taxels --shape box
uv run pgt assets generate-touch-grid
```

三条命令分别生成 `2f85_taxels.xml`、`2f85_taxels_box.xml` 和 `2f85_touch_grid.xml`。
所有命令默认读取本目录 `2f85.xml`，可用 `--base-xml` 和 `--output-xml` 显式指定输入输出。

**复现现有 3×3 插件配置时，必须显式指定尺寸和输出路径：**

```bash
uv run pgt assets generate-touch-grid --rows 3 --cols 3 \
  --output-xml assets/grippers/robotiq_2f85/2f85_touch_grid_3x3.xml
```

> `generate-touch-grid` 的默认值是 **32×32**，不是 3×3。更换输出文件后仍需确认
> `configs/model/robotiq_2f85/` 中的 `model.path` 与触觉模式匹配。

当前生成器仍位于主包 `asset_tools.py`，依赖项目环境；加载已生成模型无需 CAD 服务。
插件模型要求当前 MuJoCo 环境能够加载 `mujoco.sensor.touch_grid`。

## 验收

```bash
uv run pgt validate configs/robotiq_2f85.yaml
uv run pgt run demo --set model=robotiq_2f85/touch_grid_3x3
uv run pgt compare tactile \
  --left-set model=robotiq_2f85/box_force_sensor \
  --right-set model=robotiq_2f85/touch_grid_3x3
```

依次检查默认资源、插件运行和两种表示的对照；同时核对以下模型契约：

- 离散 taxel 每侧各有九个具名 site 和 `force` sensor，编号从 `00` 到 `22`。
- 插件的 `touch_left`、`touch_right` 存在，尺寸与所选模型一致。
- 压缩的统一 `Fz` 为正，插件 FOV 覆盖 pad 接触面。
- 比较时区分 taxel 合力、pad 合力与插件输出，按[触觉资产说明][tactile]解释差异。

## 触觉布局与读数 {#tactile-model}

两类资产共用项目的[触觉接口约定][tx-tactile-contract]，但测量结构不同。
生成命令与配置选择见前文。

### 离散 force sensor

每个 taxel 是 pad 下的子 body，具有对齐的 site 和 MuJoCo `force` sensor。

| 变体 | 几何布局 | 传感器命名 |
| --- | --- | --- |
| `sphere_force_sensor` | 每侧 3×3 球体，半径 2.8 mm，行列间距 7 mm；保留原 pad。 | `left/right_taxel_force_00` 至 `22`。 |
| `box_force_sensor` | 每侧 3×3 盒体，替换并分块覆盖原 pad。 | 同上。 |

MuJoCo 原始传感器力表示“taxel 子 body 施加给 pad 父 body 的力”。运行时读取器对
**完整三维向量取反**，记录为“物体施加给 taxel 表面的力”，使压缩时的 `Fz` 为正。

> 九个 taxel 的合力只代表这些离散单元传递的力，不等于整个 pad 的完整外力。
> `left/right_pad_force` 和 `left/right_pad_torque` 属于另一测量层级，不能与九个 taxel 通道混用。

### touch_grid 插件

`touch_grid` 通过 `mujoco.sensor.touch_grid` 插件输出力网格，每侧由 `touch_left` 或 `touch_right`
site 定义观测坐标系。生成器会将 pad 替换为对应行列数的盒体碰撞单元，但输出机制不是逐单元的 `force` sensor。

| 配置项 | 当前生成器约定 |
| --- | --- |
| 输出尺寸 | 默认 32×32；现有命名配置 `touch_grid_3x3` 使用显式生成的 3×3 文件。 |
| `size` | XML 参数顺序为 `cols rows`。 |
| 原始通道 | `(Fz, Fx, Fy)`。 |
| 对外通道 | 读取器重排为 `(Fx, Fy, Fz)`。 |
| 数值 | 牛顿，`nchannel=3`、`gamma=0`，不作归一化或对数变换。 |
| FOV | `24 15`，以触觉 site 为中心的半角，单位为度。 |

改变 pad 尺寸、site 位置或朝向后，必须重新检查 FOV 是否覆盖整个接触面。
接触存在但超出 FOV 时，该部分力不会进入插件输出；不能直接判定为物理接触失效。

### 对照时先检查什么

1. **比较对象：**球形离散 taxel 保留原 pad，盒形 taxel 与插件使用分块 pad，接触几何并非完全相同。
2. **坐标系：**统一输出仍在各自 site 的局部系；需要跨左右指尖比较方向时，先旋转到世界系。
3. **测量范围：**离散 taxel 的力传递路径与插件的 FOV 不同，不能仅凭热图相似就认为合力等价。

可视化箭头的缩放和训练观测的归一化规则由[公共接口约定][tx-tactile-contract]维护；资产页只解释几何与传感器映射。
<!-- --8<-- [end:content] -->

[tactile]: #tactile-model
[robotiq-guide]: ../../../docs/grippers/robotiq-2f85/index.md
[tx-tactile-contract]: ../../../docs/tactile-conventions.md
[tx-model]: README.md
# 项目架构

## 分层

项目分为四层：

```text
夹爪资产（MJCF/STL）
        ↓
夹爪 profile（路径、控制器、安装位姿、触觉契约）
        ↓
核心库（加载、验证、后续统一触觉读取）
        ↓
场景与实验（抓取、扰动、记录、绘图、Rerun）
```

资产层可以包含完全不同的机构；上层只依赖 profile，而不依赖 Robotiq 或自研夹爪
内部的连杆名称。

## Profile 契约

每个 `configs/*.toml` 至少声明：

- `model.path`：相对仓库根目录的 MJCF 路径；
- `control.actuator`：执行器名称；
- `control.open/closed`：控制量端点，单位由执行器本身决定；
- `mount.pos/quat`：装入公共场景时的位姿；
- `tactile.mode`：`force_sensor` 或 `contact_geom`；
- `tactile.rows/cols` 与左右通道前缀。

`pgt-check` 会编译 MJCF，并验证执行器和全部触觉通道是否存在。模型适配问题应尽量
在这里失败，而不是在长时间仿真后才以空数组或错误索引表现出来。

## 场景组合与命名

现有场景仍使用 `MjSpec.attach()` 将环境、夹爪和物体组合，并给夹爪、方块增加
`gripper/`、`cube/` 前缀。旧 Robotiq 脚本通过 `gripper_name_in_model()` 兼容组合模型
和完整外部场景。

后续公共实验入口应按 profile 查找 actuator ID，而不是假设执行器位于
`data.ctrl[0]`；控制轨迹也应由 profile 的开闭端点插值。

## 两种触觉后端

Robotiq 的离散 taxel 使用独立子 body 上的 `<force>` sensor。自研夹爪则使用真实
Pillars STL 作为接触 geom，通过接触对按 geom 汇总力。两者最终都应转换为统一的：

```text
(3, rows, cols)  # Fx, Fy, Fz，指尖局部坐标系，压缩 Fz 为正
```

记录、绘图和学习环境只消费这个统一张量，不关心底层是 force sensor、接触 geom
还是 touch-grid 插件。

## 生成资产与手工维护边界

- `assets/grippers/robotiq_2f85/2f85.xml` 是参考基础模型；
- `2f85_taxels*.xml` 与 `2f85_touch_grid*.xml` 由脚本生成；
- 自研夹爪由 Onshape 导出，再由 `prepare_onshape_export.py` 恢复稳定的 taxel 命名；
- 不应把实验参数、对象初态或控制协议写入夹爪基础资产；它们属于场景或 profile。

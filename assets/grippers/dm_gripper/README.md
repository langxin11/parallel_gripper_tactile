# DMgripper 模型资产

本目录保存曲柄滑块式平行二指夹爪的 Onshape 导出模型、碰撞变体和网格。
一个电机驱动曲柄，经左右连杆带动手指沿导轨同步开合。

**按任务阅读：**[导出和修改模型][onshape] · [Pillar 布局与接触映射][tactile] ·
[DMgripper 使用入口][dm-guide] · [模型验证与适用边界][validation]。

<!-- --8<-- [start:content] -->
## 来源与许可

导出来源由本目录 `config.json` 中的 Onshape workspace 与 Assembly URL 记录，输出格式为 MuJoCo，
模型名称为 `parallel_gripper`。该 URL 指向可变 workspace，不等同于冻结的 CAD 版本。
本目录保留了 STL 与 `.part` 资源，但未提供独立的资产许可声明，也未记录完整的上游版本与授权清单。

> 仓库根目录有项目许可证；不能仅据此推定所有 CAD 与网格的上游授权。资产来源版本与许可仍需补齐。

## 模型选择 {#model-selection}

| Hydra `model` | MJCF 文件 | 碰撞表示与用途 |
| --- | --- | --- |
| `dm_gripper/height_spheres` | `parallel_gripper_height_sphere_collision.xml` | 默认变体，球体代理保留 Pillar 高度差。 |
| `dm_gripper/original_mesh` | `parallel_gripper_prepared.xml` | 原始非共面 mesh，仅用于历史行为复现。 |

`parallel_gripper.xml` 是导出模型基线，`scene.xml` 是带地面、灯光与相机的预览场景，
`assets/` 保存网格与 Part 文件。变体的生成流程从[资产维护入口][onshape]查阅。

在**仓库根目录**执行：

```bash
uv run pgt validate configs/dm_gripper.yaml
uv run pgt view grasp --set model=dm_gripper/height_spheres
```

第一条检查默认 profile 和模型资源，第二条运行默认碰撞变体的抓取预览。
只查看资产预览场景时，可使用：

```bash
uv run -m mujoco.viewer --mjcf assets/grippers/dm_gripper/scene.xml
```

> `scene.xml` 用于资产预览。需要确认具体实验使用哪个碰撞变体时，以组合后的 `model.path` 为准。

## 机构与执行器契约

| MuJoCo 关节 | 类型 | 作用 |
| --- | --- | --- |
| `gripper_drive` | hinge | 唯一主动关节，电机输出轴。 |
| `left_link_crank_pin`、`right_link_crank_pin` | hinge | 左右被动曲柄销轴。 |
| `left_finger_slide`、`right_finger_slide` | slide | 左右被动滑台。 |

当前 prepared 模型及碰撞变体使用名为 `gripper_drive` 的 `<motor>`；输入是输出轴力矩，单位为 N·m，
MJCF 限幅为 `±4 N·m`，主动关节范围为 `[0, 1.7] rad`。曲柄半径为 30 mm，连杆两销轴中心距为 40 mm。
MIT 内环、连续力矩限制和硬件建模假设见[电机建模摘要][motor-model]；控制流程见[DMgripper 使用入口][dm-guide]。

> `config.json` 的导出基线配置为 position actuator。`prepare-onshape` 只命名触觉碰撞几何，
> 不会自动转换执行器。重新导出后必须核对主项目所需的 motor 类型、单位和限幅。

## 闭环与触觉命名

两个运动学闭环分别是 `base → drive crank → link → finger → base`。
每侧树外 Revolute 销轴用两条 `<connect>`：中心点重合，轴线上的 `_z` 点重合。
当前模型共四条约束，只保留销轴绕轴转动；仅约束中心点会等价于球铰。

| 检查对象 | 应满足的条件 |
| --- | --- |
| 左闭环 | `closing_left_link_finger_pin_1/2` 及其 `_z` 对。 |
| 右闭环 | `closing_right_link_finger_pin_1/2` 及其 `_z` 对。 |
| 触觉碰撞 | 左右各九个 `left/right_taxel_geom_00` 至 `22`。 |
| 触觉坐标系 | 同编号的 `left/right_taxel_00` 至 `22` site。 |

控制使用的法向主量遵循[公共触觉约定][tactile-contract]中的平均单侧力定义。
本页不另行定义 PID 目标或传感器噪声，避免模型说明与运行时接口发生冲突。

## 触觉布局与读数 {#tactile-model}

本节说明模型几何如何映射到触觉通道。统一数组、单位、力的正负号与控制主量见
[公共触觉接口约定][tx-tactile-contract]。

### 几何布局

每侧有 **3×3 个 Pillar 通道**。默认 `height_spheres` 用半径 `0.0028 m` 的球体作为碰撞代理，
保留原阵列的高度差；可视化仍保留 Pillar mesh。

| 原始阵列位置 | 左指尖局部 site `z` | 相对四角高度 |
| --- | --- | --- |
| 四角：`00`、`02`、`20`、`22` | `0.03295 m` | `0 mm`。 |
| 边中：`01`、`10`、`12`、`21` | `0.03325 m` | 约 `0.30 mm`。 |
| 中心：`11` | `0.03345 m` | 约 `0.50 mm`。 |

右指尖也保持中心高、四周低的规律。该阵列不能按平面接触解释单元接触顺序。
原始 mesh 保留相同的 site 高度，但 Pillar 碰撞表示不同，不能把球体代理的接触行为套用到原始 mesh。

### 命名与读取链路

| 对象 | 名称或配置 |
| --- | --- |
| 碰撞 geom | `left_taxel_geom_RC`、`right_taxel_geom_RC`。 |
| 对应 site | `left_taxel_RC`、`right_taxel_RC`。 |
| 行列编号 | `R`、`C` 各为 `0`、`1`、`2`，配置按行优先列举。 |
| 读取模式 | `contact_geom`。 |
| 每侧输出 | `(3, 3, 3)`，通道为 `(Fx, Fy, Fz)`，单位为 N。 |

`ContactTaxelReader` 按以下顺序聚合接触力：

1. 对属于具名 Pillar geom 的接触调用 `mj_contactForce`。
2. 将接触系力旋转到世界系；该接口报告作用在 `geom2` 上的力，目标为 `geom1` 时取反。
3. 使用对应 site 的 `data.site_xmat` 转回局部系，得到作用于 Pillar 表面的力。
4. 把同一单元的所有接触点累加；无接触单元保留为零。

> 这里直接聚合 MuJoCo 接触力，不需要 Robotiq 的 `force` sensor，也不需要额外生成 Robotiq 球形 taxel。

### 修改与检查

Onshape 的 `frame_left_taxel_00...22` 和 `frame_right_taxel_00...22` Mate Connector
在导出后移除 `frame_` 前缀，形成 18 个 site。碰撞白名单和重新命名步骤从[资产维护入口][tx-onshape]查阅。

在仓库根目录执行：

```bash
uv run pgt run grasp
uv run pgt compare contact
```

抓取演示可检查压缩时 `Fz > 0`；切向对照用“自由方块＋mocap weld”夹具施加受控世界系位移，
其中世界 `y` 对应局部 `Fy`，世界 `z` 对应局部 `Fx`。该夹具用于核对方向与通道映射，
不用于标定真实摩擦或接触刚度。

需要排查高载荷振荡、`multiccd` 或碰撞变体差异时，阅读[模型验证与高载荷接触边界][tx-validation]。
`solref` 是接触求解参数，不能直接视为已标定的硅胶材料刚度或传感器精度。
<!-- --8<-- [end:content] -->

[onshape]: onshape-export.md
[tactile]: #tactile-model
[dm-guide]: ../../../docs/grippers/dmgripper/index.md
[validation]: ../../../docs/control-comparison-ablation.md#collision-geometry-conclusions
[motor-model]: DM_J4310P_24V_MJCF_建模摘要.md
[tactile-contract]: ../../../docs/tactile-conventions.md
[tx-tactile-contract]: ../../../docs/tactile-conventions.md
[tx-onshape]: onshape-export.md
[tx-validation]: ../../../docs/control-comparison-ablation.md#collision-geometry-conclusions

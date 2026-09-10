# Custom Parallel Gripper

本目录包含从 Onshape 导出的曲柄滑块式平行二指夹爪 MuJoCo 模型、预览场景、验证脚本和网格资源。

夹爪由一个电机驱动曲柄，曲柄通过左右传动连杆带动两个手指滑台沿导轨同步开合。模型包含两个运动学闭环：

```text
base -> drive crank -> left link  -> left finger  -> base
base -> drive crank -> right link -> right finger -> base
```

## 目录内容

| 路径 | 说明 |
| --- | --- |
| `parallel_gripper.xml` | MuJoCo 主模型 |
| `parallel_gripper_prepared.xml` | 导出后处理版本；18 个 Pillar 碰撞 geom 按最近的命名 taxel site 赋予稳定名称，供项目 profile 使用 |
| `scene.xml` | 带地面、灯光和相机参数的预览场景 |
| `config.json` | `onshape-to-robot` 导出配置，当前指向 Onshape B1 workspace |
| `verify_mujoco.py` | 无界面结构与运动学检查 |
| `assets/` | Onshape 导出的 STL 和 Part 文件 |

## 快速使用

在仓库根目录预览模型：

```bash
uv run -m mujoco.viewer \
  --mjcf assets/grippers/dm_gripper/scene.xml
```

运行本模型的基础验证：

```bash
uv run pgt validate configs/dm_gripper.yaml
```

运行项目级模型检查：

```bash
uv run pgt validate configs/dm_gripper.yaml
```

以静态方块压住某个 Pillar，并在 Viewer 中查看 taxel site 坐标轴、接触点和三维接触力：

```bash
uv run pgt view grasp --profile configs/dm_gripper.yaml
```

无界面验证右侧对应单元的正压缩力：

```bash
uv run pgt run grasp --profile configs/dm_gripper.yaml
```

以受控世界系切向位移验证剪切通道（`y` 对应局部 `Fy`，`z` 对应局部 `Fx`）：

```bash
uv run pgt compare contact --profile configs/dm_gripper.yaml
```

切向测试使用“自由方块 + mocap weld”夹具提供规定运动；它用于确认剪切方向与通道映射，
并不用于标定真实摩擦或接触刚度。

该演示调用项目中的 `ContactTaxelReader`，直接从 `left/right_taxel_geom_*` 接触读取
`Fx/Fy/Fz`；不依赖 Robotiq 专用 force sensor，也不会额外生成球形 taxel。

## 模型结构与控制

模型有 5 个关节，其中只有电机输出轴是主动关节：

| MuJoCo 关节 | 类型 | 作用 |
| --- | --- | --- |
| `gripper_drive` | hinge | 电机输出轴，唯一主动关节 |
| `left_link_crank_pin` | hinge | 左侧被动曲柄销轴 |
| `right_link_crank_pin` | hinge | 右侧被动曲柄销轴 |
| `left_finger_slide` | slide | 左手指沿导轨移动 |
| `right_finger_slide` | slide | 右手指沿导轨移动 |

唯一执行器为 `gripper_drive`，它是 `<motor>` 输出轴纯力矩源，`data.ctrl` 的单位为 N·m。
B1 导出的关节范围按当前达妙 MIT 配置扩展为 `[0, 1.7] rad`。曲柄半径为 30 mm、连杆两销轴
中心距为 40 mm。无被抓物时，左右 Pillars 在约 `1.34 rad` 开始接触；因此默认抓取闭合目标
仍保持在 `1.30 rad`。

Python 控制层按 MIT 形式计算 `kp*(p_des-p)+kd*(v_des-v)+t_ff`。当前 `T_MAX=4 N·m`，
MJCF 执行器也限制为 `±4 N·m`；连续或长时仿真仍应按额定 `3.5 N·m` 或热模型降额。
进入控制律前，命令会先模拟达妙 CAN/串口帧量化：位置 16 bit，速度和力矩 12 bit，
刚度按 `0..500`、阻尼按 `0..5` 的 12 bit 区间编码。
详细参数与建模假设见 `DM_J4310P_24V_MJCF_建模摘要.md`。

抓取验收采用两层混合控制：未接触时按 MIT 位置目标低速闭合；左右指尖 taxel 均连续
检测到接触后，切换到 `simple-pid` 法向力外环。PID 跟踪两侧 taxel 法向力之和，并输出
接触位置附近的有限位置修正；在线刚度估计器用 `dF/dc` 给出位置前馈，曲柄滑块开度雅可比
给出准静态 MIT 力矩前馈，再由 MIT 内环转换成最终电机力矩。默认目标总法向力为 `8 N`，
接触阈值、滤波频率、PID 增益和最大位置修正均在 profile 的 `[control.force]` 中配置。
控制反馈使用 Contactile/PapillArray 空载记录标定的逐 taxel 加性高斯白噪声。空载记录
给出的是 3×3 指尖总力噪声，因此 profile 将其除以 `sqrt(9)` 后作为单 taxel 标准差：
左右单 taxel 法向 `sigma=0.0067/0.0133 N`，切向 `sigma=0.0033/0.0100 N`。
法向总力测量再经过 `20 Hz` 一阶低通进入默认力控反馈。

## 闭环约束

左右两侧各有一个树外 Revolute 销轴。每个销轴使用两条 `<connect>` equality：一条重合销轴中心，另一条重合轴线上的 `_z` site。两点约束共同保持销轴位置与轴线方向，只保留绕轴旋转自由度。

因此 `parallel_gripper.xml` 中应始终有 4 条 `<connect>`：

```xml
<connect site1="closing_left_finger_pin_1" site2="closing_left_finger_pin_2"/>
<connect site1="closing_left_finger_pin_1_z" site2="closing_left_finger_pin_2_z"/>
<connect site1="closing_right_link_finger_pin_1" site2="closing_right_link_finger_pin_2"/>
<connect site1="closing_right_link_finger_pin_1_z" site2="closing_right_link_finger_pin_2_z"/>
```

只约束中心点等价于球铰，不能完整表达严格三维机构中的 Revolute 副。重新导出后，应确认 `_z` 约束仍然存在。

## 碰撞与触觉单元

夹持接触来自左右手指的 Pillars STL。左右各有 9 个有效碰撞 mesh，共 18 个；另有 base、MGN9 rail、stator 和 bracket 四个外壳碰撞 mesh。motor、crank 与传动标准件只保留可视化，避免内部传动碰撞。导出配置通过以下规则持久化碰撞白名单：

同一指尖的 3×3 taxel site 不是共面阵列，而是随 Pillars 顶面形成轻微凸起：四角 site
高度最低，边中间高约 0.30 mm，中心最高且比四角高约 0.50 mm。以左指尖局部 site 为例，
`left_taxel_00/02/20/22` 的 `z=0.03295`，`left_taxel_01/10/12/21` 的 `z=0.03325`，
`left_taxel_11` 的 `z=0.03345`；右指尖保持同样的中心高、四周低规律。

该结构对应 Contactile PapillArray 类传感器：Contactile 官方说明其触觉阵列由可独立偏转的
soft silicone pillars 组成，可在每个阵列单元测量 3D displacement、3D force 和 vibration
（见 [Contactile technology](https://contactile.com/novel-optical-sensing-technology/) 与
[Contactile products](https://contactile.com/products/)）。公开产品页给出的 PapillArray
规格为 3×3 阵列、pillar 直径 6 mm、高度 4.2 mm、间距 7 mm、硅胶 Shore A40、Z 向位移
量程 +2.5 mm、Z 向力量程 15 N（见
[Scivaro PapillArray specs](https://www.scivaro.com/index.php?c=show&id=454)）。若按
`15 N / 2.5 mm` 换算，`6000 N/m` 可作为偏硬上界；若 15 N 是整个 3×3 阵列的总量程，
单个 pillar 的等效法向刚度约为 667 N/m。PapillArray 原型论文报告过单 pillar 弹簧常数
约 `1.174 N/mm = 1174 N/m`（见
[PapillArray slip sensor paper](https://www.sciencedirect.com/science/article/pii/S0924424717313419)）。
当前 MJCF 采用 `solref="-1200 -10"`，对应约 `1200 N/m`，并将力外环积分增益调到
`ki=0.100` 以保持 `8 N` 目标法向力下的跟踪精度。

```json
"ignore": {
  "*": "collision",
  "!Pillars*": "collision",
  "!base": "collision",
  "!stator": "collision",
  "!bracket*": "collision",
  "!*MGN9 Rail": "collision"
}
```

Assembly 中的 `frame_left_taxel_00...22`、`frame_right_taxel_00...22` Mate Connector 会分别导出为 18 个 `left_taxel_*`、`right_taxel_*` site；`frame_` 前缀被 exporter 移除。

在 MuJoCo Viewer 中，按 `3` 显示或隐藏 collision group 3，按 `C` 显示接触点，按 `F` 显示接触力。

## 从 Onshape 重新导出

### Assembly 刚体划分

顶层 Assembly 建议按以下概念组织。第一个实例会被 exporter 当作基座，因此应让 `base` 排在第一位，并用 Onshape 的 **Fixed** feature 固定到世界。

| Onshape 实例或子装配 | MJCF 结果 | 说明 |
| --- | --- | --- |
| `gripper_base` | 固定 base body | 定子、连接架和左右导轨用 Fastened 合并 |
| `drive_crank` | 转动 body | 电机转子和驱动曲柄固定为一个刚体 |
| `left_link` / `right_link` | 两个 body | 左右传动连杆 |
| `left_finger` / `right_finger` | 两个 body | 滑台与指体分别固定合并 |

如需稳定导出的 link 名，可在实例上放置并命名 Mate Connector，例如 `link_gripper_base`、`link_drive_crank` 和 `link_left_finger`。

### Mate 命名

名称应设置在 Assembly 的 Mate feature 上。`dof_` 会导出普通关节，`fix_` 会合并固定刚体，`closing_` 会导出闭环 equality。每个 Mate Connector 的 Z 轴是关节轴。

| 两端刚体 | Mate 类型 | 建议名称 | 作用 |
| --- | --- | --- | --- |
| base ↔ drive crank | Revolute | `dof_gripper_drive` | 唯一主动关节 |
| rotor ↔ drive crank | Fastened | `fix_crank_to_rotor` | 合并转子与曲柄 |
| drive crank ↔ left link | Revolute | `dof_left_link_crank_pin` | 左侧被动销轴 |
| base ↔ left finger | Slider | `dof_left_finger_slide` | 左侧被动滑台 |
| left link ↔ left finger | Revolute | `closing_left_link_finger_pin` | 左侧闭环断点 |
| drive crank ↔ right link | Revolute | `dof_right_link_crank_pin` | 右侧被动销轴 |
| base ↔ right finger | Slider | `dof_right_finger_slide` | 右侧被动滑台 |
| right link ↔ right finger | Revolute | `closing_right_link_finger_pin` | 右侧闭环断点 |

不要把 `closing_` 用在 Slider 副上：当前 exporter 只为 `closing_` 的 Fastened、Revolute 和 Ball mate 生成闭环约束。每个闭环应在“连杆—滑块”的 Revolute 副处断开，也不要给该副再加 `dof_`。

如果开合方向相反，可将驱动 Mate 命名为 `dof_gripper_drive_inv`；导出的关节仍叫 `gripper_drive`，但轴方向会翻转。

### Mate Connector 与限位

- Revolute 的两个 Mate Connector 原点应位于销轴中心，Z 轴沿销轴。
- Slider 的 Mate Connector Z 轴应沿导轨运动方向。
- 为两个手指 Slider 设置合理行程，导出后会成为 joint range。
- 为左右被动曲柄销设置宽松限位，例如 `-180°` 至 `180°`，以消除导出警告。
- Assembly 默认姿态应无干涉且无不合理预紧。

### 导出命令

`config.json` 中的 URL 必须指向正确的 Onshape workspace 和 Assembly 标签页。安装好 `onshape-to-robot[mujoco]` 并配置 Onshape 凭据后，在独立导出环境中执行：

```bash
uv run onshape-to-robot assets/grippers/dm_gripper
uv run onshape-to-robot-mujoco assets/grippers/dm_gripper
```

导出配置中的 `joint_properties` 使用移除 `dof_` 后的关节名。除 `gripper_drive` 外，四个被动关节都必须设置为 `"actuated": false`。

## 重新导出后的检查清单

- `gripper_drive` 与四个被动关节均存在，且只有 `gripper_drive` 有执行器。
- 两个闭环各有中心点和 `_z` 点约束，共 4 条 equality。
- 18 个 Pillars geom 与 4 个外壳 mesh 启用碰撞；motor/crank 不参与碰撞。
- 有 18 个 `left_taxel_*` / `right_taxel_*` site，且无独立 `frame_freejoint`。
- 运行 `pgt validate` 前，先执行导出后处理，为 18 个 Pillars collision geom 赋予稳定名称。
- 低、中、高三个目标位置均无非有限状态、明显跳变或严重跟踪误差。

## 已知事项与排障

- 重新导出会覆盖主 XML 的导出后处理结果；应再次执行 `pgt assets prepare-onshape`，并确认闭环 `_z` 约束仍存在。
- 模型发散或闭环松软时，先检查 Mate Connector 的原点、Z 轴和默认姿态，再调节 equality 的 `solref` / `solimp`；不要一开始就极端增大刚度。
- Onshape 中可动但导出后卡住时，优先检查轴向和 mate limits 是否包含默认姿态。
- 出现额外自由度时，检查转子与曲柄、滑台与指体，以及 `frame` 与 `base` 是否正确固定或合并。

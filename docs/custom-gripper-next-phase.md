# 自研平行夹爪下一阶段实施路线图

本文描述自研曲柄滑块平行夹爪接入触觉仿真框架后的路线图。该文档随实现进度维护：
已完成项标注 ✅ 并给出落地位置，部分完成标注 ⏳，未开始标注 ⬜。

## 最终目标

完成后应能够：

1. 从 Onshape 稳定导出单根节点、质量和关节范围正确的夹爪模型；
2. 以左右各 9 个 Pillars STL 作为唯一指尖接触区域；
3. 将每个 Pillar 的接触力整理为左右两个 `(3, 3, 3)` 张量；
4. 使用名称查找执行器，而不是依赖 `data.ctrl[0]`；
5. 在公共抓取、扰动、CSV 和 Rerun 工作流中切换 Robotiq 与自研夹爪；
6. 通过自动测试证明闭环、控制、触觉符号、坐标系和数值稳定性正确。

### 当前状态总览

| 目标 | 状态 | 说明 |
|---|---|---|
| 1. Onshape 单根节点/质量/关节范围 | ⏳ 部分 | 单根节点与真实质量已完成；pin 关节范围与 drive 行程未复核（见阶段 0） |
| 2. 18 个 Pillars 唯一接触 | ✅ | `scripts/verify_mujoco.py` 验证 Pillars 碰撞面 18、外部碰撞面已过滤 |
| 3. `(3, 3, 3)` 触觉张量 | ✅ | `ContactTaxelReader`（阶段 1） |
| 4. 名称查找执行器 | ⏳ 部分 | 自研夹爪已按名称查 actuator；Robotiq 侧 `ctrl[0]`/`220` 仍为遗留（阶段 2） |
| 5. 公共工作流切换 | ⏳ 部分 | 自研夹爪已有全套脚本；`pgt-grasp` 单入口未建（阶段 2） |
| 6. 自动测试 | ✅ | 35 项 pytest + CI（ruff check / format / pytest）+ `pgt-check` 双 profile |

统一触觉张量约定为：

```text
shape = (3, rows, cols)
channel 0 = Fx
channel 1 = Fy
channel 2 = Fz
单位 = N
坐标系 = 对应指尖局部坐标系
符号 = 指向传感表面的压缩 Fz 为正
```

## 阶段 0：修正 Onshape 动力学源模型

**状态：⏳ 部分完成。** 这是后续工作的前置条件。不要在 Python 层长期掩盖 CAD 拓扑或质量问题。

### 已完成

- ✅ 只有一个 root node（`base_freejoint` 自由体），无参考 `frame` 独立自由体；
- ✅ 无 `1e-9 kg` 占位质量，所有惯性参数为导出实测值；
- ✅ 导出流程有后处理与验证脚本：`scripts/prepare_onshape_export.py` +
  [Onshape 导出与升级建议](onshape-export-upgrade.md) +
  `scripts/verify_mujoco.py`（扫掠、闭环、碰撞过滤自动检查）。

### 待完成

- ⬜ 给 `left_link_crank_pin`、`right_link_crank_pin` 设置合理 `range`（当前 XML 无 range）；
- ⬜ 复核 `gripper_drive` 的实际机械行程：`verify_mujoco.py` 报告
  `WARNING: largest position tracking error is 0.1491 rad. Do not use the full exported drive range until the mechanical limit is reconciled.`
  ——当前导出范围 `[-3.8e-11, 1.5708] rad` 末端不可达，需在 Onshape 中复核行程上限；
- ⬜ 在 Onshape 中完成完整开闭行程的干涉检查。

### 验收标准

```text
root nodes: 1
主动自由度: gripper_drive
被动自由度: 4
无 no mass 警告
无 revolute has no limits 警告
无 frame_freejoint
verify_mujoco.py 扫掠无位置跟踪警告
```

## 阶段 1：实现 Pillars 接触力读取器

**状态：✅ 已完成。**

自研夹爪不再叠加球形 taxel。18 个 Pillars mesh geom 既是碰撞面，也是触觉通道。

### 落地位置

```text
src/parallel_gripper_tactile/contact_taxels.py
```

```python
class ContactTaxelReader:
    def __init__(self, model, tactile: TactileLayout) -> None: ...
    @classmethod
    def from_profile(cls, model, profile: GripperProfile) -> ContactTaxelReader: ...
    def read(self, data: mujoco.MjData) -> ContactTaxelFrame: ...

@dataclass(frozen=True, slots=True)
class ContactTaxelFrame:
    left: np.ndarray   # (3, rows, cols)
    right: np.ndarray  # (3, rows, cols)
```

> 与早期提案的差异：帧类型名为 `ContactTaxelFrame`（原提案 `TactileFrame`），
> `read()` 只接收 `data`（`model` 在构造时解析并缓存）。

### 计算流程（已实现）

每个仿真步：

1. 构造时按 profile 将 `left/right_*_geom_00...22` 解析为 geom ID 和对应 site ID；
2. `read()` 遍历 `data.contact[:data.ncon]`；
3. 只处理至少一侧 geom 属于 Pillars 集合的接触（双侧同属时去重，`mj_contactForce` 只调用一次）；
4. 用 `mujoco.mj_contactForce()` 取接触坐标系力，`contact.frame` 转置旋转到世界系；
5. 根据 Pillar 位于 `geom1` 还是 `geom2` 取 `±`，保证两种顺序下方向一致；
6. 再旋转到对应指尖 site 局部坐标系（`data.site_xmat`）；
7. 同一 Pillar 多个接触点逐项求和；
8. 按 `(row, col)` 写入左右触觉张量。

### 测试覆盖

`tests/test_contact_taxels.py`：

- ✅ 无接触时输出全零（`test_custom_gripper_reader_resolves_all_taxels_and_reports_zero_without_contact`）；
- ✅ Pillar 分别作为 `geom1`、`geom2` 时结果方向一致（`test_contact_force_sign_depends_on_contact_geom_order`）；
- ✅ 世界系切向力正确变换为左右指尖局部 `Fx/Fy`（`test_prescribed_world_shear_maps_to_expected_local_components`）。

可选补充（当前未显式覆盖）：

- ⬜ 单个 Pillar 受压时只有对应单元非零（部分由全零测试 + 演示运行覆盖）；
- ⬜ 同一 Pillar 多接触点时正确求和；
- ⬜ 输出始终有限，不出现 NaN/Inf；
- ⬜ 左右对称受压时两侧局部 `Fz` 都为正的显式断言（演示运行实测左 taxel_11 = +115.7 N，符号正确）。

### 验收标准（已满足）

```text
left.shape  == (3, 3, 3)
right.shape == (3, 3, 3)
压缩 Fz > 0
无接触单元严格为 0
```

## 阶段 2：泛化场景和控制接口

**状态：⏳ 部分完成。**

### 已完成

- ✅ profile 已含控制与安装字段：`actuator`、`open_control`、`closed_control`、
  `mount_pos`、`mount_quat`（`src/parallel_gripper_tactile/profiles.py`）；
- ✅ 自研夹爪按名称查 actuator：`model.actuator(f"{GRIPPER_PREFIX}{profile.actuator}").id`
  （`scripts/run_custom_grasp_validation.py`），不再依赖 `data.ctrl[0]`；
- ✅ 公共扰动协议 `DisturbanceProtocol`（`src/parallel_gripper_tactile/protocols.py`）：
  `step(..., actuator_id=..., close_control=...)` 统一控制斜坡、支撑释放与外力注入，
  Robotiq 与自研夹爪共用；
- ✅ 自研夹爪场景从 profile 构造：`scripts/custom_grasp_scene.py`（含 `DEFAULT_PROFILE`）。

> 早期提案中的 `src/.../scene.py` 与 `control.py` 未单独创建：场景构造落在
> `scripts/custom_grasp_scene.py`，控制轨迹与外力协议落在 `protocols.py`，
> 两文件提案已被此分工取代。

### 待完成

- ⬜ `pgt-grasp` 统一入口（当前仅有 `pgt-check` 验证命令）：

  ```bash
  uv run pgt-grasp configs/robotiq_2f85.toml
  uv run pgt-grasp configs/custom_parallel_gripper.toml
  ```

- ⬜ Robotiq 侧遗留常量迁移：`scripts/grasp_scene.py` 的 keyframe
  `ctrl=[0.0]` / `ctrl=[220.0]` 仍硬编码（Robotiq 基线，历史遗留）；
- ⬜ 对无执行器、重名执行器和控制范围错误给出明确异常。

### 验收标准

```text
公共模块中不再出现模型专属的 ctrl[0]、220、fingers_actuator 或 gripper_drive 常量
（当前仅自定义夹爪侧达成，Robotiq 侧遗留）
```

## 阶段 3：接入记录、绘图和 Rerun

**状态：⏳ 部分完成。**

### 已完成

- ✅ Pillars 读取器已接入自定义夹爪链路：`scripts/run_custom_grasp_validation.py`
  输出 CSV/PNG 至 `outputs/custom_gripper/validation/`；
- ✅ 抓取与扰动实验共用 `DisturbanceProtocol`，同一套 CSV 记录格式；
- ✅ Rerun 记录已接入公共演示循环（`scripts/recording.py` 的 `RerunTactileLogger`）：
  左右 `tactile/{side}/pressure` 以 `(row, column)` 张量记录压力热图，`raw` 记录
  完整 `(component, row, column)` 网格，离线 `.rrd` 可重放；
- ✅ 输出按实验分类归档（实际采用的布局）：

  ```text
  outputs/
    robotiq/
      taxel_demo/ touch_grid_demo/ comparison/ disturbance_video/
    custom_gripper/
      validation/ disturbance_video/ experiments/
  ```

  > 早期提案的 `outputs/<date>-<profile>-<experiment>/`（含 `metadata.json`、
  > `figures/` 子目录）未采用，实际布局以上为准。

### 待完成

- ⬜ `metadata.json`：保存 profile 名称、模型哈希、MuJoCo 版本、时间步和接触参数
  （当前 CSV 无元数据）；
- ⬜ 绘图脚本不再根据文件名猜测模型类型：`experiments/` 下的历史扫描文件仍按
  文件名前缀（`soft_*`、`yz_*` 等）归拢，属已知遗留；
- ⬜ Rerun 中热图单元与实际受压 Pillar 一一对应的展示校验。

### 验收标准

```text
两种夹爪生成相同 schema 的 CSV          （自定义侧已达成，Robotiq 侧列略有差异）
Rerun 中热图单元与实际受压 Pillar 一一对应 （待校验）
左右合力能够在世界坐标系中比较          （已达成）
记录可以离线重放，不依赖原仿真进程       （已达成，.rrd 文件）
```

## 阶段 4：物理参数与实机标定

**状态：⬜ 未开始。** 仿真稳定不等于物理可信。该阶段需要结合实机、材料数据或台架测量。

### 需要标定的参数

- Pillars 与被抓物体之间的摩擦系数；
- `solref`、`solimp` 与接触刚度/阻尼；
- 每个 Pillar 的零偏、增益和饱和范围；
- 夹爪连杆、滑块与电机转子的质量和惯量；
- 关节阻尼、摩擦损失和机构回差；
- 电机峰值 `12.5 N·m` 与连续 `3.5 N·m` 的使用边界；
- 控制器 `kp`、稳定时间和最大允许跟踪误差。

### 推荐实验

1. 单 Pillar 法向压缩：校准压力方向、增益和接触参数；
2. 单 Pillar 切向加载：校准摩擦与 `Fx/Fy`；
3. 已知质量方块静态夹持：校验左右合力；
4. 逐步增大扰动力：测量滑移阈值；
5. 完整开闭空载实验：校验关节轨迹与电机力矩；
6. 不同时间步复现实验：检查数值收敛性。

### 验收标准

验收阈值应在拿到实测噪声和重复性后确定。至少报告：

- 法向力 RMSE；
- 左右总力偏差；
- 滑移阈值误差；
- 关节位置跟踪误差；
- 不同时间步下结果变化；
- 峰值力矩持续时间和连续力矩占用率。

## 阶段 5：回归测试与发布门槛

**状态：✅ 主体已完成。**

### 自动检查（本地全部通过）

```bash
uv run ruff check .
uv run ruff format --check .
uv run pytest                # 35 passed
uv run pgt-check configs/robotiq_2f85.toml          # PASS
uv run pgt-check configs/custom_parallel_gripper.toml  # PASS
uv run python scripts/verify_mujoco.py --steps 60   # PASS
```

### CI（.github/workflows/ci.yml）

- ✅ `uv sync --locked --all-groups`
- ✅ `ruff check .`
- ✅ `ruff format --check .`
- ✅ `pytest`
- ⬜ 建议补充：CI 中增加双 profile `pgt-check` 与 `verify_mujoco.py` 扫掠。

### 人工检查

在 MuJoCo viewer 中检查：

- 完整开闭动作（注意阶段 0：drive 行程上限未复核，勿用满量程）；
- equality 闭环是否分离（`verify_mujoco.py` 自动覆盖）；
- 18 个 Pillars 碰撞面（自动覆盖）；
- 接触点和接触力方向；
- frame 是否仍为独立自由体；
- 指尖是否在机械极限前发生自碰撞。

## 推荐实施顺序

已完成链路（✅ 不再重做）：

```text
Onshape 单根节点与质量修正（已完成）
        ↓
重新导出、后处理、profile 验证（已完成：prepare_onshape_export.py + verify_mujoco.py）
        ↓
Pillars 接触力读取器与单元测试（已完成：contact_taxels.py）
        ↓
CSV / Rerun / 扰动实验接入（已完成：validation + recording + DisturbanceProtocol）
```

剩余工作按优先级：

```text
阶段 0 收尾：pin 关节 range + gripper_drive 行程复核（verify_mujoco 0.149 rad 警告）
        ↓
阶段 2 收尾：pgt-grasp 统一入口 + Robotiq 遗留常量迁移
        ↓
阶段 3 收尾：metadata.json + 绘图按元数据而非文件名
        ↓
阶段 4：实机标定与回归基准（未开始）
```

不要在阶段 0 收尾前使用 drive 满量程做正式实验，也不要在标定前把触觉数据直接用于策略训练。

## 下一次开发建议

下一次开发迭代建议只完成一个闭环目标：

> 复核 `gripper_drive` 机械行程与 `left/right_link_crank_pin` 关节范围，消除
> `verify_mujoco.py` 的 0.1491 rad 跟踪误差警告，使开闭扫掠在导出量程内无警告通过。

这个结果一旦达成，阶段 2 的 `pgt-grasp` 统一入口和阶段 4 的标定实验就可以建立在
一个量程可信的模型上。

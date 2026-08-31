# ✋ 触觉读数约定

项目对外使用的触觉数组均遵循以下约定：

<pre><code>
shape   = (3, rows, cols)
channel = (Fx, Fy, Fz)
unit    = N
frame   = 对应指尖触觉 site 的局部坐标系
</code></pre>

压向传感表面的法向力定义为正，因此正确朝向的触觉 site 在压缩时应报告 `Fz > 0`。
左右指尖各有自己的局部坐标系；不能直接逐分量相加。若要比较左右总力或扰动响应，先使用
该 site 的 `data.site_xmat` 旋转到世界坐标系。

## Robotiq 离散 taxel

`2f85_taxels.xml` 的每个指尖包含 3×3 个 taxel。每个 taxel 是 pad 下的子 body，具有
对齐的 site 与 MuJoCo `force` sensor。MuJoCo 原始传感器力表示“taxel 子 body 施加给 pad
父 body 的力”，因此 `run_cube_grasp_demo.py` 会对完整三维向量取反，记录为“物体施加给
taxel 表面的力”。这样记录中的压缩 `Fz` 为正。

taxel 合力仅代表这些离散触觉单元传递的力，不等同于整个 pad 的完整外力。`pad_force` 与
`pad_torque` 是不同层级的测量，不应与九个 taxel 通道混用。

## Robotiq `touch_grid`

`touch_grid` 是 MuJoCo 插件，而非离散几何 taxel。插件原始通道顺序为 `(Fz, Fx, Fy)`；
演示和记录代码会整理为统一的 `(Fx, Fy, Fz)`。其输出保留牛顿单位，不作归一化或对数变换。

`touch_grid` 的 FOV 是以触觉 site 为中心的半角。修改 pad 尺寸、site 位置或朝向时，必须
重新检查 FOV 是否覆盖整个接触面，否则物理接触存在但部分力不会进入插件输出。

## 自研夹爪 Pillars

自研夹爪的左右指尖各有 3×3 个 Pillars STL。每个 Pillar 的 mesh geom 是唯一的主动指尖
碰撞面，同时也是一个触觉通道。`ContactTaxelReader` 遍历 `data.contact`，通过
`mj_contactForce` 读取接触坐标系力，转换到世界系后再转换到对应触觉 site 局部系，并按
`left/right_taxel_geom_00` 至 `22` 聚合。

Pillars 的 3×3 触觉面不是严格共面：中心 taxel 最高，四个边中间次之，四角最低。当前
MJCF 中中心比四角高约 0.50 mm，边中间比四角高约 0.30 mm。这个几何形状会影响最先接触
的 taxel 顺序，不能把它当作完全平整的 3×3 平面压力阵列。

### 2026-08-31：高载荷接触 A/B 观察

在 `configs/force_tracking/default_waypoints.yaml`（峰值 10 N/侧）、`full` 控制器、`hard`
材料和固定噪声种子 `20260814` 下，原 mesh 碰撞模型出现了明显的高载荷接触切换：活跃接触数在
18 至 72 间变化，跟踪阶段记录到 22 次接触塌陷事件。对应的 RMSE 为 0.222 N、滤波后峰值误差
为 1.661 N，原始触觉力峰值误差为 5.723 N；力矩饱和比例仍为 0%。

对照模型 `parallel_gripper_flat_sphere_collision.xml` 保留原视觉 mesh，仅把 18 个 Pillar 的
碰撞体替换为半径 2.8 mm、面向方块最高点共面的球体。其它实验条件完全相同。该对照中活跃接触数
恒为 18、接触塌陷事件为 0，RMSE 降至 0.092 N、滤波后峰值误差降至 0.231 N，原始触觉力峰值
误差降至 0.195 N。

为区分“非共面”“mesh 几何”和“multiccd”三个因素，随后在相同任务、控制器、材料和噪声种子下
增加三个对照。完整五条件结果如下：

| 碰撞条件 | RMSE (N) | MAE (N) | 峰值误差 (N) | 活跃接触数 | 塌陷事件 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 原非共面 mesh + multiccd | 0.222 | 0.139 | 1.661 | 18–72 | 22 |
| 保留原高度差的球体 + multiccd | 0.094 | 0.072 | 0.236 | 18 | 0 |
| 共面 mesh + multiccd | 0.088 | 0.067 | 0.220 | 72 | 0 |
| 原非共面 mesh，关闭 multiccd | 0.094 | 0.072 | 0.236 | 18 | 0 |
| 共面球体 + multiccd | 0.092 | 0.070 | 0.231 | 18 | 0 |

这些对照修正了只根据原 A/B 得出的过强结论：非共面本身不是充分原因，因为保留高度差的球体仍然
稳定；mesh 和每对四点接触本身也不是充分原因，因为共面 mesh 始终保持 72 个接触且跟踪最好。
高载荷振荡来自三者的交互——原非共面 mesh 在 multiccd 下反复切换接触流形，使总接触数在
18 与 72 之间跳变。关闭 multiccd、消除 mesh 流形或使 mesh 共面，任一种都能消除这种切换。

可使用以下命令复现实验：

```bash
uv run python scripts/experiments/force_tracking_diagnosis.py \
  --config configs/studies/force_tracking_diagnosis.yaml \
  --phase collision-geometry
```

实物 Pillar 确认为“中心高、边中间次之、四角低”后，默认 profile 采用保留该高度差的球体碰撞
代理 `parallel_gripper_height_sphere_collision.xml`，并保持 `multiccd` 开启。它保留分阶段接触的
物理几何趋势，同时避免非共面 mesh 接触流形切换。

原非共面 mesh 加 `--disable-multiccd` 保留为候选物理设置，供后续以实物面接触承载、摩擦和
力—压入标定进行比较；在完成该标定前，不应把它替换为默认模型。

该指尖按 Contactile PapillArray 类传感器处理。公开资料说明 PapillArray 是 soft silicone
pillar 阵列，每个阵列单元可测 3D displacement、3D force 和 vibration；产品规格可参考
[Contactile technology](https://contactile.com/novel-optical-sensing-technology/)、
[Contactile products](https://contactile.com/products/) 和
[Scivaro PapillArray specs](https://www.scivaro.com/index.php?c=show&id=454)。按
`15 N / 2.5 mm` 的 Z 向量程换算，`6000 N/m` 可视为偏硬上界；若 15 N 对应整阵列总量程，
单 pillar 约为 667 N/m。PapillArray 原型论文报告的单 pillar 弹簧常数约为
`1.174 N/mm`，即 `1174 N/m`
（[PapillArray slip sensor paper](https://www.sciencedirect.com/science/article/pii/S0924424717313419)）。
当前 MJCF 采用接近该公开实测值的 `1200 N/m`。

因此同一 Pillar 的多个接触点会累加；无接触单元严格为零。读取器返回
`ContactTaxelFrame(left, right)`，两个数组均为 `(3, 3, 3)`。

## 实机传感器接口

实机传感器接口直接提供以 N 为单位的测量力；本项目不负责将原始电信号、图像或位移标定为力。
适配层只需把左右传感器的有效法向合力规范为 `F_L`、`F_R`，再统一派生控制主量
`f_n=(F_L+F_R)/2`。仿真中的 taxel/Pillar 读取是 MuJoCo 接触力的聚合，不能与实机传感器的
内部信号处理混为一谈。

抓取验收中的仿真触觉观测在理想接触力之上加入依据空载记录建立的测量噪声模型。当前自研夹爪
profile 使用 2026-08-14 Contactile/PapillArray 空载记录的指尖总力噪声，并按 9 个 taxel 独立同分布
假设除以 `sqrt(9)`：左右单 taxel 法向 `sigma=0.0067/0.0133 N`，左右单 taxel 切向
`sigma=0.0033/0.0100 N`。汇总后的法向测量再以 `20 Hz` 一阶低通进入 PID。
这只模拟测量链路，不向 MuJoCo 接触物理施加随机外力。

## 显示与记录

Rerun 中的压力图、切向箭头是可视化；箭头可能为便于观察而聚合或缩放，不能根据屏幕长度
反推牛顿值。CSV 与 RRD 保存的是数值记录，应作为后处理和比较的依据。

用于策略学习时，如需归一化、滤波或对数压缩，应在观测包装层显式完成，不要改变传感器
语义或原始日志。

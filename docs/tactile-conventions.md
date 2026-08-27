# 触觉读数约定

项目对外使用的触觉数组均遵循以下约定：

```text
shape   = (3, rows, cols)
channel = (Fx, Fy, Fz)
unit    = N
frame   = 对应指尖触觉 site 的局部坐标系
```

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

## 显示与记录

Rerun 中的压力图、切向箭头是可视化；箭头可能为便于观察而聚合或缩放，不能根据屏幕长度
反推牛顿值。CSV 与 RRD 保存的是数值记录，应作为后处理和比较的依据。

用于策略学习时，如需归一化、滤波或对数压缩，应在观测包装层显式完成，不要改变传感器
语义或原始日志。

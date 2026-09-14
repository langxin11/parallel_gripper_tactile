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
父 body 的力”，因此 `tactile.py` 的读取器对完整三维向量取反，记录为“物体施加给
taxel 表面的力”。这样记录中的压缩 `Fz` 为正。

taxel 合力仅代表这些离散触觉单元传递的力，不等同于整个 pad 的完整外力。`pad_force` 与
`pad_torque` 是不同层级的测量，不应与九个 taxel 通道混用。

## Robotiq `touch_grid`

`touch_grid` 是 MuJoCo 插件，而非离散几何 taxel。插件原始通道顺序为 `(Fz, Fx, Fy)`；
演示和记录代码会整理为统一的 `(Fx, Fy, Fz)`。其输出保留牛顿单位，不作归一化或对数变换。

`touch_grid` 的 FOV 是以触觉 site 为中心的半角。修改 pad 尺寸、site 位置或朝向时，必须
重新检查 FOV 是否覆盖整个接触面，否则物理接触存在但部分力不会进入插件输出。

## DMgripper Pillars

DMgripper 每侧有 3×3 个 Pillar 通道。默认 `height_spheres` 模型使用保留高度差的球体碰撞代理；
`ContactTaxelReader` 按 `left/right_taxel_geom_00` 至 `22` 聚合 `mj_contactForce`，从接触系经
世界系转换到触觉 site 局部系。多个接触点累加，无接触单元为零；每侧数组为 `(3, 3, 3)`。
中心比四角高约 0.50 mm，边中间比四角高约 0.30 mm，不能按平整阵列解释接触顺序。

### 高载荷接触的定性结论 {: #collision-geometry-conclusions }

在高目标力下，原非共面 mesh 与 `multiccd` 同时启用时会出现接触流形切换、活跃接触数跳变和明显更大的
跟踪误差。保留高度差但改用球体、使 mesh 共面，或关闭 `multiccd` 后，接触数与力跟踪都会明显稳定。

因此，当前模型中的高载荷振荡不是“非共面”“mesh”或“multiccd”任一单独因素的必然结果，而是原非共面
mesh 与多接触点求解方式的交互。该结论描述当前 MuJoCo 模型与任务条件，仍需通过实物接触试验验证。

默认保持高度差球体与 `multiccd` 开启；原 mesh、共面 mesh 与关闭 `multiccd` 的模型只用于对照。
碰撞诊断可选择 `research=archive/model_bug_diagnosis/study study.phase=collision-geometry`。
`solref` 等效接触参数不是独立硅胶形变模型或已完成的实机力学标定，不能由它推导传感器精度。

## 实机传感器接口

实机传感器接口直接提供以 N 为单位的测量力；本项目不负责将原始电信号、图像或位移标定为力。
适配层只需把左右传感器的有效法向合力规范为 `F_L`、`F_R`，再统一派生控制主量
`f_n=(F_L+F_R)/2`。仿真中的 taxel/Pillar 读取是 MuJoCo 接触力的聚合，不能与实机传感器的
内部信号处理混为一谈。

抓取验收中的仿真触觉观测在理想接触力之上加入依据空载记录建立的测量噪声模型。当前 DMgripper
profile 使用 2026-08-14 Contactile/PapillArray 空载记录的指尖总力噪声，并按 9 个 taxel 独立同分布
假设除以 `sqrt(9)`：左右单 taxel 法向 `sigma=0.0067/0.0133 N`，左右单 taxel 切向
`sigma=0.0033/0.0100 N`。汇总后的法向测量再以 `20 Hz` 一阶低通进入 PID。
这只模拟测量链路，不向 MuJoCo 接触物理施加随机外力。

## 显示与记录

Rerun 中的压力图、切向箭头是可视化；箭头可能为便于观察而聚合或缩放，不能根据屏幕长度
反推牛顿值。CSV 与 RRD 保存的是数值记录，应作为后处理和比较的依据。

用于策略学习时，如需归一化、滤波或对数压缩，应在观测包装层显式完成，不要改变传感器
语义或原始日志。

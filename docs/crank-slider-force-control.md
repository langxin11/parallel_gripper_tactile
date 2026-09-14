# 曲柄滑块力控模型

本文为 DMgripper 建立从机构位置、接触柔顺性到目标法向力控制的局部模型。
它适用于左右手指同步闭合、物体大致居中、接触集合在一个控制周期内不变化的准静态阶段。
接触建立、脱离、滑移或新的 Pillar 加入接触时，模型的导数不连续，应退回保守反馈控制。

## 1. 机构开口与闭合行程

以曲柄角 `q`（rad）表示驱动关节位置。两指开口宽度为：

\[
\omega(q)=2\left[r\cos(q+\theta_0)+
\sqrt{l^2-\left(r\sin(q+\theta_0)-e\right)^2}\right]
\]

其中：

\[
\theta_0=\frac{\pi}{4},\qquad r=0.03\ \mathrm{m},\qquad
l=0.04\ \mathrm{m},\qquad e=\frac{0.03}{\sqrt2}\ \mathrm{m},
\qquad q\in[0,1.7].
\]

定义从张开初始位置起算的总闭合行程：

\[
c(q)=\omega(0)-\omega(q).
\]

令 \(\alpha=q+\theta_0\)，并记

\[
s(q)=\sqrt{l^2-\left(r\sin\alpha-e\right)^2},
\]

则总闭合行程相对于曲柄角的运动学雅可比为：

\[
\boxed{
J_c(q)=\frac{\partial c}{\partial q}=
2r\left[\sin\alpha+
\frac{\left(r\sin\alpha-e\right)\cos\alpha}{s(q)}\right]
}
\]

单位为 m/rad。在给定行程中 \(J_c(q)>0\)，即增加 `q` 会闭合夹爪。

| 曲柄角 | 开口宽度 \(\omega\) | 闭合行程雅可比 \(J_c\) |
| --- | ---: | ---: |
| \(q=0\) | 122.43 mm | 42.43 mm/rad |
| \(q=\pi/4\) | 78.05 mm | 60.00 mm/rad |
| \(q=\pi/2\) | 37.57 mm | 42.43 mm/rad |
| \(q=1.7\) | 32.25 mm | 30.49 mm/rad |

因此，夹爪在中间开口附近具有更大的位移传动比；固定的关节位置增益会随姿态表现出不同的力控增益。

## 2. 等效法向刚度

设 \(c_{\mathrm{contact}}\) 是双侧刚建立接触时的闭合行程，并定义接触压入量
\(\Delta c=c(q)-c_{\mathrm{contact}}\)。在当前稳定接触集合和小范围行程内，使用整体等效刚度
\(k_{\mathrm{pair}}\) 近似描述 Pillar—物体—机构/接触链路：

\[
f_n=F_{\mathrm{side}}=k_{\mathrm{pair}}\,\Delta c,\qquad
\]

`k_pair` 是控制用的组合局部斜率，不是材料弹性模量，也不拆分或在线反推任何单独部件参数。
触觉读数仍分别保留左右法向力，并可派生总法向力：

\[
F_\Sigma=F_L+F_R,\qquad f_n=\frac{F_\Sigma}{2}.
\]

后续控制主量采用平均单侧力 \(f_n\)，而不是总法向力 \(F_\Sigma\)。这样目标力、跟踪误差和刚度估计都直接对应夹爪实际施加在物体每一侧的法向夹持力；\(F_\Sigma\) 仅作为派生记录量或摩擦容量计算中的总接触力使用。

这只适用于接触柱集合不变、法向近似一致的局部线性化；不能把所有九个柱简单相加后视为全行程常数。

## 3. 力雅可比与电机力矩

平均单侧法向力相对曲柄角的局部雅可比为：

\[
\boxed{
J_f(q)=\frac{\partial f_n}{\partial q}
\simeq k_{\mathrm{pair}}J_c(q)
}
\]

单位为 N/rad。这是把目标力误差转化为曲柄位置修正时应使用的量。

理想对称、每侧一根刚度 \(k_p\) 的 Pillar 且物体足够硬时，\(k_{pair}=k_p/2\)。
改用总法向力时，对应力雅可比才是上述值的两倍。

虚功关系还给出理想准静态的力矩映射：

\[
\tau\simeq f_nJ_c(q).
\]

因此，电机力矩可作为力估计的辅助信息，但不应取代触觉反馈：关节摩擦、连杆摩擦、闭环约束、惯性和接触切向力都会使该估计产生偏差。

## 4. 用于目标力控制

令 \(e_f=f_{ref}-f_n\)，逆刚度位置修正与机构力矩前馈分别为：

\[
\Delta q_{stiff}=\alpha\frac{e_f}{\hat k_{pair}J_c(q)},\qquad
\tau_{ff}=\beta f_{ref}J_c(q).
\]

逆刚度项与 PI 相加时，二者均依赖实时力误差，因此属于模型辅助反馈，可能重复补偿；
它不是严格意义上的参考前馈。刚度也可用于限制每周期位置增量或把期望力变化率换算成关节速度。
三种实现的离散公式、限幅和控制频率语义统一见[动态目标力跟踪](force-tracking.md)。

对于二阶直接力矩 MB-ADRC，电机输出轴的控制导向模型为：

\[
I_{eq}(q)\ddot q+B_{eq}(q)\dot q+\tau_f(\dot q)+J_c(q)f_n=\tau+d_\tau,
\qquad
f_n\simeq k_{pair}(c-c_{contact})+d_{pair}\dot c+d_f.
\]

把惯量、摩擦、接触阻尼、雅可比变化和刚度误差并入残差扰动，在目标频段内近似为：

\[
\ddot f_n=f_{res}+b_0\tau_{res},\qquad
b_0\simeq s_b\frac{\hat k_{pair}J_c(q)}{I_{eq}}.
\]

\(I_{eq}\) 是输出轴等效惯量，不能用伪造惯量代替输入增益尺度 \(s_b\) 的辨识。
模型前馈 \(\tau_{model}=f_{ref}J_c(q)\) 承担静态力矩，LESO 只使用
\(\tau_{res}=\tau_{applied}-\tau_{model}\) 估计残差。当前实现没有参数收敛律，不能称为在线参数学习控制器。

运动学、刚度估计和控制算法位于 `packages/dm_grasp_core/src/dm_grasp_core/control/`，
仿真适配位于 `src/parallel_gripper_tactile/control.py`。位置、速度与力矩边界由实际组合配置决定。

## 5. 刚度辨识与 MuJoCo 模型解释 {#stiffness-identification}

通过缓慢、小幅的试探压入，可拟合局部关系

\[
\Delta f_n\simeq J_f(q)\Delta q,
\]

进而得到 \(\hat J_f\)，或在已知 \(J_c(q)\) 后反算 \(\hat k_{\mathrm{pair}}\)。
这辨识的是“Pillar—物体—机构/接触链路”组合的整体等效 \(k_{\mathrm{pair}}\)，用于前馈、增益调度
和实验比较；它不表示材料弹性模量，也不用于在线反推物体参数。

MuJoCo 的响应以编译模型中的显式接触对参数为准，不能仅由 geom 参数推断混合结果，
也不能把 `solref` 数值当作材料刚度。

正式真值研究不把 `solref` 的数值直接当作 (N/m) 参考，而是在相同平均单侧力 (f_n) 与总闭合行程
(c) 语义下，对加载、卸载两个分支分别采集平衡工作点。内部点使用中心差分：

\[
k_{\mathrm{ref}}(c_i)\simeq
\frac{f_n^{\mathrm{eq}}(c_{i+1})-f_n^{\mathrm{eq}}(c_{i-1})}
{c_{i+1}-c_{i-1}}.
\]

实际闭合网格通常不等距，实现使用三点非均匀插值导数。采样窗口须同时满足闭合跨度与力跨度阈值；
在线估计器独立记录有效比例，未激励的初值不能作为合格估计。在线拟合不参与参考构造。
同一指令偏移的加载／卸载力差仅为路径依赖诊断，并非严格同一实际闭合位置的材料迟滞。
此参考仍是数值近似：正式选型前还需缩小扫描间距、物理步长并收紧求解器容差，检查排序是否稳定。

跨材料比较以对数 RMSE 为主指标：

\[
\operatorname{RMSE}_{\log k}=\sqrt{\frac{1}{N}\sum_i
\left(\log\hat{k}_i-\log k_{\mathrm{ref},i}\right)^2}.
\]

同时报告相对 RMSE、相对偏差、低估率、估计抖动，以及相邻平衡工作点的力增量预测误差
\(e_{\Delta f}=\Delta f_n-\hat{k}\Delta c\) 的 RMSE 和仅加载分支的正误差 95% 分位数。
这不是一个控制周期内的动态预测误差。力跟踪 RMSE 只用于
评价估计器对下游控制的影响，不能替代上述估计精度指标。

## 6. 适用边界

本文模型不应直接用于以下情况：

- 物体明显偏心，左右法向力不平衡；
- 接触面曲率大，接触法向随位姿快速旋转；
- 接触柱集合频繁增减；
- 材料有显著黏弹性、塑性或加载/卸载滞回；
- 控制带宽接近机构柔性模态或传感器延迟主导的频段。

在这些场景中，应将上述关系视为前馈和增益调度的先验，并持续使用触觉闭环来保证最终力跟踪。

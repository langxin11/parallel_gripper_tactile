# 曲柄滑块二指平行夹爪：接触刚度与目标力控制

本文为自研曲柄滑块二指平行夹爪建立从机构位置、接触柔顺性到目标法向力控制的局部模型。
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
\qquad q\in[0,\pi/2].
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

因此，夹爪在中间开口附近具有更大的位移传动比；固定的关节位置增益会随姿态表现出不同的力控增益。

## 2. 等效法向刚度

在单个稳定的左右接触对中，设左右 Pillar 等效法向刚度为 \(k_L,k_R\)，物体沿夹持方向的等效刚度为 \(k_{\mathrm{obj}}\)。设 \(c_{\mathrm{contact}}\) 是双侧刚建立接触时的闭合行程，并定义接触压入量 \(\Delta c=c(q)-c_{\mathrm{contact}}\)。若忽略机构、装夹和传感器的额外柔顺性，接触保持期间的单侧夹持力近似为：

\[
F_{\mathrm{side}}=k_{\mathrm{pair}}\,\Delta c,\qquad
\frac1{k_{\mathrm{pair}}}=
\frac1{k_L}+\frac1{k_{\mathrm{obj}}}+\frac1{k_R}.
\]

该串联模型表示：接触之后新增的闭合行程由左接触层、物体和右接触层共同吸收。触觉读取采用双侧法向力之和：

\[
F_\Sigma=F_L+F_R\simeq2F_{\mathrm{side}}.
\]

若每侧有多个稳定且并联的 Pillar，则可先取 \(k_L=\sum_i k_{L,i}\)、\(k_R=\sum_i k_{R,i}\)。
这只适用于接触柱集合不变、法向近似一致的局部线性化；不能把所有九个柱简单相加后视为全行程常数。

## 3. 力雅可比与电机力矩

总法向力相对曲柄角的局部雅可比为：

\[
\boxed{
J_F(q)=\frac{\partial F_\Sigma}{\partial q}
\simeq2k_{\mathrm{pair}}J_c(q)
}
\]

单位为 N/rad。这是把目标力误差转化为曲柄位置修正时应使用的量。

对于两侧均为一根刚度 \(k_p\) 的 Pillar、且物体足够硬的理想对称情形，
\(k_{\mathrm{pair}}=k_p/2\)，因此 \(J_F=k_pJ_c\)。若取当前 Pillar 名义刚度
\(k_p=6000\ \mathrm{N/m}\)，则 \(J_F\) 在行程两端约为 \(255\ \mathrm{N/rad}\)，在中间约为
\(360\ \mathrm{N/rad}\)。这些值只是局部小扰动近似，不应用于预测大位移下的接触力。

虚功关系还给出理想准静态的力矩映射：

\[
\tau\simeq F_{\mathrm{side}}J_c(q)
=\frac{F_\Sigma}{2}J_c(q).
\]

因此，电机力矩可作为力估计的辅助信息，但不应取代触觉反馈：关节摩擦、连杆摩擦、闭环约束、惯性和接触切向力都会使该估计产生偏差。

## 4. 用于目标力控制

令 \(e_F=F_{\mathrm{ref}}-F_\Sigma\)，使用估计值 \(\hat J_F(q)\) 的位置前馈为：

\[
\Delta q_{\mathrm{ff}}=\frac{e_F}{\hat J_F(q)}.
\]

建议保留现有力反馈环，并采用受限的组合命令：

\[
\Delta q=
\operatorname{clip}\left(
\alpha\frac{e_F}{\hat J_F(q)}+\Delta q_{\mathrm{PI}},
-\Delta q_{\max},\Delta q_{\max}
\right),
\qquad 0<\alpha<1.
\]

其中 \(\alpha\) 是保守系数，\(\Delta q_{\mathrm{PI}}\) 用来抵消建模误差和稳态偏差。
对较硬物体，\(\hat J_F\) 较大，位置修正会自动减小以抑制过冲；对较软物体，位置修正会增大以更快达到目标力。

实际实现中应：

1. 对 \(\hat J_F\) 进行低通滤波，并设置正的上下限；
2. 仅在双侧接触确认、接触柱集合稳定时更新刚度估计；
3. 在接触建立、脱离、滑移、力突变或接触柱数量变化时冻结估计，并退回保守 PI；
4. 根据 \(1/\hat J_F\) 对位置式力环的比例和积分增益做调度，保持不同物体上的闭环带宽接近；
5. 对 \(\Delta q\)、目标位置、MIT 力矩命令分别限幅，保持在 profile 中定义的机械与执行器范围内。

## 5. 刚度辨识与 MuJoCo 模型解释

通过缓慢、小幅的试探压入，可拟合局部关系

\[
\Delta F_\Sigma\simeq J_F(q)\Delta q,
\]

进而得到 \(\hat J_F\)，或在已知 \(J_c(q)\) 后反算 \(\hat k_{\mathrm{pair}}\)。
这首先辨识的是“Pillar—物体—机构”组合的等效刚度；只有在 Pillar、机构和装夹柔顺性已经独立标定时，才能可靠反推物体本身的 \(k_{\mathrm{obj}}\)。

当前 MuJoCo Pillar 使用 `solref="-6000 -10"` 和
`solimp="0.75 0.95 0.0025 0.5 2"`。目标方块也声明了
`solref="0.015 1"`、`solimp="0.90 0.95 0.002"` 的接触参数。
动态生成的方块—Pillar 接触对中，Pillar 的负值 `solref` 使用直接格式并主导该对的 `solref`；
`solimp` 则依默认接触混合规则合成。因而仿真中从试探接触识别出的刚度是求解器接触层的等效响应，
不等同于实体硅胶或方块材料的独立弹性模量。

## 6. 适用边界

本文模型不应直接用于以下情况：

- 物体明显偏心，左右法向力不平衡；
- 接触面曲率大，接触法向随位姿快速旋转；
- 接触柱集合频繁增减；
- 材料有显著黏弹性、塑性或加载/卸载滞回；
- 控制带宽接近机构柔性模态或传感器延迟主导的频段。

在这些场景中，应将上述关系视为前馈和增益调度的先验，并持续使用触觉闭环来保证最终力跟踪。

## 参考

- [MuJoCo：Solver parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters)
- [MuJoCo：Contact parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters)

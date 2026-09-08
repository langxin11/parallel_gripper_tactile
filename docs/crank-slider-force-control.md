# ⚙️ 曲柄滑块二指平行夹爪：接触刚度与目标力控制

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

对于两侧均为一根刚度 \(k_p\) 的 Pillar、且物体足够硬的理想对称情形，
\(k_{\mathrm{pair}}=k_p/2\)，因此 \(J_f=(k_p/2)J_c\)。若取当前 Pillar 名义刚度
\(k_p=6000\ \mathrm{N/m}\)，则 \(J_f\) 在行程两端约为 \(127\ \mathrm{N/rad}\)，在中间约为
\(180\ \mathrm{N/rad}\)。若改用总力 \(F_\Sigma\)，对应雅可比才会变成这些数值的 2 倍：
在行程两端约为 \(255\ \mathrm{N/rad}\)，在中间约为
\(360\ \mathrm{N/rad}\)。这些值只是局部小扰动近似，不应用于预测大位移下的接触力。

虚功关系还给出理想准静态的力矩映射：

\[
\tau\simeq f_nJ_c(q).
\]

因此，电机力矩可作为力估计的辅助信息，但不应取代触觉反馈：关节摩擦、连杆摩擦、闭环约束、惯性和接触切向力都会使该估计产生偏差。

## 4. 用于目标力控制

令 \(e_f=f_{\mathrm{ref}}-f_n\)，并在线估计平均单侧法向力相对总闭合行程的整体等效刚度：

\[
\hat k_{\mathrm{pair}}\simeq\frac{\Delta f_n}{\Delta c}.
\]

则 \(\hat J_f(q)=\hat k_{\mathrm{pair}}J_c(q)\)，使用估计值的位置前馈为：

\[
\Delta q_{\mathrm{ff}}=\frac{e_f}{\hat k_{\mathrm{pair}}J_c(q)}.
\]

历史 `pid-stiffness-ff` 变体把逆刚度位置修正与 PI 输出相加：

\[
\Delta q=
\operatorname{clip}\left(
\alpha\frac{e_f}{\hat J_f(q)}+\Delta q_{\mathrm{PI}},
-\Delta q_{\max},\Delta q_{\max}
\right),
\qquad 0<\alpha<1.
\]

其中 \(\alpha\) 是保守系数，\(\Delta q_{\mathrm{PI}}\) 用来抵消建模误差和稳态偏差。
对较硬物体，\(\hat J_f\) 较大，位置修正会自动减小以抑制过冲；对较软物体，位置修正会增大以更快达到目标力。
由于该项和 PI 同时使用实时误差，它属于模型辅助反馈而不是严格意义上的参考前馈，估计误差还可能与 PI
重复补偿。新的 `pid-stiffness-limit` 因此关闭该加法项，仅把在线刚度用于 PID 位置目标的周期增量约束。
设允许的预测平均单侧力变化率为 \(\dot f_{lim}\)，刚度安全系数为 \(\gamma_k\ge1\)，则：

\[
k_{safe,k}=\gamma_k\hat k_{pair,k},\qquad
\Delta f_{lim,k}=\min\left(|e_{f,k}|,\dot f_{lim}\Delta t\right),
\]

\[
\Delta q_{lim,k}=\frac{\Delta f_{lim,k}}{k_{safe,k}J_c(q_k)},
\]

\[
\Delta q_{cmd,k}=\operatorname{clip}\left(
\Delta q_{PI,k},
\Delta q_{cmd,k-1}-\Delta q_{lim,k},
\Delta q_{cmd,k-1}+\Delta q_{lim,k}
\right).
\]

该动态边界同时受全局 `max_position_adjustment` 约束，并直接设置为 PID 的输出上下界，使积分项在限幅期间
同步裁剪。误差为零时 \(\Delta q_{lim,k}=0\)，控制器保持上一周期的平衡位置，而不是把绝对位置修正拉回零。
该变体默认保留机构力矩前馈；`position_limit_force_rate_n_s` 与
`position_limit_stiffness_safety_factor` 仍是待正式 study 验证的仿真起点，不是实机安全认证参数。

同时使用开度雅可比计算准静态力矩前馈：

\[
\tau_{\mathrm{ff}}=\beta f_{\mathrm{ref}}J_c(q),
\qquad 0\le\beta\le1.
\]

该项只作为 MIT `t_ff` 的前馈输入，最终仍由达妙协议量化和 `T_MAX` 限幅保护。

也可以构造直接力矩式力控。此时 MIT 内环的 `kp` 和 `kd` 设为 0，不再通过位置误差产生输出力矩，
而是把力反馈项和机构模型前馈直接合成为 `t_ff`：

\[
t_{\mathrm{ff}}=
\tau_{\mathrm{force\ feedback}}+
\tau_{\mathrm{model\ feedforward}}.
\]

其中一种最小形式为：

\[
\tau_{\mathrm{force\ feedback}}=
K_p^F e_f+
K_i^F\int e_f\,dt+
K_d^F\frac{de_f}{dt},
\qquad
\tau_{\mathrm{model\ feedforward}}=
\beta f_{\mathrm{ref}}J_c(q).
\]

该模式更直接地测试“力误差到电机力矩”的闭环，不受 MIT 位置刚度主导，适合作为当前位置式力控的对照。
但它接触前没有位置弹簧提供闭合趋势，因此仍需要单独的接近阶段；接触后也必须处理噪声、积分饱和、
脱离接触和 `T_MAX` 限幅。实现时应至少保留接触状态机、低通滤波、积分 anti-windup 和力矩斜率限制。

对于二阶直接力矩 ADRC，采用电机输出轴上的控制导向动力学：

\[
I_{eq}(q)\ddot q+B_{eq}(q)\dot q+\tau_f(\dot q)+J_c(q)f_n=\tau+d_\tau,
\]

\[
f_n\simeq k_{pair}(c-c_{contact})+d_{pair}\dot c+d_f.
\]

将惯量、摩擦、接触阻尼、雅可比变化与刚度误差并入残差总扰动后，在目标频段内近似为：

\[
\ddot f_n=f_{res}+b_0\tau_{res},\qquad
b_0\simeq s_b\frac{\hat k_{pair}J_c(q)}{I_{eq}}.
\]

这里 `s_b` 是由小信号 `τ→f_n` 辨识校准的输入增益尺度，不应通过伪造惯量来调节；
`I_eq` 仍保留明确的输出轴等效惯量物理意义。机构前馈
`τ_model=f_ref·J_c(q)` 单独承担名义静态力矩，LESO 只使用
`τ_res=τ_applied-τ_model` 作为已知输入并估计剩余 `f_res`。

实际实现中应：

1. 对 \(\hat k_{\mathrm{pair}}\) 进行低通滤波，并设置正的上下限；
2. 仅在双侧接触确认后更新刚度估计，接触柱集合变化时应冻结估计；
3. 在接触建立、脱离、滑移、力突变或接触柱数量变化时冻结估计，并退回保守 PI；
4. 根据 \(1/\hat J_f\) 对位置式力环的比例和积分增益做调度，保持不同物体上的闭环带宽接近；
5. 对 \(\Delta q\)、目标位置、MIT 力矩命令分别限幅，保持在 profile 中定义的机械与执行器范围内。

当前实现位于 `src/parallel_gripper_tactile/control.py`：

- `CrankSliderKinematics` 计算 \(\omega(q)\)、\(c(q)\) 和 \(J_c(q)\)；
- `ContactStiffnessEstimator` 用滑动窗口拟合或历史 secant-EWMA 方法估计 \(\hat k_{\mathrm{pair}}\)；
- `NormalForceController` 将 \(\Delta q_{\mathrm{ff}}\)、PI 修正和 \(\tau_{\mathrm{ff}}\) 合并后交给 MIT 力矩内环。

直接力矩式力控已实现为 `direct-torque` 控制器变体，profile 入口是
`control.force.torque_feedback_gain`（大于 0 时启用，`direct-torque` 变体取 1.0）。
跟踪阶段由控制器逐周期把 MIT kp/kd 覆盖为 0（profile 增益不动，接近阶段仍用同一组增益
做位置伺服闭合），力误差与模型前馈按上式合成 `t_ff`，仍经达妙量化与 `T_MAX` 限幅；
刚度估计器照常运行以保持 trace 中刚度曲线可比。该变体与当前位置式控制器共享同一个
`force-track` benchmark，见 [控制器对比研究](control-comparison-ablation.md)。

二阶直接力矩 MB-ADRC 已实现为 `adrc-torque` 变体，profile 入口是
`control.force.torque_adrc`。其三状态 current LESO 估计力、力变化率和残差总扰动；名义 PD
位于 ADRC 控制律内部，因此跟踪阶段可以旁路 MIT `kp/kd`。接近到跟踪的切换使用上一周期实际力矩
初始化扰动状态，在线调度 `b0` 时同步缩放扰动状态；力矩变化率和幅值限制后的实际输入会反馈给
下一周期 LESO。触觉力先经过独立的 40 Hz 一阶轻度预处理再进入 LESO，不复用 PID、刚度估计与
指标使用的 20 Hz 公共低通；力和力变化率的主要估计仍由 LESO 完成。当前实现仍是 MB-ADRC，
不包含参数收敛律；PL-ADRC 应在完成实机
`τ→q̈`、`c→f_n`、`τ→f_n` 辨识后另行增加带投影约束的参数学习通道。

## 5. 刚度辨识与 MuJoCo 模型解释

通过缓慢、小幅的试探压入，可拟合局部关系

\[
\Delta f_n\simeq J_f(q)\Delta q,
\]

进而得到 \(\hat J_f\)，或在已知 \(J_c(q)\) 后反算 \(\hat k_{\mathrm{pair}}\)。
这辨识的是“Pillar—物体—机构/接触链路”组合的整体等效 \(k_{\mathrm{pair}}\)，用于前馈、增益调度
和实验比较；它不表示材料弹性模量，也不用于在线反推物体参数。

当前 MuJoCo Pillar 使用 `solref="-1200 -10"` 和
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

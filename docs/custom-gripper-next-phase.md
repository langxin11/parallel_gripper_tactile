# 自研夹爪状态与后续工作

自研曲柄滑块平行夹爪已接入项目的 profile、接触触觉读取、MIT 力矩控制、法向力闭环、
抓取验收与视频记录链路。本页记录当前能力边界及下一步工作，不把已落地的功能误标为路线图。

## 已具备的能力

| 能力 | 实现位置 |
| --- | --- |
| 3×3 Pillars 作为唯一指尖接触与触觉通道 | `parallel_gripper_prepared.xml`、`contact_taxels.py` |
| 左右 `(Fx, Fy, Fz)` 接触力阵列 | `ContactTaxelReader` |
| Profile 驱动的模型、安装、控制与触觉布局 | `profiles.py`、`configs/custom_parallel_gripper.toml` |
| 受限 MIT 力矩内环 | `MITTorqueController` |
| 双侧接触确认后的法向力跟踪 | `NormalForceController` |
| 无支撑保持、切向扰动与恢复实验 | `run_custom_grasp_validation.py`、`DisturbanceProtocol` |
| 离线 CSV、图像和 MP4 输出 | 抓取验收与视频脚本 |
| MJCF、触觉符号、控制与场景回归测试 | `tests/`、`pgt-check` |

默认法向力目标、接触阈值、MIT 限制和安装位姿都由
`configs/custom_parallel_gripper.toml` 管理。改变控制或触觉行为时，应优先评估 profile 参数，
避免在多个脚本中复制模型专属常量。

## 当前实验边界

抓取验收使用 `6 × 25 × 25 mm` 的测试块，默认质量为 `50 g`，夹爪横向安装。实验时序为：

```text
1.0 s 闭合
→ 0.5 s 带支撑稳定
→ 0.5 s 无支撑保持
→ 1.0 s 切向正弦扰动
→ 0.5 s 恢复
```

默认扰动是世界 Y 方向 `5 N`、`2 Hz` 正弦力；滑移以世界 YZ 接触平面内的位移判断，默认门限
为 `2 mm`。验收结果还包含法向力跟踪误差、可用摩擦容量和仿真数值稳定性。

该仿真链路用于比较与回归，不等同于对实机抗滑移能力的保证。接触参数、摩擦、Pillar 材料、
机构柔顺性和电机连续热限制尚未通过实测标定。

## 建议的后续工作

1. **机械与行程复核**：在 CAD、MJCF 扫掠和实机之间核对主动关节及被动销轴的可达范围，
   并在每次导出后运行 `verify_mujoco.py`。
2. **触觉标定**：进行单 Pillar 法向/切向加载，得到增益、零偏、饱和、摩擦和接触刚度的可追溯参数。
3. **实验元数据**：为 CSV、图像和视频补充模型版本、profile、时间步和接触参数，方便复现实验。
4. **统一实验入口**：在现有专用脚本稳定后，再考虑以一个 profile 参数化命令统一 Robotiq 与自研夹爪。
5. **实机闭环验证**：基于测得的噪声、延迟和热限制，为法向力控制与滑移检测建立验收阈值。

## 最小回归检查

```bash
uv run pgt-check configs/custom_parallel_gripper.toml
uv run scripts/verify_mujoco.py
uv run pytest
```

交互式检查安装姿态和闭合状态：

```bash
uv run scripts/view_custom_grasp_scene.py
uv run scripts/view_custom_grasp_scene.py --closed
```

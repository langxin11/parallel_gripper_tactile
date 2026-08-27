"""由 demo、benchmark 与视频脚本共享的抓取实验时序协议。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Vector3 = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class DisturbanceProtocol:
    """释放-扰动抓取实验的时序与波形。

    该实验闭合夹爪，让物体在临时支撑上稳定，移除支撑，在无支撑状态下
    保持物体，在物体的质心处施加切向正弦力，并观察恢复过程。力的方向
    可以在世界 YZ 切平面内以恒定角速度旋转。

    Attributes:
        close_duration: 闭合斜坡的持续时间 (s)。
        support_settle_duration: 支撑生效期间的稳定时间 (s)。
        hold_duration: 支撑释放后无支撑保持的时间 (s)。
        disturbance_duration: 切向扰动的持续时间 (s)。
        recovery_duration: 扰动结束后观察的时间 (s)。
        force_n: 扰动力的幅值 (N)。
        frequency_hz: 正弦扰动的频率 (Hz)。
        rotation_rate_rad_s: 力方向在 YZ 平面内的角速度 (rad/s)；为 0
            时力保持沿世界 Y 方向。
        slip_threshold_m: 定义打滑的切向位移阈值 (m)。
    """

    close_duration: float = 1.0
    support_settle_duration: float = 0.5
    hold_duration: float = 0.5
    disturbance_duration: float = 1.0
    recovery_duration: float = 0.5
    force_n: float = 5.0
    frequency_hz: float = 2.0
    rotation_rate_rad_s: float = 0.0
    slip_threshold_m: float = 0.002

    def __post_init__(self) -> None:
        """拒绝非正持续时间与负幅值。"""
        positive = {
            "close_duration": self.close_duration,
            "disturbance_duration": self.disturbance_duration,
            "frequency_hz": self.frequency_hz,
            "slip_threshold_m": self.slip_threshold_m,
        }
        nonnegative = {
            "support_settle_duration": self.support_settle_duration,
            "hold_duration": self.hold_duration,
            "recovery_duration": self.recovery_duration,
            "force_n": self.force_n,
            "rotation_rate_rad_s": self.rotation_rate_rad_s,
        }
        for name, value in positive.items():
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        for name, value in nonnegative.items():
            if value < 0:
                raise ValueError(f"{name} must be nonnegative")

    @property
    def release_time(self) -> float:
        """临时支撑被移除的时刻。"""
        return self.close_duration + self.support_settle_duration

    @property
    def disturbance_start(self) -> float:
        """切向扰动开始的时刻。"""
        return self.release_time + self.hold_duration

    @property
    def disturbance_end(self) -> float:
        """切向扰动结束的时刻。"""
        return self.disturbance_start + self.disturbance_duration

    @property
    def total_duration(self) -> float:
        """仿真实验的总持续时间 (s)。"""
        return self.disturbance_end + self.recovery_duration

    def phase_at(self, time_s: float) -> str:
        """返回包含 ``time_s`` 的实验阶段。"""
        if time_s < self.close_duration:
            return "close"
        if time_s < self.release_time:
            return "support_settle"
        if time_s < self.disturbance_start:
            return "unsupported_hold"
        if time_s < self.disturbance_end:
            return "disturbance"
        return "recovery"

    def force_vector_at(self, time_s: float) -> np.ndarray:
        """返回施加在物体质心处的世界坐标系三维力。"""
        if not self.disturbance_start <= time_s < self.disturbance_end:
            return np.zeros(3, dtype=np.float64)
        elapsed = time_s - self.disturbance_start
        magnitude = self.force_n * math.sin(2.0 * math.pi * self.frequency_hz * elapsed)
        angle = self.rotation_rate_rad_s * elapsed
        return np.array(
            [0.0, magnitude * math.cos(angle), magnitude * math.sin(angle)],
            dtype=np.float64,
        )

    def force_y_at(self, time_s: float) -> float:
        """返回扰动力的世界 Y 分量。"""
        return float(self.force_vector_at(time_s)[1])

    def close_target_at(self, time_s: float, open_control: float, closed_control: float) -> float:
        """返回该时刻线性斜坡后的夹爪位置目标。"""
        progress = min(1.0, max(0.0, time_s / self.close_duration))
        return open_control + progress * (closed_control - open_control)

    def step(
        self,
        model,
        data,
        *,
        actuator_id: int,
        close_control: float,
        support_geom_id: int,
        cube_body_id: int,
        apply_actuator_control: bool = True,
    ) -> Vector3:
        """应用本步的控制、支撑状态与物体力。

        驱动目标在 ``close_duration`` 内线性上升。在 ``release_time`` 之后，
        支撑几何体将其 ``contype``/``conaffinity`` 置零从而退出碰撞过滤，
        这样支撑摩擦力就无法抵抗切向扰动。世界坐标系力会被写入
        ``xfrc_applied`` 的前三个分量并返回。

        Returns:
            所施加的世界坐标系力向量。
        """
        time_s = float(data.time)
        if apply_actuator_control:
            data.ctrl[actuator_id] = close_control * min(1.0, time_s / self.close_duration)
        if time_s >= self.release_time:
            model.geom_contype[support_geom_id] = 0
            model.geom_conaffinity[support_geom_id] = 0
        data.xfrc_applied[cube_body_id] = 0.0
        applied = self.force_vector_at(time_s)
        data.xfrc_applied[cube_body_id, :3] = applied
        return (float(applied[0]), float(applied[1]), float(applied[2]))

"""刚度归一化速率控制器的小规模参数调优配置。"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

from ..config.profiles import StiffnessEstimatorMethod
from ..scenes.custom import ObjectMaterial
from .force_tracking_ablation import SeedSweep


class _TuningModel(BaseModel):
    """拒绝未知字段且不可变的调优配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


@dataclass(frozen=True, slots=True)
class StiffnessRateCandidate:
    """一组可复现的速率式 PID 参数。"""

    kp_s_inv: float
    max_force_rate_n_s: float

    @property
    def identifier(self) -> str:
        """返回适合条件目录与结果表的稳定标识。"""
        return f"kp{self.kp_s_inv:g}-rate{self.max_force_rate_n_s:g}"


class ForceTrackingStiffnessRateTuningConfig(_TuningModel):
    """按候选、任务、材料和 seed 展开的速率控制调优方案。"""

    name: str = Field(default="force_tracking_stiffness_rate_tuning", min_length=1)
    analysis_mode: Literal["tuning", "confirmation"] = "tuning"
    profile: Path = Field(default=Path("configs/dm_gripper.yaml"), exclude=True)
    tasks: tuple[Path, ...]
    stiffness_estimator_method: StiffnessEstimatorMethod = "window_linear"
    materials: tuple[ObjectMaterial, ...]
    seeds: SeedSweep = SeedSweep()
    kp_s_inv: tuple[float, ...]
    max_force_rate_n_s: tuple[float, ...]
    max_joint_velocity_rad_s: float = Field(default=0.5, gt=0.0)
    max_plateau_force_std_n: float = Field(default=0.03, gt=0.0)
    max_overshoot_ratio: float = Field(default=0.10, ge=0.0)
    steady_window_s: tuple[float, float] = (1.4, 2.9)
    output_root: Path = Field(default=Path("outputs/studies"), exclude=True)

    @field_validator("tasks", "materials", "kp_s_inv", "max_force_rate_n_s")
    @classmethod
    def require_nonempty_unique(cls, value: tuple[object, ...]) -> tuple[object, ...]:
        """拒绝空维度和重复条件。"""
        if not value:
            raise ValueError("must contain at least one value")
        if len(set(value)) != len(value):
            raise ValueError("must not contain duplicate values")
        return value

    @field_validator("kp_s_inv", "max_force_rate_n_s")
    @classmethod
    def require_positive_values(cls, value: tuple[float, ...]) -> tuple[float, ...]:
        """调优网格只接受正参数。"""
        if any(item <= 0 for item in value):
            raise ValueError("tuning values must be positive")
        return value

    @field_validator("steady_window_s")
    @classmethod
    def require_increasing_window(cls, value: tuple[float, float]) -> tuple[float, float]:
        """稳态统计窗口必须为非负递增区间。"""
        if value[0] < 0 or value[1] <= value[0]:
            raise ValueError("steady_window_s must be non-negative and increasing")
        return value

    def candidates(self) -> tuple[StiffnessRateCandidate, ...]:
        """按配置顺序展开 K_P 与最大力变化率的笛卡尔积候选。"""
        return tuple(
            StiffnessRateCandidate(kp, force_rate)
            for kp, force_rate in product(self.kp_s_inv, self.max_force_rate_n_s)
        )

    def conditions(
        self,
    ) -> tuple[tuple[StiffnessRateCandidate | None, Path, ObjectMaterial, int], ...]:
        """展开性能基线与全部候选的配对条件。"""
        controllers: tuple[StiffnessRateCandidate | None, ...] = (None, *self.candidates())
        return tuple(product(controllers, self.tasks, self.materials, self.seeds.values()))


__all__ = ["ForceTrackingStiffnessRateTuningConfig", "StiffnessRateCandidate"]

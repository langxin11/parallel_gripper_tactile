"""刚度位置限幅三臂配对研究的配置模型。"""

from __future__ import annotations

from itertools import product
from pathlib import Path
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from ..scenes.custom import ObjectMaterial
from .force_tracking_ablation import SeedSweep


StiffnessLimitMode = Literal["no-limit", "window-linear", "oracle-k"]


class ForceTrackingStiffnessLimitConfig(BaseModel):
    """固定控制结构，仅改变刚度位置限幅输入的研究配置。"""

    model_config = ConfigDict(extra="forbid", frozen=True)

    name: str = Field(default="force_tracking_stiffness_limit_pilot", min_length=1)
    profile: Path = Field(default=Path("configs/dm_gripper.yaml"), exclude=True)
    tasks: tuple[Path, ...]
    modes: tuple[StiffnessLimitMode, ...] = ("no-limit", "window-linear", "oracle-k")
    materials: tuple[ObjectMaterial, ...]
    seeds: SeedSweep = SeedSweep()
    oracle_stiffness_n_per_m: dict[ObjectMaterial, Annotated[float, Field(gt=0)]]
    contact_peak_window_s: Annotated[float, Field(gt=0)] = 0.2
    position_limit_force_rate_n_s: Annotated[float, Field(gt=0)] = 50.0
    output_root: Path = Field(default=Path("outputs/studies"), exclude=True)

    @field_validator("tasks", "modes", "materials")
    @classmethod
    def require_nonempty_unique(cls, value: tuple[object, ...]) -> tuple[object, ...]:
        """拒绝空维度与重复条件。"""
        if not value:
            raise ValueError("must contain at least one value")
        if len(set(value)) != len(value):
            raise ValueError("must not contain duplicate values")
        return value

    @model_validator(mode="after")
    def require_oracle_for_every_material(self) -> "ForceTrackingStiffnessLimitConfig":
        """要求每种材料都有冻结的独立参考刚度。"""
        missing = set(self.materials) - set(self.oracle_stiffness_n_per_m)
        if missing:
            raise ValueError(f"oracle stiffness is missing for materials: {sorted(missing)}")
        return self

    def conditions(self) -> tuple[tuple[StiffnessLimitMode, Path, ObjectMaterial, int], ...]:
        """按 mode × task × material × seed 顺序展开条件。"""
        return tuple(product(self.modes, self.tasks, self.materials, self.seeds.values()))


__all__ = ["ForceTrackingStiffnessLimitConfig", "StiffnessLimitMode"]

"""等效接触刚度参考验证研究的显式配置模型。"""

from __future__ import annotations

from itertools import product
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, field_validator

from ..config.profiles import StiffnessEstimatorMethod
from ..scenes.custom import ObjectMaterial
from .force_tracking_ablation import SeedSweep


class StiffnessGroundTruthValidationConfig(BaseModel):
    """展开估计器、接触材料和噪声 seed 的准静态标定矩阵。"""

    model_config = ConfigDict(extra="forbid", frozen=True)

    name: str = Field(default="stiffness_ground_truth_validation", min_length=1)
    profile: Path = Field(default=Path("configs/dm_gripper.yaml"), exclude=True)
    task: Path
    estimators: tuple[StiffnessEstimatorMethod, ...]
    materials: tuple[ObjectMaterial, ...]
    seeds: SeedSweep = SeedSweep()
    output_root: Path = Field(default=Path("outputs/studies"), exclude=True)

    @field_validator("estimators", "materials")
    @classmethod
    def require_nonempty_unique(cls, value: tuple[str, ...]) -> tuple[str, ...]:
        """拒绝空条件维度与重复条件。"""
        if not value:
            raise ValueError("must contain at least one value")
        if len(set(value)) != len(value):
            raise ValueError("must not contain duplicate values")
        return value

    def conditions(self) -> tuple[tuple[StiffnessEstimatorMethod, ObjectMaterial, int], ...]:
        """按 estimator × material × seed 的固定顺序展开条件矩阵。"""
        return tuple(product(self.estimators, self.materials, self.seeds.values()))


__all__ = ["SeedSweep", "StiffnessGroundTruthValidationConfig"]

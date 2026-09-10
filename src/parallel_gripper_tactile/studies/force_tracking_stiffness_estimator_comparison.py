"""滑动窗刚度估计器对比研究的显式配置模型。"""

from __future__ import annotations

from itertools import product
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator
import yaml

from ..config.profiles import StiffnessEstimatorMethod
from ..scenes.custom import ObjectMaterial
from .force_tracking_ablation import SeedSweep, StudyConfigError


class _EstimatorComparisonStudyModel(BaseModel):
    """拒绝未知字段且不可变的刚度估计器 study 配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class ForceTrackingStiffnessEstimatorComparisonConfig(_EstimatorComparisonStudyModel):
    """固定位置刚度前馈控制器、展开估计器对比条件的 protocol。"""

    name: str = Field(default="force_tracking_stiffness_estimator_comparison", min_length=1)
    profile: Path
    tasks: tuple[Path, ...]
    estimators: tuple[StiffnessEstimatorMethod, ...]
    materials: tuple[ObjectMaterial, ...]
    seeds: SeedSweep = SeedSweep()
    output_root: Path = Path("outputs/studies")

    @field_validator("tasks", "estimators", "materials")
    @classmethod
    def require_nonempty_unique(
        cls, value: tuple[Path, ...] | tuple[str, ...]
    ) -> tuple[Path, ...] | tuple[str, ...]:
        """拒绝空条件维度与重复条件，避免无效或重复执行。"""
        if not value:
            raise ValueError("must contain at least one value")
        if len(set(value)) != len(value):
            raise ValueError("must not contain duplicate values")
        return value

    def conditions(self) -> tuple[tuple[StiffnessEstimatorMethod, Path, ObjectMaterial, int], ...]:
        """按 estimator × task × material × seed 的配置顺序展开条件矩阵。"""
        return tuple(product(self.estimators, self.tasks, self.materials, self.seeds.values()))


def _resolve_relative_path(path: Path, *, base: Path) -> Path:
    """将 YAML 中的相对路径解析为相对该 YAML 文件的绝对路径。"""
    return (base / path).resolve() if not path.is_absolute() else path


def load_stiffness_estimator_comparison_config(
    path: str | Path,
) -> ForceTrackingStiffnessEstimatorComparisonConfig:
    """加载估计器对比 YAML，并解析相对输入、输出路径。"""
    config_path = Path(path).resolve()
    try:
        with config_path.open(encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except OSError as error:
        raise StudyConfigError(
            f"cannot read stiffness estimator comparison config: {config_path}"
        ) from error
    except yaml.YAMLError as error:
        raise StudyConfigError(f"invalid YAML in {config_path}") from error
    if not isinstance(raw, dict):
        raise StudyConfigError(
            f"stiffness estimator comparison config root must be a mapping: {config_path}"
        )
    try:
        config = ForceTrackingStiffnessEstimatorComparisonConfig.model_validate(raw)
    except ValidationError as error:
        raise StudyConfigError(str(error)) from error

    base = config_path.parent
    return config.model_copy(
        update={
            "profile": _resolve_relative_path(config.profile, base=base),
            "tasks": tuple(_resolve_relative_path(task, base=base) for task in config.tasks),
            "output_root": _resolve_relative_path(config.output_root, base=base),
        }
    )


__all__ = [
    "ForceTrackingStiffnessEstimatorComparisonConfig",
    "SeedSweep",
    "StudyConfigError",
    "load_stiffness_estimator_comparison_config",
]

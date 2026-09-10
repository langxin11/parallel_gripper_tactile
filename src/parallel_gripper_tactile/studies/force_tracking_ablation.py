"""力跟踪消融研究的显式配置模型。"""

from __future__ import annotations

from itertools import product
from pathlib import Path
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator
import yaml

from ..experiments.force_tracking import CONTROLLER_VARIANTS, ControllerVariant
from ..scenes.custom import ObjectMaterial


class StudyConfigError(ValueError):
    """当 study YAML 无法解析或未通过 schema 校验时抛出。"""


class _StudyModel(BaseModel):
    """拒绝未知字段的不可变 study 配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class SeedSweep(_StudyModel):
    """连续的、可复现的传感器噪声种子范围。"""

    start: int = Field(default=0, ge=0)
    count: int = Field(default=3, ge=1)

    def values(self) -> tuple[int, ...]:
        """返回此次 study 要执行的种子。"""
        return tuple(range(self.start, self.start + self.count))


class ForceTrackingAblationConfig(_StudyModel):
    """控制器、材料和噪声种子组成的力跟踪消融 protocol。"""

    name: str = Field(default="force_tracking_ablation", min_length=1)
    profile: Path
    task: Path
    controllers: tuple[ControllerVariant, ...]
    materials: tuple[ObjectMaterial, ...]
    seeds: SeedSweep = SeedSweep()
    output_root: Path = Path("outputs/studies")

    @field_validator("controllers", "materials")
    @classmethod
    def require_nonempty(cls, value: tuple[str, ...]) -> tuple[str, ...]:
        """拒绝没有任何条件的 study。"""
        if not value:
            raise ValueError("must contain at least one value")
        if len(set(value)) != len(value):
            raise ValueError("must not contain duplicate values")
        return value

    def conditions(self) -> tuple[tuple[ControllerVariant, ObjectMaterial, int], ...]:
        """按配置顺序展开 controller × material × seed 矩阵。"""
        return tuple(product(self.controllers, self.materials, self.seeds.values()))


def load_study_config(path: str | Path) -> ForceTrackingAblationConfig:
    """加载 study YAML；相对输入与输出路径相对于 YAML 文件解析。"""
    config_path = Path(path).resolve()
    try:
        with config_path.open(encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except OSError as error:
        raise StudyConfigError(f"cannot read study config: {config_path}") from error
    except yaml.YAMLError as error:
        raise StudyConfigError(f"invalid YAML in {config_path}") from error
    if not isinstance(raw, dict):
        raise StudyConfigError(f"study config root must be a mapping: {config_path}")
    base = config_path.parent
    study = raw.get("study")
    if isinstance(study, dict) and isinstance(study.get("definition"), dict):
        raw = study["definition"]
        base = Path(__file__).resolve().parents[3]
    try:
        config = ForceTrackingAblationConfig.model_validate(raw)
    except ValidationError as error:
        raise StudyConfigError(str(error)) from error
    return config.model_copy(
        update={
            "profile": (base / config.profile).resolve()
            if not config.profile.is_absolute()
            else config.profile,
            "task": (base / config.task).resolve()
            if not config.task.is_absolute()
            else config.task,
            "output_root": (base / config.output_root).resolve()
            if not config.output_root.is_absolute()
            else config.output_root,
        }
    )


__all__ = [
    "CONTROLLER_VARIANTS",
    "ForceTrackingAblationConfig",
    "SeedSweep",
    "StudyConfigError",
    "load_study_config",
]

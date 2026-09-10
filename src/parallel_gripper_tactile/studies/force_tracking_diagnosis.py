"""力跟踪因果诊断研究的显式配置与 phase 定义。"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator
import yaml

from ..experiments.force_tracking import CONTROLLER_VARIANTS, ControllerVariant
from ..scenes.custom import ObjectContactModel, ObjectMaterial


Phase = Literal[
    "reproducibility",
    "controllers",
    "materials",
    "force-scale",
    "contact-model",
    "collision-geometry",
    "force-semantics",
    "position-limit",
    "integral-gain",
    "filter-cutoff",
]
ALL_PHASES: tuple[Phase, ...] = (
    "reproducibility",
    "controllers",
    "materials",
    "force-scale",
    "contact-model",
    "collision-geometry",
    "force-semantics",
    "position-limit",
    "integral-gain",
    "filter-cutoff",
)


class DiagnosisConfigError(ValueError):
    """诊断 study 配置无效时抛出。"""


class CollisionGeometryCondition(BaseModel):
    """一项碰撞几何模型及其多接触设置。"""

    model_config = ConfigDict(extra="forbid", frozen=True)

    label: str = Field(min_length=1)
    model: Path
    multiccd_enabled: bool = True


class DiagnosisConfig(BaseModel):
    """一个力跟踪诊断 protocol 的精简显式配置。"""

    model_config = ConfigDict(extra="forbid", frozen=True)

    name: str = Field(default="force_tracking_diagnosis", min_length=1)
    profile: Path
    task: Path
    output_root: Path = Path("outputs/studies")
    sensor_noise_seed: int = Field(default=20260814, ge=0)
    reproducibility_repeats: int = Field(default=2, ge=2)
    controllers: tuple[ControllerVariant, ...] = CONTROLLER_VARIANTS
    materials: tuple[ObjectMaterial, ...] = ("soft", "medium", "hard")
    force_scales: tuple[Annotated[float, Field(gt=0)], ...] = (0.5, 1.0)
    contact_models: tuple[ObjectContactModel, ...] = ("explicit", "legacy")
    collision_geometry_models: tuple[CollisionGeometryCondition, ...]
    stability_controller: ControllerVariant = "pid-torque-ff"
    position_adjustment_limits_rad: tuple[Annotated[float, Field(gt=0)], ...] = (
        0.15,
        0.10,
        0.075,
        0.05,
        0.03,
    )
    integral_gains: tuple[Annotated[float, Field(ge=0)], ...] = (0.2, 0.1, 0.05, 0.025, 0.0)
    filter_cutoffs_hz: tuple[Annotated[float, Field(gt=0)], ...] = (20.0, 30.0, 40.0, 60.0)

    @field_validator(
        "controllers",
        "materials",
        "force_scales",
        "contact_models",
        "position_adjustment_limits_rad",
        "integral_gains",
        "filter_cutoffs_hz",
    )
    @classmethod
    def require_nonempty(cls, value: tuple[object, ...]) -> tuple[object, ...]:
        """拒绝空的或重复的诊断维度。"""
        if not value:
            raise ValueError("至少需要包含一个值。")
        if len(set(value)) != len(value):
            raise ValueError("不能包含重复值。")
        return value

    @field_validator("collision_geometry_models")
    @classmethod
    def require_unique_collision_labels(
        cls, value: tuple[CollisionGeometryCondition, ...]
    ) -> tuple[CollisionGeometryCondition, ...]:
        """拒绝空的几何研究或重复的条件标签。"""
        if not value:
            raise ValueError("至少需要包含一个碰撞几何。")
        labels = [condition.label for condition in value]
        if len(set(labels)) != len(labels):
            raise ValueError("碰撞几何标签必须唯一。")
        return value


def load_diagnosis_config(path: str | Path) -> DiagnosisConfig:
    """加载 YAML，并相对 YAML 目录解析所有路径。"""
    config_path = Path(path).resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        config = DiagnosisConfig.model_validate(raw)
    except (OSError, ValidationError, yaml.YAMLError) as error:
        raise DiagnosisConfigError(f"诊断配置无效：{config_path}") from error
    base = config_path.parent
    return config.model_copy(
        update={
            "profile": (base / config.profile).resolve(),
            "task": (base / config.task).resolve(),
            "output_root": (base / config.output_root).resolve(),
            "collision_geometry_models": tuple(
                condition.model_copy(
                    update={
                        "model": (
                            condition.model
                            if condition.model.is_absolute()
                            else (base / condition.model).resolve()
                        )
                    }
                )
                for condition in config.collision_geometry_models
            ),
        }
    )


__all__ = [
    "ALL_PHASES",
    "CollisionGeometryCondition",
    "DiagnosisConfig",
    "DiagnosisConfigError",
    "Phase",
    "load_diagnosis_config",
]

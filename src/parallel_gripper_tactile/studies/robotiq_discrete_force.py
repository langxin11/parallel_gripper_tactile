"""Robotiq 离散力控制的控制器、刚度、目标与噪声实验矩阵。"""

from __future__ import annotations

from itertools import product
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, ValidationError, field_validator
import yaml

from ..experiments.robotiq_discrete_force import ControllerVariant
from ..scenes.robotiq import RobotiqObjectMaterial
from .force_tracking_ablation import SeedSweep, StudyConfigError


class _StudyModel(BaseModel):
    """拒绝未知字段的不可变 study 配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class RobotiqDiscreteForceStudyConfig(_StudyModel):
    """离散控制消融与鲁棒性验证的笛卡尔积配置。"""

    name: str = Field(default="robotiq_discrete_force", min_length=1)
    # 仅供旧的 protocol 直调入口使用；正式 Hydra 研究由 experiment 组合 profile。
    profile: Path = Field(default=Path("configs/robotiq_2f85.yaml"), exclude=True)
    task: Path
    controllers: tuple[ControllerVariant, ...]
    materials: tuple[RobotiqObjectMaterial, ...]
    noise_std_n: tuple[FiniteFloat, ...]
    seeds: SeedSweep = SeedSweep(start=0, count=1)
    # 仅供旧的 protocol 直调入口使用；正式研究目录由 execution 组唯一管理。
    output_root: Path = Field(default=Path("outputs/studies"), exclude=True)

    @field_validator("controllers", "materials", "noise_std_n")
    @classmethod
    def require_unique_nonempty(cls, value: tuple[object, ...]) -> tuple[object, ...]:
        """矩阵每个轴必须非空且没有重复条件。"""
        if not value:
            raise ValueError("matrix axes must not be empty")
        if len(set(value)) != len(value):
            raise ValueError("matrix axes must not contain duplicates")
        return value

    @field_validator("noise_std_n")
    @classmethod
    def require_nonnegative_noise(cls, value: tuple[FiniteFloat, ...]) -> tuple[FiniteFloat, ...]:
        """噪声标准差不能为负。"""
        if any(item < 0 for item in value):
            raise ValueError("noise standard deviations must be non-negative")
        return value

    def conditions(
        self,
    ) -> tuple[tuple[ControllerVariant, RobotiqObjectMaterial, float, int], ...]:
        """按配置顺序展开完整实验矩阵。"""
        return tuple(
            product(
                self.controllers,
                self.materials,
                (float(value) for value in self.noise_std_n),
                self.seeds.values(),
            )
        )


def load_robotiq_discrete_force_study_config(
    path: str | Path,
) -> RobotiqDiscreteForceStudyConfig:
    """加载 study，并以 YAML 所在目录解析输入与输出路径。"""
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
        config = RobotiqDiscreteForceStudyConfig.model_validate(raw)
    except ValidationError as error:
        raise StudyConfigError(str(error)) from error
    return config.model_copy(
        update={
            "profile": (
                None
                if config.profile is None
                else (base / config.profile).resolve()
                if not config.profile.is_absolute()
                else config.profile
            ),
            "task": (base / config.task).resolve()
            if not config.task.is_absolute()
            else config.task,
            "output_root": (base / config.output_root).resolve()
            if not config.output_root.is_absolute()
            else config.output_root,
        }
    )


__all__ = [
    "RobotiqDiscreteForceStudyConfig",
    "load_robotiq_discrete_force_study_config",
]

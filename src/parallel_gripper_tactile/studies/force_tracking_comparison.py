"""力跟踪控制器对比研究的显式配置模型。"""

from __future__ import annotations

from itertools import product
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator
import yaml

from ..experiments.force_tracking import ControllerVariant
from ..config.profiles import StiffnessEstimatorMethod
from ..scenes.custom import ObjectMaterial
from .force_tracking_ablation import SeedSweep, StudyConfigError


class _ComparisonStudyModel(BaseModel):
    """拒绝未知字段且不可变的控制器对比 study 配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class ForceTrackingComparisonConfig(_ComparisonStudyModel):
    """按控制器、任务、材料和噪声种子展开的力跟踪对比 protocol。"""

    name: str = Field(default="force_tracking_controller_comparison", min_length=1)
    # 仅供旧的 protocol 直调入口使用；正式 Hydra 研究由 experiment 组合 profile。
    profile: Path = Field(default=Path("configs/dm_gripper.yaml"), exclude=True)
    tasks: tuple[Path, ...]
    controllers: tuple[ControllerVariant, ...]
    stiffness_estimator_method: StiffnessEstimatorMethod = "window_linear"
    materials: tuple[ObjectMaterial, ...]
    seeds: SeedSweep = SeedSweep()
    trace_at_control_rate: bool = False
    # 仅供旧的 protocol 直调入口使用；正式研究目录由 execution 组唯一管理。
    output_root: Path = Field(default=Path("outputs/studies"), exclude=True)

    @field_validator("tasks", "controllers", "materials")
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

    def conditions(self) -> tuple[tuple[ControllerVariant, Path, ObjectMaterial, int], ...]:
        """按 controller × task × material × seed 的配置顺序展开条件矩阵。"""
        return tuple(product(self.controllers, self.tasks, self.materials, self.seeds.values()))


def _resolve_relative_path(path: Path, *, base: Path) -> Path:
    """将 YAML 中的相对路径解析为相对该 YAML 文件的绝对路径。"""
    return (base / path).resolve() if not path.is_absolute() else path


def load_comparison_config(path: str | Path) -> ForceTrackingComparisonConfig:
    """加载控制器对比 YAML，并将所有相对输入、输出路径相对 YAML 文件解析。"""
    config_path = Path(path).resolve()
    try:
        with config_path.open(encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except OSError as error:
        raise StudyConfigError(f"cannot read comparison config: {config_path}") from error
    except yaml.YAMLError as error:
        raise StudyConfigError(f"invalid YAML in {config_path}") from error
    if not isinstance(raw, dict):
        raise StudyConfigError(f"comparison config root must be a mapping: {config_path}")
    base = config_path.parent
    study = raw.get("study")
    if isinstance(study, dict) and isinstance(study.get("definition"), dict):
        raw = study["definition"]
        base = Path(__file__).resolve().parents[3]
    try:
        config = ForceTrackingComparisonConfig.model_validate(raw)
    except ValidationError as error:
        raise StudyConfigError(str(error)) from error

    return config.model_copy(
        update={
            "profile": (
                None
                if config.profile is None
                else _resolve_relative_path(config.profile, base=base)
            ),
            "tasks": tuple(_resolve_relative_path(task, base=base) for task in config.tasks),
            "output_root": _resolve_relative_path(config.output_root, base=base),
        }
    )


__all__ = [
    "ForceTrackingComparisonConfig",
    "SeedSweep",
    "StudyConfigError",
    "load_comparison_config",
]

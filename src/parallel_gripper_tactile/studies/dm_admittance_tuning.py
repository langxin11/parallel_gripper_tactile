"""DMgripper 二阶导纳 Ramp 调参研究的显式配置模型。"""

from __future__ import annotations

from itertools import product
import math
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator, model_validator
import yaml

from ..scenes.custom import ObjectMaterial
from .force_tracking_ablation import SeedSweep, StudyConfigError


class _TuningStudyModel(BaseModel):
    """拒绝未知字段且不可变的调参 study 配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class DMAdmittanceCandidate(_TuningStudyModel):
    """一组可复现的导纳动力学、接触切换和限幅参数。"""

    mass_kg: float = Field(gt=0)
    damping_ns_m: float = Field(ge=0)
    stiffness_n_m: float = Field(ge=0)
    filter_cutoff_hz: float = Field(gt=0)
    velocity_limit_rad_s: float = Field(gt=0)
    approach_velocity_rad_s: float = Field(gt=0)
    contact_stable_time_s: float = Field(ge=0)
    contact_transition_time_s: float = Field(gt=0)
    approach_feedforward_force_n: float = Field(ge=0)

    @model_validator(mode="after")
    def validate_finite_parameters(self) -> "DMAdmittanceCandidate":
        """拒绝非有限参数，避免生成不可复现实验 profile。"""
        values = (
            self.mass_kg,
            self.damping_ns_m,
            self.stiffness_n_m,
            self.filter_cutoff_hz,
            self.velocity_limit_rad_s,
            self.approach_velocity_rad_s,
            self.contact_stable_time_s,
            self.contact_transition_time_s,
            self.approach_feedforward_force_n,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("导纳候选参数必须为有限数值。")
        if self.approach_velocity_rad_s > self.velocity_limit_rad_s:
            raise ValueError("接近速度不得超过导纳速度上限。")
        return self

    @property
    def identifier(self) -> str:
        """返回稳定且适合文件路径的候选标识。"""
        return (
            f"m{self.mass_kg:.9g}-b{self.damping_ns_m:.9g}-"
            f"k{self.stiffness_n_m:.9g}-fc{self.filter_cutoff_hz:.9g}-"
            f"v{self.velocity_limit_rad_s:.9g}-av{self.approach_velocity_rad_s:.9g}-"
            f"cs{self.contact_stable_time_s:.9g}-ct{self.contact_transition_time_s:.9g}-"
            f"aff{self.approach_feedforward_force_n:.9g}"
        )


class DMAdmittanceTuningConfig(_TuningStudyModel):
    """固定 Ramp 任务下的 DMgripper 导纳参数调优协议。"""

    name: str = Field(default="dm_admittance_tuning", min_length=1)
    profile: Path
    task: Path
    materials: tuple[ObjectMaterial, ...]
    seeds: SeedSweep = SeedSweep()
    candidates: tuple[DMAdmittanceCandidate, ...]
    max_workers: int = Field(default=4, gt=0)
    output_root: Path = Path("outputs/studies")
    minimum_force_tracking_ratio: float = Field(default=0.995, gt=0, le=1)

    @field_validator("materials", "candidates")
    @classmethod
    def require_nonempty_unique(
        cls,
        value: tuple[ObjectMaterial, ...] | tuple[DMAdmittanceCandidate, ...],
    ) -> tuple[ObjectMaterial, ...] | tuple[DMAdmittanceCandidate, ...]:
        """拒绝空或重复的材料和候选条件。"""
        if not value:
            raise ValueError("调参条件不能为空。")
        if len(set(value)) != len(value):
            raise ValueError("调参条件不能包含重复项。")
        if value and isinstance(value[0], DMAdmittanceCandidate):
            identifiers = [candidate.identifier for candidate in value]
            if len(set(identifiers)) != len(identifiers):
                raise ValueError("候选标识不能重复。")
        return value

    def conditions(self) -> tuple[tuple[DMAdmittanceCandidate, ObjectMaterial, int], ...]:
        """按候选、材料和 seed 的配置顺序展开运行条件。"""
        return tuple(product(self.candidates, self.materials, self.seeds.values()))


def _resolve(path: Path, base: Path) -> Path:
    """把相对路径解析为相对于 study YAML 的绝对路径。"""
    return path if path.is_absolute() else (base / path).resolve()


def load_dm_admittance_tuning_config(path: str | Path) -> DMAdmittanceTuningConfig:
    """加载 DMgripper 导纳 Ramp 调参 YAML。"""
    config_path = Path(path).resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise StudyConfigError(f"无法读取导纳调参配置：{config_path}") from error
    except yaml.YAMLError as error:
        raise StudyConfigError(f"导纳调参 YAML 无效：{config_path}") from error
    if not isinstance(raw, dict):
        raise StudyConfigError(f"导纳调参配置根节点必须为映射：{config_path}")
    try:
        config = DMAdmittanceTuningConfig.model_validate(raw)
    except ValidationError as error:
        raise StudyConfigError(str(error)) from error
    base = config_path.parent
    return config.model_copy(
        update={
            "profile": _resolve(config.profile, base),
            "task": _resolve(config.task, base),
            "output_root": _resolve(config.output_root, base),
        }
    )


__all__ = [
    "DMAdmittanceCandidate",
    "DMAdmittanceTuningConfig",
    "SeedSweep",
    "StudyConfigError",
    "load_dm_admittance_tuning_config",
]

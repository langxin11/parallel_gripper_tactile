"""自适应撤支撑力调度验证的显式 study 配置。"""

from __future__ import annotations

from itertools import product
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator
import yaml

from .force_tracking_ablation import SeedSweep, StudyConfigError


class _StudyModel(BaseModel):
    """拒绝未知字段的不可变配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class SupportReleaseArm(_StudyModel):
    """一个对照臂：基座 experiment、组覆盖与任务／调度器结构补丁。"""

    name: str = Field(min_length=1)
    experiment: str
    overrides: tuple[str, ...] = ()
    # Hydra 点覆盖无法表达的嵌套结构（如 tactile_sampling、风险观测块）以补丁并入。
    task_patch: dict[str, object] = Field(default_factory=dict)
    scheduler_patch: dict[str, object] = Field(default_factory=dict)


class ForceSchedulingSupportReleaseConfig(_StudyModel):
    """对照臂 × 噪声种子组成的自适应撤支撑验证矩阵。"""

    name: str = Field(default="force_scheduling_support_release", min_length=1)
    # 仅供旧的 protocol 直调入口使用；正式 Hydra 研究由 experiment 组合 profile。
    profile: Path = Field(default=Path("configs/dm_gripper.yaml"), exclude=True)
    arms: tuple[SupportReleaseArm, ...]
    seeds: SeedSweep = SeedSweep()
    # 仅供旧的 protocol 直调入口使用；正式研究目录由 execution 组唯一管理。
    output_root: Path = Field(default=Path("outputs/studies"), exclude=True)

    @field_validator("arms")
    @classmethod
    def require_arms(cls, value: tuple[SupportReleaseArm, ...]) -> tuple[SupportReleaseArm, ...]:
        """要求对照臂非空且名称唯一。"""
        if not value:
            raise ValueError("arms must not be empty")
        names = [arm.name for arm in value]
        if len(set(names)) != len(names):
            raise ValueError("arm names must be unique")
        return value

    def conditions(self) -> tuple[tuple[SupportReleaseArm, int], ...]:
        """按配置顺序展开对照臂与 seed 矩阵。"""
        return tuple(product(self.arms, self.seeds.values()))


def load_force_scheduling_support_release_config(
    path: str | Path,
) -> ForceSchedulingSupportReleaseConfig:
    """加载配置并相对 YAML 所在目录解析路径。"""
    config_path = Path(path).resolve()
    try:
        with config_path.open(encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except OSError as error:
        raise StudyConfigError(f"cannot read study config: {config_path}") from error
    except yaml.YAMLError as error:
        raise StudyConfigError(f"invalid YAML in study config: {config_path}") from error
    if not isinstance(raw, dict):
        raise StudyConfigError(f"study config root must be a mapping: {config_path}")
    base = config_path.parent
    study = raw.get("study")
    if isinstance(study, dict) and isinstance(study.get("definition"), dict):
        raw = study["definition"]
        base = Path(__file__).resolve().parents[3]
    try:
        config = ForceSchedulingSupportReleaseConfig.model_validate(raw)
    except ValidationError as error:
        raise StudyConfigError(str(error)) from error
    return config.model_copy(
        update={
            "profile": (base / config.profile).resolve()
            if not config.profile.is_absolute()
            else config.profile,
            "output_root": (base / config.output_root).resolve()
            if not config.output_root.is_absolute()
            else config.output_root,
        }
    )


__all__ = [
    "ForceSchedulingSupportReleaseConfig",
    "SupportReleaseArm",
    "load_force_scheduling_support_release_config",
]

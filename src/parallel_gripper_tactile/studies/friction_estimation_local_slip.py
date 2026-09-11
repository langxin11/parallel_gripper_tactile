"""纯力局部起滑多种子验证的显式 study 配置。"""

from __future__ import annotations

from itertools import product
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator
import yaml

from .force_tracking_ablation import SeedSweep, StudyConfigError


class _StudyModel(BaseModel):
    """拒绝未知字段的不可变配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class LocalSlipScenario(_StudyModel):
    """一个局部起滑验证场景及其期望。"""

    task: Path
    expect_local_slip: bool


class FrictionEstimationLocalSlipStudyConfig(_StudyModel):
    """任务场景与噪声种子组成的局部起滑验证矩阵。"""

    name: str = Field(default="friction_estimation_local_slip", min_length=1)
    # 仅供旧的 protocol 直调入口使用；正式 Hydra 研究由 experiment 组合 profile。
    profile: Path = Field(default=Path("configs/dm_gripper.yaml"), exclude=True)
    scenarios: tuple[LocalSlipScenario, ...]
    seeds: SeedSweep = SeedSweep()
    # 仅供旧的 protocol 直调入口使用；正式研究目录由 execution 组唯一管理。
    output_root: Path = Field(default=Path("outputs/studies"), exclude=True)

    @field_validator("scenarios")
    @classmethod
    def require_scenarios(
        cls, value: tuple[LocalSlipScenario, ...]
    ) -> tuple[LocalSlipScenario, ...]:
        """要求场景非空且 task 路径不重复。"""
        if not value:
            raise ValueError("scenarios must not be empty")
        tasks = [scenario.task for scenario in value]
        if len(set(tasks)) != len(tasks):
            raise ValueError("scenario task paths must be unique")
        return value

    def conditions(self) -> tuple[tuple[LocalSlipScenario, int], ...]:
        """按配置顺序展开场景与 seed 矩阵。"""
        return tuple(product(self.scenarios, self.seeds.values()))


def load_local_slip_study_config(
    path: str | Path,
) -> FrictionEstimationLocalSlipStudyConfig:
    """加载配置并相对 YAML 所在目录解析路径。"""
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
        config = FrictionEstimationLocalSlipStudyConfig.model_validate(raw)
    except ValidationError as error:
        raise StudyConfigError(str(error)) from error
    scenarios = tuple(
        scenario.model_copy(
            update={
                "task": (base / scenario.task).resolve()
                if not scenario.task.is_absolute()
                else scenario.task
            }
        )
        for scenario in config.scenarios
    )
    return config.model_copy(
        update={
            "profile": (
                None
                if config.profile is None
                else (base / config.profile).resolve()
                if not config.profile.is_absolute()
                else config.profile
            ),
            "scenarios": scenarios,
            "output_root": (base / config.output_root).resolve()
            if not config.output_root.is_absolute()
            else config.output_root,
        }
    )


__all__ = [
    "FrictionEstimationLocalSlipStudyConfig",
    "LocalSlipScenario",
    "load_local_slip_study_config",
]

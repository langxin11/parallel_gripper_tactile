"""二阶直接力矩 ADRC 调参研究的显式配置模型。"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
import math
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator
import yaml

from ..config.profiles import StiffnessEstimatorMethod
from ..scenes.custom import ObjectMaterial
from .common import SeedSweep, StudyConfigError


TorqueAdrcTuningStageName = Literal["coarse", "confirm"]


class _TuningStudyModel(BaseModel):
    """拒绝未知字段且不可变的调参 study 配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


@dataclass(frozen=True, slots=True)
class TorqueAdrcCandidate:
    """一组可复现的二阶直接力矩 ADRC 调参候选。"""

    measurement_filter_cutoff_hz: float
    controller_bandwidth_rad_s: float
    observer_bandwidth_ratio: float

    @property
    def observer_bandwidth_rad_s(self) -> float:
        """返回由观测器带宽比例推导出的 ``ωo``。"""
        return self.controller_bandwidth_rad_s * self.observer_bandwidth_ratio

    @property
    def identifier(self) -> str:
        """返回适合目录和 CSV 的稳定候选标识。"""
        return (
            f"fc{self.measurement_filter_cutoff_hz:g}-"
            f"wc{self.controller_bandwidth_rad_s:g}-"
            f"ratio{self.observer_bandwidth_ratio:g}"
        )


class TorqueAdrcTuningStage(_TuningStudyModel):
    """一个调参阶段的候选空间与重复条件。"""

    materials: tuple[ObjectMaterial, ...]
    seeds: SeedSweep = SeedSweep()
    measurement_filter_cutoffs_hz: tuple[float, ...]
    controller_bandwidths_rad_s: tuple[float, ...]
    observer_bandwidth_ratios: tuple[float, ...]

    @field_validator(
        "materials",
        "measurement_filter_cutoffs_hz",
        "controller_bandwidths_rad_s",
        "observer_bandwidth_ratios",
    )
    @classmethod
    def require_nonempty_unique(
        cls, value: tuple[ObjectMaterial, ...] | tuple[float, ...]
    ) -> tuple[ObjectMaterial, ...] | tuple[float, ...]:
        """拒绝空条件维度、重复项和非正数值。"""
        if not value:
            raise ValueError("must contain at least one value")
        if len(set(value)) != len(value):
            raise ValueError("must not contain duplicate values")
        if value and isinstance(value[0], float) and any(float(item) <= 0 for item in value):
            raise ValueError("numeric tuning values must be positive")
        return value

    def candidates(self) -> tuple[TorqueAdrcCandidate, ...]:
        """展开满足测量带宽约束的候选组合。"""
        candidates = tuple(
            TorqueAdrcCandidate(filter_cutoff, controller_bandwidth, observer_ratio)
            for filter_cutoff, controller_bandwidth, observer_ratio in product(
                self.measurement_filter_cutoffs_hz,
                self.controller_bandwidths_rad_s,
                self.observer_bandwidth_ratios,
            )
            if filter_cutoff + 1e-12 >= controller_bandwidth * observer_ratio / (2.0 * math.pi)
        )
        if not candidates:
            raise ValueError("no candidate satisfies measurement filter bandwidth constraint")
        return candidates


class TorqueAdrcTuningConstraints(_TuningStudyModel):
    """候选可行性筛选约束。"""

    max_torque_saturation_ratio: float = Field(default=0.01, ge=0.0, le=1.0)
    max_ramp_rmse_ratio_to_baseline: float = Field(default=1.10, ge=1.0)
    max_mixed_rmse_ratio_to_baseline: float = Field(default=1.10, ge=1.0)


class ForceTrackingTorqueAdrcTuningConfig(_TuningStudyModel):
    """二阶段二阶直接力矩 ADRC 调参 protocol。"""

    name: str = Field(default="force_tracking_torque_adrc_tuning", min_length=1)
    # 仅供旧的 protocol 直调入口使用；正式 Hydra 研究由 experiment 组合 profile。
    profile: Path = Field(default=Path("configs/dm_gripper.yaml"), exclude=True)
    tasks: tuple[Path, ...]
    stiffness_estimator_method: StiffnessEstimatorMethod = "secant_ewma"
    # 仅供旧的 protocol 直调入口使用；正式研究目录由 execution 组唯一管理。
    output_root: Path = Field(default=Path("outputs/studies"), exclude=True)
    baseline: TorqueAdrcCandidate
    coarse: TorqueAdrcTuningStage
    confirm: TorqueAdrcTuningStage
    confirm_top_candidates: int = Field(default=5, ge=1)
    constraints: TorqueAdrcTuningConstraints = TorqueAdrcTuningConstraints()

    @field_validator("tasks")
    @classmethod
    def require_tasks(cls, value: tuple[Path, ...]) -> tuple[Path, ...]:
        """要求至少一个且不重复的目标任务。"""
        if not value:
            raise ValueError("tasks must not be empty")
        if len(set(value)) != len(value):
            raise ValueError("tasks must not contain duplicates")
        return value

    def stage_config(self, stage: TorqueAdrcTuningStageName) -> TorqueAdrcTuningStage:
        """返回阶段配置，并显式拒绝未知阶段。"""
        if stage == "coarse":
            return self.coarse
        if stage == "confirm":
            return self.confirm
        raise ValueError(f"unsupported torque ADRC tuning stage: {stage}")

    def stage_candidates(self, stage: TorqueAdrcTuningStageName) -> tuple[TorqueAdrcCandidate, ...]:
        """返回指定阶段的候选，并确保基线始终参与。"""
        source = self.stage_config(stage)
        candidates = source.candidates()
        return candidates if self.baseline in candidates else (self.baseline, *candidates)

    def conditions(
        self,
        stage: TorqueAdrcTuningStageName,
        *,
        candidates: tuple[TorqueAdrcCandidate, ...] | None = None,
    ) -> tuple[tuple[TorqueAdrcCandidate, Path, ObjectMaterial, int], ...]:
        """按候选→任务→材料→seed 展开阶段唯一有序条件。"""
        stage_config = self.stage_config(stage)
        selected = self.stage_candidates(stage) if candidates is None else candidates
        return tuple(
            (candidate, task, material, seed)
            for candidate, task, material, seed in product(
                selected,
                self.tasks,
                stage_config.materials,
                stage_config.seeds.values(),
            )
        )


def _resolve(path: Path, base: Path) -> Path:
    """把相对路径解析为相对于 study YAML 的绝对路径。"""
    return path if path.is_absolute() else (base / path).resolve()


def load_torque_adrc_tuning_config(path: str | Path) -> ForceTrackingTorqueAdrcTuningConfig:
    """加载二阶直接力矩 ADRC 调参 YAML。"""
    config_path = Path(path).resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise StudyConfigError(f"cannot read torque ADRC tuning config: {config_path}") from error
    except yaml.YAMLError as error:
        raise StudyConfigError(f"invalid YAML in {config_path}") from error
    if not isinstance(raw, dict):
        raise StudyConfigError(f"torque ADRC tuning config root must be a mapping: {config_path}")
    base = config_path.parent
    study = raw.get("study")
    if isinstance(study, dict) and isinstance(study.get("definition"), dict):
        raw = study["definition"]
        base = Path(__file__).resolve().parents[3]
    try:
        config = ForceTrackingTorqueAdrcTuningConfig.model_validate(raw)
    except ValidationError as error:
        raise StudyConfigError(str(error)) from error
    return config.model_copy(
        update={
            "profile": None if config.profile is None else _resolve(config.profile, base),
            "tasks": tuple(_resolve(task, base) for task in config.tasks),
            "output_root": _resolve(config.output_root, base),
        }
    )


__all__ = [
    "ForceTrackingTorqueAdrcTuningConfig",
    "TorqueAdrcCandidate",
    "TorqueAdrcTuningStageName",
    "TorqueAdrcTuningConstraints",
    "TorqueAdrcTuningStage",
    "load_torque_adrc_tuning_config",
]

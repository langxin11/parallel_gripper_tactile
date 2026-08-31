"""Run focused force-tracking diagnostics without routing through the CLI."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import UTC, datetime
import json
from pathlib import Path
from typing import Annotated, Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator
import yaml

from parallel_gripper_tactile.control import ForceSemantics
from parallel_gripper_tactile.experiments.force_tracking import (
    CONTROLLER_VARIANTS,
    ControllerVariant,
)
from parallel_gripper_tactile.runners import execute_force_tracking
from parallel_gripper_tactile.scenes.custom import ObjectContactModel, ObjectMaterial


Phase = Literal[
    "reproducibility",
    "controllers",
    "materials",
    "force-scale",
    "contact-model",
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
    "force-semantics",
    "position-limit",
    "integral-gain",
    "filter-cutoff",
)


class DiagnosisConfigError(ValueError):
    """Raised when the diagnostic study configuration is invalid."""


class DiagnosisConfig(BaseModel):
    """Small, explicit configuration for one force-tracking diagnosis protocol."""

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
        """Reject empty or duplicated diagnostic dimensions."""
        if not value:
            raise ValueError("must contain at least one value")
        if len(set(value)) != len(value):
            raise ValueError("must not contain duplicate values")
        return value


def load_config(path: Path) -> DiagnosisConfig:
    """Load YAML and resolve all relative paths from the YAML directory."""
    config_path = path.resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        config = DiagnosisConfig.model_validate(raw)
    except (OSError, ValidationError, yaml.YAMLError) as error:
        raise DiagnosisConfigError(f"invalid diagnosis config: {config_path}") from error
    base = config_path.parent
    return config.model_copy(
        update={
            "profile": (base / config.profile).resolve(),
            "task": (base / config.task).resolve(),
            "output_root": (base / config.output_root).resolve(),
        }
    )


def _study_directory(config: DiagnosisConfig, phase: Phase) -> Path:
    """Create an exclusive directory for one diagnostic phase."""
    identifier = f"{datetime.now(UTC):%Y%m%dT%H%M%SZ}-{uuid4().hex[:8]}"
    directory = config.output_root / config.name / phase / identifier
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def _scaled_task(source: Path, scale: float, task_dir: Path) -> Path:
    """Materialize a scaled task so every run snapshots its actual force reference."""
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise DiagnosisConfigError(f"force tracking task must be a mapping: {source}")
    approach = raw.get("approach")
    reference = raw.get("reference")
    if not isinstance(approach, dict) or not isinstance(reference, dict):
        raise DiagnosisConfigError(
            f"force tracking task lacks approach/reference mappings: {source}"
        )
    approach["feedforward_force_n"] = float(approach["feedforward_force_n"]) * scale
    waypoints = reference.get("waypoints")
    if not isinstance(waypoints, list):
        raise DiagnosisConfigError(f"force tracking task has no waypoint list: {source}")
    for waypoint in waypoints:
        if not isinstance(waypoint, dict) or "force_n" not in waypoint:
            raise DiagnosisConfigError(f"invalid force tracking waypoint in {source}")
        waypoint["force_n"] = float(waypoint["force_n"]) * scale
    task_dir.mkdir(parents=True, exist_ok=True)
    output = task_dir / f"force-scale-{scale:.3f}.yaml"
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return output


def _semantics_scaled_profile(source: Path, profile_dir: Path) -> Path:
    """Materialize the parameter conversion needed for an equivalent average-side task."""
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        force = raw["control"]["force"]
        stiffness = force["stiffness"]
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(f"profile lacks force/stiffness settings: {source}") from error
    for key in ("kp", "ki", "kd"):
        force[key] = float(force[key]) * 2.0
    for key in ("initial_n_per_m", "min_n_per_m", "max_n_per_m", "min_delta_force_n"):
        stiffness[key] = float(stiffness[key]) * 0.5
    try:
        model_path = Path(raw["model"]["path"])
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(f"profile lacks model path: {source}") from error
    raw["model"]["path"] = str(
        model_path if model_path.is_absolute() else (source.parent / model_path).resolve()
    )
    profile_dir.mkdir(parents=True, exist_ok=True)
    output = profile_dir / "average-side-scaled-parameters.yaml"
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return output


def _tuning_profile(
    source: Path, profile_dir: Path, label: str, override: tuple[str, float]
) -> Path:
    """Materialize one profile with exactly one force-loop parameter override."""
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        force = raw["control"]["force"]
        model_path = Path(raw["model"]["path"])
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(f"profile lacks force/model settings: {source}") from error
    key, value = override
    force[key] = value
    raw["model"]["path"] = str(
        model_path if model_path.is_absolute() else (source.parent / model_path).resolve()
    )
    profile_dir.mkdir(parents=True, exist_ok=True)
    output = profile_dir / f"{label}.yaml"
    output.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return output


def _force_parameters(profile_path: Path) -> dict[str, float]:
    """Read the force-control parameters recorded beside a diagnostic run."""
    raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    try:
        force = raw["control"]["force"]
        stiffness = force["stiffness"]
    except (KeyError, TypeError) as error:
        raise DiagnosisConfigError(
            f"profile lacks force/stiffness settings: {profile_path}"
        ) from error
    return {
        "pid_kp": float(force["kp"]),
        "pid_ki": float(force["ki"]),
        "pid_kd": float(force["kd"]),
        "stiffness_initial_n_per_m": float(stiffness["initial_n_per_m"]),
        "stiffness_min_n_per_m": float(stiffness["min_n_per_m"]),
        "stiffness_max_n_per_m": float(stiffness["max_n_per_m"]),
        "min_delta_force_n": float(stiffness["min_delta_force_n"]),
        "max_position_adjustment_rad": float(force["max_position_adjustment"]),
        "filter_cutoff_hz": float(force["filter_cutoff_hz"]),
    }


def _task_targets(task_path: Path, force_semantics: ForceSemantics) -> dict[str, float]:
    """Return final target force in both the selected and physical force conventions."""
    raw = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    try:
        target = float(raw["reference"]["waypoints"][-1]["force_n"])
    except (KeyError, IndexError, TypeError) as error:
        raise DiagnosisConfigError(f"task lacks reference waypoints: {task_path}") from error
    side = target if force_semantics == "average_side" else 0.5 * target
    return {
        "target_force_n": target,
        "equivalent_side_force_n": side,
        "equivalent_total_force_n": 2.0 * side,
    }


def _contact_diagnostics(trace_path: Path) -> dict[str, object]:
    """Summarize large taxel-contact collapses and raw single-side-force swings."""
    with trace_path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["phase"] == "track_reference"]
    if not rows:
        return {"max_active_taxel_contacts": 0, "contact_collapse_events": 0}
    contacts = [int(row["active_taxel_contacts"]) for row in rows]
    forces = [
        0.5 * (float(row["left_taxel_normal_force_n"]) + float(row["right_taxel_normal_force_n"]))
        for row in rows
    ]
    maximum = max(contacts)
    collapse_threshold = 0.5 * maximum
    events = sum(
        previous >= collapse_threshold and current < collapse_threshold
        for previous, current in zip(contacts, contacts[1:])
    )

    def peak_to_peak(column: str) -> float:
        values = [float(row[column]) for row in rows]
        return max(values) - min(values)

    left_forces = [float(row["left_taxel_normal_force_n"]) for row in rows]
    right_forces = [float(row["right_taxel_normal_force_n"]) for row in rows]
    return {
        "max_active_taxel_contacts": maximum,
        "contact_collapse_events": events,
        "mean_left_force_n": sum(left_forces) / len(left_forces),
        "mean_right_force_n": sum(right_forces) / len(right_forces),
        "mean_average_side_force_n": sum(forces) / len(forces),
        "mean_total_force_n": 2.0 * sum(forces) / len(forces),
        "pid_position_adjustment_peak_to_peak_rad": peak_to_peak("pid_position_adjustment_rad"),
        "stiffness_position_adjustment_peak_to_peak_rad": peak_to_peak(
            "stiffness_position_adjustment_rad"
        ),
        "force_feedforward_torque_peak_to_peak_nm": peak_to_peak("force_feedforward_torque_n_m"),
        "motor_torque_peak_to_peak_nm": peak_to_peak("motor_torque_n_m"),
    }


def _conditions(
    config: DiagnosisConfig, phase: Phase
) -> tuple[
    tuple[
        str,
        ControllerVariant,
        ObjectMaterial,
        float,
        ObjectContactModel,
        ForceSemantics,
        bool,
        tuple[str, float] | None,
    ],
    ...,
]:
    """Return the deliberately one-factor-at-a-time conditions for one phase."""
    if phase == "reproducibility":
        return tuple(
            (f"repeat-{index:02d}", "full", "hard", 1.0, "explicit", "average_side", False, None)
            for index in range(config.reproducibility_repeats)
        )
    if phase == "controllers":
        return tuple(
            (variant, variant, "hard", 1.0, "explicit", "average_side", False, None)
            for variant in config.controllers
        )
    if phase == "materials":
        return tuple(
            (material, "full", material, 1.0, "explicit", "average_side", False, None)
            for material in config.materials
        )
    if phase == "force-scale":
        return tuple(
            (f"scale-{scale:.3f}", "full", "hard", scale, "explicit", "average_side", False, None)
            for scale in config.force_scales
        )
    if phase == "contact-model":
        return tuple(
            (model, "full", "hard", 0.5, model, "average_side", False, None)
            for model in config.contact_models
        )
    if phase == "force-semantics":
        return (
            ("legacy-sum-8N", "full", "hard", 1.0, "legacy", "total", False, None),
            ("average-4N-unscaled", "full", "hard", 0.5, "legacy", "average_side", False, None),
            ("average-4N-scaled", "full", "hard", 0.5, "legacy", "average_side", True, None),
            ("average-8N-current", "full", "hard", 1.0, "explicit", "average_side", False, None),
        )
    if phase == "position-limit":
        return tuple(
            (
                f"limit-{value:.3f}",
                config.stability_controller,
                "hard",
                1.0,
                "explicit",
                "average_side",
                False,
                ("max_position_adjustment", value),
            )
            for value in config.position_adjustment_limits_rad
        )
    if phase == "integral-gain":
        return tuple(
            (
                f"ki-{value:.3f}",
                config.stability_controller,
                "hard",
                1.0,
                "explicit",
                "average_side",
                False,
                ("ki", value),
            )
            for value in config.integral_gains
        )
    return tuple(
        (
            f"cutoff-{value:.0f}",
            config.stability_controller,
            "hard",
            1.0,
            "explicit",
            "average_side",
            False,
            ("filter_cutoff_hz", value),
        )
        for value in config.filter_cutoffs_hz
    )


def run_phase(config: DiagnosisConfig, phase: Phase, *, config_source: Path) -> Path:
    """Run one diagnosis phase and return its study directory."""
    study_dir = _study_directory(config, phase)
    (study_dir / "study.yaml").write_bytes(config_source.read_bytes())
    rows: list[dict[str, object]] = []
    for (
        label,
        controller,
        material,
        scale,
        contact_model,
        force_semantics,
        scale_parameters,
        tuning_override,
    ) in _conditions(config, phase):
        task_path = _scaled_task(config.task, scale, study_dir / "tasks")
        profile_path = (
            _semantics_scaled_profile(config.profile, study_dir / "profiles")
            if scale_parameters
            else config.profile
        )
        if tuning_override is not None:
            profile_path = _tuning_profile(
                config.profile, study_dir / "profiles", label, tuning_override
            )
        targets = _task_targets(task_path, force_semantics)
        run, result = execute_force_tracking(
            profile=profile_path,
            task_path=task_path,
            output_root=study_dir / "runs",
            run_prefix=label,
            object_material=material,
            object_contact_model=contact_model,
            force_semantics=force_semantics,
            controller_variant=controller,
            sensor_noise_seed=config.sensor_noise_seed,
        )
        rows.append(
            {
                "label": label,
                "controller_variant": controller,
                "object_material": material,
                "object_contact_model": contact_model,
                "force_semantics": force_semantics,
                "force_scale": scale,
                "parameter_scale_conversion": scale_parameters,
                "swept_parameter": None if tuning_override is None else tuning_override[0],
                "swept_value": None if tuning_override is None else tuning_override[1],
                "sensor_noise_seed": config.sensor_noise_seed,
                "run_directory": str(run.path.relative_to(study_dir)),
                **targets,
                **_force_parameters(profile_path),
                **asdict(result),
                **_contact_diagnostics(run.path / "trace.csv"),
            }
        )
    with (study_dir / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (study_dir / "summary.json").write_text(
        json.dumps({"phase": phase, "runs": rows}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return study_dir


def main() -> None:
    """Parse a diagnosis configuration and execute one selected phase."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="Diagnosis study YAML path")
    parser.add_argument("--phase", choices=(*ALL_PHASES, "all"), required=True)
    arguments = parser.parse_args()
    config_path = arguments.config.resolve()
    config = load_config(config_path)
    phases: tuple[Phase, ...] = ALL_PHASES if arguments.phase == "all" else (arguments.phase,)
    for phase in phases:
        print(f"{phase}: {run_phase(config, phase, config_source=config_path)}")


if __name__ == "__main__":
    main()

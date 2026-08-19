"""Compile-time validation shared by CI and command-line workflows."""

from __future__ import annotations

from dataclasses import dataclass

from .profiles import GripperProfile


@dataclass(frozen=True, slots=True)
class ValidationReport:
    model_name: str
    actuator: str
    tactile_channels: int
    equalities: int


def validate_profile(profile: GripperProfile) -> ValidationReport:
    """Compile a profile's MJCF and verify its actuator and tactile contract."""
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("install project dependencies with `uv sync`") from error

    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, profile.actuator)
    if actuator_id < 0:
        raise ValueError(f"model is missing actuator {profile.actuator!r}")

    object_type = (
        mujoco.mjtObj.mjOBJ_GEOM
        if profile.tactile.mode == "contact_geom"
        else mujoco.mjtObj.mjOBJ_SENSOR
    )
    names = profile.tactile.names("left") + profile.tactile.names("right")
    missing = [name for name in names if mujoco.mj_name2id(model, object_type, name) < 0]
    if missing:
        preview = ", ".join(missing[:4])
        raise ValueError(f"model is missing {len(missing)} tactile channels: {preview}")

    return ValidationReport(
        model_name=profile.name,
        actuator=profile.actuator,
        tactile_channels=len(names),
        equalities=model.neq,
    )

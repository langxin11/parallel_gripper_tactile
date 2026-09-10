"""由 CI 与命令行工作流共享的编译期校验。"""

from __future__ import annotations

from dataclasses import dataclass

from .config.profiles import GripperProfile


@dataclass(frozen=True, slots=True)
class ValidationReport:
    """针对其编译后 MJCF 模型进行校验的 profile 摘要。

    Attributes:
        model_name: 来自配置文件的 profile 名称。
        actuator: 编译后模型中预期的执行器名称。
        tactile_channels: 触觉通道总数（左侧加右侧）。
        equalities: 编译后模型中等式约束的数量。
    """

    model_name: str
    actuator: str
    tactile_channels: int
    equalities: int


def validate_profile(profile: GripperProfile) -> ValidationReport:
    """编译 profile 的 MJCF 并验证其执行器与触觉约定。"""
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("install project dependencies with `uv sync`") from error

    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, profile.actuator)
    if actuator_id < 0:
        raise ValueError(f"model is missing actuator {profile.actuator!r}")
    if profile.control_mode == "mit_torque":
        if profile.mit is None:
            raise ValueError("MIT torque profile is missing control parameters")
        ctrl_limit = min(abs(float(value)) for value in model.actuator_ctrlrange[actuator_id])
        force_limit = min(abs(float(value)) for value in model.actuator_forcerange[actuator_id])
        if profile.mit.t_max > min(ctrl_limit, force_limit):
            raise ValueError("MIT t_max exceeds the compiled actuator limits")
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, profile.actuator)
        if joint_id < 0:
            raise ValueError(f"model is missing MIT-controlled joint {profile.actuator!r}")
        joint_min, joint_max = (float(value) for value in model.jnt_range[joint_id])
        if profile.mit.p_min < joint_min or profile.mit.p_max > joint_max:
            raise ValueError("MIT position range exceeds the controlled joint range")

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

    if profile.tactile.mode == "contact_geom":
        site_names = [name.replace("_geom_", "_", 1) for name in names]
        missing_sites = [
            site_name
            for site_name in site_names
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name) < 0
        ]
        if missing_sites:
            preview = ", ".join(missing_sites[:4])
            raise ValueError(f"model is missing {len(missing_sites)} tactile sites: {preview}")

    return ValidationReport(
        model_name=profile.name,
        actuator=profile.actuator,
        tactile_channels=len(names),
        equalities=model.neq,
    )

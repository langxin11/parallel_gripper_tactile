"""验证无支撑保持与切向扰动抵抗能力。

实验与 ``compare_tactile_models.py`` 使用相同的时序和默认 5 N、2 Hz 世界 Y 向
扰动。自研夹爪的 ``base`` 在其 profile 的 mount 位姿处固定，代表未来的转接
法兰，而不修改 CAD 基础本体。
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

from ..scenes.custom import (
    CUBE_PREFIX,
    DEFAULT_CUBE_HALF_CONTACT_SIDE,
    DEFAULT_CUBE_HALF_THICKNESS,
    DEFAULT_CUBE_MASS,
    DEFAULT_PROFILE,
    GRIPPER_PREFIX,
    ObjectMaterial,
    SUPPORT_GEOM_NAME,
    build_custom_grasp_model,
)
from ..contact_taxels import ContactTaxelReader
from ..control import NormalForceController
from ..plotstyle import paper_figsize, save_publication_figure, science_pyplot
from ..profiles import load_profile
from ..protocols import DisturbanceProtocol
from ..timing import SimulationTimer

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT_CSV = (
    REPOSITORY_ROOT / "outputs" / "custom_gripper" / "validation" / "custom_gripper_grasp.csv"
)
CUBE_BODY_NAME = f"{CUBE_PREFIX}target_cube"
CUBE_JOINT_NAME = f"{CUBE_PREFIX}target_cube_free_joint"


@dataclass(frozen=True, slots=True)
class GraspAcceptance:
    """两次无支撑抓取验收检查的结果。"""

    hold_displacement_m: float
    disturbance_displacement_m: float
    disturbance_speed_m_s: float
    minimum_friction_margin_n: float
    peak_friction_utilization: float
    force_tracking_rmse_n: float
    force_tracking_mean_n: float
    hold_passed: bool
    disturbance_passed: bool
    force_tracking_passed: bool
    simulation_stable: bool

    @property
    def passed(self) -> bool:
        """是否同时满足保持、扰动与仿真稳定性三项检查。"""
        return (
            self.hold_passed
            and self.disturbance_passed
            and self.force_tracking_passed
            and self.simulation_stable
        )


@dataclass(frozen=True, slots=True)
class FrictionCapacity:
    """方块—Pillar 接触可提供的法向力与切向摩擦容量。"""

    normal_force_n: float
    left_normal_force_n: float
    right_normal_force_n: float
    available_friction_n: float
    active_contacts: int


@dataclass(frozen=True, slots=True)
class TactileMeasurement:
    """带测量噪声的左右 taxel 三轴力。"""

    left: np.ndarray
    right: np.ndarray

    @property
    def left_force(self) -> np.ndarray:
        """返回左指尖带噪总三轴力。"""
        return self.left.sum(axis=(1, 2))

    @property
    def right_force(self) -> np.ndarray:
        """返回右指尖带噪总三轴力。"""
        return self.right.sum(axis=(1, 2))

    @property
    def normal_capacity(self) -> FrictionCapacity:
        """返回带噪左右法向力汇总；控制器内部使用平均单侧力。"""
        left_normal = max(0.0, float(self.left_force[2]))
        right_normal = max(0.0, float(self.right_force[2]))
        return FrictionCapacity(
            normal_force_n=left_normal + right_normal,
            left_normal_force_n=left_normal,
            right_normal_force_n=right_normal,
            available_friction_n=0.0,
            active_contacts=0,
        )


def _prefixed_reader(model, profile) -> ContactTaxelReader:
    """创建读取 ``MjSpec.attach`` 前缀命名通道的接触读取器。"""
    tactile = profile.tactile.model_copy(
        update={
            "left_prefix": f"{GRIPPER_PREFIX}{profile.tactile.left_prefix}",
            "right_prefix": f"{GRIPPER_PREFIX}{profile.tactile.right_prefix}",
        }
    )
    return ContactTaxelReader(model, tactile)


def _tangential_displacement(position: np.ndarray, reference: np.ndarray) -> float:
    """度量模型定义的 YZ 指尖接触平面内的位移。"""
    return float(np.linalg.norm(position[1:3] - reference[1:3]))


def _taxel_geom_sides(model, profile) -> dict[int, str]:
    """解析组合模型中左右两侧全部 Pillar 的 geom ID 与所属侧。"""
    return {
        model.geom(f"{GRIPPER_PREFIX}{name}").id: side
        for side in ("left", "right")
        for name in profile.tactile.names(side)
    }


def _friction_capacity(
    model,
    data,
    *,
    taxel_geom_sides: dict[int, str],
    cube_geom_id: int,
) -> FrictionCapacity:
    """返回方块—Pillar 接触的法向力、摩擦容量与活跃接触数。

    每一项摩擦容量按接触对的滑动摩擦系数乘以法向接触力计算。该和是可抵抗
    任意切向合力的上界；实际可用力还会受接触位置、力矩平衡和接触脱离影响。
    """
    contact_force = np.empty(6, dtype=np.float64)
    total_normal_force = 0.0
    normal_force_by_side = {"left": 0.0, "right": 0.0}
    total_friction_capacity = 0.0
    active_contacts = 0
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        if cube_geom_id not in {contact.geom1, contact.geom2}:
            continue
        taxel_geom_id = next(
            (geom_id for geom_id in (contact.geom1, contact.geom2) if geom_id in taxel_geom_sides),
            None,
        )
        if taxel_geom_id is None:
            continue
        mujoco.mj_contactForce(model, data, contact_id, contact_force)
        normal_force = max(0.0, float(contact_force[0]))
        total_normal_force += normal_force
        normal_force_by_side[taxel_geom_sides[taxel_geom_id]] += normal_force
        total_friction_capacity += float(contact.friction[0]) * normal_force
        active_contacts += 1
    return FrictionCapacity(
        normal_force_n=total_normal_force,
        left_normal_force_n=normal_force_by_side["left"],
        right_normal_force_n=normal_force_by_side["right"],
        available_friction_n=total_friction_capacity,
        active_contacts=active_contacts,
    )


def _tactile_measurement(
    tactile,
    *,
    left_normal_std_n: float,
    right_normal_std_n: float,
    left_shear_std_n: float,
    right_shear_std_n: float,
    rng: np.random.Generator,
) -> TactileMeasurement:
    """返回逐 taxel 加性高斯白噪声测量。"""
    left_std = np.array([left_shear_std_n, left_shear_std_n, left_normal_std_n])[:, None, None]
    right_std = np.array([right_shear_std_n, right_shear_std_n, right_normal_std_n])[:, None, None]
    left = tactile.left + rng.normal(0.0, left_std, size=tactile.left.shape)
    right = tactile.right + rng.normal(0.0, right_std, size=tactile.right.shape)
    return TactileMeasurement(left=left, right=right)


def plot_trace(path: Path, rows: list[dict[str, float | str]]) -> None:
    """绘制发表风格的控制量、触觉力与物体运动轨迹图。"""
    if not rows:
        raise ValueError("cannot plot an empty grasp trace")
    plt = science_pyplot()
    times = [float(row["time_s"]) for row in rows]
    control = [float(row["control"]) for row in rows]
    drive_position = [float(row["drive_position_rad"]) for row in rows]
    applied_force = [float(row["applied_world_fy"]) for row in rows]
    left_shear = [
        math.hypot(float(row["measured_left_fx"]), float(row["measured_left_fy"])) for row in rows
    ]
    right_shear = [
        math.hypot(float(row["measured_right_fx"]), float(row["measured_right_fy"])) for row in rows
    ]
    friction_capacity = [float(row["available_friction_n"]) for row in rows]
    static_hold_demand = [float(row["static_hold_demand_tangential_n"]) for row in rows]
    friction_margin = [float(row["friction_margin_n"]) for row in rows]
    measured_normal_force = [float(row["measured_normal_force_n"]) for row in rows]
    filtered_normal_force = [float(row["filtered_normal_force_n"]) for row in rows]
    target_normal_force = [float(row["target_normal_force_n"]) for row in rows]
    left_normal_force = [float(row["left_taxel_normal_force_n"]) for row in rows]
    right_normal_force = [float(row["right_taxel_normal_force_n"]) for row in rows]
    mean_side_normal_force = [
        0.5 * (left_force + right_force)
        for left_force, right_force in zip(left_normal_force, right_normal_force)
    ]
    initial_position = np.array([float(rows[0]["cube_y"]), float(rows[0]["cube_z"])])
    displacement = [
        1000.0
        * float(
            np.linalg.norm(
                np.array([float(row["cube_y"]), float(row["cube_z"])]) - initial_position
            )
        )
        for row in rows
    ]
    tangential_speed = [
        1000.0 * math.hypot(float(row["cube_vy"]), float(row["cube_vz"])) for row in rows
    ]

    colors = {"black": "#000000", "blue": "#0072B2", "orange": "#D55E00", "green": "#009E73"}
    figure, axes = plt.subplots(
        7, 1, figsize=paper_figsize(12.4), sharex=True, layout="constrained"
    )
    (
        control_axis,
        force_axis,
        tactile_axis,
        normal_axis,
        friction_axis,
        displacement_axis,
        speed_axis,
    ) = axes
    control_axis.plot(times, control, color=colors["black"], linewidth=1.3, label="drive target")
    control_axis.plot(
        times,
        drive_position,
        color=colors["blue"],
        linewidth=1.1,
        linestyle="--",
        label="drive position",
    )
    control_axis.set_ylabel("Drive target\n(rad)")
    force_axis.plot(
        times, applied_force, color=colors["orange"], linewidth=1.4, label=r"Applied $F_y^W$"
    )
    force_axis.set_ylabel("Force\n(N)")
    tactile_axis.plot(
        times, left_shear, color=colors["blue"], linewidth=1.2, label="Left Pillar shear"
    )
    tactile_axis.plot(
        times,
        right_shear,
        color=colors["orange"],
        linewidth=1.2,
        linestyle="--",
        label="Right Pillar shear",
    )
    tactile_axis.set_ylabel("Local shear\n(N)")
    normal_axis.plot(
        times,
        mean_side_normal_force,
        color=colors["black"],
        linewidth=1.3,
        label="Mean side normal pressure",
    )
    normal_axis.plot(
        times,
        filtered_normal_force,
        color=colors["green"],
        linewidth=1.0,
        linestyle=":",
        label="Filtered normal pressure",
    )
    normal_axis.plot(
        times,
        measured_normal_force,
        color="0.55",
        linewidth=0.7,
        alpha=0.8,
        label="Measured normal pressure",
    )
    normal_axis.plot(
        times,
        target_normal_force,
        color="0.35",
        linewidth=0.9,
        linestyle="-.",
        label="Force target",
    )
    normal_axis.plot(
        times,
        left_normal_force,
        color=colors["blue"],
        linewidth=1.1,
        label="Left Pillar normal",
    )
    normal_axis.plot(
        times,
        right_normal_force,
        color=colors["orange"],
        linewidth=1.1,
        linestyle="--",
        label="Right Pillar normal",
    )
    normal_axis.set_ylabel("Normal\npressure (N)")
    friction_axis.plot(
        times,
        friction_capacity,
        color=colors["blue"],
        linewidth=1.2,
        label=r"Available $\sum \mu F_n$",
    )
    friction_axis.plot(
        times,
        static_hold_demand,
        color=colors["orange"],
        linewidth=1.2,
        label="Static hold demand",
    )
    friction_axis.plot(
        times,
        friction_margin,
        color=colors["green"],
        linewidth=1.1,
        linestyle="--",
        label="Friction margin",
    )
    friction_axis.axhline(0.0, color="0.45", linewidth=0.7, zorder=0)
    friction_axis.set_ylabel("Friction\n(N)")
    displacement_axis.plot(
        times, displacement, color=colors["blue"], linewidth=1.3, label="YZ displacement"
    )
    displacement_axis.set_ylabel("YZ displacement\n(mm)")
    speed_axis.plot(times, tangential_speed, color=colors["green"], linewidth=1.3, label="YZ speed")
    speed_axis.set_ylabel("YZ speed\n(mm s$^{-1}$)")
    speed_axis.set_xlabel("Simulation time (s)")
    if max(displacement, default=0.0) > 10.0:
        displacement_axis.set_yscale("symlog", linthresh=1.0)
    if max(tangential_speed, default=0.0) > 100.0:
        speed_axis.set_yscale("symlog", linthresh=1.0)
    for axis in axes:
        axis.legend(frameon=False, loc="upper left")
        axis.tick_params(direction="in", which="both", top=True, right=True)
    phases = [str(row["phase"]) for row in rows]
    transitions = [index for index in range(1, len(phases)) if phases[index] != phases[index - 1]]
    phase_labels = {
        "close": "Close",
        "support_settle": "Support settle",
        "unsupported_hold": "Unsupported hold",
        "disturbance": "Disturbance",
        "recovery": "Recovery",
    }
    for axis in axes:
        for index in transitions:
            axis.axvline(times[index], color="0.55", linewidth=0.6, linestyle=":", zorder=0)
    boundaries = [0, *transitions, len(rows)]
    for start, end in zip(boundaries, boundaries[1:]):
        phase_start = times[start]
        phase_end = times[end - 1]
        for axis in axes:
            axis.axvspan(phase_start, phase_end, color="0.5", alpha=0.045, linewidth=0, zorder=-1)
        control_axis.text(
            0.5 * (phase_start + phase_end),
            0.93,
            phase_labels[phases[start]],
            ha="center",
            va="top",
            fontsize=9,
            color="0.35",
            transform=control_axis.get_xaxis_transform(),
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    save_publication_figure(figure, path)
    plt.close(figure)


def run_acceptance(
    profile_path: Path = DEFAULT_PROFILE,
    *,
    protocol: DisturbanceProtocol = DisturbanceProtocol(),
    hold_threshold_m: float = 0.002,
    slip_threshold_m: float = 0.002,
    cube_half_thickness: float = DEFAULT_CUBE_HALF_THICKNESS,
    cube_half_contact_side: float = DEFAULT_CUBE_HALF_CONTACT_SIDE,
    cube_mass: float = DEFAULT_CUBE_MASS,
    object_material: ObjectMaterial = "hard",
    target_force_n: float | None = None,
    force_rmse_threshold_n: float = 0.5,
    control_period_s: float = 0.002,
    output_csv: Path | None = None,
    output_plot: Path | None = None,
) -> GraspAcceptance:
    """无 viewer 运行固定基座保持与扰动检查。

    Args:
        profile_path: 夹爪 profile 路径。
        protocol: 实验时序与扰动波形。
        hold_threshold_m: 无支撑保持阶段允许的最大 YZ 切向位移。
        slip_threshold_m: 扰动阶段判定滑移的最大 YZ 切向位移。
        cube_half_thickness: 测试块 X 方向半厚。
        cube_half_contact_side: 测试块 YZ 接触面半边长。
        cube_mass: 测试块质量。
        object_material: 触觉—物体显式接触材料档位。
        target_force_n: 可选目标平均单侧法向力；省略时使用 profile 配置。
        force_rmse_threshold_n: 接触切换后稳态法向力 RMSE 验收阈值。
        control_period_s: 法向力控制器的仿真时间更新周期（s）。
        output_csv: 可选逐步轨迹 CSV 输出路径。
        output_plot: 可选轨迹图输出路径。

    Returns:
        两项检查与仿真稳定性的验收结果。
    """
    if (
        hold_threshold_m <= 0
        or slip_threshold_m <= 0
        or force_rmse_threshold_n <= 0
        or control_period_s <= 0
    ):
        raise ValueError("位移与法向力 RMSE 阈值必须为正数。")
    if target_force_n is not None and target_force_n <= 0:
        raise ValueError("target_force_n 必须为正数。")
    profile = load_profile(profile_path)
    if target_force_n is not None:
        if profile.normal_force is None:
            raise ValueError("profile 未配置 control.force。")
        profile = profile.model_copy(
            update={
                "control": profile.control.model_copy(
                    update={
                        "force": profile.normal_force.model_copy(
                            update={"target_n": target_force_n}
                        )
                    }
                )
            }
        )
    model = build_custom_grasp_model(
        profile,
        cube_half_thickness=cube_half_thickness,
        cube_half_contact_side=cube_half_contact_side,
        cube_mass=cube_mass,
        object_material=object_material,
    )
    if control_period_s + 1e-12 < float(model.opt.timestep):
        raise ValueError("control_period_s must not be smaller than the physics timestep")
    data = mujoco.MjData(model)
    control_timer = SimulationTimer(control_period_s, float(data.time))
    reader = _prefixed_reader(model, profile)
    controller = NormalForceController.from_profile(model, profile, name_prefix=GRIPPER_PREFIX)
    force_config = profile.normal_force
    noise_rng = np.random.default_rng(
        0 if force_config is None else int(force_config.sensor_noise_seed)
    )
    actuator_id = controller.actuator_id
    support_id = model.geom(SUPPORT_GEOM_NAME).id
    cube_body_id = model.body(CUBE_BODY_NAME).id
    cube_joint_id = model.joint(CUBE_JOINT_NAME).id
    cube_geom_id = model.geom(f"{CUBE_PREFIX}target_cube_geom").id
    cube_dof = model.jnt_dofadr[cube_joint_id]
    taxel_geom_sides = _taxel_geom_sides(model, profile)
    cube_mass_in_kg = float(model.body_mass[cube_body_id])
    gravity_tangent = cube_mass_in_kg * np.asarray(model.opt.gravity[1:3], dtype=np.float64)

    hold_reference: np.ndarray | None = None
    disturbance_reference: np.ndarray | None = None
    hold_displacement = 0.0
    disturbance_displacement = 0.0
    disturbance_speed = 0.0
    minimum_friction_margin = math.inf
    peak_friction_utilization = 0.0
    simulation_stable = True
    rows: list[dict[str, float | str]] = []
    force_command = None
    steps = math.ceil(protocol.total_duration / model.opt.timestep)
    for _ in range(steps):
        time_s = float(data.time)
        phase = protocol.phase_at(time_s)
        protocol.step(
            model,
            data,
            actuator_id=actuator_id,
            close_control=profile.closed_control,
            support_geom_id=support_id,
            cube_body_id=cube_body_id,
            apply_actuator_control=False,
        )
        target_position = protocol.close_target_at(
            time_s, profile.open_control, profile.closed_control
        )
        control_dt = control_timer.pop_due(time_s)
        if control_dt is not None:
            feedback_tactile = reader.read(data)
            if force_config is not None:
                feedback_measurement = _tactile_measurement(
                    feedback_tactile,
                    left_normal_std_n=float(force_config.sensor_taxel_normal_noise_std_n[0]),
                    right_normal_std_n=float(force_config.sensor_taxel_normal_noise_std_n[1]),
                    left_shear_std_n=float(force_config.sensor_taxel_shear_noise_std_n[0]),
                    right_shear_std_n=float(force_config.sensor_taxel_shear_noise_std_n[1]),
                    rng=noise_rng,
                )
            else:
                feedback_measurement = TactileMeasurement(
                    left=feedback_tactile.left, right=feedback_tactile.right
                )
            feedback_capacity = feedback_measurement.normal_capacity
            force_command = controller.apply(
                data,
                approach_position=target_position,
                total_normal_force_n=feedback_capacity.normal_force_n,
                left_normal_force_n=feedback_capacity.left_normal_force_n,
                right_normal_force_n=feedback_capacity.right_normal_force_n,
                dt=control_dt,
            )
        if force_command is None:
            raise RuntimeError("control timer did not produce an initial command")
        motor_command = force_command.mit
        mujoco.mj_step(model, data)
        if (
            data.time <= time_s
            or not np.isfinite(data.qpos).all()
            or not np.isfinite(data.qvel).all()
        ):
            simulation_stable = False
            break

        position = data.xpos[cube_body_id].copy()
        velocity = data.qvel[cube_dof : cube_dof + 3]
        tactile = reader.read(data)
        if force_config is not None:
            tactile_measurement = _tactile_measurement(
                tactile,
                left_normal_std_n=float(force_config.sensor_taxel_normal_noise_std_n[0]),
                right_normal_std_n=float(force_config.sensor_taxel_normal_noise_std_n[1]),
                left_shear_std_n=float(force_config.sensor_taxel_shear_noise_std_n[0]),
                right_shear_std_n=float(force_config.sensor_taxel_shear_noise_std_n[1]),
                rng=noise_rng,
            )
        else:
            tactile_measurement = TactileMeasurement(left=tactile.left, right=tactile.right)
        capacity = _friction_capacity(
            model,
            data,
            taxel_geom_sides=taxel_geom_sides,
            cube_geom_id=cube_geom_id,
        )
        applied_tangent = np.asarray(data.xfrc_applied[cube_body_id, 1:3], dtype=np.float64)
        static_hold_demand = float(np.linalg.norm(applied_tangent + gravity_tangent))
        friction_margin = capacity.available_friction_n - static_hold_demand
        friction_utilization = (
            static_hold_demand / capacity.available_friction_n
            if capacity.available_friction_n > 1e-12
            else (math.inf if static_hold_demand > 1e-12 else 0.0)
        )
        if phase in {"unsupported_hold", "disturbance", "recovery"}:
            minimum_friction_margin = min(minimum_friction_margin, friction_margin)
            peak_friction_utilization = max(peak_friction_utilization, friction_utilization)
        if phase == "unsupported_hold":
            if hold_reference is None:
                hold_reference = position
            hold_displacement = max(
                hold_displacement, _tangential_displacement(position, hold_reference)
            )
        if phase in {"disturbance", "recovery"}:
            if disturbance_reference is None:
                disturbance_reference = position
            disturbance_displacement = max(
                disturbance_displacement, _tangential_displacement(position, disturbance_reference)
            )
            disturbance_speed = max(disturbance_speed, float(np.linalg.norm(velocity[1:3])))
        rows.append(
            {
                "time_s": float(data.time),
                "phase": phase,
                "control_state": force_command.state,
                "control": motor_command.target_position,
                "drive_position_rad": motor_command.position,
                "drive_velocity_rad_s": motor_command.velocity,
                "motor_torque_n_m": motor_command.torque,
                "target_normal_force_n": force_command.target_force_n,
                "measured_normal_force_n": force_command.measured_force_n,
                "filtered_normal_force_n": force_command.filtered_force_n,
                "normal_force_error_n": force_command.force_error_n,
                "force_position_adjustment_rad": force_command.position_adjustment,
                "stiffness_position_adjustment_rad": (force_command.stiffness_position_adjustment),
                "force_feedforward_torque_n_m": force_command.force_feedforward_torque,
                "mit_feedforward_torque_n_m": motor_command.feedforward_torque,
                "estimated_contact_stiffness_n_per_m": (
                    force_command.estimated_contact_stiffness_n_per_m
                    if force_command.estimated_contact_stiffness_n_per_m is not None
                    else math.nan
                ),
                "closure_jacobian_m_per_rad": (
                    force_command.closure_jacobian_m_per_rad
                    if force_command.closure_jacobian_m_per_rad is not None
                    else math.nan
                ),
                "aperture_m": (
                    force_command.aperture_m if force_command.aperture_m is not None else math.nan
                ),
                "applied_world_fy": float(data.xfrc_applied[cube_body_id, 1]),
                "cube_y": float(position[1]),
                "cube_z": float(position[2]),
                "cube_vy": float(velocity[1]),
                "cube_vz": float(velocity[2]),
                "left_fx": float(tactile.left[0].sum()),
                "left_fy": float(tactile.left[1].sum()),
                "left_fz": float(tactile.left[2].sum()),
                "right_fx": float(tactile.right[0].sum()),
                "right_fy": float(tactile.right[1].sum()),
                "right_fz": float(tactile.right[2].sum()),
                "measured_left_fx": float(tactile_measurement.left_force[0]),
                "measured_left_fy": float(tactile_measurement.left_force[1]),
                "measured_left_fz": float(tactile_measurement.left_force[2]),
                "measured_right_fx": float(tactile_measurement.right_force[0]),
                "measured_right_fy": float(tactile_measurement.right_force[1]),
                "measured_right_fz": float(tactile_measurement.right_force[2]),
                "taxel_normal_force_n": capacity.normal_force_n,
                "left_taxel_normal_force_n": capacity.left_normal_force_n,
                "right_taxel_normal_force_n": capacity.right_normal_force_n,
                "active_taxel_contacts": capacity.active_contacts,
                "available_friction_n": capacity.available_friction_n,
                "applied_tangential_force_n": float(np.linalg.norm(applied_tangent)),
                "gravity_tangential_force_n": float(np.linalg.norm(gravity_tangent)),
                "static_hold_demand_tangential_n": static_hold_demand,
                "friction_margin_n": friction_margin,
                "friction_utilization": friction_utilization,
            }
        )
    # 首步即失稳时 rows 为空：跳过输出，仍以 simulation_stable=False 返回。
    if rows:
        if output_csv is not None:
            output_csv.parent.mkdir(parents=True, exist_ok=True)
            with output_csv.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        if output_plot is not None:
            plot_trace(output_plot, rows)
    tracking_rows = [row for row in rows if row["control_state"] == "force_tracking"]
    if tracking_rows:
        settled_after = float(tracking_rows[0]["time_s"]) + 0.2
        settled_forces = [
            float(row["filtered_normal_force_n"])
            for row in tracking_rows
            if float(row["time_s"]) >= settled_after
        ]
    else:
        settled_forces = []
    if settled_forces:
        force_target = float(tracking_rows[0]["target_normal_force_n"])
        force_tracking_mean = float(np.mean(settled_forces))
        force_tracking_rmse = float(
            np.sqrt(np.mean([(force - force_target) ** 2 for force in settled_forces]))
        )
    else:
        force_tracking_mean = 0.0
        force_tracking_rmse = math.inf
    return GraspAcceptance(
        hold_displacement_m=hold_displacement,
        disturbance_displacement_m=disturbance_displacement,
        disturbance_speed_m_s=disturbance_speed,
        minimum_friction_margin_n=minimum_friction_margin,
        peak_friction_utilization=peak_friction_utilization,
        force_tracking_rmse_n=force_tracking_rmse,
        force_tracking_mean_n=force_tracking_mean,
        hold_passed=hold_displacement <= hold_threshold_m,
        disturbance_passed=disturbance_displacement <= slip_threshold_m,
        force_tracking_passed=force_tracking_rmse <= force_rmse_threshold_n,
        simulation_stable=simulation_stable,
    )

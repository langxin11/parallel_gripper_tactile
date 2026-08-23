"""验证无支撑保持与切向扰动抵抗能力。

实验与 ``compare_tactile_models.py`` 使用相同的时序和默认 5 N、2 Hz 世界 Y 向
扰动。自研夹爪的 ``base`` 在其 profile 的 mount 位姿处固定，代表未来的转接
法兰，而不修改 CAD 基础本体。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
import math
from pathlib import Path

import mujoco
import numpy as np

from custom_grasp_scene import (
    CUBE_PREFIX,
    DEFAULT_CUBE_HALF_CONTACT_SIDE,
    DEFAULT_CUBE_HALF_THICKNESS,
    DEFAULT_CUBE_MASS,
    DEFAULT_PROFILE,
    GRIPPER_PREFIX,
    SUPPORT_GEOM_NAME,
    build_custom_grasp_model,
)
from parallel_gripper_tactile import ContactTaxelReader, DisturbanceProtocol, load_profile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
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
    hold_passed: bool
    disturbance_passed: bool
    simulation_stable: bool

    @property
    def passed(self) -> bool:
        """是否同时满足保持、扰动与仿真稳定性三项检查。"""
        return self.hold_passed and self.disturbance_passed and self.simulation_stable


def _prefixed_reader(model, profile) -> ContactTaxelReader:
    """创建读取 ``MjSpec.attach`` 前缀命名通道的接触读取器。"""
    tactile = replace(
        profile.tactile,
        left_prefix=f"{GRIPPER_PREFIX}{profile.tactile.left_prefix}",
        right_prefix=f"{GRIPPER_PREFIX}{profile.tactile.right_prefix}",
    )
    return ContactTaxelReader(model, tactile)


def _tangential_displacement(position: np.ndarray, reference: np.ndarray) -> float:
    """度量模型定义的 YZ 指尖接触平面内的位移。"""
    return float(np.linalg.norm(position[1:3] - reference[1:3]))


def plot_trace(path: Path, rows: list[dict[str, float | str]]) -> None:
    """绘制发表风格的控制量、触觉力与物体运动轨迹图。"""
    if not rows:
        raise ValueError("cannot plot an empty grasp trace")
    try:
        import matplotlib.pyplot as plt
        import scienceplots  # noqa: F401 -- 导入后注册 SciencePlots 样式。
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    plt.style.use(["science", "ieee", "no-latex"])
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.labelsize": 10,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 8,
        }
    )
    times = [float(row["time_s"]) for row in rows]
    control = [float(row["control"]) for row in rows]
    applied_force = [float(row["applied_world_fy"]) for row in rows]
    left_shear = [math.hypot(float(row["left_fx"]), float(row["left_fy"])) for row in rows]
    right_shear = [math.hypot(float(row["right_fx"]), float(row["right_fy"])) for row in rows]
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
    figure, axes = plt.subplots(5, 1, figsize=(7.16, 9.2), sharex=True, layout="constrained")
    control_axis, force_axis, tactile_axis, displacement_axis, speed_axis = axes
    control_axis.plot(times, control, color=colors["black"], linewidth=1.3, label="drive target")
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
    figure.savefig(path, dpi=600)
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
        output_csv: 可选逐步轨迹 CSV 输出路径。
        output_plot: 可选轨迹图输出路径。

    Returns:
        两项检查与仿真稳定性的验收结果。
    """
    if hold_threshold_m <= 0 or slip_threshold_m <= 0:
        raise ValueError("hold_threshold_m 和 slip_threshold_m 必须为正数。")
    profile = load_profile(profile_path)
    model = build_custom_grasp_model(
        profile,
        cube_half_thickness=cube_half_thickness,
        cube_half_contact_side=cube_half_contact_side,
        cube_mass=cube_mass,
    )
    data = mujoco.MjData(model)
    reader = _prefixed_reader(model, profile)
    actuator_id = model.actuator(f"{GRIPPER_PREFIX}{profile.actuator}").id
    support_id = model.geom(SUPPORT_GEOM_NAME).id
    cube_body_id = model.body(CUBE_BODY_NAME).id
    cube_joint_id = model.joint(CUBE_JOINT_NAME).id
    cube_dof = model.jnt_dofadr[cube_joint_id]

    hold_reference: np.ndarray | None = None
    disturbance_reference: np.ndarray | None = None
    hold_displacement = 0.0
    disturbance_displacement = 0.0
    disturbance_speed = 0.0
    simulation_stable = True
    rows: list[dict[str, float | str]] = []
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
        )
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
                "control": float(data.ctrl[actuator_id]),
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
    return GraspAcceptance(
        hold_displacement_m=hold_displacement,
        disturbance_displacement_m=disturbance_displacement,
        disturbance_speed_m_s=disturbance_speed,
        hold_passed=hold_displacement <= hold_threshold_m,
        disturbance_passed=disturbance_displacement <= slip_threshold_m,
        simulation_stable=simulation_stable,
    )


def main() -> int:
    """运行两次验收检查，任一失败时返回非零。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument(
        "--cube-half-thickness", type=float, default=DEFAULT_CUBE_HALF_THICKNESS, metavar="M"
    )
    parser.add_argument(
        "--cube-half-contact-side", type=float, default=DEFAULT_CUBE_HALF_CONTACT_SIDE, metavar="M"
    )
    parser.add_argument("--cube-mass", type=float, default=DEFAULT_CUBE_MASS, metavar="KG")
    parser.add_argument("--hold-threshold", type=float, default=0.002, metavar="M")
    parser.add_argument("--slip-threshold", type=float, default=0.002, metavar="M")
    parser.add_argument("--disturbance-force", type=float, default=5.0, metavar="N")
    parser.add_argument("--disturbance-frequency", type=float, default=2.0, metavar="HZ")
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV)
    parser.add_argument("--output-plot", type=Path, help="输出轨迹图；默认与 CSV 同名的 PNG")
    args = parser.parse_args()
    if args.cube_half_thickness <= 0 or args.cube_half_contact_side <= 0 or args.cube_mass <= 0:
        parser.error("方块尺寸和质量必须为正数")
    protocol = DisturbanceProtocol(
        force_n=args.disturbance_force, frequency_hz=args.disturbance_frequency
    )
    result = run_acceptance(
        args.profile,
        protocol=protocol,
        hold_threshold_m=args.hold_threshold,
        slip_threshold_m=args.slip_threshold,
        cube_half_thickness=args.cube_half_thickness,
        cube_half_contact_side=args.cube_half_contact_side,
        cube_mass=args.cube_mass,
        output_csv=args.output_csv,
        output_plot=args.output_plot or args.output_csv.with_suffix(".png"),
    )
    print(f"CSV:  {args.output_csv}")
    print(f"Plot: {args.output_plot or args.output_csv.with_suffix('.png')}")
    print("horizontal unsupported hold")
    print(
        f"max YZ displacement: {1000.0 * result.hold_displacement_m:.3f} mm  "
        f"{'PASS' if result.hold_passed else 'FAIL'}"
    )
    print("release-and-disturbance")
    print(
        f"max YZ displacement: {1000.0 * result.disturbance_displacement_m:.3f} mm; "
        f"max YZ speed: {result.disturbance_speed_m_s:.4f} m/s  "
        f"{'PASS' if result.disturbance_passed else 'FAIL'}"
    )
    if not result.simulation_stable:
        print("simulation stability: FAIL (MuJoCo reported a non-finite or reset state)")
    print(f"acceptance: {'PASS' if result.passed else 'FAIL'}")
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())

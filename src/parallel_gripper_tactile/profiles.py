"""Typed, path-safe gripper profiles used by simulations and tooling."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import tomllib


@dataclass(frozen=True, slots=True)
class TactileLayout:
    """Names and shape of the two fingertip tactile arrays."""

    mode: str
    rows: int
    cols: int
    left_prefix: str
    right_prefix: str

    def names(self, side: str) -> tuple[str, ...]:
        """Return row-major sensor or geometry names for one fingertip."""
        if side not in {"left", "right"}:
            raise ValueError("side must be 'left' or 'right'")
        prefix = self.left_prefix if side == "left" else self.right_prefix
        return tuple(f"{prefix}{row}{col}" for row in range(self.rows) for col in range(self.cols))


@dataclass(frozen=True, slots=True)
class MITControl:
    """MIT-style output-shaft position, velocity, and torque limits and gains."""

    p_min: float
    p_max: float
    v_max: float
    t_max: float
    kp: float
    kd: float
    t_ff: float


@dataclass(frozen=True, slots=True)
class NormalForceControl:
    """Outer-loop normal-force tracking parameters."""

    target_n: float
    contact_threshold_n: float
    contact_confirm_steps: int
    release_threshold_n: float
    release_confirm_steps: int
    kp: float
    ki: float
    kd: float
    max_position_adjustment: float
    filter_cutoff_hz: float


@dataclass(frozen=True, slots=True)
class GripperProfile:
    """Model-independent description of a parallel gripper."""

    name: str
    model_path: Path
    actuator: str
    open_control: float
    closed_control: float
    control_mode: str
    mit: MITControl | None
    normal_force: NormalForceControl | None
    mount_pos: tuple[float, float, float]
    mount_quat: tuple[float, float, float, float]
    tactile: TactileLayout


def _float_tuple(values: list[object], length: int, field: str) -> tuple[float, ...]:
    if len(values) != length:
        raise ValueError(f"{field} must contain {length} values")
    return tuple(float(value) for value in values)


def _find_repository_root(profile_path: Path) -> Path:
    """Walk upward from the profile to the directory containing ``assets``.

    Falls back to the profile's grandparent, preserving the historical
    ``configs/<profile>.toml`` layout for repositories without an ``assets``
    directory at the expected level.
    """
    for parent in profile_path.parents:
        if (parent / "assets").is_dir():
            return parent
    return profile_path.parents[1]


def _load_mit_control(control: dict[str, object]) -> MITControl | None:
    """Load and validate the optional MIT torque-controller configuration."""
    mode = str(control.get("mode", "position"))
    if mode == "position":
        return None
    if mode != "mit_torque":
        raise ValueError(f"unsupported control.mode {mode!r}")
    raw = control.get("mit")
    if not isinstance(raw, dict):
        raise ValueError("control.mit is required when control.mode='mit_torque'")
    mit = MITControl(
        p_min=float(raw["p_min"]),
        p_max=float(raw["p_max"]),
        v_max=float(raw["v_max"]),
        t_max=float(raw["t_max"]),
        kp=float(raw["kp"]),
        kd=float(raw["kd"]),
        t_ff=float(raw.get("t_ff", 0.0)),
    )
    if mit.p_min >= mit.p_max:
        raise ValueError("control.mit.p_min must be smaller than p_max")
    if mit.v_max <= 0 or mit.t_max <= 0:
        raise ValueError("control.mit.v_max and t_max must be positive")
    if mit.kp < 0 or mit.kd < 0:
        raise ValueError("control.mit.kp and kd must be nonnegative")
    if abs(mit.t_ff) > mit.t_max:
        raise ValueError("control.mit.t_ff must not exceed t_max")
    return mit


def _load_normal_force_control(control: dict[str, object]) -> NormalForceControl | None:
    """Load and validate the optional taxel normal-force outer loop."""
    raw = control.get("force")
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("control.force must be a table")
    force = NormalForceControl(
        target_n=float(raw["target_n"]),
        contact_threshold_n=float(raw["contact_threshold_n"]),
        contact_confirm_steps=int(raw["contact_confirm_steps"]),
        release_threshold_n=float(raw["release_threshold_n"]),
        release_confirm_steps=int(raw["release_confirm_steps"]),
        kp=float(raw["kp"]),
        ki=float(raw["ki"]),
        kd=float(raw.get("kd", 0.0)),
        max_position_adjustment=float(raw["max_position_adjustment"]),
        filter_cutoff_hz=float(raw["filter_cutoff_hz"]),
    )
    if force.target_n <= 0:
        raise ValueError("control.force.target_n must be positive")
    if force.contact_threshold_n <= 0:
        raise ValueError("control.force.contact_threshold_n must be positive")
    if not 0 <= force.release_threshold_n <= force.contact_threshold_n:
        raise ValueError("control.force.release_threshold_n must lie within contact threshold")
    if force.contact_confirm_steps <= 0 or force.release_confirm_steps <= 0:
        raise ValueError("control.force confirmation step counts must be positive")
    if force.kp < 0 or force.ki < 0 or force.kd < 0:
        raise ValueError("control.force PID gains must be nonnegative")
    if force.max_position_adjustment <= 0 or force.filter_cutoff_hz <= 0:
        raise ValueError("control.force position adjustment and filter cutoff must be positive")
    return force


def load_profile(path: str | Path, *, repository_root: str | Path | None = None) -> GripperProfile:
    """Load a TOML profile and resolve its MJCF path against the repository root."""
    profile_path = Path(path).resolve()
    with profile_path.open("rb") as stream:
        raw = tomllib.load(stream)

    root = (
        Path(repository_root).resolve() if repository_root else _find_repository_root(profile_path)
    )
    model_path = (root / raw["model"]["path"]).resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"gripper MJCF does not exist: {model_path}")

    control = raw["control"]
    mount = raw["mount"]
    tactile = raw["tactile"]
    control_mode = str(control.get("mode", "position"))
    mit = _load_mit_control(control)
    normal_force = _load_normal_force_control(control)
    if normal_force is not None and mit is None:
        raise ValueError("control.force requires control.mode='mit_torque'")
    open_control = float(control["open"])
    closed_control = float(control["closed"])
    if mit is not None and not (
        mit.p_min <= open_control <= mit.p_max and mit.p_min <= closed_control <= mit.p_max
    ):
        raise ValueError("control.open/closed must lie within control.mit position limits")
    return GripperProfile(
        name=str(raw["name"]),
        model_path=model_path,
        actuator=str(control["actuator"]),
        open_control=open_control,
        closed_control=closed_control,
        control_mode=control_mode,
        mit=mit,
        normal_force=normal_force,
        mount_pos=_float_tuple(mount["pos"], 3, "mount.pos"),  # type: ignore[arg-type]
        mount_quat=_float_tuple(mount["quat"], 4, "mount.quat"),  # type: ignore[arg-type]
        tactile=TactileLayout(
            mode=str(tactile["mode"]),
            rows=int(tactile["rows"]),
            cols=int(tactile["cols"]),
            left_prefix=str(tactile["left_prefix"]),
            right_prefix=str(tactile["right_prefix"]),
        ),
    )

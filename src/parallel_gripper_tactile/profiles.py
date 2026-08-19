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
        return tuple(
            f"{prefix}{row}{col}" for row in range(self.rows) for col in range(self.cols)
        )


@dataclass(frozen=True, slots=True)
class GripperProfile:
    """Model-independent description of a parallel gripper."""

    name: str
    model_path: Path
    actuator: str
    open_control: float
    closed_control: float
    mount_pos: tuple[float, float, float]
    mount_quat: tuple[float, float, float, float]
    tactile: TactileLayout


def _float_tuple(values: list[object], length: int, field: str) -> tuple[float, ...]:
    if len(values) != length:
        raise ValueError(f"{field} must contain {length} values")
    return tuple(float(value) for value in values)


def load_profile(path: str | Path, *, repository_root: str | Path | None = None) -> GripperProfile:
    """Load a TOML profile and resolve its MJCF path against the repository root."""
    profile_path = Path(path).resolve()
    with profile_path.open("rb") as stream:
        raw = tomllib.load(stream)

    root = Path(repository_root).resolve() if repository_root else profile_path.parents[1]
    model_path = (root / raw["model"]["path"]).resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"gripper MJCF does not exist: {model_path}")

    control = raw["control"]
    mount = raw["mount"]
    tactile = raw["tactile"]
    return GripperProfile(
        name=str(raw["name"]),
        model_path=model_path,
        actuator=str(control["actuator"]),
        open_control=float(control["open"]),
        closed_control=float(control["closed"]),
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

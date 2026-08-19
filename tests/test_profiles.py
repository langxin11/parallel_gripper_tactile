from pathlib import Path

import pytest

from parallel_gripper_tactile.profiles import load_profile
from parallel_gripper_tactile.validation import validate_profile


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "profile_name, actuator, channels",
    [
        ("robotiq_2f85.toml", "fingers_actuator", 18),
        ("custom_parallel_gripper.toml", "gripper_drive", 18),
    ],
)
def test_profiles_compile_and_satisfy_contract(
    profile_name: str, actuator: str, channels: int
) -> None:
    profile = load_profile(ROOT / "configs" / profile_name)
    report = validate_profile(profile)
    assert report.actuator == actuator
    assert report.tactile_channels == channels


def test_tactile_names_are_row_major() -> None:
    profile = load_profile(ROOT / "configs" / "custom_parallel_gripper.toml")
    assert profile.tactile.names("left") == (
        "left_taxel_geom_00",
        "left_taxel_geom_01",
        "left_taxel_geom_02",
        "left_taxel_geom_10",
        "left_taxel_geom_11",
        "left_taxel_geom_12",
        "left_taxel_geom_20",
        "left_taxel_geom_21",
        "left_taxel_geom_22",
    )

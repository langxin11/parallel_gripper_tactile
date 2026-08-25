"""验证 profile 加载、编译契约与行优先命名约定。"""

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
    """两个 profile 均可编译且满足触觉通道契约。"""
    profile = load_profile(ROOT / "configs" / profile_name)
    report = validate_profile(profile)
    assert report.actuator == actuator
    assert report.tactile_channels == channels


def test_tactile_names_are_row_major() -> None:
    """taxel 命名按行优先（00..22）排列。"""
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


def test_custom_profile_defines_bounded_mit_torque_control() -> None:
    """自研夹爪使用不超过 MJCF 与电机峰值的 MIT 力矩控制参数。"""
    profile = load_profile(ROOT / "configs" / "custom_parallel_gripper.toml")

    assert profile.control_mode == "mit_torque"
    assert profile.mit is not None
    assert profile.mit.p_min <= profile.open_control < profile.closed_control <= profile.mit.p_max
    assert profile.mit.v_max <= 20.943951023931955
    assert profile.mit.t_max <= 10.0
    assert profile.normal_force is not None
    assert profile.normal_force.target_n == 8.0
    assert profile.normal_force.release_threshold_n < profile.normal_force.contact_threshold_n

"""无 ROS、MuJoCo 依赖的 DMgripper 共享控制核。"""

from .control import (
    CrankSliderKinematics,
    MinimumJerkTrajectory,
    quintic_blend,
    within_zero_window,
    ContactTransition,
    SecondOrderAdmittance,
    limit_mit_position_for_torque,
    ContactDetector,
)
from .command import MITCommand, MITCommandConfig, build_mit_command, step_admittance

__version__ = "0.1.0"
__all__ = [
    "CrankSliderKinematics",
    "MinimumJerkTrajectory",
    "quintic_blend",
    "within_zero_window",
    "ContactTransition",
    "SecondOrderAdmittance",
    "limit_mit_position_for_torque",
    "ContactDetector",
    "MITCommand",
    "MITCommandConfig",
    "build_mit_command",
    "step_admittance",
]

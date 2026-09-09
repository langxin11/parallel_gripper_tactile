"""无 ROS、MuJoCo 依赖的 DMgripper 共享控制核。"""

from .control import CrankSliderKinematics, SecondOrderAdmittance, limit_mit_position_for_torque
from .grasp import (
    ContactTransition,
    MITCommand,
    MITCommandConfig,
    MinimumJerkTrajectory,
    build_mit_command,
    quintic_blend,
    step_admittance,
)
from .tactile import ContactDetector, within_zero_window

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

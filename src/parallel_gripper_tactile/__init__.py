"""MuJoCo parallel-gripper tactile simulation toolkit."""

from .profiles import GripperProfile, TactileLayout, load_profile

__all__ = ["GripperProfile", "TactileLayout", "load_profile"]
__version__ = "0.2.0"

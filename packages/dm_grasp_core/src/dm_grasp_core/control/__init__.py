"""DMgripper 的机构、导纳、MIT 协议量化与法向力跟踪核心算法。"""

import math  # noqa: F401
from dataclasses import dataclass  # noqa: F401

from .admittance import (
    SecondOrderAdmittance as SecondOrderAdmittance,
    limit_mit_position_for_torque as limit_mit_position_for_torque,
)
from .adrc import (
    SecondOrderTorqueLADRC as SecondOrderTorqueLADRC,
    TorqueAdrcConfig as TorqueAdrcConfig,
    TorqueAdrcStep as TorqueAdrcStep,
)
from .kinematics import CrankSliderKinematics as CrankSliderKinematics
from .mit import (
    DAMIAO_DAMPING_RANGE as DAMIAO_DAMPING_RANGE,
    DAMIAO_GAIN_BITS as DAMIAO_GAIN_BITS,
    DAMIAO_POSITION_BITS as DAMIAO_POSITION_BITS,
    DAMIAO_STIFFNESS_RANGE as DAMIAO_STIFFNESS_RANGE,
    DAMIAO_TORQUE_BITS as DAMIAO_TORQUE_BITS,
    DAMIAO_VELOCITY_BITS as DAMIAO_VELOCITY_BITS,
    MITControlCommand as MITControlCommand,
    MITControlConfig as MITControlConfig,
    MITTorqueModel as MITTorqueModel,
)
from .normal_force import (
    AdrcConfig as AdrcConfig,
    ForceControlObservation as ForceControlObservation,
    ForceControlReference as ForceControlReference,
    ForceSemantics as ForceSemantics,
    MITTorqueInner as MITTorqueInner,
    NormalForceConfig as NormalForceConfig,
    NormalForceControlCommand as NormalForceControlCommand,
    NormalForceController as NormalForceController,
)
from .stiffness import (
    ContactStiffnessConfig as ContactStiffnessConfig,
    ContactStiffnessEstimator as ContactStiffnessEstimator,
    StiffnessEstimatorMethod as StiffnessEstimatorMethod,
)
from ..grasp.motion import (
    ContactTransition as ContactTransition,
    MinimumJerkTrajectory as MinimumJerkTrajectory,
    quintic_blend as quintic_blend,
)
from ..tactile import ContactDetector as ContactDetector, within_zero_window as within_zero_window

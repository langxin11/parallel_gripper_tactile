"""DM4310P USB2CAN 的纯 Python 硬件基础层。"""

from .config import DEFAULT_USB2CAN_BAUD_RATE, Usb2CanDeviceConfig
from .deployment import Dm4310PGripperConfig, make_dm4310p_gripper_config
from .adapter import (
    CoreMotorObservation,
    DmMitCommandAdapter,
    PreparedMitCommand,
    core_observation_from_feedback,
    map_core_mit_command,
)
from .protocol import (
    CAN_BROADCAST_ID,
    CMD_DISABLE,
    CMD_ENABLE,
    CMD_SET_ZERO,
    CONTROL_MODE_REGISTER,
    STATUS_DISABLED,
    STATUS_ENABLED,
    ForcePositionCommand,
    MitCommand,
    MotorFeedback,
    MotorLimits,
    PositionVelocityCommand,
    RegisterReply,
    TorqueCommand,
    Usb2CanFrameExtractor,
    Usb2CanProtocol,
    VelocityCommand,
    motor_status_is_fault,
    motor_status_to_fault_code,
)
from .runtime import DmStateRefresher, ShortWriteError, StateRefreshTimeoutError
from .serial_transport import PySerialTransport, SerialFactory
from .transport import ByteTransport, FakeTransport, TransportClosedError

__version__ = "0.1.0"
__all__ = [
    "CAN_BROADCAST_ID",
    "CMD_DISABLE",
    "CMD_ENABLE",
    "CMD_SET_ZERO",
    "CONTROL_MODE_REGISTER",
    "DEFAULT_USB2CAN_BAUD_RATE",
    "Dm4310PGripperConfig",
    "STATUS_DISABLED",
    "STATUS_ENABLED",
    "ByteTransport",
    "CoreMotorObservation",
    "DmStateRefresher",
    "DmMitCommandAdapter",
    "FakeTransport",
    "ForcePositionCommand",
    "MitCommand",
    "MotorFeedback",
    "MotorLimits",
    "PositionVelocityCommand",
    "PreparedMitCommand",
    "PySerialTransport",
    "RegisterReply",
    "SerialFactory",
    "ShortWriteError",
    "StateRefreshTimeoutError",
    "TorqueCommand",
    "TransportClosedError",
    "Usb2CanFrameExtractor",
    "Usb2CanDeviceConfig",
    "Usb2CanProtocol",
    "VelocityCommand",
    "motor_status_is_fault",
    "motor_status_to_fault_code",
    "core_observation_from_feedback",
    "map_core_mit_command",
    "make_dm4310p_gripper_config",
]

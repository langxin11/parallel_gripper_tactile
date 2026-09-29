"""Contactile PapillArray 的 PTS v2.0 同步串口采集接口。"""

from .acquisition import (
    FirstOrderLowPassFilter,
    PacketIntegrityDiagnostics,
    PacketIntegrityError,
    PacketIntegrityTracker,
    TactileSnapshot,
    TactileWorker,
)

from .client import (
    DEFAULT_PAPILLARRAY_PORT,
    TACTILE_NORMAL_RANGE_N,
    TACTILE_TANGENTIAL_RANGE_N,
    TAXELS_PER_SIDE,
    PapillArraySerialClient,
    PapillArraySerialConfig,
)
from .pillar_friction import (
    PillarFrictionConfig,
    PillarFrictionEstimate,
    PillarFrictionEstimator,
)
from .standalone import StandaloneSlipConfig, StandaloneSlipSession
from .protocol import (
    PacketChecksumError,
    ProtocolError,
    PtsPacket,
    PtsReadDiagnostics,
    PtsReadTimeout,
    PtsStreamReader,
    parse_packet,
)

__all__ = [
    "FirstOrderLowPassFilter",
    "PacketChecksumError",
    "DEFAULT_PAPILLARRAY_PORT",
    "TACTILE_NORMAL_RANGE_N",
    "TACTILE_TANGENTIAL_RANGE_N",
    "TAXELS_PER_SIDE",
    "PapillArraySerialClient",
    "PapillArraySerialConfig",
    "PillarFrictionConfig",
    "PillarFrictionEstimate",
    "PillarFrictionEstimator",
    "PacketIntegrityDiagnostics",
    "PacketIntegrityError",
    "PacketIntegrityTracker",
    "ProtocolError",
    "PtsPacket",
    "PtsReadDiagnostics",
    "PtsReadTimeout",
    "PtsStreamReader",
    "TactileSnapshot",
    "TactileWorker",
    "StandaloneSlipConfig",
    "StandaloneSlipSession",
    "parse_packet",
]

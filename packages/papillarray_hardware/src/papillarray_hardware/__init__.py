"""Contactile PapillArray 的 PTS v2.0 同步串口采集接口。"""

from .acquisition import (
    FirstOrderLowPassFilter,
    PacketIntegrityDiagnostics,
    PacketIntegrityError,
    PacketIntegrityTracker,
    TactileSnapshot,
    TactileWorker,
)

from .client import DEFAULT_PAPILLARRAY_PORT, PapillArraySerialClient, PapillArraySerialConfig
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
    "PapillArraySerialClient",
    "PapillArraySerialConfig",
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
    "parse_packet",
]

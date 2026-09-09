"""Contactile PapillArray 的 PTS v2.0 同步串口采集接口。"""

from .client import PapillArraySerialClient, PapillArraySerialConfig
from .protocol import PacketChecksumError, ProtocolError, PtsPacket, PtsStreamReader, parse_packet

__all__ = [
    "PacketChecksumError",
    "PapillArraySerialClient",
    "PapillArraySerialConfig",
    "ProtocolError",
    "PtsPacket",
    "PtsStreamReader",
    "parse_packet",
]

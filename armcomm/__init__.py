"""ArmPi Ultra 上位机通信库。

* :mod:`armcomm.protocol` —— AA55 帧编解码与 CRC-8
* :mod:`armcomm.servos` —— 舵机 ID、关节映射、角度与位置换算
"""

from .protocol import (
    BusServoCommand,
    BusServoReport,
    FrameParser,
    PacketFunction,
    ParsedFrame,
    READ_ONLY_BUS_SERVO_COMMANDS,
    assert_read_only_frame,
    build_bus_servo_frame,
    build_frame,
    crc8,
    decode_bus_servo_report,
    hexs,
)

__all__ = [
    "BusServoCommand",
    "BusServoReport",
    "FrameParser",
    "PacketFunction",
    "ParsedFrame",
    "READ_ONLY_BUS_SERVO_COMMANDS",
    "assert_read_only_frame",
    "build_bus_servo_frame",
    "build_frame",
    "crc8",
    "decode_bus_servo_report",
    "hexs",
]

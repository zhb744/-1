"""ArmPi Ultra 上位机通信层 —— AA55 帧协议。

实现严格对齐 STM32 固件的实测行为，不是"看起来合理的通用协议"：

* 帧结构与接收状态机 —— ``Hiwonder/Misc/packet.c``
* CRC-8（反射多项式 0x8C，初值 0，无输出异或）—— ``Hiwonder/Misc/checksum.c``
* 子命令与回包布局 —— ``Hiwonder/System/packet_handle.c`` + ``packet_reports.h``

需要特别注意固件的两个既成事实，本模块都按真实行为处理：

1. **回包尾部多 1 个垃圾字节**。``packet.c`` 的 ``transmit()`` 写 ``data_len + 6`` 字节，
   而一帧实际只需要 ``data_len + 5`` 字节，因此每帧后面跟 1 个未初始化字节。
   解析器必须按帧头状态机走，绝不能按定长切包。
2. **回包第 3 字节的 0 才是成功**。固件把 ``serial_servo_read_xxx()`` 的返回值
   （0=成功，-1=舵机无应答）原样写进该字段，所以 ``0x00`` 表示读取成功，
   ``0xFF`` 表示无应答。字段名叫 ``success`` 是误导性的，本模块称之为 ``error_code``。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, unique
from typing import Iterable, List, Sequence

__all__ = [
    "START_BYTE_1",
    "START_BYTE_2",
    "PACKET_FUNC_NONE",
    "MAX_DATA_LENGTH",
    "CRC8_TABLE",
    "crc8",
    "PacketFunction",
    "BusServoCommand",
    "READ_ONLY_BUS_SERVO_COMMANDS",
    "ParsedFrame",
    "BusServoReport",
    "FrameParser",
    "build_frame",
    "build_bus_servo_frame",
    "assert_read_only_frame",
    "hexs",
]

START_BYTE_1 = 0xAA
START_BYTE_2 = 0x55

#: ``PACKET_FUNC_NONE``，功能号必须严格小于它，否则固件状态机直接丢帧。
PACKET_FUNC_NONE = 14

#: 长度字段是 1 字节，data 段最长 255 字节。
MAX_DATA_LENGTH = 255

#: 帧头 2 + 功能号 1 + 长度 1 + 校验 1。
FRAME_OVERHEAD = 5


def _build_crc8_table(polynomial: int = 0x8C) -> tuple:
    """构造反射 CRC-8 查表。

    0x8C 是 0x07 的位反转，配合"初值 0、无反射输入输出、无异或"就是固件
    ``crc8_table[]`` 的前 32 项 ``0,94,188,226,97,63,...``（由测试逐项核对）。
    """
    table = []
    for index in range(256):
        crc = index
        for _ in range(8):
            crc = (crc >> 1) ^ polynomial if crc & 1 else crc >> 1
        table.append(crc)
    return tuple(table)


CRC8_TABLE = _build_crc8_table()


def crc8(data: bytes) -> int:
    """固件 ``checksum_crc8()`` 的等价实现。"""
    crc = 0
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


@unique
class PacketFunction(IntEnum):
    SYS = 0
    LED = 1
    BUZZER = 2
    MOTOR = 3
    PWM_SERVO = 4
    BUS_SERVO = 5
    KEY = 6
    IMU = 7
    GAMEPAD = 8
    SBUS = 9
    OLED = 10
    RGB = 11
    CONVEYOR = 12
    STEPPER_MOTOR = 13


@unique
class BusServoCommand(IntEnum):
    """``packet_handle.c`` 里 ``packet_serial_servo_handle()`` 实际支持的子命令。

    标注 ``R`` 的会回包，其余只执行不回包。
    """

    SET_POSITION = 0x01  # 多舵机 + 时间，会产生运动
    STOP = 0x03
    READ_POSITION = 0x05  # R
    READ_VIN = 0x07  # R
    READ_TEMP = 0x09  # R
    UNLOAD = 0x0B  # 卸力，舵机失去保持力矩
    LOAD = 0x0C  # 加载
    READ_LOAD_STATE = 0x0D  # R
    WRITE_ID = 0x10  # 改舵机 ID，误写等于把舵机从总线上弄丢
    READ_ID = 0x12  # R
    SET_DEVIATION = 0x20
    READ_DEVIATION = 0x22  # R
    SAVE_DEVIATION = 0x24
    SET_ANGLE_LIMIT = 0x30
    READ_ANGLE_LIMIT = 0x32  # R
    SET_VIN_LIMIT = 0x34
    READ_VIN_LIMIT = 0x36  # R
    SET_TEMP_LIMIT = 0x38
    READ_TEMP_LIMIT = 0x3A  # R


#: 自检脚本唯一允许发送的子命令集合 —— 全部只读，不会让任何关节动一下。
READ_ONLY_BUS_SERVO_COMMANDS = frozenset(
    {
        BusServoCommand.READ_POSITION,
        BusServoCommand.READ_VIN,
        BusServoCommand.READ_TEMP,
        BusServoCommand.READ_LOAD_STATE,
        BusServoCommand.READ_ID,
        BusServoCommand.READ_DEVIATION,
        BusServoCommand.READ_ANGLE_LIMIT,
        BusServoCommand.READ_VIN_LIMIT,
        BusServoCommand.READ_TEMP_LIMIT,
    }
)


@dataclass(frozen=True)
class ParsedFrame:
    function: int
    data: bytes

    def __str__(self) -> str:  # pragma: no cover - 仅便于打印
        return f"func={self.function} len={len(self.data)} data={hexs(self.data)}"


@dataclass(frozen=True)
class BusServoReport:
    """``PacketReportSerialServoTypeDef`` 的解码结果。

    回包 data 段布局：``servo_id | sub_command | error_code | args...``
    """

    servo_id: int
    sub_command: int
    error_code: int
    args: bytes

    @property
    def answered(self) -> bool:
        """``error_code == 0`` 表示舵机在总线上应答了（固件语义见模块 docstring）。"""
        return self.error_code == 0

    @property
    def int16(self) -> int:
        """位置回读：小端有符号 16 位，范围 -1000~1000。"""
        return int.from_bytes(self.args[:2], "little", signed=True)

    @property
    def uint16(self) -> int:
        """电压类回读：小端无符号 16 位。"""
        return int.from_bytes(self.args[:2], "little", signed=False)

    @property
    def uint8(self) -> int:
        """温度、加载状态、ID 等单字节回读。"""
        return self.args[0]


class FrameParser:
    """与固件 ``receive_handle()`` 等价的状态机解析器。

    逐字节状态转移完全照抄固件，因此上位机对同一串字节的接受/拒绝结果与板子一致；
    尾部垃圾字节会被状态机自然吞掉。
    """

    _START_1 = "STARTBYTE1"
    _START_2 = "STARTBYTE2"
    _FUNCTION = "FUNCTION"
    _LENGTH = "LENGTH"
    _DATA = "DATA"
    _CHECKSUM = "CHECKSUM"

    def __init__(self) -> None:
        self.state = self._START_1
        self._function = 0
        self._length = 0
        self._index = 0
        self._buffer = bytearray()
        #: 统计信息，用于诊断波特率不匹配或线路噪声。
        self.bytes_consumed = 0
        self.discarded_bytes = 0
        self.checksum_errors = 0
        self.frames_parsed = 0

    def reset(self) -> None:
        """彻底复位，统计一并清零。"""
        self.__init__()

    def resync(self) -> None:
        """只把状态机拉回"找帧头"，保留统计计数。

        固件每帧尾部多发 1 个未初始化字节。它通常是普通垃圾，但一旦正好等于 0xAA，
        就会把下一帧的真帧头吞掉，导致之后所有回包整体错位一帧（实机 COM4 上确实出现过：
        请求舵机 4 的 ID，收到的却是舵机 3 的回包）。发新请求前配合清空系统接收缓冲调用它，
        才能彻底摆脱上一次的残留。
        """
        self.state = self._START_1
        self._function = 0
        self._length = 0
        self._index = 0
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[ParsedFrame]:
        """喂入任意长度的字节块，返回本次解析出的完整帧（可能为 0 帧）。"""
        frames: List[ParsedFrame] = []
        for byte in bytes(chunk):
            self.bytes_consumed += 1
            frame = self._step(byte)
            if frame is not None:
                frames.append(frame)
        return frames

    def _step(self, byte: int):
        state = self.state

        if state == self._START_1:
            if byte == START_BYTE_1:
                self.state = self._START_2
            else:
                self.discarded_bytes += 1
            return None

        if state == self._START_2:
            # 固件这里不区分"又是 0xAA"，不匹配就退回 START_1，照抄。
            if byte == START_BYTE_2:
                self.state = self._FUNCTION
            else:
                self.discarded_bytes += 1
                self.state = self._START_1
            return None

        if state == self._FUNCTION:
            if byte < PACKET_FUNC_NONE:
                self._function = byte
                self.state = self._LENGTH
            else:
                self.discarded_bytes += 1
                self.state = self._START_1
            return None

        if state == self._LENGTH:
            self._length = byte
            self._index = 0
            self._buffer = bytearray()
            self.state = self._CHECKSUM if byte == 0 else self._DATA
            return None

        if state == self._DATA:
            self._buffer.append(byte)
            self._index += 1
            if self._index >= self._length:
                self.state = self._CHECKSUM
            return None

        # CHECKSUM
        expected = crc8(bytes([self._function, self._length]) + bytes(self._buffer))
        if byte == expected:
            self.frames_parsed += 1
            frame = ParsedFrame(function=self._function, data=bytes(self._buffer))
        else:
            self.checksum_errors += 1
            frame = None
        self._buffer = bytearray()
        self._index = 0
        self.state = self._START_1
        return frame


def build_frame(function: PacketFunction | int, data: bytes = b"") -> bytes:
    """构造一帧。校验范围是 ``function + length + data``，不含帧头。"""
    func = int(function)
    payload = bytes(data)
    if not 0 <= func < PACKET_FUNC_NONE:
        raise ValueError(f"功能号 {func} 超出固件接受范围 0~{PACKET_FUNC_NONE - 1}")
    if len(payload) > MAX_DATA_LENGTH:
        raise ValueError(f"data 段 {len(payload)} 字节，超过上限 {MAX_DATA_LENGTH}")
    body = bytes([func, len(payload)]) + payload
    return bytes([START_BYTE_1, START_BYTE_2]) + body + bytes([crc8(body)])


def build_bus_servo_frame(command: BusServoCommand | int, args: Sequence[int] = ()) -> bytes:
    """构造总线舵机类（功能号 5）指令帧。"""
    cmd = int(command)
    if cmd > MAX_DATA_LENGTH - 1:
        raise ValueError("子命令码异常")
    data = bytes([cmd]) + bytes(args)
    return build_frame(PacketFunction.BUS_SERVO, data)


def assert_read_only_frame(frame: bytes) -> ParsedFrame:
    """安全闸门：只有"回包会被丢弃、舵机不会动"的只读帧才放行。

    自检脚本对每一帧都过这道关，任何运动类、写参数类指令在这里直接抛异常，
    避免误调用把机械臂通电。
    """
    parser = FrameParser()
    frames = parser.feed(frame)
    if len(frames) != 1:
        raise ValueError("不是单个合法帧，拒绝发送")
    parsed = frames[0]
    if parsed.function != PacketFunction.BUS_SERVO:
        raise ValueError(f"自检模式只允许 BUS_SERVO 只读指令，收到功能号 {parsed.function}")
    if not parsed.data:
        raise ValueError("帧缺少子命令字节")
    command = parsed.data[0]
    if command not in READ_ONLY_BUS_SERVO_COMMANDS:
        raise ValueError(f"子命令 0x{command:02X} 不是只读指令，自检模式已拦截")
    return parsed


def decode_bus_servo_report(frame: ParsedFrame) -> BusServoReport:
    """把功能号 5 的回包解成结构体。"""
    if frame.function != PacketFunction.BUS_SERVO:
        raise ValueError(f"不是 BUS_SERVO 回包（功能号 {frame.function}）")
    if len(frame.data) < 3:
        raise ValueError(f"回包长度 {len(frame.data)} 不足 3 字节头部")
    return BusServoReport(
        servo_id=frame.data[0],
        sub_command=frame.data[1],
        error_code=frame.data[2],
        args=frame.data[3:],
    )


def hexs(data: Iterable[int]) -> str:
    return " ".join(f"{b:02X}" for b in bytes(data))

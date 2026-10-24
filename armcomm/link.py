"""串口传输层 —— 把"发一帧、只认这次请求的回包"的纪律集中在一处。

自检脚本和仿真服务共用这一层，保证两者的收包行为完全一致。纪律是被实机逼出来的：
固件 ``transmit()`` 每帧尾部多写 1 个未初始化字节，它偶尔正好是 ``0xAA``，会吞掉下一帧的
真帧头，导致之后所有回包整体错位一帧。所以每条请求前必须清空系统接收缓冲并复位状态机，
回包还要同时核对功能号、子命令、舵机 ID 三项才采纳。
"""

from __future__ import annotations

import time
from typing import Dict, Iterable, List

from .protocol import (
    BusServoCommand,
    BusServoReport,
    FrameParser,
    PacketFunction,
    ParsedFrame,
    build_bus_servo_frame,
    decode_bus_servo_report,
)
from .servos import JOINTS, validate_positions

DEFAULT_BAUD = 1_000_000  # 固件 USART1 就是 1 Mbps，不是 115200
DEFAULT_TIMEOUT_S = 0.3


def reply_matches(reply: ParsedFrame, sub_command=None, servo_id=None) -> bool:
    """只承认功能号、子命令、舵机 ID 三项全对得上的回包。"""
    if reply.function != int(PacketFunction.BUS_SERVO):
        return False
    if len(reply.data) < 3:
        return False
    if sub_command is not None and reply.data[1] != int(sub_command):
        return False
    if servo_id is not None and reply.data[0] != int(servo_id):
        return False
    return True


def exchange(port, parser: FrameParser, frame: bytes, timeout_s: float, sub_command=None, servo_id=None) -> tuple:
    """写一帧，只等待属于本次请求的回包。返回 (回包或 None, 被跳过的杂帧数)。"""
    port.reset_input_buffer()
    parser.resync()
    port.write(frame)
    deadline = time.monotonic() + timeout_s
    skipped = 0
    while time.monotonic() < deadline:
        chunk = port.read(1)
        if not chunk:
            continue
        for reply in parser.feed(chunk):
            if reply_matches(reply, sub_command, servo_id):
                return reply, skipped
            skipped += 1
    return None, skipped


def set_position_frame(positions: Dict[int, int], duration_ms: int = 1000) -> bytes:
    """构造子命令 ``0x01`` 多舵机同步位置帧。

    data 段：``01 | duration u16LE | servo_num | (id, position i16LE) × n``。
    所有关节放在同一帧里下发，固件会连续写完再让舵机按同一个 duration 运动，
    分多条帧发就会出现各关节不同步的扭曲姿态。
    """
    errors: List[str] = []
    if not positions:
        raise ValueError("位置表为空")
    if not 0 <= duration_ms <= 30000:
        errors.append(f"运行时间 {duration_ms}ms 超出固件范围 0~30000ms")
    validate_positions(positions, errors)
    if errors:
        raise ValueError("；".join(errors))

    body = bytearray()
    body += int(duration_ms).to_bytes(2, "little")
    body.append(len(positions))
    for servo_id in sorted(positions):
        position = int(positions[servo_id])
        body.append(servo_id)
        body += position.to_bytes(2, "little", signed=True)
    return build_bus_servo_frame(BusServoCommand.SET_POSITION, bytes(body))


def stop_frame(servo_ids: Iterable[int]) -> bytes:
    """构造子命令 ``0x03`` 停止帧（安全动作，不算运动指令）。"""
    ids = sorted(set(int(sid) for sid in servo_ids))
    if not ids:
        raise ValueError("停止指令至少要指定一个舵机")
    unknown = [sid for sid in ids if sid not in JOINTS]
    if unknown:
        raise ValueError(f"舵机 ID 不在 1~6 范围内：{unknown}")
    return build_bus_servo_frame(BusServoCommand.STOP, bytes([len(ids)]) + bytes(ids))


class SerialLink:
    """独占一个 COM 口的连接对象。Windows 串口是独占的，全局只允许一个实例持有端口。"""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout_s: float = DEFAULT_TIMEOUT_S):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.parser = FrameParser()
        self.skipped_frames = 0
        self._serial = None

    # ---------------------------------------------------------------- 生命周期
    def open(self):
        if self._serial is not None:
            return self
        import serial  # 延迟导入：单元测试用假端口，不需要 pyserial

        self._serial = serial.Serial(
            port=self.port, baudrate=self.baud, bytesize=8, parity="N", stopbits=1, timeout=0.02
        )
        self.parser = FrameParser()
        self.skipped_frames = 0
        return self

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc_info):
        self.close()
        return False

    # ---------------------------------------------------------------- 原语
    def _require_port(self):
        if self._serial is None:
            raise RuntimeError(f"端口 {self.port} 未打开")
        return self._serial

    def read_servo(self, command: BusServoCommand, servo_id: int) -> BusServoReport | None:
        """发一条只读子命令并解出回包；None 表示板子没回或回包对不上。"""
        port = self._require_port()
        frame = build_bus_servo_frame(command, [servo_id])
        reply, skipped = exchange(port, self.parser, frame, self.timeout_s, command, servo_id)
        self.skipped_frames += skipped
        if reply is None:
            return None
        return decode_bus_servo_report(reply)

    def read_positions(self, servo_ids: Iterable[int]) -> dict:
        """{servo_id: 位置}，只包含应答成功的舵机。"""
        result = {}
        for servo_id in servo_ids:
            report = self.read_servo(BusServoCommand.READ_POSITION, servo_id)
            if report is not None and report.answered:
                result[servo_id] = report.int16
        return result

    def read_snapshot(self, servo_ids: Iterable[int]) -> dict:
        """一次完整只读快照：位置、电压、温度、加载状态。"""
        snapshot = {}
        for servo_id in servo_ids:
            entry = {"name": JOINTS[servo_id].name if servo_id in JOINTS else f"ID {servo_id}"}
            position = self.read_servo(BusServoCommand.READ_POSITION, servo_id)
            vin = self.read_servo(BusServoCommand.READ_VIN, servo_id)
            temp = self.read_servo(BusServoCommand.READ_TEMP, servo_id)
            load = self.read_servo(BusServoCommand.READ_LOAD_STATE, servo_id)
            entry["ok"] = bool(position and position.answered)
            entry["position"] = position.int16 if position and position.answered else None
            entry["vin"] = vin.uint16 if vin and vin.answered else None
            entry["temperature"] = temp.uint8 if temp and temp.answered else None
            entry["loaded"] = bool(load.uint8) if load and load.answered else None
            snapshot[servo_id] = entry
        return snapshot

    def write_positions(self, positions: Dict[int, int], duration_ms: int = 1000) -> bytes:
        """下发同步位置指令。帧内容由 :func:`set_position_frame` 校验过限位。"""
        port = self._require_port()
        frame = set_position_frame(positions, duration_ms)
        port.reset_input_buffer()
        self.parser.resync()
        port.write(frame)
        return frame

    def write_stop(self, servo_ids: Iterable[int]) -> bytes:
        port = self._require_port()
        frame = stop_frame(servo_ids)
        port.reset_input_buffer()
        self.parser.resync()
        port.write(frame)
        return frame

    def set_loaded(self, loaded: bool, servo_ids: Iterable[int]) -> List[bytes]:
        """逐条加载(0x0C)/卸力(0x0B)。不会让关节运动，但决定舵机是否保持力矩。"""
        command = BusServoCommand.LOAD if loaded else BusServoCommand.UNLOAD
        ids = sorted(set(int(sid) for sid in servo_ids))
        unknown = [sid for sid in ids if sid not in JOINTS]
        if unknown:
            raise ValueError(f"舵机 ID 不在 1~6 范围内：{unknown}")
        port = self._require_port()
        frames = []
        for servo_id in ids:
            frame = build_bus_servo_frame(command, [servo_id])
            port.reset_input_buffer()
            self.parser.resync()
            port.write(frame)
            frames.append(frame)
        return frames

    @property
    def stats(self) -> dict:
        return {
            "bytes": self.parser.bytes_consumed,
            "frames": self.parser.frames_parsed,
            "discarded": self.parser.discarded_bytes,
            "checksum_errors": self.parser.checksum_errors,
            "skipped": self.skipped_frames,
        }

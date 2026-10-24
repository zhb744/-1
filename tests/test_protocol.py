"""协议层单元测试。

这些测试的作用不是"证明 Python 写法自洽"，而是把实现钉死在 STM32 固件的真实行为上：
CRC 表逐字节抄自 ``Hiwonder/Misc/checksum.c``，样例帧由固件打包规则推出后再用本模块
重算，尾部垃圾字节、error_code 语义等坑都来自 ``packet.c`` / ``packet_handle.c``。
"""

import pytest

from armcomm.protocol import (
    BusServoCommand,
    FrameParser,
    PacketFunction,
    START_BYTE_1,
    START_BYTE_2,
    assert_read_only_frame,
    build_bus_servo_frame,
    build_frame,
    crc8,
    CRC8_TABLE,
    decode_bus_servo_report,
)

# 固件 Hiwonder/Misc/checksum.c 里 static const uint8_t crc8_table[] 的全部 256 项，
# 逐行照抄，作为独立基准（不是由本模块算法生成，避免自证）。
FIRMWARE_CRC8_TABLE = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
]


def test_crc8_table_matches_firmware_byte_by_byte():
    assert list(CRC8_TABLE) == FIRMWARE_CRC8_TABLE


def test_crc8_table_is_complete():
    assert len(FIRMWARE_CRC8_TABLE) == 256
    assert len(CRC8_TABLE) == 256


@pytest.mark.parametrize(
    "data, expected",
    [
        (bytes([0x05, 0x07, 0x01, 0xE8, 0x03, 0x01, 0x06, 0xF4, 0x01]), 0x8F),
    ],
)
def test_crc8_golden_values(data, expected):
    assert crc8(data) == expected


def test_build_frame_matches_firmware_layout():
    """AA 55 05 07 01 E8 03 01 06 F4 01 8F

    = 读/写位置(0x01) + duration=1000ms(小端 E8 03) + 舵机数 1 + (ID 6, 位置 500)。
    这一帧是与固件 checksum_crc8() 的等价实现算出来的，逐字节锁定，防止以后改错。
    """
    frame = bytes(
        [
            BusServoCommand.SET_POSITION,
            0xE8,
            0x03,
            0x01,
            0x06,
            0xF4,
            0x01,
        ]
    )
    assert build_frame(PacketFunction.BUS_SERVO, frame) == bytes(
        [0xAA, 0x55, 0x05, 0x07, 0x01, 0xE8, 0x03, 0x01, 0x06, 0xF4, 0x01, 0x8F]
    )


def test_build_bus_servo_read_position_frame():
    frame = build_bus_servo_frame(BusServoCommand.READ_POSITION, [6])
    assert frame[:4] == bytes([START_BYTE_1, START_BYTE_2, 0x05, 0x02])
    assert frame[4:6] == bytes([0x05, 0x06])
    assert crc8(frame[2:6]) == frame[6]


def test_build_frame_rejects_out_of_range_function():
    with pytest.raises(ValueError):
        build_frame(14, b"\x01")
    with pytest.raises(ValueError):
        build_frame(-1, b"\x01")


def test_build_frame_rejects_oversized_payload():
    with pytest.raises(ValueError):
        build_frame(PacketFunction.BUS_SERVO, bytes(256))


def test_parser_roundtrip():
    parser = FrameParser()
    frames = parser.feed(build_bus_servo_frame(BusServoCommand.READ_ID, [3]))
    assert len(frames) == 1
    assert frames[0].function == int(PacketFunction.BUS_SERVO)
    assert frames[0].data == bytes([int(BusServoCommand.READ_ID), 3])


def test_parser_handles_byte_by_byte_input():
    """串口是分片到达的，逐字节喂入必须得到同一帧。"""
    data = build_bus_servo_frame(BusServoCommand.READ_TEMP, [1])
    parser = FrameParser()
    collected = []
    for index in range(len(data)):
        collected.extend(parser.feed(data[index : index + 1]))
    assert len(collected) == 1
    assert collected[0].data == bytes([int(BusServoCommand.READ_TEMP), 1])


def test_parser_tolerates_firmware_trailing_junk_byte():
    """packet.c 的 transmit() 多写 1 个字节，解析器必须能吞掉它并继续正常工作。"""
    good = build_bus_servo_frame(BusServoCommand.READ_POSITION, [6])
    stream = good + b"\xA5" + good  # 帧后垃圾字节
    parser = FrameParser()
    frames = parser.feed(stream)
    assert len(frames) == 2
    assert parser.checksum_errors == 0
    assert parser.discarded_bytes == 1


def test_parser_ignores_leading_noise():
    parser = FrameParser()
    frames = parser.feed(b"\x00\x11\xAA\x99" + build_bus_servo_frame(BusServoCommand.READ_VIN, [2]))
    assert len(frames) == 1
    assert parser.discarded_bytes >= 3


def test_parser_drops_frame_with_bad_checksum():
    frame = bytearray(build_bus_servo_frame(BusServoCommand.READ_ID, [4]))
    frame[-1] ^= 0xFF
    parser = FrameParser()
    assert parser.feed(bytes(frame)) == []
    assert parser.checksum_errors == 1
    assert parser.frames_parsed == 0


def test_parser_rejects_unknown_function_number():
    """固件要求功能号 < PACKET_FUNC_NONE(14)，>=14 直接丢帧。"""
    parser = FrameParser()
    assert parser.feed(build_frame(0x05, b"\x05\x06"))  # 正常帧能解析
    bad = bytes([START_BYTE_1, START_BYTE_2, 0x14, 0x01, 0x05, 0x00])
    assert parser.feed(bad) == []


def test_parser_supports_zero_length_frame():
    frame = build_frame(PacketFunction.SYS, b"")
    parser = FrameParser()
    frames = parser.feed(frame)
    assert len(frames) == 1
    assert frames[0].data == b""


def test_parser_splits_multiple_frames_in_one_chunk():
    stream = b"".join(build_bus_servo_frame(BusServoCommand.READ_ID, [sid]) for sid in range(1, 7))
    parser = FrameParser()
    assert len(parser.feed(stream)) == 6
    assert parser.frames_parsed == 6


def test_stray_start_byte_swallows_the_next_frame_like_firmware():
    """照抄固件状态机就继承了它的缺陷：一个多余的 0xAA 会把紧随其后的真帧整帧吞掉，
    要到再下一帧才恢复。所以桥接层超时后必须清空接收缓冲并重发，不能只等一次。"""
    good = build_bus_servo_frame(BusServoCommand.READ_ID, [1])
    parser = FrameParser()
    assert parser.feed(b"\xAA" + good) == []
    assert len(parser.feed(good)) == 1


def test_non_aa_noise_does_not_disturb_framing():
    good = build_bus_servo_frame(BusServoCommand.READ_ID, [1])
    parser = FrameParser()
    assert len(parser.feed(b"\x00\x7F\x12" + good)) == 1


def test_decode_report_position_is_signed_int16():
    """位置回读是小端有符号 16 位，-100 应该解成负数而不是 65436。"""
    parser = FrameParser()
    payload = bytes([0x06, 0x05, 0x00]) + (-100).to_bytes(2, "little", signed=True)
    frame = build_frame(PacketFunction.BUS_SERVO, payload)
    report = decode_bus_servo_report(parser.feed(frame)[0])
    assert report.servo_id == 6
    assert report.sub_command == int(BusServoCommand.READ_POSITION)
    assert report.error_code == 0
    assert report.answered is True
    assert report.int16 == -100


def test_report_error_code_zero_means_success():
    """固件把 read_xxx() 的返回值(0=成功/-1=无应答)原样写入该字节，
    所以 0x00 才是成功，0xFF 是无应答。语义与普通 'success' 字段相反。"""
    parser = FrameParser()
    ok = parser.feed(build_frame(PacketFunction.BUS_SERVO, bytes([0x01, 0x05, 0x00, 0xF4, 0x01])))[0]
    fail = parser.feed(build_frame(PacketFunction.BUS_SERVO, bytes([0x01, 0x05, 0xFF, 0x00, 0x00])))[0]
    assert decode_bus_servo_report(ok).answered is True
    assert decode_bus_servo_report(fail).answered is False


def test_decode_report_rejects_wrong_function_and_short_payload():
    parser = FrameParser()
    led_frame = parser.feed(build_frame(PacketFunction.LED, b"\x01\x02"))[0]
    with pytest.raises(ValueError):
        decode_bus_servo_report(led_frame)
    short = parser.feed(build_frame(PacketFunction.BUS_SERVO, b"\x05\x06"))[0]
    with pytest.raises(ValueError):
        decode_bus_servo_report(short)


@pytest.mark.parametrize("command", list(BusServoCommand))
def test_read_only_gate_allows_exactly_the_read_commands(command):
    frame = build_bus_servo_frame(command, [1])
    if command in {
        BusServoCommand.READ_POSITION,
        BusServoCommand.READ_VIN,
        BusServoCommand.READ_TEMP,
        BusServoCommand.READ_LOAD_STATE,
        BusServoCommand.READ_ID,
        BusServoCommand.READ_DEVIATION,
        BusServoCommand.READ_ANGLE_LIMIT,
        BusServoCommand.READ_VIN_LIMIT,
        BusServoCommand.READ_TEMP_LIMIT,
    }:
        parsed = assert_read_only_frame(frame)
        assert parsed.function == int(PacketFunction.BUS_SERVO)
    else:
        with pytest.raises(ValueError):
            assert_read_only_frame(frame)


@pytest.mark.parametrize(
    "command",
    [
        BusServoCommand.SET_POSITION,
        BusServoCommand.STOP,
        BusServoCommand.LOAD,
        BusServoCommand.UNLOAD,
        BusServoCommand.WRITE_ID,
        BusServoCommand.SET_DEVIATION,
        BusServoCommand.SAVE_DEVIATION,
        BusServoCommand.SET_ANGLE_LIMIT,
        BusServoCommand.SET_VIN_LIMIT,
        BusServoCommand.SET_TEMP_LIMIT,
    ],
)
def test_read_only_gate_blocks_every_command_that_can_change_state(command):
    with pytest.raises(ValueError):
        assert_read_only_frame(build_bus_servo_frame(command, [1, 2, 3]))


def test_read_only_gate_blocks_non_servo_functions():
    with pytest.raises(ValueError):
        assert_read_only_frame(build_frame(PacketFunction.BUZZER, b"\x01"))


def test_read_only_gate_blocks_malformed_frames():
    with pytest.raises(ValueError):
        assert_read_only_frame(b"\xAA\x55\x05")
    good = build_bus_servo_frame(BusServoCommand.READ_ID, [1])
    with pytest.raises(ValueError):
        assert_read_only_frame(good + good)

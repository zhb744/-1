"""传输层测试：位置帧的字节布局、限位校验，以及 SerialLink 在假端口上的读写行为。"""

import pytest

from armcomm.link import SerialLink, exchange, reply_matches, set_position_frame, stop_frame
from armcomm.protocol import BusServoCommand, FrameParser, PacketFunction, build_frame, crc8

# 与固件规则离线算出的黄金帧：舵机 6，1000 ms，位置 500
GOLDEN_SET_POSITION = bytes.fromhex("AA55050701E8030106F4018F")


class FakePort:
    """假串口。

    ``replies`` 是预先塞进接收缓冲的字节（用来验证"发请求前会清空残留"）；
    ``responder`` 在每次 write 后生成回包 —— 因为 ``exchange()`` 会先 flush 接收缓冲，
    只有写在请求之后产生的回包才可能被读到，这正是修掉错位回包的关键行为。
    """

    def __init__(self, replies=(), responder=None):
        self.queue = bytearray()
        for reply in replies:
            self.queue.extend(reply)
        self.responder = responder
        self.writes = []
        self.flush_count = 0

    def _reply_for_request(self, frame):
        data = FrameParser().feed(frame)
        if not data:
            return b""
        sub_command, servo_id = data[0].data[0], data[0].data[1] if len(data[0].data) > 1 else 0
        return self.responder(sub_command, servo_id)

    def write(self, data):
        frame = bytes(data)
        self.writes.append(frame)
        if self.responder is not None:
            self.queue.extend(self._reply_for_request(frame))
            self.queue.extend(b"\x7F")  # 固件 transmit() 多写的那 1 个字节
        return len(frame)

    def read(self, size=1):
        if not self.queue:
            return b""
        chunk = bytes(self.queue[:size])
        del self.queue[:size]
        return chunk

    def reset_input_buffer(self):
        self.flush_count += 1
        self.queue = bytearray()

    def reset_output_buffer(self):
        pass


def report_bytes(servo_id, sub_command, error_code=0, args=b""):
    return build_frame(PacketFunction.BUS_SERVO, bytes([servo_id, int(sub_command), error_code]) + bytes(args))


def position_responder(value_bytes=b"\x00\x00"):
    def responder(sub_command, servo_id):
        return report_bytes(servo_id, BusServoCommand.READ_POSITION, 0, value_bytes)

    return responder


# ---------------------------------------------------------------- 位置帧布局


def test_set_position_frame_matches_the_firmware_golden_frame():
    assert set_position_frame({6: 500}, 1000) == GOLDEN_SET_POSITION


def test_set_position_frame_layout_for_multiple_servos():
    frame = set_position_frame({6: 500, 1: 700}, 1500)
    parser = FrameParser()
    parsed = parser.feed(frame)
    assert len(parsed) == 1 and parser.checksum_errors == 0
    data = parsed[0].data
    assert data[0] == int(BusServoCommand.SET_POSITION)
    assert int.from_bytes(data[1:3], "little") == 1500
    assert data[3] == 2
    # 舵机按 ID 升序排列，位置是有符号小端 16 位
    assert data[4] == 1 and int.from_bytes(data[5:7], "little", signed=True) == 700
    assert data[7] == 6 and int.from_bytes(data[8:10], "little", signed=True) == 500


def test_set_position_frame_checksum_covers_function_length_data():
    frame = set_position_frame({3: 110}, 800)
    assert crc8(frame[2:-1]) == frame[-1]


@pytest.mark.parametrize(
    "targets, message",
    [
        ({1: 900}, "700"),  # 夹爪超固件上限
        ({6: 1001}, "越界"),
        ({6: -1}, "越界"),
        ({7: 500}, "不在 1~6"),
    ],
)
def test_set_position_frame_rejects_illegal_targets(targets, message):
    with pytest.raises(ValueError) as error:
        set_position_frame(targets)
    assert message in str(error.value)


def test_set_position_frame_rejects_bad_duration_and_empty():
    with pytest.raises(ValueError):
        set_position_frame({6: 500}, 40000)
    with pytest.raises(ValueError):
        set_position_frame({})


def test_stop_frame_lists_every_servo():
    frame = stop_frame([6, 1, 3])
    data = FrameParser().feed(frame)[0].data
    assert data[0] == int(BusServoCommand.STOP)
    assert data[1] == 3
    assert list(data[2:]) == [1, 3, 6]
    with pytest.raises(ValueError):
        stop_frame([])
    with pytest.raises(ValueError):
        stop_frame([7])


# ---------------------------------------------------------------- SerialLink


def make_link(port):
    link = SerialLink("COM-FAKE", timeout_s=0.02)
    link._serial = port  # 注入假端口，绕开 pyserial
    return link


def test_link_read_servo_returns_decoded_report():
    port = FakePort(responder=position_responder(b"\xf4\x01"))
    link = make_link(port)
    report = link.read_servo(BusServoCommand.READ_POSITION, 6)
    assert report is not None
    assert report.answered is True
    assert report.int16 == 500
    assert port.flush_count == 1  # 每条请求前都先清空接收缓冲


def test_link_read_servo_returns_none_when_board_stops_answering():
    port = FakePort(responder=lambda sub, sid: b"")
    link = make_link(port)
    assert link.read_servo(BusServoCommand.READ_ID, 4) is None


def test_link_ignores_reply_for_another_servo():
    port = FakePort(responder=lambda sub, sid: report_bytes(3, sub, 0, b"\x64\x00"))
    link = make_link(port)
    assert link.read_servo(BusServoCommand.READ_POSITION, 5) is None
    assert link.skipped_frames == 1


def test_link_read_positions_only_returns_answered_servos():
    def responder(sub_command, servo_id):
        if servo_id == 2:
            return report_bytes(servo_id, sub_command, 0xFF)
        value = {1: b"\x3c\x01", 3: b"\xe7\xff"}[servo_id]  # 316 与 -25
        return report_bytes(servo_id, sub_command, 0, value)

    link = make_link(FakePort(responder=responder))
    assert link.read_positions([1, 2, 3]) == {1: 316, 3: -25}


def test_link_write_positions_sends_one_synchronised_frame():
    port = FakePort(responder=lambda sub, sid: b"")
    link = make_link(port)
    frame = link.write_positions({6: 500, 5: 600}, duration_ms=1200)
    assert port.writes == [frame]
    assert len(frame) == len(GOLDEN_SET_POSITION) + 3  # 多带一个舵机 = 多 id + 2 字节位置
    data = FrameParser().feed(frame)[0].data
    assert int.from_bytes(data[1:3], "little") == 1200
    assert data[3] == 2


def test_link_write_positions_refuses_before_touching_the_port():
    """限位不合法时必须在写串口之前就抛错，端口一次都不能被写。"""
    port = FakePort(responder=lambda sub, sid: b"")
    link = make_link(port)
    with pytest.raises(ValueError):
        link.write_positions({1: 999})
    assert port.writes == []


def test_link_write_stop_is_available_for_safety():
    port = FakePort(responder=lambda sub, sid: b"")
    link = make_link(port)
    frame = link.write_stop(range(1, 7))
    data = FrameParser().feed(frame)[0].data
    assert data[0] == int(BusServoCommand.STOP)
    assert data[1] == 6


def test_link_requires_open_port():
    link = SerialLink("COM-FAKE")
    with pytest.raises(RuntimeError):
        link.read_servo(BusServoCommand.READ_ID, 1)
    with pytest.raises(RuntimeError):
        link.write_positions({6: 500})
    assert link.is_open is False


def test_link_stats_expose_desync_counters():
    port = FakePort(responder=lambda sub, sid: report_bytes(9, sub, 0, b"\x09"))
    link = make_link(port)
    link.read_servo(BusServoCommand.READ_ID, 1)
    stats = link.stats
    assert stats["skipped"] == 1
    assert stats["frames"] == 1
    assert stats["checksum_errors"] == 0


def test_reply_matches_needs_all_three_fields():
    parser = FrameParser()
    frame = parser.feed(report_bytes(4, BusServoCommand.READ_ID))[0]
    assert reply_matches(frame, BusServoCommand.READ_ID, 4) is True
    assert reply_matches(frame, BusServoCommand.READ_TEMP, 4) is False
    assert reply_matches(frame, BusServoCommand.READ_ID, 5) is False
    assert reply_matches(frame, None, None) is True


def test_exchange_flushes_before_writing():
    port = FakePort([b"\xAA\x55\x99"])  # 上一次的残留
    parser = FrameParser()
    reply, skipped = exchange(port, parser, GOLDEN_SET_POSITION, 0.01, BusServoCommand.READ_ID, 1)
    assert reply is None
    assert skipped == 0  # 残留已被清空，不会被误当成回包
    assert port.queue == bytearray()

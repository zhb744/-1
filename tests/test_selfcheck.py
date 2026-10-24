"""自检脚本测试。

用独立的黄金回包帧（按固件 CRC 规则离线算出）验证解析，用假串口验证分类逻辑，
并锁死一条安全红线：自检在总线上**绝不会发出运动类指令**。

黄金回包帧：

    AA 55 05 05 06 05 00 F4 01 C3     # 舵机 6 位置回读 = 500，error_code=0（成功）
    AA 55 05 05 06 05 FF 00 00 ED     # 舵机 6 无应答
    AA 55 05 05 06 12 00 06 00 2B     # 舵机 6 的 ID 回读 = 6
"""

import importlib.util
import sys
from pathlib import Path

import pytest

from armcomm.protocol import (
    BusServoCommand,
    FrameParser,
    PacketFunction,
    READ_ONLY_BUS_SERVO_COMMANDS,
    build_bus_servo_frame,
    build_frame,
)

REPO_ROOT = Path(__file__).resolve().parent.parent

_spec = importlib.util.spec_from_file_location("selfcheck", REPO_ROOT / "tools" / "selfcheck.py")
selfcheck = importlib.util.module_from_spec(_spec)
# 必须先注册进 sys.modules，否则模块内的 @dataclass 解析不到所属模块。
sys.modules["selfcheck"] = selfcheck
_spec.loader.exec_module(selfcheck)

GOLDEN_POSITION_OK = bytes.fromhex("AA550505060500F401C3")
GOLDEN_SERVO_ABSENT = bytes.fromhex("AA55050506 05FF0000ED".replace(" ", ""))
GOLDEN_ID_OK = bytes.fromhex("AA55050506120006002B")

#: 舵机位置类子命令的回包参数长度
_ARGS_BY_COMMAND = {
    int(BusServoCommand.READ_POSITION): (0xF4, 0x01),  # 500
    int(BusServoCommand.READ_VIN): (0xB8, 0x04),
    int(BusServoCommand.READ_TEMP): (0x2A,),
    int(BusServoCommand.READ_LOAD_STATE): (0x01,),
    int(BusServoCommand.READ_ID): (0x06,),
}


def make_report(sub_command: int, servo_id: int, error_code: int = 0) -> bytes:
    args = _ARGS_BY_COMMAND.get(int(sub_command), (0x00,))
    if error_code:
        args = (0x00, 0x00)
    return build_frame(PacketFunction.BUS_SERVO, bytes([servo_id, int(sub_command), error_code]) + bytes(args))


class FakePort:
    """假串口。write 时按 responder 生成回包字节；reset_input_buffer 真的清空缓冲，
    和 Windows 上的行为一致 —— 这一点正是修掉错位回包的关键。"""

    def __init__(self, responder=None, junk=b"\x7F"):
        self.responder = responder
        self.junk = junk
        self.queue = bytearray()
        self.writes = []

    def write(self, data):
        frame = bytes(data)
        self.writes.append(frame)
        parser = FrameParser()
        parsed = parser.feed(frame)
        if not parsed:
            return len(frame)
        sub_command = parsed[0].data[0]
        servo_id = parsed[0].data[1] if len(parsed[0].data) > 1 else None
        if self.responder is not None:
            self.queue.extend(self.responder(sub_command, servo_id))
        else:
            self.queue.extend(make_report(sub_command, servo_id))
        self.queue.extend(self.junk)  # 固件 transmit() 多写的那 1 个字节
        return len(frame)

    def read(self, size=1):
        if not self.queue:
            return b""
        chunk = bytes(self.queue[:size])
        del self.queue[:size]
        return chunk

    def reset_input_buffer(self):
        self.queue = bytearray()

    def reset_output_buffer(self):
        pass


def test_exchange_returns_only_the_matching_reply():
    port = FakePort()
    parser = FrameParser()
    reply, skipped = selfcheck.exchange(
        port, parser, build_bus_servo_frame(BusServoCommand.READ_POSITION, [6]), 0.05,
        BusServoCommand.READ_POSITION, 6,
    )
    assert reply is not None
    assert skipped == 0
    assert reply.data == make_report(int(BusServoCommand.READ_POSITION), 6)[4:-1]


def test_exchange_parses_golden_reply_frame():
    """用离线算好的字节验证，不依赖本模块的编码器。"""

    def responder(sub_command, servo_id):
        return GOLDEN_POSITION_OK + b"\x7F"

    port = FakePort(responder=responder)
    parser = FrameParser()
    reply, _ = selfcheck.exchange(
        port, parser, build_bus_servo_frame(BusServoCommand.READ_POSITION, [6]), 0.05,
        BusServoCommand.READ_POSITION, 6,
    )
    assert reply is not None
    assert int.from_bytes(reply.data[3:5], "little", signed=True) == 500


def test_exchange_skips_a_misaligned_reply_and_times_out():
    """回包自称是别的舵机 → 必须跳过并计入杂帧，不能被当成有效读数。"""

    def responder(sub_command, servo_id):
        return make_report(sub_command, 3) + b"\x7F"

    port = FakePort(responder=responder)
    parser = FrameParser()
    reply, skipped = selfcheck.exchange(
        port, parser, build_bus_servo_frame(BusServoCommand.READ_ID, [4]), 0.01,
        BusServoCommand.READ_ID, 4,
    )
    assert reply is None
    assert skipped == 1


def test_exchange_flushes_stale_bytes_before_each_request():
    """上一条请求残留的回包不能污染下一条 —— 这正是实机上"错位一帧"的修法。"""
    port = FakePort()
    parser = FrameParser()
    # 先制造残留：往缓冲里塞一条属于上一次请求的回包
    port.queue.extend(make_report(int(BusServoCommand.READ_ID), 9) + b"\xAA")
    reply, skipped = selfcheck.exchange(
        port, parser, build_bus_servo_frame(BusServoCommand.READ_TEMP, [2]), 0.05,
        BusServoCommand.READ_TEMP, 2,
    )
    assert reply is not None
    assert skipped == 0


def test_junk_byte_equal_to_aa_does_not_break_framing():
    """垃圾字节恰好是 0xAA 时（实机出现过），清空缓冲 + resync 之后必须还能正常解析。"""
    port = FakePort(junk=b"\xAA")
    parser = FrameParser()
    for servo_id in (1, 2, 3):
        reply, skipped = selfcheck.exchange(
            port, parser, build_bus_servo_frame(BusServoCommand.READ_POSITION, [servo_id]), 0.05,
            BusServoCommand.READ_POSITION, servo_id,
        )
        assert reply is not None, f"舵机 {servo_id} 因帧头被吞而失联"
        assert skipped == 0


def test_exchange_returns_none_on_silence():
    port = FakePort(responder=lambda sub, sid: b"")
    parser = FrameParser()
    reply, skipped = selfcheck.exchange(
        port, parser, build_bus_servo_frame(BusServoCommand.READ_ID, [1]), 0.01,
        BusServoCommand.READ_ID, 1,
    )
    assert reply is None
    assert skipped == 0


def test_probe_servos_reports_ok_rows():
    port = FakePort()
    results, any_reply, parser, skipped = selfcheck.probe_servos(port, [6], 0.05, verbose=False)
    assert any_reply is True
    assert skipped == 0
    assert len(results) == len(selfcheck.PROBES)
    assert all(r.status == "OK" for r in results), [r.status for r in results]
    assert parser.frames_parsed == len(selfcheck.PROBES)


def test_probe_servos_classifies_servo_absent():
    port = FakePort(responder=lambda sub, sid: make_report(sub, sid, error_code=0xFF) + b"\x7F")
    results, any_reply, _, _ = selfcheck.probe_servos(port, [6], 0.05, verbose=False)
    assert any_reply is True
    assert all(r.status == "舵机无应答" for r in results)


def test_probe_servos_classifies_board_silence():
    port = FakePort(responder=lambda sub, sid: b"")
    results, any_reply, _, skipped = selfcheck.probe_servos(port, [6], 0.01, verbose=False)
    assert any_reply is False
    assert skipped == 0
    assert all(r.status == "无回包" for r in results)


def test_probe_reports_misaligned_replies_instead_of_trusting_them():
    port = FakePort(responder=lambda sub, sid: make_report(sub, 1) + b"\x7F")
    results, any_reply, _, skipped = selfcheck.probe_servos(port, [2], 0.01, verbose=False)
    assert skipped == len(selfcheck.PROBES)
    assert any_reply is True
    assert all(r.status == "无回包" for r in results)
    assert all("杂帧" in r.detail for r in results)


def test_probe_never_transmits_a_non_read_only_frame():
    """安全红线：探测期间写出的每一帧都必须是只读子命令。

    SET_POSITION / STOP / LOAD / UNLOAD / WRITE_ID 只要出现在这里，就意味着"只读自检"
    有可能让机械臂动起来 —— 绝对不允许。
    """
    port = FakePort(responder=lambda sub, sid: b"")
    selfcheck.probe_servos(port, list(selfcheck.DEFAULT_SERVO_IDS), 0.005, verbose=False)
    assert port.writes
    parser = FrameParser()
    for frame in port.writes:
        frames = parser.feed(frame)
        assert len(frames) == 1
        assert frames[0].function == int(PacketFunction.BUS_SERVO)
        command = frames[0].data[0]
        assert command in READ_ONLY_BUS_SERVO_COMMANDS, f"自检发出了会动的指令 0x{command:02X}"


def test_probe_only_asks_for_the_requested_servos():
    port = FakePort(responder=lambda sub, sid: b"")
    selfcheck.probe_servos(port, [2, 5], 0.005, verbose=False)
    asked = {frame[5] for frame in port.writes}  # data 段第 2 字节 = 舵机 ID
    assert asked == {2, 5}


@pytest.mark.parametrize(
    "spec, expected",
    [("1-6", (1, 2, 3, 4, 5, 6)), ("1,3,5", (1, 3, 5)), ("4", (4,)), ("6-1", (1, 2, 3, 4, 5, 6))],
)
def test_parse_servo_ids(spec, expected):
    assert selfcheck.parse_servo_ids(spec) == expected


def _rows(status):
    command, label, _ = selfcheck.PROBES[0]
    return [selfcheck.ProbeResult(6, command, label, status, "")]


def test_print_results_exit_codes(capsys):
    parser = FrameParser()
    parser.feed(GOLDEN_POSITION_OK)
    capsys.readouterr()

    assert selfcheck.print_results(_rows("OK"), True, parser, 0, "COM4", 1_000_000) == 0
    assert "三段全通" in capsys.readouterr().out

    assert selfcheck.print_results(_rows("舵机无应答"), True, parser, 0, "COM4", 1_000_000) == 2
    text = capsys.readouterr().out
    assert "舵机供电" in text
    assert "三段全通" not in text

    assert selfcheck.print_results(_rows("无回包"), False, FrameParser(), 0, "COM4", 1_000_000) == 3
    assert "波特率是不是" in capsys.readouterr().out

    noisy = FrameParser()
    noisy.bytes_consumed = 400
    noisy.discarded_bytes = 400
    assert selfcheck.print_results(_rows("无回包"), False, noisy, 0, "COM4", 1_000_000) == 3
    assert "凑不出任何合法帧" in capsys.readouterr().out


def test_print_results_surfaces_skipped_frames(capsys):
    parser = FrameParser()
    capsys.readouterr()
    selfcheck.print_results(_rows("无回包"), False, parser, 7, "COM4", 1_000_000)
    assert "跳过错位回包 7 帧" in capsys.readouterr().out


def test_default_baud_is_one_megabit():
    """USART1 在固件里是 1,000,000，不是 115200；写错就永远不通。"""
    assert selfcheck.DEFAULT_BAUD == 1_000_000


def test_reply_matches_requires_function_sub_command_and_servo_id():
    parser = FrameParser()
    frame = parser.feed(make_report(int(BusServoCommand.READ_ID), 4))[0]
    assert selfcheck.reply_matches(frame, int(BusServoCommand.READ_ID), 4) is True
    assert selfcheck.reply_matches(frame, int(BusServoCommand.READ_TEMP), 4) is False
    assert selfcheck.reply_matches(frame, int(BusServoCommand.READ_ID), 5) is False
    assert selfcheck.reply_matches(frame, None, None) is True


def test_load_port_config_falls_back_when_file_missing(tmp_path):
    assert selfcheck.load_port_config(tmp_path / "nope.json") == {}


def test_load_port_config_reads_serial_section(tmp_path):
    path = tmp_path / "config.local.json"
    path.write_text('{"serial": {"port": "COM4", "baudrate": 1000000}}', encoding="utf-8")
    config = selfcheck.load_port_config(path)
    assert config["port"] == "COM4"
    assert int(config["baudrate"]) == 1_000_000


def test_main_reports_missing_port_gracefully(capsys):
    """打不开的端口要返回明确错误码，而不是抛栈。"""
    code = selfcheck.main(["--port", "COM999", "--servos", "1", "--timeout", "0.01"])
    assert code == 4
    assert "端口打不开" in capsys.readouterr().out


def test_golden_id_reply_decodes_to_servo_six():
    parser = FrameParser()
    frame = parser.feed(GOLDEN_ID_OK)[0]
    assert frame.data[1] == int(BusServoCommand.READ_ID)
    assert frame.data[3] == 6

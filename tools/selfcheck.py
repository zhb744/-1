"""串口链路自检 —— 只读，不会让任何关节动一下。

用途：验证 Windows ↔ STM32 USART1 @ 1 Mbps 这条链路是否真的通，以及 6 个总线舵机
是否在线应答。它把三种完全不同的故障分开了：

* **端口打不开** —— COM 号错、驱动没装、或被别的程序（官方上位机/串口助手）占用；
  Windows 串口是独占的，同时只能有一个使用者。
* **端口能开，但板子一个字节都不回** —— 波特率不对（本板是 1,000,000，不是常见的
  115200）、USB 转串口芯片跑不动 1 Mbps、或固件没在跑。
* **板子回包了，但 error_code != 0** —— 上位机链路完全正常，问题在板内那条
  115200 的舵机总线：舵机没供电、BUS_EN 方向线、或总线接线。

安全闸门：每一条要发出的帧都先过 ``assert_read_only_frame()``，任何运动类子命令
（``SET_POSITION``/``STOP``/``LOAD``/``UNLOAD``/``WRITE_ID`` 等）会直接抛异常，
自检脚本从代码层面发不出让臂动的指令。

用法::

    python tools/selfcheck.py --list          # 看有哪些 COM 口
    python tools/selfcheck.py                 # 用 config.local.json 里的端口
    python tools/selfcheck.py --port COM9     # 指定端口
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from armcomm.protocol import (  # noqa: E402
    BusServoCommand,
    FrameParser,
    PacketFunction,
    ParsedFrame,
    assert_read_only_frame,
    build_bus_servo_frame,
    decode_bus_servo_report,
    hexs,
)
from armcomm.servos import JOINTS, position_to_angle  # noqa: E402

DEFAULT_BAUD = 1_000_000  # 固件里 USART1 就是 1 Mbps，不是 115200
DEFAULT_PORT_NAME = "COM9"
DEFAULT_SERVO_IDS = (1, 2, 3, 4, 5, 6)

#: (子命令, 表头, 解码函数) —— 全部来自 READ_ONLY_BUS_SERVO_COMMANDS
PROBES = (
    (BusServoCommand.READ_ID, "ID", lambda r: str(r.uint8)),
    (BusServoCommand.READ_POSITION, "位置/角度", lambda r: f"{r.int16} / {position_to_angle(r.int16):+.1f}°"),
    (BusServoCommand.READ_VIN, "电压原始值", lambda r: f"{r.uint16}（单位待实机标定）"),
    (BusServoCommand.READ_TEMP, "温度", lambda r: f"{r.uint8} °C"),
    (BusServoCommand.READ_LOAD_STATE, "加载状态", lambda r: ("加载" if r.uint8 else "卸力")),
)


@dataclass
class ProbeResult:
    servo_id: int
    command: BusServoCommand
    label: str
    status: str
    detail: str


def load_port_config(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open(encoding="utf-8") as handle:
        config = json.load(handle)
    return config.get("serial", {})


def reply_matches(reply: ParsedFrame, sub_command: int | None, servo_id: int | None) -> bool:
    """只承认"功能号 5、子命令与舵机 ID 都对得上"的回包。

    这条校验是实机逼出来的：固件回包尾部的垃圾字节偶尔等于 0xAA，会吞掉下一帧帧头，
    让后续所有回包整体错位一帧。不按子命令和 ID 核对，就会把上一个舵机的读数当成
    当前舵机的，表现就是"请求 4 号 ID，回包自称 3 号"。
    """
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


def probe_servos(port, servo_ids, timeout_s: float, verbose: bool) -> tuple:
    """返回 (结果列表, 板子是否有任何应答, 解析器统计, 跳过的杂帧总数)。"""
    parser = FrameParser()
    results = []
    any_reply = False
    skipped_total = 0

    for servo_id in servo_ids:
        joint = JOINTS.get(servo_id)
        name = joint.name if joint else f"ID {servo_id}"
        if verbose:
            print(f"探测 舵机 {servo_id}（{name}）…")

        for command, label, decode in PROBES:
            frame = build_bus_servo_frame(command, [servo_id])
            assert_read_only_frame(frame)  # 安全闸门，非只读会在这里抛异常
            reply, skipped = exchange(port, parser, frame, timeout_s, command, servo_id)
            skipped_total += skipped

            if reply is None:
                if skipped:
                    any_reply = True
                    results.append(ProbeResult(servo_id, command, label, "无回包", f"超时，另丢弃 {skipped} 个杂帧"))
                else:
                    results.append(ProbeResult(servo_id, command, label, "无回包", "板子未应答"))
                continue

            any_reply = True
            report = decode_bus_servo_report(reply)
            if not report.answered:
                results.append(
                    ProbeResult(
                        servo_id,
                        command,
                        label,
                        "舵机无应答",
                        f"error_code=0x{report.error_code:02X}" + (f"，另丢弃 {skipped} 杂帧" if skipped else ""),
                    )
                )
                continue

            results.append(ProbeResult(servo_id, command, label, "OK", decode(report)))

    return results, any_reply, parser, skipped_total


def print_results(results, any_reply: bool, parser: FrameParser, skipped_total: int, port_name: str, baud: int) -> int:
    width = max([len(r.label) for r in results] + [12])
    print()
    print(f"端口 {port_name} @ {baud} bps  共 {len(results)} 次探测")
    print(
        f"解析统计：收到 {parser.bytes_consumed} 字节，解析出 {parser.frames_parsed} 帧，"
        f"丢弃 {parser.discarded_bytes} 个非帧字节，CRC 失败 {parser.checksum_errors} 次，"
        f"跳过错位回包 {skipped_total} 帧"
    )
    print("-" * (60 + width))
    print(f"{'舵机':<6}{'项目':<{width + 4}}{'结果':<12}说明")
    print("-" * (60 + width))
    for r in results:
        print(f"{r.servo_id:<6}{r.label:<{width + 4}}{r.status:<12}{r.detail}")
    print("-" * (60 + width))

    ok = sum(1 for r in results if r.status == "OK")
    no_servo = sum(1 for r in results if r.status == "舵机无应答")
    no_board = sum(1 for r in results if r.status == "无回包")

    print(f"合计：OK {ok} / 舵机无应答 {no_servo} / 板子无回包 {no_board}")
    print()

    if ok:
        print("结论：上位机 ↔ STM32 ↔ 舵机总线 三段全通，可以进入控制层开发。")
        return 0
    if any_reply:
        print(
            "结论：上位机 ↔ STM32 已通（板子有回包），但舵机总线上没有任何应答。\n"
            "      下一步查舵机供电（必须独立电源且与板子共地）、总线接线、BUS_EN 方向。\n"
            "      注意：这一步已经证明 1 Mbps 链路和协议解析都是对的。"
        )
        return 2
    if parser.bytes_consumed:
        print(
            f"结论：收到 {parser.bytes_consumed} 个字节，但凑不出任何合法帧 —— 这是波特率不匹配的\n"
            f"      典型特征（丢弃 {parser.discarded_bytes} 字节、CRC 失败 {parser.checksum_errors} 次）。\n"
            f"      本板 USART1 是 {DEFAULT_BAUD} bps 8N1，先确认 USB 转串口芯片支持 1 Mbps，\n"
            f"      再确认没有用 USART2（PA2/PA3，舵机总线，115200）去接上位机。"
        )
        return 3
    print(
        "结论：端口能打开但板子一个字节都没回。按顺序排：\n"
        f"      1) 波特率是不是 {baud}（本板 USART1 = 1 Mbps，不是 115200）；\n"
        "      2) USB 转串口芯片能不能跑 1 Mbps（CH340/CH9102 可以，老 CP2102/PL2303 常失败）；\n"
        "      3) 这根线对应的是不是 USART1（PA9/PA10），而不是 USART2 舵机总线；\n"
        "      4) 板子有没有上电、固件有没有在跑（复位时机械臂会先做一次 1500ms 归位动作）。"
    )
    return 3


def list_ports() -> int:
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    if not ports:
        print("没发现任何串口设备。")
        return 1
    print(f"{'设备名':<10}{'描述'}")
    for item in sorted(ports, key=lambda p: p.device):
        print(f"{item.device:<10}{item.description}")
        if item.serial_number:
            print(f"{'':<10}序列号 {item.serial_number}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ArmPi Ultra 串口只读自检")
    parser.add_argument("--port", help="COM 口号，缺省读 config.local.json")
    parser.add_argument("--baud", type=int, default=None, help="波特率，缺省读配置或 1000000")
    parser.add_argument("--timeout", type=float, default=0.4, help="单次等待回包秒数，默认 0.4")
    parser.add_argument("--servos", default="1-6", help="舵机范围，如 1-6 或 1,3,5")
    parser.add_argument("--quiet", action="store_true", help="不打印逐条进度")
    parser.add_argument("--list", action="store_true", help="只列出现有串口")
    return parser


def parse_servo_ids(spec: str) -> tuple:
    ids = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            low, high = (int(x) for x in part.split("-", 1))
            if low > high:
                low, high = high, low
            ids.extend(range(low, high + 1))
        else:
            ids.append(int(part))
    return tuple(sorted(set(ids)))


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.list:
        return list_ports()

    serial_config = load_port_config(REPO_ROOT / "config.local.json")
    port_name = args.port or serial_config.get("port") or DEFAULT_PORT_NAME
    baud = args.baud or int(serial_config.get("baudrate", DEFAULT_BAUD))

    try:
        import serial
    except ImportError:
        print("缺少 pyserial，请先执行：python -m pip install pyserial")
        return 1

    print(f"打开 {port_name} @ {baud} bps 8N1 …")
    try:
        port = serial.Serial(port=port_name, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0.05)
    except Exception as error:  # noqa: BLE001 - 端口问题的种类很多，统一转成人话
        print(f"端口打不开：{type(error).__name__}: {error}")
        print(
            "常见原因：COM 号填错；CH340/CP210x 驱动未装；该串口被官方上位机或串口助手占用\n"
            "（Windows 串口独占，同一时刻只能有一个程序连接）。"
        )
        return 4

    try:
        port.reset_input_buffer()
        port.reset_output_buffer()
        results, any_reply, parser, skipped_total = probe_servos(
            port, parse_servo_ids(args.servos), args.timeout, not args.quiet
        )
    finally:
        port.close()

    return print_results(results, any_reply, parser, skipped_total, port_name, baud)


if __name__ == "__main__":
    raise SystemExit(main())

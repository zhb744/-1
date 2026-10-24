"""六轴轨迹队列：完整校验、新回读、限速、逐帧可取消执行。

串口锁仅覆盖单次读/写。状态锁不覆盖串口 I/O；所有双锁路径均先串口、后状态。
取消者先设置本次 run 的 Event，再持串口锁 STOP，最后释放 busy。旧线程只持有
旧 link/token，取消后即使延迟退出也不能发帧或覆盖下一次运行的状态。
completed 仅表示命令发送及计划等待结束，不表示回读确认到位。
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from uuid import uuid4

from .protocol import BusServoCommand
from .servos import (
    JOINTS, claw_position_to_angle, position_to_angle,
    roll_position_to_angle, servo_angle_to_position,
)

SERVO_IDS = tuple(sorted(JOINTS))
TRAJECTORY_LIMITS = {
    "max_points": 600,
    "min_duration_ms": 80,
    "max_duration_ms": 30000,
    "max_total_ms": 600000,
    "max_speed_deg_s": 30,
}
STANDBY_ANGLES = {"6": -90, "3": -90}


class TrajectoryBusyError(RuntimeError):
    """运动队列占用真机，HTTP 应返回 409。"""


class _Cancelled(Exception):
    pass


def validate_duration(duration_ms):
    if (type(duration_ms) is not int or
            not TRAJECTORY_LIMITS["min_duration_ms"] <= duration_ms <= TRAJECTORY_LIMITS["max_duration_ms"]):
        raise ValueError("duration_ms 必须是 80~30000 的整数")
    return duration_ms


def _valid_angle(sid, value):
    joint = JOINTS[sid]
    # 先检查范围，避免超大 JSON 整数在 isfinite/float 中溢出。
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and joint.min_angle <= value <= joint.max_angle and math.isfinite(value))


def validate_points(points):
    """严格拒绝整条非法路径，不夹紧、不丢点；返回独立副本。"""
    if not isinstance(points, list) or not 1 <= len(points) <= TRAJECTORY_LIMITS["max_points"]:
        raise ValueError("points 必须是非空数组，最多 600 点")
    result = []
    total = 0
    for index, point in enumerate(points, 1):
        if not isinstance(point, dict):
            raise ValueError(f"第 {index} 点必须是对象")
        angles = point.get("angles")
        if not isinstance(angles, dict) or set(angles) != {str(sid) for sid in SERVO_IDS}:
            raise ValueError(f"第 {index} 点 angles 必须且仅包含字符串 ID 1~6")
        for sid in SERVO_IDS:
            if not _valid_angle(sid, angles[str(sid)]):
                raise ValueError(f"第 {index} 点 ID {sid} 角度必须是关节限位内的有限数值")
        duration = validate_duration(point.get("duration_ms"))
        total += duration
        copied = {"angles": dict(angles), "duration_ms": duration}
        if "positions" in point:
            positions = point["positions"]
            if not isinstance(positions, dict) or set(positions) != {str(sid) for sid in SERVO_IDS}:
                raise ValueError(f"第 {index} 点 positions 必须且仅包含字符串 ID 1~6")
            for sid in SERVO_IDS:
                raw = positions[str(sid)]
                # 在浮点解码之前拒绝超大整数，不夹紧、不改变固件的 int 编码规则。
                if type(raw) is not int or not 0 <= raw <= 1000:
                    raise ValueError(f"第 {index} 点 ID {sid} 原始位置必须是合法整数")
                decoder = claw_position_to_angle if sid == 1 else roll_position_to_angle if sid == 2 else position_to_angle
                decoded = decoder(raw)
                if not _valid_angle(sid, decoded) or abs(decoded - angles[str(sid)]) > 1e-9:
                    raise ValueError(f"第 {index} 点 ID {sid} 原始位置越界或与角度不一致")
            copied["positions"] = dict(positions)
        result.append(copied)
    if total > TRAJECTORY_LIMITS["max_total_ms"]:
        raise ValueError("轨迹总时长不能超过 600000ms")
    return result


@dataclass
class _Run:
    link: object
    run_id: str = field(default_factory=lambda: uuid4().hex)
    cancel: threading.Event = field(default_factory=threading.Event)
    error: str | None = None


class TrajectoryRunner:
    def __init__(self, serial_lock):
        self._serial_lock = serial_lock
        self._state_lock = threading.Lock()
        self._active: _Run | None = None
        self._thread: threading.Thread | None = None
        self._wait = lambda cancel, seconds: cancel.wait(seconds)
        self._status = {"state": "idle", "index": 0, "total": 0, "error": None,
                        "run_id": None, "current_angles": {}}

    def status(self):
        with self._state_lock:
            return {**self._status, "current_angles": dict(self._status["current_angles"])}

    def require_idle(self):
        with self._state_lock:
            if self._active is not None:
                raise TrajectoryBusyError("轨迹执行中，只允许停止或断开联动")

    def start(self, link, points, standby_duration=None):
        """调用者持串口锁，且已完成输入校验及联动闸门检查。"""
        with self._state_lock:
            if self._active is not None:
                raise TrajectoryBusyError("轨迹执行中")
            run = _Run(link)
            self._active = run
            self._status = {"state": "preparing", "index": 0,
                            "total": 1 if standby_duration is not None else len(points),
                            "error": None, "run_id": run.run_id, "current_angles": {}}
            response = {**self._status, "current_angles": {}}
            self._thread = threading.Thread(
                target=self._execute, args=(run, points, standby_duration),
                name=f"trajectory-{run.run_id[:8]}", daemon=True,
            )
            try:
                self._thread.start()
            except Exception as error:
                self._status.update(state="error", error=f"{type(error).__name__}: {error}")
                self._active = None
                raise
            return response

    def cancel(self):
        """不等串口锁；busy 保留至取消者完成 STOP。"""
        with self._state_lock:
            run = self._active
            if run is not None:
                run.cancel.set()
            return run

    def finish_cancel(self, run, error=None):
        """取消者持串口锁，在 STOP（及可选 close）之后调用。"""
        with self._state_lock:
            if run is not None and self._active is run:
                message = error or run.error
                self._status.update(state="error" if message else "stopped", error=message)
                self._active = None

    def _check(self, run):
        with self._state_lock:
            if self._active is not run or run.cancel.is_set():
                raise _Cancelled

    def _read(self, run, command, sid, size):
        with self._serial_lock:
            self._check(run)
            report = run.link.read_servo(command, sid)
        self._check(run)
        if (report is None or not report.answered or report.servo_id != sid
                or report.sub_command != command or len(report.args) != size):
            raise ValueError(f"ID {sid} {command.name} 回读失败或数据不完整")
        return report

    def _prepare(self, run, points, standby_duration):
        actual, raw = {}, {}
        for sid in SERVO_IDS:
            position = self._read(run, BusServoCommand.READ_POSITION, sid, 2).int16
            decoder = claw_position_to_angle if sid == 1 else roll_position_to_angle if sid == 2 else position_to_angle
            angle = decoder(position)
            if not _valid_angle(sid, angle):
                raise ValueError(f"ID {sid} 回读位置 {position} 超出合法关节角域")
            loaded = self._read(run, BusServoCommand.READ_LOAD_STATE, sid, 1).uint8
            if loaded != 1:
                raise ValueError(f"ID {sid} 未加载或加载状态无效；不会自动加载")
            actual[str(sid)] = angle
            raw[sid] = position
        if standby_duration is not None:
            points = [{"angles": {**actual, **STANDBY_ANGLES}, "duration_ms": standby_duration}]
        plan, total = [], 0
        previous = raw
        decoders = {sid: claw_position_to_angle if sid == 1 else
                    roll_position_to_angle if sid == 2 else position_to_angle for sid in SERVO_IDS}
        for point in points:
            angles = point["angles"]
            positions = ({sid: point["positions"][str(sid)] for sid in SERVO_IDS} if "positions" in point else
                         {sid: servo_angle_to_position(sid, angles[str(sid)]) for sid in SERVO_IDS})
            if standby_duration is not None:
                # 保持轴沿用原始回读，避免角度往返截断造成 1 个位置单位的漂移。
                positions.update({sid: raw[sid] for sid in SERVO_IDS if str(sid) not in STANDBY_ANGLES})
            # 只保证编码计划的限速：从新回读 raw 起算，后续段也基于最终整数位置。
            delta = max(abs(decoders[sid](positions[sid]) - decoders[sid](previous[sid])) for sid in SERVO_IDS)
            duration = max(point["duration_ms"], math.ceil(delta / TRAJECTORY_LIMITS["max_speed_deg_s"] * 1000))
            total += duration
            if duration > TRAJECTORY_LIMITS["max_duration_ms"] or total > TRAJECTORY_LIMITS["max_total_ms"]:
                raise ValueError("限速后轨迹时长超限，未发送任何运动帧")
            plan.append((angles, positions, duration))
            previous = positions
        return plan

    def _execute(self, run, points, standby_duration):
        try:
            plan = self._prepare(run, points, standby_duration)
            for index, (angles, positions, duration) in enumerate(plan, 1):
                with self._serial_lock:
                    self._check(run)
                    run.link.write_positions(positions, duration)
                    with self._state_lock:
                        self._status.update(state="running", index=index, current_angles=dict(angles))
                if self._wait(run.cancel, duration / 1000):
                    raise _Cancelled
                self._check(run)
            with self._state_lock:
                if self._active is run and not run.cancel.is_set():
                    self._status["state"] = "completed"
                    self._active = None
        except _Cancelled:
            # STOP/断连调用者负责发布终态；不能提前清忙让新 run 抢在 STOP 前开始。
            pass
        except Exception as error:
            self._fail(run, error)

    def _fail(self, run, error):
        with self._serial_lock:
            with self._state_lock:
                if self._active is not run:
                    return
                run.error = f"{type(error).__name__}: {error}"
                if run.cancel.is_set():
                    return
            try:
                run.link.write_stop(SERVO_IDS)
            except Exception as stop_error:
                run.error += f"；STOP 失败：{stop_error}"
            with self._state_lock:
                if self._active is run and not run.cancel.is_set():
                    self._status.update(state="error", error=run.error)
                    self._active = None

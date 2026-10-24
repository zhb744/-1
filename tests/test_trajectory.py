"""轨迹安全测试：只用假串口及随机端口 HTTP，不连接真实设备。"""

import json
import threading
import urllib.error
import urllib.request

import pytest

from armcomm.protocol import BusServoCommand, BusServoReport
from tools.serve import build_backend, serve

IDS = (1, 2, 3, 4, 5, 6)


def point(duration=80, **angles):
    return {"angles": {str(sid): angles.get(str(sid), 0) for sid in IDS}, "duration_ms": duration}


class FakeLink:
    port = "FAKE-NO-HARDWARE"
    stats = {}

    def __init__(self):
        self.positions = {1: 700, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
        self.sent = []
        self.reads = []
        self.stops = []
        self.actions = []
        self.loaded = []
        self.closed = False
        self.read_hook = None
        self.write_error = False
        self.stop_error = False
        self.responses = {}

    def read_servo(self, command, sid):
        assert not self.closed, "旧线程不能读已关闭串口"
        self.reads.append((command, sid))
        if self.read_hook:
            self.read_hook(command, sid)
        if (command, sid) in self.responses:
            return self.responses[command, sid]
        args = self.positions[sid].to_bytes(2, "little", signed=True) if command == BusServoCommand.READ_POSITION else b"\x01"
        return BusServoReport(sid, command, 0, args)

    def write_positions(self, positions, duration_ms):
        assert not self.closed, "旧线程不能写已关闭串口"
        if self.write_error:
            raise OSError("write failed")
        self.sent.append((dict(positions), duration_ms))
        self.actions.append("move")

    def write_stop(self, ids):
        assert not self.closed
        self.stops.append(tuple(ids))
        self.actions.append("stop")
        if self.stop_error:
            raise OSError("stop failed")

    def close(self):
        self.closed = True
        self.actions.append("close")

    def set_loaded(self, loaded, ids):
        self.loaded.append((loaded, tuple(ids)))

    def read_snapshot(self, ids):
        raise AssertionError("轨迹不得使用不可取消的整批快照")


@pytest.fixture
def backend():
    backend = build_backend(None, {"port": "FAKE-NO-HARDWARE"})
    backend.link = FakeLink()
    backend.motion_enabled = True
    yield backend
    # 不假定失败的测试已创建轨迹模块。
    runner = getattr(backend, "trajectory", None)
    if runner:
        backend.close_link()
        if runner._thread:
            runner._thread.join(3)
            assert not runner._thread.is_alive()


def finish(backend):
    backend.trajectory._thread.join(3)
    assert not backend.trajectory._thread.is_alive(), "工作线程未退出"
    return backend.trajectory_status()


def instant(backend):
    backend.trajectory._wait = lambda cancel, seconds: cancel.is_set()


class Waiting:
    def __init__(self):
        self.entered = threading.Event()
        self.durations = []

    def __call__(self, cancel, seconds):
        self.durations.append(seconds)
        self.entered.set()
        assert cancel.wait(3), "测试必须取消等待"
        return True


INVALID_POINTS = [
    None, {}, "bad", [], [None], [1], [point()] * 601,
    [{"angles": {"1": 0}, "duration_ms": 80}],
    [{"angles": {**point()["angles"], "7": 0}, "duration_ms": 80}],
    [{"angles": {sid: 0 for sid in IDS}, "duration_ms": 80}],
    [{"angles": [], "duration_ms": 80}],
    [{"angles": point()["angles"]}],
    *[[point(duration)] for duration in (None, True, 80.0, "80", 79, 30001)],
    *[[point(**{"6": value})] for value in (True, None, "0", [], {}, float("nan"), float("inf"), -float("inf"), 121, -121, 10**1000)],
    [point(**{"1": -1})], [point(**{"1": 91})], [point(**{"2": 91})],
    [point(30000)] * 21,
    [point(), point(**{"3": 121})],
]


@pytest.mark.parametrize("points", INVALID_POINTS)
def test_invalid_whole_path_is_rejected_without_serial_access(backend, points):
    with pytest.raises(ValueError):
        backend.start_trajectory(points)
    assert backend.link.reads == []
    assert backend.link.sent == []
    assert backend.link.stops == []


def test_offline_start_and_standby_are_forbidden(backend):
    backend.link = None
    backend.motion_enabled = False
    with pytest.raises(PermissionError):
        backend.start_trajectory([point()])
    with pytest.raises(PermissionError):
        backend.standby(600)


def test_status_is_cached_and_initially_idle(backend):
    status = backend.trajectory_status()
    assert status == {"state": "idle", "index": 0, "total": 0, "error": None,
                      "run_id": None, "current_angles": {}, "motion_epoch": backend.motion_epoch}
    assert backend.status()["trajectory"] == status
    assert backend.link.reads == []


def test_six_axis_frames_speed_limit_and_repeated_runs(backend):
    durations = []
    backend.trajectory._wait = lambda cancel, seconds: durations.append(seconds) or cancel.is_set()
    response = backend.start_trajectory([point(**{"6": 30}), point(2500, **{"6": -30, "1": 45, "2": 30})])
    assert response["state"] == "preparing"
    assert response["index"] == 0 and response["total"] == 2
    result = finish(backend)
    assert result["state"] == "completed"
    assert result["index"] == 2 and result["error"] is None
    assert result["current_angles"] == point(**{"6": -30, "1": 45, "2": 30})["angles"]
    assert backend.link.sent == [
        ({1: 700, 2: 500, 3: 500, 4: 500, 5: 500, 6: 625}, 1000),
        ({1: 450, 2: 375, 3: 500, 4: 500, 5: 500, 6: 375}, 2500),
    ]
    assert durations == [1.0, 2.5]
    assert len(backend.link.reads) == 12
    result["current_angles"]["6"] = 999
    assert backend.trajectory_status()["current_angles"]["6"] == -30
    backend.start_trajectory([point()])
    again = finish(backend)
    assert again["run_id"] != result["run_id"]
    assert again["state"] == "completed"
    assert len(backend.link.reads) == 24, "每次运行必须新回读"


def test_speed_extension_total_rejected_before_first_movement(backend):
    instant(backend)
    backend.start_trajectory([point(**{"6": -120 if i % 2 == 0 else 120}) for i in range(100)])
    status = finish(backend)
    assert status["state"] == "error" and status["error"]
    assert backend.link.sent == []
    assert backend.link.stops == [IDS]


def test_first_segment_uses_real_position_and_ceil(backend):
    instant(backend)
    backend.link.positions[6] = 625  # 实际 +30°，不是模型安装偏移 +90°。
    backend.start_trajectory([point(**{"6": -0.01})])
    assert finish(backend)["state"] == "completed"
    assert backend.link.sent[0][1] == 1000  # 500 + int(delta)，raw 625 → 500


@pytest.mark.parametrize("start,angles,expected", [
    (500, [2.39, 4.8], [(509, 80), (520, 89)]),  # 浮点解码差向上取整，保守多 1ms
    (625, [0.11], [(500, 1000)]),
])
def test_speed_limit_uses_encoded_positions_on_every_segment(backend, start, angles, expected):
    instant(backend)
    backend.link.positions[6] = start
    backend.start_trajectory([point(**{"6": angle}) for angle in angles])
    assert finish(backend)["state"] == "completed"
    assert [(positions[6], duration) for positions, duration in backend.link.sent] == expected
    previous = start
    for positions, duration in backend.link.sent:
        assert abs(positions[6] - previous) * .24 * 1000 / duration <= 30 + 1e-10
        previous = positions[6]


@pytest.mark.parametrize("command,reply", [
    (BusServoCommand.READ_POSITION, None),
    (BusServoCommand.READ_POSITION, BusServoReport(1, 5, 1, b"\xbc\x02")),
    (BusServoCommand.READ_POSITION, BusServoReport(1, 5, 0, b"\x00")),
    (BusServoCommand.READ_POSITION, BusServoReport(2, 5, 0, b"\xbc\x02")),
    (BusServoCommand.READ_POSITION, BusServoReport(1, 13, 0, b"\xbc\x02")),
    (BusServoCommand.READ_POSITION, BusServoReport(1, 5, 0, b"\xff\xff")),
    (BusServoCommand.READ_POSITION, BusServoReport(1, 5, 0, b"\xbd\x02")),
    (BusServoCommand.READ_POSITION, BusServoReport(1, 5, 0, b"\x00\x00")),
    (BusServoCommand.READ_LOAD_STATE, None),
    (BusServoCommand.READ_LOAD_STATE, BusServoReport(1, 13, 1, b"\x01")),
    (BusServoCommand.READ_LOAD_STATE, BusServoReport(1, 13, 0, b"")),
    (BusServoCommand.READ_LOAD_STATE, BusServoReport(1, 13, 0, b"\x00")),
    (BusServoCommand.READ_LOAD_STATE, BusServoReport(1, 13, 0, b"\x02")),
])
def test_read_failure_or_unloaded_refuses_motion(backend, command, reply):
    instant(backend)
    backend.link.responses[command, 1] = reply
    backend.start_trajectory([point()])
    status = finish(backend)
    assert status["state"] == "error" and status["error"]
    assert status["index"] == 0
    assert backend.link.sent == [] and backend.link.loaded == []
    assert backend.link.stops == [IDS]
    backend.link.responses.clear()
    backend.start_trajectory([point()])
    assert finish(backend)["state"] == "completed", "错误必须释放 busy"


def test_stop_interrupts_wait_and_prevents_later_frames(backend):
    waiting = Waiting()
    backend.trajectory._wait = waiting
    backend.start_trajectory([point(30000), point()])
    assert waiting.entered.wait(3)
    backend.stop()
    result = finish(backend)
    assert result["state"] == "stopped" and result["index"] == 1
    assert backend.link.actions == ["move", "stop"]


def test_stop_during_preparation_waits_for_only_current_read(backend):
    entered, release = threading.Event(), threading.Event()

    def block(command, sid):
        entered.set()
        assert release.wait(3)

    backend.link.read_hook = block
    backend.start_trajectory([point()])
    assert entered.wait(3)
    run = backend.trajectory._active
    stop = threading.Thread(target=backend.stop)
    stop.start()
    try:
        assert run.cancel.wait(3), "STOP 必须先取消，不能先等串口锁"
        assert backend.trajectory_status()["state"] == "preparing"
    finally:
        release.set()
        stop.join(3)
    assert not stop.is_alive()
    assert finish(backend)["state"] == "stopped"
    assert len(backend.link.reads) == 1
    assert backend.link.sent == [] and backend.link.stops == [IDS]


def test_busy_rejects_all_manual_operations(backend):
    waiting = Waiting()
    backend.trajectory._wait = waiting
    backend.start_trajectory([point()])
    assert waiting.entered.wait(3)
    from armcomm.trajectory import TrajectoryBusyError
    for operation in (
        lambda: backend.move({"6": 0}, 600), backend.read_snapshot,
        lambda: backend.set_load_state(True), lambda: backend.open_link("MUST-NOT-OPEN"),
        lambda: backend.start_trajectory([point()]), lambda: backend.standby(600),
    ):
        with pytest.raises(TrajectoryBusyError):
            operation()
    assert len(backend.link.reads) == 12
    backend.stop()
    finish(backend)


def test_disconnect_stops_before_close_and_old_worker_cannot_touch_new_link(backend):
    entered, release = threading.Event(), threading.Event()

    def delayed_exit(cancel, seconds):
        entered.set()
        assert cancel.wait(3)
        assert release.wait(3)
        return True

    backend.trajectory._wait = delayed_exit
    backend.start_trajectory([point(), point()])
    assert entered.wait(3)
    old_link = backend.link
    old_thread = backend.trajectory._thread
    backend.close_link()
    assert old_link.actions == ["move", "stop", "close"]
    assert backend.trajectory_status()["state"] == "stopped"
    backend.link = FakeLink()
    backend.motion_enabled = True
    instant(backend)
    response = backend.start_trajectory([point()])
    result = finish(backend)
    try:
        assert result["state"] == "completed"
    finally:
        release.set()
        old_thread.join(3)
    assert not old_thread.is_alive()
    assert backend.trajectory_status()["run_id"] == response["run_id"]
    assert backend.trajectory_status()["state"] == "completed"
    assert backend.link.actions == ["move"]
    assert old_link.actions == ["move", "stop", "close"]


def test_write_error_stops_and_releases_busy(backend):
    instant(backend)
    backend.link.write_error = True
    backend.start_trajectory([point(), point()])
    result = finish(backend)
    assert result["state"] == "error" and "write failed" in result["error"]
    assert backend.link.stops == [IDS]
    backend.link.write_error = False
    backend.start_trajectory([point()])
    assert finish(backend)["state"] == "completed"


def test_standby_preserves_other_four_actual_axes(backend):
    instant(backend)
    # 这些原始数值的角度反算后可能有浮点尾数，不得让保持轴漂移一个脉冲。
    backend.link.positions = {1: 226, 2: 623, 3: 500, 4: 801, 5: 599, 6: 625}
    backend.standby(600)
    result = finish(backend)
    assert result["state"] == "completed" and result["total"] == 1
    assert backend.link.sent == [({1: 226, 2: 623, 3: 125, 4: 801, 5: 599, 6: 125}, 4000)]
    assert result["current_angles"]["3"] == -90
    assert result["current_angles"]["6"] == -90
    assert result["current_angles"]["1"] == pytest.approx(85.32)
    assert backend.link.loaded == []


@pytest.mark.parametrize("duration", [None, "600", True, 600.0, 79, 30001])
def test_standby_rejects_invalid_duration_before_reads(backend, duration):
    with pytest.raises(ValueError):
        backend.standby(duration)
    assert backend.link.reads == []


@pytest.fixture
def http(backend):
    httpd = serve("127.0.0.1", 0, backend)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield "http://127.0.0.1:%d" % httpd.server_address[1]
    httpd.shutdown()
    httpd.server_close()
    thread.join(3)


def request(base, path, payload=None, *, raw=None, with_epoch=True):
    if with_epoch and payload is not None and (path in ("/api/move", "/api/trajectory/start", "/api/standby", "/api/load")
            or path == "/api/link" and payload.get("enabled") is True):
        epoch = json.loads(request(base, "/api/status")[1])["motion_epoch"]
        payload = {"motion_epoch": epoch, **payload}
    data = raw if raw is not None else (json.dumps(payload).encode() if payload is not None else None)
    req = urllib.request.Request(base + path, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=3) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as error:
        return error.code, error.read()


def test_http_trajectory_contract_and_busy_responses(http, backend):
    waiting = Waiting()
    backend.trajectory._wait = waiting
    status, body = request(http, "/api/trajectory/start", {"points": [point(), point()]})
    assert status == 200
    initial = json.loads(body)
    assert set(initial) == {"state", "index", "total", "error", "run_id", "current_angles"}
    assert waiting.entered.wait(3)
    for path, payload in (
        ("/api/trajectory/start", {"points": [point()]}), ("/api/standby", {}),
        ("/api/move", {"targets": {"6": 0}}), ("/api/read", {}),
        ("/api/load", {"loaded": True}), ("/api/link", {"enabled": True, "port": "MUST-NOT-OPEN"}),
    ):
        assert request(http, path, payload)[0] == 409
    code, body = request(http, "/api/trajectory/status")
    running = json.loads(body)
    assert code == 200 and running["state"] == "running" and running["index"] == 1
    code, body = request(http, "/api/status")
    assert json.loads(body)["trajectory"] == running
    assert len(backend.link.reads) == 12
    assert request(http, "/api/stop", {})[0] == 200
    assert finish(backend)["state"] == "stopped"
    instant(backend)
    assert request(http, "/api/standby", {"duration_ms": 600})[0] == 200
    assert finish(backend)["state"] == "completed"
    assert request(http, "/api/link", {"enabled": False})[0] == 200
    assert request(http, "/api/trajectory/start", {"points": [point()]})[0] == 403
    assert request(http, "/api/standby", {})[0] == 403


@pytest.mark.parametrize("raw", [b"[]", b"null", b"1", b"true", b'"x"', b"{oops", b" " * (1024 * 1024 + 1)],
                         ids=["array", "null", "number", "bool", "string", "malformed", "oversize"])
def test_http_rejects_invalid_body_with_400(http, backend, raw):
    assert request(http, "/api/trajectory/start", raw=raw)[0] == 400
    assert backend.link.reads == [] and backend.link.sent == []


@pytest.mark.parametrize("path,payload", [
    ("/api/trajectory/start", {}), ("/api/trajectory/start", {"points": [point(True)]}),
    ("/api/standby", {"duration_ms": "600"}), ("/api/link", {"enabled": True, "port": []}),
    ("/api/move", {"targets": [1]}), ("/api/load", {"loaded": []}),
])
def test_http_invalid_types_are_400(http, backend, path, payload):
    assert request(http, path, payload)[0] == 400
    assert backend.link.sent == []


def test_http_model_limits_and_explicit_static_whitelist(http, monkeypatch, tmp_path):
    import tools.serve as service
    # 不编辑前端资产；临时资源验证唯一静态白名单。
    (tmp_path / "trajectory.js").write_text("window.testTrajectory = true;", encoding="utf-8")
    (tmp_path / "secret.js").write_text("not public", encoding="utf-8")
    monkeypatch.setattr(service, "WEB_DIR", tmp_path)
    code, body = request(http, "/trajectory.js")
    assert code == 200 and b"testTrajectory" in body
    assert request(http, "/secret.js")[0] == 404
    assert request(http, "/trajectory.js/../secret.js")[0] == 404
    code, body = request(http, "/model.json")
    model = json.loads(body)
    assert model["trajectory_limits"] == {"max_points": 600, "min_duration_ms": 80,
        "max_duration_ms": 30000, "max_total_ms": 600000, "max_speed_deg_s": 30}
    assert model["standby_angles"] == {"6": -90, "3": -90}


def test_serial_open_has_bounded_write_timeout(monkeypatch):
    import serial
    from armcomm.link import SerialLink
    opened = []

    def fake_serial(**kwargs):
        opened.append(kwargs)
        return FakeLink()

    monkeypatch.setattr(serial, "Serial", fake_serial)
    link = SerialLink("FAKE-NO-HARDWARE").open()
    try:
        assert 0 < opened[0].get("write_timeout", float("inf")) <= 0.3
    finally:
        link.close()


@pytest.mark.parametrize("payload", [
    {"targets": {"6": True}}, {"targets": {"6": "10"}},
    {"targets": {"6": float("inf")}}, {"targets": {"6": float("nan")}},
    {"targets": {"6": 0}, "duration_ms": True},
    {"targets": {"6": 0}, "duration_ms": "600"},
    {"targets": {"6": 0}, "duration_ms": 600.5},
    {"targets": {"6": 0}, "duration_ms": None},
])
def test_http_manual_move_rejects_invalid_numeric_types(http, backend, payload):
    assert request(http, "/api/move", payload)[0] == 400
    assert backend.link.sent == []


@pytest.mark.parametrize("points", [[point(1000)] * 600, [point(30000)] * 20])
def test_point_and_total_duration_boundaries_are_inclusive(backend, points):
    instant(backend)
    backend.start_trajectory(points)
    status = finish(backend)
    assert status["state"] == "completed" and status["index"] == len(points)
    assert len(backend.link.sent) == len(points)
    assert sum(duration for _, duration in backend.link.sent) == 600000


def test_all_joint_angle_boundaries_are_inclusive(backend):
    instant(backend)
    backend.start_trajectory([point(**{"1": 90, "2": -90, "3": -120, "4": 120, "5": -120, "6": 120})])
    assert finish(backend)["state"] == "completed"
    assert backend.link.sent == [({1: 200, 2: 875, 3: 0, 4: 1000, 5: 0, 6: 1000}, 4000)]


def test_start_copies_input_before_background_preparation(backend):
    entered, release = threading.Event(), threading.Event()

    def block(command, sid):
        entered.set()
        assert release.wait(3)

    backend.link.read_hook = block
    instant(backend)
    points = [point(**{"6": 30})]
    backend.start_trajectory(points)
    try:
        assert entered.wait(3)
        points[0]["angles"]["6"] = 999
        points.append(point(**{"1": -999}))
    finally:
        release.set()
    assert finish(backend)["total"] == 1
    assert backend.link.sent == [({1: 700, 2: 500, 3: 500, 4: 500, 5: 500, 6: 625}, 1000)]


def test_late_load_failure_still_sends_no_movement(backend):
    instant(backend)
    backend.link.responses[BusServoCommand.READ_LOAD_STATE, 6] = BusServoReport(6, 13, 0, b"\x00")
    backend.start_trajectory([point()])
    assert finish(backend)["state"] == "error"
    assert len(backend.link.reads) == 12
    assert backend.link.sent == [] and backend.link.loaded == []


def test_busy_during_preparation_is_rejected_without_waiting_for_read(backend):
    from armcomm.trajectory import TrajectoryBusyError
    entered, release, rejected = threading.Event(), threading.Event(), threading.Event()

    def block(command, sid):
        entered.set()
        assert release.wait(3)

    backend.link.read_hook = block
    instant(backend)
    backend.start_trajectory([point()])
    assert entered.wait(3)

    def attempt_manual_read():
        with pytest.raises(TrajectoryBusyError):
            backend.read_snapshot()
        rejected.set()

    thread = threading.Thread(target=attempt_manual_read)
    thread.start()
    try:
        assert rejected.wait(3)
        assert backend.status()["trajectory"]["state"] == "preparing"
    finally:
        release.set()
        thread.join(3)
    assert finish(backend)["state"] == "completed"


def test_stop_holds_busy_until_stop_write_completes(backend):
    from armcomm.trajectory import TrajectoryBusyError
    waiting = Waiting()
    backend.trajectory._wait = waiting
    backend.start_trajectory([point(), point()])
    assert waiting.entered.wait(3)
    old_thread = backend.trajectory._thread
    entered, release = threading.Event(), threading.Event()
    original_stop = backend.link.write_stop

    def slow_stop(ids):
        entered.set()
        assert release.wait(3)
        return original_stop(ids)

    backend.link.write_stop = slow_stop
    stop_thread = threading.Thread(target=backend.stop)
    stop_thread.start()
    try:
        assert entered.wait(3)
        old_thread.join(3)
        assert not old_thread.is_alive()
        with pytest.raises(TrajectoryBusyError):
            backend.start_trajectory([point()])
    finally:
        release.set()
        stop_thread.join(3)
    assert backend.trajectory_status()["state"] == "stopped"
    instant(backend)
    backend.start_trajectory([point()])
    assert finish(backend)["state"] == "completed"
    assert backend.link.actions == ["move", "stop", "move"]


def test_stop_write_failure_is_error_not_completed_and_releases_busy(backend):
    waiting = Waiting()
    backend.trajectory._wait = waiting
    backend.start_trajectory([point(), point()])
    assert waiting.entered.wait(3)
    backend.link.stop_error = True
    with pytest.raises(OSError, match="stop failed"):
        backend.stop()
    status = finish(backend)
    assert status["state"] == "error" and "stop failed" in status["error"]
    assert backend.link.actions == ["move", "stop"]
    instant(backend)
    backend.link.stop_error = False
    backend.start_trajectory([point()])
    assert finish(backend)["state"] == "completed"


def test_disconnect_closes_even_if_stop_fails(backend):
    waiting = Waiting()
    backend.trajectory._wait = waiting
    backend.start_trajectory([point(), point()])
    assert waiting.entered.wait(3)
    old_link = backend.link
    old_link.stop_error = True
    status = backend.close_link()
    assert status["linked"] is False and status["motion_enabled"] is False
    assert "stop failed" in status["error"]
    assert finish(backend)["state"] == "error"
    assert old_link.actions == ["move", "stop", "close"]


def test_read_exception_and_failed_emergency_stop_report_both_errors(backend):
    instant(backend)

    def fail_read(command, sid):
        raise OSError("read failed")

    backend.link.read_hook = fail_read
    backend.link.stop_error = True
    backend.start_trajectory([point()])
    result = finish(backend)
    assert result["state"] == "error"
    assert "read failed" in result["error"] and "stop failed" in result["error"]
    assert backend.link.sent == []
    backend.link.read_hook = None
    backend.link.stop_error = False
    backend.start_trajectory([point()])
    assert finish(backend)["state"] == "completed"


def test_disconnection_during_preparation_prevents_remaining_reads(backend):
    entered, release = threading.Event(), threading.Event()

    def block(command, sid):
        entered.set()
        assert release.wait(3)

    backend.link.read_hook = block
    backend.start_trajectory([point()])
    assert entered.wait(3)
    run = backend.trajectory._active
    old_link = backend.link
    close_thread = threading.Thread(target=backend.close_link)
    close_thread.start()
    try:
        assert run.cancel.wait(3)
    finally:
        release.set()
        close_thread.join(3)
    assert not close_thread.is_alive()
    assert finish(backend)["state"] == "stopped"
    assert len(old_link.reads) == 1
    assert old_link.actions == ["stop", "close"]


def test_http_disconnect_can_cancel_a_running_trajectory(http, backend):
    waiting = Waiting()
    backend.trajectory._wait = waiting
    assert request(http, "/api/trajectory/start", {"points": [point(), point()]})[0] == 200
    assert waiting.entered.wait(3)
    old_link = backend.link
    assert request(http, "/api/link", {"enabled": False})[0] == 200
    assert finish(backend)["state"] == "stopped"
    assert old_link.actions == ["move", "stop", "close"]


MOTION_REQUESTS = [
    ("/api/move", {"targets": {"6": 0}}, "move"),
    ("/api/trajectory/start", {"points": [point()]}, "start_trajectory"),
    ("/api/standby", {}, "standby"),
    ("/api/link", {"enabled": True, "port": "FAKE-NO-HARDWARE"}, "open_link"),
    ("/api/load", {"loaded": True}, "set_load_state"),
]


@pytest.mark.parametrize("path,payload,method", MOTION_REQUESTS)
def test_http_motion_requires_epoch_even_when_unlinked(http, backend, monkeypatch, path, payload, method):
    from armcomm.link import SerialLink
    monkeypatch.setattr(SerialLink, "open", lambda self: (_ for _ in ()).throw(AssertionError("禁止打开串口")))
    backend.link = None
    backend.motion_enabled = False
    code, body = request(http, path, payload, with_epoch=False)
    assert code == 400
    assert "motion_epoch" in json.loads(body)["error"]


@pytest.mark.parametrize("interrupt", ["/api/stop", "/api/link"])
@pytest.mark.parametrize("path,payload,method", MOTION_REQUESTS)
def test_http_late_motion_after_interrupt_is_fenced(http, backend, monkeypatch, interrupt, path, payload, method):
    from armcomm.link import SerialLink
    monkeypatch.setattr(SerialLink, "open", lambda self: pytest.fail("不能打开任何宿主机串口"))
    epoch = json.loads(request(http, "/api/status")[1]).get("motion_epoch")
    assert isinstance(epoch, str) and epoch
    old = backend.link
    response = json.loads(request(http, interrupt, {"enabled": False})[1])
    assert response["motion_epoch"] != epoch
    assert response["trajectory"]["motion_epoch"] == response["motion_epoch"]
    code, body = request(http, path, {**payload, "motion_epoch": epoch})
    assert code == 409 and "motion_epoch" in json.loads(body)["error"]
    assert old.sent == [] and old.loaded == []
    assert backend.trajectory_status()["state"] == "idle"


@pytest.mark.parametrize("path,payload,method", MOTION_REQUESTS)
def test_http_motion_rechecks_epoch_after_preparation(http, backend, monkeypatch, path, payload, method):
    epoch = backend.status().get("motion_epoch")
    assert epoch
    entered, release = threading.Event(), threading.Event()
    original = getattr(backend, method)
    result = []

    def delayed(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return original(*args, **kwargs)

    monkeypatch.setattr(backend, method, delayed)
    thread = threading.Thread(target=lambda: result.append(request(http, path, {**payload, "motion_epoch": epoch})))
    thread.start()
    try:
        assert entered.wait(3)
        assert request(http, "/api/stop", {})[0] == 200
    finally:
        release.set()
        thread.join(3)
    assert result[0][0] == 409
    assert backend.link.actions == ["stop"]
    assert backend.link.loaded == []


def test_epoch_rotates_before_stop_can_acquire_serial_lock(backend):
    epoch = backend.status().get("motion_epoch")
    assert epoch
    entered = threading.Event()
    cancel = backend.trajectory.cancel

    def observe_cancel():
        entered.set()
        return cancel()

    backend.trajectory.cancel = observe_cancel
    backend.lock.acquire()
    thread = threading.Thread(target=backend.stop)
    thread.start()
    try:
        assert entered.wait(3)
        assert backend.status()["motion_epoch"] != epoch
        assert backend.link.stops == []
    finally:
        backend.lock.release()
        thread.join(3)
    assert backend.link.actions == ["stop"]


@pytest.mark.parametrize("path,payload,method", MOTION_REQUESTS)
def test_http_epoch_rechecked_inside_serial_lock_not_only_at_entry(http, backend, monkeypatch, path, payload, method):
    from armcomm.link import SerialLink
    monkeypatch.setattr(SerialLink, "open", lambda self: FakeLink())
    serial_lock = backend.lock
    queued, cancelled = threading.Event(), threading.Event()
    original_cancel = backend.trajectory.cancel

    class ObservedLock:
        def __enter__(self):
            queued.set()
            serial_lock.acquire()

        def __exit__(self, *args):
            serial_lock.release()

    def cancel():
        run = original_cancel()
        cancelled.set()
        return run

    monkeypatch.setattr(backend, "lock", ObservedLock())
    monkeypatch.setattr(backend.trajectory, "cancel", cancel)
    epoch = backend.motion_epoch
    results = {}
    motion = threading.Thread(target=lambda: results.update(motion=request(http, path, {**payload, "motion_epoch": epoch})))
    stop = threading.Thread(target=lambda: results.update(stop=request(http, "/api/stop", {})))
    old_link = backend.link
    serial_lock.acquire()
    motion.start()
    try:
        assert queued.wait(3), "运动请求已完成前置校验，正在等待串口锁"
        stop.start()
        assert cancelled.wait(3)
        assert backend.motion_epoch != epoch
        assert old_link.actions == []
    finally:
        serial_lock.release()
        motion.join(3)
        if stop.ident is not None:
            stop.join(3)
    assert results["motion"][0] == 409
    assert results["stop"][0] == 200
    assert old_link.actions == ["stop"]
    assert old_link.loaded == []
    assert backend.link is old_link


def test_new_process_backend_does_not_reuse_motion_epoch(backend):
    other = build_backend(None, {"port": "FAKE-NO-HARDWARE"})
    assert other.motion_epoch != backend.motion_epoch

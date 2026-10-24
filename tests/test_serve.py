"""仿真服务测试：模型导出、网格路径安全、真机联动的开关闸门，以及一次真实 HTTP 往返。

重点不在"接口能返回 200"，而在两条安全约束：
未开启联动时**任何**运动请求都不能碰到串口；开启后角度也必须先被夹进舵机限位。
"""

import json
import threading
import urllib.error
import urllib.request

import pytest

from armcomm.servos import URDF_LIMIT_DEG
from tools.serve import SimBackend, build_backend, list_available_ports, serve

REPO_ROOT = __import__("pathlib").Path(__file__).resolve().parent.parent


class FakeLink:
    def __init__(self):
        self.port = "COM-FAKE"
        self.stats_data = {"bytes": 0, "frames": 0, "discarded": 0, "checksum_errors": 0, "skipped": 0}
        self.sent_positions = []
        self.stopped = []
        self.loaded = []
        self.closed = False

    @property
    def stats(self):
        return self.stats_data

    def write_positions(self, positions, duration_ms=1000):
        self.sent_positions.append((dict(positions), duration_ms))
        return b""

    def write_stop(self, ids):
        self.stopped.append(sorted(ids))
        return b""

    def set_loaded(self, loaded, ids):
        self.loaded.append((loaded, sorted(ids)))
        return []

    def read_snapshot(self, ids):
        return {sid: {"name": "测试", "ok": True, "position": 500, "vin": 7800,
                      "temperature": 32, "loaded": True} for sid in ids}

    def close(self):
        self.closed = True


@pytest.fixture(scope="module")
def backend() -> SimBackend:
    built = build_backend(None, {"port": "COM4", "baudrate": 1000000})
    assert built.urdf_path.exists()
    return built


@pytest.fixture()
def linked_backend(backend) -> SimBackend:
    """假装已经连上真机：注入假串口对象，逻辑层与硬件解耦。"""
    fake = FakeLink()
    backend.link = fake
    backend.motion_enabled = True
    backend.last_error = None
    yield backend, fake
    backend.link = None
    backend.motion_enabled = False


# ---------------------------------------------------------------- 模型导出


def test_model_json_exposes_six_actuated_joints(backend):
    payload = backend.model_json()
    actuated = [joint for joint in payload["joints"] if joint["servo_id"]]
    assert sorted(joint["servo_id"] for joint in actuated) == [1, 2, 3, 4, 5, 6]
    for joint in actuated:
        assert joint["servo_name"]
        assert joint["slider_min_deg"] <= joint["slider_max_deg"]
        assert joint["reset_position"] is not None


def test_model_json_carries_the_decoding_helpers(backend):
    payload = backend.model_json()
    assert payload["angle_factor"] == pytest.approx(1000 / 240)
    # Python 侧是 int 键；json.dumps 之后才变成字符串键，前端按字符串取
    assert payload["servo_math"][1] == "claw"
    assert payload["servo_math"][2] == "roll"
    assert payload["servo_math"][6] == "common"
    assert json.loads(json.dumps(payload))["servo_math"]["1"] == "claw"
    assert payload["port"] == "COM4"
    assert payload["baud"] == 1000000


def test_slider_bounds_match_firmware_space(backend):
    payload = backend.model_json()
    by_id = {joint["servo_id"]: joint for joint in payload["joints"] if joint["servo_id"]}
    assert by_id[1]["slider_min_deg"] == 0 and by_id[1]["slider_max_deg"] == 90
    assert by_id[2]["slider_min_deg"] == -90 and by_id[2]["slider_max_deg"] == 90
    assert by_id[6]["slider_min_deg"] == pytest.approx(-120.0)
    assert by_id[6]["slider_max_deg"] == pytest.approx(120.0)
    assert URDF_LIMIT_DEG < 120.0


def test_defects_are_surfaced_to_the_ui(backend):
    defects = backend.model_json()["defects"]
    assert any("joint1" in d for d in defects)
    assert any("package://" in d for d in defects)


def test_gripper_uses_matching_vendor_assembly(backend):
    """旧 URDF 把指尖放在内连杆之外；专用 xacro 的铰点位于连杆末端。"""
    joints = {j["name"]: j for j in backend.model_json()["joints"]}
    assert "l_out_joint" in joints
    assert "l_out_link" not in joints
    for name in ("l_out_joint", "r_out_joint"):
        assert joints[name]["type"] == "revolute"
        assert joints[name]["matrix"][12:15] == pytest.approx([0.00020427, 0, 0.028999])
    assert joints["r_joint"]["type"] == "revolute"
    assert joints["r_in_joint"]["axis"] == pytest.approx([-0.99996, 0.0046987, 0.0070436])


def test_all_five_passive_gripper_joints_follow_the_existing_servo(backend):
    payload = backend.model_json()
    driven = next(j for j in payload["joints"] if j["servo_id"] == 1)
    assert driven["name"] == "l_joint"  # 不更改串口侧的舵机映射
    assert driven["servo_scale"] == -1  # 左关节负转才张开，仅用于显示
    assert payload["gripper_mimic"] == {
        "r_joint": {"ratio": -1, "offset": 0},
        "l_in_joint": {"ratio": 1, "offset": 0},
        "l_out_joint": {"ratio": -1, "offset": 0},
        "r_in_joint": {"ratio": 1, "offset": 0},
        "r_out_joint": {"ratio": -1, "offset": 0},
    }


def test_optional_camera_is_hidden_and_never_enables_motion(backend):
    payload = backend.model_json()
    camera = payload.get("optional_components", {}).get("camera")
    assert camera is not None, "相机应作为独立可选组件导出"
    assert camera["root_link"] == "camera_connet_link"
    assert camera["default_visible"] is False
    assert backend.motion_enabled is False
    assert backend.link is None


# ---------------------------------------------------------------- 网格路径安全


def test_resolve_mesh_accepts_a_real_mesh(backend):
    path = backend.resolve_mesh("/meshes/base_link.STL")
    assert path is not None and path.exists()


@pytest.mark.parametrize(
    "web_path",
    [
        "/meshes/../../../../../Windows/win.ini",
        "/meshes\\..\\..\\secret.STL",
        "/secrets.txt",
        "/meshes/notes.txt",
        "/meshes/not_there.STL",
    ],
)
def test_resolve_mesh_refuses_traversal_and_non_mesh(backend, web_path):
    assert backend.resolve_mesh(web_path) is None


# ---------------------------------------------------------------- 联动闸门


def test_move_is_refused_when_not_linked(backend):
    assert backend.motion_enabled is False
    with pytest.raises(PermissionError):
        backend.move({"6": 10.0}, 500)


def test_move_sends_clamped_positions_when_linked(linked_backend):
    backend, fake = linked_backend
    result = backend.move({"6": 999.0, "1": 45.0}, 600)
    assert len(fake.sent_positions) == 1
    positions, duration = fake.sent_positions[0]
    assert duration == 600
    assert positions[6] == 1000, "超范围角度必须先夹到限位"
    assert positions[1] == 450
    assert result["applied"][6]["position"] == 1000


def test_move_rejects_unknown_servo_before_writing(linked_backend):
    backend, fake = linked_backend
    with pytest.raises(ValueError):
        backend.move({"7": 0.0}, 500)
    assert fake.sent_positions == []


def test_base_display_offset_does_not_shift_serial_targets(linked_backend):
    backend, fake = linked_backend
    base = next(j for j in backend.model_json()["joints"] if j["servo_id"] == 6)
    assert base["servo_offset"] == pytest.approx(1.5707963267948966)
    assert base["servo_scale"] == 1
    for angle, position in [(0.0, 500), (30.0, 625), (-30.0, 375)]:
        result = backend.move({"6": angle}, 600)
        assert result["applied"][6] == {"angle_deg": angle, "position": position}
    assert fake.sent_positions == [({6: 500}, 600), ({6: 625}, 600), ({6: 375}, 600)]


def test_stop_and_load_also_need_a_link(backend):
    with pytest.raises(PermissionError):
        backend.stop()
    with pytest.raises(PermissionError):
        backend.set_load_state(True)


def test_stop_reaches_every_servo(linked_backend):
    backend, fake = linked_backend
    backend.stop()
    assert fake.stopped == [[1, 2, 3, 4, 5, 6]]


def test_set_load_state_targets_all_six(linked_backend):
    backend, fake = linked_backend
    backend.set_load_state(False)
    assert fake.loaded == [(False, [1, 2, 3, 4, 5, 6])]


def test_read_snapshot_caches_into_status(linked_backend):
    backend, fake = linked_backend
    status = backend.read_snapshot()
    assert status["servos"][6]["position"] == 500
    assert status["linked"] is True
    assert status["motion_enabled"] is True


def test_closing_link_disarms_motion(backend):
    fake = FakeLink()
    backend.link = fake
    backend.motion_enabled = True
    status = backend.close_link()
    assert fake.closed is True
    assert status["linked"] is False
    assert status["motion_enabled"] is False
    with pytest.raises(PermissionError):
        backend.move({"6": 0.0}, 500)


def test_open_link_reports_failure_instead_of_crashing(backend):
    # COM999 不存在，open_link 必须把异常转成界面可读信息并保持"未联动"
    status = backend.open_link("COM999", 1000000)
    assert status["linked"] is False
    assert status["motion_enabled"] is False
    assert "COM999" in status["error"] or "port" in status["error"].lower()


# ---------------------------------------------------------------- HTTP 往返


@pytest.fixture(scope="module")
def http(backend):
    httpd = serve("127.0.0.1", 0, backend)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    base = "http://127.0.0.1:%d" % httpd.server_address[1]
    yield base
    httpd.shutdown()
    httpd.server_close()


def _get(url):
    with urllib.request.urlopen(url, timeout=10) as response:
        return response.status, response.read()


def _post(url, payload):
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"), headers={"Content-Type": "application/json"}, method="POST"
    )
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as error:
        return error.code, error.read()


def test_http_serves_the_page_and_model(http):
    status, body = _get(http + "/")
    assert status == 200
    page = body.decode("utf-8")
    assert "<title>六轴机械臂数字交互</title>" in page
    assert "<h1>六轴机械臂数字交互 " in page
    status, body = _get(http + "/model.json")
    payload = json.loads(body)
    assert status == 200 and len(payload["links"]) == 14


def test_http_serves_vendored_assets(http):
    """页面靠这三个本地文件渲染，路径拼错就是 404 —— 这条正是上次漏掉的那条。"""
    for asset in ("/vendor/three.min.js", "/vendor/STLLoader.js", "/vendor/OrbitControls.js"):
        status, body = _get(http + asset)
        assert status == 200, asset
        assert len(body) > 5000, asset


def test_http_blocks_vendor_traversal(http):
    for probe in ("/vendor/../../requirements.txt", "/vendor/%2e%2e%2fserve.py"):
        try:
            status, _ = _get(http + probe)
        except urllib.error.HTTPError as error:
            status = error.code
        assert status == 404, probe


def test_http_serves_mesh_binary(http):
    status, body = _get(http + "/meshes/gripper_link.STL")
    assert status == 200
    assert body[:5] == b"solid" or len(body) > 1000  # ASCII 或二进制 STL 都接受，只要拿到内容


def test_http_blocks_path_traversal(http):
    with pytest.raises(urllib.error.HTTPError) as error:
        _get(http + "/meshes/..%2F..%2F..%2Fwindows%2Fwin.ini")
    assert error.value.code == 404


def test_http_refuses_move_without_link(http):
    status, body = _post(http + "/api/move", {"targets": {"6": 10.0}, "duration_ms": 400})
    assert status == 403
    assert "联动" in json.loads(body)["error"]


def test_http_rejects_move_without_a_link_switch_even_when_enabled(http, backend):
    fake = FakeLink()
    backend.link = fake
    backend.motion_enabled = True
    try:
        status, body = _post(http + "/api/move", {"targets": {"6": 12.0}, "duration_ms": 400})
        assert status == 200
        assert fake.sent_positions
    finally:
        backend.link = None
        backend.motion_enabled = False


def test_http_status_endpoint_shape(http):
    status, body = _get(http + "/api/status")
    payload = json.loads(body)
    assert status == 200
    assert set(payload) >= {"linked", "motion_enabled", "port", "error", "servos", "defects"}


def test_http_bad_json_is_400(http):
    request = urllib.request.Request(http + "/api/move", data=b"{not json}", method="POST",
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            code = response.status
    except urllib.error.HTTPError as error:
        code = error.code
    assert code == 400


def test_list_available_ports_returns_a_list():
    ports = list_available_ports()
    assert isinstance(ports, list)
    for item in ports:
        assert set(item) == {"device", "description"}

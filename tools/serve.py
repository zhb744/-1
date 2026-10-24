"""仿真服务的后端：独占 COM 口，给浏览器 3D 页面提供模型、网格与控制接口。

分层沿用之前定下的方案 —— 串口逻辑集中在服务端，界面刷新、掉线重连都不会让机械臂
停在半路的指令上；将来把这一层搬到树莓派，前端一行都不用改。

安全约定：

* **真机联动默认关闭**。页面必须先显式调 ``/api/link`` 打开并指定端口，``/api/move``
  才会真的往串口写；否则只在浏览器里动，一个字节都不发出去。
* 角度先按 ``armcomm.servos`` 的关节限位夹紧，再换算成 0~1000 位置；``SerialLink``
  在写串口之前还会做一次位置合法性校验，夹爪硬上限 700（防堵转）。
* 所有 6 个关节放在**同一帧** ``0x01`` 里下发，避免各关节不同步导致姿态扭曲。
* ``/api/stop`` 发 ``0x03`` 停止，是唯一允许的运动中干预指令。

启动::

    python tools/serve.py                 # 默认 http://127.0.0.1:8300
    python tools/serve.py --port COM4     # 预先记住端口（仍需页面里手动开启联动）
"""

from __future__ import annotations

import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

REPO_ROOT = Path(__file__).resolve().parent.parent
import sys

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from armcomm.link import DEFAULT_BAUD, SerialLink  # noqa: E402
from armcomm.servos import JOINTS, servo_angle_to_position, urdf_joint_to_servo_id  # noqa: E402
from armcomm.urdf import find_default_urdf, parse_urdf, to_web_payload  # noqa: E402

WEB_DIR = REPO_ROOT / "web"
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".stl": "application/octet-stream",
    ".STL": "application/octet-stream",
}
SERVO_IDS = tuple(sorted(JOINTS))


def _servo_math_kind(servo_id: int) -> str:
    """前端解码位置用的换算类型，与 ``armcomm.servos`` 的三个函数一一对应。"""
    if servo_id == 1:
        return "claw"
    if servo_id == 2:
        return "roll"
    return "common"


def load_config(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


class SimBackend:
    """持有串口的那一层。Windows 串口独占，所以全局只能有一个实例连着同一个 COM 口。"""

    def __init__(self, urdf_path: Path, serial_config: dict):
        self.urdf_path = Path(urdf_path)
        self.model = parse_urdf(self.urdf_path, force_revolute=set(urdf_joint_to_servo_id()))
        self.payload = to_web_payload(self.model, urdf_joint_to_servo_id())
        self.serial_config = serial_config
        self.lock = threading.Lock()
        self.link: SerialLink | None = None
        self.last_error: str | None = None
        self.motion_enabled = False
        self.snapshot_cache: dict = {}

    # ------------------------------------------------------------------ 资源
    def model_json(self) -> dict:
        joints = []
        for joint in self.payload["joints"]:
            servo_id = joint["servo_id"]
            info = JOINTS.get(servo_id)
            joints.append(
                {
                    **joint,
                    "servo_name": info.name if info else None,
                    "slider_min_deg": info.min_angle if info else None,
                    "slider_max_deg": info.max_angle if info else None,
                    "reset_position": info.reset_position if info else None,
                }
            )
        return {
            "robot": self.payload["robot"],
            "root_link": self.payload["root_link"],
            "links": self.payload["links"],
            "joints": joints,
            "defects": self.payload["defects"],
            "gripper_mimic": self.payload["gripper_mimic"],
            "servo_ids": list(SERVO_IDS),
            # 位置→角度的解码式子集中在这里，前端不再自己抄一份公式
            "angle_factor": 1000 / 240,
            "servo_math": {sid: _servo_math_kind(sid) for sid in SERVO_IDS},
            "port": self.serial_config.get("port"),
            "baud": int(self.serial_config.get("baudrate", DEFAULT_BAUD)),
        }

    def resolve_mesh(self, web_path: str) -> Path | None:
        """只允许取包目录下 meshes/ 里的 STL，防止路径穿越。"""
        name = unquote(web_path).lstrip("/")
        if ".." in name or "\\" in name:
            return None
        candidate = (self.model.package_root / name).resolve()
        root = (self.model.package_root / "meshes").resolve()
        if root not in candidate.parents and candidate.parent != root:
            return None
        if candidate.suffix.lower() not in (".stl",):
            return None
        return candidate if candidate.exists() else None

    # ------------------------------------------------------------------ 串口
    def open_link(self, port: str, baud: int | None = None) -> dict:
        with self.lock:
            if self.link is not None:
                self.close_link_locked()
            try:
                self.link = SerialLink(port, baud or int(self.serial_config.get("baudrate", DEFAULT_BAUD))).open()
            except Exception as error:  # noqa: BLE001 - 端口问题种类多，统一转成界面可读信息
                self.link = None
                self.last_error = f"{type(error).__name__}: {error}"
                self.motion_enabled = False
                return self.status()
            self.last_error = None
            self.motion_enabled = True
            return self.status()

    def close_link_locked(self) -> dict:
        if self.link is not None:
            try:
                self.link.close()
            finally:
                self.link = None
        self.motion_enabled = False
        return self.status()

    def close_link(self) -> dict:
        with self.lock:
            return self.close_link_locked()

    def status(self) -> dict:
        return {
            "linked": self.link is not None,
            "motion_enabled": self.motion_enabled,
            "port": self.link.port if self.link else self.serial_config.get("port"),
            "error": self.last_error,
            "stats": self.link.stats if self.link else None,
            "servos": self.snapshot_cache,
            "defects": self.payload["defects"],
        }

    def read_snapshot(self) -> dict:
        with self.lock:
            if self.link is None:
                self.last_error = "未连接真机，无法回读"
                return self.status()
            self.snapshot_cache = self.link.read_snapshot(SERVO_IDS)
            return self.status()

    def move(self, targets_deg: dict, duration_ms: int) -> dict:
        """按舵机角度（度）下发；先在服务端夹到限位，再换算成位置。"""
        positions = {}
        applied = {}
        for key, angle in targets_deg.items():
            servo_id = int(key)
            if servo_id not in JOINTS:
                raise ValueError(f"舵机 ID {servo_id} 不在 1~6 范围内")
            angle = float(angle)
            position = servo_angle_to_position(servo_id, angle)
            positions[servo_id] = position
            applied[servo_id] = {"angle_deg": angle, "position": position}
        with self.lock:
            if not self.motion_enabled or self.link is None:
                raise PermissionError("真机联动未开启，指令未发出")
            self.link.write_positions(positions, duration_ms)
            return {"sent": positions, "applied": applied, "stats": self.link.stats}

    def stop(self) -> dict:
        with self.lock:
            if self.link is None:
                raise PermissionError("未连接真机")
            self.link.write_stop(SERVO_IDS)
            return {"stopped": list(SERVO_IDS)}

    def set_load_state(self, loaded: bool) -> dict:
        """加载/卸力（子命令 0x0C / 0x0B）。卸力后臂会变软，加载后舵机才带保持力矩。"""
        with self.lock:
            if not self.motion_enabled or self.link is None:
                raise PermissionError("真机联动未开启，指令未发出")
            self.link.set_loaded(loaded, SERVO_IDS)
            return {"loaded": loaded, "servo_ids": list(SERVO_IDS)}


class Handler(BaseHTTPRequestHandler):
    backend: SimBackend = None  # 由 serve() 注入

    def log_message(self, fmt, *args):  # 静音默认访问日志，避免刷屏
        if getattr(self.server, "verbose", False):
            super().log_message(fmt, *args)

    # ------------------------------------------------------------------ 工具
    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: int, payload) -> None:
        self._send(status, json.dumps(payload, ensure_ascii=False).encode("utf-8"), CONTENT_TYPES[".json"])

    def _fail(self, status: int, message: str) -> None:
        self._json(status, {"error": message})

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError(f"请求体不是合法 JSON：{error}") from error

    # ------------------------------------------------------------------ 路由
    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler 约定
        path = urlparse(self.path).path
        try:
            if path in ("/", "/index.html"):
                return self._serve_file(WEB_DIR / "index.html")
            if path.startswith("/vendor/"):
                name = path[len("/vendor/"):]
                if ".." in name or not name:
                    return self._fail(404, f"非法资源路径：{path}")
                return self._serve_file(WEB_DIR / "vendor" / name)
            if path == "/model.json":
                return self._json(200, self.backend.model_json())
            if path.startswith("/meshes/"):
                mesh = self.backend.resolve_mesh(path)
                if mesh is None:
                    return self._fail(404, f"网格不存在或被拒绝：{path}")
                return self._serve_file(mesh)
            if path == "/api/status":
                return self._json(200, self.backend.status())
            if path == "/api/ports":
                return self._json(200, {"ports": list_available_ports()})
        except PermissionError as error:
            return self._fail(403, str(error))
        except ValueError as error:
            return self._fail(400, str(error))
        return self._fail(404, f"没有这个路径：{path}")

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            body = self._body()
            if path == "/api/link":
                if body.get("enabled"):
                    port = (body.get("port") or "").strip()
                    if not port:
                        return self._fail(400, "开启联动必须指定端口")
                    return self._json(200, self.backend.open_link(port, body.get("baud")))
                return self._json(200, self.backend.close_link())
            if path == "/api/read":
                return self._json(200, self.backend.read_snapshot())
            if path == "/api/move":
                targets = body.get("targets") or {}
                if not targets:
                    return self._fail(400, "targets 为空")
                return self._json(200, self.backend.move(targets, int(body.get("duration_ms", 800))))
            if path == "/api/stop":
                return self._json(200, self.backend.stop())
            if path == "/api/load":
                return self._json(200, self.backend.set_load_state(bool(body.get("loaded", True))))
        except PermissionError as error:
            return self._fail(403, str(error))
        except ValueError as error:
            return self._fail(400, str(error))
        except Exception as error:  # noqa: BLE001 - 不让后端异常把连接打断成半截响应
            return self._fail(500, f"{type(error).__name__}: {error}")
        return self._fail(404, f"没有这个接口：{path}")

    def _serve_file(self, file: Path) -> None:
        file = Path(file)
        if not file.exists() or not file.is_file():
            return self._fail(404, f"文件不存在：{file.name}")
        content_type = CONTENT_TYPES.get(file.suffix, CONTENT_TYPES.get(file.suffix.lower(), "application/octet-stream"))
        self._send(200, file.read_bytes(), content_type)


def list_available_ports() -> list:
    from serial.tools import list_ports

    return [
        {"device": item.device, "description": item.description}
        for item in sorted(list_ports.comports(), key=lambda p: p.device)
    ]


def build_backend(urdf_path: str | None = None, serial_config: dict | None = None) -> SimBackend:
    urdf = Path(urdf_path) if urdf_path else find_default_urdf(REPO_ROOT)
    if urdf is None or not Path(urdf).exists():
        raise SystemExit("找不到 URDF 文件，请在 config.local.json 的 sim.urdf_path 里指定")
    return SimBackend(Path(urdf), serial_config or {})


def serve(host: str, port: int, backend: SimBackend, verbose: bool = False) -> ThreadingHTTPServer:
    Handler.backend = backend
    httpd = ThreadingHTTPServer((host, port), Handler)
    httpd.verbose = verbose
    return httpd


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="ArmPi Ultra 仿真 + 真机联动服务")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=None, help="HTTP 端口，默认读配置或 8300")
    parser.add_argument("--urdf", default=None, help="URDF 路径，默认自动找厂商功能包")
    parser.add_argument("--com", default=None, help="记住这个 COM 口（仍需页面里手动开启联动）")
    parser.add_argument("--verbose", action="store_true", help="打印访问日志")
    args = parser.parse_args(argv)

    config = load_config(REPO_ROOT / "config.local.json")
    serial_config = dict(config.get("serial", {}))
    if args.com:
        serial_config["port"] = args.com
    http_port = args.port or int(config.get("service", {}).get("port", 8300))

    backend = build_backend(args.urdf or config.get("sim", {}).get("urdf_path"), serial_config)
    httpd = serve(args.host, http_port, backend, args.verbose)

    print(f"仿真页面：http://{args.host}:{http_port}/")
    print(f"模型：{backend.urdf_path.name}，{len(backend.model.links)} 个 link / {len(backend.model.joints)} 个关节")
    print(f"可用端口：{', '.join(p['device'] for p in list_available_ports()) or '未发现串口'}")
    if backend.payload["defects"]:
        print("URDF 已自动处理的问题：")
        for defect in backend.payload["defects"]:
            print(f"  - {defect}")
    print("真机联动默认关闭，需要在页面里手动开启。Ctrl+C 退出。")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n正在关闭……")
    finally:
        backend.close_link()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""描画サービス `sim_render`（WP-SIM-03）。DetailedDesign-sim.md §5。

hve の偽物（昇降部・カメラ・映像）に観察でつなぎ、今の向きと高さで
`fk_sim.render` の 1 枚を `GET /frame` で返す。あわせて天井の距離を
カメラの偽物の裏口（`POST /api/fake` の `ceiling`・`steady: true`）へ
差し込み、起動直後はヨーの原点合わせ（`yaw_zero`）を 200 が返るまで送り直す。
機体の位置は `pose_source: fixed`（パラメータ）か Gazebo の
`gazebo_ros_state` サービス（`/get_entity_state`）から取る。

このモジュールは ROS 2・ソケット・pyrender を読まない純関数
（天井の距離・観察の `state` の読み取り・四元数からヨー・hello の形）と、
標準ライブラリだけの最小の WebSocket 接続を持つ。ROS 2 のノード・描画の
呼び出しは `main()` の中で遅れて読むので、純関数は ROS なしでも試験できる。
"""

from __future__ import annotations

import base64
import json
import math
import os
import socket
import struct
import threading
import time
import urllib.request

GAZEBO_STATE_SERVICE = "/get_entity_state"
OBSERVE_NAME = "sim-render"


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) \
        and math.isfinite(float(value))


# ---------------------------------------------------------------------------
# 純関数（ROS 2・ソケット・描画を読まない）

def observe_hello(name: str = OBSERVE_NAME) -> dict:
    """観察の接続の `hello`（hve の昇降部・カメラのどちらにも同じ形）。"""
    return {"t": "hello", "name": name, "role": "observe", "operator": "auto", "fw": ""}


def parse_camera_state(msg) -> dict | None:
    """カメラの `state`（観察の接続で届く）→ `{"yaw_deg", "pitch_deg", "yaw_zeroed"}`。

    `t` が `state` でないものは `None`。ヨーは原点の前は `null` で来るので
    `yaw_deg` が `None` のまま残る（描画に使えない向きとして扱う）。
    """
    if not isinstance(msg, dict) or msg.get("t") != "state":
        return None
    yaw = msg.get("yaw_deg")
    pitch = msg.get("pitch_deg")
    return {
        "yaw_deg": float(yaw) if _is_number(yaw) else None,
        "pitch_deg": float(pitch) if _is_number(pitch) else None,
        "yaw_zeroed": msg.get("yaw_zeroed") is True,
    }


def parse_lift_state(msg) -> float | None:
    """昇降部の `state` → `height_mm`（m ではなく mm のまま）。読めなければ `None`。"""
    if not isinstance(msg, dict) or msg.get("t") != "state":
        return None
    height = msg.get("height_mm")
    return float(height) if _is_number(height) else None


def ceiling_distance_mm(ceiling_h_m: float, lift_height_m: float,
                        camera_height_offset_m: float,
                        ceiling_sensor_offset_m: float = 0.0) -> float:
    """天井までの距離（mm）。

    距離 = 天井の高さ −（昇降機の高さ ＋ ss の `camera_height_offset_m`
    ＋ 天井の距離計の差 `ceiling_sensor_offset_m`）。下限は付けない
    （負の値もそのまま返す。呼び出し側が `TOO_NEAR` に直す）。
    """
    return (float(ceiling_h_m) - (float(lift_height_m) + float(camera_height_offset_m)
                                  + float(ceiling_sensor_offset_m))) * 1000.0


def ceiling_payload(distance_mm: float) -> dict:
    """カメラの偽物の裏口（`POST /api/fake`）へ送る `ceiling` の形。

    距離が負（距離計が天井を越している）なら `TOO_NEAR` に直す
    （hve の裏口は `MEASURED` の `mm` に負を受け付けない）。
    """
    if distance_mm < 0:
        return {"status": "TOO_NEAR", "steady": True}
    return {"status": "MEASURED", "mm": int(round(float(distance_mm))), "steady": True}


def ceiling_height_from_scene(scene_dir: str) -> float:
    """シーンの出力（`scene_dir/scene.json`）から天井の高さ（m）を取る。"""
    path = os.path.join(str(scene_dir), "scene.json")
    with open(path, encoding="utf-8") as f:
        answer = json.load(f)
    return float(answer["room"]["ceiling_h"])


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Gazebo の姿勢（四元数）からヨー（rad）を出す。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_from_gazebo(response) -> tuple[float, float, float] | None:
    """`GetEntityState` の応答 → `(x, y, yaw)`。読めなければ `None`。

    本物の応答は `state`（`gazebo_msgs/EntityState`）の中に `pose` を持つ。
    """
    try:
        entity = getattr(response, "state", response)
        pose = entity.pose
        pos, ori = pose.position, pose.orientation
        if not all(_is_number(v) for v in (pos.x, pos.y, ori.x, ori.y, ori.z, ori.w)):
            return None
        return (float(pos.x), float(pos.y),
                yaw_from_quaternion(float(ori.x), float(ori.y),
                                    float(ori.z), float(ori.w)))
    except AttributeError:
        return None


def frame_ready(cam, lift_height_mm, pose) -> bool:
    """`/frame` が 1 枚返せるか（向き・高さ・位置が全部読めているか）。"""
    if not isinstance(cam, dict):
        return False
    if cam.get("yaw_deg") is None or cam.get("pitch_deg") is None:
        return False
    if not _is_number(lift_height_mm) or pose is None:
        return False
    return True


# ---------------------------------------------------------------------------
# 最新値の置き場（観察のスレッドと描画・差し込みのスレッドが共有する）

class StateStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._cam = None
        self._lift_mm = None
        self._pose = None

    def set_cam(self, cam):
        with self._lock:
            self._cam = cam

    def clear_cam(self):
        with self._lock:
            self._cam = None

    def set_lift(self, height_mm):
        with self._lock:
            self._lift_mm = height_mm

    def clear_lift(self):
        with self._lock:
            self._lift_mm = None

    def set_pose(self, x, y, yaw):
        with self._lock:
            self._pose = (float(x), float(y), float(yaw))

    def snapshot(self):
        with self._lock:
            return self._cam, self._lift_mm, self._pose


# ---------------------------------------------------------------------------
# 標準ライブラリだけの最小 WebSocket 接続（観察・試験の操作のどちらにも使う）

class WsTextClient:
    """`ws://HOST:PORT/PATH` へのテキスト接続。送受信は JSON の辞書で渡す。"""

    def __init__(self, hostport: str, path: str = "/ws/module", timeout: float = 5.0):
        host, _, port = str(hostport).partition(":")
        self._sock = socket.create_connection((host, int(port)), timeout=timeout)
        self._sock.settimeout(timeout)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (f"GET {path} HTTP/1.1\r\nHost: {hostport}\r\n"
               "Upgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
        self._sock.sendall(req.encode("ascii"))
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("WebSocket の接続に失敗した")
            buf += chunk
        head = buf.split(b"\r\n\r\n", 1)[0].decode("latin-1")
        if "101" not in head.splitlines()[0]:
            raise ConnectionError(f"WebSocket の接続が拒否された: {head.splitlines()[0]}")
        self._rest = buf.split(b"\r\n\r\n", 1)[1]

    def _read_exact(self, n: int) -> bytes:
        while len(self._rest) < n:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("接続が切れた")
            self._rest += chunk
        out, self._rest = self._rest[:n], self._rest[n:]
        return out

    def _send_frame(self, opcode: int, payload: bytes):
        mask = os.urandom(4)
        header = bytes([0x80 | opcode])
        n = len(payload)
        if n < 126:
            header += bytes([0x80 | n])
        elif n < 65536:
            header += bytes([0x80 | 126]) + struct.pack("!H", n)
        else:
            header += bytes([0x80 | 127]) + struct.pack("!Q", n)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self._sock.sendall(header + mask + masked)

    def send_json(self, obj):
        self._send_frame(0x1, json.dumps(obj).encode("utf-8"))

    def recv_json(self):
        """1 本のテキストメッセージを辞書で返す。切れたら `None`。"""
        chunks = b""
        while True:
            b1, b2 = self._read_exact(2)
            opcode = b1 & 0x0F
            masked = b2 & 0x80
            n = b2 & 0x7F
            if n == 126:
                n = struct.unpack("!H", self._read_exact(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", self._read_exact(8))[0]
            payload = self._read_exact(n)
            if masked:
                mask = self._read_exact(4)
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
            if opcode == 0x9:  # ping → pong
                self._send_frame(0xA, payload)
                continue
            if opcode == 0x8:
                return None
            chunks += payload
            if b1 & 0x80:
                return json.loads(chunks.decode("utf-8"))

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass


def http_post_json(url: str, obj, timeout: float = 2.0):
    """JSON を POST して `(状態, 本文)` を返す。届かなければ例外。"""
    req = urllib.request.Request(
        url, data=json.dumps(obj).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, resp.read()


# ---------------------------------------------------------------------------
# `/frame` の応答（読み取り専用。描画の呼び出しは引数で受ける）

def render_frame(store: StateStore, render_fn) -> bytes | None:
    """今の向き・高さ・位置で 1 枚描く。どれかが読めていなければ `None`（503）。"""
    cam, lift_mm, pose = store.snapshot()
    if not frame_ready(cam, lift_mm, pose):
        return None
    x, y, yaw = pose
    return render_fn(x=x, y=y, yaw=yaw, height_m=float(lift_mm) / 1000.0,
                     cam_yaw_deg=float(cam["yaw_deg"]),
                     cam_pitch_deg=float(cam["pitch_deg"]))


# ---------------------------------------------------------------------------
# ノード（ROS 2 の部分はここだけ。遅れて読む）

def _start_http_server(core, store, render_fn, port: int):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.split("?", 1)[0] != "/frame":
                self.send_error(404)
                return
            try:
                t0 = time.monotonic()
                data = render_frame(store, render_fn)
                if os.environ.get("SIM_RENDER_DEBUG"):
                    cam, lift_mm, pose = store.snapshot()
                    core.log("info",
                             f"/frame cam={cam} lift_mm={lift_mm} pose={pose} "
                             f"bytes={0 if data is None else len(data)} "
                             f"took={time.monotonic() - t0:.2f}s")
            except Exception as exc:  # 描画の失敗は 500 で落とす（黙って 503 にしない）
                core.log("error", f"/frame の描画に失敗した: {exc}")
                self.send_error(500)
                return
            if data is None:
                self.send_error(503)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", int(port)), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


class SimRender:
    """`sim_render` ノードの本体（rclpy の `Node` の代わりに試験しやすく分ける）。"""

    def __init__(self, params: dict, logger=None):
        self.params = dict(params)
        self._logger = logger
        self.store = StateStore()
        self._stop = threading.Event()
        self._threads = []
        self._http_server = None
        self._renderer = None
        self._renderer_lock = threading.Lock()
        self._gz_future = None
        self._gz_future_sent_at = None
        self._gz_client = None

    def log(self, level: str, msg: str):
        if self._logger is not None:
            getattr(self._logger, level)(msg)

    # --- 描画 ---------------------------------------------------------------

    def _render_fn(self, **kwargs):
        with self._renderer_lock:
            if self._renderer is None:
                from fk_sim.render.core import Renderer
                self._renderer = Renderer(self.params["scene_dir"])
            return self._renderer.render(seed=0, **kwargs)

    # --- 観察（カメラ・昇降部） ---------------------------------------------

    def _observe_loop(self, kind: str, hostport: str):
        clear = self.store.clear_cam if kind == "camera" else self.store.clear_lift
        parse = parse_camera_state if kind == "camera" else parse_lift_state
        set_value = self.store.set_cam if kind == "camera" else self.store.set_lift
        while not self._stop.is_set():
            client = None
            try:
                client = WsTextClient(hostport)
                client.send_json(observe_hello())
                while not self._stop.is_set():
                    msg = client.recv_json()
                    if msg is None:
                        break
                    value = parse(msg)
                    if value is not None:
                        set_value(value)
            except Exception as exc:
                self.log("warn", f"{kind} の観察が切れた（繋ぎ直す）: {exc}")
            finally:
                clear()
                if client is not None:
                    client.close()
            self._stop.wait(0.5)

    # --- 天井の差し込み ------------------------------------------------------

    def _ceiling_loop(self, ceiling_h_m: float, camera_height_offset_m: float):
        period = max(0.01, float(self.params["ceiling_period_ms"]) / 1000.0)
        url = f"http://{self.params['camera_host']}/api/fake"
        while not self._stop.is_set():
            _cam, lift_mm, _pose = self.store.snapshot()
            if lift_mm is not None:
                distance = ceiling_distance_mm(
                    ceiling_h_m, float(lift_mm) / 1000.0, camera_height_offset_m,
                    float(self.params["ceiling_sensor_offset_m"]))
                try:
                    http_post_json(url, {"ceiling": ceiling_payload(distance)})
                except Exception:
                    pass  # 次の周期で送り直す
            self._stop.wait(period)

    # --- ヨーの原点合わせ ------------------------------------------------------

    def _yaw_zero_loop(self):
        url = f"http://{self.params['camera_host']}/api/fake"
        while not self._stop.is_set():
            cam, _lift, _pose = self.store.snapshot()
            if cam is not None and cam.get("yaw_zeroed"):
                return
            try:
                status, _body = http_post_json(url, {"yaw_zero": True})
                if status == 200:
                    self.log("info", "ヨーの原点合わせが通った")
            except Exception:
                pass
            self._stop.wait(1.0)

    # --- Gazebo の位置 ---------------------------------------------------------

    def poll_gazebo_once(self):
        """タイマーから呼ぶ。未送信なら送り、終わっていれば位置を取り直す。

        送った通話が 2 秒以上終わらなければ取り消して送り直す（発見が遅れた
        ときに 1 通の通話で止まり続けるのを防ぐ。WP-SIM-04 の実測）。
        """
        if self._gz_client is None:
            return
        now = time.monotonic()
        if self._gz_future is None:
            from gazebo_msgs.srv import GetEntityState
            req = GetEntityState.Request()
            req.name = str(self.params["robot_entity"])
            self._gz_future = self._gz_client.call_async(req)
            self._gz_future_sent_at = now
        elif self._gz_future.done():
            try:
                pose = pose_from_gazebo(self._gz_future.result())
            except Exception as exc:
                self.log("warn", f"Gazebo の位置の読み取りに失敗した: {exc}")
                pose = None
            if pose is not None:
                self.store.set_pose(*pose)
            self._gz_future = None
        elif self._gz_future_sent_at is not None and now - self._gz_future_sent_at > 2.0:
            self._gz_future.cancel()
            self._gz_future = None

    # --- 起動・停止 -------------------------------------------------------------

    def start(self):
        ceiling_h = ceiling_height_from_scene(self.params["scene_dir"])
        from fk_sim.render.core import load_ss_camera
        camera_height_offset = float(load_ss_camera()["height_offset_m"])
        pose_source = str(self.params["pose_source"])
        if pose_source == "fixed":
            self.store.set_pose(float(self.params["fixed_x"]),
                                float(self.params["fixed_y"]),
                                float(self.params["fixed_yaw"]))
        elif pose_source != "gazebo":
            raise ValueError(f"pose_source は gazebo か fixed: {pose_source!r}")

        self._http_server = _start_http_server(
            self, self.store, self._render_fn, int(self.params["frame_port"]))
        jobs = [
            ("camera", lambda: self._observe_loop("camera", str(self.params["camera_host"]))),
            ("lift", lambda: self._observe_loop("lift", str(self.params["lift_host"]))),
            ("ceiling", lambda: self._ceiling_loop(ceiling_h, camera_height_offset)),
            ("yaw-zero", self._yaw_zero_loop),
        ]
        for name, fn in jobs:
            thread = threading.Thread(target=fn, name=f"sim-render-{name}", daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self):
        self._stop.set()
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
        with self._renderer_lock:
            if self._renderer is not None:
                self._renderer.close()
                self._renderer = None


def main():
    import rclpy
    from rclpy.node import Node

    class SimRenderNode(Node):
        def __init__(self):
            super().__init__("sim_render")
            self.declare_parameter("scene_dir", "")
            self.declare_parameter("frame_port", 18100)
            self.declare_parameter("camera_host", "127.0.0.1:18000")
            self.declare_parameter("lift_host", "127.0.0.1:18080")
            self.declare_parameter("pose_source", "fixed")
            self.declare_parameter("robot_entity", "th_robot")
            self.declare_parameter("fixed_x", 0.0)
            self.declare_parameter("fixed_y", 0.0)
            self.declare_parameter("fixed_yaw", 0.0)
            self.declare_parameter("ceiling_period_ms", 150)
            self.declare_parameter("ceiling_sensor_offset_m", 0.0)
            params = {k: self.get_parameter(k).value for k in (
                "scene_dir", "frame_port", "camera_host", "lift_host", "pose_source",
                "robot_entity", "fixed_x", "fixed_y", "fixed_yaw",
                "ceiling_period_ms", "ceiling_sensor_offset_m")}
            if not params["scene_dir"]:
                raise SystemExit("scene_dir が要る（生成器の出力）")
            self.core = SimRender(params, logger=self.get_logger())
            if str(params["pose_source"]) == "gazebo":
                from gazebo_msgs.srv import GetEntityState
                self.core._gz_client = self.create_client(
                    GetEntityState, GAZEBO_STATE_SERVICE)
                self.create_timer(0.2, self.core.poll_gazebo_once)
            self.core.start()

        def destroy_node(self):
            self.core.stop()
            super().destroy_node()

    rclpy.init()
    node = SimRenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 既に落とされている（終了時の二重呼び出し）


if __name__ == "__main__":
    main()

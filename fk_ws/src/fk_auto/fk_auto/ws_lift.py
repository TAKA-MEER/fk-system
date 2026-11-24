"""昇降機の口の本物 WsLiftPort（DetailedDesign-auto.md §2.8）。

名前と形は hve の DetailedDesign-protocol-modules.md §1〜§4 と fake_lift_server.py のものだけを使う:
- つなぎ先 ws://<host>:<port>/ws/module。hello は name ``fk``・role ``control``・
  operator ``auto``・ceiling_sensor ``false``・busy_label「測定操作中」
- 送るのは下げる（hold の dir ``down``）と止める（新しい press の dir ``stop`` のあと release）
  だけ。上げる・goto は入れない
- state の欄: bottom・height_mm・height_ok・homed・owner・owner_name・owner_operator・
  arm{attached,link,confirm,out}

WebSocket のクライアントは標準ライブラリ（socket）だけで書く。新しい依存を増やさない
（コンテナ・ホストのどちらにも websocket のライブラリが入っていないため）。
受信は別スレッドで行い、最新の state と受けた時刻だけを status() に渡す。ROS 2 の
実行器は塞がない。ここだけが時計 (time.monotonic) を読む。
"""
from __future__ import annotations

import base64
import json
import os
import socket
import struct
import threading
import time
from typing import Dict, Optional

from fk_auto import core
from fk_auto.ports import LiftPort

MODULE_NAME = "fk"
HELLO = {
    "t": "hello",
    "name": MODULE_NAME,
    "role": "control",
    "operator": "auto",
    "ceiling_sensor": False,
    "busy_label": "測定操作中",
    "fw": "0.1.0",
}


def _now_ms() -> float:
    return time.monotonic() * 1000.0


def _bool_or_none(v) -> Optional[bool]:
    return v if isinstance(v, bool) else None


# ---------------------------------------------------------------------------
# state の写し方（純関数。DetailedDesign-auto.md §2.8 の表）
# ---------------------------------------------------------------------------

def lift_status_from_state(state: Optional[dict], received_ms: Optional[float],
                           now_ms: float, stale_ms: float) -> Optional[core.LiftStatus]:
    """hve の state の JSON と受けた時刻から core.LiftStatus を作る。

    state が無い・読めない・stale_ms を超えて古いときは None。
    owner_operator が持ち主なし (null) のとき operator_is_human は False
    (None にすると状態機械が人の割り込みのあとの「走行」を受けなくなる)。
    height_ok が偽なら高さは None。arm.out が null なら arm_stowed は None。
    """
    if not isinstance(state, dict) or received_ms is None:
        return None
    if now_ms - received_ms > stale_ms:
        return None
    owner_operator = state.get("owner_operator")
    if owner_operator == "human":
        operator_is_human: Optional[bool] = True
    else:
        # auto・持ち主なし (null)・読めない値は、人の割り込みではない側に倒す
        operator_is_human = False
    height_mm = state.get("height_mm")
    if state.get("height_ok") is True and isinstance(height_mm, (int, float)) \
            and not isinstance(height_mm, bool):
        height: Optional[float] = float(height_mm)
    else:
        height = None
    arm = state.get("arm")
    arm = arm if isinstance(arm, dict) else {}
    out = arm.get("out")
    return core.LiftStatus(
        height_read_ms=received_ms,
        at_bottom=_bool_or_none(state.get("bottom")),
        height_above_bottom_mm=height,
        arm_read_ms=received_ms,
        arm_stowed=(not out) if isinstance(out, bool) else None,
        mechanism_attached=_bool_or_none(arm.get("attached")),
        arm_link_ok=_bool_or_none(arm.get("link")),
        origin_ok=_bool_or_none(state.get("homed")),
        operator_is_human=operator_is_human,
    )


def _owner_is_self(state: Optional[dict]) -> bool:
    return bool(state) and state.get("owner") == "module" \
        and state.get("owner_name") == MODULE_NAME


def _owner_free_or_self(state: Optional[dict]) -> bool:
    """持ち主が無いか、自分 (fk) のときだけ押してよい（§2.8 の奪わない行）。"""
    if not state:
        return False
    return state.get("owner") is None or _owner_is_self(state)


# ---------------------------------------------------------------------------
# 最小の WebSocket クライアント（標準ライブラリだけ。ws:// のみ）
# ---------------------------------------------------------------------------

class _WsConnection:
    def __init__(self, host: str, port: int, path: str = "/ws/module",
                 timeout_s: float = 2.0):
        self._sock = socket.create_connection((host, port), timeout=timeout_s)
        self._sock.settimeout(timeout_s)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (
            f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        self._send_all(req.encode("ascii"))
        header = self._read_until(b"\r\n\r\n")
        if b" 101 " not in header.split(b"\r\n", 1)[0]:
            raise RuntimeError("websocket handshake failed")
        self._buf = b""
        self._sock.settimeout(0.05)

    def _send_all(self, data: bytes) -> None:
        self._sock.sendall(data)

    def _read_until(self, marker: bytes) -> bytes:
        data = b""
        while marker not in data:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise RuntimeError("websocket closed during handshake")
            data += chunk
        head, _, rest = data.partition(marker)
        self._buf = rest
        return head

    def _recv_exact(self, n: int) -> bytes:
        while len(self._buf) < n:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise RuntimeError("websocket closed")
            self._buf += chunk
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def send_text(self, text: str) -> None:
        payload = text.encode("utf-8")
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        n = len(payload)
        if n < 126:
            head = bytes([0x81, 0x80 | n])
        elif n < 65536:
            head = bytes([0x81, 0xFE]) + struct.pack(">H", n)
        else:
            head = bytes([0x81, 0xFF]) + struct.pack(">Q", n)
        self._send_all(head + mask + masked)

    def poll_text(self) -> Optional[str]:
        """届いている text フレームを 1 つ返す。無ければ None。"""
        chunks = []
        while True:
            try:
                b0, b1 = self._recv_exact(2)
            except (socket.timeout, TimeoutError):
                return b"".join(chunks).decode("utf-8", "replace") if chunks else None
            fin = b0 & 0x80
            opcode = b0 & 0x0F
            masked = b1 & 0x80
            length = b1 & 0x7F
            if length == 126:
                length = struct.unpack(">H", self._recv_exact(2))[0]
            elif length == 127:
                length = struct.unpack(">Q", self._recv_exact(8))[0]
            mask = self._recv_exact(4) if masked else None
            payload = self._recv_exact(length) if length else b""
            if mask:
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
            if opcode == 0x8:      # close
                raise RuntimeError("websocket closed by peer")
            if opcode == 0x9:      # ping -> pong（送り手は mask する）
                pong_mask = os.urandom(4)
                masked_pong = bytes(b ^ pong_mask[i % 4] for i, b in enumerate(payload))
                self._send_all(bytes([0x8A, 0x80 | len(payload)]) + pong_mask + masked_pong)
                continue
            if opcode in (0x1, 0x2, 0x0):
                chunks.append(payload)
            if fin:
                return b"".join(chunks).decode("utf-8", "replace") if chunks else None

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# WsLiftPort
# ---------------------------------------------------------------------------

class WsLiftPort(LiftPort):
    """hve の昇降部の口。送るのは下げる・止めるだけ（§2.8）。

    lower() が呼ばれている間、lift_cmd_period_ms ごとに hold (dir down) を送る。
    持ち主が無いか fk のときだけ押す。press は持ち主が無い間に下げ始めるときだけ
    増やす。state が lift_state_stale_ms 届かなければ自分で閉じて hello からやり直す。
    """

    def __init__(self, host: str = "hve-lift.local", port: int = 80,
                 cmd_period_ms: float = 100, state_stale_ms: float = 500,
                 lower_speed_mmps: int = 100):
        self._host, self._port = host, int(port)
        self._period_ms = float(cmd_period_ms)
        self._stale_ms = float(state_stale_ms)
        self._speed = int(lower_speed_mmps)
        self._lock = threading.Lock()
        self._state: Optional[dict] = None
        self._state_ms: Optional[float] = None
        self._lowering = False               # lower() が呼ばれている間だけ True
        self._last_lower_call_ms = 0.0
        self._press = 0
        self._press_active = False           # 今の press で送っている間だけ True
        self._last_hold_ms = 0.0
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="ws-lift", daemon=True)
        self._thread.start()

    # --- LiftPort ---
    def status(self) -> Optional[core.LiftStatus]:
        with self._lock:
            state, received = self._state, self._state_ms
        return lift_status_from_state(state, received, _now_ms(), self._stale_ms)

    def lower(self) -> None:
        with self._lock:
            self._lowering = True
            self._last_lower_call_ms = _now_ms()

    def stop(self) -> None:
        with self._lock:
            self._lowering = False

    def close(self) -> None:
        with self._lock:
            self._closed = True

    def _is_closed(self) -> bool:
        with self._lock:
            return self._closed

    # --- 送る側（受信スレッドから） ---
    def _maybe_send(self, conn: _WsConnection, state: Optional[dict]) -> None:
        now = _now_ms()
        with self._lock:
            lowering = self._lowering
            last_call = self._last_lower_call_ms
            press_active = self._press_active
        # lower() が呼ばれなくなった（数回分）・下端に着いた・stop() されたときは
        # 自分が持ち主なら止めて離す（新しい press の stop のあと release）
        want_down = lowering and (now - last_call) <= 3 * self._period_ms
        at_bottom = bool(state) and state.get("bottom") is True
        if not want_down or at_bottom:
            if press_active and _owner_is_self(state):
                self._stop_and_release(conn)
            with self._lock:
                self._lowering = False
                self._press_active = False
            return
        if not _owner_free_or_self(state):
            # 人・ss・アームの制御器が持っている間は押さない。press も増やさない
            with self._lock:
                self._press_active = False
            return
        if now - self._last_hold_ms < self._period_ms:
            return
        with self._lock:
            if not self._press_active:
                self._press += 1             # 持ち主が無い間に下げ始めるときだけ増やす
                self._press_active = True
            press = self._press
        conn.send_text(json.dumps({"t": "hold", "press": press,
                                   "dir": "down", "speed": self._speed}))
        with self._lock:
            self._last_hold_ms = now

    def _stop_and_release(self, conn: _WsConnection) -> None:
        with self._lock:
            self._press += 1                 # 止めるのも新しい押し始め（§2.1）
            press = self._press
            self._press_active = False
        conn.send_text(json.dumps({"t": "hold", "press": press,
                                   "dir": "stop", "speed": 0}))
        conn.send_text(json.dumps({"t": "release", "press": press}))

    # --- 受信スレッド ---
    def _run(self) -> None:
        while not self._is_closed():
            conn: Optional[_WsConnection] = None
            try:
                conn = _WsConnection(self._host, self._port)
                conn.send_text(json.dumps(HELLO))
                with self._lock:
                    self._press = 0          # press は接続ごと（§1.2）
                    self._press_active = False
                    self._state, self._state_ms = None, None
                while not self._is_closed():
                    text = conn.poll_text()
                    if text is not None:
                        try:
                            msg = json.loads(text)
                        except ValueError:
                            msg = None
                        if isinstance(msg, dict) and msg.get("t") == "state":
                            with self._lock:
                                self._state, self._state_ms = msg, _now_ms()
                    with self._lock:
                        state = self._state
                    self._maybe_send(conn, state)
                    # state が古くなったら自分で閉じて hello からやり直す
                    with self._lock:
                        received = self._state_ms
                    if received is not None and _now_ms() - received > self._stale_ms:
                        break
            except Exception:
                time.sleep(0.05)
            finally:
                if conn is not None:
                    conn.close()
                with self._lock:
                    self._state, self._state_ms = None, None
                    self._press_active = False

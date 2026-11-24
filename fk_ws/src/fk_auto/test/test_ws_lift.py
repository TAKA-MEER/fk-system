"""WsLiftPort の試験（WP-AUTO-05）。

- 写し方の純関数 (lift_status_from_state) はホストで回す（ROS なし・依存なし）。
- hve の偽物 (tools/fake_lift_server.py) を試験の中で起こして WsLiftPort を繋ぐ。
  偽物は aiohttp を要る。無い環境では、hve の .venv の python を使う。
  それも無ければ明示的に skip する（コンテナには python3-aiohttp が入るのでそこで走る）。
"""
import importlib.util
import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.request

import pytest

from fk_auto import core
from fk_auto.ws_lift import WsLiftPort, _WsConnection, lift_status_from_state

FAKE_CANDIDATES = [
    os.environ.get("FAKE_LIFT_SERVER", ""),
    "/root/hve-system/tools/fake_lift_server.py",
    "/home/yisahia787/code/MIRS-Local/TM/hve-system/tools/fake_lift_server.py",
]
HVE_VENV_PYTHON = "/home/yisahia787/code/MIRS-Local/TM/hve-system/.venv/bin/python"


def _fake_server_python():
    if importlib.util.find_spec("aiohttp") is not None:
        return sys.executable
    if os.path.exists(HVE_VENV_PYTHON):
        return HVE_VENV_PYTHON
    return None


FAKE_PATH = next((p for p in FAKE_CANDIDATES if p and os.path.exists(p)), None)
FAKE_PYTHON = _fake_server_python()
needs_fake = pytest.mark.skipif(
    FAKE_PATH is None or FAKE_PYTHON is None,
    reason="hve の偽物 (fake_lift_server.py) か aiohttp が無い環境")


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakeLiftServer:
    def __init__(self):
        self.port = _free_port()
        self.proc = subprocess.Popen(
            [FAKE_PYTHON, FAKE_PATH, "--port", str(self.port)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("fake lift server exited early")
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=0.2):
                    return
            except OSError:
                time.sleep(0.05)
        self.stop()
        raise RuntimeError("fake lift server did not start")

    def post_fake(self, body):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/fake",
            data=json.dumps(body).encode(), method="POST",
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def _wait_until(cond, timeout_s=10.0, interval_s=0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(interval_s)
    return bool(cond())


def _make_port(port):
    return WsLiftPort(host="127.0.0.1", port=port, cmd_period_ms=50,
                      state_stale_ms=500, lower_speed_mmps=255)


class StateReader:
    """試験側の WS 接続。最新の state を読むだけ（hello は送らない）。"""

    def __init__(self, port, path="/ws/module"):
        self.conn = _WsConnection("127.0.0.1", port, path)
        self.state = None

    def pump(self):
        # 届いている分を全部読む（1 つだけ読むとバッファの古い state で遅れる）
        while True:
            text = self.conn.poll_text()
            if text is None:
                break
            msg = json.loads(text)
            if isinstance(msg, dict) and msg.get("t") == "state":
                self.state = msg

    def latest(self, timeout_s=3.0):
        _wait_until(lambda: self._pump() or self.state is not None, timeout_s)
        return self.state

    def _pump(self):
        self.pump()
        return self.state is not None

    def close(self):
        self.conn.close()


# ---------------------------------------------------------------------------
# 写し方の純関数
# ---------------------------------------------------------------------------

def _state(**over):
    s = {"t": "state", "bottom": False, "height_mm": 800, "height_ok": True,
         "homed": True, "owner": "module", "owner_name": "fk",
         "owner_operator": "auto",
         "arm": {"attached": True, "link": True, "confirm": None, "out": False}}
    s.update(over)
    return s


def test_map_every_field():
    st = lift_status_from_state(_state(), 1000.0, 1100.0, 500)
    assert st == core.LiftStatus(
        height_read_ms=1000.0, at_bottom=False, height_above_bottom_mm=800.0,
        arm_read_ms=1000.0, arm_stowed=True, mechanism_attached=True,
        arm_link_ok=True, origin_ok=True, operator_is_human=False)


def test_map_human_owner():
    st = lift_status_from_state(
        _state(owner="ui", owner_name=None, owner_operator="human"), 0.0, 0.0, 500)
    assert st.operator_is_human is True


def test_map_no_owner_is_not_human():
    # 持ち主なし (null) のときは False。None にすると状態機械が「走行」を受けなくなる
    st = lift_status_from_state(
        _state(owner=None, owner_name=None, owner_operator=None), 0.0, 0.0, 500)
    assert st.operator_is_human is False


def test_map_height_not_ok():
    st = lift_status_from_state(_state(height_ok=False), 0.0, 0.0, 500)
    assert st.height_above_bottom_mm is None


def test_map_arm_out_null_and_missing():
    st = lift_status_from_state(
        _state(arm={"attached": None, "link": None, "confirm": None, "out": None}),
        0.0, 0.0, 500)
    assert st.arm_stowed is None
    assert st.arm_link_ok is None
    assert st.mechanism_attached is None
    s = _state()
    del s["arm"]
    st2 = lift_status_from_state(s, 0.0, 0.0, 500)
    assert st2.arm_stowed is None and st2.arm_link_ok is None


def test_map_stale_and_missing():
    assert lift_status_from_state(_state(), 0.0, 501.0, 500) is None
    assert lift_status_from_state(_state(), 0.0, 500.0, 500) is not None
    assert lift_status_from_state(None, 0.0, 0.0, 500) is None
    assert lift_status_from_state(_state(), None, 0.0, 500) is None
    assert lift_status_from_state({"t": "state"}, 0.0, 0.0, 500) is not None


# ---------------------------------------------------------------------------
# hve の偽物相手の結合
# ---------------------------------------------------------------------------

@needs_fake
def test_lower_to_bottom_and_release():
    srv = FakeLiftServer()
    port = _make_port(srv.port)
    reader = StateReader(srv.port)
    try:
        assert _wait_until(lambda: port.status() is not None), "state が来ない"
        assert port.status().height_above_bottom_mm == 800.0
        # 下げ始める（高さが 0 になるまで lower() を送り続ける。状態機械の呼び方）
        def reached_zero():
            port.lower()
            st = port.status()
            return st is not None and st.height_above_bottom_mm == 0.0
        assert _wait_until(reached_zero, timeout_s=20), "下がらない"
        # 偽物は下端を自分で立てない。試験の側で立てる
        srv.post_fake({"bottom": True})
        assert _wait_until(
            lambda: port.status() is not None and port.status().at_bottom is True,
            timeout_s=5), "下端にならない"
        # 昇降機が止まり、fk が持ち主でなくなる
        def released():
            reader.pump()
            return reader.state is not None and reader.state.get("owner") is None
        assert _wait_until(released, timeout_s=5), "持ち主が残っている"
        reader.pump()
        assert reader.state.get("dir") == "stop"
        # もう一度下げられること（press は接続ごとに増え続ける。止めたあとに
        # 同じ press では持ち主を取り返せない）
        srv.post_fake({"height_mm": 800, "bottom": False})
        # post 後の state を読む（古い state の高さ 0 で通らないように）
        assert _wait_until(
            lambda: port.status() is not None
            and port.status().height_above_bottom_mm == 800.0, timeout_s=5)

        def lowering_again():
            port.lower()
            st = port.status()
            return st is not None and st.height_above_bottom_mm <= 750.0
        assert _wait_until(lowering_again, timeout_s=10), "2 度目に下がらない"
    finally:
        port.close()
        reader.close()
        srv.stop()


@needs_fake
def test_human_interrupt_stops_fk():
    srv = FakeLiftServer()
    port = _make_port(srv.port)
    ui = None
    try:
        assert _wait_until(lambda: port.status() is not None)

        def lowering():
            port.lower()
            st = port.status()
            return st is not None and st.height_above_bottom_mm <= 750.0
        assert _wait_until(lowering, timeout_s=10), "下がり始めない"
        # 人の口 (/ws/ui) から別の press で止める。600 ms 途切れると持ち主を失う
        # （人の割り込みが解けたことになる）ので、送り続けながら確かめる
        ui = _WsConnection("127.0.0.1", srv.port, "/ws/ui")
        ui_stop = threading.Event()

        def ui_hold_loop():
            while not ui_stop.is_set():
                ui.send_text(json.dumps({"t": "hold", "press": 1, "dir": "stop", "speed": 0}))
                time.sleep(0.1)

        threading.Thread(target=ui_hold_loop, daemon=True).start()
        assert _wait_until(
            lambda: port.status() is not None and port.status().operator_is_human is True,
            timeout_s=5), "人の割り込みにならない"
        height = port.status().height_above_bottom_mm
        # fk が lower() を送り続けても、人は優先で高さは動かない（押し直さない）
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            port.lower()
            time.sleep(0.05)
        st = port.status()
        assert st is not None and st.height_above_bottom_mm == height
        assert st.operator_is_human is True
        # 人が離したら割り込みは解ける
        ui_stop.set()
        ui.send_text(json.dumps({"t": "release", "press": 1}))
        assert _wait_until(
            lambda: port.status() is not None and port.status().operator_is_human is False,
            timeout_s=5), "人の割り込みが解けない"
    finally:
        port.close()
        if ui is not None:
            ui.close()
        srv.stop()


@needs_fake
def test_ss_holding_is_not_preempted():
    srv = FakeLiftServer()
    port = _make_port(srv.port)
    ss = None
    stop_flag = threading.Event()

    def ss_hold_loop():
        while not stop_flag.is_set():
            ss.send_text(json.dumps({"t": "hold", "press": 1, "dir": "stop", "speed": 0}))
            time.sleep(0.1)

    try:
        assert _wait_until(lambda: port.status() is not None)
        ss = _WsConnection("127.0.0.1", srv.port, "/ws/module")
        ss.send_text(json.dumps({
            "t": "hello", "name": "ss", "role": "control", "operator": "auto",
            "ceiling_sensor": False, "busy_label": "画像認識で操作中", "fw": "0.1.0"}))
        t = threading.Thread(target=ss_hold_loop, daemon=True)
        t.start()
        # ss が持ち主になる
        reader = StateReader(srv.port)
        try:
            def ss_owner():
                reader.pump()
                return reader.state is not None and reader.state.get("owner_name") == "ss"
            assert _wait_until(ss_owner, timeout_s=5), "ss が持ち主にならない"
            # fk が lower() を送っても ss の持ち主は取られない（press を増やさない）
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                port.lower()
                time.sleep(0.05)
            reader.pump()
            assert reader.state.get("owner_name") == "ss", "fk が ss から持ち主を奪った"
        finally:
            reader.close()
    finally:
        stop_flag.set()
        port.close()
        if ss is not None:
            ss.close()
        srv.stop()


@needs_fake
def test_reconnect_after_fake_restart():
    srv = FakeLiftServer()
    port = _make_port(srv.port)
    try:
        assert _wait_until(lambda: port.status() is not None)
        srv.post_fake({"restart": True})
        saw_none = _wait_until(lambda: port.status() is None, timeout_s=5)
        assert saw_none, "再起動のあと status() が None にならない"
        assert _wait_until(lambda: port.status() is not None, timeout_s=15), \
            "繋ぎ直して戻らない"
    finally:
        port.close()
        srv.stop()

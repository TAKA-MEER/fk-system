"""hve の偽物 3 つと `sim_render` の結合試験（描画の像の中でのみ回す）。

`fk_ws/sim/hve_up.sh` と同じ起こし方で、昇降部の偽物（18080）・カメラの
偽物（18000）・映像の偽物（8080）と `sim_render` を起こし、受け入れ条件
3 の (a)〜(d) を確かめる。`pose_source: fixed`、シーンは `public_room`、
機体は盤が見える位置と向き（記述の robot.pose）。

像の外（hve の venv・シーンの生成物が無い）では全部スキップする。
"""

import io
import json
import math
import os
import socket
import subprocess
import sys
import time
import urllib.request

import pytest

HVE_VENV_PYTHON = "/opt/hve-venv/bin/python"
HVE_REPO = os.environ.get("HVE_REPO", "/hve")
SS_REPO = os.environ.get("SS_REPO", "/ss")
SCENE_DIR = os.environ.get("SIM_SCENE_DIR", "/work/sim_out/scenes/public_room")
LIFT = "127.0.0.1:18080"
CAMERA = "127.0.0.1:18000"
VIDEO = "127.0.0.1:8080"
FRAME_PORT = 18100

pytestmark = pytest.mark.skipif(
    not os.path.isfile(HVE_VENV_PYTHON) or not os.path.isdir(HVE_REPO)
    or not os.path.isfile(os.path.join(SCENE_DIR, "scene.glb")),
    reason="描画の像（hve の venv・HVE_REPO・シーンの生成物）の中で回す試験")


def _wait_port(hostport, seconds):
    host, _, port = hostport.partition(":")
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, int(port)), timeout=0.5):
                return
        except OSError:
            time.sleep(0.2)
    raise AssertionError(f"{hostport} が {seconds} s 開きませんでした")


def _start(argv, cwd=None, env=None):
    full_env = dict(os.environ)
    if env:
        full_env.update(env)
    out = subprocess.DEVNULL
    if os.environ.get("SIM_LOGS"):
        out = open(f"/work/.briefs/tmp/stack-{len(argv) % 97}-{int(time.time()) % 100000}.log", "wb")
    return subprocess.Popen(argv, cwd=cwd, env=full_env, stdout=out, stderr=out)


@pytest.fixture(scope="module")
def hve_stack():
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    procs = []
    try:
        procs.append(_start([HVE_VENV_PYTHON,
                             os.path.join(HVE_REPO, "tools", "fake_lift_server.py"),
                             "--port", "18080"]))
        _wait_port(LIFT, 15)
        procs.append(_start(
            [HVE_VENV_PYTHON, "-m", "hve_camera", "--fake", "--lift", "127.0.0.1:18080",
             "--port", "18000"],
            cwd=os.path.join(HVE_REPO, "camera"),
            env={"PYTHONPATH": os.path.join(HVE_REPO, "camera")}))
        _wait_port(CAMERA, 15)
        procs.append(_start(
            [HVE_VENV_PYTHON, "-m", "hve_video", "--fake", "--frame-url",
             f"http://127.0.0.1:{FRAME_PORT}/frame", "--port", "8080"],
            cwd=os.path.join(HVE_REPO, "camera"),
            env={"PYTHONPATH": os.path.join(HVE_REPO, "camera")}))
        _wait_port(VIDEO, 15)
        render = _start(
            [sys.executable, "-m", "fk_sim.sim_render", "--ros-args",
             "-p", f"scene_dir:={SCENE_DIR}", "-p", f"frame_port:={FRAME_PORT}",
             "-p", f"camera_host:={CAMERA}", "-p", f"lift_host:={LIFT}",
             "-p", "pose_source:=fixed",
             "-p", "fixed_x:=6.0", "-p", "fixed_y:=2.5", "-p", "fixed_yaw:=1.5708"],
            env={"ROS_DOMAIN_ID": "78"})
        procs.append(render)
        _wait_port(f"127.0.0.1:{FRAME_PORT}", 15)
        yield {"render": render}
    finally:
        for proc in reversed(procs):
            if proc.poll() is None:
                proc.terminate()
        for proc in procs:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


def _observe_until(hostport, predicate, seconds, what):
    from fk_sim.sim_render import WsTextClient, observe_hello

    client = WsTextClient(hostport)
    client.send_json(observe_hello("sim-test-observe"))
    deadline = time.monotonic() + seconds
    last = None
    try:
        while time.monotonic() < deadline:
            msg = client.recv_json()
            if msg is None:
                break
            last = msg
            if predicate(msg):
                return msg
    finally:
        client.close()
    raise AssertionError(f"{what} が {seconds} s 来ませんでした（最後: {last}）")


def _aim_and_wait(yaw_deg, pitch_deg=0.0, seconds=20.0):
    """ss と同じ道でカメラを向ける。`aim` は着くまで送り続ける。"""
    from fk_sim.sim_render import WsTextClient

    client = WsTextClient(CAMERA)
    client.send_json({"t": "hello", "name": "sim-test", "role": "control",
                      "operator": "auto", "busy_label": "sim-test", "fw": "test"})
    press = int(time.monotonic() * 1000) % 100000000 + 1
    aim = {"t": "aim", "press": press, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg}
    deadline = time.monotonic() + seconds
    last = None
    try:
        while time.monotonic() < deadline:
            client.send_json(aim)
            msg = client.recv_json()
            if msg is None:
                break
            last = msg
            record = msg.get("aim") if isinstance(msg, dict) else None
            # 前の aim の ARRIVED を拾わない（press が自分のものか確かめる）
            if (isinstance(record, dict) and record.get("status") == "ARRIVED"
                    and record.get("press") == press):
                return msg
            time.sleep(0.05)
    finally:
        client.close()
    raise AssertionError(f"yaw {yaw_deg} へ着きませんでした（最後: {last}）")


def _snapshot():
    """`(状態, ヘッダ, 本文)`。503（まだ 1 枚も受けていない）も例外にしない。"""
    try:
        with urllib.request.urlopen(f"http://{VIDEO}/snapshot", timeout=15) as resp:
            headers = dict(resp.headers.items())
            return resp.status, headers, resp.read()
    except urllib.error.HTTPError as error:
        return error.code, {}, b""


def _settled_snapshot(seconds=60.0):
    """映像が今の向きで落ち着いた 1 枚を返す（連続する 2 枚が同じになるまで待つ）。

    描画に 0.3〜3 s かかるので、向けた直後の静止画は古い向きの 1 枚を返すことがある。
    """
    deadline = time.monotonic() + seconds
    prev = None
    while time.monotonic() < deadline:
        status, _headers, data = _snapshot()
        if status == 200 and data:
            if prev is not None and data == prev:
                return data
            prev = data
        time.sleep(0.5)
    raise AssertionError(f"{seconds} s 静止画が落ち着きませんでした")


def _snapshot_changed_from(previous, seconds=120.0):
    """前の 1 枚と違う絵が来るまで待つ（向きを変えたあとの新しい絵）。"""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        status, _headers, data = _snapshot()
        if status == 200 and data and data != previous:
            return data
        time.sleep(0.5)
    raise AssertionError(f"{seconds} s 絵が変わりませんでした")


def _jpeg_image(data):
    from PIL import Image
    import numpy as np
    img = Image.open(io.BytesIO(data))
    return img, np.asarray(img.convert("RGB")).astype(np.int32)


def test_a_yaw_is_zeroed_within_20s(hve_stack):
    state = _observe_until(
        CAMERA, lambda m: m.get("t") == "state" and m.get("yaw_zeroed") is True,
        20.0, "yaw_zeroed")
    assert state["yaw_deg"] is not None


def test_b_snapshot_through_ss_path_and_changes_with_yaw(hve_stack):
    import numpy as np

    _aim_and_wait(0.0)
    first = _settled_snapshot()
    img, arr1 = _jpeg_image(first)
    assert img.size == (1280, 720)
    assert img.format == "JPEG"

    _aim_and_wait(30.0)
    changed = _snapshot_changed_from(first)
    second = _settled_snapshot()
    if second == first:
        second = changed
    _img, arr2 = _jpeg_image(second)
    diff = float(np.abs(arr1 - arr2).mean())
    print(f"[sim] 向きを変えたときの絵の平均差 {diff:.3f}")
    assert diff > 1.0, f"向きを変えても絵が変わらない（平均差 {diff:.3f}）"


def test_c_snapshot_matches_direct_render(hve_stack):
    import numpy as np

    from fk_sim.render.core import Renderer, load_ss_camera

    _aim_and_wait(30.0)
    data = _settled_snapshot()
    _img, via_stack = _jpeg_image(data)
    height_state = _observe_until(
        LIFT, lambda m: m.get("t") == "state" and m.get("height_mm") is not None,
        10.0, "height_mm")
    height_m = float(height_state["height_mm"]) / 1000.0
    camera = load_ss_camera()
    renderer = Renderer(SCENE_DIR, camera=camera)
    try:
        direct = renderer.render(6.0, 2.5, 1.5708, height_m, 30.0, 0.0, seed=0)
    finally:
        renderer.close()
    _img2, direct_arr = _jpeg_image(direct)
    mean_diff = float(np.abs(via_stack - direct_arr).mean())
    print(f"[sim] 直接描画との平均差 {mean_diff:.3f}")
    # 向きを 30 度間違えると平均差が約 3.9 になる（(b) の実測）ので 2.0 で縛る
    assert mean_diff < 2.0, f"平均の差が大きい（{mean_diff:.3f}）"


def test_d_ceiling_fed_while_render_runs_then_stale(hve_stack):
    # 昇降機 800 mm・カメラの高さ 1.82 m・天井 4.0 m → 1380 mm
    def measured(m):
        ceiling = m.get("ceiling") if m.get("t") == "state" else None
        return isinstance(ceiling, dict) and ceiling.get("status") == "MEASURED"

    state = _observe_until(LIFT, measured, 10.0, "天井 MEASURED")
    assert state["ceiling"]["mm"] == 1380, state["ceiling"]

    hve_stack["render"].terminate()
    hve_stack["render"].wait(timeout=10)

    def unreadable(m):
        ceiling = m.get("ceiling") if m.get("t") == "state" else None
        return isinstance(ceiling, dict) and ceiling.get("status") != "MEASURED"

    state = _observe_until(LIFT, unreadable, 5.0, "天井が読めない")
    assert state["ceiling"]["status"] in ("READ_ERROR", None)

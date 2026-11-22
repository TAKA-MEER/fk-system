"""`fk_sim.sim_render` の純関数の試験（ROS 2・ソケット・pyrender なし）。

ホストでも回る（`fk_sim.sim_render` は標準ライブラリだけを先に読む）。
天井の距離の符号・単位、観察の `state` の読み取り、四元数からヨー、
`/frame` が 1 枚返せるかの判定を縛る。
"""

import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fk_sim.sim_render import (  # noqa: E402
    StateStore,
    ceiling_distance_mm,
    ceiling_height_from_scene,
    ceiling_payload,
    frame_ready,
    observe_hello,
    parse_camera_state,
    parse_lift_state,
    pose_from_gazebo,
    render_frame,
    yaw_from_quaternion,
)


# --- 天井の距離 ----------------------------------------------------------

def test_ceiling_distance_sign_and_unit():
    """距離 = 天井 −（昇降機 ＋ カメラの高さ ＋ 距離計の差）。mm に直す。"""
    # public_room: 天井 4.0 m、昇降機 0.8 m、ss の camera_height_offset 1.82 m
    mm = ceiling_distance_mm(4.0, 0.8, 1.82, 0.0)
    assert mm == pytest.approx(1380.0)
    # 距離計の差を足す（カメラの高さより上にある距離計は天井に近づく）
    assert ceiling_distance_mm(4.0, 0.8, 1.82, 0.1) == pytest.approx(1280.0)
    # 昇降機を上げると距離はその分だけ短くなる
    assert ceiling_distance_mm(4.0, 1.3, 1.82, 0.0) == pytest.approx(880.0)


def test_ceiling_distance_can_be_negative():
    """天井を越えた（カメラが天井の上）ときも負の値をそのまま返す（下限を付けない）。"""
    assert ceiling_distance_mm(2.0, 0.8, 1.82, 0.0) == pytest.approx(-620.0)


def test_ceiling_payload_measured():
    assert ceiling_payload(1380.0) == {"status": "MEASURED", "mm": 1380, "steady": True}
    # 四捨五入して整数 mm にする
    assert ceiling_payload(1380.4)["mm"] == 1380
    assert ceiling_payload(1380.6)["mm"] == 1381


def test_ceiling_payload_too_near_when_negative():
    """負の距離は hve の裏口が受け付けないので TOO_NEAR に直す（mm は送らない）。"""
    payload = ceiling_payload(-0.1)
    assert payload == {"status": "TOO_NEAR", "steady": True}
    assert "mm" not in payload


def test_ceiling_height_from_scene(tmp_path):
    scene_dir = tmp_path / "scene"
    scene_dir.mkdir()
    (scene_dir / "scene.json").write_text(
        json.dumps({"room": {"ceiling_h": 4.0}}), encoding="utf-8")
    assert ceiling_height_from_scene(str(scene_dir)) == pytest.approx(4.0)


# --- 観察の state の読み取り ---------------------------------------------

def test_observe_hello_shape():
    hello = observe_hello()
    assert hello["t"] == "hello"
    assert hello["role"] == "observe"
    assert hello["operator"] == "auto"
    assert hello["name"] == "sim-render"


def test_parse_camera_state():
    msg = {"t": "state", "pitch_deg": -5.0, "yaw_deg": 30.0, "yaw_zeroed": True}
    assert parse_camera_state(msg) == {
        "yaw_deg": 30.0, "pitch_deg": -5.0, "yaw_zeroed": True}


def test_parse_camera_state_yaw_unknown_before_zero():
    """原点の前はヨーが null で来る（描画に使えない向きとして残る）。"""
    msg = {"t": "state", "pitch_deg": 0.0, "yaw_deg": None, "yaw_zeroed": False}
    parsed = parse_camera_state(msg)
    assert parsed["yaw_deg"] is None
    assert parsed["yaw_zeroed"] is False


def test_parse_camera_state_rejects_non_state_and_bad_types():
    assert parse_camera_state({"t": "hello"}) is None
    assert parse_camera_state("state") is None
    # 真偽値は数値として読まない（rclpy や JSON の読み違いを拾う）
    parsed = parse_camera_state({"t": "state", "yaw_deg": True, "pitch_deg": "3"})
    assert parsed["yaw_deg"] is None
    assert parsed["pitch_deg"] is None
    assert parsed["yaw_zeroed"] is False


def test_parse_lift_state():
    assert parse_lift_state({"t": "state", "height_mm": 800}) == 800.0
    assert parse_lift_state({"t": "state", "height_mm": 1234.5}) == 1234.5
    assert parse_lift_state({"t": "state"}) is None
    assert parse_lift_state({"t": "hello", "height_mm": 800}) is None
    assert parse_lift_state({"t": "state", "height_mm": True}) is None


# --- Gazebo の四元数からヨー ---------------------------------------------

def test_yaw_from_quaternion_identity_and_quarter_turn():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    assert yaw_from_quaternion(0.0, 0.0, s, c) == pytest.approx(math.pi / 2.0)
    assert yaw_from_quaternion(0.0, 0.0, -s, c) == pytest.approx(-math.pi / 2.0)
    # 180 度回転
    assert abs(yaw_from_quaternion(0.0, 0.0, 1.0, 0.0)) == pytest.approx(math.pi)


class _Vec:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def test_pose_from_gazebo():
    s = math.sin(0.3)
    c = math.cos(0.3)
    resp = _Vec(pose=_Vec(position=_Vec(x=6.0, y=2.5, z=0.0),
                          orientation=_Vec(x=0.0, y=0.0, z=s, w=c)))
    x, y, yaw = pose_from_gazebo(resp)
    assert (x, y) == (6.0, 2.5)
    assert yaw == pytest.approx(0.6)


def test_pose_from_gazebo_bad_response():
    assert pose_from_gazebo(None) is None
    assert pose_from_gazebo(_Vec()) is None
    resp = _Vec(pose=_Vec(position=_Vec(x="6", y=2.5), orientation=_Vec(x=0, y=0, z=0, w=1)))
    assert pose_from_gazebo(resp) is None


# --- /frame が 1 枚返せるか ------------------------------------------------

def test_frame_ready():
    cam = {"yaw_deg": 10.0, "pitch_deg": 0.0, "yaw_zeroed": True}
    assert frame_ready(cam, 800.0, (6.0, 2.5, 1.57)) is True
    assert frame_ready(None, 800.0, (6.0, 2.5, 1.57)) is False
    assert frame_ready({"yaw_deg": None, "pitch_deg": 0.0}, 800.0, (6.0, 2.5, 1.57)) is False
    assert frame_ready({"yaw_deg": 10.0, "pitch_deg": None}, 800.0, (6.0, 2.5, 1.57)) is False
    assert frame_ready(cam, None, (6.0, 2.5, 1.57)) is False
    assert frame_ready(cam, 800.0, None) is False


def test_render_frame_passes_observed_values():
    """呼び出し側: 観察で読んだ向き・高さ・位置をそのまま描画に渡す。"""
    store = StateStore()
    store.set_cam({"yaw_deg": 25.0, "pitch_deg": -10.0, "yaw_zeroed": True})
    store.set_lift(1300.0)
    store.set_pose(1.0, 2.0, 0.7)
    seen = {}

    def render_fn(**kwargs):
        seen.update(kwargs)
        return b"jpeg"

    assert render_frame(store, render_fn) == b"jpeg"
    assert seen == {"x": 1.0, "y": 2.0, "yaw": 0.7, "height_m": 1.3,
                    "cam_yaw_deg": 25.0, "cam_pitch_deg": -10.0}


def test_render_frame_not_ready_returns_none_without_rendering():
    store = StateStore()
    called = []

    def render_fn(**kwargs):
        called.append(kwargs)
        return b"jpeg"

    assert render_frame(store, render_fn) is None
    assert called == []
    # 向きだけ読めて高さが無いときも描かない
    store.set_cam({"yaw_deg": 0.0, "pitch_deg": 0.0, "yaw_zeroed": True})
    store.set_pose(0.0, 0.0, 0.0)
    assert render_frame(store, render_fn) is None
    assert called == []

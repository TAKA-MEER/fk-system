"""Gazebo の道（`pose_source: gazebo`）の試験。

本物の Gazebo はまだ無い（WP-SIM-04）ので、`gazebo_ros_state` の
`/get_entity_state` サービスを偽の ROS 2 サービスで返す。
応答が壊れたら前の位置を保ち、位置が一度も読めなければ `/frame` は 503。
"""

import json
import math
import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
gazebo_msgs = pytest.importorskip("gazebo_msgs")
pytest.importorskip("gazebo_msgs.srv")

from fk_sim.sim_render import SimRender, render_frame  # noqa: E402


def _quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class _FakeGazeboService:
    """`/get_entity_state` の偽物。応答は `good` が True の間だけ正常。"""

    def __init__(self, node, x, y, yaw):
        from gazebo_msgs.srv import GetEntityState

        self._x, self._y, self._yaw = x, y, yaw
        self.good = True
        self._node = node
        self._service = node.create_service(
            GetEntityState, "/get_entity_state", self._handle)

    def _handle(self, request, response):
        if request.name != "th_robot":
            response.success = False
            return response
        if not self.good:
            response.state.pose.orientation.w = float("inf")  # 読めない応答
            response.success = True
            return response
        s, c = _quaternion(self._yaw)
        response.state.pose.position.x = self._x
        response.state.pose.position.y = self._y
        response.state.pose.position.z = 0.0
        response.state.pose.orientation.x = 0.0
        response.state.pose.orientation.y = 0.0
        response.state.pose.orientation.z = s
        response.state.pose.orientation.w = c
        response.success = True
        return response

    def destroy(self):
        self._node.destroy_service(self._service)


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def gazebo_core(ros, tmp_path):
    """`pose_source: gazebo` の本体。子スレッドは起こさない（start しない）。"""
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    scene_dir = tmp_path / "scene"
    scene_dir.mkdir()
    (scene_dir / "scene.json").write_text(
        json.dumps({"room": {"ceiling_h": 4.0}}), encoding="utf-8")
    node = Node("sim_render_test")
    fake = _FakeGazeboService(node, 6.0, 2.5, math.pi / 2.0)
    core = SimRender({
        "scene_dir": str(scene_dir), "frame_port": 0,
        "camera_host": "127.0.0.1:1", "lift_host": "127.0.0.1:1",
        "pose_source": "gazebo", "robot_entity": "th_robot",
        "fixed_x": 0.0, "fixed_y": 0.0, "fixed_yaw": 0.0,
        "ceiling_period_ms": 150, "ceiling_sensor_offset_m": 0.0})
    from gazebo_msgs.srv import GetEntityState
    core._gz_client = node.create_client(GetEntityState, "/get_entity_state")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield core, fake
    finally:
        executor.shutdown()
        node.destroy_node()


def _poll_until_pose(core, seconds=5.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        core.poll_gazebo_once()
        _cam, _lift, pose = core.store.snapshot()
        if pose is not None:
            return pose
        time.sleep(0.02)
    raise AssertionError("Gazebo から位置が読めませんでした")


def test_pose_comes_from_gazebo_service(gazebo_core):
    core, _fake = gazebo_core
    x, y, yaw = _poll_until_pose(core)
    assert (x, y) == (6.0, 2.5)
    assert yaw == pytest.approx(math.pi / 2.0)


def test_frame_uses_gazebo_pose(gazebo_core):
    """呼び出し側: Gazebo の位置を描画に渡す（固定値でない）。"""
    core, _fake = gazebo_core
    _poll_until_pose(core)
    core.store.set_cam({"yaw_deg": 10.0, "pitch_deg": 0.0, "yaw_zeroed": True})
    core.store.set_lift(800.0)
    seen = {}

    def render_fn(**kwargs):
        seen.update(kwargs)
        return b"jpeg"

    assert render_frame(core.store, render_fn) == b"jpeg"
    assert (seen["x"], seen["y"]) == (6.0, 2.5)
    assert seen["yaw"] == pytest.approx(math.pi / 2.0)


def test_bad_response_keeps_previous_pose(gazebo_core):
    core, fake = gazebo_core
    pose = _poll_until_pose(core)
    fake.good = False
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        core.poll_gazebo_once()
        time.sleep(0.02)
    _cam, _lift, kept = core.store.snapshot()
    assert kept == pose


def test_no_pose_yet_means_no_frame(gazebo_core):
    core, _fake = gazebo_core
    # サービスへ問い合わせる前に位置は無い（503 の側）
    assert core.store.snapshot()[2] is None
    core.store.set_cam({"yaw_deg": 0.0, "pitch_deg": 0.0, "yaw_zeroed": True})
    core.store.set_lift(800.0)
    assert render_frame(core.store, lambda **kw: b"jpeg") is None

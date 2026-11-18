"""WP-AUTO-06 結合試験: auto_runner（本物の ss の口 RosSsPort）＋ fake_th ＋ fake_ss を ROS 2 で通す。

既存の結合試験（test_integration_auto.py）と同じ土台で、ss の口だけ本物にする。
fake_ss は MultiThreadedExecutor で回す（アクションの実行が別スレッドで待つため）。
"""
import dataclasses
import math
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from unique_identifier_msgs.msg import UUID

from conftest import FAKE_PARAMS, RUNNER_PARAMS, body_rect, wait_until
from fk_auto.auto_runner import AutoRunner
from fk_auto.fakes import FakeLift, SsResponse
from fk_msgs.msg import AutoState, Layout, PanelResults
from fk_msgs.srv import AutoCommand
from fk_sim.fake_ss import FakeSsNode
from fk_sim.fake_th import FakeThNode
from ss_msgs.msg import CleanupDone

LONG_S = 30.0
PANEL = (3.0, 0.0)


def ids_of(h):
    return [p.panel_id for p in h.layouts[-1].panels] if h.layouts else []


class SsHarness:
    """auto_runner (ros_ss=True) と fake_th と fake_ss を別スレッドで回し、probe で見る。"""

    def __init__(self, ss_version='3', survey_depth=0.6, recheck_depth=None,
                 recheck_measured=True, reject=False, result_delay_ms=200,
                 cleanup_delay_ms=200, fake_params=None):
        self.errors = []
        self.survey_depth = survey_depth
        self.recheck_depth = recheck_depth
        self.recheck_measured = recheck_measured
        self.lift = FakeLift(mechanism_attached=False)
        self.runner = AutoRunner(lift=self.lift, ros_ss=True, pre_step=[self.lift.advance],
                                 parameter_overrides=[Parameter(k, value=v)
                                                      for k, v in RUNNER_PARAMS.items()])
        fp = dict(FAKE_PARAMS, **(fake_params or {}))
        self.fake = FakeThNode(parameter_overrides=[Parameter(k, value=v) for k, v in fp.items()])
        self.ssnode = FakeSsNode(parameter_overrides=[
            Parameter('interface_version', value=ss_version),
            Parameter('result_delay_ms', value=result_delay_ms),
            Parameter('cleanup_delay_ms', value=cleanup_delay_ms),
            Parameter('status_period_ms', value=100)])
        self.ssnode.fake.responder = self._respond
        self.ssnode.fake.reject_requests = reject
        self.probe = Node('fk_test_ss_probe')
        self.states, self.layouts, self.results = [], [], []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.probe.create_subscription(AutoState, '/fk/auto/state', self.states.append, qos)
        self.probe.create_subscription(Layout, '/fk/auto/layout', self.layouts.append, qos)
        self.probe.create_subscription(PanelResults, '/fk/auto/results', self.results.append, qos)
        self.auto_cmd = self.probe.create_client(AutoCommand, '/fk/auto/command')
        self._cleanup_pub = self.probe.create_publisher(
            CleanupDone, '/ss/locate/cleanup_done',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._stop = False
        self.execs = []
        for node, ex in ((self.runner, SingleThreadedExecutor()),
                         (self.fake, SingleThreadedExecutor()),
                         (self.probe, SingleThreadedExecutor()),
                         (self.ssnode, MultiThreadedExecutor())):
            ex.add_node(node)
            t = threading.Thread(target=self._spin, args=(ex,), daemon=True)
            t.start()
            self.execs.append((ex, t))

    def _spin(self, ex):
        while not self._stop:
            try:
                ex.spin_once(timeout_sec=0.02)
            except Exception as e:   # 試験の道具: 死なずに記録する
                self.errors.append(e)

    def _respond(self, req):
        m = self.fake.model
        pose = (m.x, m.y, m.yaw)
        is_recheck = req.azimuth_span_rad < 2 * math.pi - 1e-6
        depth = self.recheck_depth if (is_recheck and self.recheck_depth is not None) \
            else self.survey_depth
        measured = self.recheck_measured if is_recheck else True
        rect = dataclasses.replace(
            body_rect(PANEL[0], PANEL[1], math.pi, pose, d=depth), depth_measured=measured)
        dist = math.hypot(rect.x_m, rect.y_m)
        rel = (math.atan2(rect.y_m, rect.x_m) - req.azimuth_from_rad) % (2 * math.pi)
        visible = dist <= (req.max_distance_m or 5.0) and rel <= req.azimuth_span_rad
        rects = (rect,) if visible else ()
        return SsResponse('completed' if rects else 'no_lugs', rects,
                          (req.azimuth_from_rad,), (req.azimuth_span_rad,))

    @property
    def state(self):
        return self.states[-1] if self.states else None

    @property
    def phase(self):
        return self.states[-1].phase if self.states else None

    @property
    def gate_reason(self):
        return self.states[-1].gate_reason if self.states else None

    def command(self, name, ids=(), timeout_s=5.0):
        assert self.auto_cmd.wait_for_service(timeout_sec=timeout_s)
        req = AutoCommand.Request()
        req.command, req.panel_ids = name, list(ids)
        fut = self.auto_cmd.call_async(req)
        assert wait_until(fut.done, timeout_s), 'auto_runner が応答しない'
        return fut.result().accepted, fut.result().reject_reason

    def enter_auto(self):
        assert wait_until(self.fake.link_ok, 3.0)
        assert self.fake.do(self.fake.model.human_enter_auto)
        assert wait_until(lambda: self.state and self.state.th_mode == 'AUTO_MEASURE', 3.0)

    def to_select(self):
        self.enter_auto()
        assert self.command('auto.begin')[0]
        assert wait_until(lambda: len(ids_of(self)) == 1 and self.phase == 'SELECT', LONG_S), \
            self.state

    def publish_foreign_cleanup(self):
        m = CleanupDone()
        m.header.stamp = self.probe.get_clock().now().to_msg()
        m.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        m.reason = 'completed'
        m.lift_restored = True
        self._cleanup_pub.publish(m)

    def close(self):
        self._stop = True
        for ex, t in self.execs:
            t.join(timeout=2.0)
            ex.shutdown()
        for n in (self.runner, self.fake, self.probe, self.ssnode):
            n.destroy_node()


@pytest.fixture
def make_ss_harness(ros_context):
    made = []

    def make(**kw):
        h = SsHarness(**kw)
        made.append(h)
        return h
    yield make
    for h in made:
        h.close()


def test_layout_is_made_through_ros_ss_port(make_ss_harness):
    """配置図ができる（/fk/auto/layout に盤が出て、depth_measured が fake_ss の指定どおり）。"""
    h = make_ss_harness(survey_depth=0.6)
    h.to_select()
    panel = h.layouts[-1].panels[0]
    assert panel.depth_measured is True and panel.depth_m == pytest.approx(0.6)
    assert h.command('auto.start', ids=ids_of(h))[0]
    assert wait_until(lambda: h.phase == 'FINISH', LONG_S), h.state
    assert not h.errors


def test_fake_ss_rejection_gives_ss_rejected_and_does_not_stall(make_ss_harness):
    """fake_ss が依頼を断る → survey_reason が ss_rejected、止まらない（SELECT に出る）。"""
    h = make_ss_harness(reject=True)
    h.enter_auto()
    assert h.command('auto.begin')[0]
    assert wait_until(lambda: h.phase == 'SELECT', LONG_S), h.state
    assert h.layouts[-1].survey_reason == 'ss_rejected'
    assert h.layouts[-1].panels == []
    assert not h.errors


def test_foreign_cleanup_done_does_not_finish_the_request(make_ss_harness):
    """自分が出していない goal_id の後片付けの完了を受けても、依頼は終わらない。"""
    h = make_ss_harness(result_delay_ms=2500)
    h.enter_auto()
    assert h.command('auto.begin')[0]
    assert wait_until(lambda: len(h.ssnode.fake.requests) >= 1, LONG_S)
    h.publish_foreign_cleanup()
    time.sleep(1.0)
    assert h.phase == 'SURVEY'      # 終わったことになっていない
    assert wait_until(lambda: h.phase == 'SELECT', LONG_S), h.state
    assert len(ids_of(h)) == 1


def test_ss_version_2_does_not_send_the_request(make_ss_harness):
    """fake_ss の版が "2" → 依頼を出さない（gate_reason が state_unreadable）。"""
    h = make_ss_harness(ss_version='2')
    h.enter_auto()
    assert h.command('auto.begin')[0]
    assert wait_until(lambda: h.phase == 'SURVEY' and h.gate_reason == 'state_unreadable',
                      LONG_S), h.state
    time.sleep(0.5)
    assert h.ssnode.fake.requests == []
    assert not h.errors


def test_recheck_depth_measured_false_uses_survey_depth(make_ss_harness):
    """盤前の認識し直しで depth_measured が false（配置図では true）→
    配置図の盤の奥行きは配置図の値のまま、中心は §2.5 の式どおり。"""
    h = make_ss_harness(survey_depth=1.0, recheck_depth=0.86, recheck_measured=False,
                        fake_params=dict(nav_time_ms=300))
    h.to_select()
    assert h.layouts[-1].panels[0].depth_measured is True
    assert h.command('auto.start', ids=ids_of(h))[0]
    assert wait_until(lambda: h.phase == 'FINISH', LONG_S), h.state
    panel = h.layouts[-1].panels[0]
    assert panel.depth_m == pytest.approx(1.0)
    assert panel.depth_measured is True
    expected_x = PANEL[0] - (1.0 - 0.86) / 2 * math.cos(math.pi)
    assert panel.x == pytest.approx(expected_x)
    assert panel.y == pytest.approx(0.0)
    assert not h.errors


def test_ros_ss_port_itself_discards_foreign_cleanup(make_ss_harness):
    """RosSsPort が他人の goal_id の後片付けの完了を捨てること（Runner の絞り込みとは別に）。"""
    from fk_auto.ros_ss import RosSsPort
    h = make_ss_harness(result_delay_ms=800)
    node = Node('fk_ros_ss_port_probe')
    port = RosSsPort(node)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    stop = False

    def spin():
        while not stop:
            ex.spin_once(timeout_sec=0.02)
    t = threading.Thread(target=spin, daemon=True)
    t.start()
    try:
        assert wait_until(lambda: port.status().state == 'IDLE', LONG_S), port.status()
        rid = port.request(-math.pi, 2 * math.pi, 0.0, 10.0)
        assert rid and len(rid) == 32
        h.publish_foreign_cleanup()
        time.sleep(0.5)
        assert port.poll_cleanups() == []       # 他人の完了は溜めない
        got = []
        assert wait_until(lambda: (got.extend(port.poll_cleanups()), bool(got))[1], LONG_S)
        assert [c.request_id for c in got] == [rid]
    finally:
        stop = True
        t.join(timeout=2.0)
        ex.shutdown()
        node.destroy_node()

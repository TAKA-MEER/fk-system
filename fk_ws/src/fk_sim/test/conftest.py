"""結合試験の土台。auto_runner と fake_th を 1 つの pytest プロセスの中で別の実行器（別スレッド）に起こし、
ROS 2（DDS）で通す。ROS_DOMAIN_ID は 78（scripts/test.sh が 10 では回さない）。"""
import os
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import math

from fk_auto import core
from fk_auto.auto_runner import AutoRunner
from fk_auto.fakes import FakeLift, FakeSs, SsResponse
from fk_msgs.msg import AutoState, Layout, PanelResults
from fk_msgs.srv import AutoCommand
from fk_sim.fake_th import FakeThNode
from th_system_msgs.srv import MeasureCommand

# 試験は速く回すため、既定より短くする。待つ上限はここから計算する（固定 sleep にしない）
RUNNER_PARAMS = dict(heartbeat_period_ms=100, runner_stall_ms=300, runner_period_ms=50,
                     state_period_ms=200, th_status_stale_ms=300)
FAKE_PARAMS = dict(measure_link_timeout_ms=600, status_period_ms=50)
MARGIN_S = 1.5   # DDS の発見・スケジューラのゆらぎ用の余裕
ALLOWED_CMDS = {'meas.goal', 'meas.start', 'meas.reroute', 'meas.stop', 'meas.abort', 'meas.finish'}


def wait_until(cond, timeout_s, interval_s=0.02):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(interval_s)
    return bool(cond())


def _overrides(params):
    return [Parameter(k, value=v) for k, v in params.items()]


def body_rect(wx, wy, wyaw, pose, w=1.0, d=0.6):
    """世界の盤（地図の座標）を、th の位置を基準にした機体基準の矩形にする。"""
    dx, dy = wx - pose[0], wy - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return core.SsRect(x_m=c * dx + s * dy, y_m=-s * dx + c * dy,
                       yaw_rad=core.wrap_angle(wyaw - pose[2]), width_m=w, depth_m=d,
                       pos_std_m=0.02, yaw_std_deg=1.0, lugs=4, top_height_m=2.35)


class Harness:
    """auto_runner と fake_th を別の実行器（別スレッド）で回し、probe で /fk/auto/* を見る。

    ports=True: 昇降機（FakeLift。測定機構なし）と ss（FakeSs。panels の盤を返す）を auto_runner に渡す。
    ports=False: 口を渡さない（本番の起動と同じ）。
    実行器のスレッドは例外で死なない（記録して回り続ける）。break_step=True で step の前で例外を出す。
    """

    def __init__(self, fake_version='1', ports=False, panels=((3.0, 0.0), (3.0, 4.0)),
                 fake_params=None, runner_params=None):
        self.panels = list(panels)
        self.errors = []
        self.break_step = False
        self.th_commands = []
        self.lift = self.ss = None
        pre = []
        if ports:
            self.lift = FakeLift(mechanism_attached=False)
            self.ss = FakeSs(responder=self._respond)
            pre = [self.lift.advance, self.ss.advance, self._maybe_break]
        else:
            pre = [self._maybe_break]
        self.runner = AutoRunner(lift=self.lift, ss=self.ss, pre_step=pre,
                                 parameter_overrides=_overrides(dict(RUNNER_PARAMS,
                                                                     **(runner_params or {}))))
        fp = dict(FAKE_PARAMS, interface_version=fake_version, **(fake_params or {}))
        self.fake = FakeThNode(parameter_overrides=_overrides(fp))
        real_submit = self.fake.model.submit

        def logging_submit(cid, cmd, now, **kw):
            self.th_commands.append(cmd)
            return real_submit(cid, cmd, now, **kw)
        self.fake.model.submit = logging_submit
        self.probe = Node('fk_test_probe')
        self.states, self.layouts, self.results = [], [], []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.probe.create_subscription(AutoState, '/fk/auto/state', self.states.append, qos)
        self.probe.create_subscription(Layout, '/fk/auto/layout', self.layouts.append, qos)
        self.probe.create_subscription(PanelResults, '/fk/auto/results', self.results.append, qos)
        self.cmd = self.probe.create_client(MeasureCommand, '/measure/command')
        self.auto_cmd = self.probe.create_client(AutoCommand, '/fk/auto/command')
        self._stop = False
        self.execs = []
        for n in (self.runner, self.fake, self.probe):
            ex = SingleThreadedExecutor()
            ex.add_node(n)
            t = threading.Thread(target=self._spin, args=(ex,), daemon=True)
            t.start()
            self.execs.append((ex, t))

    def _spin(self, ex):
        while not self._stop:
            try:
                ex.spin_once(timeout_sec=0.02)
            except Exception as e:   # 試験の道具: 死なずに記録する
                self.errors.append(e)

    def _maybe_break(self, now_ms):
        if self.break_step:
            raise RuntimeError('step を止める（試験）')

    def _respond(self, req):
        m = self.fake.model
        pose = (m.x, m.y, m.yaw)
        rects = []
        for wx, wy in self.panels:
            r = body_rect(wx, wy, math.pi, pose)
            if math.hypot(r.x_m, r.y_m) > (req.max_distance_m or 5.0):
                continue
            rel = (math.atan2(r.y_m, r.x_m) - req.azimuth_from_rad) % (2 * math.pi)
            if req.azimuth_span_rad >= 2 * math.pi - 1e-6 or rel <= req.azimuth_span_rad:
                rects.append(r)
        return SsResponse('completed' if rects else 'no_lugs', tuple(rects),
                          (req.azimuth_from_rad,), (req.azimuth_span_rad,))

    @property
    def state(self):
        return self.states[-1] if self.states else None

    @property
    def gate_reason(self):
        return self.states[-1].gate_reason if self.states else None

    @property
    def phase(self):
        return self.states[-1].phase if self.states else None

    def command(self, name, ids=(), arm_confirmed=False, timeout_s=5.0):
        """/fk/auto/command を呼ぶ。応答 (accepted, reject_reason)。"""
        assert self.auto_cmd.wait_for_service(timeout_sec=timeout_s)
        req = AutoCommand.Request()
        req.command, req.panel_ids, req.arm_confirmed = name, list(ids), arm_confirmed
        fut = self.auto_cmd.call_async(req)
        assert wait_until(fut.done, timeout_s), 'auto_runner が応答しない'
        return fut.result().accepted, fut.result().reject_reason

    def send_command(self, command_id='t1', command='meas.start', timeout_s=5.0):
        assert self.cmd.wait_for_service(timeout_sec=timeout_s)
        req = MeasureCommand.Request()
        req.command_id = command_id
        req.command = command
        fut = self.cmd.call_async(req)
        assert wait_until(fut.done, timeout_s), 'fake_th が応答しない'
        return fut.result()

    def enter_auto(self):
        """人が自律測定に入る（fake_th）。auto_runner が AUTO_MEASURE を読むまで待つ。"""
        assert wait_until(self.fake.link_ok, MARGIN_S + 1.0)
        assert self.fake.do(self.fake.model.human_enter_auto)
        assert wait_until(lambda: self.state and self.state.th_mode == 'AUTO_MEASURE', MARGIN_S)

    def close(self):
        self._stop = True
        for ex, t in self.execs:
            t.join(timeout=2.0)
            ex.shutdown()
        for n in (self.runner, self.fake, self.probe):
            n.destroy_node()


@pytest.fixture(scope='session', autouse=True)
def ros_context():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '10'), 'ROS_DOMAIN_ID が実機(10)か未設定'
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness():
    h = Harness()
    yield h
    h.close()


@pytest.fixture
def make_harness():
    made = []

    def make(**kw):
        h = Harness(**kw)
        made.append(h)
        return h
    yield make
    for h in made:
        h.close()


@pytest.fixture
def harness_v2():
    h = Harness(fake_version='2')
    yield h
    h.close()

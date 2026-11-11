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

from fk_auto.auto_runner import AutoRunner
from fk_msgs.msg import AutoState
from fk_sim.fake_th import FakeTh
from th_system_msgs.srv import MeasureCommand

# 試験は速く回すため、既定より短くする。待つ上限はここから計算する（固定 sleep にしない）
RUNNER_PARAMS = dict(heartbeat_period_ms=100, runner_stall_ms=300, runner_period_ms=50,
                     state_period_ms=200, th_status_stale_ms=300)
FAKE_PARAMS = dict(measure_link_timeout_ms=600, status_period_ms=50)
MARGIN_S = 1.5   # DDS の発見・スケジューラのゆらぎ用の余裕


def wait_until(cond, timeout_s, interval_s=0.02):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(interval_s)
    return bool(cond())


def _overrides(params):
    return [Parameter(k, value=v) for k, v in params.items()]


class Harness:
    def __init__(self, fake_version='1'):
        self.runner = AutoRunner(parameter_overrides=_overrides(RUNNER_PARAMS))
        self.fake = FakeTh(parameter_overrides=_overrides(dict(FAKE_PARAMS, interface_version=fake_version)))
        self.probe = Node('fk_test_probe')
        self.states = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.probe.create_subscription(AutoState, '/fk/auto/state', self.states.append, qos)
        self.cmd = self.probe.create_client(MeasureCommand, '/measure/command')
        self.execs = []
        for n in (self.runner, self.fake, self.probe):
            ex = SingleThreadedExecutor()
            ex.add_node(n)
            t = threading.Thread(target=ex.spin, daemon=True)
            t.start()
            self.execs.append((ex, t))

    @property
    def gate_reason(self):
        return self.states[-1].gate_reason if self.states else None

    def send_command(self, command_id='t1', command='meas.start', timeout_s=5.0):
        assert self.cmd.wait_for_service(timeout_sec=timeout_s)
        req = MeasureCommand.Request()
        req.command_id = command_id
        req.command = command
        fut = self.cmd.call_async(req)
        assert wait_until(fut.done, timeout_s), 'fake_th が応答しない'
        return fut.result()

    def close(self):
        for ex, t in self.execs:
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
def harness_v2():
    h = Harness(fake_version='2')
    yield h
    h.close()

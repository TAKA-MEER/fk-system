"""偽の ss ノード fake_ss（試験用）。クラスは FakeSsNode。

ss の口（アクション /ss/locate/panels・状態 /ss/locate/status・後片付けの完了
/ss/locate/cleanup_done）を ROS 2 で真似る。振る舞いは fk_auto.fakes.FakeSs に
任せ、このノードは ROS 2 との受け渡しだけ（fake_th が ThModel を使うのと同じ）。

- 状態は 200 ms ごと＋変化時に出す。interface_version はパラメータ
  interface_version（既定 "3"。試験で "2" にして版の食い違いを作れる）。
- 依頼の ID はアクションの goal UUID。FakeSs の内部の ID と対応付けて、
  結果はアクションの結果に、後片付けの完了は CleanupDone に直す。
- 試験用の操作は self.fake (FakeSs) の属性をそのまま使える（default・responder・
  responses・reject_requests・result_delay_ms・cleanup_delay_ms など）。
時刻はここだけが時計 (time.monotonic) を読む。
実行器は MultiThreadedExecutor で回すこと（アクションの実行が別スレッドで待つため）。
"""
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from unique_identifier_msgs.msg import UUID

from fk_auto.fakes import FakeSs
from fk_auto.ports import SsResult
from ss_msgs.action import LocatePanels
from ss_msgs.msg import CleanupDone, LocatorStatus, PanelTop

ACTION_NAME = '/ss/locate/panels'
CLEANUP_TOPIC = '/ss/locate/cleanup_done'
STATUS_TOPIC = '/ss/locate/status'

STATUS_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
CLEANUP_QOS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _ms() -> float:
    return time.monotonic() * 1000.0


class FakeSsNode(Node):
    def __init__(self, **kwargs):
        super().__init__('fake_ss', **kwargs)
        self.declare_parameter('interface_version', '3')
        self.declare_parameter('status_period_ms', 200)
        self.declare_parameter('result_delay_ms', 200)
        self.declare_parameter('cleanup_delay_ms', 200)

        self.lock = threading.RLock()
        self.fake = FakeSs(
            result_delay_ms=float(self.get_parameter('result_delay_ms').value),
            cleanup_delay_ms=float(self.get_parameter('cleanup_delay_ms').value))
        self.ss = self.fake      # 試験で self.ss とも書けるようにする
        self.interface_version = str(self.get_parameter('interface_version').value)
        self._phase_published: Optional[str] = None
        # FakeSs の内部の依頼 ID -> (goal の UUID メッセージ, 結果が出た印)
        self._by_fake_id: Dict[str, UUID] = {}
        self._results: Dict[str, SsResult] = {}
        self._events: Dict[str, threading.Event] = {}

        self._cleanup_pub = self.create_publisher(CleanupDone, CLEANUP_TOPIC, CLEANUP_QOS)
        self._status_pub = self.create_publisher(LocatorStatus, STATUS_TOPIC, STATUS_QOS)
        self._action_server = ActionServer(
            self, LocatePanels, ACTION_NAME, self._execute,
            goal_callback=self._on_goal, handle_accepted_callback=self._on_accepted,
            cancel_callback=self._on_cancel)
        self._timer = self.create_timer(
            self.get_parameter('status_period_ms').value / 1000.0, self._on_timer)

    # --- 試験用: ThModel の操作と同じ形で、ノードの実行器と同じ錠をかけて呼ぶ ---
    def do(self, fn, *args):
        with self.lock:
            return fn(*args)

    # --- アクション ---
    def _on_goal(self, goal_request):
        with self.lock:
            self.fake.advance(_ms())
            if (self.fake.reject_requests or self.fake._state != 'IDLE'
                    or self.fake.external_busy):
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle):
        goal = goal_handle.request
        with self.lock:
            fake_id = self.fake.request(float(goal.azimuth_from_rad),
                                        float(goal.azimuth_span_rad),
                                        float(goal.max_distance_m), float(goal.time_limit_s))
            if fake_id is not None:
                self._by_fake_id[fake_id] = goal_handle.goal_id
                self._events.setdefault(fake_id, threading.Event())
        goal_handle.execute()

    def _on_cancel(self, goal_handle):
        with self.lock:
            for fake_id, uuid_msg in self._by_fake_id.items():
                if bytes(uuid_msg.uuid) == bytes(goal_handle.goal_id.uuid):
                    self.fake.cancel(fake_id)
                    return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def _execute(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid)
        with self.lock:
            fake_id = next((k for k, v in self._by_fake_id.items()
                            if bytes(v.uuid) == goal_id), None)
            event = self._events.setdefault(fake_id, threading.Event()) if fake_id else None
        # 結果が出るまで待つ。タイマーと実行が同じ実行器でも進むよう、待つ間も進める
        while event is not None and not event.wait(0.05):
            with self.lock:
                self.fake.advance(_ms())
                self._drain()
        with self.lock:
            result = self._results.pop(fake_id, None) if fake_id else None
        return self._to_result(result) if result is not None else LocatePanels.Result()


    # --- 周期 ---
    def _on_timer(self):
        with self.lock:
            self.fake.advance(_ms())
            self._drain()
            self._publish_status()

    def _drain(self):
        """FakeSs の結果と後片付けの完了を取り出し、ROS 2 に直す（錠をかけた状態で呼ぶ）。"""
        for r in self.fake.poll_results():
            uuid_msg = self._by_fake_id.get(r.request_id)
            if uuid_msg is None:
                continue
            self._results[r.request_id] = r
            event = self._events.setdefault(r.request_id, threading.Event())
            event.set()
        for c in self.fake.poll_cleanups():
            uuid_msg = self._by_fake_id.pop(c.request_id, None)
            if uuid_msg is None:
                continue
            m = CleanupDone()
            m.header.stamp = self.get_clock().now().to_msg()
            m.goal_id = uuid_msg
            m.reason = c.reason
            m.lift_restored = bool(c.lift_restored)
            self._cleanup_pub.publish(m)

    def _publish_status(self):
        st = self.fake.status()
        phase = st.state if st is not None else None
        self._phase_published = phase
        m = LocatorStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.interface_version = self.interface_version
        m.phase = phase or 'IDLE'
        m.goal_id = next(iter(self._by_fake_id.values()), UUID())
        m.last_reject_reason = ''
        self._status_pub.publish(m)

    # --- 直す ---
    def _to_result(self, r: SsResult):
        out = LocatePanels.Result()
        out.header.stamp = self.get_clock().now().to_msg()
        out.reason = r.reason
        for p in r.panels:
            t = PanelTop()
            t.x_m, t.y_m = float(p.x_m), float(p.y_m)
            t.top_height_m = float(p.top_height_m)
            t.height_measured = False
            t.yaw_rad = float(p.yaw_rad)
            t.width_m, t.depth_m = float(p.width_m), float(p.depth_m)
            t.pos_std_m, t.yaw_std_deg = float(p.pos_std_m), float(p.yaw_std_deg)
            t.lugs = int(p.lugs)
            t.front_known = bool(p.front_known)
            t.front_yaw_rad = float(p.front_yaw_rad)
            t.depth_measured = bool(p.depth_measured)
            out.panels.append(t)
        out.seen_from_rad = [float(v) for v in r.seen_from_rad]
        out.seen_span_rad = [float(v) for v in r.seen_span_rad]
        out.cleanup_limit_s = float(r.cleanup_limit_s) if r.cleanup_limit_s is not None else 0.0
        return out


def main(args=None):
    rclpy.init(args=args)
    node = FakeSsNode()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

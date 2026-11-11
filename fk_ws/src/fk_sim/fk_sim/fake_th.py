"""偽の th ノード fake_th（試験用）。クラスは FakeThNode。

th の約束（th DetailedDesign-measure.md §1.2・§2・§3.1〜§3.4）どおりに振る舞う。
振る舞いは fk_auto.fakes.ThModel（ROS なし）に任せ、このノードは ROS 2 の入れ物だけ。
パラメータは names.md §5.1。

/measure/command の拒否の順は ThModel（WP-AUTO-01 の順を引き継いだ）:
  1. command_id が空 -> bad_argument
  2. 最後に受けた生存確認の版が違う -> version_mismatch
  3. 生存確認が途絶えている -> link_lost
  4. 状態に応じた拒否
版が違う生存確認は受信に数えない（th §3.4）。

時刻は ThModel に引数で渡す。ここだけが時計（time.monotonic）を読む。
試験用の操作は self.model のメソッドをそのまま呼べる（人が自律測定に入る・ジョグ・塞がる・見失う）。
ノードの実行器とは別のスレッドから呼ぶときは self.do(fn, ...) で包む（同じ錠をかける）。
"""
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fk_auto.fakes import ThModel
from th_system_msgs.msg import MeasureHeartbeat, MeasureResult, MeasureStatus
from th_system_msgs.srv import MeasureCommand

QOS_STATUS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_RESULT = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
QOS_HEARTBEAT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
QOS_MAP = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _ms() -> float:
    return time.monotonic() * 1000.0


class FakeThNode(Node):
    def __init__(self, **kwargs):
        super().__init__('fake_th', **kwargs)
        self.declare_parameter('interface_version', '1')
        self.declare_parameter('measure_link_timeout_ms', 1000)
        self.declare_parameter('status_period_ms', 100)
        self.declare_parameter('nav_time_ms', 1000)

        self.lock = threading.RLock()
        self.model = ThModel(
            interface_version=self.get_parameter('interface_version').value,
            measure_link_timeout_ms=self.get_parameter('measure_link_timeout_ms').value,
            status_period_ms=self.get_parameter('status_period_ms').value,
            nav_time_ms=self.get_parameter('nav_time_ms').value)
        self._map_id_published = None
        self._status_on = True

        self._status_pub = self.create_publisher(MeasureStatus, '/measure/status', QOS_STATUS)
        self._result_pub = self.create_publisher(MeasureResult, '/measure/result', QOS_RESULT)
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', QOS_MAP)
        self.create_subscription(MeasureHeartbeat, '/measure/heartbeat', self._on_heartbeat,
                                 QOS_HEARTBEAT)
        self.create_service(MeasureCommand, '/measure/command', self._on_command)
        self._publish_map()
        self._status_timer = self.create_timer(
            self.get_parameter('status_period_ms').value / 1000.0, self._on_timer)

    # --- 試験用 ---
    def do(self, fn, *args):
        """ThModel の操作を、ノードの実行器と同じ錠をかけて呼ぶ。"""
        with self.lock:
            return fn(*args)

    def link_ok(self) -> bool:
        with self.lock:
            self.model.advance(_ms())
            return self.model.link_ok()

    @property
    def _last_hb_version(self):
        return self.model._last_hb_version

    def stop_status(self):
        """状態の配信を止める（th が落ちた想定）。"""
        self._status_timer.cancel()

    def resume_status(self):
        self._status_timer.reset()

    # --- ROS 2 ---
    def _on_heartbeat(self, msg):
        with self.lock:
            self.model.on_heartbeat(msg.interface_version, _ms())

    def _publish_results(self):
        for r in self.model.drain_results():
            m = MeasureResult()
            m.header.stamp = self.get_clock().now().to_msg()
            m.command_id, m.command, m.result, m.reason = r.command_id, r.command, r.result, r.reason
            self._result_pub.publish(m)

    def _on_timer(self):
        with self.lock:
            self.model.advance(_ms())
            self._publish_results()
            self._publish_map()
            self._publish_status()

    def _publish_status(self):
        mo = self.model
        msg = MeasureStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_version = mo.interface_version
        msg.mode, msg.state = mo.mode, mo.state
        msg.working = False
        msg.link_ok = mo.link_ok()
        msg.pose_valid = mo.pose_valid
        msg.map_instance_id = mo.map_instance_id
        msg.x, msg.y, msg.yaw = mo.x, mo.y, mo.yaw
        msg.pending_command_id = mo.pending[0] if mo.pending else ''
        msg.pending_command = mo.pending[1] if mo.pending else ''
        msg.pause_reason = ''             # ThModel は理由を持たない
        msg.blocked = mo.state == 'BLOCKED'
        msg.reentry_required = mo.reentry_required
        msg.v_limit_mps = 0.0
        msg.w_limit_radps = 0.0
        self._status_pub.publish(msg)

    def _publish_map(self):
        mo = self.model
        if self._map_id_published == mo.map_instance_id:
            return
        self._map_id_published = mo.map_instance_id
        g = mo.grid
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.info.resolution = float(g.resolution)
        m.info.width, m.info.height = g.width, g.height
        m.info.origin.position.x, m.info.origin.position.y = g.origin_x, g.origin_y
        m.info.origin.orientation = Quaternion(w=1.0)
        m.data = [int(v) for v in g.data]
        self._map_pub.publish(m)

    def _on_command(self, request, response):
        with self.lock:
            now = _ms()
            self.model.advance(now)
            self.model.submit(request.command_id, request.command, now,
                              map_instance_id=request.map_instance_id, x=request.x,
                              y=request.y, yaw=request.yaw)
            self.model.advance(now)       # 応答はすぐ返す（サービスの応答。結果は /measure/result）
            replies = [r for r in self.model.drain_replies() if r.command_id == request.command_id]
            self._publish_results()
            r = replies[-1] if replies else None
            response.accepted = bool(r and r.accepted)
            response.reject_reason = r.reject_reason if r else 'bad_argument'
            response.mode, response.state = self.model.mode, self.model.state
            self._publish_status()
            return response


def main(args=None):
    rclpy.init(args=args)
    node = FakeThNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

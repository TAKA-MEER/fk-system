"""偽の th ノード fake_th（試験用）。

th の約束（th DetailedDesign-measure.md §1.2・§2・§3.1・§3.3・§3.4）どおりに振る舞う。
パラメータは names.md §5.1。

/measure/command の拒否の順（th §3.1 は順を決めていないので、ここで決めた順。WP-AUTO-01）:
  1. command_id が空                     -> bad_argument
  2. 最後に受けた生存確認の版が違う       -> version_mismatch
  3. 生存確認が途絶えている               -> link_lost
  4. それ以外はいまは                     -> state_not_accepting
     （状態に応じた受け付けは WP-AUTO-03 で足す）
版が違う生存確認は受信に数えない（th §3.4）ので link_ok にも寄与しない。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from th_system_msgs.msg import MeasureHeartbeat, MeasureResult, MeasureStatus
from th_system_msgs.srv import MeasureCommand

QOS_STATUS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_RESULT = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
QOS_HEARTBEAT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class FakeTh(Node):
    def __init__(self, **kwargs):
        super().__init__('fake_th', **kwargs)
        self.declare_parameter('interface_version', '1')
        self.declare_parameter('measure_link_timeout_ms', 1000)
        self.declare_parameter('status_period_ms', 100)

        # 試験から変えられる（初めは指示を受けない状態: SystemState の IDLE / NONE）
        self.mode = 'IDLE'
        self.state = 'NONE'

        self._last_hb_version = None
        self._last_valid_hb = None   # 版が合った最後の生存確認の受信時刻

        self._status_pub = self.create_publisher(MeasureStatus, '/measure/status', QOS_STATUS)
        self._result_pub = self.create_publisher(MeasureResult, '/measure/result', QOS_RESULT)
        self.create_subscription(MeasureHeartbeat, '/measure/heartbeat', self._on_heartbeat, QOS_HEARTBEAT)
        self.create_service(MeasureCommand, '/measure/command', self._on_command)
        self._status_timer = self.create_timer(
            self.get_parameter('status_period_ms').value / 1000.0, self._publish_status)

    @property
    def interface_version(self):
        return self.get_parameter('interface_version').value

    def link_ok(self):
        if self._last_valid_hb is None:
            return False
        timeout = self.get_parameter('measure_link_timeout_ms').value / 1000.0
        return time.monotonic() - self._last_valid_hb <= timeout

    def _on_heartbeat(self, msg):
        self._last_hb_version = msg.interface_version
        if msg.interface_version == self.interface_version:   # 版が違うものは受信に数えない
            self._last_valid_hb = time.monotonic()

    def _publish_status(self):
        msg = MeasureStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_version = self.interface_version
        msg.mode = self.mode
        msg.state = self.state
        msg.link_ok = self.link_ok()
        msg.pose_valid = True
        msg.map_instance_id = 'fake-map'
        self._status_pub.publish(msg)

    # --- 試験用 ---
    def stop_status(self):
        """状態の配信を止める（th が落ちた想定）。"""
        self._status_timer.cancel()

    def resume_status(self):
        self._status_timer.reset()

    def _on_command(self, request, response):
        response.mode = self.mode
        response.state = self.state
        response.accepted = False
        if request.command_id == '':
            response.reject_reason = 'bad_argument'
        elif self._last_hb_version is not None and self._last_hb_version != self.interface_version:
            response.reject_reason = 'version_mismatch'
        elif not self.link_ok():
            response.reject_reason = 'link_lost'
        else:
            response.reject_reason = 'state_not_accepting'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FakeTh()
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

"""ノード auto_runner（WP-AUTO-01 の骨）。

正本: docs/plan/detailed/DetailedDesign-auto.md §1.2。
状態機械のタイマーと生存確認のタイマーを、1 つのシングルスレッドの実行器で回す。
生存確認は、状態機械が最後に回った時刻が runner_stall_ms 以内のときだけ出す
（判断が止まれば生存確認も止まり、th が一時停止に落とす）。
生存確認を別のノード・スレッドで出してはいけない。

この版は th へ指示を出さない（/measure/command を呼ぶのは WP-AUTO-03 から）。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fk_msgs.msg import AutoState
from th_system_msgs.msg import MeasureHeartbeat, MeasureResult, MeasureStatus

# 取り込んだ th の型の版（names.md §2）。fk_ws/th_ref を進めるときに見直す。
TH_INTERFACE_VERSION = '1'
SENDER = 'fk-system'

# QoS: th DetailedDesign-measure.md §2 / names.md §3
QOS_STATUS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_RESULT = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
QOS_HEARTBEAT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
QOS_AUTO_STATE = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AutoRunner(Node):
    def __init__(self, **kwargs):   # kwargs: 試験からの parameter_overrides など
        super().__init__('auto_runner', **kwargs)
        self.declare_parameter('heartbeat_period_ms', 200)
        self.declare_parameter('runner_stall_ms', 500)
        self.declare_parameter('runner_period_ms', 100)
        self.declare_parameter('state_period_ms', 500)
        self.declare_parameter('th_status_stale_ms', 500)

        self._stall_s = self._ms('runner_stall_ms')
        self._stale_s = self._ms('th_status_stale_ms')

        self._seq = 0
        self._last_run = time.monotonic()   # 状態機械が最後に回った時刻
        self._th_status = None              # 最後に受けた MeasureStatus
        self._th_status_at = None           # その受信時刻
        self._th_result = None
        self._th_result_at = None
        self._last_state_key = None

        self._hb_pub = self.create_publisher(MeasureHeartbeat, '/measure/heartbeat', QOS_HEARTBEAT)
        self._state_pub = self.create_publisher(AutoState, '/fk/auto/state', QOS_AUTO_STATE)
        self.create_subscription(MeasureStatus, '/measure/status', self._on_status, QOS_STATUS)
        self.create_subscription(MeasureResult, '/measure/result', self._on_result, QOS_RESULT)

        self._runner_timer = self.create_timer(self._ms('runner_period_ms'), self._run_state_machine)
        self._hb_timer = self.create_timer(self._ms('heartbeat_period_ms'), self._on_heartbeat_timer)
        self._state_timer = self.create_timer(self._ms('state_period_ms'), self._publish_state)

    def _ms(self, name):
        return self.get_parameter(name).value / 1000.0

    # --- 試験用: 状態機械だけを止める／戻す（実行器は動いたまま） ---
    def stop_state_machine(self):
        self._runner_timer.cancel()

    def resume_state_machine(self):
        self._runner_timer.reset()

    # --- 購読 ---
    def _on_status(self, msg):
        self._th_status = msg
        self._th_status_at = time.monotonic()
        self._publish_if_changed()

    def _on_result(self, msg):
        self._th_result = msg
        self._th_result_at = time.monotonic()

    # --- 状態機械（いまは中身が空。回った時刻を記録するだけ） ---
    def _run_state_machine(self):
        self._last_run = time.monotonic()
        self._publish_if_changed()

    # --- 生存確認 ---
    def _on_heartbeat_timer(self):
        # 状態機械が runner_stall_ms 以内に回っていなければ出さない（th が途絶とみなす）
        if time.monotonic() - self._last_run > self._stall_s:
            return
        msg = MeasureHeartbeat()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_version = TH_INTERFACE_VERSION
        msg.sender = SENDER
        self._seq += 1
        msg.seq = self._seq
        self._hb_pub.publish(msg)

    # --- /fk/auto/state ---
    def _gate_reason(self):
        st = self._th_status
        if st is None or time.monotonic() - self._th_status_at > self._stale_s:
            return 'state_unreadable'
        if st.interface_version != TH_INTERFACE_VERSION:
            return 'version_mismatch'
        return ''

    def _build_state(self):
        msg = AutoState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.phase = 'IDLE'
        st = self._th_status
        if st is not None:
            msg.th_mode = st.mode
            msg.th_state = st.state
            msg.reentry_required = st.reentry_required
        msg.gate_reason = self._gate_reason()
        # 昇降機・腕の欄はまだ口が無いので false
        msg.lift_at_bottom = False
        msg.lift_readable = False
        msg.arm_stowed = False
        msg.mechanism_attached = False
        return msg

    @staticmethod
    def _key(msg):
        return (msg.phase, msg.panel_id, tuple(msg.queue), msg.halt_reason, msg.gate_reason,
                msg.lift_at_bottom, msg.lift_readable, msg.arm_stowed, msg.mechanism_attached,
                msg.reentry_required, msg.th_mode, msg.th_state)

    def _publish_state(self):
        msg = self._build_state()
        self._last_state_key = self._key(msg)
        self._state_pub.publish(msg)

    def _publish_if_changed(self):
        msg = self._build_state()
        key = self._key(msg)
        if key != self._last_state_key:
            self._last_state_key = key
            self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoRunner()
    try:
        rclpy.spin(node)   # シングルスレッドの実行器
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

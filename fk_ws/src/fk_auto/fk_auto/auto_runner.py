"""ノード auto_runner（DetailedDesign-auto.md §1.2）。

RosThPort（th の口の ROS 2 の実装）と、Runner（fk_auto.runner。ROS なし）の配線。
状態機械のタイマーと生存確認のタイマーを、1 つのシングルスレッドの実行器で回す。
生存確認は、状態機械が **step を呼び終えた** 最後の時刻が runner_stall_ms 以内のときだけ出す
（判断が止まれば生存確認も止まり、th が一時停止に落とす）。生存確認を別のノード・スレッドで
出してはいけない。

th への指示は Runner の送り口（Runner._emit）からだけ。このファイルのどこからも
RosThPort.send を直接呼ばない。
昇降機と ss の口は、コンストラクタで渡されたときだけ使う（本物はまだ無い。渡さなければ None で、
関門が何も通さない）。
"""
import dataclasses
import math
import time
from typing import List, Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fk_auto import core
from fk_auto.ports import NoMechanism, ThPort, ThReply, ThResult
from fk_auto.runner import Runner
from fk_msgs.msg import AutoState, Layout, Panel, PanelResult, PanelResults
from fk_msgs.srv import AutoCommand
from th_system_msgs.msg import MeasureHeartbeat, MeasureResult, MeasureStatus
from th_system_msgs.srv import MeasureCommand

TH_INTERFACE_VERSION = core.TH_INTERFACE_VERSION
SENDER = 'fk-system'

# QoS: th DetailedDesign-measure.md §2 / names.md §3
QOS_STATUS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_RESULT = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
QOS_HEARTBEAT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
QOS_MAP = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

# 型は yaml が整数でも小数でも受ける（rclpy は整数と小数を変換しない）
_DYN = ParameterDescriptor(dynamic_typing=True)
# core.Params 以外の names.md §5 の値（ノードが使うもの・今は使わないもの）
_NODE_PARAMS = {
    'heartbeat_period_ms': 200, 'runner_stall_ms': 500, 'runner_period_ms': 100,
    'state_period_ms': 500, 'lift_cmd_period_ms': 100, 'lift_host': 'hve-lift.local',
    'no_lower_move_max_m': 0.0,
}


def now_ms() -> float:
    return time.monotonic() * 1000.0


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RosThPort(ThPort):
    """th の口。/measure/command は非同期で呼び、返事は溜めて poll_replies で渡す（待って塞がない）。"""

    def __init__(self, node: Node):
        self._node = node
        self._status: Optional[core.ThStatus] = None
        self._grid: Optional[core.Grid] = None
        self._replies: List[ThReply] = []
        self._results: List[ThResult] = []
        self._unsent: List[MeasureCommand.Request] = []     # サービスがまだ見えないあいだ溜める
        node.create_subscription(MeasureStatus, '/measure/status', self._on_status, QOS_STATUS)
        node.create_subscription(MeasureResult, '/measure/result', self._on_result, QOS_RESULT)
        node.create_subscription(OccupancyGrid, '/map', self._on_map, QOS_MAP)
        self._cli = node.create_client(MeasureCommand, '/measure/command')

    # --- 購読 ---
    def _on_status(self, m: MeasureStatus) -> None:
        self._status = core.ThStatus(
            read_ms=now_ms(), interface_version=m.interface_version, mode=m.mode, state=m.state,
            pose_valid=m.pose_valid, map_instance_id=m.map_instance_id, x=m.x, y=m.y, yaw=m.yaw,
            reentry_required=m.reentry_required)

    def _on_result(self, m: MeasureResult) -> None:
        self._results.append(ThResult(m.command_id, m.command, m.result, m.reason))

    def _on_map(self, m: OccupancyGrid) -> None:
        o = m.info.origin
        self._grid = core.Grid(m.info.resolution, o.position.x, o.position.y, m.info.width,
                               m.info.height, list(m.data), yaw_of(o.orientation))

    # --- ThPort ---
    def status(self):
        return self._status

    def grid(self):
        return self._grid

    def send(self, command_id, command, map_instance_id='', x=0.0, y=0.0, yaw=0.0) -> None:
        req = MeasureCommand.Request()
        req.command_id, req.command, req.map_instance_id = command_id, command, map_instance_id
        req.x, req.y, req.yaw = float(x), float(y), float(yaw)
        self._unsent.append(req)
        self.flush()

    def flush(self) -> None:
        """サービスが見えていれば、溜めた指示を非同期で呼ぶ。"""
        if not self._unsent or not self._cli.service_is_ready():
            return
        pending, self._unsent = self._unsent, []
        for req in pending:
            fut = self._cli.call_async(req)
            fut.add_done_callback(lambda f, cid=req.command_id: self._on_reply(cid, f))

    def _on_reply(self, command_id: str, fut) -> None:
        r = fut.result()
        if r is not None:
            self._replies.append(ThReply(command_id, r.accepted, r.reject_reason, r.mode, r.state))

    def poll_replies(self):
        out, self._replies = self._replies, []
        return out

    def poll_results(self):
        out, self._results = self._results, []
        return out


class AutoRunner(Node):
    def __init__(self, lift=None, ss=None, arm=None, pre_step=(), **kwargs):
        """lift・ss: 昇降機・ss の口（渡されなければ None）。pre_step: step の前に now_ms を渡して
        呼ぶ関数（試験で偽の口の時刻を進める）。kwargs: parameter_overrides など。"""
        super().__init__('auto_runner', **kwargs)
        defaults = core.Params()
        for f in dataclasses.fields(core.Params):
            if f.name != 'th_interface_version':
                self.declare_parameter(f.name, getattr(defaults, f.name), _DYN)
        for k, v in _NODE_PARAMS.items():
            self.declare_parameter(k, v, _DYN)

        self._stall_s = self._ms('runner_stall_ms')
        self._pre_step = list(pre_step)
        self._seq = 0
        self._last_run = time.monotonic()   # 状態機械が最後に step を呼び終えた時刻
        self._last_state_key = None
        self._last_layout_key = None
        self._last_results_key = None

        self._th = RosThPort(self)
        self._cp = self._params()
        self.runner = Runner(self._th, lift, ss, arm if arm is not None else NoMechanism(self._cp),
                             self._cp)

        self._hb_pub = self.create_publisher(MeasureHeartbeat, '/measure/heartbeat', QOS_HEARTBEAT)
        self._state_pub = self.create_publisher(AutoState, '/fk/auto/state', QOS_LATCHED)
        self._layout_pub = self.create_publisher(Layout, '/fk/auto/layout', QOS_LATCHED)
        self._results_pub = self.create_publisher(PanelResults, '/fk/auto/results', QOS_LATCHED)
        self.create_service(AutoCommand, '/fk/auto/command', self._on_command)

        self._runner_timer = self.create_timer(self._ms('runner_period_ms'), self._run_state_machine)
        self._hb_timer = self.create_timer(self._ms('heartbeat_period_ms'), self._on_heartbeat_timer)
        self._state_timer = self.create_timer(self._ms('state_period_ms'), self._publish_state)

    def _ms(self, name):
        return self.get_parameter(name).value / 1000.0

    def _params(self) -> core.Params:
        d = core.Params()
        kw = {}
        for f in dataclasses.fields(core.Params):
            if f.name == 'th_interface_version':
                continue
            v = self.get_parameter(f.name).value
            kw[f.name] = int(v) if isinstance(getattr(d, f.name), int) else float(v)
        return core.Params(**kw)

    # --- 試験用: 状態機械だけを止める／戻す（実行器は動いたまま） ---
    def stop_state_machine(self):
        self._runner_timer.cancel()

    def resume_state_machine(self):
        self._runner_timer.reset()

    # --- 状態機械 ---
    def _run_state_machine(self):
        t = now_ms()
        for fn in self._pre_step:
            fn(t)
        self.runner.step(t)             # 例外は握りつぶさない
        self._th.flush()
        self._last_run = time.monotonic()   # step を呼び終えてから記録する
        self._publish_if_changed()

    def _on_command(self, request, response):
        r = self.runner.command(request.command, list(request.panel_ids), request.arm_confirmed)
        response.accepted, response.reject_reason = r.accepted, r.reject_reason
        self._publish_if_changed()
        return response

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

    # --- 出す ---
    def _th_gate_reason(self) -> str:
        """Runner が何も送ろうとしていないあいだも、th の様子が使えないことを画面に出す。"""
        st = self._th.status()
        if st is None or now_ms() - st.read_ms > self._cp.th_status_stale_ms:
            return 'state_unreadable'
        if st.interface_version != TH_INTERFACE_VERSION:
            return 'version_mismatch'
        return ''

    def _build_state(self):
        v = self.runner.get_state()
        m = AutoState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.phase, m.panel_id, m.queue = v.phase, v.panel_id, list(v.queue)
        m.halt_reason = v.halt_reason
        m.gate_reason = v.gate_reason or self._th_gate_reason()
        m.lift_at_bottom, m.lift_readable = v.lift_at_bottom, v.lift_readable
        m.arm_stowed, m.mechanism_attached = v.arm_stowed, v.mechanism_attached
        m.reentry_required = v.reentry_required
        m.th_mode, m.th_state, m.th_reject_reason = v.th_mode, v.th_state, v.th_reject_reason
        return m

    @staticmethod
    def _state_key(m):
        return (m.phase, m.panel_id, tuple(m.queue), m.halt_reason, m.gate_reason,
                m.lift_at_bottom, m.lift_readable, m.arm_stowed, m.mechanism_attached,
                m.reentry_required, m.th_mode, m.th_state, m.th_reject_reason)

    def _publish_state(self):
        m = self._build_state()
        self._last_state_key = self._state_key(m)
        self._state_pub.publish(m)

    def _publish_if_changed(self):
        m = self._build_state()
        key = self._state_key(m)
        if key != self._last_state_key:
            self._last_state_key = key
            self._state_pub.publish(m)
        lv = self.runner.get_layout()
        lkey = (lv.map_instance_id, tuple(lv.panels), tuple(lv.unseen_from_rad),
                tuple(lv.unseen_to_rad), lv.survey_reason)
        if lkey != self._last_layout_key:
            self._last_layout_key = lkey
            lm = Layout()
            lm.header.stamp = self.get_clock().now().to_msg()
            lm.map_instance_id, lm.survey_reason = lv.map_instance_id, lv.survey_reason
            lm.unseen_from_rad, lm.unseen_to_rad = list(lv.unseen_from_rad), list(lv.unseen_to_rad)
            for p in lv.panels:
                pm = Panel()
                for f in dataclasses.fields(p):
                    setattr(pm, f.name, getattr(p, f.name))
                lm.panels.append(pm)
            self._layout_pub.publish(lm)
        res = self.runner.get_results()
        rkey = tuple(res)
        if rkey != self._last_results_key:
            self._last_results_key = rkey
            rm = PanelResults()
            rm.header.stamp = self.get_clock().now().to_msg()
            for r in res:
                prm = PanelResult()
                prm.panel_id, prm.result, prm.reason = r.panel_id, r.result, r.reason
                rm.results.append(prm)
            self._results_pub.publish(rm)


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

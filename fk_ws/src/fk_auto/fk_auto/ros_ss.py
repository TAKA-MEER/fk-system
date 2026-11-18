"""ss の口の本物 RosSsPort（DetailedDesign-auto.md §2.5。ss の口の版 3）。

名前と形は ss DetailedDesign-names.md §3・§9a のものだけを使う:
- 状態 /ss/locate/status (ss_msgs/LocatorStatus。reliable・transient_local・depth 1)
- 後片付けの完了 /ss/locate/cleanup_done (ss_msgs/CleanupDone。reliable・transient_local・depth 10)
- 依頼のアクション /ss/locate/panels (ss_msgs/LocatePanels)

ここだけが時計 (time.monotonic) を読む。依頼は待って塞がない: goal の UUID は自分で作り、
send_goal_async(..., goal_uuid=...) で出し、その 16 進を依頼の ID として返す。受け付けの
返事・結果・取り消しはすべてコールバックで受ける。
"""
import time
import uuid
from typing import Dict, List, Optional, Set

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from unique_identifier_msgs.msg import UUID

from fk_auto import core
from fk_auto.ports import SsCleanup, SsPort, SsResult
from ss_msgs.action import LocatePanels
from ss_msgs.msg import CleanupDone, LocatorStatus

SS_INTERFACE_VERSION = "3"
STATUS_TOPIC = '/ss/locate/status'
CLEANUP_TOPIC = '/ss/locate/cleanup_done'
ACTION_NAME = '/ss/locate/panels'

QOS_STATUS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_CLEANUP = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

REJECTED_REASON = 'ss_rejected'


def _now_ms() -> float:
    return time.monotonic() * 1000.0


def _uuid_msg(u: uuid.UUID) -> UUID:
    m = UUID()
    m.uuid = list(u.bytes)
    return m


def _uuid_hex(m: UUID) -> str:
    return bytes(m.uuid).hex()


class RosSsPort(SsPort):
    """ss の口。SsPort の形で、ROS 2 のアクションとトピックで話す。

    - status(): 最後に受けた /ss/locate/status。interface_version が "3" でなければ
      状態は読めない (state None) とする。
    - request(): 最後の状態が IDLE でない・サーバーが見えなければ None。出すときは
      goal の UUID を自分で作ってその 16 進を返す。goal が拒否された・送れなかった
      ときは、その ID の SsResult(reason="ss_rejected") と SsCleanup を溜める。
    - 後片付けの完了は自分が出した ID のものだけを溜める (transient_local で古いものが届く)。
    - 取り消しは、受けてもらう前なら受けてもらったときに送る。
    """

    def __init__(self, node: Node):
        self._node = node
        self._version: Optional[str] = None
        self._phase: Optional[str] = None
        self._status_ms: Optional[float] = None
        self._results: List[SsResult] = []
        self._cleanups: List[SsCleanup] = []
        self._issued: Set[str] = set()                      # 自分が出した依頼の ID
        self._handles: Dict[str, object] = {}               # 受けてもらった goal の handle
        self._pending_cancel: Set[str] = set()              # 受けてもらう前に取り消しを頼まれた ID
        node.create_subscription(LocatorStatus, STATUS_TOPIC, self._on_status, QOS_STATUS)
        node.create_subscription(CleanupDone, CLEANUP_TOPIC, self._on_cleanup, QOS_CLEANUP)
        self._client = ActionClient(node, LocatePanels, ACTION_NAME)

    # --- 購読 ---
    def _on_status(self, m: LocatorStatus) -> None:
        self._version = m.interface_version
        self._phase = m.phase
        self._status_ms = _now_ms()

    def _on_cleanup(self, m: CleanupDone) -> None:
        request_id = _uuid_hex(m.goal_id)
        if request_id not in self._issued:
            return          # 自分が出していない完了は捨てる（transient_local の古いもの）
        self._cleanups.append(SsCleanup(request_id, m.reason, bool(m.lift_restored)))

    # --- SsPort ---
    def status(self) -> Optional[core.SsStatus]:
        if self._status_ms is None or self._version != SS_INTERFACE_VERSION:
            return core.SsStatus(read_ms=self._status_ms, state=None)
        return core.SsStatus(read_ms=self._status_ms, state=self._phase)

    def request(self, azimuth_from_rad, azimuth_span_rad, max_distance_m, time_limit_s):
        st = self.status()
        if st is None or st.state != 'IDLE':
            return None                     # 最後の状態が IDLE でない（読めないのを含む）
        if not self._client.server_is_ready():
            return None                     # サーバーが見えない
        goal_uuid = uuid.uuid4()
        request_id = goal_uuid.hex
        goal = LocatePanels.Goal()
        goal.azimuth_from_rad = float(azimuth_from_rad)
        goal.azimuth_span_rad = float(azimuth_span_rad)
        goal.max_distance_m = float(max_distance_m)
        goal.time_limit_s = float(time_limit_s)
        self._issued.add(request_id)
        try:
            fut = self._client.send_goal_async(goal, goal_uuid=_uuid_msg(goal_uuid))
        except Exception:
            self._reject(request_id)        # 送れない
            return request_id
        fut.add_done_callback(lambda f, rid=request_id: self._on_goal_sent(rid, f))
        return request_id

    def cancel(self, request_id: str) -> None:
        handle = self._handles.get(request_id)
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        else:
            self._pending_cancel.add(request_id)   # 受けてもらう前: 受けてもらったときに取り消す

    def poll_results(self) -> List[SsResult]:
        out, self._results = self._results, []
        return out

    def poll_cleanups(self) -> List[SsCleanup]:
        out, self._cleanups = self._cleanups, []
        return out

    # --- コールバック ---
    def _reject(self, request_id: str) -> None:
        """依頼を断られた・送れなかった: ss は何も動かしていないので、結果と後片付けの
        完了を続けて溜める（DetailedDesign-auto.md §2.5）。"""
        self._results.append(SsResult(request_id, REJECTED_REASON, (), (), (), cleanup_limit_s=0.0))
        self._cleanups.append(SsCleanup(request_id, REJECTED_REASON, lift_restored=True))

    def _on_goal_sent(self, request_id: str, fut) -> None:
        try:
            handle = fut.result()
        except Exception:
            handle = None
        if handle is None:
            self._reject(request_id)        # 送れなかった
            return
        if not handle.accepted:
            self._reject(request_id)        # ss が依頼を断った
            return
        self._handles[request_id] = handle
        if request_id in self._pending_cancel:
            self._pending_cancel.discard(request_id)
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        try:
            result_fut = handle.get_result_async()
        except Exception:
            return          # 結果が取れなかったときは何も溜めない（状態機械の時間切れに任せる）
        result_fut.add_done_callback(lambda f, rid=request_id: self._on_result(rid, f))

    def _on_result(self, request_id: str, fut) -> None:
        try:
            wrapped = fut.result()
        except Exception:
            return          # 取れなかった: 何も溜めない
        if wrapped is None:
            return
        try:
            result = wrapped.result
            panels = tuple(
                core.SsRect(x_m=float(p.x_m), y_m=float(p.y_m), yaw_rad=float(p.yaw_rad),
                            width_m=float(p.width_m), depth_m=float(p.depth_m),
                            pos_std_m=float(p.pos_std_m), yaw_std_deg=float(p.yaw_std_deg),
                            lugs=int(p.lugs), front_known=bool(p.front_known),
                            front_yaw_rad=float(p.front_yaw_rad),
                            top_height_m=float(p.top_height_m),
                            depth_measured=bool(p.depth_measured))
                for p in result.panels)
            self._results.append(SsResult(
                request_id, str(result.reason), panels,
                tuple(float(v) for v in result.seen_from_rad),
                tuple(float(v) for v in result.seen_span_rad),
                cleanup_limit_s=float(result.cleanup_limit_s)))
        except Exception:
            return          # 中身が取れなかった: 何も溜めない

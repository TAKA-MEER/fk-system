"""自律測定の移動の純関数（DetailedDesign-auto.md §3・§4）。

標準ライブラリだけ。ROS 2・通信・ss や th の型には触れない。
入力はこのファイルの dataclass で受ける。分からないものは None（「読めない」として扱う）。
外に出る文字列（段・理由キー・指示）は DetailedDesign-names.md §4 のものだけ。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

TWO_PI = 2.0 * math.pi


def wrap_angle(a: float) -> float:
    """角度を −π〜π に折り返す（π は π のまま、−π は π になる）。"""
    r = math.fmod(a + math.pi, TWO_PI)
    if r <= 0.0:
        r += TWO_PI
    return r - math.pi


def angle_diff(a: float, b: float) -> float:
    """a − b を −π〜π に折り返した値。"""
    return wrap_angle(a - b)


# ---------------------------------------------------------------------------
# 値（DetailedDesign-names.md §5。欄の名前は §5 に揃える）
# ---------------------------------------------------------------------------

TH_INTERFACE_VERSION = "1"


@dataclass(frozen=True)
class Params:
    th_status_stale_ms: float = 500
    lift_state_stale_ms: float = 500
    ss_status_stale_ms: float = 1000
    lift_drift_stop_mm: float = 50
    arrive_tol_m: float = 0.12
    arrive_tol_rad: float = 0.12
    stand_distance_min_m: float = 0.389
    stand_margin_m: float = 0.0
    ss_pos_std_max_m: float = 0.15
    ss_yaw_std_max_deg: float = 5
    ss_pose_tol_m: float = 0.02
    ss_pose_tol_rad: float = 0.02
    row_yaw_tol_rad: float = 0.17
    row_gap_max_m: float = 0.3
    footprint_half_m: float = 0.275
    map_occupied_min: int = 1
    th_interface_version: str = TH_INTERFACE_VERSION
    # 状態機械（runner.py）が使う値（names.md §5。WP-AUTO-03 で足した）
    reroute_max: int = 3
    blocked_wait_s: float = 5
    regoal_max: int = 1
    lift_lower_timeout_s: float = 60
    ss_result_timeout_s: float = 300
    ss_cleanup_timeout_s: float = 60
    ss_max_distance_m: float = 0
    recheck_span_rad: float = 3.1416
    ss_retry_max: int = 2


# ---------------------------------------------------------------------------
# 入力の型
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class SsRect:
    """ss の盤 1 面（PanelTop）。依頼した時点の機体基準（前が x・左が y・角度は反時計回り）。

    yaw_rad は基準の辺（機体側に最も向いていた辺）の外向きの法線。width_m はその辺の長さ、
    depth_m はそれと直交する辺の長さ。
    """
    x_m: float
    y_m: float
    yaw_rad: float
    width_m: float
    depth_m: float
    pos_std_m: float = 0.0
    yaw_std_deg: float = 0.0
    lugs: int = 0
    front_known: bool = False
    front_yaw_rad: float = 0.0
    top_height_m: float = 0.0   # 天面の高さ。to_map は触らない（Runner が順序で対応づける。WP-AUTO-03）


@dataclass(frozen=True)
class MapRect:
    """地図の座標に直した矩形。yaw_rad は基準の辺の外向きの法線（地図の座標）。"""
    x: float
    y: float
    yaw_rad: float
    width_m: float
    depth_m: float
    pos_std_m: float = 0.0
    yaw_std_deg: float = 0.0
    lugs: int = 0
    front_known: bool = False
    front_yaw_rad: float = 0.0  # front_known のとき地図の座標


@dataclass(frozen=True)
class ToMapResult:
    panels: Tuple[MapRect, ...]
    # 見ていない方向（地図の座標。反時計回りに from → to。どちらも −π〜π）
    unseen: Tuple[Tuple[float, float], ...]


@dataclass(frozen=True)
class FrontEdge:
    index: int                 # 0 = 基準の辺。以降、反時計回りに 1〜3
    front_yaw: float           # 正面の向き（盤から外向き。地図の座標）
    half_distance_m: float     # 中心から正面の辺まで
    length_m: float            # 正面の辺の長さ
    front_doubt: bool          # ss の正面と選んだ辺が違う
    ss_front_yaw: Optional[float]  # ss が分かっていると返したときの参考の向き


@dataclass(frozen=True)
class Panel:
    """配置図の盤（回る順・立ち位置の入力）。"""
    panel_id: str
    x: float
    y: float
    front_yaw: float
    front_half_m: float = 0.0    # 中心から正面の辺まで
    front_length_m: float = 0.0  # 正面の辺の長さ


@dataclass(frozen=True)
class Grid:
    """nav_msgs/OccupancyGrid と同じ意味。data は行優先（index = row * width + col）。−1 は分からない。"""
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: Sequence[int]
    origin_yaw: float = 0.0


@dataclass(frozen=True)
class ThStatus:
    """/measure/status から読んだもの。read_ms は読めた時刻。"""
    read_ms: Optional[float] = None
    interface_version: Optional[str] = None
    mode: Optional[str] = None
    state: Optional[str] = None
    pose_valid: bool = False
    map_instance_id: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    reentry_required: bool = False


@dataclass(frozen=True)
class LiftStatus:
    """昇降機の口が読むもの。None は読めない。"""
    height_read_ms: Optional[float] = None   # 高さを読めた時刻
    at_bottom: Optional[bool] = None
    height_above_bottom_mm: Optional[float] = None
    arm_read_ms: Optional[float] = None      # 腕の状態を読めた時刻
    arm_stowed: Optional[bool] = None
    # 測定機構が付いているか。False のときだけ腕の条件を外す。None（分からない）は付いているとみなす
    mechanism_attached: Optional[bool] = None
    arm_link_ok: Optional[bool] = None       # 腕の制御器と通信できるか（None は読めない = できない扱い）
    # 以下は WP-AUTO-03 で足した（None は読めない）
    origin_ok: Optional[bool] = None         # 昇降機の原点が取れているか（PREPARE）
    operator_is_human: Optional[bool] = None  # いま昇降機を操作している元が人か（割り込み）


@dataclass(frozen=True)
class SsStatus:
    read_ms: Optional[float] = None
    state: Optional[str] = None


@dataclass(frozen=True)
class Snapshot:
    """状態機械が集めた「いまの様子」。None は口が無い・読めない。"""
    now_ms: float
    th: Optional[ThStatus] = None
    lift: Optional[LiftStatus] = None
    ss: Optional[SsStatus] = None


@dataclass(frozen=True)
class GateResult:
    allowed: bool
    reason: Optional[str] = None


@dataclass(frozen=True)
class WatchResult:
    stop: bool
    reason: Optional[str] = None


# ---------------------------------------------------------------------------
# to_map・is_confident・front_edge
# ---------------------------------------------------------------------------

def _seen_arcs(seen_from: Sequence[float], seen_span: Sequence[float], yaw0: float):
    """見た範囲を [0, 2π) の弧（開始, 終了）に直す。全周なら None。"""
    arcs: List[Tuple[float, float]] = []
    for f, s in zip(seen_from, seen_span):
        if s <= 0.0:
            continue
        if s >= TWO_PI - 1e-9:
            return None
        start = math.fmod(f + yaw0, TWO_PI)
        if start < 0.0:
            start += TWO_PI
        end = start + s
        if end > TWO_PI:
            arcs.append((start, TWO_PI))
            arcs.append((0.0, end - TWO_PI))
        else:
            arcs.append((start, end))
    return arcs


def unseen_ranges(seen_from: Sequence[float], seen_span: Sequence[float],
                  yaw0: float) -> Tuple[Tuple[float, float], ...]:
    """機体基準で見た範囲（開始・幅）から、地図の座標での見ていない方向（from, to）を求める。

    yaw0 は依頼した時点の機体の向き。重なる範囲・±π をまたぐ範囲・全周に対応する。
    """
    arcs = _seen_arcs(seen_from, seen_span, yaw0)
    if arcs is None:
        return ()
    arcs.sort()
    merged: List[List[float]] = []
    for a, b in arcs:
        if merged and a <= merged[-1][1] + 1e-9:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    if not merged:
        return ((-math.pi, math.pi),)
    gaps: List[Tuple[float, float]] = []
    for i in range(len(merged) - 1):
        gaps.append((merged[i][1], merged[i + 1][0]))
    # 末尾と先頭の隙間は 0 をまたいで 1 つになる
    wrap_start, wrap_end = merged[-1][1], merged[0][0] + TWO_PI
    gaps.append((wrap_start, wrap_end))
    out = []
    for g0, g1 in gaps:
        if g1 - g0 > 1e-9:
            out.append((wrap_angle(g0), wrap_angle(g1)))
    return tuple(out)


def to_map(panels: Sequence[SsRect], request_pose: Pose,
           seen_from_rad: Sequence[float] = (),
           seen_span_rad: Sequence[float] = ()) -> ToMapResult:
    """ss の矩形（依頼した時点の機体基準）を地図の座標に直す。

    あわせて、見た方位角の範囲の残りを「見ていない方向」にする（Spec-auto.md §2・§3）。
    """
    c, s = math.cos(request_pose.yaw), math.sin(request_pose.yaw)
    out = []
    for p in panels:
        out.append(MapRect(
            x=request_pose.x + c * p.x_m - s * p.y_m,
            y=request_pose.y + s * p.x_m + c * p.y_m,
            yaw_rad=wrap_angle(p.yaw_rad + request_pose.yaw),
            width_m=p.width_m, depth_m=p.depth_m,
            pos_std_m=p.pos_std_m, yaw_std_deg=p.yaw_std_deg, lugs=p.lugs,
            front_known=p.front_known,
            front_yaw_rad=wrap_angle(p.front_yaw_rad + request_pose.yaw) if p.front_known else 0.0,
        ))
    return ToMapResult(tuple(out),
                       unseen_ranges(seen_from_rad, seen_span_rad, request_pose.yaw))


def is_confident(pos_std_m: float, yaw_std_deg: float,
                 ss_pos_std_max_m: float, ss_yaw_std_max_deg: float) -> bool:
    """確からしさがしきい値を満たすか（以下なら満たす）。"""
    return pos_std_m <= ss_pos_std_max_m and yaw_std_deg <= ss_yaw_std_max_deg


def _edge_geometry(rect: MapRect, k: int) -> Tuple[float, float, float]:
    """辺 k の（外向きの向き, 中心からの距離, 辺の長さ）。"""
    normal = wrap_angle(rect.yaw_rad + k * math.pi / 2.0)
    if k % 2 == 0:
        return normal, rect.depth_m / 2.0, rect.width_m
    return normal, rect.width_m / 2.0, rect.depth_m


def front_edge(rect: MapRect, observer_xy: Tuple[float, float]) -> FrontEdge:
    """認識したときの機体の位置に向いていた辺を正面にする。

    ss が front_known で返したときも正面は変えず、ss の正面に最も近い辺が選んだ辺と違えば
    front_doubt を立てる。同点は番号の小さい辺。
    """
    dx, dy = observer_xy[0] - rect.x, observer_xy[1] - rect.y
    best_k, best_dot = 0, -math.inf
    for k in range(4):
        n = rect.yaw_rad + k * math.pi / 2.0
        dot = math.cos(n) * dx + math.sin(n) * dy
        if dot > best_dot + 1e-12:
            best_k, best_dot = k, dot
    yaw, half, length = _edge_geometry(rect, best_k)
    doubt = False
    ss_yaw: Optional[float] = None
    if rect.front_known:
        ss_yaw = rect.front_yaw_rad
        ss_k = min(range(4), key=lambda k: abs(angle_diff(ss_yaw, rect.yaw_rad + k * math.pi / 2.0)))
        doubt = ss_k != best_k
    return FrontEdge(best_k, yaw, half, length, doubt, ss_yaw)


def make_panel(panel_id: str, rect: MapRect, edge: FrontEdge) -> Panel:
    return Panel(panel_id, rect.x, rect.y, edge.front_yaw, edge.half_distance_m, edge.length_m)


# ---------------------------------------------------------------------------
# group_rows・order_panels・stand_pose
# ---------------------------------------------------------------------------

def _right_axis(front_yaw: float) -> Tuple[float, float]:
    """正面に向かって右（盤の正面に立って盤を見たときの右手）の単位ベクトル。正面の向き + 90°。"""
    a = front_yaw + math.pi / 2.0
    return math.cos(a), math.sin(a)


def _circ_mean(angles: Sequence[float]) -> float:
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


def group_rows(panels: Sequence[Panel], row_yaw_tol_rad: float,
               row_gap_max_m: float) -> List[List[str]]:
    """盤の一覧 → 列（盤 id の一覧の一覧）。

    正面の向きの差が row_yaw_tol_rad 以内で、横方向の隙間（中心間の横距離から両方の正面の辺の
    半分ずつを引いた値）が row_gap_max_m 以内、かつ正面の面の前後のずれも row_gap_max_m 以内の
    盤どうしを 1 列にする（つながりで推移的に束ねる。前後のずれの条件は設計書に無く、前後に並んだ
    盤を同じ列にしないために足した）。
    列の中は正面に向かって右から。列は先頭の盤の入力順。
    """
    n = len(panels)
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        for j in range(i + 1, n):
            a, b = panels[i], panels[j]
            if abs(angle_diff(a.front_yaw, b.front_yaw)) > row_yaw_tol_rad:
                continue
            rx, ry = _right_axis(_circ_mean([a.front_yaw, b.front_yaw]))
            lateral = abs((b.x - a.x) * rx + (b.y - a.y) * ry)
            gap = lateral - a.front_length_m / 2.0 - b.front_length_m / 2.0
            # 前後に離れた盤（正面の面の位置が row_gap_max_m を超えてずれる）は別の列
            fyaw = _circ_mean([a.front_yaw, b.front_yaw])
            fa = a.x * math.cos(fyaw) + a.y * math.sin(fyaw) + a.front_half_m
            fb = b.x * math.cos(fyaw) + b.y * math.sin(fyaw) + b.front_half_m
            if gap <= row_gap_max_m + 1e-9 and abs(fa - fb) <= row_gap_max_m + 1e-9:
                parent[find(i)] = find(j)
    groups: Dict[int, List[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    rows = []
    for idxs in sorted(groups.values(), key=lambda g: g[0]):
        rows.append(_sort_right_to_left([panels[i] for i in idxs]))
    return [[p.panel_id for p in r] for r in rows]


def _sort_right_to_left(ps: Sequence[Panel]) -> List[Panel]:
    rx, ry = _right_axis(_circ_mean([p.front_yaw for p in ps]))
    return sorted(ps, key=lambda p: -(p.x * rx + p.y * ry))


def order_panels(selected_ids: Sequence[str], rows: Sequence[Sequence[str]],
                 panels: Sequence[Panel], current_xy: Tuple[float, float]) -> List[str]:
    """選ばれた盤 → 回る順。近い列から（列までの距離は選ばれた盤のうち最も近いもの）。

    列の中は正面に向かって右から。列に入っていない盤は 1 枚だけの列として扱う。
    距離が同じ列は入力の順。
    """
    by_id = {p.panel_id: p for p in panels}
    sel = [i for i in selected_ids if i in by_id]
    sel_set = set(sel)
    row_lists: List[List[Panel]] = []
    used = set()
    for r in rows:
        members = [by_id[i] for i in r if i in sel_set and i not in used]
        used.update(i for i in r if i in sel_set)
        if members:
            row_lists.append(members)
    for i in sel:
        if i not in used:
            row_lists.append([by_id[i]])
            used.add(i)

    def dist(ps: Sequence[Panel]) -> float:
        return min(math.hypot(p.x - current_xy[0], p.y - current_xy[1]) for p in ps)

    order = sorted(range(len(row_lists)), key=lambda k: (dist(row_lists[k]), k))
    result: List[str] = []
    for k in order:
        result.extend(p.panel_id for p in _sort_right_to_left(row_lists[k]))
    return result


def stand_pose(panel: Panel, stand_distance_min_m: float, stand_margin_m: float) -> Pose:
    """立ち位置。正面の辺の中央から外へ stand_distance_min_m + stand_margin_m、向きは盤に向く。"""
    d = panel.front_half_m + stand_distance_min_m + stand_margin_m
    return Pose(panel.x + d * math.cos(panel.front_yaw),
                panel.y + d * math.sin(panel.front_yaw),
                wrap_angle(panel.front_yaw + math.pi))


# ---------------------------------------------------------------------------
# stand_blocked・needs_regoal・ss_result_valid
# ---------------------------------------------------------------------------

def stand_blocked(grid: Grid, stand: Pose, footprint_half_m: float,
                  map_occupied_min: int) -> bool:
    """立ち位置に機体が立てないか（True = 塞がっている）。

    立ち位置に、立ち位置の向きに回した一辺 2 × footprint_half_m の正方形を置き、その中に
    中心がある map_occupied_min 以上のマスが 1 つでもあれば塞がっている。
    −1（分からない）と地図の外は塞がっていないとみなす（DetailedDesign-auto.md §3）。
    """
    h = footprint_half_m
    eps = 1e-9
    # 正方形の外接の軸平行な箱（地図の座標）
    ext = h * (abs(math.cos(stand.yaw)) + abs(math.sin(stand.yaw)))
    ext_y = h * (abs(math.sin(stand.yaw)) + abs(math.cos(stand.yaw)))
    gc, gs = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    # 地図の座標 → 格子の座標（原点基準・格子の軸）
    corners = []
    for sx in (-ext, ext):
        for sy in (-ext_y, ext_y):
            wx, wy = stand.x + sx - grid.origin_x, stand.y + sy - grid.origin_y
            corners.append((gc * wx + gs * wy, -gs * wx + gc * wy))
    res = grid.resolution
    c0 = max(0, int(math.floor(min(c[0] for c in corners) / res)) - 1)
    c1 = min(grid.width - 1, int(math.floor(max(c[0] for c in corners) / res)) + 1)
    r0 = max(0, int(math.floor(min(c[1] for c in corners) / res)) - 1)
    r1 = min(grid.height - 1, int(math.floor(max(c[1] for c in corners) / res)) + 1)
    cy, sy_ = math.cos(stand.yaw), math.sin(stand.yaw)
    for row in range(r0, r1 + 1):
        for col in range(c0, c1 + 1):
            v = grid.data[row * grid.width + col]
            if v < 0 or v < map_occupied_min:
                continue
            lx, ly = (col + 0.5) * res, (row + 0.5) * res
            wx = grid.origin_x + gc * lx - gs * ly
            wy = grid.origin_y + gs * lx + gc * ly
            dx, dy = wx - stand.x, wy - stand.y
            ux, uy = cy * dx + sy_ * dy, -sy_ * dx + cy * dy
            if abs(ux) <= h + eps and abs(uy) <= h + eps:
                return True
    return False


def needs_regoal(new_stand: Pose, current: Pose, arrive_tol_m: float,
                 arrive_tol_rad: float) -> bool:
    """求め直した立ち位置が今の位置から許容差を超えてずれているか（目的地の出し直しが要るか）。"""
    if math.hypot(new_stand.x - current.x, new_stand.y - current.y) > arrive_tol_m:
        return True
    return abs(angle_diff(new_stand.yaw, current.yaw)) > arrive_tol_rad


def ss_result_valid(at_request: Optional[ThStatus], at_result: Optional[ThStatus],
                    waited_throughout: bool, ss_pose_tol_m: float,
                    ss_pose_tol_rad: float) -> bool:
    """ss の結果を使ってよいか。

    依頼した時点と結果の時点の th の様子が読めて、どちらも WAIT・自己位置が有効・地図の世代が
    同じで、依頼のあいだ状態がずっと WAIT（waited_throughout。状態機械が見張る）、位置の差が
    許容差以内なら使える。外れたら結果を捨てて依頼し直す。
    """
    if at_request is None or at_result is None or not waited_throughout:
        return False
    if at_request.state != "WAIT" or at_result.state != "WAIT":
        return False
    if not (at_request.pose_valid and at_result.pose_valid):
        return False
    if at_request.map_instance_id != at_result.map_instance_id:
        return False
    if math.hypot(at_result.x - at_request.x, at_result.y - at_request.y) > ss_pose_tol_m:
        return False
    return abs(angle_diff(at_result.yaw, at_request.yaw)) <= ss_pose_tol_rad


# ---------------------------------------------------------------------------
# 読めるかの判定（共通）
# ---------------------------------------------------------------------------

def _fresh(read_ms: Optional[float], now_ms: float, stale_ms: float) -> bool:
    """読めた時刻があり、今から stale_ms を超えて古くない。"""
    return read_ms is not None and (now_ms - read_ms) <= stale_ms


def _th_readable(s: Snapshot, p: Params) -> bool:
    return s.th is not None and _fresh(s.th.read_ms, s.now_ms, p.th_status_stale_ms)


def _lift_height_readable(s: Snapshot, p: Params) -> bool:
    return (s.lift is not None and s.lift.at_bottom is not None
            and _fresh(s.lift.height_read_ms, s.now_ms, p.lift_state_stale_ms))


def _mechanism_absent(s: Snapshot) -> bool:
    """測定機構が付いていないと読めたときだけ True。分からないときは付いているとみなす。"""
    return s.lift is not None and s.lift.mechanism_attached is False


def _arm_readable(s: Snapshot, p: Params) -> bool:
    return (s.lift is not None and s.lift.arm_stowed is not None
            and _fresh(s.lift.arm_read_ms, s.now_ms, p.lift_state_stale_ms))


def _ss_readable(s: Snapshot, p: Params) -> bool:
    return (s.ss is not None and s.ss.state is not None
            and _fresh(s.ss.read_ms, s.now_ms, p.ss_status_stale_ms))


def _arm_reason(s: Snapshot, p: Params) -> Optional[str]:
    """腕の条件（格納位置にある。測定機構なしを除く）。満たさなければ理由キー。"""
    if _mechanism_absent(s):
        return None
    if not _arm_readable(s, p):
        return "state_unreadable"
    if not s.lift.arm_stowed:
        return "arm_not_stowed"
    return None


def _ss_reason(s: Snapshot, p: Params) -> Optional[str]:
    """ss の依頼が終わっている（状態が IDLE。本システム以外の依頼も含む）。"""
    if not _ss_readable(s, p):
        return "state_unreadable"
    if s.ss.state != "IDLE":
        return "ss_busy"
    return None


# ---------------------------------------------------------------------------
# gate・drive_watch（DetailedDesign-auto.md §4）
# ---------------------------------------------------------------------------

# 関門に渡す指示。th への指示は th の名前のまま（names.md §4.2・th の文書）。
# 昇降機を下げる・ss への依頼は、関門の中だけの名前（外には出さない。DetailedDesign-names.md §2）。
CMD_LIFT_LOWER = "lift.lower"
CMD_SS_REQUEST = "ss.request"
_MOVING_COMMANDS = ("meas.goal", "meas.start", "meas.reroute")
_ALWAYS_COMMANDS = ("meas.stop", "meas.abort", "meas.finish")


def gate(command: str, snapshot: Snapshot, params: Params = Params()) -> GateResult:
    """th への指示と昇降機への命令の関門。通すなら allowed、通さないなら理由キー。

    条件が複数欠けたときは、次の順で最初に当たった 1 つを返す。
    - meas.goal・meas.start・meas.reroute: th の様子（読めない → state_unreadable）→ th の版
      （version_mismatch）→ 入り直し（reentry_required）→ 昇降機の高さ（読めない → state_unreadable、
      下端でない → lift_not_bottom）→ 腕（読めない → state_unreadable、格納位置でない →
      arm_not_stowed）→ ss（読めない → state_unreadable、IDLE でない → ss_busy）。
    - meas.stop・meas.abort・meas.finish: いつでも通す。
    - 昇降機を下げる（CMD_LIFT_LOWER）: 腕の通信（arm_link_lost。通信が切れていれば腕の状態が読めない
      のも当然なので先に見る）→ 腕 → ss。
    - ss への依頼（CMD_SS_REQUEST）: th（読めない → state_unreadable、WAIT でない → th_not_waiting）
      → 昇降機の高さが読める（state_unreadable）→ 腕（読めない → state_unreadable、格納位置でない →
      arm_not_stowed）。
    - 表に無い指示は not_listed。
    測定機構が付いていないと読めたときだけ腕の条件（通信を含む）を外す。
    """
    s, p = snapshot, params
    if command in _ALWAYS_COMMANDS:
        return GateResult(True)
    if command in _MOVING_COMMANDS:
        if not _th_readable(s, p):
            return GateResult(False, "state_unreadable")
        if s.th.interface_version != p.th_interface_version:
            return GateResult(False, "version_mismatch")
        if s.th.reentry_required:
            return GateResult(False, "reentry_required")
        if not _lift_height_readable(s, p):
            return GateResult(False, "state_unreadable")
        if not s.lift.at_bottom:
            return GateResult(False, "lift_not_bottom")
        r = _arm_reason(s, p) or _ss_reason(s, p)
        return GateResult(False, r) if r else GateResult(True)
    if command == CMD_LIFT_LOWER:
        if not _mechanism_absent(s) and (s.lift is None or s.lift.arm_link_ok is not True):
            return GateResult(False, "arm_link_lost")
        r = _arm_reason(s, p) or _ss_reason(s, p)
        return GateResult(False, r) if r else GateResult(True)
    if command == CMD_SS_REQUEST:
        if not _th_readable(s, p):
            return GateResult(False, "state_unreadable")
        if s.th.state != "WAIT":
            return GateResult(False, "th_not_waiting")
        if not _lift_height_readable(s, p):
            return GateResult(False, "state_unreadable")
        r = _arm_reason(s, p)
        return GateResult(False, r) if r else GateResult(True)
    return GateResult(False, "not_listed")


_WATCHED_TH_STATES = ("NAV", "ALIGN", "BLOCKED")


def drive_watch(snapshot: Snapshot, params: Params = Params()) -> WatchResult:
    """走っている間の見張り。止めるなら stop=True と理由キー。

    th の状態が NAV・ALIGN・BLOCKED のときだけ見る（ADJUST を含むそれ以外は何もしない）。
    th の状態が古くても、最後に読めた state で判定する。th の状態の通り道が不調でも生存確認は出続け、
    th は走り続けうるので、その間の昇降機の見張りを外さない（迷ったら止める側）。
    一度も読めていない（th が None）ときだけ何もしない。
    昇降機の高さか腕の状態（測定機構なしを除く）が lift_state_stale_ms を超えて届かない →
    state_unreadable。下端から lift_drift_stop_mm 以上離れた → lift_drift。
    """
    s, p = snapshot, params
    if s.th is None or s.th.state not in _WATCHED_TH_STATES:
        return WatchResult(False)
    height_ok = (s.lift is not None and s.lift.height_above_bottom_mm is not None
                 and _fresh(s.lift.height_read_ms, s.now_ms, p.lift_state_stale_ms))
    if not height_ok:
        return WatchResult(True, "state_unreadable")
    if not _mechanism_absent(s) and not _arm_readable(s, p):
        return WatchResult(True, "state_unreadable")
    if s.lift.height_above_bottom_mm >= p.lift_drift_stop_mm:
        return WatchResult(True, "lift_drift")
    return WatchResult(False)


# ---------------------------------------------------------------------------
# can_run
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RunDecision:
    """人の「走行」の結果。kind は "run"（meas.start を出してよい）・"lower_first"（先に下げる）・
    "reject"（拒否。reason に理由キー）。"""
    kind: str
    reason: Optional[str] = None


def can_run(snapshot: Snapshot, params: Params = Params()) -> RunDecision:
    """人の「走行」で発進してよいか。

    拒否の順: th の様子（state_unreadable）→ 版（version_mismatch）→ 入り直し（reentry_required）→
    th が PAUSE でない（phase_not_accepting）→ 腕（state_unreadable・arm_not_stowed）→
    ss（state_unreadable・ss_busy）→ 昇降機の高さが読めない（state_unreadable）。
    すべて通って下端でなければ「先に下げる」（拒否ではない）。
    """
    s, p = snapshot, params
    if not _th_readable(s, p):
        return RunDecision("reject", "state_unreadable")
    if s.th.interface_version != p.th_interface_version:
        return RunDecision("reject", "version_mismatch")
    if s.th.reentry_required:
        return RunDecision("reject", "reentry_required")
    if s.th.state != "PAUSE":
        return RunDecision("reject", "phase_not_accepting")
    r = _arm_reason(s, p) or _ss_reason(s, p)
    if r:
        return RunDecision("reject", r)
    if not _lift_height_readable(s, p):
        return RunDecision("reject", "state_unreadable")
    if not s.lift.at_bottom:
        return RunDecision("lower_first")
    return RunDecision("run")


# ---------------------------------------------------------------------------
# classify_skip・classify_halt
# ---------------------------------------------------------------------------

class SkipEvent(Enum):
    """盤を飛ばす出来事（DetailedDesign-auto.md §2.1。Spec-auto.md §6 の上段）。"""
    PATH_NOT_FOUND = 1           # 経路の再計算をしても解けない
    STAND_AREA_OCCUPIED = 2      # 正面側の床が地図で塞がっていて立てない
    STAND_MISMATCH_TWICE = 3     # 認識し直した立ち位置が、目的地を 1 回出し直してもまだ許容差を超える
    TOP_NOT_FOUND = 4            # 着いて認識し直しても天面が見つからない（確からしさ不足を含む）
    ARM_SPACE_BLOCKED = 5        # 腕が振れる範囲が左右とも塞がっている（測定機構の口が返す）
    MECHANISM_ABSENT = 6         # 測定機構が付いていない
    SS_TIMEOUT = 7               # ss の結果が時間内に来ない
    SS_RESULT_UNUSABLE = 8       # 依頼し直しても使える結果が得られない
    CAMERA_TOO_LOW = 9           # ss が必要な高さまで上がれない


class HaltEvent(Enum):
    """全体を止める出来事（Spec-auto.md §6 の下段・§7）。"""
    ARM_STUCK = 1                # 腕が出たまま動けない
    LIFT_CANNOT_LOWER = 2        # 昇降機を下げられない
    LIFT_OR_ARM_UNREADABLE = 3   # 昇降機の高さか腕の状態が読めない
    SS_CLEANUP_NOT_DONE = 4      # ss の後片付けの完了が来ない
    SS_LINK_LOST = 5             # ss と通信できない
    ARM_LINK_LOST = 6            # 腕が出ている間に腕の制御器と通信できない


_SKIP_REASONS = {
    SkipEvent.PATH_NOT_FOUND: "no_path",
    SkipEvent.STAND_AREA_OCCUPIED: "stand_blocked",
    SkipEvent.STAND_MISMATCH_TWICE: "stand_mismatch",
    SkipEvent.TOP_NOT_FOUND: "top_not_found",
    SkipEvent.ARM_SPACE_BLOCKED: "arm_space_blocked",
    SkipEvent.MECHANISM_ABSENT: "no_mechanism",
    SkipEvent.SS_TIMEOUT: "ss_failed",
    SkipEvent.SS_RESULT_UNUSABLE: "ss_failed",
    SkipEvent.CAMERA_TOO_LOW: "camera_too_low",
}

_HALT_REASONS = {
    HaltEvent.ARM_STUCK: "arm_stuck",
    HaltEvent.LIFT_CANNOT_LOWER: "lift_not_lowered",
    HaltEvent.LIFT_OR_ARM_UNREADABLE: "state_unreadable",
    HaltEvent.SS_CLEANUP_NOT_DONE: "ss_cleanup_timeout",
    HaltEvent.SS_LINK_LOST: "ss_link_lost",
    HaltEvent.ARM_LINK_LOST: "arm_link_lost",
}


def classify_skip(event: SkipEvent) -> str:
    """飛ばす出来事 → 飛ばす理由キー（names.md §4.3）。"""
    return _SKIP_REASONS[event]


def classify_halt(event: HaltEvent) -> str:
    """止める出来事 → 止める理由キー（names.md §4.3）。"""
    return _HALT_REASONS[event]

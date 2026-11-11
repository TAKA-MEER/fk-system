"""自律測定の移動の純関数（DetailedDesign-auto.md §3・§4）。

標準ライブラリだけ。ROS 2・通信・ss や th の型には触れない。
入力はこのファイルの dataclass で受ける。分からないものは None（「読めない」として扱う）。
外に出る文字列（段・理由キー・指示）は DetailedDesign-names.md §4 のものだけ。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
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

"""§3 の表: to_map・is_confident・front_edge・group_rows・order_panels・stand_pose の試験。"""
import math

import pytest

from fk_auto.core import (MapRect, Panel, Pose, SsRect, angle_diff, front_edge, group_rows,
                          is_confident, make_panel, order_panels, stand_pose, to_map,
                          unseen_ranges, wrap_angle)

PI = math.pi


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# --- 角度の折り返し ---------------------------------------------------------

def test_wrap_angle_range_and_crossing():
    assert close(wrap_angle(PI + 0.1), -PI + 0.1)
    assert close(wrap_angle(-PI - 0.1), PI - 0.1)
    assert close(wrap_angle(2 * PI + 0.3), 0.3)
    assert close(wrap_angle(0.0), 0.0)
    assert close(angle_diff(-PI + 0.05, PI - 0.05), 0.1)


# --- to_map -----------------------------------------------------------------

def test_to_map_translates_and_rotates():
    r = SsRect(x_m=2.0, y_m=1.0, yaw_rad=PI, width_m=1.0, depth_m=0.5)
    out = to_map([r], Pose(10.0, 20.0, PI / 2)).panels[0]
    # 機体の前 2 m・左 1 m、機体が +y を向いている → (10 - 1, 20 + 2)
    assert close(out.x, 9.0) and close(out.y, 22.0)
    # 向きは機体の向きを足して折り返す: π + π/2 → −π/2
    assert close(out.yaw_rad, -PI / 2)
    assert out.width_m == 1.0 and out.depth_m == 0.5


def test_to_map_yaw_wraps_across_pi():
    r = SsRect(0, 0, PI - 0.1, 1, 1)
    out = to_map([r], Pose(0, 0, 0.3)).panels[0]
    assert close(out.yaw_rad, -PI + 0.2)


def test_to_map_front_known_rotated_and_wrapped():
    r = SsRect(0, 0, 0.0, 1, 1, front_known=True, front_yaw_rad=PI - 0.1)
    out = to_map([r], Pose(0, 0, 0.3)).panels[0]
    assert out.front_known and close(out.front_yaw_rad, -PI + 0.2)
    r2 = SsRect(0, 0, 0.0, 1, 1)
    assert to_map([r2], Pose(0, 0, 0.3)).panels[0].front_yaw_rad == 0.0


def test_to_map_unseen_basic():
    # 機体の前 ±90° だけ見た
    res = to_map([], Pose(0, 0, 0.0), [-PI / 2], [PI])
    assert len(res.unseen) == 1
    f, t = res.unseen[0]
    assert close(f, PI / 2) and close(t, -PI / 2)


def test_to_map_unseen_rotated_by_pose():
    res = to_map([], Pose(0, 0, PI / 2), [-PI / 2], [PI])
    f, t = res.unseen[0]
    assert close(f, PI) and close(t, 0.0)


def test_unseen_full_circle_seen_is_empty():
    assert unseen_ranges([0.5], [2 * PI], 0.0) == ()
    assert unseen_ranges([0.0, 0.0], [PI, 2 * PI], 1.0) == ()


def test_unseen_nothing_seen_is_whole_circle():
    out = unseen_ranges([], [], 0.0)
    assert len(out) == 1 and close(out[0][0], -PI) and close(out[0][1], PI)


def test_unseen_overlapping_ranges_merge():
    # 0〜1.5 と 1.0〜2.5 が重なる → 見たのは 0〜2.5。残りは 2.5 → 2π
    out = unseen_ranges([0.0, 1.0], [1.5, 1.5], 0.0)
    assert len(out) == 1
    assert close(out[0][0], 2.5) and close(out[0][1], wrap_angle(2 * PI))


def test_unseen_range_crossing_pi():
    # π − 0.5 から幅 1.0 → π をまたぐ。残りは (π + 0.5) → (π − 0.5) の 2π − 1 幅
    out = unseen_ranges([PI - 0.5], [1.0], 0.0)
    assert len(out) == 1
    assert close(out[0][0], -PI + 0.5) and close(out[0][1], PI - 0.5)


def test_unseen_two_gaps():
    out = unseen_ranges([0.0, 3.0], [1.0, 1.0], 0.0)
    assert len(out) == 2
    assert close(out[0][0], 1.0) and close(out[0][1], 3.0)
    assert close(out[1][0], wrap_angle(4.0)) and close(out[1][1], wrap_angle(2 * PI))


def test_unseen_adjacent_ranges_no_gap():
    assert unseen_ranges([0.0, PI], [PI, PI], 0.0) == ()


# --- is_confident -----------------------------------------------------------

def test_is_confident_thresholds():
    assert is_confident(0.15, 5, 0.15, 5)
    assert not is_confident(0.16, 1, 0.15, 5)
    assert not is_confident(0.01, 5.1, 0.15, 5)


# --- front_edge -------------------------------------------------------------

def rect(yaw=0.0, w=1.0, d=0.5, **kw):
    return MapRect(0.0, 0.0, yaw, w, d, **kw)


def test_front_edge_faces_observer_each_side():
    r = rect(yaw=0.0)
    e = front_edge(r, (5.0, 0.0))
    assert e.index == 0 and close(e.front_yaw, 0.0)
    assert close(e.half_distance_m, 0.25) and close(e.length_m, 1.0)
    e = front_edge(r, (0.0, 5.0))
    assert e.index == 1 and close(e.front_yaw, PI / 2)
    assert close(e.half_distance_m, 0.5) and close(e.length_m, 0.5)
    e = front_edge(r, (-5.0, 0.0))
    assert e.index == 2 and close(abs(e.front_yaw), PI)
    assert close(e.half_distance_m, 0.25) and close(e.length_m, 1.0)
    e = front_edge(r, (0.0, -5.0))
    assert e.index == 3 and close(e.front_yaw, -PI / 2)


def test_front_edge_rotated_rect():
    r = rect(yaw=PI - 0.1)
    e = front_edge(r, (-5.0, 0.0))
    assert e.index == 0 and close(e.front_yaw, PI - 0.1)
    e = front_edge(r, (5.0, 0.0))
    assert e.index == 2 and close(e.front_yaw, wrap_angle(PI - 0.1 + PI))


def test_front_edge_ss_front_ignored_but_doubt_flagged():
    r = rect(yaw=0.0, front_known=True, front_yaw_rad=PI)
    e = front_edge(r, (5.0, 0.0))
    assert e.index == 0 and close(e.front_yaw, 0.0)  # 正面は変えない
    assert e.front_doubt and close(e.ss_front_yaw, PI)


def test_front_edge_ss_front_agrees_no_doubt():
    r = rect(yaw=0.0, front_known=True, front_yaw_rad=0.05)
    e = front_edge(r, (5.0, 0.0))
    assert not e.front_doubt and e.ss_front_yaw is not None


def test_front_edge_ss_front_nearest_edge_across_pi():
    # ss の正面 −π + 0.05 は辺 2（向き π）に最も近い
    r = rect(yaw=0.0, front_known=True, front_yaw_rad=-PI + 0.05)
    assert not front_edge(r, (-5.0, 0.0)).front_doubt
    assert front_edge(r, (5.0, 0.0)).front_doubt


def test_front_edge_without_ss_front():
    e = front_edge(rect(), (5.0, 0.0))
    assert not e.front_doubt and e.ss_front_yaw is None


# --- group_rows -------------------------------------------------------------

def P(pid, x, y, yaw=0.0, length=1.0):
    return Panel(pid, x, y, yaw, 0.3, length)


def test_group_rows_same_row_by_gap():
    # 正面 +x。横は y 方向。幅 1.0、中心間 1.2 → 隙間 0.2 ≤ 0.3
    rows = group_rows([P("a", 0, 0), P("b", 0, 1.2)], 0.17, 0.3)
    assert len(rows) == 1 and set(rows[0]) == {"a", "b"}


def test_group_rows_gap_too_big_splits():
    rows = group_rows([P("a", 0, 0), P("b", 0, 1.4)], 0.17, 0.3)  # 隙間 0.4
    assert len(rows) == 2


def test_group_rows_gap_boundary_inclusive():
    rows = group_rows([P("a", 0, 0), P("b", 0, 1.3)], 0.17, 0.3)  # 隙間 0.3 ちょうど
    assert len(rows) == 1


def test_group_rows_yaw_difference_splits():
    rows = group_rows([P("a", 0, 0, 0.0), P("b", 0, 1.1, 0.3)], 0.17, 0.3)
    assert len(rows) == 2


def test_group_rows_yaw_tol_across_pi():
    rows = group_rows([P("a", 0, 0, PI - 0.05), P("b", 0, 1.1, -PI + 0.05)], 0.17, 0.3)
    assert len(rows) == 1


def test_group_rows_back_to_back_not_same_row():
    rows = group_rows([P("a", 0, 0, 0.0), P("b", 0, 1.1, PI)], 0.17, 0.3)
    assert len(rows) == 2


def test_group_rows_transitive_chain():
    rows = group_rows([P("a", 0, 0), P("c", 0, 2.4), P("b", 0, 1.2)], 0.17, 0.3)
    assert len(rows) == 1 and len(rows[0]) == 3


def test_group_rows_right_to_left_inside_row():
    # 正面 +x の盤に正面から向かうと、右は −y 側ではなく +y 側（前 +x を見返すので）。
    # 正面に立って盤を見る人の右手 = 正面の向き + 90° = +y。右から = y の大きい方から
    rows = group_rows([P("a", 0, 0), P("b", 0, 1.2), P("c", 0, -1.2)], 0.17, 0.3)
    assert rows == [["b", "a", "c"]]


def test_group_rows_empty():
    assert group_rows([], 0.17, 0.3) == []


# --- order_panels -----------------------------------------------------------

def test_order_panels_nearest_row_first_and_right_to_left():
    panels = [P("a", 0, 0), P("b", 0, 1.2),            # 列 1 (x=0)
              P("c", 10, 0), P("d", 10, 1.2)]          # 列 2 (x=10)
    rows = group_rows(panels, 0.17, 0.3)
    assert order_panels(["a", "b", "c", "d"], rows, panels, (9.0, 0.0)) == ["d", "c", "b", "a"]
    assert order_panels(["a", "b", "c", "d"], rows, panels, (-1.0, 0.0)) == ["b", "a", "d", "c"]


def test_order_panels_only_selected():
    panels = [P("a", 0, 0), P("b", 0, 1.2), P("c", 10, 0)]
    rows = group_rows(panels, 0.17, 0.3)
    assert order_panels(["a", "c"], rows, panels, (0.0, 0.0)) == ["a", "c"]
    assert order_panels([], rows, panels, (0.0, 0.0)) == []


def test_order_panels_row_distance_uses_nearest_selected_member():
    panels = [P("a", 0, 0), P("b", 0, 6.0), P("c", 5, 0)]
    rows = [["a", "b"], ["c"]]
    # 現在位置 (5, 0)。列 1 の選ばれた盤は b のみ（距離 ≈ 7.8）、列 2 は c（0）
    assert order_panels(["b", "c"], rows, panels, (5.0, 0.0)) == ["c", "b"]


def test_order_panels_unlisted_selected_panel_is_own_row():
    panels = [P("a", 0, 0), P("z", 3, 0)]
    assert order_panels(["a", "z"], [], panels, (3.0, 0.0)) == ["z", "a"]


# --- stand_pose -------------------------------------------------------------

def test_stand_pose_front_x():
    p = Panel("a", 1.0, 2.0, 0.0, front_half_m=0.25, front_length_m=1.0)
    s = stand_pose(p, 0.389, 0.1)
    assert close(s.x, 1.0 + 0.25 + 0.489) and close(s.y, 2.0)
    assert close(abs(s.yaw), PI)  # 盤に向く


def test_stand_pose_rotated_and_yaw_wrap():
    p = Panel("a", 0.0, 0.0, PI / 2 + 0.2, 0.5, 1.0)
    s = stand_pose(p, 0.4, 0.0)
    d = 0.9
    assert close(s.x, d * math.cos(PI / 2 + 0.2)) and close(s.y, d * math.sin(PI / 2 + 0.2))
    assert close(s.yaw, wrap_angle(PI / 2 + 0.2 + PI))
    assert -PI <= s.yaw <= PI


def test_make_panel_and_stand_pose_from_front_edge():
    r = MapRect(3.0, 4.0, 0.0, 1.0, 0.5)
    e = front_edge(r, (10.0, 4.0))
    s = stand_pose(make_panel("a", r, e), 0.389, 0.0)
    assert close(s.x, 3.0 + 0.25 + 0.389) and close(s.y, 4.0)


def test_group_rows_front_back_offset_splits():
    """前後に離れた同じ向きの盤は、横が重なっても別の列。"""
    rows = group_rows([P("a", 0, 0), P("b", 2.0, 0)], 0.17, 0.3)
    assert len(rows) == 2


def test_unseen_contained_range_does_not_shrink_merged():
    """大きい範囲の中に小さい範囲が入っていても、見た範囲は縮まない。"""
    out = unseen_ranges([0.0, 0.5], [3.0, 1.0], 0.0)
    assert len(out) == 1
    assert close(out[0][0], 3.0) and close(out[0][1], wrap_angle(2 * PI))

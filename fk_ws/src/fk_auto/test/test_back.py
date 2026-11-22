"""WP-AUTO-10 純関数（DetailedDesign-auto.md §3 の needs_back・order_back・
back_stand_pose・match_back・merge_back）。境目の値を含む。"""
import math

import pytest

from fk_auto import core
from fk_auto.core import MapRect, Panel

P = core.Params()
NEEDS = dict(arm_reach_m=P.arm_reach_m, front_approach_min_m=P.front_approach_min_m,
             corner_inset_m=P.corner_inset_m)


def panel(depth, measured=True, pid="P001", x=0.0, y=0.0, yaw=math.pi, length=1.0):
    return Panel(pid, x, y, yaw, depth / 2.0, length, measured)


# --- needs_back ---

@pytest.mark.parametrize("depth,expected", [(1.0, False), (1.4, True), (2.0, True)])
def test_needs_back_depth_boundary(depth, expected):
    assert core.needs_back(panel(depth), **NEEDS) is expected


def test_needs_back_unmeasured_always_true():
    assert core.needs_back(panel(0.6, measured=False), **NEEDS) is True
    assert core.needs_back(panel(0.0, measured=False), **NEEDS) is True


def test_needs_back_exact_threshold_is_false():
    # depth - inset == reach - approach なら超えていないので偽
    depth = NEEDS["arm_reach_m"] - NEEDS["front_approach_min_m"] + NEEDS["corner_inset_m"]
    assert core.needs_back(panel(depth), **NEEDS) is False
    assert core.needs_back(panel(depth + 0.001), **NEEDS) is True


def test_params_defaults_for_back():
    p = core.Params()
    assert (p.arm_reach_m, p.front_approach_min_m, p.corner_inset_m, p.panel_depth_max_m) == \
        (1.76, 0.58, 0.10, 2.04)


# --- back_stand_pose ---

def test_back_stand_pose_is_behind_back_face_facing_panel():
    # front_yaw = pi（正面は -x へ外向き）。盤は原点，奥行き 2.0。背面の面は x=+1.0，
    # 立ち位置はそこから外（+x）へ 0.389、向きは盤へ（0 rad）
    pose = core.back_stand_pose(panel(2.0), P.panel_depth_max_m,
                                 P.stand_distance_min_m, P.stand_margin_m)
    assert pose.x == pytest.approx(1.0 + 0.389)
    assert pose.y == pytest.approx(0.0)
    # 向きは盤へ: front_yaw(pi) のまま（背面から中心へ向かう向き）
    assert pose.yaw == pytest.approx(math.pi)


def test_back_stand_pose_unmeasured_uses_panel_depth_max():
    pose = core.back_stand_pose(panel(0.86, measured=False), P.panel_depth_max_m,
                                 P.stand_distance_min_m, P.stand_margin_m)
    # 正面の面 x=-0.43 から内向き（+x）へ 2.04 → 背面 x=1.61、そこから外へ 0.389
    assert pose.x == pytest.approx(-0.43 + 2.04 + 0.389)


def test_back_stand_pose_measured_does_not_use_max():
    pose = core.back_stand_pose(panel(1.0), P.panel_depth_max_m,
                                 P.stand_distance_min_m, P.stand_margin_m)
    assert pose.x == pytest.approx(0.5 + 0.389)


# --- order_back ---

def _order(panels, cur, rows):
    return core.order_back(panels, cur, rows, P.panel_depth_max_m,
                           P.stand_distance_min_m, P.stand_margin_m)


def test_order_back_nearest_row_first_and_nearest_end():
    # 1 列: R1 = [P001(y=0), P002(y=3)]、R2 = [P003(y=10)]。正面 yaw=pi なので
    # 背面の立ち位置は盤の +x 側。現在位置は y=3 側（P002 の近く）
    ps = [panel(0.6, pid="P001", x=0.0, y=0.0), panel(0.6, pid="P002", x=0.0, y=3.0),
          panel(0.6, pid="P003", x=0.0, y=10.0)]
    rows = [["P001", "P002"], ["P003"]]
    assert _order(ps, (1.0, 2.9), rows) == ["P002", "P001", "P003"]


def test_order_back_second_row_from_last_stand():
    # 現在位置は R1 の近く。R1 を終えた位置（y=0 側）から近い R2 を次に
    ps = [panel(0.6, pid="P001", x=0.0, y=0.0), panel(0.6, pid="P002", x=0.0, y=3.0),
          panel(0.6, pid="P003", x=0.0, y=-2.0)]
    rows = [["P001", "P002"], ["P003"]]
    assert _order(ps, (1.0, 0.1), rows) == ["P001", "P002", "P003"]


def test_order_back_empty_and_singleton_not_in_rows():
    assert _order([], (0.0, 0.0), []) == []
    ps = [panel(0.6, pid="P009", x=5.0)]
    assert _order(ps, (0.0, 0.0), []) == ["P009"]


# --- match_back ---

def _rect(x, y, yaw, w=1.0, d=2.0):
    return MapRect(x, y, yaw, w, d)


def test_match_back_picks_smallest_lateral_and_rejects_others():
    p = panel(2.0)  # 中心 (0,0)，front_yaw=pi
    good = _rect(0.0, 0.05, 0.0)        # 背面側の矩形（yaw は反対向き＝平行）
    far = _rect(0.0, 0.9, 0.0)          # 幅のずれが大きい
    not_parallel = _rect(0.0, 0.0, math.pi / 2)
    in_front = _rect(-2.0, 0.0, 0.0)    # 正面の面より前（中心が正面側）
    too_deep = _rect(3.0, 0.0, 0.0)     # 正面の面から panel_depth_max を超えて奥
    assert core.match_back(p, [far, not_parallel, in_front, too_deep, good],
                           P.recheck_match_max_m, P.panel_depth_max_m, P.row_yaw_tol_rad) is good
    assert core.match_back(p, [far], P.recheck_match_max_m, P.panel_depth_max_m,
                           P.row_yaw_tol_rad) is None


def test_match_back_accepts_centre_shifted_by_unmeasured_depth():
    # 奥行きが測れていない盤（front_half=0.43）の背面側の矩形は、基準の辺に
    # 基準を置くため中心が盤の中心からずれる（−x 側へ）。中心距離では拾えないが
    # 幅の向きのずれ（y 方向）だけを見れば拾える。
    # 配置図の盤は正面側から基準の辺に基準を置いて中心がずれている（x=-0.5）。
    # 背面側の矩形は背面の辺に基準を置くので中心は +0.5 側になる。
    p = Panel("P001", -0.5, 0.0, math.pi, 0.43, 1.0, False)
    shifted = _rect(0.5, 0.02, 0.0, d=1.86)
    assert core.match_back(p, [shifted], P.recheck_match_max_m, P.panel_depth_max_m,
                           P.row_yaw_tol_rad) is shifted


def test_match_back_parallel_allows_opposite_direction():
    p = panel(2.0)
    r = _rect(0.0, 0.0, math.pi)   # front_yaw と同じ向きでも平行
    assert core.match_back(p, [r], P.recheck_match_max_m, P.panel_depth_max_m,
                           P.row_yaw_tol_rad) is r


# --- merge_back ---

def test_merge_back_uses_front_face_and_facing_edge():
    p = panel(0.86, measured=False)   # 正面の面 x=-0.43
    # 背面側の矩形: 中心 (0.07, 0)，yaw=0（+x 外向き），depth=2.0 → 背面の面 x=1.07
    merged = core.merge_back(p, _rect(0.07, 0.0, 0.0, d=2.0),
                             P.panel_depth_max_m, P.recheck_match_max_m)
    assert merged is not None
    assert merged.depth_measured is True
    assert 2 * merged.front_half_m == pytest.approx(1.5)      # -0.43 .. 1.07
    assert merged.x == pytest.approx((-0.43 + 1.07) / 2)
    assert merged.y == pytest.approx(0.0)
    assert merged.front_yaw == pytest.approx(math.pi)
    assert merged.front_length_m == pytest.approx(1.0)


def test_merge_back_rejects_too_deep_and_nonpositive():
    p = panel(0.86, measured=False)
    assert core.merge_back(p, _rect(0.0, 0.0, 0.0, d=5.0),
                           P.panel_depth_max_m, P.recheck_match_max_m) is None
    # 選択した矩形の基準の辺が正面の面より前にある（奥行きが負になる）
    assert core.merge_back(p, _rect(-1.0, 0.0, 0.0, d=0.2),
                           P.panel_depth_max_m, P.recheck_match_max_m) is None

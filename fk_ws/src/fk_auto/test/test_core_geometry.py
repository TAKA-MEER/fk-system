"""§3 の表: stand_blocked・needs_regoal・ss_result_valid の試験。"""
import math

from fk_auto.core import Grid, Pose, ThStatus, needs_regoal, ss_result_valid, stand_blocked

PI = math.pi


def make_grid(w=40, h=40, res=0.05, ox=0.0, oy=0.0, fill=0, yaw=0.0):
    return [fill] * (w * h), dict(resolution=res, origin_x=ox, origin_y=oy, width=w, height=h,
                                  origin_yaw=yaw)


def grid_with(cells, **kw):
    data, meta = make_grid(**kw)
    g = Grid(data=data, **meta)
    d = list(data)
    for (c, r), v in cells.items():
        d[r * meta["width"] + c] = v
    return Grid(data=d, **meta)


# --- stand_blocked ----------------------------------------------------------

def test_stand_blocked_free_map():
    assert not stand_blocked(grid_with({}), Pose(1.0, 1.0, 0.0), 0.275, 1)


def test_stand_blocked_occupied_cell_inside():
    # 立ち位置 (1,1)。セル (20,20) の中心は (1.025,1.025)
    assert stand_blocked(grid_with({(20, 20): 100}), Pose(1.0, 1.0, 0.0), 0.275, 1)


def test_stand_blocked_occupied_cell_outside():
    # 中心 (1.425, 1.0) は一辺 0.55 の正方形（±0.275）の外
    assert not stand_blocked(grid_with({(28, 20): 100}), Pose(1.0, 1.0, 0.0), 0.275, 1)


def test_stand_blocked_unknown_cell_is_not_blocked():
    """−1（分からない）は塞がっていないとみなす（§3 の注記）。"""
    g = grid_with({}, fill=-1)
    assert not stand_blocked(g, Pose(1.0, 1.0, 0.0), 0.275, 1)
    # しきい値が 0 でも −1 は塞がっていない
    assert not stand_blocked(grid_with({}, fill=-1), Pose(1.0, 1.0, 0.0), 0.275, 0)
    # 分からないマスの中に占有が 1 つあれば塞がっている
    assert stand_blocked(grid_with({(20, 20): 100}, fill=-1), Pose(1.0, 1.0, 0.0), 0.275, 1)


def test_stand_blocked_threshold_map_occupied_min():
    g = grid_with({(20, 20): 30})
    assert stand_blocked(g, Pose(1.0, 1.0, 0.0), 0.275, 30)
    assert not stand_blocked(g, Pose(1.0, 1.0, 0.0), 0.275, 31)


def test_stand_blocked_square_is_rotated_with_pose():
    # 正方形の角（0.275,0.275）方向のセルは、45° 回すと外、0° なら内
    g = grid_with({(33, 33): 100}, w=60, h=60)  # 中心 (1.675,1.675)
    stand0 = Pose(1.4, 1.4, 0.0)    # 角 (1.675,1.675) 付近 → 差 0.275 → 内（境界）
    assert stand_blocked(g, stand0, 0.275, 1)
    stand45 = Pose(1.4, 1.4, PI / 4)  # 回すと角は軸上に伸び、(0.275,0.275) は外
    assert not stand_blocked(g, stand45, 0.275, 1)


def test_stand_blocked_rotated_yaw_wraps():
    g = grid_with({(20, 20): 100})
    assert stand_blocked(g, Pose(1.0, 1.0, PI + 0.1), 0.275, 1)
    assert stand_blocked(g, Pose(1.0, 1.0, -PI - 0.1), 0.275, 1)


def test_stand_blocked_outside_map_is_not_blocked():
    g = grid_with({(0, 0): 100})
    assert not stand_blocked(g, Pose(-5.0, -5.0, 0.0), 0.275, 1)
    assert not stand_blocked(g, Pose(100.0, 100.0, 0.0), 0.275, 1)


def test_stand_blocked_at_map_edge_partially_outside():
    # 立ち位置が地図の端にかかっていて、地図の中に占有がある → 塞がっている
    g = grid_with({(0, 0): 100})
    assert stand_blocked(g, Pose(0.0, 0.0, 0.0), 0.275, 1)
    # 地図の外にはみ出していて、中の占有は無い → 塞がっていない
    assert not stand_blocked(grid_with({}), Pose(0.0, 0.0, 0.0), 0.275, 1)


def test_stand_blocked_far_edge_cells():
    g = grid_with({(39, 39): 100})  # 中心 (1.975,1.975)
    assert stand_blocked(g, Pose(2.0, 2.0, 0.0), 0.275, 1)


def test_stand_blocked_origin_offset():
    g = grid_with({(0, 0): 100}, ox=-5.0, oy=-5.0)  # 中心 (-4.975,-4.975)
    assert stand_blocked(g, Pose(-5.0, -5.0, 0.0), 0.275, 1)
    assert not stand_blocked(g, Pose(0.0, 0.0, 0.0), 0.275, 1)


# --- needs_regoal -----------------------------------------------------------

def test_needs_regoal_within_tolerance():
    assert not needs_regoal(Pose(1.0, 1.0, 0.0), Pose(1.1, 1.0, 0.1), 0.12, 0.12)


def test_needs_regoal_distance_exceeds():
    assert needs_regoal(Pose(1.0, 1.0, 0.0), Pose(1.13, 1.0, 0.0), 0.12, 0.12)
    assert needs_regoal(Pose(1.0, 1.0, 0.0), Pose(1.09, 1.09, 0.0), 0.12, 0.12)


def test_needs_regoal_yaw_exceeds_and_wraps():
    assert needs_regoal(Pose(0, 0, 0.0), Pose(0, 0, 0.13), 0.12, 0.12)
    # +π と −π をまたぐ: 差は 0.1 しかない
    assert not needs_regoal(Pose(0, 0, PI - 0.05), Pose(0, 0, -PI + 0.05), 0.12, 0.12)
    assert needs_regoal(Pose(0, 0, PI - 0.1), Pose(0, 0, -PI + 0.1), 0.12, 0.12)


# --- ss_result_valid --------------------------------------------------------

def th(**kw):
    base = dict(state="WAIT", pose_valid=True, map_instance_id="m1", x=1.0, y=2.0, yaw=0.0)
    base.update(kw)
    return ThStatus(**base)


def valid(req=None, res=None, waited=True):
    return ss_result_valid(req if req is not None else th(), res if res is not None else th(),
                           waited, 0.02, 0.02)


def test_ss_result_valid_all_ok():
    assert valid()


def test_ss_result_invalid_unreadable():
    assert not ss_result_valid(None, th(), True, 0.02, 0.02)
    assert not ss_result_valid(th(), None, True, 0.02, 0.02)


def test_ss_result_invalid_not_waited_throughout():
    assert not valid(waited=False)


def test_ss_result_invalid_state_not_wait_either_end():
    assert not valid(req=th(state="NAV"))
    assert not valid(res=th(state="PAUSE"))


def test_ss_result_invalid_map_instance_changed():
    assert not valid(res=th(map_instance_id="m2"))


def test_ss_result_invalid_pose_not_valid_either_end():
    assert not valid(req=th(pose_valid=False))
    assert not valid(res=th(pose_valid=False))


def test_ss_result_position_tolerance_boundary():
    assert valid(res=th(x=1.015))
    assert not valid(res=th(x=1.03))
    assert not valid(res=th(x=1.02, y=2.02))  # 差 0.028


def test_ss_result_yaw_tolerance_wraps():
    assert valid(req=th(yaw=PI - 0.005), res=th(yaw=-PI + 0.005))
    assert not valid(req=th(yaw=PI - 0.02), res=th(yaw=-PI + 0.02))

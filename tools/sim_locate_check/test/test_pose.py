"""座標の直し（向きのある機体で符号を縛る）。

表記は `tools/sim_locate_check/pose.py` の通り：
  world = R(yaw) * base_link_xy + (x, y)

いくつかは日常の変異（回転の符号の取り違え・平行移動の符号の取り違え）が
入ったときに赤くなる。
"""

import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)

from pose import (  # noqa: E402
    angle_in_base_link,
    base_link_to_world,
    normalize_angle,
    panel_compare,
    world_to_base_link,
)


def _approx_point(p, expected, tol=1e-9):
    return math.hypot(p[0] - expected[0], p[1] - expected[1]) <= tol


def test_round_trip_any_pose():
    robot = (1.23, -0.45, 0.77)
    for point in ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (-2.5, 3.25)):
        back = world_to_base_link(base_link_to_world(point, robot), robot)
        assert _approx_point(back, point)
        fwd = base_link_to_world(world_to_base_link(point, robot), robot)
        assert _approx_point(fwd, point)


def test_forward_is_x_axis():
    """機体の前方（base_link +X）が world で yaw の向きに進む（符号の誤りの検出）。

    真の姿勢が strava せず直すに至るための要、符号の誤りを起こす。
    """
    robot = (0.0, 0.0, math.pi / 2)  # 前が +Y 方向を指す
    assert _approx_point(base_link_to_world((1.0, 0.0), robot), (0.0, 1.0))
    assert _approx_point(base_link_to_world((0.0, 1.0), robot), (-1.0, 0.0))
    back = world_to_base_link((0.0, 1.0), robot)
    assert _approx_point(back, (1.0, 0.0))


@pytest.mark.parametrize(
    "yaw, expected",
    [
        (0.0, 0.0),
        (math.pi / 2, math.pi / 2),
        (math.pi, math.pi),
        (-math.pi / 2, -math.pi / 2),
        (2 * math.pi - 0.3, -0.3),
        (-2 * math.pi + 0.4, 0.4),
    ],
)
def test_normalize_angle(yaw, expected):
    assert math.isclose(normalize_angle(yaw), expected, abs_tol=1e-12)


def test_angle_in_base_link():
    robot = (0.0, 0.0, math.pi / 2)
    # world の +X（0 rad）は base_link で -π/2 として見える
    assert math.isclose(angle_in_base_link(0.0, robot), -math.pi / 2, abs_tol=1e-12)
    assert math.isclose(angle_in_base_link(math.pi / 2, robot), 0.0, abs_tol=1e-12)
    # 左右の符号が逆候補になる：+π/4 にある対象は robot の前（+π/2）に比べて
    # 向きの符号が負になる
    assert math.isclose(angle_in_base_link(math.pi / 4, robot), -math.pi / 4, abs_tol=1e-12)


def test_panel_compare_zero_error():
    """ss の結果が真値と完全に一致すれば、全部 0 になる（向きの定義の相殺も含む）。

    機体は world の (10, 20, 0.3) の配置にあり、盤は site に置けるよう
    base_link 側の座標に直す。正面の向きも合わせる。
    """
    robot = (10.0, 20.0, 0.3)
    truth_front = 1.1
    truth_center_world = (11.5, 21.2)
    ss_center = world_to_base_link(truth_center_world, robot)
    ss_front_base = angle_in_base_link(truth_front, robot)
    ss_panel = {
        "x_m": ss_center[0], "y_m": ss_center[1],
        "yaw_rad": ss_front_base + math.pi / 2,
        "width_m": 0.8, "depth_m": 0.8,
        "front_known": True, "front_yaw_rad": ss_front_base,
    }
    # 真の四隅は scene.json の FL/FR/BR/BL の慣習に合わせる（base_link の角度は yaw_rad の向きから作る）
    # ss 変換の正しい値に対して、truth の形は一致するものとして与える。
    # --- 真の四隅を ss の回転から引いて作る（独立の実装では作らない）
    # ss の矩形と同じ形で、centre は truth の候補で作る。
    corners_world = [base_link_to_world(c, robot) for c in
                     make_square(ss_center, 0.8, 0.8, ss_panel["yaw_rad"])]
    truth_panel = {
        "center": list(truth_center_world), "front_yaw": truth_front,
        "top_corners": {k: [c[0], c[1], 2.35] for k, c in zip(
            ("FL", "FR", "BR", "BL"), corners_world)},
    }
    result = panel_compare(ss_panel, truth_panel, robot)
    assert result["center_err_m"] == pytest.approx(0.0, abs=1e-9)
    assert result["yaw_err_rad"] == pytest.approx(0.0, abs=1e-9)
    # 同相 4 通りの中で正しい組み合わせが選ばれるので、誤差は 0 に近くなる
    assert max(result["corner_errs_m"]) == pytest.approx(0.0, abs=1e-9)


def make_square(center, w, d, normal):
    cx, cy = center
    ex, ey = -math.sin(normal), math.cos(normal)
    fx, fy = math.cos(normal), math.sin(normal)
    out = []
    for sw, sd in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2)):
        out.append((cx + ex * sw + fx * sd, cy + ey * sw + fy * sd))
    return out

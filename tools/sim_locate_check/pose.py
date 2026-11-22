"""機体の置き場（world）↔ ss の結果（`base_link`）の座標の直しと比べ。

ROS 2・Gazebo・ss には触れない（純関数のみ）。`reason = base_link から見た配置図`。
ss が返す盤の矩形は、機体の `base_link` を原点にした座標（ロボットの前が +X）。
真値はシーンの `scene.json` のワールド座標。

coordinate 変換:
    world = R(yaw) * base_link_xy + (x, y)
回転行列は標準の左手回りの（ccW）+yaw：

    R(θ) = [[cosθ, -sinθ], [sinθ, cosθ]]

`world_to_base_link` はその逆（平行移動の後、反対向きに回す）。
"""

from __future__ import annotations

import math

TAU = 2.0 * math.pi


def normalize_angle(angle_rad: float) -> float:
    """角度を `-π < a <= π` に正規化する（-π は +π に直す）。"""
    a = (float(angle_rad) + math.pi) % TAU
    if a <= 0.0:  # -π の境界（a == 0 は -π）→ +π
        a += TAU
    return a - math.pi


def base_link_to_world(point, robot_pose):
    """base_link 座標の点 → ワールド座標。

    `robot_pose = (x, y, yaw)`（Gazebo の world フレーム。向きは rad）。
    """
    x, y, yaw = robot_pose
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = float(point[0]), float(point[1])
    return (x + c * bx - s * by, y + s * bx + c * by)


def world_to_base_link(point, robot_pose):
    """ワールド座標の点 → base_link 座標（`base_link_to_world` の逆）。"""
    x, y, yaw = robot_pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = float(point[0]) - x, float(point[1]) - y
    return (c * dx + s * dy, -s * dx + c * dy)


def angle_in_base_link(angle_rad, robot_pose):
    """world 座標の向き（rad）→ base_link 座標の向き（`robot_pose[2]` 込み）。"""
    _, _, yaw = robot_pose
    return normalize_angle(float(angle_rad) - yaw)


def rect_corners(center, width_m, depth_m, normal_rad):
    """ss の盤の矩形の四隅（base_link）。

    ss の `yaw_rad` は「基準の辺の外向き法線の向き」。その辺の長さは `width_m`、
    奥行き方向の長さは `depth_m`。
    """
    cx, cy = float(center[0]), float(center[1])
    w, d = float(width_m), float(depth_m)
    # 辺の向き（法線の垂直）
    ex, ey = -math.sin(normal_rad), math.cos(normal_rad)
    fx, fy = math.cos(normal_rad), math.sin(normal_rad)
    out = []
    for sw, sd in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2)):
        out.append((cx + ex * sw + fx * sd, cy + ey * sw + fy * sd))
    return out


def corner_errors(corners_a, corners_b):
    """2 つの四隅（同じ順）の距離の並び。順がずれる場合は対応をずらして使う。"""
    return [math.hypot(ax - bx, ay - by) for (ax, ay), (bx, by) in zip(corners_a, corners_b)]


def nearest_match(query, candidates):
    """`candidates` の並びから `query` に最も近い要素と、その距離を返す。"""
    q = (float(query[0]), float(query[1]))
    best, best_d = None, None
    for cand in candidates:
        d = math.hypot(float(cand[0]) - q[0], float(cand[1]) - q[1])
        if best_d is None or d < best_d:
            best, best_d = cand, d
    return best, best_d


def panel_compare(ss_panel, truth_panel, robot_pose):
    """ss の盤 1 件分（base_link）と真の盤 1 件（world）を比べる。

    戻り値は `{"center_err_m", "yaw_err_rad", "corner_errs_m", "distance_m"}`。
    真の向きは正面（`front_yaw`）。ss の正面は `front_yaw_rad`（front_known のときだけ意味がある）。
    """
    ss_center_world = base_link_to_world((ss_panel["x_m"], ss_panel["y_m"]), robot_pose)
    truth_center = (float(truth_panel["center"][0]), float(truth_panel["center"][1]))
    center_err = math.hypot(ss_center_world[0] - truth_center[0],
                            ss_center_world[1] - truth_center[1])

    if ss_panel.get("front_known"):
        observed_front_world = normalize_angle(float(ss_panel["front_yaw_rad"]) + robot_pose[2])
        yaw_err = normalize_angle(observed_front_world - float(truth_panel["front_yaw"]))
    else:
        yaw_err = None

    ss_corners_world = [
        base_link_to_world(corner, robot_pose)
        for corner in rect_corners((ss_panel["x_m"], ss_panel["y_m"]),
                                   ss_panel["width_m"], ss_panel["depth_m"],
                                   float(ss_panel["yaw_rad"]))
    ]
    # 真の四隅は 2D のみで使う（z は高さで無関係）。
    # ss の矩形には四隅の名札が無いので、真の四隅との対応を回転でずらしながら比べ、
    # 最大誤差が最も小さい組み合わせの距離を取る。向きの定義の相違（90°・180°）を
    # 含めることに加え、両者の回る向きが逆な場合もあるので逆向き経路も試す
    # （front_known=False のときは表裏が決まらないため）。選んだ組み合わせも返す。
    top = truth_panel.get("top_corners") or {}
    truth_corners = []
    for key in ("FL", "FR", "BR", "BL"):
        v = top.get(key)
        if v is not None:
            truth_corners.append((float(v[0]), float(v[1])))
    if len(truth_corners) == 4:
        candidates = []
        for mirrored, ss_order in ((False, ss_corners_world),
                                   (True, list(reversed(ss_corners_world)))):
            for shift in range(4):
                rotated = truth_corners[shift:] + truth_corners[:shift]
                errs = corner_errors(ss_order, rotated)
                candidates.append((max(errs), errs, shift, mirrored))
        best_max, corner_errs, best_shift, best_mirrored = min(candidates, key=lambda item: item[0])
        corner_side = ("mirror " if best_mirrored else "") + f"{best_shift * 90}°"
    else:
        corner_errs = []
        corner_side = "0°"
    return {
        "center_err_m": center_err,
        "yaw_err_rad": yaw_err,
        "corner_errs_m": corner_errs,
        "corner_side": corner_side,
        "distance_m": math.hypot(float(ss_panel["x_m"]), float(ss_panel["y_m"])),
    }

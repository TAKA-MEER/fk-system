"""機体の置き場を選ぶ・確かめる純関数（DetailedDesign-sim.md §3.1、WP-SIM-06）。

Blender も ROS 2 も要らない。`build_scene`（`__main__.py`）から呼ばれる。

「見える盤」:
  盤の吊り金具がすべて次の 2 つを満たすもの。
  ① 機体中心からの水平距離が「探す距離 − 余裕（既定 0.3 m）」以内
  ② カメラ（機体中心の真上、ss の走査の高さ）から金具への線分が、
     障害物とほかの盤の箱に当たらない

探す距離・カメラの高さは ss の `locator.yaml` から生成のたびに読む
（`load_locator_params`）。機体の半径は th の `nav2_params_sim.yaml` の
`robot_radius`（`load_robot_radius`）。読めなければ止める（既定の数字で
黙って進めない）。
"""

import math
import os

import yaml

from . import model

EPS = 1e-9
DISTANCE_MARGIN_M = 0.3   # 探す距離から引く余裕（既定 0.3 m）
FOOTPRINT_MARGIN_M = 0.2  # 機体の半径に足す余裕（既定 0.2 m）
GRID_STEP_M = 0.25        # 置ける所を探す格子の幅（既定 0.25 m）


class StartPoseError(model.SceneError):
    """ss・th の設定が読めない、置ける所が無い等の誤り。"""


def load_locator_params(ss_repo):
    """ss の locator.yaml から (探す距離, カメラの高さ) を読む。読めなければ止める。"""
    path = os.path.join(ss_repo, "ss_ws", "src", "ss_locator", "config", "locator.yaml")
    try:
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise StartPoseError(
            f"ss の設定 '{path}' を読めません（数字の既定では進めません）: {e}") from e
    try:
        params = raw["locator_node"]["ros__parameters"]
        return (float(params["max_distance_default_m"]),
                float(params["scan_camera_height_m"]))
    except (TypeError, KeyError, ValueError) as e:
        raise StartPoseError(
            f"ss の設定 '{path}' に max_distance_default_m か scan_camera_height_m がありません: {e}") from e


def load_robot_radius(nav2_params_path):
    """nav2 のパラメータから `robot_radius` を読む（最初に見つかったもの）。読めなければ止める。"""
    try:
        with open(nav2_params_path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise StartPoseError(
            f"th の設定 '{nav2_params_path}' を読めません（数字の既定では進めません）: {e}") from e

    def find(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key == "robot_radius" and isinstance(value, (int, float)) \
                        and not isinstance(value, bool):
                    return float(value)
                found = find(value)
                if found is not None:
                    return found
        elif isinstance(node, list):
            for item in node:
                found = find(item)
                if found is not None:
                    return found
        return None

    radius = find(raw)
    if radius is None:
        raise StartPoseError(
            f"th の設定 '{nav2_params_path}' に robot_radius がありません")
    return radius


def _segment_hits_box(p0, p1, center_xy, yaw, half_x, half_y, z0, z1):
    """線分が回転した箱（z0〜z1 の範囲）に当たるか。箱のローカル座標で調べる。"""
    cx, cy = center_xy
    c, s = math.cos(yaw), math.sin(yaw)
    # ワールド -> ローカル（回転 -yaw の後、中心でずらす）
    def to_local(p):
        dx, dy = p[0] - cx, p[1] - cy
        return (c * dx + s * dy, -s * dx + c * dy, p[2])

    a, b = to_local(p0), to_local(p1)
    t_enter, t_exit = 0.0, 1.0
    for axis, half, zlo, zhi in ((0, half_x, None, None), (1, half_y, None, None), (2, None, z0, z1)):
        lo = -half if half is not None else zlo
        hi = half if half is not None else zhi
        origin = a[axis]
        direction = b[axis] - a[axis]
        if abs(direction) < EPS:
            if origin < lo or origin > hi:
                return False
            continue
        t1 = (lo - origin) / direction
        t2 = (hi - origin) / direction
        t_enter = max(t_enter, min(t1, t2))
        t_exit = min(t_exit, max(t1, t2))
        if t_exit < t_enter:
            return False
    return t_exit >= 0.0 and t_enter <= 1.0


def _point_rect_distance(x, y, center_xy, yaw, half_x, half_y):
    """回転した塗りつぶし矩形までの距離（中に入れば 0）。"""
    cx, cy = center_xy
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - cx, y - cy
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    ex = max(abs(lx) - half_x, 0.0)
    ey = max(abs(ly) - half_y, 0.0)
    return math.hypot(ex, ey)


def _fits(x, y, radius, scene):
    """機体の円（半径 radius）が壁・盤・障害物に当たらないか。"""
    room = scene["room"]
    inset = room.get("wall_t", 0.0) / 2  # 壁の内側の面は wall_t/2 入った所
    if (x < inset + radius - EPS or x > room["size_x"] - inset - radius + EPS
            or y < inset + radius - EPS or y > room["size_y"] - inset - radius + EPS):
        return False
    for panel in scene["panels"]:
        d = _point_rect_distance(x, y, panel["center"], model.panel_yaw(panel),
                                 panel["width"] / 2, panel["depth"] / 2)
        if d < radius - EPS:
            return False
    for ob in scene["obstacles"]:
        d = _point_rect_distance(x, y, ob["center"], ob["yaw"],
                                 ob["size"][0] / 2, ob["size"][1] / 2)
        if d < radius - EPS:
            return False
    return True


def _blockers(scene):
    """線分を遮りうる箱の一覧（障害物とすべての盤）。"""
    boxes = []
    for ob in scene["obstacles"]:
        boxes.append((ob["center"], ob["yaw"], ob["size"][0] / 2, ob["size"][1] / 2,
                      0.0, ob["size"][2]))
    for panel in scene["panels"]:
        boxes.append((panel["center"], model.panel_yaw(panel), panel["width"] / 2,
                      panel["depth"] / 2, 0.0, panel["height"], panel["id"]))
    return boxes


def visible_panels_at(scene, x, y, search_distance_m, camera_height_m,
                      distance_margin_m=DISTANCE_MARGIN_M):
    """機体中心 (x, y) から見える盤を数える。

    戻り値は (見える盤の id の並び, [{"id", "reason": "far"|"blocked"}])。
    理由は距離を先に確かめる（遠ければ blocked より先に far）。
    金具の無い盤（lugs: none）は ss が組めないので見えない盤（blocked）なす。
    """
    threshold = search_distance_m - distance_margin_m
    camera = (x, y, camera_height_m)
    boxes = _blockers(scene)
    visible, hidden = [], []
    for panel in scene["panels"]:
        lugs = model.lug_positions(panel)
        if not lugs:
            hidden.append({"id": panel["id"], "reason": "blocked"})
            continue
        points = [(lug["ring_center"][0], lug["ring_center"][1], lug["ring_center"][2])
                  for lug in lugs.values()]
        if any(math.hypot(px - x, py - y) > threshold + EPS for px, py, _ in points):
            hidden.append({"id": panel["id"], "reason": "far"})
            continue
        blocked = False
        for lug in points:
            for box in boxes:
                if len(box) == 7 and box[6] == panel["id"]:
                    continue  # 自分の盤の箱は遮りにならない
                center, yaw, hx, hy, z0, z1 = box[:6]
                if _segment_hits_box(camera, lug, center, yaw, hx, hy, z0, z1):
                    blocked = True
                    break
            if blocked:
                break
        if blocked:
            hidden.append({"id": panel["id"], "reason": "blocked"})
        else:
            visible.append(panel["id"])
    return visible, hidden


def _grid_values(size, step):
    count = int(math.floor(size / step + 1e-9))
    return [round(i * step, 10) for i in range(count + 1)]


def choose_start_pose(scene, search_distance_m, camera_height_m, robot_radius_m,
                      distance_margin_m=DISTANCE_MARGIN_M,
                      footprint_margin_m=FOOTPRINT_MARGIN_M,
                      grid_step_m=GRID_STEP_M):
    """`robot` を記述に書いていないときの置き場を選ぶ（決定論的）。

    見える盤が最も多い所 → 同数なら見える盤までのいちばん遠い距離が短い所
    → 格子の順（x が外側、内側が y。どちらも昇順）。向きは見える盤の中心の
    平均へ。同じ記述からは同じ置き場。
    """
    radius = robot_radius_m + footprint_margin_m
    room = scene["room"]
    xs = _grid_values(room["size_x"], grid_step_m)
    ys = _grid_values(room["size_y"], grid_step_m)
    best = None  # (評価値, (x, y), 見える盤)
    for x in xs:
        for y in ys:
            if not _fits(x, y, radius, scene):
                continue
            visible, _ = visible_panels_at(scene, x, y, search_distance_m, camera_height_m,
                                           distance_margin_m)
            centers = {p["id"]: p["center"] for p in scene["panels"]}
            if visible:
                max_dist = max(math.hypot(centers[pid][0] - x, centers[pid][1] - y)
                               for pid in visible)
            else:
                max_dist = 0.0
            key = (-len(visible), max_dist, x, y)
            if best is None or key < best[0]:
                best = (key, (x, y), visible)
    if best is None:
        raise StartPoseError("機体を置ける所がありません（部屋が狭すぎるか、障害物が多すぎます）")
    _, (x, y), visible = best
    visible, hidden = visible_panels_at(scene, x, y, search_distance_m, camera_height_m,
                                        distance_margin_m)
    if visible:
        centers = {p["id"]: p["center"] for p in scene["panels"]}
        mx = sum(centers[pid][0] for pid in visible) / len(visible)
        my = sum(centers[pid][1] for pid in visible) / len(visible)
        yaw = math.atan2(my - y, mx - x) if math.hypot(mx - x, my - y) > EPS else 0.0
    else:
        yaw = 0.0
    return {"pose": [x, y, yaw], "pose_source": "auto",
            "visible_panels": visible, "hidden_panels": hidden}


def evaluate_given_pose(scene, pose, search_distance_m, camera_height_m,
                        distance_margin_m=DISTANCE_MARGIN_M):
    """`robot` が記述にあるとき: 置き場はそのままで、見える盤だけ数える。"""
    x, y = pose[0], pose[1]
    visible, hidden = visible_panels_at(scene, x, y, search_distance_m, camera_height_m,
                                        distance_margin_m)
    return {"pose": [float(v) for v in pose], "pose_source": "given",
            "visible_panels": visible, "hidden_panels": hidden}


def resolve_robot(scene, ss_repo, nav2_params_path):
    """設定を読み、置き場を決める。`__main__.build_scene` から呼ぶ。"""
    search_distance, camera_height = load_locator_params(ss_repo)
    robot_radius = load_robot_radius(nav2_params_path)
    if scene.get("robot") is None:
        return choose_start_pose(scene, search_distance, camera_height, robot_radius)
    return evaluate_given_pose(scene, scene["robot"]["pose"], search_distance, camera_height)

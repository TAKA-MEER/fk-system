"""WP-SIM-11: 単盤 P004 の描画切り分け A・B の純関数（ホストで回す試験の対象）。

姿勢の組み立て（立ち位置・ss の `scan_plan` の段）・金具の近い／遠いの振り分け・
検出との突き合わせ（15 px 以内）をここに置く。描画・検出はしない。

ss の `scan_plan` は読み取りで import する（写さない）。
立ち位置は `fk_auto.core.stand_pose` で計算する（stand_distance_min_m 0.389・
stand_margin_m 0.5 は `fk_ws/src/fk_bringup/config/params.yaml` の値）。
"""

import importlib.util
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "fk_ws", "src", "fk_auto"))

from fk_auto.core import Panel, stand_pose  # noqa: E402

SS_REPO_DEFAULT = "C:/Users/yisah/Documents/code/MIRS-local/TM/ss-system"

# P004（public_room.yaml）: 中心 (3.0, 3.0)・幅 1.2・奥行 1.0・高さ 2.35。
# 盤前の認識し直しでは機体から見えた北側を正面とみなす（WP-SIM-10 の記録）ので、
# fk_auto 側の front_yaw は北向き (pi/2)。幅は正面の辺に沿う長さなので北の面は中心から
# width/2 = 0.6 m、正面の辺の長さは depth = 1.0 m。
STAND_EXPECTED_Y = 4.489  # 3.0 + 0.6 + 0.389 + 0.5（数で書く。試験が縛る）
PANEL = {"id": "P004", "x": 3.0, "y": 3.0, "width": 1.2, "depth": 1.0, "height": 2.35}
FRONT_YAW_NORTH = math.pi / 2.0
STAND_DISTANCE_MIN_M = 0.389
STAND_MARGIN_M = 0.5
# 記録からの読み取り（WP-SIM-10）。stand_pose の計算値とは食い違うので、別に 1 姿勢入れる。
RECORDED_POSE = {"x": 3.0, "y": 4.66, "yaw": -math.pi / 2.0, "kind": "recorded"}
# 昇降機の高さ: カメラ 3.10 m (scan_camera_height_m) − camera_height_offset_m 1.82 m
LIFT_HEIGHT_M = 3.10 - 1.82
# 後処理のノイズの種。1 姿勢につき 20 個以上。
SEEDS = list(range(20))
MATCH_TOL_PX = 15.0


def load_scan_plan(ss_repo=SS_REPO_DEFAULT):
    """ss の `scan_plan.py` を読み取りで import する（写さない）。"""
    path = os.path.join(ss_repo, "ss_ws", "src", "ss_locator", "ss_locator", "scan_plan.py")
    spec = importlib.util.spec_from_file_location("ss_scan_plan", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _camera_K(ss_repo=SS_REPO_DEFAULT):
    """ss の `locator.yaml` から camera_K（yaml なしで読む）。"""
    path = os.path.join(ss_repo, "ss_ws", "src", "ss_locator", "config", "locator.yaml")
    with open(path, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"camera_K:\s*\[([^\]]+)\]", text)
    vals = [float(v) for v in m.group(1).split(",")]
    return vals  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]


def stand_base_pose():
    """stand_pose の計算値（立ち位置の基準）。

    P004 は正面が西（front_yaw \u03c0）で幅 1.2・奥行 1.0。幅は正面の辺に沿う長さなので
    y 方向に 1.2 m、北の面は中心から 0.6 m（y=3.6）。北を正面とみなすなら
    front_half_m = width/2 = 0.6、front_length_m = depth = 1.0。
    立ち位置は y = 3.0 + 0.6 + 0.389 + 0.5 = 4.489。
    """
    panel = Panel(panel_id=PANEL["id"], x=PANEL["x"], y=PANEL["y"],
                  front_yaw=FRONT_YAW_NORTH,
                  front_half_m=PANEL["width"] / 2.0,
                  front_length_m=PANEL["depth"])
    pose = stand_pose(panel, STAND_DISTANCE_MIN_M, STAND_MARGIN_M)
    return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}


def scan_pitches_deg(ss_repo=SS_REPO_DEFAULT):
    """ss の走査のピッチの段（下向きの度）。`scan_plan.plan_pitches` に渡す値は `locator.yaml`
    の scan_pitch_down_deg 20・scan_view_down_max_deg 70・scan_overlap_ratio 0.1 と vfov。"""
    sp = load_scan_plan(ss_repo)
    K = _camera_K(ss_repo)
    vfov = sp.vfov_from_fy(K[4])
    pitches = sp.plan_pitches(math.radians(20.0), math.radians(70.0), vfov, 0.1)
    return [math.degrees(p) for p in pitches]


def build_poses(ss_repo=SS_REPO_DEFAULT):
    """描く姿勢の一覧。

    位置: 計算の立ち位置を基準に \u00b10.1 m の 4 点と中心、記録の位置 (3.0, 4.66)。
    ピッチ: ss の `scan_plan` の段（下向き度）。中心位置の中段だけヨー \u00b112\u00b0 を足す。
    ヨー: P004 が画角に入る中央（0\u00b0）。
    末尾に、計算の立ち位置と記録の位置で下向き 38.2\u00b0 の段のヨー −24〜+24\u00b0（6\u00b0 刻み）を
    振った `sweep_stand` / `sweep_recorded` を足す（見逃し (b) が出る姿勢を増やすため）。
    """
    sp = load_scan_plan(ss_repo)
    K = _camera_K(ss_repo)
    hfov = sp.hfov_from_fx(K[0])
    center_yaws = sp.plan_yaws(-hfov / 2.0, hfov, hfov, 0.1)  # 画角ちょうどの依頼 → 中央 1 本
    center_yaw_deg = math.degrees(center_yaws[0]) if center_yaws else 0.0
    pitches = scan_pitches_deg(ss_repo)
    base = stand_base_pose()
    positions = [
        {"x": base["x"], "y": base["y"], "yaw": base["yaw"], "kind": "center"},
        {"x": base["x"] + 0.1, "y": base["y"], "yaw": base["yaw"], "kind": "jitter_east"},
        {"x": base["x"] - 0.1, "y": base["y"], "yaw": base["yaw"], "kind": "jitter_west"},
        {"x": base["x"], "y": base["y"] + 0.1, "yaw": base["yaw"], "kind": "jitter_north"},
        {"x": base["x"], "y": base["y"] - 0.1, "yaw": base["yaw"], "kind": "jitter_south"},
        dict(RECORDED_POSE),
    ]
    poses = []
    for pos in positions:
        for pitch in pitches:
            yaws = [center_yaw_deg]
            if pos["kind"] == "center" and abs(pitch - pitches[len(pitches) // 2]) < 1e-9:
                yaws += [center_yaw_deg - 12.0, center_yaw_deg + 12.0]
            for yaw in yaws:
                poses.append({
                    "x": round(pos["x"], 4), "y": round(pos["y"], 4),
                    "yaw": round(pos["yaw"], 6), "height_m": LIFT_HEIGHT_M,
                    "cam_yaw_deg": round(yaw, 3), "cam_pitch_deg": round(-pitch, 3),
                    "kind": pos["kind"],
                })
    # ヨーの振り（下向き 38.2\u00b0 の段 = pitches の中央。計算の立ち位置と記録の位置）
    mid_pitch = pitches[len(pitches) // 2]
    sweep_positions = [
        {"x": base["x"], "y": base["y"], "yaw": base["yaw"], "kind": "sweep_stand"},
        {"x": RECORDED_POSE["x"], "y": RECORDED_POSE["y"], "yaw": RECORDED_POSE["yaw"],
         "kind": "sweep_recorded"},
    ]
    for pos in sweep_positions:
        for yaw in range(-24, 25, 6):
            poses.append({
                "x": round(pos["x"], 4), "y": round(pos["y"], 4),
                "yaw": round(pos["yaw"], 6), "height_m": LIFT_HEIGHT_M,
                "cam_yaw_deg": float(yaw), "cam_pitch_deg": round(-mid_pitch, 3),
                "kind": pos["kind"],
            })
    return poses


def classify_lug(lug_xyz, robot_xy, panel_xy):
    """金具が近い側か遠い側か（機体からの距離が盤の中心より近い＝近い）。"""
    def dist2(a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
    return "near" if dist2(lug_xyz, robot_xy) < dist2(panel_xy, robot_xy) else "far"


# 見逃しの種類の境（リングの投影半径の目安。約 76 px）
EDGE_RING_RADIUS_PX = 76.0


def edge_class(uv, w=1280, h=720):
    """見逃しの種類。(a) 縁で切れている（リング中心が縁から投影半径未満）、
    (b) まるごと画面の中。"""
    return "a" if edge_margin_px(uv, w, h) < EDGE_RING_RADIUS_PX else "b"


def edge_margin_px(uv, w=1280, h=720):
    """リング中心から画角の縁までの px（小さいほうが縁に近い）。"""
    u, v = float(uv[0]), float(uv[1])
    return min(u, w - u, v, h - v)


def match_detections(gt_points, det_centers, tol_px=MATCH_TOL_PX):
    """正解の点と検出の中心を 1 対 1 で突き合わせる（check.py と同じ貪欲）。

    戻り値: 正解ごとの [{found: bool, err_px or None}] と誤検出の数。
    """
    used = set()
    results = []
    for g in gt_points:
        best, best_d = None, None
        for j, c in enumerate(det_centers):
            if j in used:
                continue
            d = math.hypot(c[0] - float(g[0]), c[1] - float(g[1]))
            if best_d is None or d < best_d:
                best, best_d = j, d
        if best is not None and best_d <= tol_px:
            used.add(best)
            results.append({"found": True, "err_px": best_d})
        else:
            results.append({"found": False, "err_px": None})
    return results, len(det_centers) - len(used)


# 描き方（A は金具の見た目だけ変える。既定は base のまま）。
VARIANTS = {
    "base": {"scene_overrides": {}, "renderer_overrides": {}},
    "a_bright": {
        "scene_overrides": {"lug_rgb": (0.95, 0.95, 0.97), "lug_rough": 0.4, "lug_metallic": 0.0},
        "renderer_overrides": {},
    },
    "a_dark": {
        "scene_overrides": {"lug_rgb": (0.15, 0.15, 0.18), "lug_rough": 0.4, "lug_metallic": 0.0},
        "renderer_overrides": {},
    },
    "a_gloss": {
        "scene_overrides": {"lug_rgb": (0.95, 0.95, 0.97), "lug_rough": 0.15, "lug_metallic": 0.0},
        "renderer_overrides": {"metallic_factor": 0.0, "roughness_factor": 0.15},
    },
    "b_cycles": {"scene_overrides": {}, "renderer_overrides": {}},
}

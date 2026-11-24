"""正解（scene.json）の形と、Blender の外接箱と scene.json の比べ。

Blender のバッチ（blender/build_scene.py）は書き出し前にオブジェクトごとに
外接箱（ワールド座標）を blender_objects.json に吐く。glb の盤と吊り金具の
位置が scene.json と合っているかは、このモジュールの compare で確かめる
（trimesh は持っていないので、glb を読まずに済ませる）。
"""

import math

from . import model


def build_answer(scene):
    """検査済みの scene から scene.json の辞書を作る。"""
    panels = []
    for panel in scene["panels"]:
        panels.append({
            "id": panel["id"],
            "center": list(panel["center"]),
            "front_yaw": panel["front_yaw"],
            "width": panel["width"],
            "depth": panel["depth"],
            "height": panel["height"],
            "label": panel["label"],
            "top_corners": {k: list(v) for k, v in model.top_corners(panel).items()},
            "lugs": {k: {"base": list(v["base"]), "ring_center": list(v["ring_center"])}
                     for k, v in model.lug_positions(panel).items()},
        })
    return {
        "name": scene["name"],
        "visibility": scene["visibility"],
        "room": dict(scene["room"]),
        "panels": panels,
        "obstacles": [{"center": list(o["center"]), "size": list(o["size"]), "yaw": o["yaw"]}
                      for o in scene["obstacles"]],
        "robot": {
            "pose": list(scene["robot"]["pose"]),
            "pose_source": scene["robot"].get("pose_source", "given"),
            "visible_panels": list(scene["robot"].get("visible_panels", [])),
            "hidden_panels": [dict(h) for h in scene["robot"].get("hidden_panels", [])],
        },
    }


def _close(a, b, tol):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def compare_blender_objects(answer, blender_objects, tol_m=0.01):
    """Blender の外接箱（blender_objects.json の中身）と scene.json が合うか。

    blender_objects は {オブジェクト名: {"min": [x,y,z], "max": [x,y,z]}}。
    名前は blender/build_scene.py の決まり:
      panel_<id>_body / panel_<id>_roof / lug_<id>_<corner>
    戻り値は (ok, [問題の文字列])。盤の body は高さ方向 HB〜H-ROOF_T を占め、
    roof は H-ROOF_T〜H を占める（model.HB・model.ROOF_T）。
    """
    problems = []
    by_id = {p["id"]: p for p in answer["panels"]}
    for pid, panel in by_id.items():
        corners = panel["top_corners"]
        xs = [c[0] for c in corners.values()]
        ys = [c[1] for c in corners.values()]
        # 回転した盤の外接箱は天面の四隅の min/max（回転は z だけに効くから）
        exp_min, exp_max = [min(xs), min(ys)], [max(xs), max(ys)]
        body = blender_objects.get(f"panel_{pid}_body")
        roof = blender_objects.get(f"panel_{pid}_roof")
        for label, obj, zmin, zmax in (("body", body, model.HB, panel["height"] - model.ROOF_T),
                                       ("roof", roof, panel["height"] - model.ROOF_T, panel["height"])):
            if obj is None:
                problems.append(f"panel {pid} の {label} が blender_objects.json にありません")
                continue
            if not _close(obj["min"][:2], exp_min, tol_m) or not _close(obj["max"][:2], exp_max, tol_m):
                problems.append(f"panel {pid} の {label} の XY が scene.json と合いません: "
                                f"{obj['min'][:2]}..{obj['max'][:2]} != {exp_min}..{exp_max}")
            if abs(obj["min"][2] - zmin) > tol_m or abs(obj["max"][2] - zmax) > tol_m:
                problems.append(f"panel {pid} の {label} の Z が scene.json と合いません: "
                                f"{obj['min'][2]:.4f}..{obj['max'][2]:.4f} != {zmin:.4f}..{zmax:.4f}")
        for corner, lug in panel["lugs"].items():
            obj = blender_objects.get(f"lug_{pid}_{corner}")
            if obj is None:
                problems.append(f"panel {pid} の金具 {corner} が blender_objects.json にありません")
                continue
            cx = (obj["min"][0] + obj["max"][0]) / 2
            cy = (obj["min"][1] + obj["max"][1]) / 2
            ex, ey = lug["base"][0], lug["base"][1]
            if math.hypot(cx - ex, cy - ey) > tol_m:
                problems.append(f"panel {pid} の金具 {corner} の中心が scene.json と合いません: "
                                f"({cx:.4f}, {cy:.4f}) != ({ex:.4f}, {ey:.4f})")
            if abs(obj["min"][2] - lug["base"][2]) > tol_m:
                problems.append(f"panel {pid} の金具 {corner} の座面が scene.json と合いません: "
                                f"{obj['min'][2]:.4f} != {lug['base'][2]:.4f}")
    return (not problems), problems

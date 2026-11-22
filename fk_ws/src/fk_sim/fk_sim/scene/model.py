"""シーンの記述（YAML）の読み込み・検査と、盤の幾何（天面・吊り金具）。

記述の形（Gazebo のワールド座標。単位は m・rad）:

    visibility: public | site      # site は本リポジトリの中へ書けない（guard）
    name: public_room              # 出力先 sim_out/scenes/<name>/ の名前
    room:
      size_x: 12.0                 # 部屋の内法。部屋は x in [0, size_x], y in [0, size_y]
      size_y: 8.0
      ceiling_h: 4.0               # 天井の高さ
      wall_h: 4.0                  # 壁の高さ（既定は ceiling_h）
      wall_t: 0.15                 # 壁の厚さ（既定 0.15）
      floor_color: [0.6, 0.6, 0.6] # RGB（既定は下の既定値）
      wall_color: [0.8, 0.8, 0.8]
    panels:                        # 1 面以上
      - id: P001                   # 省略時は P001 から自動
        center: [x, y]             # 盤底面中心（床上）
        front_yaw: 0.0             # 正面（扉のある側）の向き。正面の外向き法線の yaw
        width: 0.8                 # 幅（左右）と奥行き
        depth: 0.8
        height: 2.35               # 高さ（既定 2.35 m）
        label: "No.1"              # 名札の文字（広報版は架空の文字だけ）
        lugs: four                 # four（四隅）| none。既定は four
    obstacles:                     # 箱。床に置く
      - center: [x, y]
        size: [sx, sy, sz]
        yaw: 0.0                   # 省略時は 0
    robot:
      pose: [x, y, yaw]            # 機体の置き場
    lights:                        # 照明（描画サービスの既定の強さに使う）
      - type: point                # point | sun | ambient
        position: [x, y, z]        # point のとき必須
        energy: 1.0                # 省略時は 1.0
    assets: []                     # 任意。手作り Blender への差し込み先（今は未使用）

盤の向きの決まり（試作の生成器 gen_scene.py と同じ）:
ローカル座標は +X が幅、+Y が背面、+Z が上。正面はローカル -Y 側。
front_yaw を φ、ローカル座標の回転を θ とすると θ = φ + π/2。
ローカル点 (lx, ly) のワールド位置は
  center + (cosθ*lx - sinθ*ly, sinθ*lx + cosθ*ly)。

吊り金具は天面の四隅に inset 0.07 m ずつ内側へ置く（試作の生成器は
0.05〜0.085 m の乱数。生成物は再現可能にするため固定値にした）。
リングの中心は天面から EB_RING_Z_M たけ上（下の EB_* 参照）。
"""

import math

import yaml

# 盤・吊り金具の寸法の出典: lifting-lug-pose の src/blender/gen_scene.py
# （コミット 9265d2d）の先頭の定数と eyebolt_mesh。読むだけで写さない。
PANEL_H = 2.35                    # 盤の高さの既定（gen_scene.py PANEL_H）
HB = 0.10                         # チャンネルベースの高さ（gen_scene.py HB）
ROOF_T = 0.025                    # 天板の厚さ（gen_scene.py ROOF_T）
EB_RING_OD, EB_RING_ID, EB_RING_T = 0.0723, 0.0395, 0.0159  # JIS B 1168 M20 実測
EB_HOLE_Z, EB_TOP_Z = 0.0164, 0.073                          # 座面〜穴下端・リング頂部
EB_SEAT_D, EB_NECK_D = 0.0349, 0.0308                        # 座・下端の径
EB_SEAT_H = 0.015                                            # 座の高さ

# リング中心の天面からの高さ（eyebolt_mesh の zc。scale は 1 とする）
EB_RING_Z_M = (EB_HOLE_Z + EB_RING_ID / 2 + EB_TOP_Z - EB_RING_OD / 2) / 2

LUG_INSET_M = 0.07                 # 金具を四隅から内側へ入れる量（試作は乱数 0.05〜0.085）
LUG_CORNERS = ("FL", "FR", "BL", "BR")  # gen_scene.py CORNERS と同じ順

EPS = 1e-9


class SceneError(ValueError):
    """記述の誤り（欠けた欄・盤の重なり・壁の外など）。分かる言葉で出す。"""


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _req(mapping, key, where):
    if not isinstance(mapping, dict) or key not in mapping:
        raise SceneError(f"{where} に欄 '{key}' がありません")
    return mapping[key]


def _num(value, where):
    if not _is_num(value):
        raise SceneError(f"{where} は数でなければいけません（'{value}'）")
    return float(value)


def _vec2(value, where):
    if not (isinstance(value, (list, tuple)) and len(value) == 2
            and all(_is_num(v) for v in value)):
        raise SceneError(f"{where} は [x, y] の数 2 つでなければいけません（'{value}'）")
    return [float(value[0]), float(value[1])]


def _vec3(value, where):
    if not (isinstance(value, (list, tuple)) and len(value) == 3
            and all(_is_num(v) for v in value)):
        raise SceneError(f"{where} は [x, y, z] の数 3 つでなければいけません（'{value}'）")
    return [float(value[0]), float(value[1]), float(value[2])]


def _color(value, where):
    c = _vec3(value, where)
    if any(v < 0.0 or v > 1.0 for v in c):
        raise SceneError(f"{where} は 0〜1 の RGB でなければいけません（'{value}'）")
    return c


def load_scene(path):
    """YAML を読んで検査済みの dict を返す。誤りは SceneError。"""
    try:
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise SceneError(f"記述 '{path}' を読めません: {e}") from e
    return validate_scene(raw, str(path))


def validate_scene(raw, where="記述"):
    """生の dict を検査して既定値を入れた dict を返す。"""
    if not isinstance(raw, dict):
        raise SceneError(f"{where} は YAML の辞書でなければいけません")
    scene = {}
    visibility = _req(raw, "visibility", where)
    if visibility not in ("public", "site"):
        raise SceneError(f"{where} の visibility は public か site でなければいけません（'{visibility}'）")
    scene["visibility"] = visibility
    name = _req(raw, "name", where)
    if not isinstance(name, str) or not name:
        raise SceneError(f"{where} の name は空でない文字でなければいけません")
    scene["name"] = name

    room_raw = _req(raw, "room", where)
    room = {}
    room["size_x"] = _num(_req(room_raw, "size_x", "room"), "room.size_x")
    room["size_y"] = _num(_req(room_raw, "size_y", "room"), "room.size_y")
    room["ceiling_h"] = _num(_req(room_raw, "ceiling_h", "room"), "room.ceiling_h")
    room["wall_h"] = _num(raw.get("room", {}).get("wall_h", room["ceiling_h"]), "room.wall_h")
    room["wall_t"] = _num(raw.get("room", {}).get("wall_t", 0.15), "room.wall_t")
    room["floor_color"] = _color(raw.get("room", {}).get("floor_color", [0.55, 0.55, 0.55]),
                                 "room.floor_color")
    room["wall_color"] = _color(raw.get("room", {}).get("wall_color", [0.8, 0.8, 0.78]),
                                "room.wall_color")
    for k in ("size_x", "size_y", "ceiling_h", "wall_h", "wall_t"):
        if room[k] <= 0:
            raise SceneError(f"room.{k} は正の数でなければいけません（{room[k]}）")
    scene["room"] = room

    panels_raw = _req(raw, "panels", where)
    if not isinstance(panels_raw, list) or not panels_raw:
        raise SceneError("panels は盤を 1 面以上持たなければいけません")
    panels = []
    seen_ids = set()
    for i, p in enumerate(panels_raw):
        w = f"panels[{i}]"
        if not isinstance(p, dict):
            raise SceneError(f"{w} は辞書でなければいけません")
        panel = {}
        pid = p.get("id", f"P{i + 1:03d}")
        if not isinstance(pid, str) or not pid:
            raise SceneError(f"{w}.id は空でない文字でなければいけません")
        if pid in seen_ids:
            raise SceneError(f"盤の id '{pid}' が重なっています")
        seen_ids.add(pid)
        panel["id"] = pid
        panel["center"] = _vec2(_req(p, "center", w), f"{w}.center")
        panel["front_yaw"] = _num(_req(p, "front_yaw", w), f"{w}.front_yaw")
        panel["width"] = _num(_req(p, "width", w), f"{w}.width")
        panel["depth"] = _num(_req(p, "depth", w), f"{w}.depth")
        panel["height"] = _num(p.get("height", PANEL_H), f"{w}.height")
        panel["label"] = str(p.get("label", ""))
        panel["lugs"] = p.get("lugs", "four")
        if panel["lugs"] not in ("four", "none"):
            raise SceneError(f"{w}.lugs は four か none でなければいけません（'{panel['lugs']}'）")
        if panel["width"] <= 0 or panel["depth"] <= 0 or panel["height"] <= 0:
            raise SceneError(f"{w} の width・depth・height は正の数でなければいけません")
        panels.append(panel)
    scene["panels"] = panels

    obstacles_raw = raw.get("obstacles", [])
    if not isinstance(obstacles_raw, list):
        raise SceneError("obstacles は一覧でなければいけません")
    obstacles = []
    for i, o in enumerate(obstacles_raw):
        w = f"obstacles[{i}]"
        if not isinstance(o, dict):
            raise SceneError(f"{w} は辞書でなければいけません")
        ob = {"center": _vec2(_req(o, "center", w), f"{w}.center"),
              "size": _vec3(_req(o, "size", w), f"{w}.size"),
              "yaw": _num(o.get("yaw", 0.0), f"{w}.yaw"),
              "label": str(o.get("label", f"obstacle_{i}"))}
        if any(s <= 0 for s in ob["size"]):
            raise SceneError(f"{w}.size は正の数でなければいけません")
        obstacles.append(ob)
    scene["obstacles"] = obstacles

    robot_raw = _req(raw, "robot", where)
    pose = _req(robot_raw, "pose", "robot")
    if not (isinstance(pose, (list, tuple)) and len(pose) == 3 and all(_is_num(v) for v in pose)):
        raise SceneError("robot.pose は [x, y, yaw] の数 3 つでなければいけません")
    scene["robot"] = {"pose": [float(v) for v in pose]}

    lights_raw = _req(raw, "lights", where)
    if not isinstance(lights_raw, list):
        raise SceneError("lights は一覧でなければいけません")
    lights = []
    for i, li in enumerate(lights_raw):
        w = f"lights[{i}]"
        if not isinstance(li, dict):
            raise SceneError(f"{w} は辞書でなければいけません")
        ltype = li.get("type", "point")
        if ltype not in ("point", "sun", "ambient"):
            raise SceneError(f"{w}.type は point・sun・ambient のどれかでなければいけません（'{ltype}'）")
        light = {"type": ltype, "energy": _num(li.get("energy", 1.0), f"{w}.energy")}
        if ltype == "point":
            light["position"] = _vec3(_req(li, "position", w), f"{w}.position")
        lights.append(light)
    scene["lights"] = lights

    scene["assets"] = raw.get("assets", []) or []
    _check_geometry(scene)
    return scene


def panel_yaw(panel):
    """ローカル座標の回転 θ。front_yaw を φ とすると θ = φ + π/2（モジュールの docstring）。"""
    return panel["front_yaw"] + math.pi / 2


def local_to_world(panel, lx, ly):
    """盤ローカルの (lx, ly) をワールド (x, y) にする。"""
    th = panel_yaw(panel)
    c, s = math.cos(th), math.sin(th)
    cx, cy = panel["center"]
    return (cx + c * lx - s * ly, cy + s * lx + c * ly)


def top_corners(panel):
    """天面の四隅（ワールド [x, y, z]）。順は FL・FR・BL・BR（gen_scene.py と同じ）。"""
    w, d, h = panel["width"], panel["depth"], panel["height"]
    local = {"FL": (-w / 2, -d / 2), "FR": (w / 2, -d / 2),
             "BL": (-w / 2, d / 2), "BR": (w / 2, d / 2)}
    out = {}
    for corner in LUG_CORNERS:
        x, y = local_to_world(panel, *local[corner])
        out[corner] = [x, y, h]
    return out


def lug_positions(panel):
    """吊り金具の位置（ワールド [x, y, z] の base と ring_center）。lugs=none なら空。"""
    if panel["lugs"] == "none":
        return {}
    w, d, h = panel["width"], panel["depth"], panel["height"]
    inset = LUG_INSET_M
    local = {"FL": (-w / 2 + inset, -d / 2 + inset), "FR": (w / 2 - inset, -d / 2 + inset),
             "BL": (-w / 2 + inset, d / 2 - inset), "BR": (w / 2 - inset, d / 2 - inset)}
    out = {}
    for corner in LUG_CORNERS:
        x, y = local_to_world(panel, *local[corner])
        out[corner] = {"base": [x, y, h],
                       "ring_center": [x, y, h + EB_RING_Z_M]}
    return out


def panel_corners_xy(panel):
    w, d = panel["width"], panel["depth"]
    return [local_to_world(panel, lx, ly)
            for lx, ly in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2))]


def obstacle_corners_xy(ob):
    cx, cy = ob["center"]
    sx, sy = ob["size"][0], ob["size"][1]
    c, s = math.cos(ob["yaw"]), math.sin(ob["yaw"])
    return [(cx + c * lx - s * ly, cy + s * lx + c * ly)
            for lx, ly in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]


def _inside_room(pt, room, what):
    # 壁は部屋の境界を中心に厚さ wall_t で置くので、内側の面は wall_t/2 入った所。
    inset = room.get("wall_t", 0.0) / 2
    x, y = pt
    if (x < inset - EPS or x > room["size_x"] - inset + EPS
            or y < inset - EPS or y > room["size_y"] - inset + EPS):
        raise SceneError(f"{what} が壁の外に出ています（x={x:.3f}, y={y:.3f}）")


def _rects_overlap(a, b):
    """2 つの回転矩形（四隅の列）が重なるか。分離軸定理で確かめる。"""
    axes = []
    for rect in (a, b):
        for i in (0, 1):
            x1, y1 = rect[i]
            x2, y2 = rect[(i + 1) % 4]
            ex, ey = x2 - x1, y2 - y1
            axes.append((-ey, ex))
    for ax, ay in axes:
        norm = math.hypot(ax, ay)
        if norm < EPS:
            continue
        ax, ay = ax / norm, ay / norm
        pa = [ax * x + ay * y for x, y in a]
        pb = [ax * x + ay * y for x, y in b]
        if max(pa) <= min(pb) + EPS or max(pb) <= min(pa) + EPS:
            return False  # テーブルが接するだけは重なりとはしない（連盤は面で接する）
    return True


def _check_geometry(scene):
    room = scene["room"]
    for panel in scene["panels"]:
        corners = panel_corners_xy(panel)
        for pt in corners:
            _inside_room(pt, room, f"盤 {panel['id']}")
        if panel["height"] > room["ceiling_h"] + EPS:
            raise SceneError(f"盤 {panel['id']} の高さが天井（{room['ceiling_h']} m）を超えています")
    for ob in scene["obstacles"]:
        for pt in obstacle_corners_xy(ob):
            _inside_room(pt, room, f"障害物 '{ob['label']}'")
    rx, ry = scene["robot"]["pose"][0], scene["robot"]["pose"][1]
    _inside_room((rx, ry), room, "機体")
    panels = scene["panels"]
    for i in range(len(panels)):
        for j in range(i + 1, len(panels)):
            a, b = panels[i], panels[j]
            if _rects_overlap(panel_corners_xy(a), panel_corners_xy(b)):
                raise SceneError(f"盤 {a['id']} と盤 {b['id']} が重なっています")
    for panel in panels:
        pc = panel_corners_xy(panel)
        for ob in scene["obstacles"]:
            if min(panel["height"], ob["size"][2]) <= EPS:
                continue
            if _rects_overlap(pc, obstacle_corners_xy(ob)):
                raise SceneError(f"盤 {panel['id']} と障害物 '{ob['label']}' が重なっています")

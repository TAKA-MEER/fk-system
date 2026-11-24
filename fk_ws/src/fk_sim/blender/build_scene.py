# -*- coding: utf-8 -*-
"""シーンの記述から描画用の形（scene.glb）を Blender バッチで作る。

使い方: blender -b -P build_scene.py -- --scene <記述.yaml> --out <出力先>
scene.glb と、書き出し前に吐いたオブジェクトごとの外接箱
(blender_objects.json) を出力先に書く。外接箱と scene.json の比べは
生成器側（fk_sim.scene.answers.compare_blender_objects）がする。

Blender の Python には yaml が無い（Blender 4.2 で確認）ので、yaml が
読めないときは生成器が先に書いた scene.json を使う（同じ記述から作ったもの）。
作るもの: 床・壁・天井・盤（ベース・本体・天板・名札・吊り金具）・障害物・機体。
色は基本色だけ（FK-E1 で手続き的な材質が落ちたため）。

盤の寸法・向きの決まりは fk_sim.scene.model と同じ：ローカル +Y が背面、
正面はローカル -Y 側。front_yaw を φ とするとローカルの回転は θ = φ + π/2。
"""

import bpy
import json
import math
import os
import sys

EB_RING_OD, EB_RING_ID = 0.0723, 0.0395
EB_SEAT_D, EB_SEAT_H = 0.0349, 0.015
EB_RING_Z = (0.0164 + 0.0395 / 2 + 0.073 - 0.0723 / 2) / 2  # fk_sim.scene.model と同じ式
HB, ROOF_T = 0.10, 0.025
LUG_INSET = 0.07
CORNERS = ("FL", "FR", "BL", "BR")


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--scene", required=True)
    p.add_argument("--out", required=True)
    return p.parse_args(argv)


def _robot_from_answer(out_dir):
    with open(os.path.join(out_dir, "scene.json"), encoding="utf-8") as f:
        return json.load(f)["robot"]


def load_description(scene_path, out_dir):
    """yaml があれば YAML、無ければ生成器の scene.json から描く用の情報を作る。"""
    try:
        import yaml
        with open(scene_path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        return {
            "name": raw["name"], "room": raw["room"],
            "panels": [{"id": p.get("id", f"P{i + 1:03d}"), "center": p["center"],
                        "front_yaw": p["front_yaw"], "width": p["width"], "depth": p["depth"],
                        "height": p.get("height", 2.35), "label": p.get("label", ""),
                        "lugs": p.get("lugs", "four")} for i, p in enumerate(raw["panels"])],
            "obstacles": raw.get("obstacles", []),
            "robot": raw.get("robot") or _robot_from_answer(out_dir),
        }
    except ImportError:
        with open(os.path.join(out_dir, "scene.json"), encoding="utf-8") as f:
            ans = json.load(f)
        print("[build_scene] yaml が無いので scene.json から描きます:", scene_path)
        return {
            "name": ans["name"], "room": ans["room"],
            "panels": [{"id": p["id"], "center": p["center"], "front_yaw": p["front_yaw"],
                        "width": p["width"], "depth": p["depth"], "height": p["height"],
                        "label": p.get("label", ""), "lugs": "four" if p.get("lugs") else "none"}
                       for p in ans["panels"]],
            "obstacles": ans["obstacles"], "robot": ans["robot"],
        }


def panel_yaw(panel):
    return panel["front_yaw"] + math.pi / 2


def local_to_world(panel, lx, ly):
    th = panel_yaw(panel)
    c, s = math.cos(th), math.sin(th)
    cx, cy = panel["center"]
    return (cx + c * lx - s * ly, cy + s * lx + c * ly)


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def make_material(name, rgb, rough=0.6):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (rgb[0], rgb[1], rgb[2], 1.0)
    bsdf.inputs["Metallic"].default_value = 0.0
    bsdf.inputs["Roughness"].default_value = rough
    return mat


def add_box(name, size, loc, rot_z, mat):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=loc, rotation=(0, 0, rot_z))
    obj = bpy.context.active_object
    obj.name = name
    obj.dimensions = size
    if mat is not None:
        obj.data.materials.append(mat)
    return obj


def add_eyebolt(name, loc, rot_z, mat):
    """簡易のアイボルト（座＋リング）。原点が座面の中心。"""
    R = (EB_RING_OD + EB_RING_ID) / 4
    a = (EB_RING_OD - EB_RING_ID) / 4
    bpy.ops.mesh.primitive_torus_add(major_radius=R, minor_radius=a,
                                     major_segments=48, minor_segments=12,
                                     location=(0, 0, EB_RING_Z), rotation=(math.pi / 2, 0, 0))
    ring = bpy.context.active_object
    bpy.ops.mesh.primitive_cylinder_add(radius=EB_SEAT_D / 2, depth=EB_SEAT_H,
                                        vertices=24, location=(0, 0, EB_SEAT_H / 2))
    seat = bpy.context.active_object
    bpy.ops.object.select_all(action="DESELECT")
    ring.select_set(True)
    seat.select_set(True)
    bpy.context.view_layer.objects.active = ring
    # 結合前に各オブジェクトの変形をメッシュへ焼き込む（結合で位置がずれるのを防ぐ）
    for obj in (ring, seat):
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.context.view_layer.objects.active = ring
    bpy.ops.object.join()
    obj = bpy.context.active_object
    obj.name = name
    obj.location = loc
    obj.rotation_euler = (0, 0, rot_z)
    obj.data.materials.append(mat)
    return obj


def bounding_boxes():
    bpy.context.view_layer.update()
    out = {}
    for obj in bpy.data.objects:
        if obj.type != "MESH":
            continue
        corners = [obj.matrix_world @ __import__("mathutils").Vector(c)
                   for c in obj.bound_box]
        out[obj.name] = {
            "min": [min(c[i] for c in corners) for i in range(3)],
            "max": [max(c[i] for c in corners) for i in range(3)],
            # 金具（アイボルト）はリング面の水平方向（局所 x）が盤の中心を向く決まりで、
            # ss が盤の広がる側をこの向きで決める。配置の回転を書き出す（確かめ用）。
            "rotation_euler": [float(v) for v in obj.rotation_euler],
        }
    return out


def build(desc):
    room = desc["room"]
    lx, ly = float(room["size_x"]), float(room["size_y"])
    wall_h = float(room.get("wall_h", room["ceiling_h"]))
    wall_t = float(room.get("wall_t", 0.15))
    ceiling = float(room["ceiling_h"])
    floor_mat = make_material("floor", room.get("floor_color", [0.55, 0.55, 0.55]))
    wall_mat = make_material("wall", room.get("wall_color", [0.8, 0.8, 0.78]))
    panel_mat = make_material("panel", [0.7, 0.7, 0.66])
    lug_mat = make_material("lug", [0.6, 0.6, 0.62], rough=0.4)
    obstacle_mat = make_material("obstacle", [0.4, 0.5, 0.6])
    plate_mat = make_material("plate", [0.9, 0.9, 0.88])
    robot_mat = make_material("robot", [0.3, 0.35, 0.4])

    add_box("floor", [lx, ly, 0.1], [lx / 2, ly / 2, -0.05], 0.0, floor_mat)
    add_box("wall_south", [lx, wall_t, wall_h], [lx / 2, 0.0, wall_h / 2], 0.0, wall_mat)
    add_box("wall_north", [lx, wall_t, wall_h], [lx / 2, ly, wall_h / 2], 0.0, wall_mat)
    add_box("wall_west", [wall_t, ly, wall_h], [0.0, ly / 2, wall_h / 2], 0.0, wall_mat)
    add_box("wall_east", [wall_t, ly, wall_h], [lx, ly / 2, wall_h / 2], 0.0, wall_mat)
    add_box("ceiling", [lx, ly, 0.1], [lx / 2, ly / 2, ceiling + 0.05], 0.0, wall_mat)

    for panel in desc["panels"]:
        pid = panel["id"]
        w, d, h = panel["width"], panel["depth"], panel["height"]
        cx, cy = panel["center"]
        th = panel_yaw(panel)
        w_in = w - 0.002
        body_h = h - HB - ROOF_T
        add_box(f"panel_{pid}_body", [w_in, d, body_h], [cx, cy, HB + body_h / 2], th, panel_mat)
        add_box(f"panel_{pid}_roof", [w_in, d + 0.012, ROOF_T], [cx, cy, h - ROOF_T / 2], th, panel_mat)
        add_box(f"panel_{pid}_plate", [min(0.34, w_in - 0.1), 0.004, 0.075],
                list(local_to_world(panel, 0, -d / 2 - 0.002)) + [h - 0.2], th, plate_mat)
        label = panel.get("label", "")
        if label:
            try:
                wx, wy = local_to_world(panel, 0, -d / 2 - 0.006)
                bpy.ops.object.text_add(location=[wx, wy, h - 0.2], rotation=(math.pi / 2, 0, th))
                text = bpy.context.active_object
                text.name = f"panel_{pid}_label"
                text.data.body = label
                text.data.size = 0.05
                text.data.align_x = "CENTER"
                text.data.align_y = "CENTER"
                text.data.materials.append(plate_mat)
                bpy.ops.object.convert(target="MESH")
            except Exception as e:  # 文字体が無い等は名札だけでもよい（描画用の形）
                print(f"[build_scene] 名札の文字を作れませんでした（{e}）。名札だけにします")
        if panel.get("lugs", "four") == "four":
            local = {"FL": (-w / 2 + LUG_INSET, -d / 2 + LUG_INSET),
                     "FR": (w / 2 - LUG_INSET, -d / 2 + LUG_INSET),
                     "BL": (-w / 2 + LUG_INSET, d / 2 - LUG_INSET),
                     "BR": (w / 2 - LUG_INSET, d / 2 - LUG_INSET)}
            for corner in CORNERS:
                x, y = local_to_world(panel, *local[corner])
                # リング面の水平方向（リングの局所 x）は盤の中心を向く（ss がこの向きで
                # 盤の広がる側を決める）。出典: 試作 gen_scene.py 675 行目（9265d2d）。
                ring_yaw = math.atan2(cy - y, cx - x)
                add_eyebolt(f"lug_{pid}_{corner}", [x, y, h], ring_yaw, lug_mat)

    for i, ob in enumerate(desc["obstacles"]):
        cx, cy = ob["center"]
        add_box(f"obstacle_{i}", list(ob["size"]), [cx, cy, ob["size"][2] / 2],
                ob.get("yaw", 0.0), obstacle_mat)

    rx, ry, ryaw = desc["robot"]["pose"]
    add_box("robot", [0.55, 0.55, 0.4], [rx, ry, 0.2], ryaw, robot_mat)


def main():
    args = parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    desc = load_description(args.scene, out)
    clear_scene()
    build(desc)
    boxes = bounding_boxes()
    with open(os.path.join(out, "blender_objects.json"), "w", encoding="utf-8") as f:
        json.dump(boxes, f, ensure_ascii=False, indent=1)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in bpy.data.objects:
        if obj.type == "MESH":
            obj.select_set(True)
    bpy.ops.export_scene.gltf(
        filepath=os.path.join(out, "scene.glb"),
        export_format="GLB", use_selection=True, export_yup=False, export_apply=True)
    print(f"[build_scene] {out} オブジェクト {len(boxes)} 個")


if __name__ == "__main__":
    main()

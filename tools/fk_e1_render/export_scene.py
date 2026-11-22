"""FK-E1: Blender の .blend から Gazebo / 軽い描画向けの形を書き出す。

`gen_scene.py --save-blend` が残した .blend（1 視点 = 1 ファイル）を読み、
- COLLADA `.dae`（Gazebo Classic 用。文字はメッシュに変換する）
- glTF `.glb`（R-PY 用。`export_yup=False` で Blender と同じ Z-up のまま）
- カメラ（内部・外部パラメータ・解像度・画角）と照明の JSON
を 1 つのフォルダに出す。

手続き的なマテリアル（ノイズ・汚れ・バンプ）は書き出しで落ちる。まずは基本色だけ残す
（Principled BSDF の Base Color の既定値をそのまま使い、リンクを外す。金属は軽い描画で
真っ黒になりやすいので metallic=0 にする）。何を落としたかは JSON の `lost` に書く。

使い方:
  blender -b <scene>.blend -P tools/fk_e1_render/export_scene.py -- --out <dir>
"""

import json
import math
import os
import sys

import bpy
from mathutils import Vector


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    return p.parse_args(argv)


def m2l(m):
    return [list(row) for row in m]


def camera_json(scene, cam):
    W = scene.render.resolution_x
    H = scene.render.resolution_y
    lens = float(cam.data.lens)
    sensor = float(cam.data.sensor_width)
    fx = lens / sensor * W
    K = [[fx, 0.0, W / 2.0], [0.0, fx, H / 2.0], [0.0, 0.0, 1.0]]
    hfov = math.degrees(2.0 * math.atan(W / (2.0 * fx)))
    vfov = math.degrees(2.0 * math.atan(H / (2.0 * fx)))
    return {
        "matrix_world": m2l(cam.matrix_world),
        "lens_mm": lens,
        "sensor_width_mm": sensor,
        "resolution": [W, H],
        "K": K,
        "hfov_deg": hfov,
        "vfov_deg": vfov,
        "near": float(cam.data.clip_start),
        "far": float(cam.data.clip_end),
    }


def lights_json():
    out = []
    for obj in bpy.data.objects:
        if obj.type != "LIGHT":
            continue
        ld = obj.data
        item = {
            "name": obj.name,
            "type": ld.type,
            "location": list(obj.matrix_world.translation),
            "rotation_euler": list(obj.rotation_euler),
            "energy": float(ld.energy),
            "color": list(ld.color),
        }
        if ld.type == "AREA":
            item["size"] = float(ld.size)
            item["size_y"] = float(ld.size_y)
            item["shape"] = ld.shape
        elif ld.type == "SUN":
            item["angle"] = float(ld.angle)
        elif ld.type == "SPOT":
            item["spot_size"] = float(ld.spot_size)
            item["spot_blend"] = float(ld.spot_blend)
        out.append(item)
    return out


def world_json(scene):
    try:
        bg = scene.world.node_tree.nodes["Background"]
        return {"color": list(bg.inputs[0].default_value),
                "strength": float(bg.inputs[1].default_value),
                "exposure": float(scene.view_settings.exposure)}
    except Exception:
        return {"color": None, "strength": None,
                "exposure": float(scene.view_settings.exposure)}


def convert_text_to_mesh():
    """名札などは曲線・文字なので、書き出せるようメッシュにする。"""
    bpy.ops.object.select_all(action="DESELECT")
    targets = [o for o in bpy.data.objects if o.type in {"FONT", "CURVE"}]
    for o in targets:
        o.select_set(True)
    if not targets:
        return 0
    bpy.context.view_layer.objects.active = targets[0]
    bpy.ops.object.convert(target="MESH")
    return len(targets)


def simplify_materials():
    """基本色だけ残す。何を落としたかを返す。"""
    lost = set()
    for mat in bpy.data.materials:
        if not mat.use_nodes or mat.node_tree is None:
            continue
        nt = mat.node_tree
        bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf is None:
            lost.add("Principled BSDF の無いマテリアル: " + mat.name)
            continue
        for socket_name, label in (("Base Color", "手続き的な基本色"),
                                   ("Roughness", "手続き的な粗さ"),
                                   ("Normal", "バンプ"),
                                   ("Metallic", "金属の設定")):
            sock = bsdf.inputs.get(socket_name)
            if sock is None:
                continue
            for link in list(sock.links):
                nt.links.remove(link)
                lost.add(label)
        bsdf.inputs["Metallic"].default_value = 0.0
        bsdf.inputs["Roughness"].default_value = 0.6
    return sorted(lost)


def select_for_export():
    bpy.ops.object.select_all(action="DESELECT")
    n = 0
    for obj in bpy.data.objects:
        if obj.type in {"MESH", "EMPTY"} and not obj.hide_render:
            obj.select_set(True)
            n += 1
    return n


def main():
    args = parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    scene = bpy.context.scene
    cam = scene.camera
    if cam is None:
        raise SystemExit("scene has no camera")

    n_text = convert_text_to_mesh()
    lost = simplify_materials()
    n_sel = select_for_export()

    dae = os.path.join(out, "scene.dae")
    obj = os.path.join(out, "scene.obj")
    glb = os.path.join(out, "scene.glb")
    bpy.ops.wm.collada_export(
        filepath=dae,
        selected=True,
        apply_modifiers=True,
        export_mesh_type_selection="render",
        export_global_up_selection="Z",
        export_global_forward_selection="-Y",
        apply_global_orientation=True,
        use_texture_copies=False,
        triangulate=True,
        use_object_instantiation=False,
    )
    # .dae は Gazebo/OGRE の読み込みが軸を変換してしまうため、.obj も出す（試行用）
    bpy.ops.wm.obj_export(
        filepath=obj,
        export_selected_objects=True,
        export_materials=True,
        export_uv=True,
        forward_axis="NEGATIVE_Z",
        up_axis="Z",
        apply_modifiers=True,
        export_triangulated_mesh=True,
    )
    bpy.ops.export_scene.gltf(
        filepath=glb,
        export_format="GLB",
        use_selection=True,
        export_yup=False,
        export_apply=True,
        export_cameras=False,
        export_lights=False,
        export_materials="EXPORT",
    )

    info = {
        "source_blend": bpy.data.filepath,
        "camera": camera_json(scene, cam),
        "lights": lights_json(),
        "world": world_json(scene),
        "objects_selected": n_sel,
        "text_converted": n_text,
        "lost": lost,
    }
    with open(os.path.join(out, "camera_lights.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, ensure_ascii=False, indent=1)
    print(f"[export] {out} objects={n_sel} text={n_text} lights={len(info['lights'])} lost={lost}")


if __name__ == "__main__":
    main()

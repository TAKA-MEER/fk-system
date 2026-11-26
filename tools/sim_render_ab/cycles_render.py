"""WP-SIM-11: Blender 4.2 の Cycles で同じシーンを描く（ホストの Blender で回す）。

使い方: blender -b -P tools/sim_render_ab/cycles_render.py -- --blend <scene.blend>
    --mats cam_mats.json --out <出力先> [--samples 96]
出力: p<姿勢:02d>.png（歪みなし・広め 1664x936）と times.json（各枚の秒）

描き方: 同じ .blend・同じ姿勢・同じ内部パラメータ（camera_K の fx・fy）。
光源は pyrender の既定に合わせる: 太陽光（Renderer の方向光と同じ向き）+ 環境光
（world の背景）+ scene.json の点光源（あれば）。サンプル数 96 は FK-E1 と同じ。
"""

import bpy
import json
import math
import os
import sys
import time

import mathutils


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--blend", required=True)
    p.add_argument("--mats", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--samples", type=int, default=96)
    return p.parse_args(argv)


def add_lights():
    # 太陽光: Renderer の方向光（斜め上）と同じ向き
    # （密封した部屋では天井に遮られるので、室内の明かりは下の点光源が持つ）
    bpy.ops.object.light_add(type="SUN", location=(0, 0, 10))
    sun = bpy.context.active_object
    sun.data.energy = 2.0
    sun.rotation_euler = (math.radians(67.5), math.radians(30.0), math.radians(90.0))
    # 環境光: Renderer の ambient 0.25 に対応する world 背景
    world = bpy.context.scene.world
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.20, 0.21, 0.225, 1.0)
    bg.inputs["Strength"].default_value = 1.0
    # 室内の点光源（Renderer の点光源の置き換え。pyrender の ambient は遮らないが、
    # Cycles の world は部屋に遮られるので、室内に置く。公開の部屋の記述に
    # lights が無いので、部屋の四隅に置く）
    for pos in [(2, 2, 3.8), (10, 2, 3.8), (2, 6, 3.8), (10, 6, 3.8)]:
        bpy.ops.object.light_add(type="POINT", location=pos)
        bpy.context.active_object.data.energy = 30.0
        bpy.context.active_object.data.shadow_soft_size = 0.5


def opencv_T_to_blender_camera(T_wc):
    """ワールド → OpenCV カメラから、Blender カメラの位置と回転を作る。

    OpenCV（X 右・Y 下・前が +Z）と Blender カメラ（X 右・Y 上・前が -Z）の
    軸の変換 diag(1, -1, -1) を噛ませる。
    """
    import mathutils
    M = mathutils.Matrix(T_wc)
    center = M.inverted().translation
    R_wc = M.to_3x3()
    S = mathutils.Matrix.Diagonal([1.0, -1.0, -1.0])
    R_w_blender = R_wc.transposed() @ S
    rot = R_w_blender.to_euler()
    cam = bpy.data.cameras.new("cam")
    cam_obj = bpy.data.objects.new("cam", cam)
    bpy.context.scene.collection.objects.link(cam_obj)
    cam_obj.location = center
    cam_obj.rotation_euler = rot
    return cam_obj, cam


def main():
    args = parse_args()
    bpy.ops.wm.open_mainfile(filepath=os.path.abspath(args.blend))
    with open(args.mats, encoding="utf-8") as f:
        data = json.load(f)
    poses, mats = data["poses"], data["mats"]
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.render.resolution_x = 1664
    scene.render.resolution_y = 936
    scene.render.image_settings.file_format = "PNG"
    scene.cycles.samples = args.samples
    scene.cycles.use_gpu = False
    scene.cycles.device = "CPU"
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "Standard"
    add_lights()
    os.makedirs(args.out, exist_ok=True)
    times = []
    for i, (pose, mat) in enumerate(zip(poses, mats)):
        cam_obj, cam = opencv_T_to_blender_camera(mat["T_wc"])
        fx = mat["K"][0][0]
        cam.lens = fx * 36.0 / 1664.0  # sensor_width 36 mm で fx を合わせる
        scene.camera = cam_obj
        t0 = time.perf_counter()
        scene.render.filepath = os.path.join(args.out, f"p{i:02d}.png")
        bpy.ops.render.render(write_still=True)
        times.append(time.perf_counter() - t0)
        bpy.data.objects.remove(cam_obj, do_unlink=True)
        bpy.data.cameras.remove(cam)
    with open(os.path.join(args.out, "times.json"), "w", encoding="utf-8") as f:
        json.dump({"render_s": times, "median_s": sorted(times)[len(times) // 2],
                   "samples": args.samples, "mean_s": sum(times) / len(times)}, f, indent=1)
    print(f"[cycles_render] {len(times)} frames mean={sum(times) / len(times):.2f}s")


if __name__ == "__main__":
    main()

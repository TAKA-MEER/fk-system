"""FK-E1: R-PY。glTF（.glb）を pyrender + OSMesa で CPU だけで描く。

GPU を使わない。同じカメラ位置・同じ画角・1280x720。照明は camera_lights.json から近いものを置く
（面積光源は点光源で近似し、全体を方向光源と ambient で底上げする）。

起動（GLB の読み込み・OSMesa の初期化）と 1 枚の描画時間を分けて測る。描画は暖機 1 回のあと
3 回の中央値を「1 枚の時間」とする（シェーダの初回コンパイルを除くため）。

使い方（コンテナの中。PYOPENGL_PLATFORM=osmesa）:
  python3 render_py.py --export-dir <dir> --out <dir>
"""

import argparse
import json
import os
import statistics
import time

os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import numpy as np
import pyrender
import trimesh
from PIL import Image


def build_scene(info, dir_intensity=1.0, ambient=0.30, point_intensity=8.0):
    glb = os.path.join(os.path.dirname(info["_json"]), "scene.glb")
    tm = trimesh.load(glb, force="scene")
    scene = pyrender.Scene.from_trimesh_scene(tm, bg_color=(0.8, 0.85, 0.9))
    # 金属は環境マップ無しだと真っ黒になるので、拡散寄りに描く
    for m in scene.meshes:
        for prim in m.primitives:
            mat = prim.material
            if isinstance(mat, pyrender.MetallicRoughnessMaterial):
                mat.metallicFactor = 0.0
                mat.roughnessFactor = 0.7

    # 全体を照らす方向光源（斜め上から）
    R = np.array([
        [0.866, -0.25, 0.433, 0.0],
        [0.0, 0.866, 0.5, 0.0],
        [-0.5, -0.433, 0.75, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=dir_intensity), pose=R)
    scene.ambient_light = np.array([ambient, ambient, ambient])

    lights = sorted(info.get("lights", []), key=lambda q: q.get("energy", 0.0), reverse=True)
    for i, q in enumerate([x for x in lights if x["type"] in {"POINT", "SPOT", "AREA"}][:4]):
        loc = np.array(q["location"], float)
        pose = np.eye(4)
        pose[:3, 3] = loc
        scene.add(pyrender.PointLight(color=np.array(q["color"], float), intensity=point_intensity), pose=pose)
    return scene


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--warmup", type=int, default=1)
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--dir-intensity", type=float, default=0.6)
    ap.add_argument("--ambient", type=float, default=0.25)
    ap.add_argument("--point-intensity", type=float, default=6.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    t0 = time.perf_counter()
    jpath = os.path.join(args.export_dir, "camera_lights.json")
    with open(jpath, encoding="utf-8") as f:
        info = json.load(f)
    info["_json"] = jpath
    scene = build_scene(info, args.dir_intensity, args.ambient, args.point_intensity)
    cam = info["camera"]
    W, H = cam["resolution"]
    fx, fy = cam["K"][0][0], cam["K"][1][1]
    cx, cy = cam["K"][0][2], cam["K"][1][2]
    camera = pyrender.IntrinsicsCamera(fx, fy, cx, cy, cam["near"], cam["far"])
    pose = np.array(cam["matrix_world"], float)
    scene.add(camera, pose=pose)
    renderer = pyrender.OffscreenRenderer(W, H)
    t_ready = time.perf_counter()

    for _ in range(args.warmup):
        renderer.render(scene)
    t_warm = time.perf_counter()

    times, color = [], None
    for _ in range(args.repeat):
        a = time.perf_counter()
        color, _depth = renderer.render(scene)
        times.append(time.perf_counter() - a)
    t_done = time.perf_counter()
    renderer.delete()

    Image.fromarray(color).save(os.path.join(args.out, "frame.png"))
    result = {
        "startup_s": t_ready - t0,
        "warmup_s": t_warm - t_ready,
        "render_s": statistics.median(times),
        "render_all_s": times,
        "total_s": t_done - t0,
        "resolution": [W, H],
    }
    with open(os.path.join(args.out, "times.json"), "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    print(f"[render_py] {args.out} startup={result['startup_s']:.2f}s "
          f"render={result['render_s']:.3f}s total={result['total_s']:.2f}s")


if __name__ == "__main__":
    main()

"""FK-E1: R-GZ。Gazebo Classic 11 のカメラセンサ（OGRE）で描く。

GPU をコンテナに渡さず、Xvfb + Mesa のソフトウェア OpenGL（llvmpipe）で動かす。
起動（最初の 1 枚が出るまで）と、そのあとの 1 枚あたりの間隔を分けて測る。

使い方（コンテナの中）:
  xvfb-run -a python3 render_gz.py --export-dir <dir> --out <dir> --frames 3
"""

import argparse
import json
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_sdf import build_world  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--update-rate", type=float, default=30.0)
    ap.add_argument("--ambient", type=float, default=0.12)
    ap.add_argument("--bg", type=float, default=0.45)
    ap.add_argument("--scene-roll", type=float, default=0.0)
    ap.add_argument("--mesh", default="scene.glb",
                    help="既定は scene.glb。trimesh でジオメトリ毎にワールド座標の .obj に変換する"
                         "（Blender の .dae は Gazebo の読み込みで軸変換され、Blender の .obj は"
                         "オブジェクト変換を焼き込まないため、変換が正しい .glb から作る）")
    ap.add_argument("--sun", type=float, default=0.18)
    ap.add_argument("--point-scale", type=float, default=0.22)
    ap.add_argument("--rpy-order", choices=["zyx", "xyz"], default="zyx")
    ap.add_argument("--max-parts", type=int, default=None, help="診断用: obj パーツ数の上限")
    args = ap.parse_args()

    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    save_dir = os.path.join(out, "gazebo_save")
    if os.path.isdir(save_dir):
        shutil.rmtree(save_dir)
    os.makedirs(save_dir)

    jpath = os.path.join(args.export_dir, "camera_lights.json")
    with open(jpath, encoding="utf-8") as f:
        info = json.load(f)
    mesh_path = os.path.join(args.export_dir, args.mesh)
    if args.mesh.endswith(".glb"):
        # .dae は Gazebo/OGRE の読み込みで軸が変換され、Blender の .obj はオブジェクトの
        # ワールド変換を焼き込まない。変換とマテリアルが正しい .glb から、ジオメトリ毎に
        # ワールド座標の .obj（＋個別の .mtl）を作り、SDF の複数 visual として置く。
        import trimesh
        tm = trimesh.load(mesh_path, force="scene")
        parts_dir = os.path.join(out, "obj_parts")
        os.makedirs(parts_dir, exist_ok=True)
        mesh_uri = []
        for i, g in enumerate(tm.dump()):
            if args.max_parts is not None and i >= args.max_parts:
                break
            opath = os.path.join(parts_dir, f"part_{i}.obj")
            g.export(opath, include_normals=True)
            mpath = os.path.join(parts_dir, "material.mtl")
            if os.path.isfile(mpath):
                shutil.move(mpath, os.path.join(parts_dir, f"part_{i}.mtl"))
                with open(opath, encoding="utf-8") as f:
                    s = f.read()
                with open(opath, "w", encoding="utf-8") as f:
                    f.write(s.replace("mtllib material.mtl", f"mtllib part_{i}.mtl"))
            mesh_uri.append("file://" + os.path.abspath(opath).replace("\\", "/"))
        print(f"[render_gz] glb -> {len(mesh_uri)} obj parts", flush=True)
    else:
        mesh_uri = "file://" + os.path.abspath(mesh_path).replace("\\", "/")
    world = build_world(info, mesh_uri, save_dir, args.update_rate, args.ambient, args.bg, args.scene_roll, args.sun, args.point_scale, args.rpy_order)
    world_path = os.path.join(out, "scene.world")
    with open(world_path, "w", encoding="utf-8") as f:
        f.write(world)

    env = dict(os.environ)
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GALLIUM_DRIVER"] = "llvmpipe"
    env.setdefault("DISPLAY", os.environ.get("DISPLAY", ""))
    cmd = ["gzserver", world_path, "--verbose"]
    log = open(os.path.join(out, "gzserver.log"), "w", encoding="utf-8")
    t0 = time.perf_counter()
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)

    seen = {}
    times = []
    try:
        while len(times) < args.frames:
            if proc.poll() is not None:
                break
            if time.perf_counter() - t0 > args.timeout:
                print("[render_gz] timeout", file=sys.stderr)
                break
            for name in sorted(os.listdir(save_dir)):
                if not name.lower().endswith((".jpg", ".jpeg", ".png")):
                    continue
                path = os.path.join(save_dir, name)
                if name in seen:
                    continue
                try:
                    size = os.path.getsize(path)
                except OSError:
                    continue
                if size <= 0:
                    continue
                seen[name] = size
                times.append(time.perf_counter() - t0)
            time.sleep(0.03)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        log.close()

    if not times:
        raise SystemExit("[render_gz] no frames were saved (see gzserver.log)")

    # 最後に保存された 1 枚を代表にする（書き込み途中でないものを選ぶ）
    files = sorted(os.listdir(save_dir))
    files = [f for f in files if f.lower().endswith((".jpg", ".jpeg", ".png"))]
    src = os.path.join(save_dir, files[-1])
    shutil.copyfile(src, os.path.join(out, "frame" + os.path.splitext(src)[1]))

    startup = times[0]
    diffs = [b - a for a, b in zip(times, times[1:])]
    result = {
        "startup_s": startup,
        "render_s": statistics.median(diffs) if diffs else None,
        "render_all_s": diffs,
        "frame_times_s": times,
        "n_frames": len(times),
        "saved_format": os.path.splitext(src)[1].lstrip("."),
    }
    with open(os.path.join(out, "times.json"), "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    print(f"[render_gz] {out} startup={startup:.2f}s "
          f"render={result['render_s']} frames={len(times)}")


if __name__ == "__main__":
    main()

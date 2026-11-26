"""WP-SIM-11: A の描き方ごとにシーン（scene.glb と .blend）を作る（ホストで回す）。

base は生成器（`fk_sim.scene build`）で作った `sim_out/scenes/public_room/`。
A の 3 通りは同じ scene.json を使って `build_scene.py` に金具の上書きを渡して作り直す。
B 用に base の .blend も保存する（`--blend`。既定の描き方は変えない）。

使い方:
  python tools/sim_render_ab/build_scenes.py [--blender <パス>]
前提: sim_out/scenes/public_room/（scene.json・scene.glb）が既にあること。
"""

import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import appearance  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(HERE))
SCENES = os.path.join(ROOT, "sim_out", "scenes")
BUILD_SCENE = os.path.join(ROOT, "fk_ws", "src", "fk_sim", "blender", "build_scene.py")
YAML = os.path.join(ROOT, "fk_ws", "src", "fk_sim", "scenes", "public_room.yaml")
BLENDER_DEFAULT = "C:/Program Files/Blender Foundation/Blender 4.2/blender.exe"


def run_blender(blender, out_dir, scene_overrides, blend=None, scene_json_dir=None):
    os.makedirs(out_dir, exist_ok=True)
    if scene_json_dir:
        shutil.copy(os.path.join(scene_json_dir, "scene.json"), out_dir)
    cmd = [blender, "-b", "-P", BUILD_SCENE, "--",
           "--scene", YAML, "--out", out_dir]
    if "lug_rgb" in scene_overrides:
        r, g, b = scene_overrides["lug_rgb"]
        cmd += ["--lug-rgb", f"{r},{g},{b}"]
    if "lug_rough" in scene_overrides:
        cmd += ["--lug-rough", str(scene_overrides["lug_rough"])]
    if "lug_metallic" in scene_overrides:
        cmd += ["--lug-metallic", str(scene_overrides["lug_metallic"])]
    if blend:
        cmd += ["--blend", blend]
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blender", default=os.environ.get("BLENDER", BLENDER_DEFAULT))
    args = ap.parse_args()
    base_dir = os.path.join(SCENES, "public_room")
    if not os.path.isfile(os.path.join(base_dir, "scene.glb")):
        raise SystemExit(f"base シーンが無い: {base_dir}（生成器で先に作る）")
    # B 用の .blend（base の描き方は既定のまま）
    blend = os.path.join(base_dir, "scene.blend")
    if not os.path.isfile(blend):
        run_blender(args.blender, base_dir, {}, blend=blend)
    for name in ("a_bright", "a_dark", "a_gloss"):
        out = os.path.join(SCENES, f"public_room_{name}")
        overrides = appearance.VARIANTS[name]["scene_overrides"]
        run_blender(args.blender, out, overrides, blend=os.path.join(out, "scene.blend"),
                    scene_json_dir=base_dir)
    print("done")


if __name__ == "__main__":
    main()

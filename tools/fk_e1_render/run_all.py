"""FK-E1 の手順を通しで回す（ホスト側）。

前提:
- Blender 4.2 が `C:/Program Files/Blender Foundation/Blender 4.2/blender.exe`
- lifting-lug-pose の .venv（ultralytics・torch・numpy・Pillow・scipy 入り）
- Docker Desktop（Linux コンテナ）

使い方（lifting-lug-pose の .venv の Python で）:
  python tools/fk_e1_render/run_all.py gen
  python tools/fk_e1_render/run_all.py export
  python tools/fk_e1_render/run_all.py docker
  python tools/fk_e1_render/run_all.py render py
  python tools/fk_e1_render/run_all.py render gz
  python tools/fk_e1_render/run_all.py variants
  python tools/fk_e1_render/run_all.py detect
"""

import argparse
import subprocess
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "sim_out" / "fk_e1"
TOOLS = ROOT / "tools" / "fk_e1_render"

BLENDER = Path(r"C:/Program Files/Blender Foundation/Blender 4.2/blender.exe")
GEN = Path(r"C:/Users/yisah/Documents/code/MIRS-local/lifting-lug-pose/src/blender/gen_scene.py")

# 単盤・連盤 × 電気室・工場。8〜10 シーン（各 2 視点）。
COMBOS = [
    ("single_er", "single", "electric_room", 3, 100),
    ("single_factory", "single", "factory", 2, 200),
    ("row_er", "row", "electric_room", 2, 300),
    ("row_factory", "row", "factory", 3, 400),
]


def run(cmd):
    cmd = [str(c) for c in cmd]
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def gen():
    for name, mode, env, num, seed in COMBOS:
        base = ["-b", "-P", GEN, "--", "--num", num, "--seed", seed,
                "--mode", mode, "--env", env, "--camera", "robot", "--views", 2,
                "--samples", 96]
        run([BLENDER, *base, "--out", OUT / "cycles_nopost" / name, "--save-blend", "--no-post"])
        run([BLENDER, *base, "--out", OUT / "cycles_post" / name])


def export():
    blends = sorted((OUT / "cycles_nopost").glob("*/*.blend"))
    if not blends:
        raise SystemExit("no .blend files; run `gen` first")
    for blend in blends:
        out = OUT / "export" / blend.parent.name / blend.stem
        run([BLENDER, "-b", blend, "-P", TOOLS / "export_scene.py", "--", "--out", out])


def docker_build():
    run(["docker", "build", "-t", "fk-e1-render", TOOLS])


def render(mode):
    if mode not in {"py", "gz"}:
        raise SystemExit("mode must be py or gz")
    vol = str(ROOT).replace("\\", "/") + ":/work"
    run(["docker", "run", "--rm", "-v", vol, "-w", "/work", "fk-e1-render",
         "bash", "tools/fk_e1_render/render_all.sh", mode])


def variants():
    for mode in ("py", "gz"):
        for src in sorted((OUT / f"render_{mode}").glob("*/*/frame.*")):
            combo = src.parent.parent.name
            stem = src.parent.name
            dst = OUT / "variants" / mode / combo / (stem + ".jpg")
            seed = zlib.crc32(f"{mode}/{combo}/{stem}".encode()) % (2 ** 31)
            run([sys.executable, TOOLS / "post_noise_jpeg.py", "--src", src, "--dst", dst,
                 "--seed", seed])


def detect():
    run([sys.executable, TOOLS / "detect_eval.py", "--root", OUT, "--out", OUT / "detect"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["gen", "export", "docker", "render", "variants", "detect"])
    ap.add_argument("mode", nargs="?", default=None)
    args = ap.parse_args()
    if args.step == "gen":
        gen()
    elif args.step == "export":
        export()
    elif args.step == "docker":
        docker_build()
    elif args.step == "render":
        render(args.mode)
    elif args.step == "variants":
        variants()
    elif args.step == "detect":
        detect()


if __name__ == "__main__":
    main()

"""WP-SIM-11: 描画の切り分け A・B を通しで回す（ホストで回す。Windows 版）。

手順:
  1. 姿勢の一覧（appearance.build_poses）を書く
  2. コンテナ（fk-sim）で基準・A の 3 通りの pyrender を描く
  3. ホストの Blender 4.2 で Cycles を描く（center と recorded の姿勢だけ）
  4. コンテナで Cycles の絵に Renderer と同じ歪ませ方・後処理を通す
  5. venv の Python で detect.py を回し、表を作る
  6. 代表の絵（基準で見逃した 1 枚と、同じ姿勢・種の A・B）を sim_out/sim_render_ab/ に集める

使い方（ホストの通常の Python で）:
  python tools/sim_render_ab/run_all.py [--skip-cycles]
検出は lifting-lug-pose の venv の Python で detect.py を直接回す（run_all は
最後にそのコマンドを出すだけで、実行はしない）。
"""

import argparse
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

import appearance  # noqa: E402

SS_REPO_DEFAULT = "C:/Users/yisah/Documents/code/MIRS-local/TM/ss-system"
VENV_PYTHON = ("C:/Users/yisah/Documents/code/MIRS-local/lifting-lug-pose/"
               ".venv/Scripts/python.exe")
OUT_ROOT = os.path.join(ROOT, "sim_out", "sim_render_ab")
FRAMES = os.path.join(OUT_ROOT, "frames")
SCENES = os.path.join(ROOT, "sim_out", "scenes")
BLENDER_DEFAULT = "C:/Program Files/Blender Foundation/Blender 4.2/blender.exe"


def docker_run(tools_script_args, extra_env=None):
    win = ROOT.replace("\\", "/")
    cmd = ["docker", "run", "--rm",
           "-v", f"{win}:/work",
           "-v", f"{SS_REPO_DEFAULT}:/ss-system:ro",
           "-e", "SS_REPO=/ss-system",
           "fk-sim"] + tools_script_args
    print("+", " ".join(cmd), flush=True)
    env = dict(os.environ)
    env["MSYS_NO_PATHCONV"] = "1"
    if extra_env:
        env.update(extra_env)
    subprocess.run(cmd, check=True, env=env)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-cycles", action="store_true")
    ap.add_argument("--blender", default=os.environ.get("BLENDER", BLENDER_DEFAULT))
    args = ap.parse_args()
    os.makedirs(FRAMES, exist_ok=True)

    poses = appearance.build_poses()
    poses_path = os.path.join(OUT_ROOT, "poses.json")
    with open(poses_path, "w", encoding="utf-8") as f:
        json.dump(poses, f, indent=1)
    seeds_csv = ",".join(str(s) for s in appearance.SEEDS)
    print(f"[poses] {len(poses)} 姿勢 x {len(appearance.SEEDS)} 種")

    scenes = {
        "base": ("public_room", {}),
        "a_bright": ("public_room_a_bright", {}),
        "a_dark": ("public_room_a_dark", {}),
        "a_gloss": ("public_room_a_gloss",
                    {"metallic": 0.0, "roughness": 0.15}),
    }
    for variant, (scene_name, rover) in scenes.items():
        out = os.path.join(FRAMES, variant)
        rel_out = os.path.relpath(out, ROOT).replace("\\", "/")
        rel_scene = os.path.relpath(os.path.join(SCENES, scene_name), ROOT).replace("\\", "/")
        rel_poses = os.path.relpath(poses_path, ROOT).replace("\\", "/")
        cmd = ["python3", "/work/tools/sim_render_ab/render_frames.py",
               "--scene", f"/work/{rel_scene}", "--poses", f"/work/{rel_poses}",
               "--out", f"/work/{rel_out}", "--seeds", seeds_csv]
        for k, v in rover.items():
            cmd += [f"--{k}", str(v)]
        docker_run(cmd)

    if not args.skip_cycles:
        # Cycles: center と recorded の姿勢（6 姿勢）だけ
        cycle_poses = [p for p in poses if p["kind"] in ("center", "recorded")]
        cycle_poses_path = os.path.join(OUT_ROOT, "poses_cycles.json")
        with open(cycle_poses_path, "w", encoding="utf-8") as f:
            json.dump(cycle_poses, f, indent=1)
        mats_path = os.path.join(OUT_ROOT, "cam_mats.json")
        # カメラ行列は fk_sim.render.core で作る（ホストの Python 312 で。numpy あり）
        host_python = sys.executable
        subprocess.run([host_python, os.path.join(HERE, "make_cam_mats.py"),
                        "--poses", cycle_poses_path, "--out", mats_path],
                       check=True)
        wide = os.path.join(OUT_ROOT, "cycles_wide")
        subprocess.run([args.blender, "-b", "-P", os.path.join(HERE, "cycles_render.py"),
                        "--", "--blend", os.path.join(SCENES, "public_room", "scene.blend"),
                        "--mats", mats_path, "--out", wide, "--samples", "96"],
                       check=True)
        import shutil as _sh
        # cycles の times.json をフレームの側へ
        times = json.load(open(os.path.join(wide, "times.json"), encoding="utf-8"))
        out = os.path.join(FRAMES, "b_cycles")
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "times.json"), "w", encoding="utf-8") as f:
            json.dump(times, f, indent=1)
        # 後処理（歪ませ方・ノイズ・JPEG）は Renderer と同じ関数をコンテナで通す。
        # cycles_render は p<姿勢>.png の名前なので、姿勢番号は元の poses での
        # center/recorded の位置に合わせる（detect は番号で照合するため置き直す）
        rel_wide = os.path.relpath(wide, ROOT).replace("\\", "/")
        rel_out = os.path.relpath(out, ROOT).replace("\\", "/")
        docker_run(["python3", "/work/tools/sim_render_ab/post_cycles.py",
                    "--wide", f"/work/{rel_wide}", "--out", f"/work/{rel_out}",
                    "--seeds", seeds_csv])
        # p 番号の置き直し（make_cam_mats の順 → poses での位置）
        orig_idx = [poses.index(p) for p in cycle_poses]
        for j, oi in enumerate(orig_idx):
            for seed in appearance.SEEDS:
                src = os.path.join(out, f"p{j:02d}_s{seed:02d}.jpg")
                dst = os.path.join(out, f"p{oi:02d}_s{seed:02d}.jpg")
                if os.path.isfile(src):
                    _sh.move(src, dst)
        # ファイルが無い姿勢（検出側は無ければ飛ばす）の扱いは detect.py のとおり

    print("[next] venv で検出する:")
    print(f'  {VENV_PYTHON} tools/sim_render_ab/detect.py --frames-root '
          f'{FRAMES.replace(chr(92), "/")} --scene {SCENES.replace(chr(92), "/")}/public_room')


if __name__ == "__main__":
    main()

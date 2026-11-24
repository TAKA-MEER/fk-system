"""シーン生成器（記述からワールド・正解・.glb を作る）。ROS 2 も Blender も import しない。

使い方: python -m fk_sim.scene build <記述.yaml> [--out <出力先>]
既定の出力先は <リポジトリ直下>/sim_out/scenes/<name>/。
環境変数 BLENDER があれば Blender を呼んで scene.glb も作り、Blender の
外接箱と scene.json が合うかまで確かめる。BLENDER が無いときは scene.glb を
作らなかったと明示して終える（黙って成功にはしない）。
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
import time

from . import answers, guard, model, start_pose, world

WORLD_NAME = "world.world"
ANSWER_NAME = "scene.json"
GLB_NAME = "scene.glb"
BLENDER_OBJECTS_NAME = "blender_objects.json"


def default_out_dir(scene, start=None):
    top = guard.repo_toplevel(start) or os.path.realpath(start or os.getcwd())
    return os.path.join(top, "sim_out", "scenes", scene["name"])


def find_build_scene_script():
    """blender/build_scene.py を探す（開発レイアウトとインストール先の両方）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "..", "..", "blender", "build_scene.py"),   # src/fk_sim/blender/
        os.path.join(here, "..", "blender", "build_scene.py"),          # 同じ深さの保険
        os.path.join(os.path.dirname(here), "blender", "build_scene.py"),
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(here))),
                     "share", "fk_sim", "blender", "build_scene.py"),    # colcon install 先
    ]
    for c in candidates:
        c = os.path.realpath(c)
        if os.path.isfile(c):
            return c
    return None


def run_blender(blender_exe, script, scene_path, out_dir):
    """Blender バッチで scene.glb を作る。外接箱と scene.json が食い違えば落とす。"""
    cmd = [blender_exe, "-b", "-P", script, "--",
           "--scene", os.path.abspath(scene_path), "--out", os.path.abspath(out_dir)]
    t0 = time.monotonic()
    proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    elapsed = time.monotonic() - t0
    print(proc.stdout[-3000:] if proc.stdout else "")
    print(proc.stderr[-1000:] if proc.stderr else "", file=sys.stderr)
    if proc.returncode != 0:
        raise RuntimeError(f"Blender が失敗しました（終了コード {proc.returncode}）")
    glb = os.path.join(out_dir, GLB_NAME)
    if not os.path.isfile(glb):
        raise RuntimeError(f"Blender が {GLB_NAME} を作りませんでした")
    objects_path = os.path.join(out_dir, BLENDER_OBJECTS_NAME)
    with open(objects_path, encoding="utf-8") as f:
        blender_objects = json.load(f)
    with open(os.path.join(out_dir, ANSWER_NAME), encoding="utf-8") as f:
        answer = json.load(f)
    ok, problems = answers.compare_blender_objects(answer, blender_objects)
    if not ok:
        raise RuntimeError("Blender の外接箱と scene.json が合いません:\n- " + "\n- ".join(problems))
    return elapsed


def _default_ss_repo(start=None):
    """ss のリポジトリ。$SS_REPO があればそれ、無ければ作業ツリーの隣（sim_up.sh の既定）。"""
    repo = os.environ.get("SS_REPO")
    if repo:
        return repo
    top = guard.repo_toplevel(os.path.dirname(start)) if start else guard.repo_toplevel()
    top = top or os.path.realpath(os.path.dirname(start) if start else os.getcwd())
    return os.path.join(os.path.dirname(top), "ss-system")


def _default_nav2_params(start=None):
    """th の nav2_params_sim.yaml の取り出し物（import_th_sim.sh の出力）。"""
    override = os.environ.get("FK_SIM_NAV2_PARAMS")
    if override:
        return override
    top = guard.repo_toplevel(os.path.dirname(start)) if start else guard.repo_toplevel()
    top = top or os.path.realpath(os.path.dirname(start) if start else os.getcwd())
    return os.path.join(top, "fk_ws", "sim", "th_src", "src", "th_bringup",
                        "config", "nav2_params_sim.yaml")


def build_scene(scene_path, out_dir=None, blender_exe=None, ss_repo=None, nav2_params=None):
    """記述から生成する。戻り値: (出力先, blender の経過秒 or None)。"""
    t_start = time.monotonic()
    scene = model.load_scene(scene_path)
    out = out_dir or default_out_dir(scene)
    out = guard.check_output(scene, out)  # 何も書く前に止める
    # 機体の置き場（DetailedDesign-sim.md §3.1）。設定は必ず読む（読めなければ止める）。
    scene["robot"] = start_pose.resolve_robot(
        scene,
        ss_repo or _default_ss_repo(scene_path),
        nav2_params or _default_nav2_params(scene_path))
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, WORLD_NAME), "w", encoding="utf-8", newline="\n") as f:
        f.write(world.build_world(scene))
    answer = answers.build_answer(scene)
    with open(os.path.join(out, ANSWER_NAME), "w", encoding="utf-8", newline="\n") as f:
        json.dump(answer, f, ensure_ascii=False, indent=1)
        f.write("\n")
    robot = scene["robot"]
    print(f"[scene] 名前={scene['name']} visibility={scene['visibility']} → {out}")
    print(f"[scene] robot pose ({robot['pose_source']}): {robot['pose']}")
    print(f"[scene] visible_panels: {robot['visible_panels']}")
    print(f"[scene] hidden_panels: {robot['hidden_panels']}")
    print(f"[scene] {WORLD_NAME}・{ANSWER_NAME} を書きました")
    exe = blender_exe if blender_exe is not None else os.environ.get("BLENDER")
    if not exe:
        print("[scene] 環境変数 BLENDER が無いので scene.glb は作りませんでした")
        return out, None
    script = find_build_scene_script()
    if script is None:
        raise RuntimeError("blender/build_scene.py が見つかりません")
    elapsed = run_blender(exe, script, scene_path, out)
    print(f"[scene] {GLB_NAME} を作りました（Blender {elapsed:.1f} 秒）")
    print(f"[scene] 合計 {time.monotonic() - t_start:.1f} 秒")
    return out, elapsed


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m fk_sim.scene")
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build", help="記述から生成する")
    build.add_argument("scene", help="シーンの記述（YAML）のパス")
    build.add_argument("--out", default=None, help="出力先（既定は sim_out/scenes/<name>/）")
    args = parser.parse_args(argv)
    try:
        build_scene(args.scene, out_dir=args.out)
    except (model.SceneError, guard.GuardError, RuntimeError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        print("  使う方: " +
              shlex.quote("python -m fk_sim.scene build <記述.yaml> [--out <出力先>]"),
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

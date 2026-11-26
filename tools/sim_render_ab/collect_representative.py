"""WP-SIM-11: 代表の絵を集める（ホスト側。lifting-lug-pose の venv）。

基準で見逃した 1 枚（中央／記録の姿勢のうち、画像に入った近い金具が見つからないもの）と、
同じ姿勢・同じ種の A・B の絵を `sim_out/sim_render_ab/representative/` へコピーする。

使い方（venv の Python で）:
  python tools/sim_render_ab/collect_representative.py --frames-root ... --scene ...
"""

import argparse
import json
import os
import shutil
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "fk_ws", "src", "fk_sim"))
sys.path.insert(0, HERE)

import appearance  # noqa: E402
from fk_sim.render.core import _load_ss_geometry, load_ss_camera, world_to_camera_T  # noqa: E402

SS_REPO_DEFAULT = "C:/Users/yisah/Documents/code/MIRS-local/TM/ss-system"
MODEL = os.path.join(SS_REPO_DEFAULT, "model", "lug_v3_s.pt")
SHA256 = "5a4689a9d49d549f61dddb1f2fb375583e2a98ca0333791776e6b140f4e83227"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames-root", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ss-repo", default=SS_REPO_DEFAULT)
    args = ap.parse_args()
    os.environ.setdefault("SS_REPO", args.ss_repo)
    os.makedirs(args.out, exist_ok=True)

    sys.path.insert(0, os.path.join(args.ss_repo, "ss_ws", "src", "ss_locator"))
    from ss_locator.detector import Detector  # noqa: E402
    detector = Detector(MODEL, SHA256, imgsz=1280, conf=0.25)
    geometry = _load_ss_geometry()
    cam = load_ss_camera()
    with open(os.path.join(args.scene, "scene.json"), encoding="utf-8") as f:
        answer = json.load(f)
    p004 = next(p for p in answer["panels"] if p["id"] == "P004")
    lugs = sorted(p004["lugs"].items())
    panel_xy = (appearance.PANEL["x"], appearance.PANEL["y"])
    poses = appearance.build_poses(args.ss_repo)

    found = None
    for i, p in enumerate(poses):
        if p["kind"] not in ("center", "recorded"):
            continue
        T_wc = world_to_camera_T(p["x"], p["y"], p["yaw"], p["height_m"],
                                 p["cam_yaw_deg"], p["cam_pitch_deg"], camera=cam)
        gt = []
        for corner, lug in lugs:
            uv, ok = geometry.project(np.array(lug["ring_center"]).reshape(1, 3),
                                      cam["K"], cam["dist"], T_wc, w=1280, h=720)
            if bool(ok[0]):
                gt.append((uv[0], appearance.classify_lug(
                    lug["ring_center"], (p["x"], p["y"]), panel_xy)))
        if not any(side == "near" for _, side in gt):
            continue
        for seed in appearance.SEEDS:
            path = os.path.join(args.frames_root, "base", f"p{i:02d}_s{seed:02d}.jpg")
            img = cv2.imread(path)
            if img is None:
                continue
            keypoints, _ = detector.detect(img)
            centers = keypoints.mean(axis=1) if len(keypoints) else np.zeros((0, 2))
            results, _false = appearance.match_detections([g[0] for g in gt], centers)
            near_missed = [g for g, r in zip(gt, results)
                           if g[1] == "near" and not r["found"]]
            near_seen = [g for g, r in zip(gt, results)
                         if g[1] == "near" and r["found"]]
            if near_missed:
                found = (i, seed, near_missed)
                break
        if found:
            break
    if not found:
        print("[representative] 見つからない枚が無かった")
        return
    i, seed, near_missed = found
    print(f"[representative] pose p{i:02d} seed s{seed:02d} 見逃した近い金具 {len(near_missed)}")
    for variant in ("base", "a_bright", "a_dark", "a_gloss", "b_cycles"):
        src = os.path.join(args.frames_root, variant, f"p{i:02d}_s{seed:02d}.jpg")
        if os.path.isfile(src):
            dst = os.path.join(args.out, f"{variant}_p{i:02d}_s{seed:02d}.jpg")
            shutil.copy(src, dst)
            print(f"  {dst}")
    with open(os.path.join(args.out, "missed.json"), "w", encoding="utf-8") as f:
        json.dump({"pose": poses[i], "pose_index": i, "seed": seed,
                   "near_missed": [[float(u[0]), float(u[1])] for u, _ in near_missed]},
                  f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()

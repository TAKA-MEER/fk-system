"""WP-SIM-11: 描いた絵に ss の検出器をかけて表にする（ホスト側。lifting-lug-pose の venv）。

描画はコンテナ／Blender 側で行い、絵は <frames-root>/<描き方>/p<姿勢:02d>_s<種:02d>.jpg
の形で揃える。P004 の 4 金具だけを `geometry.project` の正解と突き合わせる
（`tools/sim_render_check/check.py` と同じ 15 px）。

使い方（venv の Python で）:
  python tools/sim_render_ab/detect.py --frames-root sim_out/sim_render_ab/frames
      --scene sim_out/scenes/public_room
"""

import argparse
import json
import os
import sys
from collections import defaultdict

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
    ap.add_argument("--out", default=None)
    ap.add_argument("--ss-repo", default=SS_REPO_DEFAULT)
    ap.add_argument("--only-kinds", default=None,
                    help="カンマ区切りでこの kind の姿勢だけ（既定は全部）")
    args = ap.parse_args()
    os.environ.setdefault("SS_REPO", args.ss_repo)

    poses = [(i, p) for i, p in enumerate(appearance.build_poses(args.ss_repo))]
    if args.only_kinds:
        kinds = set(args.only_kinds.split(","))
        poses = [(i, p) for i, p in poses if p["kind"] in kinds]
    variants = sorted(d for d in os.listdir(args.frames_root)
                      if os.path.isdir(os.path.join(args.frames_root, d)))

    sys.path.insert(0, os.path.join(args.ss_repo, "ss_ws", "src", "ss_locator"))
    from ss_locator.detector import Detector  # noqa: E402
    detector = Detector(MODEL, SHA256, imgsz=1280, conf=0.25)
    geometry = _load_ss_geometry()
    cam = load_ss_camera()
    with open(os.path.join(args.scene, "scene.json"), encoding="utf-8") as f:
        answer = json.load(f)
    p004 = next(p for p in answer["panels"] if p["id"] == "P004")
    lugs = sorted(p004["lugs"].items())  # [(corner, {ring_center, ...})]
    panel_xy = (appearance.PANEL["x"], appearance.PANEL["y"])

    report = {"variants": {}, "poses": len(poses), "seeds": len(appearance.SEEDS)}
    for variant in variants:
        vdir = os.path.join(args.frames_root, variant)
        agg = defaultdict(lambda: {"found": 0, "total": 0})
        frames = 0
        frames_all = 0
        false_list = []
        errors = []
        for i, p in poses:
            T_wc = world_to_camera_T(p["x"], p["y"], p["yaw"], p["height_m"],
                                     p["cam_yaw_deg"], p["cam_pitch_deg"], camera=cam)
            gt = []
            for corner, lug in lugs:
                uv, ok = geometry.project(
                    np.array(lug["ring_center"]).reshape(1, 3),
                    cam["K"], cam["dist"], T_wc, w=1280, h=720)
                if bool(ok[0]):
                    gt.append((uv[0], appearance.classify_lug(
                        lug["ring_center"], (p["x"], p["y"]), panel_xy)))
            for seed in appearance.SEEDS:
                path = os.path.join(vdir, f"p{i:02d}_s{seed:02d}.jpg")
                if not os.path.isfile(path):
                    continue
                img = cv2.imread(path)
                if img is None or img.shape[:2] != (720, 1280):
                    continue
                keypoints, _confs = detector.detect(img)
                centers = keypoints.mean(axis=1) if len(keypoints) else np.zeros((0, 2))
                results, false = appearance.match_detections([g[0] for g in gt], centers)
                frames += 1
                false_list.append(false)
                all_found = True
                for (uv, side), res in zip(gt, results):
                    agg[side]["total"] += 1
                    if res["found"]:
                        agg[side]["found"] += 1
                        errors.append(res["err_px"])
                    else:
                        all_found = False
                if all_found and gt:
                    frames_all += 1
        times_path = os.path.join(vdir, "times.json")
        render_median = None
        if os.path.isfile(times_path):
            with open(times_path, encoding="utf-8") as f:
                render_median = json.load(f).get("median_s")
        report["variants"][variant] = {
            "frames": frames,
            "near": agg["near"], "far": agg["far"],
            "rate_near": agg["near"]["found"] / agg["near"]["total"] if agg["near"]["total"] else None,
            "rate_far": agg["far"]["found"] / agg["far"]["total"] if agg["far"]["total"] else None,
            "all_found_rate": frames_all / frames if frames else None,
            "false_per_frame": float(np.mean(false_list)) if false_list else None,
            "error_px_median": float(np.median(errors)) if errors else None,
            "render_median_s": render_median,
        }
    out_path = args.out or os.path.join(args.frames_root, "report.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    print(json.dumps(report, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()

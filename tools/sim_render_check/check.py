"""描いた絵に ss の検出器をかけて確かめる道具（ホスト側。lifting-lug-pose の venv で回す）。

描画は fk-sim コンテナの中で行い（poses.json が無い場合は下の既定を使う）、
正解は `scene.json` の吊り金具の `ring_center` を ss の `geometry.project` で
投影したもの（画像の中・カメラの前のものだけ）。15 px 以内を「見つけた」とする。

使い方:
  <venvpython> tools/sim_render_check/check.py --scene sim_out/scenes/public_room \
      --out .briefs/tmp/check_frames [--image fk-sim]
"""

import argparse
import json
import math
import os
import subprocess
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "fk_ws", "src", "fk_sim"))

from fk_sim.render.core import _load_ss_geometry, world_to_camera_T  # noqa: E402

SS_REPO_DEFAULT = "C:/Users/yisah/Documents/code/MIRS-local/TM/ss-system"
MODEL = os.path.join(SS_REPO_DEFAULT, "model", "lug_v3_s.pt")
SHA256 = "5a4689a9d49d549f61dddb1f2fb375583e2a98ca0333791776e6b140f4e83227"

# ss が実際に撮る条件に近い姿勢（scan_camera_height_m 3.10 m → 昇降機 1.28 m、
# scan_pitch_down_deg 20°、盤が 1.5〜5 m に入る位置）
LIFT_HEIGHT = 3.10 - 1.82
POSES = []
for (x, y, yaw) in [(5.9, 5.5, math.pi / 2), (4.5, 5.0, 1.34), (6.5, 4.8, 1.46),
                    (1.8, 3.0, 0.0), (1.6, 1.7, 0.75), (7.2, 5.2, 1.65)]:
    for (cam_yaw, cam_pitch) in [(0.0, -20.0), (12.0, -20.0), (-12.0, -16.0), (8.0, -24.0)]:
        POSES.append({"x": x, "y": y, "yaw": yaw, "height_m": LIFT_HEIGHT,
                      "cam_yaw_deg": cam_yaw, "cam_pitch_deg": cam_pitch})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=os.path.join(ROOT, "sim_out", "scenes", "public_room"))
    ap.add_argument("--out", default=os.path.join(ROOT, ".briefs", "tmp", "check_frames"))
    ap.add_argument("--image", default="fk-sim")
    ap.add_argument("--worktree", default=ROOT)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    poses_path = os.path.join(args.out, "poses.json")
    with open(poses_path, "w", encoding="utf-8") as f:
        json.dump(POSES, f, indent=1)

    # 1. コンテナの中で 24 枚描く（GPU は渡さない）
    win = args.worktree.replace("\\", "/")
    rel_out = os.path.relpath(args.out, args.worktree).replace("\\", "/")
    rel_scene = os.path.relpath(args.scene, args.worktree).replace("\\", "/")
    subprocess.run([
        "docker", "run", "--rm",
        "-v", f"{win}:/work",
        "-v", f"{SS_REPO_DEFAULT}:/ss-system:ro",
        "-e", "SS_REPO=/ss-system",
        args.image,
        "python3", "/work/tools/sim_render_check/render_frames.py",
        "--scene", f"/work/{rel_scene}",
        "--poses", f"/work/{rel_out}/poses.json",
        "--out", f"/work/{rel_out}",
        "--seed", str(args.seed),
    ], check=True, env={**os.environ, "MSYS_NO_PATHCONV": "1"})

    # 2. ホストで検出器（ss の Detector。imgsz=1280・conf=0.25・CPU）
    sys.path.insert(0, os.path.join(SS_REPO_DEFAULT, "ss_ws", "src", "ss_locator"))
    from ss_locator.detector import Detector  # noqa: E402
    detector = Detector(MODEL, SHA256, imgsz=1280, conf=0.25)

    geometry = _load_ss_geometry()
    with open(os.path.join(args.scene, "scene.json"), encoding="utf-8") as f:
        answer = json.load(f)
    with open(os.path.join(args.out, "times.json"), encoding="utf-8") as f:
        times = json.load(f)

    total_gt, matched_gt, errors, false_per_frame = 0, 0, [], []
    per_frame = []
    for i, p in enumerate(POSES):
        path = os.path.join(args.out, f"frame{i:03d}.jpg")
        img = cv2.imread(path)
        assert img is not None and img.shape[:2] == (720, 1280), path
        keypoints, confs = detector.detect(img)
        centers = keypoints.mean(axis=1) if len(keypoints) else np.zeros((0, 2))

        T_wc = world_to_camera_T(p["x"], p["y"], p["yaw"], p["height_m"],
                                 p["cam_yaw_deg"], p["cam_pitch_deg"])
        from fk_sim.render.core import load_ss_camera
        if "SS_REPO" not in os.environ:
            os.environ["SS_REPO"] = SS_REPO_DEFAULT
        cam = load_ss_camera()
        gt = []
        for panel in answer["panels"]:
            for lug in panel["lugs"].values():
                uv, ok = geometry.project(np.array(lug["ring_center"]).reshape(1, 3),
                                          cam["K"], cam["dist"], T_wc, w=1280, h=720)
                if bool(ok[0]):
                    gt.append(uv[0])
        used = set()
        n_found = 0
        for g in gt:
            best, best_d = None, 1e9
            for j, c in enumerate(centers):
                if j in used:
                    continue
                d = float(np.linalg.norm(c - g))
                if d < best_d:
                    best, best_d = j, d
            if best is not None and best_d <= 15.0:
                used.add(best)
                matched_gt += 1
                n_found += 1
                errors.append(best_d)
        total_gt += len(gt)
        false = len(centers) - len(used)
        false_per_frame.append(false)
        per_frame.append({"frame": i, "gt": len(gt), "found": n_found, "detections": len(centers)})

    report = {
        "frames": len(POSES),
        "detection_rate": matched_gt / total_gt if total_gt else 0.0,
        "matched": matched_gt,
        "gt": total_gt,
        "error_px_median": float(np.median(errors)) if errors else None,
        "error_px_p95": float(np.percentile(errors, 95)) if errors else None,
        "false_per_frame": float(np.mean(false_per_frame)),
        "render_median_s": times["median_s"],
        "render_max_s": max(times["render_s"]),
        "per_frame": per_frame,
    }
    with open(os.path.join(args.out, "report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1)
    print(json.dumps({k: report[k] for k in
                      ("frames", "detection_rate", "matched", "gt",
                       "error_px_median", "error_px_p95", "false_per_frame",
                       "render_median_s", "render_max_s")}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()

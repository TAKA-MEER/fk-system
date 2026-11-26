"""WP-SIM-11: Cycles で描くためのカメラ行列を作る（ホストで回す）。

`fk_sim.render.core.world_to_camera_T` で姿勢ごとのワールド → OpenCV カメラの
(4x4) を計算し、cycles_render.py（Blender）が読める形で書く。

使い方:
  python tools/sim_render_ab/make_cam_mats.py --poses poses.json --out cam_mats.json
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "fk_ws", "src", "fk_sim"))
sys.path.insert(0, HERE)

import appearance  # noqa: E402
from fk_sim.render.core import load_ss_camera, world_to_camera_T  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ss-repo", default=appearance.SS_REPO_DEFAULT)
    args = ap.parse_args()
    os.environ.setdefault("SS_REPO", args.ss_repo)
    with open(args.poses, encoding="utf-8") as f:
        poses = json.load(f)
    cam = load_ss_camera()
    mats = []
    for p in poses:
        T = world_to_camera_T(p["x"], p["y"], p["yaw"], p["height_m"],
                              p["cam_yaw_deg"], p["cam_pitch_deg"], camera=cam)
        mats.append({"T_wc": T.tolist(), "K": cam["K"].tolist()})
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"poses": poses, "mats": mats}, f, indent=1)
    print(f"[make_cam_mats] {len(mats)} poses, fx={cam['K'][0,0]:.1f}")


if __name__ == "__main__":
    main()

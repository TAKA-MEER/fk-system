"""検出の確かめ用の描画: 決まった姿勢の一覧（JSON）通り fk-sim コンテナの中で描く。

使い方（コンテナの中）:
  python3 tools/sim_render_check/render_frames.py --scene <シーンの出力>
      --poses poses.json --out <出力先> [--seed N]
出力: frame###.jpg と times.json（各姿勢の描画時間）
"""

import argparse
import json
import os
import statistics
import sys
import time

sys.path.insert(0, "/work/fk_ws/src/fk_sim")

from fk_sim.render.core import Renderer  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--poses", required=True, help="[{x,y,yaw,height_m,cam_yaw_deg,cam_pitch_deg}, ...]")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    with open(args.poses, encoding="utf-8") as f:
        poses = json.load(f)
    os.makedirs(args.out, exist_ok=True)
    r = Renderer(args.scene)
    times = []
    try:
        for i, p in enumerate(poses):
            t0 = time.perf_counter()
            data = r.render(p["x"], p["y"], p["yaw"], p["height_m"],
                            p["cam_yaw_deg"], p["cam_pitch_deg"], seed=args.seed)
            times.append(time.perf_counter() - t0)
            with open(os.path.join(args.out, f"frame{i:03d}.jpg"), "wb") as f:
                f.write(data)
    finally:
        r.close()
    with open(os.path.join(args.out, "times.json"), "w", encoding="utf-8") as f:
        json.dump({"render_s": times, "median_s": statistics.median(times)}, f, indent=1)
    print(f"[render_frames] {len(poses)} frames median={statistics.median(times):.3f}s")


if __name__ == "__main__":
    main()

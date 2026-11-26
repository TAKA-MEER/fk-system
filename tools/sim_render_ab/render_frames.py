"""WP-SIM-11: pyrender の描き方（基準・A）をコンテナの中で描く。

使い方（コンテナの中）:
  python3 tools/sim_render_ab/render_frames.py --scene <シーンの出力>
      --poses poses.json --out <出力先> --seeds 0,1,...
      [--metallic 値] [--roughness 値]
出力: p<姿勢:02d>_s<種:02d>.jpg と times.json（各枚の秒・中央値）
"""

import argparse
import json
import os
import statistics
import sys
import time

sys.path.insert(0, "/work/fk_ws/src/fk_sim")
sys.path.insert(0, "/work/tools/sim_render_ab")

from fk_sim.render.core import Renderer  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--poses", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seeds", required=True, help="カンマ区切りの種の一覧")
    ap.add_argument("--metallic", type=float, default=None)
    ap.add_argument("--roughness", type=float, default=None)
    args = ap.parse_args()

    with open(args.poses, encoding="utf-8") as f:
        poses = json.load(f)
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    os.makedirs(args.out, exist_ok=True)
    kw = {}
    if args.metallic is not None:
        kw["metallic_factor"] = args.metallic
    if args.roughness is not None:
        kw["roughness_factor"] = args.roughness
    r = Renderer(args.scene, **kw)
    times = []
    try:
        for i, p in enumerate(poses):
            for seed in seeds:
                t0 = time.perf_counter()
                data = r.render(p["x"], p["y"], p["yaw"], p["height_m"],
                                p["cam_yaw_deg"], p["cam_pitch_deg"], seed=seed)
                dt = time.perf_counter() - t0
                times.append(dt)
                with open(os.path.join(args.out, f"p{i:02d}_s{seed:02d}.jpg"), "wb") as f:
                    f.write(data)
    finally:
        r.close()
    with open(os.path.join(args.out, "times.json"), "w", encoding="utf-8") as f:
        json.dump({"render_s": times, "median_s": statistics.median(times)}, f, indent=1)
    print(f"[render_frames] {len(times)} frames median={statistics.median(times):.3f}s")


if __name__ == "__main__":
    main()

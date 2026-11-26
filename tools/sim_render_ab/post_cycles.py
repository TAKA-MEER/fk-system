"""WP-SIM-11: Cycles の広めの絵に Renderer と同じ歪ませ方と後処理を通す（コンテナ）。

Cycles は歪みなしを少し広めに描くので、歪ませ方（`_undistort_map`・`_bilinear`）と
後処理（`_post_noise_jpeg`）は pyrender の描画と同じ関数を通す。

使い方（コンテナの中）:
  python3 tools/sim_render_ab/post_cycles.py --wide <cycles の PNG の置き場>
      --out <出力先> --seeds 0,1,...
出力: p<姿勢:02d>_s<種:02d>.jpg（最終形）
"""

import argparse
import json
import os
import sys

sys.path.insert(0, "/work/fk_ws/src/fk_sim")

from fk_sim.render.core import _bilinear, _post_noise_jpeg, _undistort_map  # noqa: E402
from fk_sim.render.core import WIDTH, HEIGHT, _RENDER_SCALE  # noqa: E402
from fk_sim.render.core import _load_ss_geometry, load_ss_camera  # noqa: E402

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wide", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seeds", required=True)
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    os.makedirs(args.out, exist_ok=True)
    cam = load_ss_camera()
    geometry = _load_ss_geometry()
    K, dist = cam["K"], cam["dist"]
    w2 = int(round(WIDTH * _RENDER_SCALE))
    h2 = int(round(HEIGHT * _RENDER_SCALE))
    ox, oy = (w2 - WIDTH) / 2.0, (h2 - HEIGHT) / 2.0
    fold_r = geometry.fold_radius(dist)
    u0, v0, ok = _undistort_map(K, dist, WIDTH, HEIGHT, fold_r)
    map_u, map_v = u0 + ox, v0 + oy
    map_ok = ok & (map_u >= 0) & (map_u < w2 - 1) & (map_v >= 0) & (map_v < h2 - 1)
    from PIL import Image
    count = 0
    for name in sorted(os.listdir(args.wide)):
        if not name.endswith(".png"):
            continue
        stem = name[:-4]
        img = np.asarray(Image.open(os.path.join(args.wide, name)).convert("RGB"))
        assert img.shape[:2] == (h2, w2), (name, img.shape)
        warped = _bilinear(img, map_u, map_v)
        warped = np.where(map_ok[..., None], warped, 0.0)
        warped_u8 = np.clip(warped, 0.0, 255.0).astype(np.uint8)
        for seed in seeds:
            out = _post_noise_jpeg(warped_u8, seed)
            with open(os.path.join(args.out, f"{stem}_s{seed:02d}.jpg"), "wb") as f:
                f.write(out)
            count += 1
    print(f"[post_cycles] {count} frames")


if __name__ == "__main__":
    main()

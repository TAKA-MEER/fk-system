"""描画の芯の CLI: `python -m fk_sim.render <シーンの出力> --x … --y … --yaw …
--height … --cam-yaw-deg … --cam-pitch-deg … --out frame.jpg [--seed N]`。

位置・向きはワールド座標（m・rad。`--yaw` は機体の向き）、高さは昇降機の高さ（m）、
カメラの向きは hve の `yaw_deg`・`pitch_deg`（`--cam-yaw-deg`・`--cam-pitch-deg`）。
"""

import argparse
import os

from .core import Renderer


def main():
    ap = argparse.ArgumentParser(description="fk_sim.render: 機体の位置・向きから 1 枚描く")
    ap.add_argument("scene_dir")
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--height", type=float, required=True, help="昇降機の高さ (m)")
    ap.add_argument("--cam-yaw-deg", type=float, default=0.0)
    ap.add_argument("--cam-pitch-deg", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    r = Renderer(args.scene_dir)
    try:
        data = r.render(args.x, args.y, args.yaw, args.height,
                        args.cam_yaw_deg, args.cam_pitch_deg, seed=args.seed)
    finally:
        r.close()
    d = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(d, exist_ok=True)
    with open(args.out, "wb") as f:
        f.write(data)
    print(f"[render] {args.out} ({len(data)} bytes)")


if __name__ == "__main__":
    main()

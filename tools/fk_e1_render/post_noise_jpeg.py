"""FK-E1: 軽い描画の絵に、ノイズと JPEG 圧縮だけを足す（歪みは足さない）。

値は `gen_scene.py` の後処理（sample_post）に合わせる:
  noise_sigma: 0.002〜0.012、輝度ノイズはその 0.7 倍、JPEG 品質 82〜92。
使い方:
  python post_noise_jpeg.py --src a.png --dst b.jpg --seed 1
"""

import argparse
import json
import os

import numpy as np
from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    img = np.asarray(Image.open(args.src).convert("RGB"), dtype=np.float32) / 255.0
    h, w = img.shape[:2]
    rng = np.random.default_rng(args.seed)
    sigma = float(rng.uniform(0.002, 0.012))
    quality = int(rng.integers(82, 93))
    out = img + rng.normal(0.0, sigma, img.shape).astype(np.float32)
    out += rng.normal(0.0, sigma * 0.7, (h, w, 1)).astype(np.float32)
    out = np.clip(out, 0.0, 1.0)
    os.makedirs(os.path.dirname(os.path.abspath(args.dst)), exist_ok=True)
    Image.fromarray(np.round(out * 255.0).astype(np.uint8)).save(args.dst, quality=quality)
    with open(os.path.splitext(args.dst)[0] + ".json", "w", encoding="utf-8") as f:
        json.dump({"noise_sigma": sigma, "jpeg_quality": quality, "src": args.src}, f,
                  ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()

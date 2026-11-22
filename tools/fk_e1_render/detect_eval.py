"""FK-E1: 各組の絵を ss-system の Detector にかけ、Cycles の基準と比べる。

正解は注釈の金具のリング中心（`ring_center_px`。歪み無しの絵は歪み無しの投影、後処理ありの絵は
歪み込みの投影がそのまま入っている）。見えている金具（`visible`）だけを数える。

指標（組ごと）:
- 検出率: 見えている金具のうち、予測のキーポイント 4 点の平均が 15 px 以内にあるもの
- 誤差: 対応した予測の 4 点の平均と正解のリング中心の距離 [px]（中央値と 95%）
- 誤検出: 見えている金具に対応しなかった予測の数（1 枚あたり）
- 確信度: 予測の確信度の平均

使い方:
  python detect_eval.py --root sim_out/fk_e1 --out sim_out/fk_e1/detect
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.optimize import linear_sum_assignment

DEFAULT_DETECTOR = r"C:/Users/yisah/Documents/code/MIRS-local/TM/ss-system/ss_ws/src/ss_locator/ss_locator/detector.py"
DEFAULT_MODEL = r"C:/Users/yisah/Documents/code/MIRS-local/TM/ss-system/model/lug_v3_s.pt"
DEFAULT_SHA = "5a4689a9d49d549f61dddb1f2fb375583e2a98ca0333791776e6b140f4e83227"
THRESH_PX = 15.0


def load_detector(detector_py, model, sha):
    spec = importlib.util.spec_from_file_location("fk_e1_ss_detector", detector_py)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Detector(model, sha, 1280, 0.25)


def gt_lugs(ann):
    out = []
    for p in ann["panels"]:
        for lug in p["lugs"]:
            if lug.get("visible") and lug.get("in_frame"):
                out.append({"panel": p["id"], "corner": lug["corner"],
                            "center": list(lug["ring_center_px"])})
    return out


def discover(root):
    root = Path(root)
    groups = {}

    def add(name, img, ann):
        if img.is_file() and ann.is_file():
            groups.setdefault(name, []).append((img, ann))

    for combo in sorted(p for p in (root / "cycles_nopost").glob("*") if p.is_dir()):
        for ann in sorted((combo / "annotations").glob("*.json")):
            stem = ann.stem
            add("cycles_nopost", combo / "images" / (stem + ".png"), ann)
            add("cycles_post", root / "cycles_post" / combo.name / "images" / (stem + ".jpg"),
                root / "cycles_post" / combo.name / "annotations" / (stem + ".json"))
            add("py", root / "render_py" / combo.name / stem / "frame.png", ann)
            for ext in (".jpg", ".png"):
                img = root / "render_gz" / combo.name / stem / ("frame" + ext)
                if img.is_file():
                    add("gz", img, ann)
                    break
            add("py_noise", root / "variants" / "py" / combo.name / (stem + ".jpg"), ann)
            add("gz_noise", root / "variants" / "gz" / combo.name / (stem + ".jpg"), ann)
    return groups


def evaluate(group, items, detector, root):
    rows = []
    all_err, all_conf, all_conf_matched = [], [], []
    n_gt = n_pred = n_matched = 0
    for img_path, ann_path in items:
        with open(ann_path, encoding="utf-8") as f:
            ann = json.load(f)
        gts = gt_lugs(ann)
        img = np.asarray(Image.open(img_path).convert("RGB"))
        bgr = img[:, :, ::-1].copy()
        kps, confs = detector.detect(bgr)
        kps = np.asarray(kps, dtype=np.float64).reshape(-1, 4, 2)
        confs = np.asarray(confs, dtype=np.float64).reshape(-1)
        n_pred_i = len(kps)
        n_gt_i = len(gts)
        matched = []
        errs = []
        if n_pred_i and n_gt_i:
            mean = kps.mean(axis=1)
            centers = np.array([g["center"] for g in gts], float)
            d = np.linalg.norm(mean[:, None, :] - centers[None, :, :], axis=2)
            cost = np.where(d <= THRESH_PX, d, 1e6)
            ri, ci = linear_sum_assignment(cost)
            matched = [(int(i), int(j)) for i, j in zip(ri, ci) if cost[i, j] < 1e6]
            errs = [float(d[i, j]) for i, j in matched]
        n_m = len(matched)
        n_gt += n_gt_i
        n_pred += n_pred_i
        n_matched += n_m
        all_err.extend(errs)
        if n_pred_i:
            all_conf.extend(confs.tolist())
        if n_m:
            idx = [i for i, _ in matched]
            all_conf_matched.extend(confs[idx].tolist())
        rows.append({
            "image": str(Path(img_path).relative_to(root)).replace("\\", "/"),
            "n_gt": n_gt_i, "n_pred": n_pred_i, "n_matched": n_m,
            "recall": (n_m / n_gt_i) if n_gt_i else None,
            "fp": n_pred_i - n_m,
            "err_median": float(np.median(errs)) if errs else None,
            "conf_mean": float(np.mean(confs)) if n_pred_i else None,
        })
    agg = {
        "group": group,
        "n_images": len(items),
        "n_gt": n_gt,
        "n_pred": n_pred,
        "n_matched": n_matched,
        "recall": (n_matched / n_gt) if n_gt else None,
        "err_median": float(np.median(all_err)) if all_err else None,
        "err_p95": float(np.percentile(all_err, 95)) if all_err else None,
        "fp_per_image": (n_pred - n_matched) / len(items) if items else None,
        "conf_mean": float(np.mean(all_conf)) if all_conf else None,
        "conf_mean_matched": float(np.mean(all_conf_matched)) if all_conf_matched else None,
        "thresh_px": THRESH_PX,
    }
    return agg, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="sim_out/fk_e1")
    ap.add_argument("--out", default="sim_out/fk_e1/detect")
    ap.add_argument("--detector-py", default=DEFAULT_DETECTOR)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--sha", default=DEFAULT_SHA)
    args = ap.parse_args()

    root = Path(args.root)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    groups = discover(root)
    if not groups:
        raise SystemExit(f"no images under {root}")
    detector = load_detector(args.detector_py, args.model, args.sha)

    summary = {}
    for name in ["cycles_nopost", "cycles_post", "gz", "gz_noise", "py", "py_noise"]:
        if name not in groups:
            continue
        agg, rows = evaluate(name, groups[name], detector, root)
        summary[name] = agg
        with open(out / f"{name}.json", "w", encoding="utf-8") as f:
            json.dump({"summary": agg, "images": rows}, f, ensure_ascii=False, indent=1)

    with open(out / "summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)

    def f(v, n=2):
        return "-" if v is None else f"{v:.{n}f}"

    print(f"{'group':14s} {'img':>4s} {'gt':>4s} {'hit':>4s} {'recall':>7s} "
          f"{'err_med':>8s} {'err_p95':>8s} {'fp/img':>7s} {'conf':>6s}")
    for name, s in summary.items():
        print(f"{name:14s} {s['n_images']:4d} {s['n_gt']:4d} {s['n_matched']:4d} "
              f"{f(s['recall'],3):>7s} {f(s['err_median'],1):>8s} {f(s['err_p95'],1):>8s} "
              f"{f(s['fp_per_image'],2):>7s} {f(s['conf_mean'],3):>6s}")


if __name__ == "__main__":
    main()
